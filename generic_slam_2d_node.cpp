#include <ros/ros.h>
#include <ros/param.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <list>

#include <tf/transform_broadcaster.h>

#include "src/define.h"
#include "src/util.h"

#include "src/SlamOmatic.h"

tf::TransformBroadcaster *br;
ros::Publisher pubPose, pubEstimatePose, pubMap, pubPath;

SlamOmatic*     slamOmatic;
void UpdateSLAM(const sensor_msgs::LaserScan::Ptr& msg)
{
  static std::string frame_map;
  static std::string frame_base;
  static nav_msgs::Path   path;

  bool first = true;
  if(first)
  {
    first = false;
    ros::NodeHandle n;
    if(!n.getParam("frame_map",frame_map))
      frame_map = FRAME_MAP;
    path.header.frame_id = frame_map;

    if(!n.getParam("frame_base_link",frame_base))
      frame_base = FRAME_BASE;
  }
  //  _________________________
  //  ::: Update SLAM State :::
  Pose2D pose = slamOmatic->update(msg);

  //  _____________________________
  //  ::: Pour envoyer la carte :::

  //  static uint8_t stSender = 0;
  //  if(stSender == 10)
  //  {
  //    stSender = 0;
  nav_msgs::OccupancyGrid tmp_occupgrid = slamOmatic->map()->toRosOccupancyGrid();
  tmp_occupgrid.header.stamp = ros::Time::now();
  tmp_occupgrid.header.frame_id = frame_map;
  pubMap.publish(tmp_occupgrid);
  //} else stSender++;

  //  ___________________________
  //  ::: Send Pose transform :::
  utils::sendTransformPose2d(*br,pose);

  geometry_msgs::Pose2D ros_pose;
  ros_pose.x = pose.x;
  ros_pose.y = pose.y;
  ros_pose.theta = pose.theta;
  pubPose.publish(ros_pose);

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = frame_base;
  pose_stamped.pose.position.x = pose.x;
  pose_stamped.pose.position.y = -pose.y;
  pose_stamped.pose.orientation = Tools::toQuaternion(0.0f,0.0f,pose.theta);

  path.poses.push_back(pose_stamped);
  pubPath.publish(path);
}

//      ____________
//      ::: MAIN :::
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ibotslam_node",1);
  ros::NodeHandle n;
  br = new tf::TransformBroadcaster();

  //  Publisher :
  std::string topic_name;

  if(!n.getParam("topic_pose",topic_name))
    topic_name = TOPIC_POSE;
  pubPose = n.advertise<geometry_msgs::Pose2D>(topic_name, 1000);

  if(!n.getParam("topic_map",topic_name))
    topic_name = TOPIC_MAP;
  pubMap = n.advertise<nav_msgs::OccupancyGrid>(topic_name, 50);

  if(!n.getParam("topic_path",topic_name))
    topic_name = TOPIC_PATH;
  pubPath = n.advertise<nav_msgs::Path>(topic_name,100);

  //  Suscriber :
  if(!n.getParam("topic_lidar",topic_name))
    topic_name = TOPIC_LiDAR;
  ros::Subscriber subScan1 = n.subscribe(topic_name, 1, UpdateSLAM);

  //  Build SLAM object :
  slamOmatic = new SlamOmatic();

  ROS_INFO("Node is launched");
  ros::spin();

  delete slamOmatic;
  delete br;
  ROS_INFO("Node is stopped");
}















