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
#include <nav_msgs/GetMap.h>
#include <list>

#include <tf/transform_broadcaster.h>

#include "src/define.h"
#include "src/util.h"

#include "src/SlamOmatic.h"

tf::TransformBroadcaster *br;
ros::Publisher pubPose, pubEstimatePose, pubMap, pubPath;
nav_msgs::Path   path;


SlamOmatic*     slamOmatic;
void UpdateSLAM(const sensor_msgs::LaserScan::Ptr& msg)
{
  static std::string frame_map;
  static std::string frame_base;
  static ros::Rate loop_rate_map(1), loop_rate_path(1);

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

    double freq;
    if(!n.getParam("topic_map_hz",freq))
      freq = 1.0f;
    loop_rate_map = ros::Rate(freq);
    loop_rate_map.reset();

    if(!n.getParam("topic_path_hz",freq))
      freq = 1.0f;
    loop_rate_path = ros::Rate(freq);
    loop_rate_path.reset();
  }
  //  _________________________
  //  ::: Update SLAM State :::
  Pose2D pose = slamOmatic->update(msg);

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

  // Publish path
  static ros::Time t_path = ros::Time::now();
  //ROS_ERROR("dt ros : %f >? %f",(ros::Time::now() - t ).toSec(),loop_rate.expectedCycleTime().toSec());
  if((ros::Time::now() - t_path).toSec() > loop_rate_path.expectedCycleTime().toSec()){
    t_path = ros::Time::now();
    pubPath.publish(path);
  }

  // Publish map
  static ros::Time t_map = ros::Time::now();
  //ROS_ERROR("dt ros : %f >? %f",(ros::Time::now() - t ).toSec(),loop_rate.expectedCycleTime().toSec());
  if((ros::Time::now() - t_map).toSec() > loop_rate_map.expectedCycleTime().toSec()){
    t_map = ros::Time::now();
    pubMap.publish(slamOmatic->map()->getProb2OccupencyGrid());
  }
}

bool  service_prob_map(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response)
{
  response.map = slamOmatic->map()->getProb2OccupencyGrid();
  return true;
}

bool  service_cost_map(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response)
{
  response.map = slamOmatic->map()->getCost2OccupencyGrid();
  return true;
}

//      ____________
//      ::: MAIN :::
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ibotslam_node");
  ros::NodeHandle n;
  br = new tf::TransformBroadcaster();

  //  Publisher :
  std::string topic_name;

  if(!n.getParam("topic_pose",topic_name))
    topic_name = TOPIC_POSE;
  pubPose = n.advertise<geometry_msgs::Pose2D>(topic_name, 1);

  if(!n.getParam("topic_map",topic_name))
    topic_name = TOPIC_MAP;
  pubMap = n.advertise<nav_msgs::OccupancyGrid>(topic_name, 1);

  if(!n.getParam("topic_path",topic_name))
    topic_name = TOPIC_PATH;
  pubPath = n.advertise<nav_msgs::Path>(topic_name,1);

  //  Suscriber :
  if(!n.getParam("topic_lidar",topic_name))
    topic_name = TOPIC_LiDAR;
  ros::Subscriber subScan1 = n.subscribe(topic_name, 1, UpdateSLAM);

  //  Service :
  if(!n.getParam("service_get_probmap",topic_name))
    topic_name = SERVICE_PROBMAP;
  ros::ServiceServer service_server_probmap = n.advertiseService(topic_name, service_prob_map);

  if(!n.getParam("service_get_costmap",topic_name))
    topic_name = SERVICE_COSTMAP;
  ros::ServiceServer service_server_costmap = n.advertiseService(topic_name, service_cost_map);

  //  Build SLAM object :
  slamOmatic = new SlamOmatic();

  ros::spin();

  delete slamOmatic;
  delete br;
  ROS_INFO("Node is stopped");
}















