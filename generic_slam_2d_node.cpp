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

//  To send transform matrix between frame id :
tf::TransformBroadcaster *br;
//  Publisher to send simple message data to ROS (like pose, map or path, ...)
ros::Publisher pubPose, pubEstimatePose, pubMap, pubPath;
//  Global path from starting pose, updated locally after SLAM and sent at X-Hz
nav_msgs::Path   path;

//  Global var of the SLAM, used in Main & Callback
SlamOmatic*     slamOmatic;

/**
 * @brief UpdateSLAM : Callback to use data from LiDAR Topic.
 * @param msg of LiDAR data like sensor_msgs::LaserScan
 */
void UpdateSLAM(const sensor_msgs::LaserScan::Ptr& msg)
{
  // Statical values :
  //  > Frame Map Id (~ "map")
  static std::string frame_map;
  //  > Frame Robot Base Id (~ "base_link")
  static std::string frame_base;
  //  > Framerate in Hz to send map and path
  static ros::Rate loop_rate_map(1), loop_rate_path(1);
  //  > PoseStamped initialized
  static geometry_msgs::PoseStamped pose_stamped;

  // At the first iteration, initialisation must be made :
  static bool first = true;
  if(first)
  {
    first = false;
    // Create a local private NodeHandle to get parameters from roscore server :
    ros::NodeHandle nh_private("~");

    //  > Get parameter named "frame_map" :
    nh_private.param<std::string>("frame_map", frame_map, FRAME_MAP);
    path.header.frame_id = frame_map;

    //  > Get parameter named "frame_base_link" :
    nh_private.param<std::string>("frame_base_link", frame_base, FRAME_BASE);
    pose_stamped.header.frame_id = frame_base;

    double freq;

    //  > Get parameter named "topic_map_hz" :
    nh_private.param<double>("topic_map_hz", freq, 1.0f);
    loop_rate_map = ros::Rate(freq);
    loop_rate_map.reset();

    //  > Get parameter named "topic_path_hz" :
    nh_private.param<double>("topic_path_hz", freq, 1.0f);
    loop_rate_path = ros::Rate(freq);
    loop_rate_path.reset();
  }

  //  ::: Update SLAM State :::
  Pose2D pose = slamOmatic->update(msg);

  //  ::: Send Pose transform into frame Map and frame Base Link  :::
  utils::sendTransformPose2d(*br, pose, msg->header.stamp, frame_map, frame_base);

  // Create a data object message to send pose using Topic (for debug, other nodes could be use transform broadcaster)
  geometry_msgs::Pose2D ros_pose;
  ros_pose.x = pose.x;
  ros_pose.y = pose.y;
  ros_pose.theta = pose.theta;
  pubPose.publish(ros_pose);

  //  Create a data object to store XYT data into the path.
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.pose.position.x = pose.x;
  pose_stamped.pose.position.y = -pose.y;
  //  > In ROS, Euler angles are stored like Quaternion, we use toQuaternion to convert Euler angles to Quaternion ROS object
  pose_stamped.pose.orientation = Tools::toQuaternion(0.0f,0.0f,pose.theta);

  // Put PoseStamped object into the global path object :
  path.poses.push_back(pose_stamped);

  // Part to send at specified framerate path object data : (using loop_rate_path object initialized from parameter server)
  static ros::Time t_path = ros::Time::now();
  if((ros::Time::now() - t_path).toSec() > loop_rate_path.expectedCycleTime().toSec()){
    t_path = ros::Time::now();
    pubPath.publish(path);
  }

  // Part to send at specified framerate map object data : (using loop_rate_path object initialized from parameter server)
  static ros::Time t_map = ros::Time::now();
  if((ros::Time::now() - t_map).toSec() > loop_rate_map.expectedCycleTime().toSec()){
    t_map = ros::Time::now();
    pubMap.publish(slamOmatic->map()->getProb2OccupencyGrid());
  }
}

/**
 * @brief service_prob_map : ROS::Service to get the probabilistic map
 * @param request     -> No data in request
 * @param response    -> The map like an ros::OccupencyGrid
 * @return            -> Always true to send the response
 */
bool  service_prob_map(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response)
{
  response.map = slamOmatic->map()->getProb2OccupencyGrid();
  return true;
}

/**
 * @brief service_cost_map : ROS::Service to get the cost map
 * @param request     -> No data in request
 * @param response    -> The map like an ros::OccupencyGrid
 * @return            -> Always true to send the response
 */
bool  service_cost_map(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response)
{
  response.map = slamOmatic->map()->getCost2OccupencyGrid();
  return true;
}

//      ____________
//      ::: MAIN :::
int main(int argc, char **argv)
{
  // ROS Node initialisation using node name "ibotslam_node"
  ros::init(argc, argv, "ibotslam_node");

  // Create two nodehandle (one public for communication and one private for the parameters reading)
  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");

  // Create the global Transform Broadcaster :
  br = new tf::TransformBroadcaster();

  //  Initialize the topic and service names :
  std::string name;

  // Get parameter name topic_pose from parameter server and create this topic :
  nh_private.param<std::string>("topic_pose", name, TOPIC_POSE);
  pubPose = n.advertise<geometry_msgs::Pose2D>(name, 1);

  // Get parameter name topic_map from parameter server and create this topic :
  nh_private.param<std::string>("topic_map", name, TOPIC_MAP);
  pubMap = n.advertise<nav_msgs::OccupancyGrid>(name, 1);

  // Get parameter name topic_path from parameter server and create this topic :
  nh_private.param<std::string>("topic_path", name, TOPIC_PATH);
  pubPath = n.advertise<nav_msgs::Path>(name,1);

  // Get parameter name topic_lidar from parameter server and create this topic :
  nh_private.param<std::string>("topic_lidar", name, TOPIC_SCAN);
  ros::Subscriber subScan1 = n.subscribe(name, 1, UpdateSLAM);

  // Get parameter name service_get_probmap from parameter server and create this service :
  nh_private.param<std::string>("service_get_probmap", name, SERVICE_PROBMAP);
  ros::ServiceServer service_server_probmap = n.advertiseService(name, service_prob_map);

  // Get parameter name service_get_costmap from parameter server and create this service :
  nh_private.param<std::string>("service_get_costmap", name, SERVICE_COSTMAP);
  ros::ServiceServer service_server_costmap = n.advertiseService(name, service_cost_map);

  //  Build SLAM object :
  slamOmatic = new SlamOmatic();

  // Main loop managed by ROS (listen topic, service, ros:ok(), ...)
  // Function stopped if CTRL+C detected or if roscore stopped
  ros::spin();

  delete slamOmatic;
  delete br;
  ROS_INFO("Node is stopped");
}















