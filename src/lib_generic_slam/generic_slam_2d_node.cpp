#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <list>

#include "src/lib_generic_slam/GenericSlam.h"
#include "src/lib_generic_slam/IGeneric2dMap.h"
#include "src/lib_generic_slam/IGenericLocalizer.h"
#include "src/lib_generic_slam/generic_lidar_scan.h"
#include "src/lib_generic_slam/Tools.h"

#include "src/SlamOmaticMap.h"
#include "src/SlamOmaticLocalizer.h"
#include "src/AlignFilter.h"
#include "src/NewsFilter.h"


GenericSlam<uint8_t>    *global_slam;

//  ######################################
#define   USE_IMU

#ifdef USE_IMU
#define   IMU_TOPIC_NAME    "/asctec_proc/imu"

double roll=0, pitch=0, yaw=3.14015;

void UpdateIMU(const sensor_msgs::Imu::Ptr& msg)
{ // tf::Quaternion q= msg-> pose.pose.orientation();
  //  static unsigned int i = 0;
  //  if(i > 15)
  Tools::toEulerianAngle(msg->orientation,roll,pitch,yaw);
  //  else
  //    i++;
}

#endif
//  ######################################

#define   MAP_SIZE          3501
#define   MAP_COSTMAX       15
#define   MAP_SEUIL         35
#define   MAP_RESOLUTION    0.05

#define   LOC_SEARCH_X_PX     0.1    // 7.5pix
#define   LOC_SEARCH_Y_PX     0.1    // 7.5pix

#ifdef  USE_IMU
#define   LOC_SEARCH_T_DEG    3
#else
#define   LOC_SEARCH_T_DEG    8
#endif



ros::Publisher pubPose, pubMap;


void UpdateSLAM(const sensor_msgs::LaserScan::Ptr& msg)
{
  //cout << "yaw : " << yaw << endl;
  //if(yaw == 0.0 && roll == 0.0 && pitch == 0.0) return; // roll,pitch,yaw
  ROS_INFO("New data with %d points",(int)msg->ranges.size());
  LiDAR_Scan_2D scan = Tools::convertRosScan2GenericLiDARScan(msg); // MeterScan

#ifdef USE_IMU
  global_slam->update(Pose2D(),yaw,scan);
#else
  static double yaw = 0.0f;
  yaw = global_slam->update(Pose2D(),yaw,scan).theta;
#endif

  nav_msgs::OccupancyGrid occ = global_slam->map()->toRosOccupancyGrid();
  pubMap.publish(occ);
}


//      ____________
//      ::: MAIN :::
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "generic_slam_2d_node");
  ros::NodeHandle n;

  pubPose = n.advertise<geometry_msgs::Pose2D>("pose2d", 1000);
  pubMap = n.advertise<nav_msgs::OccupancyGrid>("map", 50);

#ifdef USE_IMU
  ros::Subscriber suscImu = n.subscribe(IMU_TOPIC_NAME,50,UpdateIMU);
//  for(uint8_t i = 0; i < 10; i++)
//    ros::spinOnce();
#endif


  ros::Subscriber subScan = n.subscribe("scan", 3, UpdateSLAM);

  SlamOmaticMap* map = new SlamOmaticMap(MAP_SIZE,
                                         MAP_COSTMAX,
                                         MAP_SEUIL,
                                         MAP_RESOLUTION);

  SlamOmaticLocalizer* localizer = new SlamOmaticLocalizer(LOC_SEARCH_X_PX,
                                                           LOC_SEARCH_Y_PX,
                                                           LOC_SEARCH_T_DEG*M_PI/180.0f,
                                                           Parameters::myDefault(),
                                                           EndConditions(1.0f,50));

  global_slam = new GenericSlam<uint8_t>(map,localizer);

  NewsFilter<uint8_t> news_filter(map,1.0/MAP_RESOLUTION);
  AlignFilter align_filter;

  global_slam->addFilter(&news_filter);
  //global_slam->addFilter(&align_filter);

  ROS_INFO("Node is launched");

  ros::spin();

  delete global_slam;
  delete localizer;
  delete map;

  ROS_INFO("Node is stopped");
}
