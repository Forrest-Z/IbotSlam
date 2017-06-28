#ifndef TOOLS_H
#define TOOLS_H

#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include "generic_lidar_scan.h"

namespace Tools {

LiDAR_Scan_2D convertRosScan2GenericLiDARScan(const sensor_msgs::LaserScan::Ptr& scan);

LiDAR_Scan_2D filterByDistance(LiDAR_Scan_2D &scan, double distM);

LiDAR_Scan_2D approximateScan(LiDAR_Scan_2D& scan);

LiDAR_Scan_2D filterByQuaternion(LiDAR_Scan_2D& scan, const sensor_msgs::Imu& imu, double z_min, double z_max);

geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw);

void toEulerianAngle(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw);

}

#endif // TOOLS_H
