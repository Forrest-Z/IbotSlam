#ifndef UTIL_H
#define UTIL_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

#include "lib_generic_slam/Pose2D.h"

namespace utils {

// To send transformation between map and base_link :
void      sendTransformPose2d(tf::TransformBroadcaster& broadcaster, const Pose2D  &pose);

}

#endif // UTIL_H
