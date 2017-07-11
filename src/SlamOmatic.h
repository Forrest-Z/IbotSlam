#ifndef SLAMOMATIC_H
#define SLAMOMATIC_H

#include <ros/ros.h>
#include <ros/param.h>
#include <sensor_msgs/LaserScan.h>

#include "define.h"
#include "lib_generic_slam/Tools.h"

#include "lib_generic_slam/GenericSlam.h"
#include "SlamOmaticLocalizer.h"
#include "SlamOmaticMap.h"

#include "lib_generic_slam/GenericFilter.h"
#include "NewsFilter.h"
#include "DensityFilter.h"

class SlamOmatic
{
public:
  SlamOmatic();

  /**
   * @brief update function to iterate the SLAM state using a new LiDAR data message :
   * @param msg the LiDAR data from ROS
   * @return the new pose estimated
   */
  Pose2D    update(const sensor_msgs::LaserScan::Ptr& msg);

  /**
   * @brief map getter on the global map (contains prob and cost map)
   * @return the global map
   */
  SlamOmaticMap *map() const;

  /**
   * @brief pose getter on the pose
   * @return the last estimate pose in meters
   */
  Pose2D pose() const;

protected:
  //    __________________
  //    ::: PARAMETERS :::
  double                  m_map_resolution;
  unsigned int            m_map_size;
  double                  m_map_start_x, m_map_start_y;
  uint8_t                 m_map_threshold;
  uint8_t                 m_cost_width;
  double                  m_interval_search;
  uint8_t                 m_proba_increment;
  double                  m_news_filter;

  //    ____________
  //    ::: SLAM :::
  // > Pointer to store the global map :
  SlamOmaticMap*          m_map;
  // > Pointer to store the global localizer
  SlamOmaticLocalizer*    m_localizer;
  // > Pointer to store the global SLAM algorithm
  GenericSlam<uint8_t>*   m_slam;
  // > The global pose in meters :
  Pose2D                  m_pose;
};

#endif // SLAMOMATIC_H
