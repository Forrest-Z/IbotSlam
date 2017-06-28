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

  Pose2D    update(const sensor_msgs::LaserScan::Ptr& msg);


//  Getter :
  SlamOmaticMap *map() const;
  Pose2D pose() const;

protected:
  double                  m_last_t_in_sec;

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
  SlamOmaticMap*          m_map;
  SlamOmaticLocalizer*    m_localizer;
  GenericSlam<uint8_t>*   m_slam;
  Pose2D                  m_pose;

  //    ______________
  //    ::: KALMAN :::
  Pose2D                  m_estimate_pose;




};

#endif // SLAMOMATIC_H
