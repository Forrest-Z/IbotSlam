#include "SlamOmatic.h"

SlamOmatic::SlamOmatic()
{
  // Private nodehandle to get parameter from store :
  ros::NodeHandle n;

  // Get parameter map_resolution :
  if(!n.getParam("map_resolution",(double&)m_map_resolution))
    m_map_resolution = MAP_RESOLUTION;

  // Get parameter map_size :
  if(!n.getParam("map_size",(int&)m_map_size))
    m_map_size = MAP_SIZE;

  /// Unused ...
  //  if(!ros::param::get("map_start_x",&m_map_start_x))
  //    m_map_start_x = 0.5f;
  //  if(!ros::param::get("map_start_y",&m_map_start_y))
  //    m_map_start_y = 0.5f;

  // Get parameter map_threshold :
  if(!n.getParam("map_threshold",(int&)m_map_threshold))
    m_map_threshold = MAP_SEUIL;

  // Get parameter map_cost_width :
  if(!n.getParam("map_cost_width",(int&)m_cost_width))
    m_cost_width = MAP_COSTMAX;

  // Get parameter map_interval_search :
  if(!n.getParam("map_interval_search",m_interval_search))
    m_interval_search = INTER_SEARCH;

  // Get parameter map_news_filter :
  if(!n.getParam("map_news_filter",m_news_filter))
    m_news_filter = NEWS_FILTER;

  // Create the map object (including prob and cost map)
  m_map = new SlamOmaticMap(m_map_size,m_cost_width,m_map_threshold,m_map_resolution);

  // Create the localizer object :
  m_localizer = new SlamOmaticLocalizer(m_map,m_interval_search);

  // Make the GenericSLAM using the previous made Map and Localizer :
  m_slam = new GenericSlam<uint8_t>(m_map,m_localizer);

  // Add a NewsFilter for not considering LiDAR points not included in the current map :
  m_slam->addFilter(new NewsFilter(m_map,m_news_filter));
  // Add a Density filter to apply resolution to pointcloud and keeping one point for each pair {X,Y} (integer values)
  m_slam->addFilter(new DensityFilter(m_map_resolution));
}

Pose2D SlamOmatic::update(const sensor_msgs::LaserScan::Ptr &msg)
{
  //  Conversion du scan en coordonnées polaires en coordonnées cartésiennes + filtrage bad value
  LiDAR_Scan_2D scan = Tools::convertRosScan2GenericLiDARScan(msg);

  // Main SLAM function implemented in the basis class GenericSLAM :
  // It is possible to add an estimated pose obtained using proprioceptive data like odometers or IMU... (using EKF, ...)
  // This estimated pose must be passed as the first parameter of the function update, in place of the data m_pose.
  m_pose = m_slam->update(m_pose,scan);

  // Return the current pose estimated by the SLAM :
  return m_pose;
}


SlamOmaticMap *SlamOmatic::map() const
{
  return m_map;
}

Pose2D SlamOmatic::pose() const
{
  return m_pose;
}
