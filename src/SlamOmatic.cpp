#include "SlamOmatic.h"

SlamOmatic::SlamOmatic()
  : m_last_t_in_sec(ros::Time::now().toSec())
{
  /*
    <param name="map_resolution" value="0.050"/>
    <param name="map_size" value="2048"/>
    <param name="map_start_x" value="0.5"/>
    <param name="map_start_y" value="0.5" />

  double                  m_map_resolution;
  unsigned int            m_map_size;
  double                  m_map_start_x, m_map_start_y;
  uint8_t                 m_map_threshold;
  uint8_t                 m_cost_width;
  double                  m_interval_search;
  uint8_t                 m_proba_increment;
  double                  m_news_filter;

   */
  ros::NodeHandle n;

  if(!n.getParam("map_resolution",(double&)m_map_resolution))
    m_map_resolution = MAP_RESOLUTION;

  if(!n.getParam("map_size",(int&)m_map_size))
    m_map_size = MAP_SIZE;

  /// Unused ...
  //  if(!ros::param::get("map_start_x",&m_map_start_x))
  //    m_map_start_x = 0.5f;
  //  if(!ros::param::get("map_start_y",&m_map_start_y))
  //    m_map_start_y = 0.5f;

  if(!n.getParam("map_threshold",(int&)m_map_threshold))
    m_map_threshold = MAP_SEUIL;

  if(!n.getParam("map_cost_width",(int&)m_cost_width))
    m_cost_width = MAP_COSTMAX;

  if(!n.getParam("map_interval_search",m_interval_search))
    m_interval_search = INTER_SEARCH;

  if(!n.getParam("map_news_filter",m_news_filter))
    m_news_filter = NEWS_FILTER;

  m_map = new SlamOmaticMap(m_map_size,m_cost_width,m_map_threshold,m_map_resolution);
  m_localizer = new SlamOmaticLocalizer(m_map,m_interval_search);
  m_slam = new GenericSlam<uint8_t>(m_map,m_localizer);

  m_slam->addFilter(new NewsFilter(m_map,m_news_filter));
  m_slam->addFilter(new DensityFilter(m_map_resolution));
}

Pose2D SlamOmatic::update(const sensor_msgs::LaserScan::Ptr &msg)
{
  //  Conversion du scan en coordonnées polaires en coordonnées cartésiennes + filtrage bad value
  LiDAR_Scan_2D scan = Tools::convertRosScan2GenericLiDARScan(msg);
  //  Suppression des points isolés :
  //scan = Tools::filterByDistance(scan,SEUIL_DISTANCE);

  m_pose = m_slam->update(m_estimate_pose,scan);
  m_estimate_pose = m_pose;

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
