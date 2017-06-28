//#include "LocalizationOnlySLam.h"

template<class T>
LocalizationOnlySLam<T>::LocalizationOnlySLam(IGeneric2dMap<T> *map, const IGenericLocalizer<T> *localizer)
  : m_map(map), m_localizer(localizer)
{

}

template<class T>
Pose2D LocalizationOnlySLam<T>::update(const Pose2D estimate, const Generic_Lidar_Scan<Point2D> &scan_in_meter)
{
  // On créé une copie du scan :
  Generic_Lidar_Scan<Point2D> loc_scan(scan);
  loc_scan.scale(m_map->resolution());

  m_met_pose = m_localizer->localise(m_met_pose,m_map,filtred_scan);

  m_pix_pose.x = (int)(m_met_pose.x / m_map->resolution());
  m_pix_pose.y = (int)(m_met_pose.y / m_map->resolution());
  m_pix_pose.theta = m_met_pose.theta;

  return m_met_pose;
}

template<class T>
Pose2D LocalizationOnlySLam<T>::pix_pose() const
{
  return m_pix_pose;
}

template<class T>
Pose2D LocalizationOnlySLam<T>::met_pose() const
{
  return m_met_pose;
}

template<class T>
IGeneric2dMap<T> *LocalizationOnlySLam<T>::map() const
{
  return m_localizer;
}

template<class T>
const IGenericLocalizer<T> *LocalizationOnlySLam<T>::localizer() const
{
  return m_map;
}
