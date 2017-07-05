//#include "GenericSlam.h"

using namespace std;

template<class T>
GenericSlam<T>::GenericSlam(IGeneric2dMap<T> *map, const IGenericLocalizer<T> *localizer)
  : m_map(map), m_localizer(localizer), m_first_up(true)
{

}

template<class T>
Pose2D GenericSlam<T>::update(const Pose2D estimate, const Generic_Lidar_Scan<Point2D> &scan) // SCAN EN [M]
{
  unsigned int i, size = m_filter.size();
  m_met_pose.theta = estimate.theta;  // On se fie à l'IMU

  // Si initialisation OK : (localisation)
  if(!m_first_up){
    // On applique l'ensemble des filtres à ce scan :
    Generic_Lidar_Scan<Point2D> filtred_scan(scan);
    for(i = 0; i < size; i++)
      filtred_scan = m_filter.at(i)->filter(m_met_pose,filtred_scan);

    m_met_pose = m_localizer->localise(m_met_pose,m_map,filtred_scan);
    m_map->update(m_met_pose,scan);

  } else {
    m_map->update(m_met_pose,scan);
    m_first_up = false;
  }

  return m_met_pose;
}

template<class T>
void GenericSlam<T>::addFilter(GenericFilter *filter)
{
  m_filter.push_back(filter);
}

template<class T>
size_t GenericSlam<T>::filterSize() const
{
  return m_filter.size();
}

template<class T>
Pose2D GenericSlam<T>::pix_pose() const
{
  return m_pix_pose;
}

template<class T>
Pose2D GenericSlam<T>::met_pose() const
{
  return m_met_pose;
}

template<class T>
IGeneric2dMap<T> *GenericSlam<T>::map() const
{
  return m_map;
}

template<class T>
const IGenericLocalizer<T> *GenericSlam<T>::localizer() const
{
  return m_localizer;
}
