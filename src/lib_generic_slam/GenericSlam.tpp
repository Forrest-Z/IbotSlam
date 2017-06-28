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
  static double seuil = 0.02*0.02;
  static unsigned long long last_t;
  static Pose2D last_pose;

  unsigned int i, size = m_filter.size();
  m_met_pose.theta = estimate.theta;  // On se fie à l'IMU

  // On créé une copie du scan :
  Generic_Lidar_Scan<Point2D> loc_scan(scan);

  // Si initialisation OK : (localisation)
  if(!m_first_up){
    // On applique l'ensemble des filtres à ce scan :
    Generic_Lidar_Scan<Point2D> filtred_scan(scan);
    for(i = 0; i < size; i++)
      filtred_scan = m_filter.at(i)->filter(m_met_pose,filtred_scan);

    Pose2D newPose = m_localizer->localise(m_met_pose,m_map,filtred_scan);
    double dx = newPose.x - last_pose.x, dy = newPose.y - last_pose.y;
    double dist = dx*dx + dy*dy;
    m_met_pose = newPose;

    cout << "Pose [M] : " << m_met_pose.x << ", " << m_met_pose.y << endl;
    m_pix_pose.x = (int)(m_met_pose.x / m_map->resolution());
    m_pix_pose.y = (int)(m_met_pose.y / m_map->resolution());
    m_pix_pose.theta = m_met_pose.theta;
    cout << "Pose [pix] : " << m_pix_pose.x << ", " << m_pix_pose.y << endl;
    unsigned long long t = ros::Time::now().toSec();
    //if(dist >= seuil) { // || t != last_t) {
    m_map->update(m_met_pose,scan);
    last_pose = m_met_pose;
    last_t = t;
    //}

  } else {

    //loc_scan.scale(m_map->resolution());
    //  std::vector<Point2D>::iterator it;
    //  it = std::unique (loc_scan.m_points.begin(), loc_scan.m_points.end());
    //  loc_scan.m_points.resize( std::distance(loc_scan.m_points.begin(),it) );

    m_map->update(m_met_pose,scan);
    last_pose = m_met_pose;
    last_t = ros::Time::now().toSec();
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
