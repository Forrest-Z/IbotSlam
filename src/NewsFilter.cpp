#include "NewsFilter.h"

using namespace std;

NewsFilter::NewsFilter(const IGeneric2dMap<uint8_t> *map, const double objective_max)
  : m_objective_max(objective_max/map->resolution()), m_map(map)
{

}

LiDAR_Scan_2D NewsFilter::filter(const Pose2D& lastPose, const LiDAR_Scan_2D &scan) const
{
  const double res(m_map->resolution());
  const unsigned int size(scan.size());
  // The valid pointcloud
  LiDAR_Scan_2D out;
  unsigned int i;

  // Get parameter for the 2D geometrical transformation :
  double _cos = cos(lastPose.theta), _sin = sin(lastPose.theta);

  // For each point in the LiDAR Scan object :
  for(i = 0; i < size; i++)
  {
    // IF : Point is not on {0,0} and if its cost is smaller than the threshold, then it is added to the valid pointcloud :
    if(scan.m_points[i].x != 0 && scan.m_points[i].y != 0 &&
       m_map->costAt((_cos*scan.m_points[i].x - _sin*scan.m_points[i].y + lastPose.x)/res,
                     (_sin*scan.m_points[i].x + _cos*scan.m_points[i].y + lastPose.y)/res) < m_objective_max)
    {
      out.addPoint(scan.m_points[i]);
    }
  }

  // Return the valid pointcloud :
  return out;
}
