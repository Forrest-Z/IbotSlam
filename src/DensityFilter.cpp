#include "DensityFilter.h"

DensityFilter::DensityFilter(const double resolution)
  : m_resolution(resolution)
{

}

LiDAR_Scan_2D DensityFilter::filter(const Pose2D &lastPose, const LiDAR_Scan_2D &scan) const
{
  const unsigned int size(scan.size());
  unsigned int i;
  LiDAR_Scan_2D tmp(scan), filtred(scan);

  // Scale all data in scan to the map resolution passed to the constructor :
  tmp.scale(m_resolution);

  // For each point in the pointcloud, apply a cast to transform double value to integer value :
  for(i = 0; i < size; i++)
  {
    tmp.m_points[i].x = (int)tmp.m_points[i].x;
    tmp.m_points[i].y = (int)tmp.m_points[i].y;
  }

  // Erase every point equal to one other :
  // We want only one point of each pair {X,Y}
  std::vector<Point2D>::iterator it;
  it = std::unique (tmp.m_points.begin(), tmp.m_points.end());
  tmp.m_points.resize( std::distance(tmp.m_points.begin(),it) );

  // Return the filtred and scaled LiDAR Scan :
  return tmp;
}
