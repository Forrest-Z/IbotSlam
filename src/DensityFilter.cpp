#include "DensityFilter.h"

DensityFilter::DensityFilter(const double resolution)
  : m_resolution(resolution)
{

}

LiDAR_Scan_2D DensityFilter::filter(const Pose2D &lastPosePix, const LiDAR_Scan_2D &scan) const
{
  const unsigned int size(scan.size());
  unsigned int i;
  LiDAR_Scan_2D tmp(scan), filtred(scan);

  tmp.scale(m_resolution);
  tmp.m_points[0].x = (int)tmp.m_points[0].x;
  tmp.m_points[0].y = (int)tmp.m_points[0].y;
  for(i = 1; i < size; i++)
  {
    tmp.m_points[i].x = (int)tmp.m_points[i].x;
    tmp.m_points[i].y = (int)tmp.m_points[i].y;

    if(tmp.m_points[i].x == tmp.m_points[i-1].x && tmp.m_points[i].y == tmp.m_points[i-1].y)
    {
      //      filtred.m_points[i].x = 0;
      //      filtred.m_points[i].y = 0;
      tmp.m_points[i].x = 0;
      tmp.m_points[i].y = 0;
    }
  }

  if(tmp.m_points[0].x == filtred.m_points[size-1].x && tmp.m_points[0].y == filtred.m_points[size-1].y)
  {
    //      filtred.m_points[i].x = 0;
    //      filtred.m_points[i].y = 0;
    tmp.m_points[i].x = 0;
    tmp.m_points[i].y = 0;
  }

  std::vector<Point2D>::iterator it;
  it = std::unique (tmp.m_points.begin(), tmp.m_points.end());
  tmp.m_points.resize( std::distance(tmp.m_points.begin(),it) );

  std::cout << "LE FILTRE DE DENSITE VALIDE " << tmp.size() <<
               " points parmi " << scan.size() <<
               " soit " << (100*tmp.size() / scan.size()) << " %" << std::endl;

  return tmp;
}
