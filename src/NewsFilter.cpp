#include "NewsFilter.h"

using namespace std;

NewsFilter::NewsFilter(const IGeneric2dMap<uint8_t> *map, const double objective_max)
  : m_objective_max(objective_max/map->resolution()), m_map(map)
{

}

LiDAR_Scan_2D NewsFilter::filter(const Pose2D& lastPoseM, const LiDAR_Scan_2D &scan) const
{
  const double res(m_map->resolution());
  const unsigned int size(scan.size());
  LiDAR_Scan_2D out;
  //const T cost_max(m_map->costMax());
  unsigned int i, nb_ok = 0, nb_zero = 0;

  double _cos = cos(lastPoseM.theta), _sin = sin(lastPoseM.theta);
  int x, y;

  for(i = 0; i < size; i++)
  {
    if(scan.m_points[i].x != 0 && scan.m_points[i].y != 0)
    {
      x = _cos*scan.m_points[i].x - _sin*scan.m_points[i].y + lastPoseM.x;
      y = _sin*scan.m_points[i].x + _cos*scan.m_points[i].y + lastPoseM.y;
      if(m_map->costAt(x/res, y/res) < m_objective_max)
      {
        nb_ok++;
        out.addPoint(scan.m_points[i]);
      }
    } else
      nb_zero++;
  }

  cout << "LE FILTRE VALIDE " << (int)nb_ok << " points sur " << (int)(size-nb_zero) << " points dont ("<<nb_zero<< "zeros) " << (double) ((100*nb_ok) / (size-nb_zero)) << "%" << endl;
  return out;
}
