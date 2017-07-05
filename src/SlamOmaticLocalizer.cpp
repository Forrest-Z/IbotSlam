#include "SlamOmaticLocalizer.h"


IGeneric2dMap<uint8_t>*       SlamOmaticLocalizer::m_local_map = NULL;
LiDAR_Scan_2D*                SlamOmaticLocalizer::m_last_scan = NULL;

SlamOmaticLocalizer::SlamOmaticLocalizer(IGeneric2dMap<uint8_t> *map, const double interval)
  : m_interval(interval)
{
  m_local_map = map;
}

Pose2D SlamOmaticLocalizer::localise(const Pose2D &last, const IGeneric2dMap<uint8_t> *map, const Generic_Lidar_Scan<Point2D> &scan) const
{
  m_last_scan = (LiDAR_Scan_2D*)&scan;
  double start[] = {last.x,last.y,last.theta};  // Position initiale
  double min;
  int dim = 3;
  double eps = 1.0e-8;
  double scale = m_interval;

  CostFunction _cost_func = SlamOmaticLocalizer::cost_func;
  ConstraintFunction _constraint_func = SlamOmaticLocalizer::constraint_func;

  min=simplex(_cost_func,start,dim,eps,scale,_constraint_func);

  return Pose2D(start[0],start[1],start[2]);
}

double SlamOmaticLocalizer::cost_func(double *coord)
{
  static double map_res = m_local_map->resolution();
  const unsigned int size(m_last_scan->size());

  double cost = 0.0f;                     // Global pose computed

  double p_m_x = coord[0]/map_res;        // Pose.X
  double p_m_y = coord[1]/map_res;        // Pose.Y
  double p_t = coord[2];                  // Pose.theta

  double _cos = cos(p_t), _sin = sin(p_t);

  //double cost = 0.0f;

  for(unsigned int i = 0; i < size; i++)
  {
    if(m_last_scan->m_points[i].x == 0 && m_last_scan->m_points[i].y == 0) continue;

    cost += (double)m_local_map->costAt((int)((_cos*m_last_scan->m_points[i].x - _sin*m_last_scan->m_points[i].y + p_m_x)),
                                        (int)((_sin*m_last_scan->m_points[i].x + _cos*m_last_scan->m_points[i].y + p_m_y)) );
  }

  return cost;
}

void SlamOmaticLocalizer::constraint_func(double coord[], int n)
{

}
