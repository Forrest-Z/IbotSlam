#ifndef SLAMOMATICLOCALIZER_H
#define SLAMOMATICLOCALIZER_H

#include "lib_generic_slam/IGenericLocalizer.h"

#include "slamOmaticV1/nmsimplex.h"
#include "slamOmaticV1/CostMap.h"


class SlamOmaticLocalizer : public IGenericLocalizer<uint8_t>
{
public:
  SlamOmaticLocalizer(IGeneric2dMap<uint8_t> *map, const double interval);

  // IGenericLocalizer interface
public:
  Pose2D localise(const Pose2D &last, const IGeneric2dMap<uint8_t> *map, const Generic_Lidar_Scan<Point2D> &scan) const;

protected:
  const double m_interval;



  static IGeneric2dMap<uint8_t>*       m_local_map;
  static LiDAR_Scan_2D*                m_last_scan;

  static double    cost_func(double *coord);
  static void constraint_func(double coord[],int n);
};

#endif // SLAMOMATICLOCALIZER_H

