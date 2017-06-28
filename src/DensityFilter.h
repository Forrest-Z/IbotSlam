#ifndef DENSITYFILTER_H
#define DENSITYFILTER_H

#include <math.h>
#include <iostream>
#include "lib_generic_slam/GenericFilter.h"

class DensityFilter : public GenericFilter
{
public:
  DensityFilter(const double resolution);

  // GenericFilter interface
public:
  LiDAR_Scan_2D filter(const Pose2D &lastPosePix, const LiDAR_Scan_2D &scan) const;


private:
  const double m_resolution;

};

#endif // DENSITYFILTER_H
