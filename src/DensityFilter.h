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
  /**
   * @brief filter  Function to delete some LiDAR Scan points for the localization part
   * @param lastPose  the last estimate pose known
   * @param scan      the current LiDAR scan that will be filtred
   * @return          the filtred LiDAR scan with one point max for each pair of X,Y in integer
   */
  LiDAR_Scan_2D filter(const Pose2D &lastPose, const LiDAR_Scan_2D &scan) const;


private:
  // The resolution of the map
  const double m_resolution;

};

#endif // DENSITYFILTER_H
