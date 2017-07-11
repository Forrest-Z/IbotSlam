#ifndef NEWSFILTER_H
#define NEWSFILTER_H

#include <math.h>

#include <iostream>

#include "lib_generic_slam/IGeneric2dMap.h"
#include "lib_generic_slam/GenericFilter.h"

class NewsFilter : public GenericFilter
{
public:
  NewsFilter(const IGeneric2dMap<uint8_t> *map, const double objective_max);

  // GenericFilter interface
public:
  /**
   * @brief filter  Function to delete some LiDAR Scan points for the localization part
   * @param lastPose  the last estimate pose known
   * @param scan      the current LiDAR scan that will be filtred
   * @return          the filtred LiDAR scan
   */
  LiDAR_Scan_2D filter(const Pose2D& lastPose, const LiDAR_Scan_2D &scan) const;

protected:
  // The lenght threshold in pixel (not in meter, it is converted in the constructor) :
  const double m_objective_max;

  // Local pointer on the global map to get cost of each point in the costmap
  const IGeneric2dMap<uint8_t>* m_map;


};


#endif // NEWSFILTER_H
