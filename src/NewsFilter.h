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
  LiDAR_Scan_2D filter(const Pose2D& lastPoseM, const LiDAR_Scan_2D &scan) const;

protected:
  const double m_objective_max;
  const IGeneric2dMap<uint8_t>* m_map;


};


#endif // NEWSFILTER_H
