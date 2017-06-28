#ifndef GENERICFILTER_H
#define GENERICFILTER_H


#include "generic_lidar_scan.h"
#include "Pose2D.h"


class GenericFilter
{
public:
  virtual LiDAR_Scan_2D   filter(const Pose2D& lastPosePix, const LiDAR_Scan_2D& scan) const = 0;
};

#endif // GENERICFILTER_H
