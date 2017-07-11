#ifndef GENERICFILTER_H
#define GENERICFILTER_H


#include "generic_lidar_scan.h"
#include "Pose2D.h"


class GenericFilter
{
public:
  /**
   * @brief filter  Function to delete some LiDAR Scan points for the localization part
   * @param lastPose  the last estimate pose known
   * @param scan      the current LiDAR scan that will be filtred
   * @return          the filtred LiDAR scan
   */
  virtual LiDAR_Scan_2D   filter(const Pose2D& lastPose, const LiDAR_Scan_2D& scan) const = 0;
};

#endif // GENERICFILTER_H
