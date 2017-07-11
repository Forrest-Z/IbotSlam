#ifndef IGENERICLOCALIZER_H
#define IGENERICLOCALIZER_H

#include "Pose2D.h"
#include "generic_lidar_scan.h"
#include "IGeneric2dMap.h"

template<class T>
class IGenericLocalizer
{
public:
  /**
   * @brief localise function to make an estimation of the robot pose using the last pose, the generic map and LiDAR data scan
   * @param last : the last pose known (could be the output from odometer integration, IMU, EKF, etc.)
   * @param map : the map at the last iteration
   * @param scan : the current LiDAR data object
   * @return the estimation of the current pose of the LiDAR in the map
   */
  virtual Pose2D localise(const Pose2D& last, const IGeneric2dMap<T>* map, const Generic_Lidar_Scan<Point2D>& scan) const = 0;
};

#endif // IGENERICLOCALIZER_H
