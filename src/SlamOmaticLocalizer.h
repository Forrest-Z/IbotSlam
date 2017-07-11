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
  /**
   * @brief localise function to make an estimation of the robot pose using the last pose, the generic map and LiDAR data scan
   * @param last : the last pose known (could be the output from odometer integration, IMU, EKF, etc.)
   * @param map : the map at the last iteration
   * @param scan : the current LiDAR data object
   * @return the estimation of the current pose of the LiDAR in the map
   */
  Pose2D localise(const Pose2D &last, const IGeneric2dMap<uint8_t> *map, const Generic_Lidar_Scan<Point2D> &scan) const;

protected:
  // Nelder & Mead interval :
  const double m_interval;

  // Local pointer to access the map data :
  static IGeneric2dMap<uint8_t>*       m_local_map;

  // Local pointer to access the laser data :
  static LiDAR_Scan_2D*                m_last_scan;

  // Local cost function to estimate a cost from laser scan and the map at position gived using the array *coord :
  /**
   * @brief cost_func : Local cost function to estimate a cost (used by Nelder & Mead optimisation algorithms)
   * @param coord the coordinates of the pose tested (like an array of dim 3 {X,Y,Theta})
   * @return The Cost like the sum of the cost of LiDAR data transformed into the map using coord
   */
  static double    cost_func(double *coord);

  /**
   * @brief constraint_func function to represent fixed constraints like not negative, or other (not used but necessary for the algo)
   * @param coord the coordinates of the pose tested (like an array of dim 3 {X,Y,Theta})
   * @param n ...
   */
  static void constraint_func(double coord[],int n);
};

#endif // SLAMOMATICLOCALIZER_H

