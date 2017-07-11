#ifndef IGENERIC2DMAP_H
#define IGENERIC2DMAP_H


#include "Pose2D.h"
#include "generic_lidar_scan.h"
#include <nav_msgs/OccupancyGrid.h>

template<class T>
class IGeneric2dMap
{
public:
  // Getter and setter functions in probmap from XY value or directly from array pose id :
  virtual T at(const int x,const  int y) const = 0;
  virtual T& at(const int x,const  int y) = 0;

  virtual T at(const Pose2D &p) const = 0;
  virtual T& at(const Pose2D &p) = 0;

  virtual T at(const Point2D &p) const = 0;
  virtual T& at(const Point2D &p) = 0;

  // Getter and setter functions in costmap from XY value or directly from array pose id :
  virtual T costAt(const int x,const  int y) const = 0;
  virtual T costAt(const Pose2D &p) const = 0;
  virtual T costAt(const Point2D &p) const = 0;

  // Getter for cost max in costmap :
  virtual T costMax() const = 0;

  // Getter for resolution's map :
  virtual double resolution() const = 0;

  // Function to update the maps using the current pose and current LiDAR data scan :
  virtual void  update(const Pose2D& p, const LiDAR_Scan_2D& scan) = 0;

  // Getter on CostMap, convert array to ros::OccupencyGrid :
  virtual nav_msgs::OccupancyGrid getCost2OccupencyGrid() const = 0;

  // Getter on CostMap, convert array to ros::OccupencyGrid :
  virtual nav_msgs::OccupancyGrid getProb2OccupencyGrid() const = 0;

  //  Functions to save and load the maps :
//  virtual bool    save(const char* filename) = 0;
//  virtual bool    load(const char* filename) = 0;

};

#endif // IGENERIC2DMAP_H
