#ifndef IGENERIC2DMAP_H
#define IGENERIC2DMAP_H


#include "Pose2D.h"
#include "generic_lidar_scan.h"
#include <nav_msgs/OccupancyGrid.h>

template<class T>
class IGeneric2dMap
{
public:
  virtual T at(const int x,const  int y) const = 0;
  virtual T& at(const int x,const  int y) = 0;

  virtual T at(const Pose2D &p) const = 0;
  virtual T& at(const Pose2D &p) = 0;

  virtual T at(const Point2D &p) const = 0;
  virtual T& at(const Point2D &p) = 0;

  virtual T costAt(const int x,const  int y) const = 0;
  virtual T costAt(const Pose2D &p) const = 0;
  virtual T costAt(const Point2D &p) const = 0;
  virtual T costMax() const = 0;

  virtual double resolution() const = 0;

  virtual void  update(const Pose2D& p, const LiDAR_Scan_2D& scan) = 0;
  virtual nav_msgs::OccupancyGrid getCost2OccupencyGrid() const = 0;
  virtual nav_msgs::OccupancyGrid getProb2OccupencyGrid() const = 0;

  virtual bool    save(const char* filename) = 0;
  virtual bool    load(const char* filename) = 0;

};

#endif // IGENERIC2DMAP_H
