#ifndef SLAMOMATICMAP_H
#define SLAMOMATICMAP_H

#include "lib_generic_slam/IGeneric2dMap.h"
#include "lib_generic_slam/Pose2D.h"

#include "slamOmaticV1/GridMap.h"
#include "slamOmaticV1/CostMap.h"

#include "define.h"



class SlamOmaticMap : public IGeneric2dMap<uint8_t>
{
public:
  SlamOmaticMap(const unsigned int size, const uint8_t costMax, const uint8_t seuil, const double resMpPix);

  // IGeneric2dMap interface
public:
  // Getter and setter functions in probmap from XY value or directly from array pose id :
  virtual uint8_t at(const int x, const int y) const;
  virtual uint8_t &at(const int x, const int y);
  virtual uint8_t at(const Pose2D &p) const;
  virtual uint8_t &at(const Pose2D &p);
  virtual uint8_t at(const Point2D &p) const;
  virtual uint8_t &at(const Point2D &p);

  // Getter and setter functions in costmap from XY value or directly from array pose id :
  virtual uint8_t costAt(const int x, const int y) const;
  virtual uint8_t costAt(const Pose2D &p) const;
  virtual uint8_t costAt(const Point2D &p) const;
  virtual uint8_t costMax() const;

  // Getter for resolution's map :
  virtual double resolution() const;

  // Function to update the maps using the current pose and current LiDAR data scan :
  virtual void update(const Pose2D &p, const LiDAR_Scan_2D &scan);

  // Getter on CostMap, convert array to ros::OccupencyGrid :
  virtual nav_msgs::OccupancyGrid getCost2OccupencyGrid() const;

  // Getter on ProbMap, convert array to ros::OccupencyGrid :
  virtual nav_msgs::OccupancyGrid getProb2OccupencyGrid() const;

  //  Functions to save and load the maps :
  //  virtual bool save(const char *filename);
  //  virtual bool load(const char *filename);

protected:
  // parameters for configuring maps :
  uint8_t               m_seuil, m_maxcost;
  // the probabilistic map object :
  ProbabilityGridMap    *m_probability_map;
  // the cost map object :
  CostMap               *m_cost_map;
};

#endif // SLAMOMATICMAP_H
