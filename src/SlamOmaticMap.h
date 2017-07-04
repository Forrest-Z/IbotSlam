#ifndef SLAMOMATICMAP_H
#define SLAMOMATICMAP_H

#include "lib_generic_slam/IGeneric2dMap.h"
#include "lib_generic_slam/Pose2D.h"

#include "slamOmaticV1/GridMap.h"
#include "slamOmaticV1/CostMap.h"

#include "define.h"

#include <mutex>


class SlamOmaticMap : public IGeneric2dMap<uint8_t>
{
public:
  SlamOmaticMap(const unsigned int size, const uint8_t costMax, const uint8_t seuil, const double resMpPix);

  // IGeneric2dMap interface
public:
  virtual uint8_t at(const int x, const int y) const;
  virtual uint8_t &at(const int x, const int y);
  virtual uint8_t at(const Pose2D &p) const;
  virtual uint8_t &at(const Pose2D &p);
  virtual uint8_t at(const Point2D &p) const;
  virtual uint8_t &at(const Point2D &p);

  virtual uint8_t costAt(const int x, const int y) const;
  virtual uint8_t costAt(const Pose2D &p) const;
  virtual uint8_t costAt(const Point2D &p) const;
  virtual uint8_t costMax() const;

  virtual double resolution() const;

  virtual void update(const Pose2D &p, const LiDAR_Scan_2D &scan);

  virtual nav_msgs::OccupancyGrid getCost2OccupencyGrid() const;
  virtual nav_msgs::OccupancyGrid getProb2OccupencyGrid() const;

  virtual bool save(const char *filename);
  virtual bool load(const char *filename);

protected:
  FILE*                 m_file;

  bool                  m_first_update;
  uint8_t               m_seuil, m_maxcost;
  mutable std::mutex    m_mutex_prob, m_mutex_cost;
  ProbabilityGridMap    *m_probability_map;
  CostMap              *m_cost_map;

};

#endif // SLAMOMATICMAP_H
