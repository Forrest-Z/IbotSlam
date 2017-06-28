#ifndef ROC_COSTMAP_H
#define ROC_COSTMAP_H

#include <math.h>

#include <vector>
#include <iostream>

#include "GridMap.h"
#include "ProbabilityGridMap.h"


///             ################
///             ### COST_MAP ###
///             ################

class CostMap : public GridMap<uint8_t>
{
public:
  CostMap(const uint8_t fixedmax, unsigned int size, double resMpPix);
  CostMap(const ProbabilityGridMap& prob, const uint8_t costmax=255, const uint8_t seuil = 35);
  virtual ~CostMap();

  uint8_t     max()       const { return m_max; }
  uint8_t     realMax()   const { return m_realMax; }

protected:
  void computeALL(const std::vector<unsigned int>* obstacles);

  //inline void computeRemoving(const unsigned int id);
  //inline void computeAdding(const std::vector<unsigned int> &added);

private:
  uint8_t           m_max;
  uint8_t           m_realMax;
};

#endif // ROC_COSTMAP_H
