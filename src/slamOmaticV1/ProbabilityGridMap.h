#ifndef ROC_PROBABILITYGRIDMAP_H
#define ROC_PROBABILITYGRIDMAP_H

#include <iostream>
#include <math.h>
#include <cmath>
#include <stdint.h>

#include "../define.h"

#include "GridMap.h"
#include "../lib_generic_slam/Point2D.h"
#include <stdio.h>
#include <string.h>

class ProbabilityGridMap : public GridMap<uint8_t>
{
public:
  ProbabilityGridMap(unsigned int size, double resMpPix);

  ProbabilityGridMap(const ProbabilityGridMap& other);

  virtual ~ProbabilityGridMap();

  void pushLine(const int x1, const int y1, const int x2, const int y2, const bool force=false, const bool isFree=true);

protected:
  void pushFreeLine(const int x1, const int y1, const int x2, const int y2, const bool force=false);

};

#endif // ROC_PROBABILITYGRIDMAP_H
