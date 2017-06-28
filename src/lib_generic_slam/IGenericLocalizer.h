#ifndef IGENERICLOCALIZER_H
#define IGENERICLOCALIZER_H

#include "Pose2D.h"
#include "generic_lidar_scan.h"
#include "IGeneric2dMap.h"

template<class T>
class IGenericLocalizer
{
public:
  virtual Pose2D localise(const Pose2D& last, const IGeneric2dMap<T>* map, const Generic_Lidar_Scan<Point2D>& scan) const = 0;
};

#endif // IGENERICLOCALIZER_H
