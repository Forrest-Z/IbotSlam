#ifndef LOCALIZATIONONLYSLAM_H
#define LOCALIZATIONONLYSLAM_H

#include <vector>

#include "IGeneric2dMap.h"
#include "IGenericLocalizer.h"
#include "GenericFilter.h"

template<class T>
class LocalizationOnlySLam
{
public:
  LocalizationOnlySLam(IGeneric2dMap<T>* map, const IGenericLocalizer<T>* localizer);

  Pose2D                        update(const Pose2D estimate, const Generic_Lidar_Scan<Point2D>& scan_in_meter);

  Pose2D                        pix_pose() const;
  Pose2D                        met_pose() const;

  IGeneric2dMap<T>              *map() const;
  const IGenericLocalizer<T>    *localizer() const;

protected:
  Pose2D  m_pix_pose;
  Pose2D  m_met_pose;

  IGeneric2dMap<T>*             m_map;
  const IGenericLocalizer<T>*   m_localizer;
};

#include "LocalizationOnlySLam.tpp"

#endif // LOCALIZATIONONLYSLAM_H
