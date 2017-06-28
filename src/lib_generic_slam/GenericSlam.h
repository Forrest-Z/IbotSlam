#ifndef GenericSlam_H
#define GenericSlam_H

#include <vector>

#include "IGeneric2dMap.h"
#include "IGenericLocalizer.h"
#include "GenericFilter.h"

template<class T>
class GenericSlam
{
public:
  GenericSlam(IGeneric2dMap<T>* map, const IGenericLocalizer<T>* localizer);

  Pose2D    update(const Pose2D estimate, const Generic_Lidar_Scan<Point2D>& scan);

  void    addFilter(GenericFilter* filter);
  size_t  filterSize() const;

  Pose2D pix_pose() const;

  Pose2D met_pose() const;

  IGeneric2dMap<T> *map() const;

  const IGenericLocalizer<T> *localizer() const;

protected:
  Pose2D  m_pix_pose;
  Pose2D  m_met_pose;
  bool    m_first_up;

  IGeneric2dMap<T>*             m_map;
  const IGenericLocalizer<T>*   m_localizer;

  std::vector<GenericFilter*>   m_filter;
};

#include "GenericSlam.tpp"

#endif // GenericSlam_H
