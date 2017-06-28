#ifndef GENERIC_POLAR_LIDAR_SCAN_H
#define GENERIC_POLAR_LIDAR_SCAN_H

#include "Point2D.h"

#include <vector>

template <class T>
class Generic_Lidar_Scan
{
public:
    Generic_Lidar_Scan(const unsigned int size=0);
    Generic_Lidar_Scan(const Generic_Lidar_Scan& scan);

    void        addPoint(const T& point);
    void        addPoints(const std::vector<T>& points);

    void        removePoint(const T& point);
    void        removePoints(const std::vector<T>& points);

    void        scale(const double scale);

    inline unsigned int size() const;

    std::vector<T>  m_points;

};

#include "generic_lidar_scan.tpp"

typedef Generic_Lidar_Scan<Point2D>   LiDAR_Scan_2D;

#endif // GENERIC_POLAR_LIDAR_SCAN_H
