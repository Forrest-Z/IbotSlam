//#include "generic_lidar_scan.h"

template<class T>
Generic_Lidar_Scan<T>::Generic_Lidar_Scan(const unsigned int size)
{
    m_points.resize(size);
}

template<class T>
Generic_Lidar_Scan<T>::Generic_Lidar_Scan(const Generic_Lidar_Scan &scan)
{
    unsigned int size = scan.size();
    unsigned int i;
    m_points.resize(size);
    for(i = 0; i < size; i++)
    {
        m_points[i] = scan.m_points[i];
    }
}

template<class T>
void Generic_Lidar_Scan<T>::addPoint(const T &point)
{
    m_points.push_back(point);
}

template<class T>
void Generic_Lidar_Scan<T>::addPoints(const std::vector<T> &points)
{
    unsigned int i;
    unsigned int size = points.size();
    for(i = 0; i < size; i++)
        m_points.push_back(points[i]);
}

template<class T>
void Generic_Lidar_Scan<T>::removePoint(const T &point)
{
    typename std::vector<T>::iterator it;
    for(it = m_points.begin(); it != m_points.end(); it++)
        if(*it == point)
        {
            m_points.erase(it);
            return;
        }
}

template<class T>
unsigned int Generic_Lidar_Scan<T>::size() const
{
    return m_points.size();
}

template<class T>
void Generic_Lidar_Scan<T>::removePoints(const std::vector<T> &points)
{
    unsigned int i;
    unsigned int size = points.size();
    for(i = 0; i < size; i++)
        removePoint(points[i]);
}

template<class T>
void Generic_Lidar_Scan<T>::scale(const double scale)
{
    unsigned int _size = size();
    for(unsigned int i = 0; i < _size; i++)
    {
        m_points[i].x /= scale;
        m_points[i].y /= scale;
    }
}
