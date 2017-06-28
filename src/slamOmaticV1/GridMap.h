#ifndef ROC_GRIDMAP_H
#define ROC_GRIDMAP_H

#include <iostream>
#include <vector>
#include <string>

#include <stdio.h>
#include <stdlib.h>

using namespace std;

template<typename T>
class GridMap
{
public:
    GridMap(unsigned int size, double resCmPerPix);

    GridMap(const GridMap<T> *gridmap);

    virtual ~GridMap();

    T at(int x, int y) const;

    T& at(int x, int y);

    unsigned int idAt(const int x, const int y) const;

    void xyAtId(const unsigned int id, int *x, int *y) const;

    T at(unsigned int index) const;

    T& at(unsigned int index);

    T operator()(int x, int y) const;

    T operator()(unsigned int index) const;

    void                set(unsigned int x, unsigned int y, T data);

    void                set(unsigned int index, T data);

    unsigned int        size()     const;
    double              resolutionMpPix() const;
    unsigned int        globalsize()     const;
    T*                  data()              const;

protected :
    unsigned int      m_size;
    int               m_minX, m_maxX;
    unsigned int      m_globalSize;
    unsigned int      m_offset;
    double            m_resMpPix;
    T                      *m_map;

    //  ______________________________
    //  ::: To navigate in the map :::

    unsigned int TopId(unsigned int index) const;

    unsigned int TopLeftId(unsigned int index) const;

    unsigned int TopRightId(unsigned int index) const;

    unsigned int BottomId(unsigned int index) const;

    unsigned int BottomLeftId(unsigned int index) const;

    unsigned int BottomRightId(unsigned int index) const;

    unsigned int RightId(unsigned int index) const;

    unsigned int LeftId(unsigned int index) const;

    //  ____________________________________________
    //  ::: To navigate and get value in the map :::
    T* TopValue(unsigned int index) const;

    T* TopLeftValue(unsigned int index) const;

    T* TopRightValue(unsigned int index) const;

    T* BottomValue(unsigned int index) const;

    T* BottomLeftValue(unsigned int index) const;

    T* BottomRightValue(unsigned int index) const;

    T* LeftValue(unsigned int index) const;

    T* RightValue(unsigned int index) const;




};


#include "GridMap.tpp"

#endif // ROC_GRIDMAP_H
