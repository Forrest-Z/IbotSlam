//#include "GridMap.h"

template<typename T>
GridMap<T>::GridMap(unsigned int size, double resMpPix)
  : m_size( size % 2 ? size : size+1),
    m_resMpPix(resMpPix),
    m_globalSize(m_size*m_size),
    m_offset((m_size*m_size-1)/2),
    m_minX((-((int)m_size)+1)/2),
    m_maxX((m_size+1)/2)
{
  m_map = (T*) malloc(m_globalSize*sizeof(T));
}

template<typename T>
GridMap<T>::GridMap(const GridMap<T> *gridmap)
  : m_size(gridmap->m_size),
    m_resMpPix(gridmap->m_resMpPix),
    m_globalSize(gridmap->m_globalSize),
    m_offset(gridmap->m_offset),
    m_minX(gridmap->m_minX),
    m_maxX(gridmap->m_maxX)
{
  m_map = (T*)malloc(m_globalSize*sizeof(T));
}

template<typename T>
GridMap<T>::~GridMap()
{
  free(m_map);
}


template<typename T>
T GridMap<T>::at(int x, int y) const {
  //if(x < m_minX || x > m_maxX || y < m_minX || y > m_maxX )   throw new rOc_MapException(10,"access to out of coordinate point");
  // cout << "index : "<<-y*m_size+x+m_offset<< " included in [0;"<<m_size*m_size << endl;
  // cout << "offset : " << m_offset << endl;
  //return m_map[-y/m_resCmPerPix*m_size+x/m_resCmPerPix+m_offset];
  return m_map[-y*m_size+x+m_offset];
}

template<typename T>
T &GridMap<T>::at(int x, int y) {
  unsigned int id = -y*m_size+x+m_offset;
  if(id > m_globalSize) return m_map[0];
  return m_map[id];
}

template<typename T>
unsigned int GridMap<T>::idAt(const int x, const int y) const {
  unsigned int id = -y*m_size+x+m_offset;
  if(id > m_globalSize) return 0;
  return id;
}

template<typename T>
void GridMap<T>::xyAtId(const unsigned int id, int *x, int *y) const {
  int demSize = (m_size+1)/2;
  *x = id % m_size - demSize;
  *y = demSize - (id - id % m_size) / m_size;
}

template<typename T>
T GridMap<T>::at(unsigned int index) const {
  unsigned int id;
  if(index > m_globalSize) id = 0;
  else id = index;
  return m_map[id];
}

template<typename T>
T &GridMap<T>::at(unsigned int index){
  unsigned int id;
  if(index > m_globalSize) id = 0;
  else id = index;
  return m_map[id];
}

template<typename T>
void GridMap<T>::set(unsigned int x, unsigned int y, T data) {
  unsigned int id = -y*m_size+x+m_offset;
  if(id > m_globalSize) id = 0;
  m_map[id] = data;
}

template<typename T>
void GridMap<T>::set(unsigned int index, T data) {
  unsigned int id;
  if(index > m_globalSize) id = 0;
  else id = index;
  m_map[id] = data;
}

template<typename T>
unsigned int GridMap<T>::size() const { return m_size; }

template<typename T>
double GridMap<T>::resolutionMpPix() const { return m_resMpPix; }

template<typename T>
unsigned int GridMap<T>::globalsize() const { return m_globalSize; }

template<typename T>
T* GridMap<T>::data() const { return m_map; }

template<typename T>
unsigned int GridMap<T>::TopId(unsigned int index) const {
  /*if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
            throw rOc_MapException(180,"result out of map");
        }*/
  return index - m_size;
}

template<typename T>
unsigned int GridMap<T>::TopLeftId(unsigned int index) const {
  /*if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
            throw rOc_MapException(180,"result out of map");
        }*/
  return index - m_size - 1;
}

template<typename T>
unsigned int GridMap<T>::TopRightId(unsigned int index) const {
  //        if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
  //            throw rOc_MapException(180,"result out of map");
  //        }
  return index - m_size + 1;
}

template<typename T>
unsigned int GridMap<T>::BottomId(unsigned int index) const {
  //        if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
  //            throw rOc_MapException(180,"result out of map");
  //        }
  return index + m_size;
}

template<typename T>
unsigned int GridMap<T>::BottomLeftId(unsigned int index) const {
  //        if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
  //            throw rOc_MapException(180,"result out of map");
  //        }
  return index + m_size - 1;
}

template<typename T>
unsigned int GridMap<T>::BottomRightId(unsigned int index) const {
  //        if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
  //            throw rOc_MapException(180,"result out of map");
  //        }
  return index + m_size + 1;
}

template<typename T>
unsigned int GridMap<T>::RightId(unsigned int index) const {
  //        if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
  //            throw rOc_MapException(180,"result out of map");
  //        }
  return index + 1;
}

template<typename T>
unsigned int GridMap<T>::LeftId(unsigned int index) const {
  //        if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
  //            throw rOc_MapException(180,"result out of map");
  //        }
  return index - 1;
}

template<typename T>
T *GridMap<T>::TopValue(unsigned int index) const {
  //        if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
  //            throw rOc_MapException(180,"result out of map");
  //        }
  return &at(index - m_size);
}

template<typename T>
T *GridMap<T>::TopLeftValue(unsigned int index) const {
  //        if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
  //            throw rOc_MapException(180,"result out of map");
  //        }
  return &at(index - m_size - 1);
}

template<typename T>
T *GridMap<T>::TopRightValue(unsigned int index) const {
  //        if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
  //            throw rOc_MapException(180,"result out of map");
  //        }
  return &at(index - m_size + 1);
}

template<typename T>
T *GridMap<T>::BottomValue(unsigned int index) const {
  //        if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
  //            throw rOc_MapException(180,"result out of map");
  //        }
  return &at(index + m_size);
}

template<typename T>
T *GridMap<T>::BottomLeftValue(unsigned int index) const {
  //        if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
  //            throw rOc_MapException(180,"result out of map");
  //        }
  return &at(index + m_size - 1);
}

template<typename T>
T *GridMap<T>::BottomRightValue(unsigned int index) const {
  //        if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
  //            throw rOc_MapException(180,"result out of map");
  //        }
  return &at(index + m_size + 1);
}

template<typename T>
T *GridMap<T>::LeftValue(unsigned int index) const {
  //        if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
  //            throw rOc_MapException(180,"result out of map");
  //        }
  return &at(index - 1);
}

template<typename T>
T *GridMap<T>::RightValue(unsigned int index) const {
  //        if(index < m_size || index > m_globalSize-m_size || !(index%m_size) || !(index%(m_size+1))) {
  //            throw rOc_MapException(180,"result out of map");
  //        }
  return &at(index + 1);
}

template<typename T>
T GridMap<T>::operator()(int x, int y) const {
  return at(x,y);
}

template<typename T>
T GridMap<T>::operator()(unsigned int index) const {
  return at(index);
}
