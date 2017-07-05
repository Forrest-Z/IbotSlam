#include "ProbabilityGridMap.h"

using namespace std;

ProbabilityGridMap::ProbabilityGridMap(unsigned int size, double resMpPix)
  : GridMap<uint8_t>(size,resMpPix)
{
  memset(m_map,50,m_globalSize);
}

ProbabilityGridMap::ProbabilityGridMap(const ProbabilityGridMap &other)
  : GridMap<uint8_t>(other.m_size,other.m_resMpPix)
{
  memcpy(m_map,other.m_map,m_globalSize);
}

ProbabilityGridMap::~ProbabilityGridMap() { }

void ProbabilityGridMap::pushLine(const int x1, const int y1, const int x2, const int y2, const bool force, const bool isFree) {
  if(isFree)
    pushFreeLine(x1,y1,x2,y2,force);
//else
//  #TODO make pushNotFreeLine ...
}

void ProbabilityGridMap::pushFreeLine(const int x1, const int y1, const int x2, const int y2, const bool force)
{
  int _x1 = x1, _x2=x2, _y1=y1, _y2=y2;
  unsigned int loc_id;
  uint8_t val;

  bool steep = abs(_y2-_y1) > abs(_x2-_x1);

  if(steep) {
    std::swap<int>(_x1,_y1);
    std::swap<int>(_x2,_y2);
  }

  if(_x1 > _x2) {
    std::swap<int>(_x1,_x2);
    std::swap<int>(_y1,_y2);
  }

  int dx = _x2 - _x1;
  int dy = abs(_y2 - _y1);

  double error = dx/2;
  const int ystep = (_y1 < _y2) ? 1 : -1;

  int y = (int)_y1;
  const int maxX = (int)_x2;
  int X=0,Y=0;

  for(int x=_x1; x<maxX; x++)
  {
    if(steep) {
      X=y; Y=x;
    } else {
      X=x; Y=y;
    }

    loc_id = idAt(X,Y);
    val = at(loc_id);

    static uint8_t seuil_bas = PROBA_DECREMENT;
    val <= seuil_bas ? val=0 : val-=PROBA_DECREMENT;

    at(loc_id) = val;

    error -= dy;
    if(error < 0)
    {
      y += ystep;
      error += dx;
    }
  }

  if(force) {
    //cout << "Z21"<<endl;
    set(x2,y2,100);
  }
  else
  {
    val = at(x2,y2);

    static uint8_t seuil_haut = 100 - PROBA_INCREMENT;
    val = (val >= seuil_haut ? val=100 : val += PROBA_INCREMENT);
    at(x2,y2) = val;
  }
}
