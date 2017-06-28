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

void ProbabilityGridMap::razChange()
{
  m_change.clear();
}

std::vector<Point2D> *ProbabilityGridMap::getChange()
{
  return &m_change;
}

ProbabilityGridMap::~ProbabilityGridMap() { }

void ProbabilityGridMap::pushLine(const int x1, const int y1, const int x2, const int y2, const bool force, const bool isFree) {
  int _x1 = x1, _x2=x2, _y1=y1, _y2=y2;

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

  // TODO ON A ENCORE UNE BUG DANS CETTE BOUCLE CI !!!
  //cout << "Z1"<<endl;
  for(int x=_x1; x<maxX; x++)
  {
    if(steep) {
      X=y; Y=x;
    } else {
      X=x; Y=y;
    }

    uint8_t lastval = at(X,Y);

    //    bool avant = lastval > 50, apres;
    //if(lastval <= 95) { // || lastval < 5)  {
    if(true) {
      static uint8_t seuil_haut = 100 - PROBA_DECREMENT, seuil_bas = PROBA_DECREMENT;
      if(isFree) {
        lastval <= seuil_bas ? lastval=0 : lastval-=PROBA_DECREMENT;
      } else {
        lastval <= seuil_haut ? lastval=100 : lastval+=PROBA_DECREMENT;
      }
      //    apres = lastval > 50;
      //    if(avant != apres) m_change.push_back(Point2D(X,Y));
      set(X,Y,lastval);
    }

    error -= dy;
    if(error < 0)
    {
      y += ystep;
      error += dx;
    }
  }
  //cout << "Z2"<<endl;
  if(force) {
    //cout << "Z21"<<endl;
    set(x2,y2,isFree?100:0);
  }
  else
  {
    //cout << "Z22"<<endl;
    uint8_t lastval = at(x2,y2);

    static uint8_t seuil_haut = 100 - PROBA_INCREMENT, seuil_bas = PROBA_INCREMENT;

    if(isFree) {
      lastval >= seuil_haut ? lastval=100 : lastval += PROBA_INCREMENT;
    } else {
      lastval <= seuil_bas  ? lastval=0   : lastval -= PROBA_INCREMENT;
    }
    //    if(isFree) {
    //      lastval >=92 ? lastval=100 : lastval+=8;
    //    } else {
    //      lastval <= 8 ? lastval=0 : lastval-=8;
    //    }
    set(x2,y2,lastval);
  }
}
