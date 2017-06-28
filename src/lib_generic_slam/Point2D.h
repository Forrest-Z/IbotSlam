#ifndef POINT2D_H
#define POINT2D_H


class Point2D
{
public:
  Point2D(const double x=0.0f, const double y=0.0f);
  Point2D(const Point2D& other);

  double x,y;
  Point2D &operator =(const Point2D &other);
  bool    operator ==(const Point2D &other);
};

#endif // POINT2D_H
