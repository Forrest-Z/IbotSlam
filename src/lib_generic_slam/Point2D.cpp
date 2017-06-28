#include "Point2D.h"

Point2D::Point2D(const double x, const double y)
  : x(x), y(y)
{

}

Point2D::Point2D(const Point2D &other)
  : x(other.x), y(other.y)
{

}

Point2D &Point2D::operator=(const Point2D &other)
{
  x = other.x;
  y = other.y;
  return (*this);
}

bool Point2D::operator ==(const Point2D &other)
{
  return (int)x == (int)other.x && (int)y == (int)other.y;
}
