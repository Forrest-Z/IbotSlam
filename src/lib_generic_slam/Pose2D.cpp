#include "Pose2D.h"

Pose2D::Pose2D(const double x, const double y, const double t)
 : x(x), y(y), theta(t)
{

}

Pose2D::Pose2D(const Pose2D &other)
  : x(other.x), y(other.y), theta(other.theta)
{

}

Pose2D &Pose2D::operator=(const Pose2D &other)
{
  x = other.x;
  y = other.y;
  theta = other.theta;
  return (*this);
}

geometry_msgs::Pose2D Pose2D::toRosPose() const
{
  geometry_msgs::Pose2D pose;
  pose.x = x;
  pose.y = y;
  pose.theta = theta;
  return pose;
}
