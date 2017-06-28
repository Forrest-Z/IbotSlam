#ifndef POSE2D_H
#define POSE2D_H

#include <geometry_msgs/Pose2D.h>

class Pose2D
{
public:
  Pose2D(const double x=0.0f, const double y=0.0f, const double t=0.0f);
  Pose2D(const Pose2D& other);

  double x, y, theta;

  Pose2D& operator=(const Pose2D& other);

  geometry_msgs::Pose2D toRosPose() const;



};

#endif // POSE2D_H
