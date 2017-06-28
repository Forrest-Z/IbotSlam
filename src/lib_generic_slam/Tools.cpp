#include "Tools.h"

#include <ros/ros.h>

LiDAR_Scan_2D Tools::convertRosScan2GenericLiDARScan(const sensor_msgs::LaserScan::Ptr &scan)
{
  LiDAR_Scan_2D out(scan->ranges.size());
  unsigned int i, size = scan->ranges.size();
  double angle = scan->angle_min;

  for(i = 0; i < size; i++)
  {
    if(scan->ranges[i] == 0 || scan->ranges[i] >= INFINITY || scan->ranges[i] < 0.2f || scan->ranges[i] >= 100.0f)
      out.m_points[i] = Point2D();
    else  // Ajout du -X pour essayer d'inverser l'erreur de symétrie !!!
      out.m_points[i] = Point2D(-scan->ranges[i] * cos(angle), scan->ranges[i] * sin(angle));
    angle += scan->angle_increment;
  }

  return out;
}

void Tools::toEulerianAngle(const geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw)
{
  double ysqr = q.y * q.y;

  // roll (x-axis rotation)
  double t0 = +2.0 * (q.w * q.x + q.y * q.z);
  double t1 = +1.0 - 2.0 * (q.x * q.x + ysqr);
  roll = std::atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (q.w * q.y - q.z * q.x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  pitch = std::asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (q.w * q.z + q.x * q.y);
  double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
  yaw = std::atan2(t3, t4);
}

geometry_msgs::Quaternion Tools::toQuaternion(double pitch, double roll, double yaw)
{
  geometry_msgs::Quaternion q;
  double t0 = std::cos(yaw * 0.5);
  double t1 = std::sin(yaw * 0.5);
  double t2 = std::cos(roll * 0.5);
  double t3 = std::sin(roll * 0.5);
  double t4 = std::cos(pitch * 0.5);
  double t5 = std::sin(pitch * 0.5);

  q.w = t0 * t2 * t4 + t1 * t3 * t5;
  q.x=(t0 * t3 * t4 - t1 * t2 * t5);
  q.y=(t0 * t2 * t5 + t1 * t3 * t4);
  q.z=(t1 * t2 * t4 - t0 * t3 * t5);
  return q;
}

LiDAR_Scan_2D Tools::filterByQuaternion(LiDAR_Scan_2D &scan, const sensor_msgs::Imu &imu, double z_min, double z_max)
{
  unsigned int i = 0, size = scan.size();
  double x2d,y2d;
  double x,y,z;
  double t2,t3,t4,t5,t6,t7,t8,t9,t10;
  double a = imu.orientation.w,
         b = imu.orientation.x,
         c = imu.orientation.y,
         d = imu.orientation.z;

  for(i = 0; i < size; i++)
  {
    if(scan.m_points[i].x == 0 && scan.m_points[i].y == 0) continue;
    x2d = scan.m_points[i].x;
    y2d = scan.m_points[i].y;
    t2 =   a*b;
    t3 =   a*c;
    t4 =   a*d;
    t5 =  -b*b;
    t6 =   b*c;
    t7 =   b*d;
    t8 =  -c*c;
    t9 =   c*d;
    t10 = -d*d;
    x = 2*( (t8 + t10)*scan.m_points[i].x + (t6 -  t4)*scan.m_points[i].y + (t3 + t7)*0.8 ) + scan.m_points[i].x;
    y = 2*( (t4 +  t6)*scan.m_points[i].x + (t5 + t10)*scan.m_points[i].y + (t9 - t2)*0.8 ) + scan.m_points[i].y;
    z = 2*( (t7 -  t3)*scan.m_points[i].x + (t2 +  t9)*scan.m_points[i].y + (t5 + t8)*0.8 ) + 0.8;
    std::cout << " Z = " << z << std::endl;
    if( z > z_min && z < z_max){
      scan.m_points[i].x = x;
      scan.m_points[i].y = y;
    } else {
      scan.m_points[i].x = 0;
      scan.m_points[i].y = 0;
    }
  }

  return scan;
}

LiDAR_Scan_2D Tools::filterByDistance(LiDAR_Scan_2D &scan, double distM)
{
  double dx1, dy1, dist1, dx2, dy2, dist2, seuil = distM * distM;
  unsigned int i, size = scan.size() - 1;
  unsigned int filtred = 0;

  for(i = 1; i < size; i++)
  {
    if(scan.m_points[i].x == 0 && scan.m_points[i].y == 0 &&
       scan.m_points[i+1].x == 0 && scan.m_points[i+1].y == 0 &&
       scan.m_points[i-1].x == 0 && scan.m_points[i-1].y == 0) continue;
    dx1 = scan.m_points[i].x - scan.m_points[i-1].x;
    dy1 = scan.m_points[i].y - scan.m_points[i-1].y;
    dx2 = scan.m_points[i+1].x - scan.m_points[i].x;
    dy2 = scan.m_points[i+1].y - scan.m_points[i].y;
    dist1 = dx1 *dx1 + dy1 * dy1;
    dist2 = dx2 *dx2 + dy2 * dy2;
    if(dist1 > seuil && dist2 > seuil) {
      scan.m_points[i].x = 0;
      scan.m_points[i].y = 0;
      i += 1;
      filtred += 2;
    }
  }
  std::cout << "FILTRED by DISTANCE = " << filtred << std::endl;
  return scan;
}

LiDAR_Scan_2D Tools::approximateScan(LiDAR_Scan_2D &scan)
{
  double x, y;
  unsigned int i, size = scan.size() - 1;
  std::vector<Point2D>::iterator it;

  for(i = 0; i < size; i++)
  {
    if(scan.m_points[i].x == 0 && scan.m_points[i].y == 0 && scan.m_points[i+1].x == 0 && scan.m_points[i+1].y == 0) continue;
    x = scan.m_points[i].x + (scan.m_points[i+1].x - scan.m_points[i].x) / 2.0f;
    y = scan.m_points[i].y + (scan.m_points[i+1].y - scan.m_points[i].y) / 2.0f;
    it = scan.m_points.begin() + i + 1;
    scan.m_points.insert(it,Point2D(x, y));
    i+=1;
  }

  return scan;
}

