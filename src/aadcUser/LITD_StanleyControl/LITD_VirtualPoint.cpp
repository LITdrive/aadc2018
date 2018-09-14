#include <math.h>
#include "LITD_VirtualPoint.h"
#include "Eigen/Dense"

LITD_VirtualPoint::LITD_VirtualPoint()
: LITD_VirtualPoint(0.0, 0.0, 0.0, 0.0)
{
}

LITD_VirtualPoint::LITD_VirtualPoint(double x, double y, double k, double h)
: x(x),
  y(y),
  k(k),
  h(h)
{
}


Vector2d LITD_VirtualPoint::getVector2d() const
{
  return Vector2d(x,y);
}

std::string LITD_VirtualPoint::toString() const
{
  std::stringstream ss;
  ss << "[x ";
  ss << x;
  ss << " y ";
  ss << y;
  ss << " k ";
  ss << k;
  ss << " h ";
  ss << h;

  return ss.str();
}
