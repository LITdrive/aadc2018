#include <math.h>
#include "LITD_VirtualPoint.h"

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

Pose2d LITD_VirtualPoint::getPose2d() const
{
  return Pose2d(x, y, h);
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

