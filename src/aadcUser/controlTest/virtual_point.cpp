#include <math.h>
#include "virtual_point.h"

VirtualPoint::VirtualPoint()
: VirtualPoint(0.0, 0.0, 0.0, 0.0)
{
}

VirtualPoint::VirtualPoint(double x, double y, double k, double h)
: x(x),
  y(y),
  k(k),
  h(h)
{
}

Pose2d VirtualPoint::getPose2d() const
{
  return Pose2d(x, y, h);
}

std::string VirtualPoint::toString() const
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

