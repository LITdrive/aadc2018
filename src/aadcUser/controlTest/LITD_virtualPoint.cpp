#include <math.h>
#include "LITD_virtualPoint.h"

LITD_virtualPoint::LITD_virtualPoint()
: LITD_virtualPoint(0.0, 0.0, 0.0, 0.0)
{
}

LITD_virtualPoint::LITD_virtualPoint(double x, double y, double k, double h)
: x(x),
  y(y),
  k(k),
  h(h)
{
}

Pose2d LITD_virtualPoint::getPose2d() const
{
  return Pose2d(x, y, h);
}

std::string LITD_virtualPoint::toString() const
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

