#pragma once

#include <vector>
#include "Eigen/Dense"
#include "math_utilities.h"

class LITD_VirtualPoint
{
  public:
    LITD_VirtualPoint();
    LITD_VirtualPoint(double x, double y, double k, double h);
    Vector2d getVector2d() const;
    std::string toString() const;

    double x; //x - coordinate
    double y; //y - coordinate
    double k; //curvature
    double h; //heading
};

