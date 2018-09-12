#pragma once

#include <vector>
#include "Eigen/Dense"
#include "Pose2d.h"

class VirtualPoint
{
  public:
    VirtualPoint();
    VirtualPoint(double x, double y, double k, double h);
    Pose2d getPose2d() const;
    Vector2d getVector2d() const;
    std::string toString() const;

    double x; //x - coordinate
    double y; //y - coordinate
    double k; //curvature
    double h; //heading
};

