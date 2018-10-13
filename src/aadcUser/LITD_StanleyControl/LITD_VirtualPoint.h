#pragma once

#include <cmath>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

/*********************************************************************
 * Point2d
 *********************************************************************/
using Point2d = Eigen::Vector2d;

using Points2d = std::vector<Point2d, Eigen::aligned_allocator<Point2d>>;

/*********************************************************************
 * Vector2d
 *********************************************************************/
/// \brief 2D Vector
using Vector2d = Eigen::Vector2d;

/// \brief Vector of 2D Vectors
using Vectors2d = std::vector<Vector2d, Eigen::aligned_allocator<Vector2d>>;


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


