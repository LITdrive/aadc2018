#pragma once

#include <vector>
#include "Eigen/Dense"
#include "Pose2d.h"
#include "virtual_point.h"

const double MAX_SPEED_LIMIT = 1.0;
const double MAX_SPEED_LIMIT_ARC = 1.0;

enum class MapElementType
{
    LINE,     // Straight line
    ARC       // arc as map element
};

enum class Orientation
{
  CW,         // Clockwise orientation
  CCW         // Counter clockwise (means mathematical positive orientation)
};

enum class DrivingDirection
{
    FORWARD,
    BACKWARD
};

class MapElement
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    MapElement(MapElementType type,
               const Point2d& startPoint,
              const Point2d& endPoint);
    MapElement(MapElementType type,
               Orientation orientation,
               const Point2d& startPoint,
               const Point2d& centerPoint,
              const Point2d& endPoint);
    MapElementType getType() const;
    Orientation getOrientation() const;
    VirtualPoint getVirtualPointAtDistance(double distance) const;
    VirtualPoint getVirtualPointAtEnd() const;
    Point2d getStartPoint() const;
    Point2d getCenterPoint() const;
    Point2d getEndPoint() const;
    double getRadius() const;
    double getLength() const;
    int getSegmentKey() const;

  private:
    static int m_segmentId;
    int m_segmentKey;
    const MapElementType m_type;
    Orientation m_orientation;
    const Point2d m_startPoint;
    const Point2d m_centerPoint;
    const Point2d m_endPoint;
    double m_length;
    double m_curvature;
    double m_radius;
    double m_speedLimit;
    DrivingDirection m_drivingDirection;

};

using MapElements = std::vector<MapElement,Eigen::aligned_allocator<MapElement>>;
