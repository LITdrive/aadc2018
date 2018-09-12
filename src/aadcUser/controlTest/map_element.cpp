#include <math.h>
#include <iostream>
#include "map_element.h"
#include "math_utilities.h"

int MapElement::m_segmentId = 0;

MapElement::MapElement(MapElementType type,
               const Point2d& startPoint,
               const Point2d& endPoint)
: MapElement(type, Orientation::CW, startPoint, Point2d(0.0, 0.0), endPoint)
{
}

MapElement::MapElement(MapElementType type,
               Orientation orientation,
               const Point2d& startPoint,
               const Point2d& centerPoint,
               const Point2d& endPoint)
: m_type(type),
  m_orientation(orientation),
  m_startPoint(startPoint),
  m_centerPoint(centerPoint),
  m_endPoint(endPoint)
{
  m_segmentKey = m_segmentId++;
  m_drivingDirection = DrivingDirection::FORWARD; //TODO
  if(m_type == MapElementType::LINE)
  {
    m_length = (m_endPoint - m_startPoint).norm();
    m_orientation = Orientation::CW; //only used for arcs
    m_curvature = 0.0;
    m_radius = 0.0;
    m_speedLimit = MAX_SPEED_LIMIT;
  }
  else if(m_type == MapElementType::ARC)
  {
    double radiusStart = (m_startPoint - m_centerPoint).norm();
    double radiusEnd = (m_endPoint - m_centerPoint).norm();
    double difference = fabs(radiusStart - radiusEnd);

    if(difference > 0.001)
    {
      throw std::runtime_error("[MapElement::MapElement] Radius changes over arc.");
    }
    m_radius = radiusStart;
    double startAngle = wrapTo2Pi(atan2(m_startPoint(1) - m_centerPoint(1), m_startPoint(0) - m_centerPoint(0)));
    double endAngle = wrapTo2Pi(atan2(m_endPoint(1) - m_centerPoint(1), m_endPoint(0) - m_centerPoint(0)));
    double centerAngle = 0.0;
    if(m_orientation == Orientation::CW)
    {
      centerAngle = wrapTo2Pi(startAngle - endAngle);
    }
    else
    {
      centerAngle = wrapTo2Pi(endAngle - startAngle);
    }
    m_length = centerAngle * m_radius;
    m_curvature = 1.0 / m_radius;
    m_speedLimit = MAX_SPEED_LIMIT_ARC;
  }
  std::cout << "[MapElement]: added Element " <<  (int) m_type
            << " radius: " << m_radius
            << " length: " << m_length
            << " orientation: " << (int) m_orientation
            << " start: " << m_startPoint
            << " end: " << m_endPoint << std::endl;

}
MapElementType MapElement::getType() const
{
  return m_type;
}

Orientation MapElement::getOrientation() const
{
  return m_orientation;
}
VirtualPoint MapElement::getVirtualPointAtDistance(double distance) const
{
  if(distance > m_length)
  {
    throw std::runtime_error("[MapElement::getVirtualPointAtDistance] Distance is longer than map element length.");
  }
  else if(distance < 0.0)
  {
    throw std::runtime_error("[MapElement::getVirtualPointAtDistance] Distance is less than zero.");
  }

  double x = 0.0; //x-coordinate
  double y = 0.0; //y-coordinate
  double k = 0.0; //curvature
  double h = 0.0; //heading


  if(m_type == MapElementType::LINE)
  {
    Point2d startPoint = m_startPoint;
    Point2d endPoint = m_endPoint;
    double xs = startPoint(0);
    double xe = endPoint(0);
    x = xe - xs;
    y = endPoint(1) - startPoint(1);
    x /= m_length;
    y /= m_length;

//    Point2d test = endPoint - startPoint;
//    Vector2d direction = test;
//    Vector2d directionUnitTangent = test / m_length;
    Vector2d directionUnitTangent(x, y);
    Point2d pointAtDistance = m_startPoint + (directionUnitTangent * distance);
    double angle = atan2(y, x);
    Pose2d pose2dAtDistance(pointAtDistance(0), pointAtDistance(1), angle);

    x = pose2dAtDistance.getX();
    y = pose2dAtDistance.getY();
    h = wrapToPi<double>(pose2dAtDistance.getAngle());
    k = 0;
  }
  else if(m_type == MapElementType::ARC)
  {
    double startAngle = wrapTo2Pi(atan2(m_startPoint(1) - m_centerPoint(1), m_startPoint(0) - m_centerPoint(0)));
    /*std::cout << "Start angle before wrapto2pi: "<< atan2(m_startPoint(1) - m_centerPoint(1), m_startPoint(0) - m_centerPoint(0))  << std::endl;
    std::cout << "Start angle: "<< startAngle  << std::endl;
    std::cout << "startpoint y: "<< m_startPoint(1)  << std::endl;
    std::cout << "centerpiont y: "<< m_centerPoint(1)  << std::endl;
    std::cout << "startpoint x: "<< m_startPoint(0)  << std::endl;
    std::cout << "centerpiont x: "<< m_centerPoint(0)  << std::endl;*/
    double angle = startAngle;
    double dAngle = distance / m_radius;
    if(m_orientation == Orientation::CW)
    {
      angle -= dAngle;
    }
    else
    {
      angle += dAngle;
    }

    angle = wrapTo2Pi(angle);

    double dx = m_radius * cos(angle);
    double dy = m_radius * sin(angle);
    std::cout << "dx: "<< dx  << std::endl;
    std::cout << "dy: "<< dy  << std::endl;
    double angleOfTangent = startAngle;
    double angleOfStartTangent = startAngle;
    if(m_orientation == Orientation::CW)
    {
      angleOfTangent -= dAngle;
      angleOfTangent -= M_PI / 2.0;
      angleOfStartTangent -= M_PI / 2.0;
    }
    else
    {
      angleOfTangent += dAngle;
      angleOfTangent += M_PI / 2.0;
      angleOfStartTangent += M_PI / 2.0;
    }
    angleOfStartTangent = wrapToPi<double>(angleOfStartTangent);
    Point2d pointAtDistance = Point2d(m_centerPoint(0) + dx, m_centerPoint(1) + dy);
    Pose2d pose2dAtDistance(pointAtDistance(0), pointAtDistance(1), wrapToPi<double>(angleOfTangent));

    x = pose2dAtDistance.getX();
    y = pose2dAtDistance.getY();
    h = wrapToPi<double>(angleOfTangent);
    k = m_curvature;




  }
  VirtualPoint vp(x,y,k,h);
  return vp;
}

VirtualPoint MapElement::getVirtualPointAtEnd() const
{
  return getVirtualPointAtDistance(m_length);
}

Point2d MapElement::getStartPoint() const
{
  return m_startPoint;
}

Point2d MapElement::getCenterPoint() const
{
  return m_centerPoint;
}


Point2d MapElement::getEndPoint() const
{
  return m_endPoint;
}

double MapElement::getRadius() const
{
  return m_radius;
}

double MapElement::getCurvature() const
{
  return m_curvature;
}

double MapElement::getLength() const
{
  return m_length;
}

int MapElement::getSegmentKey() const
{
  return m_segmentKey;
}

double MapElement::getSpeedLimit() const
{
  return m_speedLimit;
}
