#include <math.h>
#include "Pose2d.h"
#include "math_utilities.h"

Pose2d::Pose2d()
: Pose2d(0.0, 0.0, 0.0)
{
}

Pose2d::Pose2d(double x, double y, double angle)
: m_translation(Eigen::Vector2d(x,y)),
  m_rotation(angle)
{
}

double Pose2d::getX() const
{
  return m_translation.x();
}

double Pose2d::getY() const
{
  return m_translation.y();
}

double Pose2d::getAngle() const
{
  return m_rotation.angle();
}

Point2d Pose2d::getPoint2d() const
{
  return Point2d(getX(), getY());
}

double Pose2d::getLongitudinalDisplacement(const Pose2d& referencePose) const
{
  double displacement = 0.0;

  auto point = getPoint2d();
  auto referencePoint = referencePose.getPoint2d();

  if(point == referencePoint)
  {
    displacement = 0.0;
  }
  else
  {
    double length = (referencePoint - point).norm();
    displacement = sin(getAngle() - referencePose.getAngle()) * length;
    if(atan2(referencePoint(1) - point(1), referencePoint(0) - point(0)) > M_PI)
    {
      displacement *= -1.0;
    }
  }
  return displacement;
}

double Pose2d::getLateralDisplacement(const Pose2d& referencePose) const
{
  double displacement = 0.0;

  auto point = getPoint2d();
  auto referencePoint = referencePose.getPoint2d();

  if(point == referencePoint)
  {
    displacement = 0.0;
  }
  else
  {
    double length = (referencePoint - point).norm();
    displacement = cos(getAngle() - referencePose.getAngle()) * length;
    if(atan2(referencePoint(1) - point(1), referencePoint(0) - point(0)) > M_PI)
    {
      displacement *= -1.0;
    }
  }
  return displacement;
}
double Pose2d::getAngleDisplacement(const Pose2d& referencePose) const
{
  return wrapToPi(getAngle() - referencePose.getAngle());
}

Eigen::Matrix2d Pose2d::getRotationAsMatrix() const
{
  return m_rotation.toRotationMatrix();
}

void Pose2d::setRotationFromMatrix(const Eigen::Matrix2d& rotationMatrix)
{
  m_rotation.fromRotationMatrix(rotationMatrix);
}

bool Pose2d::operator==(const Pose2d& other) const
{
  if(fabs(getX() - other.getX()) > 0.001)
    return false;
  if(fabs(getY() - other.getY()) > 0.001)
    return false;
  if(fabs(getAngle() - other.getAngle()) > 0.001)
    return false;
  return true;
}

Pose2d Pose2d::operator+(const Pose2d& other) const
{
  return Pose2d(getX() + other.getX() * cos(getAngle()) - other.getY() * sin(getAngle()),
                getY() + other.getX() * sin(getAngle()) + other.getY() * cos(getAngle()),
                getAngle() + other.getAngle());
}
Pose2d& Pose2d::operator+=(const Pose2d& other)
{
  // Use temporary variables for the cases (A==this) or (B==this)
  const double new_x = getX() + other.getX() * cos(getAngle()) - other.getY() * sin(getAngle());
  const double new_y = getY() + other.getX() * sin(getAngle()) + other.getY() * cos(getAngle());
  m_translation.x() = new_x;
  m_translation.y() = new_y;

  m_rotation.angle() = wrapToPi<double>(getAngle() + other.getAngle());
  return *this;
}

std::string Pose2d::toString() const
{
  std::stringstream ss;
  ss << getX();
  ss << ", ";
  ss << getY();
  ss << " - ";
  ss << getAngle() / M_PI * 180.0;
  return ss.str();
}


