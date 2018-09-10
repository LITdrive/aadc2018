#pragma once

#include <vector>
#include "Eigen/Eigen"
#include "Eigen/Dense"

using Point2d = Eigen::Vector2d;

using Points2d = std::vector<Point2d, Eigen::aligned_allocator<Point2d>>;


class Pose2d
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Pose2d();
    Pose2d(double x, double y, double angle);

    double getX() const;
    double getY() const;
    double getAngle() const;
    Point2d getPoint2d() const;
    double getLongitudinalDisplacement(const Pose2d& referencePose) const;
    double getLateralDisplacement(const Pose2d& referencePose) const;
    double getAngleDisplacement(const Pose2d& referencePose) const;

    Eigen::Matrix2d getRotationAsMatrix() const;
    void setRotationFromMatrix(const Eigen::Matrix2d& rotationMatrix);

    bool operator==(const Pose2d& other) const;
    Pose2d operator+(const Pose2d& other) const ;
    Pose2d& operator+=(const Pose2d& other);
    std::string toString() const;

  private:
    Eigen::Translation2d m_translation;
    Eigen::Rotation2Dd m_rotation;

};

using Poses2d = std::vector<Pose2d,Eigen::aligned_allocator<Pose2d>>;
