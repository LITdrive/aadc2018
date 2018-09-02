#pragma once

#include <vector>
#include "Eigen/Dense"
#include "Pose2d.h"

class SpeedGenerator
{
  public:
    SpeedGenerator();
    double getActualSpeed(double dtime);
    void   setSpeedLimit(double speedLimit);
    void   setSpeedLimitDistance(double speedLimit, double distance);
    void   setAcceleration(double accel);
    void   setDeceleration(double decel);

  private:
    double m_actualSpeed;
    double m_measuredSpeed;
    double m_speedLimit;
    std::pair<double,double> m_speedLimitDistance;
    double m_acceleration;
    double m_deceleration;
};

