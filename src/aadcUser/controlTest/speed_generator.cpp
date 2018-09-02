#include <math.h>
#include "speed_generator.h"
#include "constants.h"

SpeedGenerator::SpeedGenerator()
: m_speedLimit(MAX_SPEED_LIMIT),
  m_acceleration(MAX_ACCELERATION),
  m_deceleration(MAX_DECElERATION),
  m_actualSpeed(0.0),
  m_measuredSpeed(0.0)
{
}

double SpeedGenerator::getActualSpeed(double dtime)
{
  if(m_actualSpeed < m_speedLimit)
  {
    m_actualSpeed += m_acceleration * dtime;
  }
  else
  {
    m_actualSpeed = m_speedLimit;
  }
  //TODO Deceleration
//  else if(m_actualSpeed > m_speedLimit)
//  {
//    m_actualSpeed -= m_deceleration * dtime;
//  }

  return m_actualSpeed;
}

void SpeedGenerator::setSpeedLimit(double speedLimit)
{
  m_speedLimit = speedLimit;
//  m_speedLimitChanged = true;
}

void SpeedGenerator::setSpeedLimitDistance(double speedLimit, double distance)
{
  m_speedLimitDistance = std::pair<double,double>(speedLimit, distance);
}

void SpeedGenerator::setAcceleration(double accel)
{
  m_acceleration = accel;
}

void SpeedGenerator::setDeceleration(double decel)
{
  m_deceleration = decel;
}

