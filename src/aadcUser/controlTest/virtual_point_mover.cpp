#include <math.h>
#include <iostream>
#include "virtual_point_mover.h"

VirtualPointMover::VirtualPointMover()
: m_actualSteerAngle(0.0),
  m_actualSpeed(0.0),
  m_distanceMoved(0.0),
  m_distanceOnElement(0.0),
  m_currentElementId(0),
  mp_mapElement(nullptr)
{

}

void VirtualPointMover::updateStep(double dtime)
{
  if(getActualMapElement())
  {
    m_speedGenerator.setSpeedLimit(mp_mapElement->getSpeedLimit());
    m_actualSpeed = m_speedGenerator.getActualSpeed(dtime);
    double distanceMove = m_actualSpeed * dtime;
    m_distanceMoved += distanceMove;

    while(m_distanceOnElement + distanceMove > mp_mapElement->getLength() && getActualMapElement())
    {
      distanceMove = distanceMove - (mp_mapElement->getLength() - m_distanceOnElement);
      m_distanceOnElement = 0.0;
      m_currentElementId++;
    }

    if(!getActualMapElement())
    {
      m_actualSteerAngle = 0.0;
      m_actualSpeed = 0.0;
    }
    else
    {
      m_distanceOnElement += distanceMove;
      m_actualVirtualPoint = mp_mapElement->getVirtualPointAtDistance(m_distanceOnElement);
      m_actualSteerAngle = atan(CAR_LENGTH * mp_mapElement->getCurvature());
    }
  }
  else
  {
    m_actualSteerAngle = 0.0;
    m_actualSpeed = 0.0;
  }
  std::cout << "[VirtualPointMover::updateStep] speed: " << m_actualSpeed
            << " delta: " << m_actualSteerAngle
            << " distance: " << m_distanceOnElement
            << " / " << mp_mapElement->getLength() << std::endl;
}

double VirtualPointMover::getSteerAngle() const
{
  return m_actualSteerAngle;
}

double VirtualPointMover::getSpeed() const
{
  return m_actualSpeed;
}

VirtualPoint VirtualPointMover::getVirtualPoint() const
{
  return m_actualVirtualPoint;
}

void VirtualPointMover::setActualMapElementId(int id)
{
  m_currentElementId = id;

}

void VirtualPointMover::setMapElements(const MapElements& elements)
{
  m_mapElements.clear();
  for(MapElement element : elements)
  {
    m_mapElements.push_back(element);
  }
}

bool VirtualPointMover::getActualMapElement()
{
  for(MapElement element : m_mapElements)
  {
    if(element.getSegmentKey() == m_currentElementId)
    {
      mp_mapElement = &element;
      return true;
    }
  }
  return false;
}



