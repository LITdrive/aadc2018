#pragma once

#include <vector>
#include "Eigen/Dense"
#include "Pose2d.h"
#include "virtual_point.h"
#include "map_element.h"
#include "speed_generator.h"

class VirtualPointMover
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VirtualPointMover();

    void   updateStep(double dtime);
    double getSteerAngle() const;
    double getSpeed() const;
    void   setMapElements(const MapElements& elements);
    bool   getActualMapElement();
    void   setActualMapElementId(int id);
    VirtualPoint getVirtualPoint() const;

  private:
    double m_actualSpeed;
    double m_actualSteerAngle;
    double m_distanceMoved;
    double m_distanceOnElement;
    int    m_currentElementId;
    MapElement * mp_mapElement;
    VirtualPoint m_actualVirtualPoint;
    SpeedGenerator m_speedGenerator;
    MapElements m_mapElements;


};

