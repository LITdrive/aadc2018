#ifndef LITD_MAPELEMENT_H
#define LITD_MAPELEMENT_H

#include "Pose2d.h"
#include "LITD_VirtualPoint.h"
#include <aadc_jury.h>


/********************************************************/
/* Base (virtual interface) class for all Map Elements. */
/********************************************************/

class LITD_MapElement {
public:
    /* This method should return a point on its trajectory. If the distance is out of range
     * and not in the Element, the behaviour is undefined. Check this with isDistanceInElement */
    //virtual LITD_VirtualPoint getVirtualPointAtDistance(double distance) = 0;

    /* Returns true if the given distance is inside the element */
    //virtual bool isDistanceInElement(double distance) = 0;

    /* Returns the normal Point on the Element, undefined behaviour when the point is NOT in the element. */
    virtual LITD_VirtualPoint getNormalPoint(LITD_VirtualPoint &point) = 0;

    virtual double getPointOffset(LITD_VirtualPoint &point) = 0;

    /* Returns true if the given Point is still in this element */
    virtual bool isInElement(LITD_VirtualPoint &point) = 0;

    /* Sets the policy of the map element (left, straight, right, ...). Returns the acutally executed policy */
    virtual aadc::jury::maneuver selectDriveManeuver(aadc::jury::maneuver maneuver) = 0;

    /* Returns a relative Value (0.0-1.0) that advises the speed */
    virtual double getSpeedAdvisory() = 0;

    /* Returns a absolute value ([v] in m/s) that returns the maximum possible speed for this maneuver */
    virtual double getSpeedLimit() = 0;

    /* Returns true, if this road element has correct settings */
    virtual bool isValid() = 0;
};

#endif // LITD_MAPELEMENT_H
