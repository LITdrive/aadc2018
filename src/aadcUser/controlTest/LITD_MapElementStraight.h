#ifndef LITD_MAPELEMENTSTRAIGHT_H
#define LITD_MAPELEMENTSTRAIGHT_H

#include "LITD_mapElement.h"


class LITD_mapElementStraight : public LITD_mapElement
{
public:
    /* Contructor with parameters for the element-fence, the street coordinate and an boolean value (false=road is in x-direction, y stays constant, true=road in y-direction, x stays const.) */
    LITD_mapElementStraight(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, double straight_cord, bool straight_is_y);
    //virtual LITD_virtualPoint getVirtualPointAtDistance(double distance);
    //virtual bool isDistanceInElement(double distance);
    virtual LITD_virtualPoint getNormalPoint(LITD_virtualPoint &point);
    virtual double getPointOffset(LITD_virtualPoint &point);
    virtual bool isInElement(LITD_virtualPoint &point);
    virtual aadc::jury::maneuver selectDriveManeuver(aadc::jury::maneuver maneuver);
    virtual double getSpeedAdvisory();
    virtual double getSpeedLimit();
    virtual bool isValid();
protected:
    bool valid;
    bool is_y;
    double x_min, x_max, y_min, y_max, cord;
};

#endif // LITD_MAPELEMENTSTRAIGHT_H
