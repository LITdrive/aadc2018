#ifndef LITD_MAPELEMENTDOUBLESTRAIGHT_H
#define LITD_MAPELEMENTDOUBLESTRAIGHT_H

#include "LITD_MapElement.h"


class LITD_MapElementDoubleStraight : public LITD_MapElement
{
public:
    /* Contructor with parameters for the element-fence, the street coordinate and an boolean value (false=road is in x-direction, y stays constant, true=road in y-direction, x stays const.) */
    LITD_MapElementDoubleStraight(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, double straight_cord_right, double straight_cord_left, bool straight_is_y);
    //virtual LITD_VirtualPoint getVirtualPointAtDistance(double distance);
    //virtual bool isDistanceInElement(double distance);
    virtual LITD_VirtualPoint getNormalPoint(LITD_VirtualPoint &point);
    //virtual double getPointOffset(LITD_VirtualPoint &point);
    virtual bool isInElement(LITD_VirtualPoint &point);
    virtual aadc::jury::maneuver selectDriveManeuver(aadc::jury::maneuver maneuver);
    virtual bool selectDriveLane(LITD_VirtualPoint &point);
    virtual double getSpeedAdvisory();
    virtual double getSpeedLimit();
    virtual bool isValid();
protected:
    bool valid;
    bool is_y;
    double x_min, x_max, y_min, y_max, cord_lower, cord_higher, cord_sel, angle_sel;
};

#endif // LITD_MAPELEMENTDOUBLESTRAIGHT_H
