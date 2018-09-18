#ifndef LITD_MAPELEMENTCURVE_H
#define LITD_MAPELEMENTCURVE_H

#include "LITD_MapElement.h"

/* Speed {advisory, limit} is calculated from the radius with a multiplier. */

/* Multiplies the radius with this value to get a relative speed advisory. This value is limited to 1.0 */
#define LITD_MAPELEMENTCURVE_SPEED_ADVISORY_MULTIPLIER -0.3
/* Multiplies the radius with this value to get a absolute speed limit */
#define LITD_MAPELEMENTCURVE_SPEED_LIMIT_MULTIPLIER 1.0

typedef enum { CURVE_CORNER_INVAL, CURVE_CORNER_LL, CURVE_CORNER_LR, CURVE_CORNER_UL, CURVE_CORNER_UR, CURVE_CORNER_MAX } LITD_mapElementCurve_corner_t; //{L(ower),U(upper)},{L(eft),R(ight)} is the curve corner/circle-center

class LITD_MapElementCurve : public LITD_MapElement
{
public:
    /* Contructor with parameters for the element-fence, the street coordinate and an boolean value (false=road is in x-direction, y stays constant, true=road in y-direction, x stays const.) */
    LITD_MapElementCurve(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, LITD_mapElementCurve_corner_t curve_corner, double curve_radius);
    //virtual LITD_VirtualPoint getVirtualPointAtDistance(double distance);
    //virtual bool isDistanceInElement(double distance);
    virtual LITD_VirtualPoint getNormalPoint(LITD_VirtualPoint &point);
    virtual double getPointOffset(LITD_VirtualPoint &point);
    virtual bool isInElement(LITD_VirtualPoint &point);
    virtual aadc::jury::maneuver selectDriveManeuver(aadc::jury::maneuver maneuver);
    virtual double getSpeedAdvisory();
    virtual double getSpeedLimit();
    virtual bool isValid();
protected:
    bool valid;
    LITD_mapElementCurve_corner_t corner;
    double x_min, x_max, y_min, y_max, radius;
    double speed_advisory, speed_limit;
};

#endif // LITD_MAPELEMENTCURVE_H
