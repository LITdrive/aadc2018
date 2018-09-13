#ifndef LITD_MAPELEMENTDOUBLECURVE_H
#define LITD_MAPELEMENTDOUBLECURVE_H

#include "LITD_MapElement.h"
#include "LITD_MapElementCurve.h"

/* Speed {advisory, limit} is calculated from the radius with a multiplier. */

/* Multiplies the radius with this value to get a relative speed advisory. This value is limited to 1.0 */
#define LITD_MAPELEMENTCURVE_SPEED_ADVISORY_MULTIPLIER 0.3
/* Multiplies the radius with this value to get a absolute speed limit */
#define LITD_MAPELEMENTCURVE_SPEED_LIMIT_MULTIPLIER 1.0


class LITD_MapElementDoubleCurve : public LITD_MapElement
{
public:
    /* Constructor for the Curve Element, with the fence, the corner-selection (as seen from "normal" view), and the two curve radiuses (radiusen, raden, radi?!?). The bigger and the smaller are automatically selected. */
    LITD_MapElementDoubleCurve(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, LITD_mapElementCurve_corner_t curve_corner, double curve_radius_1, double curve_radius_2);
    virtual LITD_VirtualPoint getNormalPoint(LITD_VirtualPoint &point);
    virtual bool isInElement(LITD_VirtualPoint &point);
    virtual aadc::jury::maneuver selectDriveManeuver(aadc::jury::maneuver maneuver);
    virtual bool selectDriveLane(LITD_VirtualPoint &point);
    virtual double getSpeedAdvisory();
    virtual double getSpeedLimit();
    virtual bool isValid();
protected:
    bool valid;
    LITD_mapElementCurve_corner_t corner;
    double x_min, x_max, y_min, y_max, radius_smaller, radius_bigger, radius_sel, angle_sel;
};

#endif // LITD_MAPELEMENTDOUBLECURVE_H
