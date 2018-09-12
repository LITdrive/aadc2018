#include "LITD_mapElementCurve.h"
#include "math_utilities.h"

LITD_mapElementCurve::LITD_mapElementCurve(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, LITD_mapElementCurve_corner_t curve_corner, double curve_radius) {
    valid=true;
    if(curve_corner<=CURVE_CORNER_INVAL || curve_corner>=CURVE_CORNER_MAX || curve_radius<=0.0 || curve_radius>=(fence_x_max-fence_x_min) || curve_radius>=(fence_y_max-fence_y_min)) {
        valid=false;
    }
    if(valid) {
        x_max=fence_x_max;
        x_min=fence_x_min;
        y_max=fence_y_max;
        y_min=fence_y_min;
        radius=curve_radius;
        corner=curve_corner;
        speed_advisory=radius*LITD_MAPELEMENTCURVE_SPEED_ADVISORY_MULTIPLIER;
        if(speed_advisory>1.0) {
            speed_advisory=1.0;
        }
        speed_limit=radius*LITD_MAPELEMENTCURVE_SPEED_ADVISORY_MULTIPLIER;
    }
}

LITD_virtualPoint LITD_mapElementCurve::getNormalPoint(LITD_virtualPoint &point) {
    double x=point.x,x0;
    double y=point.y,y0;
    double h=wrapTo2Pi<double>(point.h);

    double angle;

    switch(corner) {
        case CURVE_CORNER_LL:
            x0=x_min;
            y0=y_min;
            break;     
        case CURVE_CORNER_LR:
            x0=x_max;
            y0=y_min;
            break;

        case CURVE_CORNER_UL:
            x0=x_min;
            y0=y_max;
            break;

        case CURVE_CORNER_UR:
            x0=x_max;
            y0=y_max;
            break;
    }
    x-=x0;
    y-=y0;
    angle=atan2(y,x);

    double dx=radius*cos(angle);
    double dy=radius*sin(angle);

    if(angle<M_PI) {
        if(h<angle && h>(angle-M_PI)) {
            angle-=M_PI_2;
        } else {
            angle+=M_PI_2;
        }
    } else {
        if(h>angle && h<(angle+M_PI)) {
            angle+=M_PI_2;
        } else {
            angle-=M_PI_2;
        }
    }
    angle=wrapTo2Pi<double>(angle);

    

    return LITD_virtualPoint(x0+dx,y0+dy,0.0,angle);
}

double LITD_mapElementCurve::getPointOffset(LITD_virtualPoint &point) {
    double x=point.x;
    double y=point.y;
    double h=wrapTo2Pi<double>(point.h);
    return 0.0;
}

bool LITD_mapElementCurve::isInElement(LITD_virtualPoint &point) {
    double x=point.x;
    double y=point.y;
    //Returns false if: x is out of x range and y is out of y range.
    return x<=x_max && x>=x_min && y<=y_max && y>=y_min;
}

aadc::jury::maneuver LITD_mapElementCurve::selectDriveManeuver(aadc::jury::maneuver maneuver) {
    /* Curves do technically make turns, but in this manner they do not have a selection, so the maneuver is straight. */
    return aadc::jury::maneuver_straight;
}

double LITD_mapElementCurve::getSpeedAdvisory() {
    /* Curve speed advisory is dynamically calculated from the radius. */
    return speed_advisory;
}

double LITD_mapElementCurve::getSpeedLimit() {
    /* The curve speed limit is also calculated from the radius */
    return speed_limit;
}

bool LITD_mapElementCurve::isValid() {
    return valid;
}