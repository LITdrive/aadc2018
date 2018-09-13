#include "LITD_MapElementCurve.h"
#include "math_utilities.h"

#include <iostream>

LITD_MapElementCurve::LITD_MapElementCurve(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, LITD_mapElementCurve_corner_t curve_corner, double curve_radius) {
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

LITD_VirtualPoint LITD_MapElementCurve::getNormalPoint(LITD_VirtualPoint &point) {
    double x=point.x,x0=0.0;
    double y=point.y,y0=0.0;

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
        case CURVE_CORNER_INVAL:
        case CURVE_CORNER_MAX:
        default:
            std::cout << "WARNING: curve corner is invalid!!" << std::endl;
            break;
    }
    x-=x0;
    y-=y0;
    angle=wrapTo2Pi<double>(atan2(y,x));

    std::cout << "curve angle is " << angle << " dx=" << x << " dy=" << y << std::endl;

    double dx=radius*cos(angle);
    double dy=radius*sin(angle);
    angle+=angle_sel;
    std::cout << "Angle before wrapping is " << angle << std::endl;
    angle=wrapTo2Pi<double>(angle);
    return LITD_VirtualPoint(x0+dx,y0+dy,0.0,angle);
}

bool LITD_MapElementCurve::isInElement(LITD_VirtualPoint &point) {
    double x=point.x;
    double y=point.y;
    //Returns false if: x is out of x range and y is out of y range.
    return x<=x_max && x>=x_min && y<=y_max && y>=y_min;
}

aadc::jury::maneuver LITD_MapElementCurve::selectDriveManeuver(aadc::jury::maneuver maneuver) {
    /* Curves do technically make turns, but in this manner they do not have a selection, so the maneuver is straight. */
    return aadc::jury::maneuver_straight;
}

bool LITD_MapElementCurve::selectDriveLane(LITD_VirtualPoint &point) {
    double x=point.x,x0=0.0;
    double y=point.y,y0=0.0;
    double h=wrapTo2Pi<double>(point.h);

    double angle;

    std::cout << "DoubleCurve selecting lane: ";

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
        case CURVE_CORNER_INVAL:
        case CURVE_CORNER_MAX:
        default:
            std::cout << "WARNING: curve corner is invalid!!" << std::endl;
            break;
    }
    x-=x0;
    y-=y0;
    angle=wrapTo2Pi<double>(atan2(y,x));

    std::cout << "curve angle is " << angle << " dx=" << x << " dy=" << y;

    if(angle<M_PI) {
        if(h<angle || h>(angle+M_PI)) {
            angle_sel=-M_PI_2;
        } else {
            angle_sel=M_PI_2;
        }
    } else {
        if(h>=angle || h<(angle-M_PI)) {
            angle_sel=M_PI_2;
        } else {
            angle_sel=-M_PI_2;
        }
    }
    std::cout <<"; selected angle: " << angle_sel << std::endl;
    return true;
}

double LITD_MapElementCurve::getSpeedAdvisory() {
    /* Curve speed advisory is dynamically calculated from the radius. */
    return speed_advisory;
}

double LITD_MapElementCurve::getSpeedLimit() {
    /* The curve speed limit is also calculated from the radius */
    return speed_limit;
}

bool LITD_MapElementCurve::isValid() {
    return valid;
}