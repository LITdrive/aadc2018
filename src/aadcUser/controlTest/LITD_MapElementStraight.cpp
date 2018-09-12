#include "LITD_MapElementStraight.h"
#include "math_utilities.h"

LITD_MapElementStraight::LITD_MapElementStraight(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, double straight_cord, bool straight_is_y) {
    valid=true;
    if(fence_x_max>fence_x_min && fence_y_max>fence_y_min) {
        if(straight_is_y) {
            if(straight_cord<fence_x_min || straight_cord>fence_x_max) {
                valid=false;
            }
        } else {
            if(straight_cord<fence_y_min || straight_cord>fence_y_max) {
                valid=false;
            }
        }
    } else {
        valid=false;
    }
    if(valid) {
        x_max=fence_x_max;
        x_min=fence_x_min;
        y_max=fence_y_max;
        y_min=fence_y_min;
        cord=straight_cord;
        is_y=straight_is_y;
    }
}
/*
LITD_VirtualPoint LITD_MapElementStraight::getVirtualPointAtDistance(double distance) {

    return LITD_VirtualPoint(0.0,0.0,0.0,0.0);
}
*/
/*
bool LITD_MapElementStraight::isDistanceInElement(double distance) {
    return false;
}
*/
LITD_VirtualPoint LITD_MapElementStraight::getNormalPoint(LITD_VirtualPoint &point) {
    double x=point.x;
    double y=point.y;
    double h=wrapTo2Pi<double>(point.h);
    if(is_y) {
        if(h<M_PI) {
            return LITD_VirtualPoint(cord, y, 0.0, M_PI/2.0);
        } else {
            return LITD_VirtualPoint(cord, y, 0.0, 3.0*M_PI/2.0);
        }
    } else {
        if(h>3.0*M_PI/2.0 || h<M_PI/2.0) {
            return LITD_VirtualPoint(x, cord, 0.0, 0.0);
        } else {
            return LITD_VirtualPoint(x, cord, 0.0, M_PI);
        }
    }
}

double LITD_MapElementStraight::getPointOffset(LITD_VirtualPoint &point) {
    double x=point.x;
    double y=point.y;
    double h=wrapTo2Pi<double>(point.h);
    if(is_y) {
        if(h<M_PI) {
            return cord-x;
        } else {
            return x-cord;
        }
    } else {
        if(h<M_PI/2 || h>M_PI/2*3) {
            return y-cord;
        } else {
            return cord-y;
        }
    }
}

bool LITD_MapElementStraight::isInElement(LITD_VirtualPoint &point) {
    double x=point.x;
    double y=point.y;
    //Returns false if: x is out of x range and y is out of y range.
    return x<=x_max && x>=x_min && y<=y_max && y>=y_min;
}

aadc::jury::maneuver LITD_MapElementStraight::selectDriveManeuver(aadc::jury::maneuver maneuver) {
    return aadc::jury::maneuver_straight;
}

double LITD_MapElementStraight::getSpeedAdvisory() {
    return 1.0;
}

double LITD_MapElementStraight::getSpeedLimit() {
    return 10.0;
}

bool LITD_MapElementStraight::isValid() {
    return valid;
}