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

LITD_VirtualPoint LITD_MapElementStraight::getNormalPoint(LITD_VirtualPoint &point) {
    double x=point.x;
    double y=point.y;
    if(is_y) {
        return LITD_VirtualPoint(cord, y, 0.0, angle_sel);
    } else {
        return LITD_VirtualPoint(x, cord, 0.0, angle_sel);
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

bool LITD_MapElementStraight::selectDriveLane(LITD_VirtualPoint &point) {
    //This does nothing for a single lane element.
    double h=wrapTo2Pi<double>(point.h);
    if(is_y) {
        if(h<M_PI) {
            angle_sel = M_PI/2.0;
        } else {
            angle_sel = 3.0*M_PI/2.0;
        }
    } else {
        if(h<M_PI/2 || h>M_PI/2*3) {
            angle_sel=0.0;
        } else {
            angle_sel=M_PI;
        }        
    }
    return true;
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