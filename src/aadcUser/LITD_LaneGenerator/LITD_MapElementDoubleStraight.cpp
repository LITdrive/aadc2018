#include <iostream>
#include "LITD_MapElementDoubleStraight.h"
#include "math_utilities.h"

LITD_MapElementDoubleStraight::LITD_MapElementDoubleStraight(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, double straight_cord_1, double straight_cord_2, bool straight_is_y) {
    valid=true;
    if(fence_x_max>fence_x_min && fence_y_max>fence_y_min) {
        if(straight_is_y) {
            if(straight_cord_1<fence_x_min || straight_cord_1>fence_x_max || straight_cord_2<fence_x_min || straight_cord_2>fence_x_max) {
                valid=false;
            }
        } else {
            if(straight_cord_1<fence_y_min || straight_cord_1>fence_y_max || straight_cord_2<fence_y_min || straight_cord_2>fence_y_max) {
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
        if(straight_cord_1<straight_cord_2) {
            cord_lower=straight_cord_1;
            cord_higher=straight_cord_2;
        } else {
            cord_lower=straight_cord_2;
            cord_higher=straight_cord_1;
        }
        is_y=straight_is_y;
    }
}

LITD_VirtualPoint LITD_MapElementDoubleStraight::getNormalPoint(LITD_VirtualPoint &point) {
    double x=point.x;
    double y=point.y;
    if(is_y) {
        return LITD_VirtualPoint(cord_sel, y, 0.0, angle_sel);
    } else {
        return LITD_VirtualPoint(x, cord_sel, 0.0, angle_sel);
    }
}

bool LITD_MapElementDoubleStraight::isInElement(LITD_VirtualPoint &point) {
    double x=point.x;
    double y=point.y;
    //Returns false if: x is out of x range and y is out of y range.
    return x<=x_max && x>=x_min && y<=y_max && y>=y_min;
}

aadc::jury::maneuver LITD_MapElementDoubleStraight::selectDriveManeuver(aadc::jury::maneuver maneuver) {
    return aadc::jury::maneuver_straight;
}

bool LITD_MapElementDoubleStraight::selectDriveLane(LITD_VirtualPoint &point) {
    double h=wrapTo2Pi<double>(point.h);
    std::cout << "Selecting double straight (h=" << h << ") :";
    if(is_y) {
        if(h<M_PI) {
            std::cout << "Y up higher" << std::endl;
            angle_sel = M_PI/2.0;
            cord_sel=cord_higher;
        } else {
            std::cout << "Y down lower" << std::endl;
            angle_sel = 3.0*M_PI/2.0;
            cord_sel=cord_lower;
        }
    } else {
        if(h<M_PI/2 || h>M_PI/2*3) {
            std::cout << "x up lower" << std::endl;
            angle_sel=0.0;
            cord_sel=cord_lower;
        } else {
            std::cout << "X down higher" << std::endl;
            angle_sel=M_PI;
            cord_sel=cord_higher;
        }        
    }
    return true;
}

double LITD_MapElementDoubleStraight::getSpeedAdvisory() {
    return 1.0;
}

double LITD_MapElementDoubleStraight::getSpeedLimit() {
    return 10.0;
}

bool LITD_MapElementDoubleStraight::isValid() {
    return valid;
}