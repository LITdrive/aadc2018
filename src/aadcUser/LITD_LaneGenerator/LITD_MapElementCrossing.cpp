#include <iostream>
#include "LITD_MapElementCrossing.h"
#include "math_utilities.h"

const uint8_t LITD_MapElementCrossing::QUADRANT_SELECTION[][2]={{0,1},{1,0},{1,2},{2,1},{2,3},{3,2},{3,0},{0,3}};
const uint8_t LITD_MapElementCrossing::STRAIGHT_FOLLOWERS[]={2,3,0,1};
const uint8_t LITD_MapElementCrossing::RIGHT_FOLLOWERS[]={1,2,3,0};
const uint8_t LITD_MapElementCrossing::LEFT_FOLLOWERS[]={3,0,1,2};
const LITD_mapElementCurve_corner_t LITD_MapElementCrossing::RIGHT_CORNERS[]={CURVE_CORNER_UR, CURVE_CORNER_UL, CURVE_CORNER_LL, CURVE_CORNER_LR};
const LITD_mapElementCurve_corner_t LITD_MapElementCrossing::LEFT_CORNERS[]={CURVE_CORNER_LR, CURVE_CORNER_UR, CURVE_CORNER_UL, CURVE_CORNER_LL};

const double LITD_MapElementCrossing::ROAD_OFFSET_LOWER=0.25;
const double LITD_MapElementCrossing::ROAD_OFFSET_HIGHER=0.75;
const uint8_t LITD_MapElementCrossing::CROSSING_NUM_ROADS;


LITD_MapElementCrossing::LITD_MapElementCrossing(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, bool road_beyond, bool road_left, bool road_right, bool road_above) {
    valid=true;
    uint32_t n_roads=0;
    if(fence_x_max<=fence_x_min || fence_y_max<=fence_y_min) {
        valid=false;
    }
    if(road_beyond) {
        n_roads++;
    }
    roads_en[3]=road_beyond;
    if(road_left) {
        n_roads++;
    }
    roads_en[2]=road_left;
    if(road_right) {
        n_roads++;
    }
    roads_en[0]=road_right;
    if(road_above) {
        n_roads++;
    }
    roads_en[1]=road_above;

    if(n_roads<3) {
        valid=false;
    }

    if(valid) {
        x_max=fence_x_max;
        x_min=fence_x_min;
        y_max=fence_y_max;
        y_min=fence_y_min;


        k_xy=(y_max-y_min)/((x_max-x_min));
        x_med=(x_max-x_min)/2;
        y_med=(y_max-y_min)/2;
        x_med_abs=x_med+x_min;
        y_med_abs=y_med+y_min;

        rxl=ROAD_OFFSET_LOWER  * (x_max-x_min);
        rxh=ROAD_OFFSET_HIGHER * (x_max-x_min);

        ryl=ROAD_OFFSET_LOWER  * (y_max-y_min);
        ryh=ROAD_OFFSET_HIGHER * (y_max-y_min);
    }
}

LITD_VirtualPoint LITD_MapElementCrossing::getNormalPoint(LITD_VirtualPoint &point) {
    double x=point.x,x0;
    double y=point.y,y0;
    double h=point.h;
    if(from<CROSSING_NUM_ROADS && to<CROSSING_NUM_ROADS) {
        if(corn_sel==CURVE_CORNER_INVAL) {
            switch(from) {
                case 0:
                    return LITD_VirtualPoint(x, rad_cord_sel+y_min, 0.0, M_PI);
                case 2:
                    return LITD_VirtualPoint(x, rad_cord_sel+y_min, 0.0, 0.0);
                case 1:
                    return LITD_VirtualPoint(rad_cord_sel+x_min, y, 0.0, 3.0*M_PI/2.0);
                case 3:
                    return LITD_VirtualPoint(rad_cord_sel+x_min, y, 0.0, M_PI/2.0);
                default:
                    valid=false;
                    return LITD_VirtualPoint(x,y,0.0,h);
            }
        } else {
            double angle;
            switch(corn_sel) {
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
            angle=wrapTo2Pi<double>(atan2(y,x));
            double dx=rad_cord_sel*cos(angle);
            double dy=rad_cord_sel*sin(angle);
            std::cout << "Selecting angle offset, from=" << (uint32_t)from << "; to=" << (uint32_t)to << "; right follower=" << (uint32_t)RIGHT_FOLLOWERS[from] << std::endl;
            if(RIGHT_FOLLOWERS[from]==to) {
                //Right turn, angle=angle-M_PI/2.0;
                angle-=M_PI/2.0;

            } else {
                //Left turn, angle=angle+M_PI/2.0;
                angle+=M_PI/2.0;
            }
            angle=wrapTo2Pi<double>(angle);
            return LITD_VirtualPoint(x0+dx,y0+dy,0.0,angle);
        }
    } else {
        valid=false;
        return LITD_VirtualPoint(x, y, 0.0, h);
    }
    

}

bool LITD_MapElementCrossing::isInElement(LITD_VirtualPoint &point) {
    double x=point.x;
    double y=point.y;
    //Returns false if: x is out of x range and y is out of y range.
    return x<=x_max && x>=x_min && y<=y_max && y>=y_min;
}

#define MANEUVER_SET_LEFT()  corn_sel=LEFT_CORNERS[from]; \
                                 to=out_road; \
                                 rad_cord_sel=rxh; \
                                 maneuver=aadc::jury::maneuver_left

#define MANEUVER_SET_RIGHT() corn_sel=RIGHT_CORNERS[from];\
                                 to=out_road; \
                                 rad_cord_sel=rxl;\
                                 maneuver=aadc::jury::maneuver_right

#define MANEUVER_SET_STRAIGHT() if(from==0 || from==3) { \
                                        rad_cord_sel=rxh; \
                                    } else { \
                                        rad_cord_sel=rxl; \
                                    } \
                                    to=out_road; \
                                    corn_sel=CURVE_CORNER_INVAL; \
                                    maneuver=aadc::jury::maneuver_straight

aadc::jury::maneuver LITD_MapElementCrossing::selectDriveManeuver(aadc::jury::maneuver maneuver) {
    if(maneuver!=aadc::jury::maneuver_straight && maneuver!=aadc::jury::maneuver_left && maneuver!=aadc::jury::maneuver_right) {
        maneuver=aadc::jury::maneuver_straight;
    }
    if(from>=CROSSING_NUM_ROADS) {
        return aadc::jury::manuever_undefined;
    }
    uint8_t out_road=UINT8_MAX;
    switch(maneuver) {
        case aadc::jury::maneuver_straight:
            std::cout << "Selecting straight: ";
            out_road=STRAIGHT_FOLLOWERS[from];
            if(!roads_en[out_road]) {
                out_road=RIGHT_FOLLOWERS[from];
                if(!roads_en[out_road]) {
                    out_road=LEFT_FOLLOWERS[from];
                    if(!roads_en[out_road]) {
                        out_road=UINT8_MAX;
                        maneuver=maneuver=aadc::jury::manuever_undefined;
                    } else {
                        MANEUVER_SET_LEFT();
                        std::cout << "selected left" << std::endl;
                    }
                } else {
                    MANEUVER_SET_RIGHT();
                    std::cout << "selected right" << std::endl;
                }
            } else {
                MANEUVER_SET_STRAIGHT();
                std::cout << "selected straight" << std::endl;
            }
            break;
        case aadc::jury::maneuver_left:
            std::cout << "Selecting left: ";
            out_road=LEFT_FOLLOWERS[from];
            if(!roads_en[out_road]) {
                out_road=STRAIGHT_FOLLOWERS[from];
                if(!roads_en[out_road]) {
                    out_road=RIGHT_FOLLOWERS[from];
                    if(!roads_en[out_road]) {
                        out_road=UINT8_MAX;
                        maneuver=maneuver=aadc::jury::manuever_undefined;
                    } else {
                        MANEUVER_SET_RIGHT();
                        std::cout << "selected right" << std::endl;
                    }
                } else {
                    MANEUVER_SET_STRAIGHT();
                    std::cout << "selected straight" << std::endl;
                }
            } else {
                MANEUVER_SET_LEFT();
                std::cout << "selected left" << std::endl;
            }
            break;
        case aadc::jury::maneuver_right:
            std::cout << "Selecting right: ";
            out_road=RIGHT_FOLLOWERS[from];
            std::cout << "out_road is " << (uint32_t)out_road << " :";
            if(!roads_en[out_road]) {
                out_road=STRAIGHT_FOLLOWERS[from];
                if(!roads_en[out_road]) {
                    out_road=LEFT_FOLLOWERS[from];
                    if(!roads_en[out_road]) {
                        out_road=UINT8_MAX;
                        maneuver=maneuver=aadc::jury::manuever_undefined;
                    } else {
                        MANEUVER_SET_LEFT();
                        std::cout << "selected left" << std::endl;
                    }
                } else {
                    MANEUVER_SET_STRAIGHT();
                    std::cout << "selected straight" << std::endl;
                }
            } else {
                MANEUVER_SET_RIGHT();
                std::cout << "selected right" << std::endl;
            }
            break;
    }

    return maneuver;
}

bool LITD_MapElementCrossing::selectDriveLane(LITD_VirtualPoint &point) {
    //This does nothing for a single lane element.
    double x=point.x;
    double y=point.y;
    double h=wrapTo2Pi<double>(point.h);
    uint32_t in_quadrant=0;


    if(y<y_med_abs) {
        //Lower half
        if(x<x_med_abs) {
            //left quater
            if((y-y_min)<(k_xy*(x-x_min))) {
                //lower eighth, is beyond if beyond is set, is left if beyond is not set
                //std::cout << "lower left lower 6" << std::endl;
                in_quadrant=5;
            } else {
                //higher eighth
                //std::cout << "lower left higher 7" << std::endl;
                in_quadrant=4;
            }
        } else {
            //right quater
            if((y-y_min)<(k_xy*(x_max-x))) {
                //lower eighth
                //std::cout << "lower right lower 1" << std::endl;
                in_quadrant=6;
            } else {
                //higher eighth
                //std::cout << "lower right higher 2" << std::endl;
                in_quadrant=7;
            }
        }
    } else {
        //Upper half
        if(x<x_med_abs) {
            //left quater
            if((y-y_med)<(k_xy*(x_med_abs-x))) {
                //lower eighth
                //std::cout << "upper left lower 6" << std::endl;
                in_quadrant=3;
            } else {
                //higher eighth
                //std::cout << "upper left higher 5" << std::endl;
                in_quadrant=2;
            }
        } else {
            //right quater
            if((y-y_med)<(k_xy*(x-x_med_abs))) {
                //lower eighth
                //std::cout << "upper right lower 3" << std::endl;
                in_quadrant=0;
            } else {
                //higher eighth
                //std::cout << "upper right higher 4" << std::endl;
                in_quadrant=1;
            }
        }       
    }
    from=UINT8_MAX;
    uint8_t in_road=QUADRANT_SELECTION[in_quadrant][0];
    if(!roads_en[in_road]) {
        in_road=QUADRANT_SELECTION[in_quadrant][1];
    }
    if(!roads_en[in_road]) {
        //in_road holds the ID of the 
        std::cout << "No input road selectable!" << std::endl;
        return false;
    }
    from=in_road;
    std::cout << "Input road " << (uint32_t)from << " from quadrant " << (uint32_t)in_quadrant << " selected!" << std::endl;
    return true;
}

double LITD_MapElementCrossing::getSpeedAdvisory() {
    return 1.0;
}

double LITD_MapElementCrossing::getSpeedLimit() {
    return 10.0;
}

bool LITD_MapElementCrossing::isValid() {
    return valid;
}