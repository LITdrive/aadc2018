#include "LITD_map.h"


string LITD_map::strerr(LITD_map_error_t err) {
    switch(err) {
        case MAP_ENOERR:
            return "No error";
        case MAP_EUNKOWN:
            return "Unkown error";
        case MAP_EINVAL:
            return "Invalid argument";
        case MAP_ENOELEM:
            return "No element";
        default:
            return "Unkown/unimplemented error";
    }
}

LITD_map::LITD_map() {
    map_index_current=-1;
    state=MAP_ENOERR;
    man_set=aadc::jury::maneuver::maneuver_straight;
    man_cur=aadc::jury::maneuver::manuever_undefined;
}

LITD_map_error_t LITD_map::getMapState() {
    return state;
}

LITD_virtualPoint LITD_map::getNormalPoint(LITD_virtualPoint &point) {
    //Check if we do have a point and if this point is valid.
    if(map_index_current<0 || map_index_current>=map_elements.size() || !map_elements[map_index_current]->isInElement(point)) {
        findElementIndex(point);
    }
    if(map_index_current>=0) {
        return map_elements[map_index_current]->getNormalPoint(point);
    } else {
        return LITD_virtualPoint(point.x, point.y, point.k, point.h);
    }
}

double LITD_map::getPointOffset(LITD_virtualPoint &point) {
    //Check if we do have a point and if this point is valid.
    if(map_index_current<0 || map_index_current>=map_elements.size() || !map_elements[map_index_current]->isInElement(point)) {
        findElementIndex(point);
    }
    if(map_index_current>=0) {
        return map_elements[map_index_current]->getPointOffset(point);
    } else {
        return 0.0;
    }
}

void LITD_map::selectNextManeuver(aadc::jury::maneuver maneuver) {
    man_set=maneuver;
}

aadc::jury::maneuver LITD_map::getCurrentManeuver() {
    return man_cur;
}

double LITD_map::getSpeedAdvisory() {
    if(map_index_current<0 || map_index_current>=map_elements.size()) {
        return map_elements[map_index_current]->getSpeedAdvisory();
    }
    return 0.0;
}

double LITD_map::getSpeedLimit() {
    if(map_index_current<0 || map_index_current>=map_elements.size()) {
        return map_elements[map_index_current]->getSpeedLimit();
    }
    return 0.0;
}

LITD_map_error_t LITD_map::addStraightElement(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, double straight_cord, bool straight_is_y) {
    LITD_mapElementStraight *elem = new LITD_mapElementStraight(fence_x_min, fence_x_max, fence_y_min, fence_y_max, straight_cord, straight_is_y);
    if(elem->isValid()) {
        map_elements.push_back(elem);
        return MAP_ENOERR;
    }
    return MAP_EINVAL;
}

LITD_map_error_t LITD_map::addCurveElement(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, LITD_mapElementCurve_corner_t curve_corner, double curve_radius) {
    LITD_mapElementCurve *elem = new LITD_mapElementCurve(fence_x_min, fence_x_max, fence_y_min, fence_y_max, curve_corner, curve_radius);
    if(elem->isValid()) {
        map_elements.push_back(elem); 
        return MAP_ENOERR;
    }
    return MAP_EINVAL;
}

std::size_t LITD_map::findElementIndex(LITD_virtualPoint &point) {
    std::size_t last=map_elements.size();
    map_index_current=-1;
    std::cout << "Searching for point x=" << point.x << " y= " << point.y << std::endl; 
    for(std::size_t i=0; i<last; i++) {
        if(map_elements[i]->isInElement(point)) {
            std::cout << "Found element with index " << i << std::endl;
            map_index_current=i;
            man_cur=map_elements[i]->selectDriveManeuver(man_set);
            return i;
        }
    }
    man_cur=aadc::jury::maneuver::manuever_undefined;
    if(state==MAP_ENOERR) {
        state=MAP_ENOELEM;
    }
    std::cout << "Found no element!!" << std::endl;
    return 0;
}