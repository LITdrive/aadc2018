#include "LITD_Map.h"


string LITD_Map::strerr(LITD_map_error_t err) {
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

LITD_Map::LITD_Map() {
    map_index_current=-1;
    state=MAP_ENOERR;
    man_set=aadc::jury::maneuver::maneuver_straight;
    man_cur=aadc::jury::maneuver::manuever_undefined;
}

LITD_map_error_t LITD_Map::getMapState() {
    return state;
}

LITD_VirtualPoint LITD_Map::getNormalPoint(LITD_VirtualPoint &point) {
    //Check if we do have a point and if this point is valid.
    if(map_index_current<0 || map_index_current>=(int64_t)map_elements.size() || !map_elements[map_index_current]->isInElement(point)) {
        findElementIndex(point);
    }
    if(map_index_current>=0) {
        return map_elements[map_index_current]->getNormalPoint(point);
    } else {
        return LITD_VirtualPoint(point.x, point.y, point.k, point.h);
    }
}

void LITD_Map::selectNextManeuver(aadc::jury::maneuver maneuver) {
    man_set=maneuver;
}

aadc::jury::maneuver LITD_Map::getCurrentManeuver() {
    return man_cur;
}

double LITD_Map::getSpeedAdvisory() {
    if(map_index_current>=0 && map_index_current<(int64_t)map_elements.size()) {
        return map_elements[map_index_current]->getSpeedAdvisory();
    }
    return 0.0;
}

double LITD_Map::getSpeedLimit() {
    if(map_index_current>=0 && map_index_current<(int64_t)map_elements.size()) {
        return map_elements[map_index_current]->getSpeedLimit();
    }
    return 0.0;
}

LITD_map_error_t LITD_Map::addDoubleStraightElement(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, double straight_cord_1, double straight_cord_2, bool straight_is_y) {
    LITD_MapElementDoubleStraight *elem = new LITD_MapElementDoubleStraight(fence_x_min, fence_x_max, fence_y_min, fence_y_max, straight_cord_1, straight_cord_2, straight_is_y);
    if(elem->isValid()) {
        map_elements.push_back(elem);
        return MAP_ENOERR;
    }
    return MAP_EINVAL;
}

LITD_map_error_t LITD_Map::addStraightElement(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, double straight_cord, bool straight_is_y) {
    LITD_MapElementStraight *elem = new LITD_MapElementStraight(fence_x_min, fence_x_max, fence_y_min, fence_y_max, straight_cord, straight_is_y);
    if(elem->isValid()) {
        map_elements.push_back(elem);
        return MAP_ENOERR;
    }
    return MAP_EINVAL;
}

LITD_map_error_t LITD_Map::addCurveElement(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, LITD_mapElementCurve_corner_t curve_corner, double curve_radius) {
    LITD_MapElementCurve *elem = new LITD_MapElementCurve(fence_x_min, fence_x_max, fence_y_min, fence_y_max, curve_corner, curve_radius);
    if(elem->isValid()) {
        map_elements.push_back(elem); 
        return MAP_ENOERR;
    }
    return MAP_EINVAL;
}

LITD_map_error_t LITD_Map::addDoubleCurveElement(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, LITD_mapElementCurve_corner_t curve_corner, double curve_radius_1, double curve_radius_2) {
    LITD_MapElementDoubleCurve *elem = new LITD_MapElementDoubleCurve(fence_x_min, fence_x_max, fence_y_min, fence_y_max, curve_corner, curve_radius_1, curve_radius_2);
    if(elem->isValid()) {
        map_elements.push_back(elem); 
        return MAP_ENOERR;
    }
    return MAP_EINVAL;
}

LITD_map_error_t LITD_Map::addCrossingElement(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, bool road_beyond, bool road_left, bool road_right, bool road_above) {
    LITD_MapElementCrossing *elem = new LITD_MapElementCrossing(fence_x_min, fence_x_max, fence_y_min, fence_y_max, road_beyond, road_left, road_right, road_above);
    if(elem->isValid()) {
        map_elements.push_back(elem);
        return MAP_ENOERR;
    }
    return MAP_ENOERR;
}

void LITD_Map::findElementIndex(LITD_VirtualPoint &point) {
    std::size_t last=map_elements.size();
    map_index_current=-1;
    std::cout << "Searching for point x=" << point.x << " y= " << point.y << std::endl; 
    for(std::size_t i=0; i<last; i++) {
        if(map_elements[i]->isInElement(point)) {
            std::cout << "Found element with index " << i << std::endl;
            map_index_current=i;
            map_elements[i]->selectDriveLane(point);
            man_cur=map_elements[i]->selectDriveManeuver(man_set);
            return;
        }
    }
    man_cur=aadc::jury::maneuver::manuever_undefined;
    if(state==MAP_ENOERR) {
        state=MAP_ENOELEM;
    }
    std::cout << "Found no element!!" << std::endl;
    return;
}