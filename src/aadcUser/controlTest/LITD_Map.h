/*****************************************************************************************
 * LITD_Map / Virtual Point mover. 
 * This class is used to create a track for the car to follow.
 * 
 *****************************************************************************************/


#ifndef LITD_MAP_H
#define LITD_MAP_H

#include <vector>
#include <cstdint>
#include <string>
#include <iostream>

#include <aadc_jury.h>

#include "LITD_VirtualPoint.h"
#include "LITD_MapElement.h"
#include "LITD_MapElementStraight.h"
#include "LITD_MapElementCurve.h"

using namespace std;

typedef enum { MAP_ENOERR, MAP_EUNKOWN, MAP_EINVAL, MAP_ENOELEM } LITD_map_error_t;


class LITD_Map
{
public:

    static string strerr(LITD_map_error_t err);

    LITD_Map();
    /* Function to check if the map-generator is in an error-state. */
    LITD_map_error_t getMapState();
    /* This function returns the next point on the map. Always call getMapState to check if the returned Value is valid! */
    LITD_VirtualPoint getNormalPoint(LITD_VirtualPoint &point);
    /* Returns the distance in m, normally to the driving lane. Left is positive */
    double getPointOffset(LITD_VirtualPoint &point);
    /* Set the drive maneuver that should be driven as soon as possible. This value is NOT reset automatically. Defaults to STRAIGHT */
    void selectNextManeuver(aadc::jury::maneuver maneuver);
    /* Returns the drive maneuiver that is currently driven. */
    aadc::jury::maneuver getCurrentManeuver();
    /* Those functions return the Speed-advisory and limit for the current map-part */
    double getSpeedAdvisory();
    double getSpeedLimit();

    LITD_map_error_t addStraightElement(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, double straight_cord, bool straight_is_y);
    LITD_map_error_t addCurveElement(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, LITD_mapElementCurve_corner_t curve_corner, double curve_radius);

protected:
    LITD_map_error_t state;
    void findElementIndex(LITD_VirtualPoint &point);

    std::vector<LITD_MapElement*> map_elements;
    int64_t map_index_current;

    aadc::jury::maneuver man_set;
    aadc::jury::maneuver man_cur;

};

#endif // LITD_MAP_H
