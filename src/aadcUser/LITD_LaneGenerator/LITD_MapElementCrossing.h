#ifndef LITD_MAPELEMENTCROSSING_H
#define LITD_MAPELEMENTCROSSING_H

#include "LITD_MapElement.h"
#include "LITD_MapElementCurve.h"

/******************************************************************************************************
 * Some notes on the Crossing Element: Index for the crossing site: 0=right, 1=above, 2=left, 2=beyond
 * Index for the quadrants: 0=ENE, 1=NEN, 2=NWN, 3=...., 7=ESE
 * 
 * 
 * 
 * 
 *******************************************************************************************************/


typedef enum {CROSSING_UNDEFINED, CROSSING_BEYOND, CROSSING_LEFT, CROSSING_RIGHT, CROSSING_ABOVE, CROSSING_MAX} LITD_CrossingPartType;

class LITD_MapElementCrossing : public LITD_MapElement
{
public:
    /* Contructor with parameters for the element-fence, the street coordinate and an boolean value (false=road is in x-direction, y stays constant, true=road in y-direction, x stays const.) */
    LITD_MapElementCrossing(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, bool road_beyond, bool road_left, bool road_right, bool road_above);
    virtual LITD_VirtualPoint getNormalPoint(LITD_VirtualPoint &point);
    virtual bool isInElement(LITD_VirtualPoint &point);
    virtual aadc::jury::maneuver selectDriveManeuver(aadc::jury::maneuver maneuver);
    virtual bool selectDriveLane(LITD_VirtualPoint &point);
    virtual double getSpeedAdvisory();
    virtual double getSpeedLimit();
    virtual bool isValid();
protected:

    //Constants
    static const uint8_t QUADRANT_SELECTION[8][2];
    static const uint8_t STRAIGHT_FOLLOWERS[4];
    static const uint8_t RIGHT_FOLLOWERS[4];
    static const uint8_t LEFT_FOLLOWERS[4];
    static const LITD_mapElementCurve_corner_t RIGHT_CORNERS[4];
    static const LITD_mapElementCurve_corner_t LEFT_CORNERS[4];

    static const double ROAD_OFFSET_LOWER;
    static const double ROAD_OFFSET_HIGHER;
    static const uint8_t CROSSING_NUM_ROADS=4;

    //Variables
    bool valid;
    double x_min, x_max, y_min, y_max;
    double k_xy, x_med_abs, y_med_abs, x_med, y_med;
    double rxl,rxh,ryl,ryh;
    aadc::jury::maneuver man_set;

    bool roads_en[CROSSING_NUM_ROADS];

    //If selected corner is inval, we drive straight
    LITD_mapElementCurve_corner_t corn_sel;
    //used for road coordinates on straights, used for curve radius when a curve.
    double rad_cord_sel;
    uint8_t from, to;

    void setRightTurn(LITD_mapElementCurve_corner_t corner, LITD_CrossingPartType _to);
    void setLeftTurn(LITD_mapElementCurve_corner_t corner, LITD_CrossingPartType _to);
    void setStraight(double straight_cord, LITD_CrossingPartType _to);
 
};

#endif // LITD_MAPELEMENTCROSSING_H
