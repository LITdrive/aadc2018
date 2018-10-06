#pragma once

#define _USE_MATH_DEFINES

#include "LITD_MapElement.h"
#include "math_utilities.h"
#include <vector>
#include <iostream>

#define PARKING_SPOT_WIDTH 0.45 // m
#define PARKING_SPOT_DEPTH 0.85 // m
#define DRIVING_LANE_WIDTH 0.914 // m
#define VEHICLE_MAX_STEERING_ANGLE M_PI/4 // rad
#define VEHICLE_SPACE_BETWEEN_FRONT_AXLE_AND_BUMPER 0.12 // m
#define VEHICLE_SPACE_BETWEEN_REAR_AXLE_AND_BUMPER 0.108 // m
#define VEHICLE_SPACE_BETWEEN_FRONT_AND_REAR_AXLE 0.356 // m
#define VEHICLE_WIDTH 0.37 // m
#define VEHICLE_TURNING_CYCLE 1 / tan(VEHICLE_MAX_STEERING_ANGLE) // m
#define r_B2 sqrt(pow(VEHICLE_SPACE_BETWEEN_FRONT_AND_REAR_AXLE + VEHICLE_SPACE_BETWEEN_FRONT_AXLE_AND_BUMPER), 2) + pow(VEHICLE_TURNING_CYCLE + VEHICLE_WIDTH / 2, 2))
#define r_B4 sqrt(pow(VEHICLE_SPACE_BETWEEN_REAR_AXLE_AND_BUMPER, 2) + pow(VEHICLE_TURNING_CYCLE + VEHICLE_WIDTH / 2, 2))
#define OFFSET_MIN sqrt(pow(VEHICLE_TURNING_CYCLE - VEHICLE_WIDTH / 2, 2) - pow(VEHICLE_TURNING_CYCLE - PARKING_SPOT_WIDTH/2, 2))
#define OFFSET_MAX DRIVING_LANE_WIDTH - r_B2
#define OFFSET_STRAIGHT_PULL_OUT_LEFT 0.02 + 0.44 // m

using namespace std;

class LITD_MapElementParkingSpot : public LITD_MapElement
{
public:
	/* Contructor with parameters for the element-fence, the street coordinate and an boolean value (false=road is in x-direction, y stays constant, true=road in y-direction, x stays const.) */
	
	LITD_MapElementParkingSpot(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, double straight_cord, double G_x, double G_y);
	virtual LITD_VirtualPoint getNormalPoint(LITD_VirtualPoint &point);
	bool selectDriveLane(LITD_VirtualPoint & point);
	void printParkingTrajectory();
	void printPullOutLeftTrajectory();
	virtual bool isInElement(LITD_VirtualPoint &point);
	void calcParkingTrajectory();
	void calcPullOutLeftTrajectory();
	virtual aadc::jury::maneuver selectDriveManeuver(aadc::jury::maneuver maneuver);
	virtual double getSpeedAdvisory();
	virtual double getSpeedLimit();
	virtual bool isValid();
protected:
	bool valid;
	double x_min, x_max, y_min, y_max, targetPoint_x, targetPoint_y, angle_sel, cord;
	double speedAdvisory;
	aadc::jury::maneuver selectedManeuver;
	vector<LITD_VirtualPoint> trajectoryPoints;
	vector<LITD_VirtualPoint> trajectoryPointsPullOutLeft;
};