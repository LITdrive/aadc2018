#include "LITD_MapElementParkingSpot.h"

LITD_MapElementParkingSpot::LITD_MapElementParkingSpot(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, double G_x, double G_y) {
	valid = true;

	if (fence_x_max > fence_x_min && fence_y_max > fence_y_min) {
		if (targetPoint_x - PARKING_SPOT_WIDTH > fence_x_min || targetPoint_x + PARKING_SPOT_DEPTH < fence_x_max)
		{
			valid = false;
		}
		if (targetPoint_y - PARKING_SPOT_DEPTH < fence_y_min)
		{
			valid = false;
		}
	}

	else {
		valid = false;
	}

	if (valid) {
		x_max = fence_x_max;
		x_min = fence_x_min;
		y_max = fence_y_max;
		y_min = fence_y_min;
		targetPoint_x = G_x;
		targetPoint_y = G_y;
	}
}

aadc::jury::maneuver LITD_MapElementParkingSpot::selectDriveManeuver(aadc::jury::maneuver maneuver) {
	selectedManeuver = aadc::jury::maneuver_straight;

	if (aadc::jury::maneuverToString(maneuver).compare(aadc::jury::maneuverToString(aadc::jury::maneuver_cross_parking)))
	{
		selectedManeuver = aadc::jury::maneuver_cross_parking;
	}
	
	else if (aadc::jury::maneuverToString(maneuver).compare(aadc::jury::maneuverToString(aadc::jury::maneuver_pull_out_left)))
	{
		selectedManeuver = aadc::jury::maneuver_pull_out_left;
	}

	else if (aadc::jury::maneuverToString(maneuver).compare(aadc::jury::maneuverToString(aadc::jury::maneuver_pull_out_right)))
	{
		selectedManeuver =  aadc::jury::maneuver_pull_out_right;
	}

	return selectedManeuver;
}

double LITD_MapElementParkingSpot::getSpeedAdvisory() {
	double speedAdvisory = 1.0;

	if (aadc::jury::maneuverToString(selectedManeuver).compare(aadc::jury::maneuverToString(aadc::jury::maneuver_cross_parking)))
	{
		speedAdvisory = -1.0;
	}

	return speedAdvisory;
}

double LITD_MapElementParkingSpot::getSpeedLimit() {
	double speedLimit = 10.0;

	if (aadc::jury::maneuverToString(selectedManeuver).compare(aadc::jury::maneuverToString(aadc::jury::maneuver_cross_parking)))
	{
		speedAdvisory = -10.0;
	}

	return speedAdvisory;
}

bool LITD_MapElementParkingSpot::isValid() {
	return valid;
}

bool LITD_MapElementParkingSpot::isInElement(LITD_VirtualPoint &point) {
	double x = point.x;
	double y = point.y;
	//Returns false if: x is out of x range and y is out of y range.
	return x <= x_max && x >= x_min && y <= y_max && y >= y_min;
}

void LITD_MapElementParkingSpot::calcParkingTrajectory() 
{
	double s = OFFSET_MIN;
	double d_y = 1 / 100;
	double angle = 0;
	double d_angle = M_PI / 100;
	double c_2_y = targetPoint_y - PARKING_SPOT_DEPTH;
	double S_x = targetPoint_x;
	double S_y = targetPoint_y + c_2_y + s;
	double O_x = S_x - VEHICLE_TURNING_CYCLE;
	double O_y = S_y;
	LITD_VirtualPoint P = LITD_VirtualPoint(targetPoint_x, targetPoint_y, 0, M_PI / 2);
	trajectoryPoints.clear();
	trajectoryPoints.push_back(P);

	while (P.y > S_y)
	{
		P = LITD_VirtualPoint(targetPoint_x, targetPoint_y, 0, M_PI / 2);
		trajectoryPoints.push_back(P);
	}
	 
	while (angle > -M_PI / 2)
	{
		P = LITD_VirtualPoint(VEHICLE_TURNING_CYCLE*cos(angle) + O_x, VEHICLE_TURNING_CYCLE*sin(angle), 0, angle - M_PI / 2);
		trajectoryPoints.push_back(P);
		angle = angle - d_y;
	}
}

LITD_VirtualPoint LITD_MapElementParkingSpot::getNormalPoint(LITD_VirtualPoint &point) {
	double x = point.x;
	double y = point.y;
	

	if (aadc::jury::maneuverToString(selectedManeuver).compare(aadc::jury::maneuverToString(aadc::jury::maneuver_cross_parking)))
	{
		
	}
}