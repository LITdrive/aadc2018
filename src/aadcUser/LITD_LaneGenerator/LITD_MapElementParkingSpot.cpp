#include "LITD_MapElementParkingSpot.h"

LITD_MapElementParkingSpot::LITD_MapElementParkingSpot(double fence_x_min, double fence_x_max, double fence_y_min, double fence_y_max, double straight_cord, double G_x, double G_y) {
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

		if (straight_cord<fence_y_min || straight_cord>fence_y_max) {
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
		cord = straight_cord;
		targetPoint_x = G_x;
		targetPoint_y = G_y;
	}

	LITD_MapElementParkingSpot::calcParkingTrajectory();
	LITD_MapElementParkingSpot::calcPullOutLeftTrajectory();
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
		P = LITD_VirtualPoint(VEHICLE_TURNING_CYCLE*cos(angle) + O_x, VEHICLE_TURNING_CYCLE*sin(angle) + O_y, 0, M_PI / 2 + angle);
		trajectoryPoints.push_back(P);
		angle = angle - d_angle;
	}
}

void LITD_MapElementParkingSpot::calcPullOutLeftTrajectory()
{
	double s = OFFSET_MIN + OFFSET_STRAIGHT_PULL_OUT_LEFT;
	double d_y = 1 / 100;
	double angle = -M_PI;
	double d_angle = M_PI / 100;
	double c_2_y = targetPoint_y - PARKING_SPOT_DEPTH;
	double S_x = targetPoint_x;
	double S_y = targetPoint_y + c_2_y + s;
	double O_x = S_x + VEHICLE_TURNING_CYCLE;
	double O_y = S_y;
	LITD_VirtualPoint P = LITD_VirtualPoint(targetPoint_x, targetPoint_y, 0, 3 * M_PI / 2);
	trajectoryPointsPullOutLeft.clear();
	trajectoryPointsPullOutLeft.push_back(P);

	while (P.y > S_y)
	{
		P = LITD_VirtualPoint(targetPoint_x, targetPoint_y, 0, 3 * M_PI / 2);
		trajectoryPointsPullOutLeft.push_back(P);
	}

	while (angle < -M_PI / 2)
	{
		P = LITD_VirtualPoint(VEHICLE_TURNING_CYCLE*cos(angle) + O_x, VEHICLE_TURNING_CYCLE*sin(angle) + O_y, 0, M_PI / 2 - angle);
		trajectoryPointsPullOutLeft.push_back(P);
		angle = angle + d_angle;
	}
}

LITD_VirtualPoint LITD_MapElementParkingSpot::getNormalPoint(LITD_VirtualPoint &point) {
	double x = point.x;
	double y = point.y;
	double distance;
	double min_distance = DBL_MAX;
	int min_distance_index = 0;
	double P_min_distance_x;
	double P_min_distance_y;
	double P_min_distance_h;
	LITD_VirtualPoint P_min_distance = LITD_VirtualPoint(x, cord, 0.0, angle_sel);;

	if (aadc::jury::maneuverToString(selectedManeuver).compare(aadc::jury::maneuverToString(aadc::jury::maneuver_cross_parking)))
	{
		double index_last_element = trajectoryPoints.size() - 1;

		if (x > trajectoryPoints[index_last_element].x)
		{
			distance = sqrt(pow(x - trajectoryPoints[index_last_element].x, 2) + pow(y - trajectoryPoints[index_last_element].y, 2));
			min_distance = distance;
			min_distance_index = index_last_element;
			P_min_distance_x = trajectoryPoints[index_last_element].x;
			P_min_distance_y = trajectoryPoints[index_last_element].y;
			P_min_distance_h = trajectoryPoints[index_last_element].h;
		}

		else
		{
			for (std::vector<LITD_VirtualPoint>::size_type i = trajectoryPoints.size(); i >= 0; --i) {
				distance = sqrt(pow(x - trajectoryPoints[i].x, 2) + pow(y - trajectoryPoints[i].y, 2));

				if (abs(distance) < min_distance)
				{
					P_min_distance_x = trajectoryPoints[i].x;
					P_min_distance_y = trajectoryPoints[i].y;
					P_min_distance_h = trajectoryPoints[i].h;
					min_distance = distance;
					min_distance_index = i;
				}
			}
		}
	}

	else if (aadc::jury::maneuverToString(selectedManeuver).compare(aadc::jury::maneuverToString(aadc::jury::maneuver_pull_out_right)))
	{
		for (std::vector<LITD_VirtualPoint>::size_type i = 0; i < trajectoryPoints.size(); i++) {
			distance = sqrt(pow(x - trajectoryPoints[i].x, 2) + pow(y - trajectoryPoints[i].y, 2));

			if (abs(distance) < min_distance)
			{
				P_min_distance_x = trajectoryPoints[i].x;
				P_min_distance_y = trajectoryPoints[i].y;
				P_min_distance_h = trajectoryPoints[i].h + M_PI;
				min_distance = distance;
				min_distance_index = i;
			}
		}
	}

	else if (aadc::jury::maneuverToString(selectedManeuver).compare(aadc::jury::maneuverToString(aadc::jury::maneuver_pull_out_left)))
	{
		for (std::vector<LITD_VirtualPoint>::size_type i = 0; i < trajectoryPointsPullOutLeft.size(); i++) {
			distance = sqrt(pow(x - trajectoryPointsPullOutLeft[i].x, 2) + pow(y - trajectoryPointsPullOutLeft[i].y, 2));

			if (abs(distance) < min_distance)
			{
				P_min_distance_x = trajectoryPointsPullOutLeft[i].x;
				P_min_distance_y = trajectoryPointsPullOutLeft[i].y;
				P_min_distance_h = trajectoryPointsPullOutLeft[i].h;
				min_distance = distance;
				min_distance_index = i;
			}
		}
	}

	P_min_distance = LITD_VirtualPoint(P_min_distance_x, P_min_distance_y, 0, P_min_distance_h);
	return P_min_distance;
}

bool LITD_MapElementParkingSpot::selectDriveLane(LITD_VirtualPoint &point) {
	//This does nothing for a single lane element.
	double h = wrapTo2Pi<double>(point.h);

	if (aadc::jury::maneuverToString(selectedManeuver).compare(aadc::jury::maneuverToString(aadc::jury::maneuver_straight)))
	{
		if (h<M_PI / 2 || h>M_PI / 2 * 3) {
			angle_sel = 0.0;
		}
		else {
			angle_sel = M_PI;
		}

		return true;
	}

	else
	{
		return false;
	}
}