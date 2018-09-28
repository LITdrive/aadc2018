#define _USE_MATH_DEFINES
#define CAR_AXIS_DIST 0.36 // distance between front axle and rear axle in m
#define STANLEY_GAIN 1

#include "LITD_StanleyController.h"
#include <cmath>
#include <iostream>
#include "math_utilities.h"
#include "Eigen/Dense"

using namespace std;

LITD_StanleyController::LITD_StanleyController(){
    actPolyId = 0;
}

double LITD_StanleyController::calcSteeringAngle(LITD_VirtualPoint frontAxlePosition, LITD_VirtualPoint idealPoint, double carSpeed){
	double rad2degree = 180.0 / M_PI;
    //vector between car and virtualpoint
    Vector2d diff = idealPoint.getVector2d() - frontAxlePosition.getVector2d();

    //calc sign to steer in direction of road
    int sign = 1;
    double diff_heading_abs = wrapTo2Pi(atan2(diff(1), diff(0)));
    if(wrapTo2Pi(diff_heading_abs - wrapTo2Pi(idealPoint.h))> 4.712 ){
        sign = -1;
    }

    //calc normal distance of tangent to car (e)
    double e = (idealPoint.getVector2d() - frontAxlePosition.getVector2d()).norm() * sign;

    //calc angle between car heading and point tangent
    double theta_c =  wrapTo2Pi(idealPoint.h) - wrapTo2Pi(frontAxlePosition.h);

    //calc steering-angle with stanley-approach
    double carSteeringAngle = (theta_c + atan2(STANLEY_GAIN*e, carSpeed));

    if(carSteeringAngle < -M_PI/4){
        carSteeringAngle = -M_PI/4;
        std::cout << "Steering angle < -45° "  << std::endl;
    } else if(carSteeringAngle > M_PI/4){
        carSteeringAngle = M_PI/4;
        std::cout << "Steering angle > 45° "  << std::endl;
    }

    //Debug Messages
    std::cout << "----------forward-------------" << std::endl;
    std::cout << "point heading : " << idealPoint.h << "(" << rad2degree * idealPoint.h << "°)" << std::endl;
    std::cout << "car heading: " << frontAxlePosition.h << "(" << rad2degree * frontAxlePosition.h << "°)" << std::endl;
    std::cout << "diff heading: " << diff_heading_abs << "(" << rad2degree * diff_heading_abs << "°)" << std::endl;
    std::cout << "e: " << e << std::endl;
    std::cout << "Theta_C: " << theta_c << "(" << rad2degree * theta_c << "°)" << std::endl; 
    
    return carSteeringAngle;
}

void LITD_StanleyController::calculateFrontAxlePosition(LITD_VirtualPoint rearAxlePosition, LITD_VirtualPoint *frontAxlePosition){
    double dx = cos(rearAxlePosition.h)*CAR_AXIS_DIST;
    double dy = sin(rearAxlePosition.h)*CAR_AXIS_DIST;

	frontAxlePosition->x = rearAxlePosition.x + dx;
	frontAxlePosition->x = rearAxlePosition.y + dy;
	frontAxlePosition->x = rearAxlePosition.h;
}

void LITD_StanleyController::updatePolyList(poly_t polys[], uint8_t polyLen){
    //expect always "PLOYLIST_LEN" new polys in each updateStep
    if(polyLen > POLYLIST_LEN ||  polyLen < POLYLIST_LEN){
        std::cout << "Got wrong number of new polys " << std::endl;
        return;
    }

    //update polyList
    for(int i=0; i<polyLen; i++){
        polylist[i] = polys[i];
    }
}

void LITD_StanleyController::updateStep(poly_t polys[], uint8_t polyLen, LITD_VirtualPoint actPos){
	LITD_VirtualPoint frontAxlePosition;
	LITD_VirtualPoint idealPoint;
	polyPoint_t idealPolyPoint;
	double carSpeed = 0;

	// actPos = Position of the rear axle of the vehicle (reference point)
    updatePolyList(polys, polyLen);

	calculateFrontAxlePosition(actPos, &frontAxlePosition);

	getNextVirtualPointOnPoly(polys, polyLen, &idealPolyPoint, &idealPoint, frontAxlePosition);

    //calc Steeringangle
    steeringAngle = calcSteeringAngle(frontAxlePosition, idealPoint, carSpeed);
}

void LITD_StanleyController::getNextVirtualPointOnPoly(poly_t polys[], uint8_t polyLen, polyPoint_t* idealPolyPoint, LITD_VirtualPoint* idealPoint, LITD_VirtualPoint carPosition) {

	LITD_VirtualPoint actPoint;
	double min_dist = DBL_MAX;
	int min_poly_index = 0;
	double min_poly_p = 0;
	double min_dist_x = DBL_MAX;
	double min_dist_y = DBL_MAX;
	double min_dist_h = 0;

	for (int i = 0; i<polyLen; i++)
	{
		for (int j = 0; j <= ITERATION_PER_POLY; j++)
		{
			// p = [0, 1]
			double p = j / ITERATION_PER_POLY;
			calcVirtualPointfromPoly(polys[i], p, &actPoint);

			//calc norm to carPosition
			double dist = sqrt(pow(actPoint.x - carPosition.x, 2) + pow(actPoint.y - carPosition.y, 2));

			if (dist < min_dist)
			{
				min_dist = dist;
				min_poly_index = i;
				min_poly_p = p;
				min_dist_x = actPoint.x;
				min_dist_y = actPoint.y;
				min_dist_h = actPoint.h;
			}
		}
	}

	// Function value and ID of Poly with smallest distance to given car point
	idealPolyPoint->id = polys[min_poly_index].id;
	idealPolyPoint->p = min_poly_p;

	// Point on Poly with smallest distance to given car point
	idealPoint->x = min_dist_x;
	idealPoint->y = min_dist_y;
	idealPoint->h = min_dist_h;
}

void LITD_StanleyController::calcVirtualPointfromPoly(poly_t poly, double p, LITD_VirtualPoint* vp) {
	double x = poly.au * pow(p, 3) + poly.bu * pow(p, 2) + poly.cu * p + poly.du;
	double y = poly.av * pow(p, 3) + poly.bv * pow(p, 2) + poly.cv * p + poly.dv;

	//tangente durch erste ableitung berechnen
	double x_der = poly.au * pow(p, 2) + poly.bu * p + poly.cu;
	double y_der = poly.av * pow(p, 2) + poly.bv * p + poly.cv;

	double heading = wrapTo2Pi(atan2(y_der, x_der));

	vp->x = x;
	vp->y = y;
	vp->h = heading;
}


