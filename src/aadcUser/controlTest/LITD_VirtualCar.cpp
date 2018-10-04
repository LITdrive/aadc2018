#include "LITD_VirtualCar.h"
#include "LITD_VirtualPoint.h"
#include "math_utilities.h"
#include "Eigen/Dense"


#include <stdio.h>
#include <unistd.h>
#include <iostream>

LITD_VirtualCar::LITD_VirtualCar()
{
    carPosition.x = 0.0;
    carPosition.y = -0.2;
    carPosition.h = 0.0;
    carSpeed = 0.0;

    calculateBackPos(); 

    stanleyGain = 1.5;
    vKp = 0.1;
    vKi =0.5;

    vpOld.x = carPosition.x;
    vpOld.y = carPosition.y;
    vIntegrate = 0.0;

    // Fixed Polynomials of a cirle arc and a straight for testing
	// Circle Arc
	// 0.3443 x + 0.3115 x - 1.656 x + 0.363
	trajectoryArray[1].start = 0;
	trajectoryArray[1].end = 1;
	trajectoryArray[1].ax = 0.702;
	trajectoryArray[1].bx = 1.656;
	trajectoryArray[1].cx = -0.3115;
	trajectoryArray[1].dx = 0.3443;
	// -0.3443 x + 1.344 x + 2.22e-16
	trajectoryArray[1].ay = 0;
	trajectoryArray[1].by = 0;
	trajectoryArray[1].cy = 1.344;
	trajectoryArray[1].dy = 0.3443;
	// straight
	// -0.339 x + 0.702
	trajectoryArray[0].start = 0;
	trajectoryArray[0].end = 1;
	trajectoryArray[0].ax = 0.363;
	trajectoryArray[0].bx =  0.339;
	trajectoryArray[0].cx = 0;
	trajectoryArray[0].dx = 0;
	trajectoryArray[0].ay = 0;
	trajectoryArray[0].by = 0;
	trajectoryArray[0].cy = 0;
	trajectoryArray[0].dy = 0;


}

void LITD_VirtualCar::calcVirtualPointfromPoly(tTrajectory poly, double p, LITD_VirtualPoint* vp) {
	double x = poly.dx*pow(p, 3) + poly.cx*pow(p, 2) + poly.bx*p + poly.ax;
	double y = poly.dy*pow(p, 3) + poly.cy*pow(p, 2) + poly.by*p + poly.ay;

	//tangente durch erste ableitung berechnen
	double x_der = 3 * poly.dx*pow(p, 2) + 2 * poly.cx*p + poly.bx;
	double y_der = 3 * poly.dy*pow(p, 2) + 2 * poly.cy*p + poly.by;
	

	

	double heading = wrapTo2Pi(atan2(y_der, x_der));
	//LOG_INFO("Derivation Points: x_der: %f, y_der: %f from h: %f", x_der, y_der, heading );
	//double heading = wrapTo2Pi(atan2(y_p-y_m, x_p-x_m));

	

	vp->x = x;
	vp->y = y;
	vp->h = heading;
}

void LITD_VirtualCar::getNextVirtualPointOnPoly() {

	LITD_VirtualPoint actualPoint;
	//min_dist_poly_index = 0;
    double min_dist = 1000000000;
	double min_poly_p = 0;
	poly_completed = false;


	for (int i = 0; i<TRAJECTORY_ARRAY_LEN; i++)
	{
		for (double j = trajectoryArray[i].start; j <= trajectoryArray[i].end; j+=(trajectoryArray[i].end-trajectoryArray[i].start)/POINTS_PER_POLY)
		{
			// p = [0, 1]
			calcVirtualPointfromPoly(trajectoryArray[i], j, &actualPoint);

			// Target Point on Polynom has to be in front of the actual position of the front axle
			//if ((vehicleActualFrontAxlePosition.h == 0.0  && actualPoint.x > vehicleActualFrontAxlePosition.x) || (vehicleActualFrontAxlePosition.h == M_PI && actualPoint.x < vehicleActualFrontAxlePosition.x) || (vehicleActualFrontAxlePosition.h == M_PI/2 && actualPoint.y > vehicleActualFrontAxlePosition.y) || (vehicleActualFrontAxlePosition.h == (3/2)*M_PI && actualPoint.y > vehicleActualFrontAxlePosition.y) || (vehicleActualFrontAxlePosition.h > 0 && vehicleActualFrontAxlePosition.h < M_PI/2 && actualPoint.x > vehicleActualFrontAxlePosition.x && actualPoint.y > vehicleActualFrontAxlePosition.y) || (vehicleActualFrontAxlePosition.h > M_PI/2 && vehicleActualFrontAxlePosition.h < M_PI && actualPoint.x < vehicleActualFrontAxlePosition.x && actualPoint.y > vehicleActualFrontAxlePosition.y) || (vehicleActualFrontAxlePosition.h > M_PI && vehicleActualFrontAxlePosition.h < (3/2)*M_PI && actualPoint.x < vehicleActualFrontAxlePosition.x && actualPoint.y < vehicleActualFrontAxlePosition.y) || (vehicleActualFrontAxlePosition.h > (3/2)*M_PI && vehicleActualFrontAxlePosition.h < 2*M_PI && actualPoint.x > vehicleActualFrontAxlePosition.x && actualPoint.y < vehicleActualFrontAxlePosition.y))
			//if ((vehicleActualFrontAxlePosition.h >= (3/2)*M_PI && vehicleActualFrontAxlePosition.h <= M_PI/2 && actualPoint.x > vehicleActualFrontAxlePosition.x) || (vehicleActualFrontAxlePosition.h >= M_PI/2 && vehicleActualFrontAxlePosition.h <= (3/2)*M_PI && actualPoint.x < vehicleActualFrontAxlePosition.x) || (vehicleActualFrontAxlePosition.h >= 0 && vehicleActualFrontAxlePosition.h <= M_PI && actualPoint.y > vehicleActualFrontAxlePosition.y) || (vehicleActualFrontAxlePosition.h >= M_PI && vehicleActualFrontAxlePosition.h <= 2*M_PI && actualPoint.y < vehicleActualFrontAxlePosition.y))
			//{
			
			//calc vector from carfrontposition to actualpoint
			double x_vec = actualPoint.x - carPosition.x;
			double y_vec = actualPoint.y - carPosition.y;
			double angleFromCarToActualPoint = atan2(y_vec, x_vec);
			//LOG_INFO("Angle from Car to NextPoint: %f", angleFromCarToActualPoint * 180.0 / M_PI );
			if(angleFromCarToActualPoint <= M_PI/2 || angleFromCarToActualPoint >= 3/2 * M_PI){

				//calc norm to carPosition
				double dist = sqrt(pow(actualPoint.x - carPosition.x, 2) + pow(actualPoint.y - carPosition.y, 2));
				//LOG_INFO("In fancy logic-if" );
				if (dist < min_dist)
				{
					min_dist = dist;
					actual_min_dist_poly_index = i;
					min_poly_p = j;
					carPosition.x = actualPoint.x;
					carPosition.y = actualPoint.y;
					carPosition.h = actualPoint.h;

					if (trajectoryArray[i].backwards)
					{
						carPosition.h = wrapTo2Pi(carPosition.h + M_PI);
					}
				}
			}
			//}
		}
	}
	printf("Point from Poly is: x: %f, y: %f, h: %f", carPosition.x, carPosition.y, carPosition.h * 180.0 / M_PI );
	printf("End of trajektorie loop" );

	if (actual_min_dist_poly_index != last_min_dist_poly_index)
	{
		poly_completed = true;
		//last_min_dist_poly_index = actual_min_dist_poly_index;
	}

	// Function value and ID of Poly with smallest distance to given car point
	// idealPolyPoint->id = trajectories[actual_min_dist_poly_index].id;
	// TODO: not compiling, there is no parameter p
	// idealPolyPoint->p = min_poly_p;

	// Point on Poly with smallest distance to given car point
	//idealPoint->x = min_dist_x;
	//idealPoint->y = min_dist_y;
	//idealPoint->h = min_dist_h;
}

void LITD_VirtualCar::calcSteeringAngle(){
    double rad2degree = 180.0 / M_PI;
    //vector between car and virtualpoint
	// call getNextVirtualPointOnPoly -> Result is vehicleTargetFrontAxlePosition
    Vector2d diff = carPosition.getVector2d() - carPosition.getVector2d();

    //calc sign to steer in direction of road
    int sign = 1;
    double diff_heading_abs = wrapTo2Pi(atan2(diff(1), diff(0)));
    if(wrapTo2Pi(diff_heading_abs - wrapTo2Pi(carPosition.h))> 4.712 ){
        sign = -1;
    }

    //calc normal distance of tangent to car (e)
    // double e = (vehicleTargetFrontAxlePosition.getVector2d() - vehicleActualFrontAxlePosition.getVector2d()).norm() * sign;
	double e = diff.norm()*sign;

    //calc angle between car heading and point tangent
    double theta_c =  wrapTo2Pi(vehicleTargetFrontAxlePosition.h) - wrapTo2Pi(carPosition.h);

    //calc steering-angle with stanley-approach
	double dynamicStanleyPart = 0;
	if(carSpeed > 0.02){
		 dynamicStanleyPart = atan2(stanleyGain * e, carSpeed);
	}
    vehicleSteeringAngle = theta_c + dynamicStanleyPart;

    //Debug Messages
    std::cout << "-----------------------" << std::endl;
    std::cout << "point heading : " << vehicleTargetFrontAxlePosition.h << "(" << rad2degree * vehicleTargetFrontAxlePosition.h << "°)" << std::endl;
    std::cout << "car heading: " << carPosition.h << "(" << rad2degree * carPosition.h << "°)" << std::endl;
    std::cout << "diff heading: " << diff_heading_abs << "(" << rad2degree * diff_heading_abs << "°)" << std::endl;
    std::cout << "e: " << e << std::endl;
    std::cout << "Theta_C: " << theta_c << "(" << rad2degree * theta_c << "°)" << std::endl;
    std::cout << "Steering Angle: " << carSteeringAngle << "(" << rad2degree * carSteeringAngle << "°)" << std::endl;
    std::cout << "-----------------------" << std::endl;
	

}

void LITD_VirtualCar::calculateBackPos(){
    double dx = cos(carPosition.h)*CAR_AXIS_DIST;
    double dy = sin(carPosition.h)*CAR_AXIS_DIST;

    carBackPosition.x = carPosition.x - dx;
    carBackPosition.y = carPosition.y - dy;
    carBackPosition.h = carPosition.h;

}

void LITD_VirtualCar::calculateFrontkPos(){
    double dx = cos(carPosition.h)*CAR_AXIS_DIST;
    double dy = sin(carPosition.h)*CAR_AXIS_DIST;

    carPosition.x = carBackPosition.x + dx;
    carPosition.y = carBackPosition.y + dy;
    carPosition.h = carBackPosition.h;

}

double LITD_VirtualCar::getActSpeed(double dtime){
    double distance = (carPosition.getVector2d() - vpOld.getVector2d()).norm();
    std::cout << "Distance : " << distance << std::endl;
    vpOld = carPosition;
    double actSpeed = distance / dtime;
    return actSpeed;
}

void LITD_VirtualCar::speedRegulator(double dtime){
    double actSpeed = getActSpeed(dtime);
    double diff = speed - actSpeed;
    vIntegrate += diff;

    carSpeed = vKp * diff + vKi * dtime * vIntegrate;

    std::cout << "-----------------------" << std::endl;
    std::cout << "Vp speed : " << speed << std::endl;
    std::cout << "Act Speed : " << actSpeed << std::endl;
    std::cout << "Speed_diff : " << diff << std::endl;
    std::cout << "Integrate val: " << vIntegrate << std::endl;
    std::cout << "carspeed: " << carSpeed  << std::endl;
    std::cout << "-----------------------" << std::endl;
}

//Params: position of otpimum point, delta time step
//returns the new position of the car after dtime
LITD_VirtualPoint LITD_VirtualCar::updateStep(double dtime)
{
    calculateFrontkPos();
    char* buffer[1024];
    printf( "Point of FrontPosition: x: %f, y: %f, h: %f", carPosition.x, carPosition.y, carPosition.h * 180.0 / M_PI );
	getNextVirtualPointOnPoly();
    printf("Point of SetPoint: x: %f, y: %f, h: %f", vehicleTargetFrontAxlePosition.x, vehicleTargetFrontAxlePosition.y, vehicleTargetFrontAxlePosition.h * 180.0 / M_PI );
	calcSteeringAngle();
	printf("SteeringAngle in grad: %f", vehicleSteeringAngle * 180.0 / M_PI );
	

   /* double rad2degree = 180.0 / M_PI; 
    //vector between car and virtualpoint
    Vector2d diff = vp.getVector2d() - carPosition.getVector2d();

    //calc sign to steer in direction of road
    int sign = 1;
    double diff_heading_abs = wrapTo2Pi(atan2(diff(1), diff(0)));
    if(wrapTo2Pi(diff_heading_abs - wrapTo2Pi(vp.h))> 4.712 ){
        sign = -1;
    }
    //calc new direction

    //calc normal distance of tangent to car (e)
    e = (vp.getVector2d() - carPosition.getVector2d()).norm() * sign;

    //calc angle between car heading and point tangent
    theta_c =  wrapTo2Pi(vp.h) - wrapTo2Pi(carPosition.h);

    //calc steering-angle with stanley-approach
    carSteeringAngle = (theta_c + atan2(stanleyGain*e, carSpeed));

    if(carSteeringAngle < -M_PI/4){
        carSteeringAngle = -M_PI/4;
        std::cout << "Steering angle < -45° "  << std::endl;
    } else if(carSteeringAngle > M_PI/4){
        carSteeringAngle = M_PI/4;
        std::cout << "Steering angle > 45° "  << std::endl;
    }
    //Debug Messages
    std::cout << "-----------------------" << std::endl;
    std::cout << "point heading : " << vp.h << "(" << rad2degree * vp.h << "°)" << std::endl;
    std::cout << "car heading: " << carPosition.h << "(" << rad2degree * carPosition.h << "°)" << std::endl;
    std::cout << "diff heading: " << diff_heading_abs << "(" << rad2degree * diff_heading_abs << "°)" << std::endl;
    std::cout << "e: " << e << std::endl;
    std::cout << "Theta_C: " << theta_c << "(" << rad2degree * theta_c << "°)" << std::endl;*/

    
	
	
    
    
    //Simulate the carposition of the back axis
    for (int i=0; i<SIM_STEPS; i++){
    
        std::cout << "h: " << carPosition.h << std::endl;
        carBackPosition.h += tan(vehicleSteeringAngle)/CAR_AXIS_DIST * carSpeed * dtime/SIM_STEPS;
        carBackPosition.x += cos(carPosition.h) * carSpeed * dtime/SIM_STEPS;
        carBackPosition.y += sin(carPosition.h) * carSpeed * dtime/SIM_STEPS;
    }
    //calculate the new front axis point
    //calculateFrontkPos();

    /*std::cout << "new car pos x: " << carPosition.x << std::endl;
    std::cout << "new car pos y: " << carPosition.y << std::endl;
    std::cout << "new car back-pos x: " << carBackPosition.x << std::endl;
    std::cout << "new car back-pos y: " << carBackPosition.y << std::endl;
    std::cout << "steering angle: " << carSteeringAngle << "(" << rad2degree * carSteeringAngle << "°)" << std::endl;
    std::cout << "-----------------------" << std::endl;*/

    //calculateBackPos();
 
    speedRegulator(dtime);
    //calc next point of front axis
    LITD_VirtualPoint newFrontAxisPoint;
    //newFrontAxisPoint.x = carPosition.x+ cos(theta_c) * carSpeed  * dtime;
    //newFrontAxisPoint.y = carPosition.y + sin(theta_c) * carSpeed * dtime;
    //newFrontAxisPoint.h = tan(carSteeringAngle) / (CAR_AXIS_DIST);

    //carPosition = newFrontAxisPoint;



}


