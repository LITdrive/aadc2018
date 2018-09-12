#include "virtual_car.h"
#include "virtual_point.h"
#include "math_utilities.h"
#include "Eigen/Dense"

#include <stdio.h>
#include <unistd.h>
#include <iostream>

virtualCar::virtualCar()
{
    carPosition.x = 0.0;
    carPosition.y = -0.2;
    carPosition.h = 0.0;
    carSpeed = 1.0;

    stanleyGain = 0.8;
}

//Params: position of otpimum point, delta time step
//returns the new position of the car after dtime
VirtualPoint virtualCar::updateStep(VirtualPoint vp, double dtime)
{

    //vector between car and virtualpoint
    Vector2d diff = vp.getVector2d() - carPosition.getVector2d();

    //calc sign to steer in direction of road
    int sign = 1;
    double diff_heading_abs = wrapTo2Pi(atan2(diff(1), diff(0)));
    if(diff_heading_abs< wrapTo2Pi(carPosition.h)){
        sign = -1;
    }
    //calc new direction

    //calc normal distance of tangent to car (e)
    e = (vp.getVector2d() - carPosition.getVector2d()).norm() * sign;

    //calc angle between car heading and point tangent
    theta_c =  wrapTo2Pi(vp.h) - wrapTo2Pi(carPosition.h);

    //calc steering-angle with stanley-approach
    carSteeringAngle = theta_c + atan2(stanleyGain*e, carSpeed);

    //Debug Messages
    std::cout << "-----------------------" << std::endl;
    std::cout << "point heading : " << vp.h << std::endl;
    std::cout << "car heading: " << carPosition.h << std::endl;
    std::cout << "diff heading: " << diff_heading_abs << std::endl;
    std::cout << "e: " << e << std::endl;
    std::cout << "Theta_C: " << theta_c << std::endl;
    std::cout << "-----------------------" << std::endl;


    for (int i=0; i<SIM_STEPS; i++){
    
        std::cout << "h: " << carPosition.h << std::endl;
        carPosition.h += tan(carSteeringAngle)/CAR_AXIS_DIST * carSpeed * dtime/SIM_STEPS;
        carPosition.x += cos(carPosition.h) * carSpeed * dtime/SIM_STEPS;
        carPosition.y += sin(carPosition.h) * carSpeed * dtime/SIM_STEPS;
    }


    //calc next point of front axis
    VirtualPoint newFrontAxisPoint;
    //newFrontAxisPoint.x = carPosition.x+ cos(theta_c) * carSpeed  * dtime;
    //newFrontAxisPoint.y = carPosition.y + sin(theta_c) * carSpeed * dtime;
    //newFrontAxisPoint.h = tan(carSteeringAngle) / (CAR_AXIS_DIST);

    //carPosition = newFrontAxisPoint;



}
