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

double LITD_VirtualCar::getActSpeed(LITD_VirtualPoint vp, double dtime){
    double distance = (carPosition.getVector2d() - vpOld.getVector2d()).norm();
    std::cout << "Distance : " << distance << std::endl;
    vpOld = carPosition;
    double actSpeed = distance / dtime;
    return actSpeed;
}

void LITD_VirtualCar::speedRegulator(LITD_VirtualPoint vp, double dtime){
    double actSpeed = getActSpeed(vp, dtime);
    double diff = vp.speed - actSpeed;
    vIntegrate += diff;

    carSpeed = vKp * diff + vKi * dtime * vIntegrate;

    std::cout << "-----------------------" << std::endl;
    std::cout << "Vp speed : " << vp.speed << std::endl;
    std::cout << "Act Speed : " << actSpeed << std::endl;
    std::cout << "Speed_diff : " << diff << std::endl;
    std::cout << "Integrate val: " << vIntegrate << std::endl;
    std::cout << "carspeed: " << carSpeed  << std::endl;
    std::cout << "-----------------------" << std::endl;
}

//Params: position of otpimum point, delta time step
//returns the new position of the car after dtime
LITD_VirtualPoint LITD_VirtualCar::updateStep(LITD_VirtualPoint vp, double dtime)
{

    

    double rad2degree = 180.0 / M_PI; 
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
    std::cout << "Theta_C: " << theta_c << "(" << rad2degree * theta_c << "°)" << std::endl;
    
    
    //Simulate the carposition of the back axis
    for (int i=0; i<SIM_STEPS; i++){
    
        std::cout << "h: " << carPosition.h << std::endl;
        carBackPosition.h += tan(carSteeringAngle)/CAR_AXIS_DIST * carSpeed * dtime/SIM_STEPS;
        carBackPosition.x += cos(carPosition.h) * carSpeed * dtime/SIM_STEPS;
        carBackPosition.y += sin(carPosition.h) * carSpeed * dtime/SIM_STEPS;
    }
    //calculate the new front axis point
    calculateFrontkPos();

    std::cout << "new car pos x: " << carPosition.x << std::endl;
    std::cout << "new car pos y: " << carPosition.y << std::endl;
    std::cout << "new car back-pos x: " << carBackPosition.x << std::endl;
    std::cout << "new car back-pos y: " << carBackPosition.y << std::endl;
    std::cout << "steering angle: " << carSteeringAngle << "(" << rad2degree * carSteeringAngle << "°)" << std::endl;
    std::cout << "-----------------------" << std::endl;

    //calculateBackPos();
 
 speedRegulator(vp, dtime);
    //calc next point of front axis
    LITD_VirtualPoint newFrontAxisPoint;
    //newFrontAxisPoint.x = carPosition.x+ cos(theta_c) * carSpeed  * dtime;
    //newFrontAxisPoint.y = carPosition.y + sin(theta_c) * carSpeed * dtime;
    //newFrontAxisPoint.h = tan(carSteeringAngle) / (CAR_AXIS_DIST);

    //carPosition = newFrontAxisPoint;



}
