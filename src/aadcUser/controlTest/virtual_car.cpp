#include "virtual_car.h"
#include "virtual_point.h"
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

    stanleyGain = 0.5;
}

//Params: position of otpimum point, delta time step
//returns the new position of the car after dtime
VirtualPoint virtualCar::updateStep(VirtualPoint vp, double dtime)
{
 
 //calc speed
//calc distance car-virtualpoint
 double distance = (vp.getVector2d()-carPosition.getVector2d()).norm();
// carSpeed = carSpeed + 1*distance;
//carSpeed = 0.5;

Vector2d diff = vp.getVector2d() - carPosition.getVector2d();

int sign = 1;
if(atan2(diff(1), diff(0))< carPosition.h){
    sign = -1;
}
 //calc new direction

 //get optimum point (only for direction-control)
 VirtualPoint directionVirtualPoint = vp; //getOptimumPosition(carPosition);
 //calc normal distance of tangent to car (e)

  e = (directionVirtualPoint.getVector2d() - carPosition.getVector2d()).norm() * sign;
 //calc angle difference between point-tangent and car
 std::cout << "-----------------------" << std::endl;
 std::cout << "point heading : " << directionVirtualPoint.h << std::endl;
 std::cout << "car heading: " << carPosition.h << std::endl;
  theta_c =  directionVirtualPoint.h - carPosition.h  ;
std::cout << "Theta_C: " << theta_c << std::endl;
std::cout << "-----------------------" << std::endl;

 //calc steering-angle with stanley-approach
 carSteeringAngle = theta_c + atan2(stanleyGain*e, carSpeed);

 //estimate next point of the car.
 //get point on front axis
 //Vector2d axisDist(CAR_AXIS_DIST_MM*1000, 0); //axis distance in x axis
 //Rotation2Dd rot(carPosition.h);
 //Vector2d frontAxis =  rot * axisDist;


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
