#ifndef VIRTUAL_CAR_H
#define VIRTUAL_CAR_H

#include "virtual_point.h"
#include "speed_generator.h"

#define CAR_AXIS_DIST 0.365
#define SIM_STEPS 10

class virtualCar
{
public:
    virtualCar();
    VirtualPoint updateStep(VirtualPoint vp, double dtime);


//private:
    double carSpeed;
    double carSteeringAngle;
    VirtualPoint carPosition;
    SpeedGenerator speedGenerator;

    double stanleyGain;

    double e;
    double theta_c;
};

#endif // VIRTUAL_CAR_H
