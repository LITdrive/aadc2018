#ifndef LITD_VIRTUALCAR_H
#define LITD_VIRTUALCAR_H

#include "LITD_VirtualPoint.h"

#define CAR_AXIS_DIST 0.365
#define SIM_STEPS 10

class LITD_VirtualCar
{
public:
    LITD_VirtualCar();
    LITD_VirtualPoint updateStep(LITD_VirtualPoint vp, double dtime);


//private:

    void speedRegulator(LITD_VirtualPoint vp, double dtime);
    double getActSpeed(LITD_VirtualPoint vp, double dtime);

    double carSpeed;
    double carSteeringAngle;
    LITD_VirtualPoint carPosition;
    //SpeedGenerator speedGenerator;

    double e;
    double theta_c;

    //Controller Params
    double stanleyGain;
    double vKp;
    double vKi;

    //PI variables
    double vIntegrate;

    //Simulation variables
    LITD_VirtualPoint vpOld;
};

#endif // LITD_VIRTUALCAR_H
