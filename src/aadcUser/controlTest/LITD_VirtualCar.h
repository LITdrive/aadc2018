#ifndef LITD_VIRTUALCAR_H
#define LITD_VIRTUALCAR_H

#include "LITD_VirtualPoint.h"
//#include "aadc_structs.h"

#define CAR_AXIS_DIST 0.365
#define SIM_STEPS 10
#define TRAJECTORY_ARRAY_LEN 2
#define POINTS_PER_POLY 10


class LITD_VirtualCar
{
public:


    struct tTrajectory
    {
        uint32_t  id;
        double ax;
        double bx;
        double cx;
        double dx;
        double ay;
        double by;
        double cy;
        double dy;
        double start;
        double end;
        bool backwards;
    } ;

    LITD_VirtualCar();
    LITD_VirtualPoint updateStep(double dtime);


//private:


    void speedRegulator(double dtime);
    double getActSpeed(double dtime);
    void calculateBackPos();
    void calculateFrontkPos();

    void getNextVirtualPointOnPoly();
    void calcVirtualPointfromPoly(tTrajectory poly, double p, LITD_VirtualPoint* vp);
    void calcSteeringAngle();

    double carSpeed;
    double carSteeringAngle;
    LITD_VirtualPoint carPosition; //front axis
    LITD_VirtualPoint carBackPosition; //back axis
    LITD_VirtualPoint vehicleTargetFrontAxlePosition;
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

    tTrajectory trajectoryArray[TRAJECTORY_ARRAY_LEN];
    bool poly_completed;
    int actual_min_dist_poly_index = 0;
	int last_min_dist_poly_index = 0;
    double vehicleSteeringAngle;

    double speed = 0.2;
};

#endif // LITD_VIRTUALCAR_H
