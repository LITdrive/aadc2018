#ifndef LITD_STANLEYCONTROL_H
#define LITD_STANLEYCONTROL_H

#include "LITD_VirtualPoint.h"

#define POLYLIST_LEN 10
#define ITERATION_PER_POLY 10

struct polyPoint_t {
	float p;
	uint16_t id;
};

struct poly_t {
    //x polynom
    float au;
    float bu;
    float cu;
    float du;
    //y polynom
    float av;
    float bv;
    float cv;
    float dv;
    //id
    uint16_t id;
	// Nicht nötig? Immer nur Werte in Bereich [0, 1] in Polynom einsetzen
    //start value of polynom
    //float start;
    //end value of polynom
    //float end;
};



class LITD_StanleyController{
    public:
        LITD_StanleyController();

		double calcSteeringAngle(LITD_VirtualPoint frontAxlePosition, LITD_VirtualPoint idealPoint, double carSpeed);

		void calculateFrontAxlePosition(LITD_VirtualPoint rearAxlePosition, LITD_VirtualPoint * frontAxlePosition);

		void updatePolyList(poly_t polys[], uint8_t polyLen);

		void updateStep(poly_t polys[], uint8_t polyLen, LITD_VirtualPoint actPos);

		void getNextVirtualPointOnPoly(poly_t polys[], uint8_t polyLen, polyPoint_t * idealPolyPoint, LITD_VirtualPoint * idealPoint, LITD_VirtualPoint carPosition);

		void calcVirtualPointfromPoly(poly_t poly, double p, LITD_VirtualPoint * vp);

    private:
        
        poly_t polylist[POLYLIST_LEN];
        uint16_t actPolyId;

        //LITD_VirtualPoint carFrontPosition;
        //LITD_VirtualPoint carBackPosition;

        //nearestPoint from given carPosition
        //LITD_VirtualPoint idealPoint;
        //polyPoint_t idealPolyPoint;

        double steeringAngle;


};

#endif // LITD_STANLEYCONTROL_H