#include "LITD_PolyVirtualPoint.h"
#include "LITD_StanleyController.h"
#include "math_utilities.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

using namespace std;

LITD_PolyVirtualPoint::LITD_PolyVirtualPoint(){

}

 void LITD_PolyVirtualPoint::calcVirtualPointfromPoly(poly_t poly, double p, LITD_VirtualPoint* vp){
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

void LITD_PolyVirtualPoint::getNextVirtualPointOnPoly(poly_t polys[], uint8_t polyLen, polyPoint_t* idealPolyPoint, LITD_VirtualPoint* idealPoint, LITD_VirtualPoint carPosition){

    LITD_VirtualPoint actPoint;
	double min_dist = DBL_MAX;
	int min_poly_index = 0;
	double min_poly_p = 0;
	double min_dist_x = DBL_MAX;
	double min_dist_y = DBL_MAX;
	double min_dist_h = 0;

     for(int i=0; i<polyLen; i++)
	 {
         for(int j=0; j<=ITERATION_PER_POLY; j++)
		 {
			 // p = [0, 1]
            double p = j/ITERATION_PER_POLY;
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
            
            //save lowest norm in variable and give them back
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