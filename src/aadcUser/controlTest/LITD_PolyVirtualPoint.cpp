#include LITD_PolyVirtualPoint.h

#include math.h

LITD_PolyVirtualPoint::LITD_PolyVirtualPoint(){

}

 void LITD_PolyVirtualPoint::calcVirtualPointfromPoly(poly_t poly, double p, LITD_VirtualPoint* vp){
     double x = poly.au * power(p, 3) + poly.bu * power(p, 2) + poly.cu * p + poly.du;
     double y = poly.av * power(p, 3) + poly.bv * power(p, 2) + poly.cv * p + poly.dv;

     //tangente durch erste ableitung berechnen
     double x_der = poly.au * power(p, 2) + poly.bu * p + poly.cu;
     double y_der = poly.av * power(p, 2) + poly.bv * p + poly.cv;

     double heading = atan2(y_der/x_der);

     vp->x = x;
     vp->y = y;
     vp->h = heading;


 }

void LITD_PolyVirtualPoint::getNextVirtualPointOnPoly(poly_t polys[], uint16_t polyLen, polyPoint_t* idealPolyPoint, LITD_VirtualPoint* idealPoint, LITD_VirtualPoint carPosition){

    LITD_PolyVirtualPoint actPoint;

     for(int i=0; i<polyLen; i++){
         for(int=j; j<ITERATION_PER_POLY; j++){
            double p = (polys[i].end - polys[i].start) / ITERATION_PER_POLY * j;
            calcVirtualPointfromPoly(polys[i], p, actPoint);

            //calc norm to carPosition
            //save lowest norm in variable and give them back
         }
     }


 }