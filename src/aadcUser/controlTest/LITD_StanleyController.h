#ifndef LITD_STANLEYCONTROL_H
#define LITD_STANLEYCONTROL_H

#include LITD_PolyVirtualPoint.h

#define PLOYLIST_LEN 10


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
    //start value of polynom
    float start;
    //end value of polynom
    float end;
};



class LITD_StanleyController{
    public:
        LITD_StanleyController();

    private:
        
        poly_t polylist[PLOYLIST_LEN];
        uint16_t actPolyId;

        LITD_VirtualPoint carFrontPosition;
        LITD_VirtualPoint carBackPosition;

        //nearestPoint from given carPosition
        LITD_VirtualPoint idealPoint;
        polyPoint_t idealPolyPoint;

        double steeringAngle;


}

#endif // LITD_STANLEYCONTROL_H