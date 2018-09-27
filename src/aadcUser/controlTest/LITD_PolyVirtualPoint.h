#ifndef LITD_POLYVIRTUALPOINT_H
#define LITD_POLYVIRTUALPOINT_H

#include LITD_VirtualPoint.h

#define ITERATION_PER_POLY 10

struct polyPoint_t {
    float p;
    uint16_t id;
};

class LITD_PolyVirtualPoint{

    public:
    LITD_PolyVirtualPoint();

    void getNextVirtualPointOnPoly(poly_t polys[], uint16_t polyLen, polyPoint_t idealPolyPoint, LITD_VirtualPoint idealPoint);

    private:

}


#endif