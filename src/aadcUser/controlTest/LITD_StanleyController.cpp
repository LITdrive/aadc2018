#include LITD_StanleyController.h

#include "LITD_VirtualPoint.h"
#include "math_utilities.h"
#include "Eigen/Dense"

LITD_StanleyController::LITD_StanleyController(){
    actPolyId = 0;
}

double LITD_StanleyController::calcSteeringAngle(){
    double rad2degree = 180.0 / M_PI;
    //vector between car and virtualpoint
    Vector2d diff = vp.getVector2d() - carPosition.getVector2d();

    //calc sign to steer in direction of road
    int sign = 1;
    double diff_heading_abs = wrapTo2Pi(atan2(diff(1), diff(0)));
    if(wrapTo2Pi(diff_heading_abs - wrapTo2Pi(vp.h))> 4.712 ){
        sign = -1;
    }
        //calc normal distance of tangent to car (e)
    double e = (vp.getVector2d() - carPosition.getVector2d()).norm() * sign;

    //calc angle between car heading and point tangent
    double theta_c =  wrapTo2Pi(vp.h) - wrapTo2Pi(carPosition.h);

    //calc steering-angle with stanley-approach
    double carSteeringAngle = (theta_c + atan2(stanleyGain*e, carSpeed));

    if(carSteeringAngle < -M_PI/4){
        carSteeringAngle = -M_PI/4;
        std::cout << "Steering angle < -45° "  << std::endl;
    } else if(carSteeringAngle > M_PI/4){
        carSteeringAngle = M_PI/4;
        std::cout << "Steering angle > 45° "  << std::endl;
    }
    //Debug Messages
    std::cout << "----------forward-------------" << std::endl;
    std::cout << "point heading : " << vp.h << "(" << rad2degree * vp.h << "°)" << std::endl;
    std::cout << "car heading: " << carPosition.h << "(" << rad2degree * carPosition.h << "°)" << std::endl;
    std::cout << "diff heading: " << diff_heading_abs << "(" << rad2degree * diff_heading_abs << "°)" << std::endl;
    std::cout << "e: " << e << std::endl;
    std::cout << "Theta_C: " << theta_c << "(" << rad2degree * theta_c << "°)" << std::endl; 
    
    return carSteeringAngle;


}

void LITD_StanleyController::calculateFrontkPos(){
    double dx = cos(carPosition.h)*CAR_AXIS_DIST;
    double dy = sin(carPosition.h)*CAR_AXIS_DIST;

    carFrontPosition.x = carBackPosition.x + dx;
    carFrontPosition.y = carBackPosition.y + dy;
    carFrontPosition.h = carBackPosition.h;

}

void LITD_StanleyController::updatePolyList(poly_t polys[], uint8_t polyLen,){
    //expect always "PLOYLIST_LEN" new polys in each updateStep
    if(polyLen > PLOYLIST_LEN ||  polyLen < PLOYLIST_LEN){
        std::cout << "Got wrong number of new polys " << std::endl;
        return;
    }
    //update polyList
    for(int i=0; i<polyLen; i++){
        polylist[i] = polys[i];
    }
}



void LITD_StanleyController::updateStep(poly_t polys[], uint8_t polyLen, LITD_VirtualPoint actPos ){

    updatePolyList();

    carBackPosition = actPos;
    calculateFrontkPos();

    //get nearest Point on Poly-track
    getNextVirtualPointOnPoly(polys, polyLen, idealPolyPoint, idealPoint, carFrontPosition);

    //calc Steeringangle
    steeringAngle = calcSteeringAngle();
}


