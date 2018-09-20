//
// Created by Stefan on 23.08.2018.
//

#ifndef AADC_USER_FINELOCATOR_H
#include <opencv2/opencv.hpp>
#include <math.h>
#define AADC_USER_FINELOCATOR_H

using namespace cv;

class FineLocator {
private:
    /*! The scaled image of the Map*/
    Mat scaledMap;

public:
    /*! Constructor*/
    FineLocator();
    FineLocator(char* pathToScaledMap);

    /*! Destructor*/
    ~FineLocator();

    /*! call localisation*/
    Point3f localize(Mat img_bv, float theta, Point2f pos, int size=20);

    void setMap(char* pathToScaledMap);
};


#endif //AADC_USER_FINELOCATOR_H
