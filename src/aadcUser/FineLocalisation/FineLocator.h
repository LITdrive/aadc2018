//
// Created by Stefan on 23.08.2018.
//

#ifndef AADC_USER_FINELOCATOR_H
#include <opencv2/opencv.hpp>
#include <math.h>
#include "PixelMetricTransformer.h"

#define AADC_USER_FINELOCATOR_H

using namespace cv;

class FineLocator {
private:
    /*! The scaled image of the Map*/
    Mat scaledMap;

    float angleMin = 0, angleMax = 0, angleInc  = 1, angleCnt = 1;

    float ret[4] = {0, 0, 0, 0};

    PixelMetricTransformer pmt;

public:
    /*! Constructor*/
    FineLocator();
    FineLocator(char* pathToScaledMap);

    /*! Destructor*/
    ~FineLocator();

    /*! \brief  returns x,y and confidence
     *          returns a Point3f with x=x, y=y, z=confidence level
     * */

    float* localize(Mat img_bv, float theta, Point2f pos, float offset, int size=20);

    void setMap(char* pathToScaledMap);
    void setPixelMetricTransformer(PixelMetricTransformer pixelMetricTransformer);

    void setAngleSearchSpace(float min, float max, int cnt);
};


#endif //AADC_USER_FINELOCATOR_H
