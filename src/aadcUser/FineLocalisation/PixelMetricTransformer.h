//
// Created by aadc on 10.09.18.
//

#ifndef AADC_USER_PIXELMETRICTRANSFORMER_H
#define AADC_USER_PIXELMETRICTRANSFORMER_H

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class PixelMetricTransformer {
public:

    PixelMetricTransformer(float matrix[2][3]);

    float *toMeter(float x, float y);

    float *toPixel(float x, float y);

private:
    Matx23f matrix;
    Matx23f inverted;

};


#endif //AADC_USER_PIXELMETRICTRANSFORMER_H
