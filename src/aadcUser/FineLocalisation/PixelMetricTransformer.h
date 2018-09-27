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

    PixelMetricTransformer(double matrix[2][3]);
    PixelMetricTransformer();

    Point2d toMeter(double x, double y);

    Point2d toPixel(double x, double y);

private:
    Matx23f matrix;
    Matx23f inverted;

};


#endif //AADC_USER_PIXELMETRICTRANSFORMER_H
