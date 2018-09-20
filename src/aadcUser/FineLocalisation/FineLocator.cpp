//
// Created by Stefan on 23.08.2018.
//

#include "FineLocator.h"
#include "stdafx.h"
#define WEIGHT_SEARCH_SIZE 1        //1 for 3x3 2 for 5x5 etc.

FineLocator::FineLocator(){

}

FineLocator::FineLocator(char *pathToScaledMap) {
    scaledMap = imread(pathToScaledMap);
}

FineLocator::~FineLocator() {
    scaledMap.release();
}

float rad2grad(float x){
    return (float)(x*M_PI/180.0f);
}

void FineLocator::setMap(char* pathToScaledMap){
    scaledMap = imread(pathToScaledMap);
}

float* FineLocator::localize(Mat img_bv, float theta, Point2f pos, int size) {
    double angleSum = 0, weightedAngleOff = 0, weightedXSum = 0, weightedYSum = 0;
    for(float angleOff= angleMin; angleOff <= angleMax; angleOff = angleOff + angleInc) {
        // left upper corner of map -> car location
        Mat car_coord_shift = Mat::eye(3, 3, CV_32F);
        car_coord_shift.at<float>(0, 2) = -pos.x;
        car_coord_shift.at<float>(1, 2) = -pos.y;
        // map rotation -> car rotation
        Mat car_rot = Mat::eye(3, 3, CV_32F);
        Mat rot = getRotationMatrix2D(Point2f(0, 0), rad2grad(-theta) - angleOff, 1.0f);
        rot(Rect(0, 0, 2, 2)).copyTo(car_rot(Rect(0, 0, 2, 2)));
        // car location -> picture location
        Mat offset = Mat::eye(3, 3, CV_32F);
        offset.at<float>(0, 2) += img_bv.size[0] / 2.f + size / 2.f;
        offset.at<float>(1, 2) += img_bv.size[1] + size / 2.f;
        // combine in reverse order
        Mat combined = offset * car_rot * car_coord_shift;
        combined = combined(Rect(0, 0, 3, 2)).clone(); // only select the Affine Part of the Transformation
        Mat search_space;
        warpAffine(scaledMap, search_space, combined, Size(img_bv.size[1] + size, img_bv.size[0] + size), INTER_LINEAR, BORDER_REPLICATE);
        Mat search_result;
        matchTemplate(search_space, img_bv, search_result, TM_CCOEFF_NORMED);
        double mi, ma;
        Point mil, mal;
        minMaxLoc(search_result, &mi, &ma, &mil, &mal);
        float max_loc_weighted_x = 0, max_loc_weighted_y = 0, sum = 0;
        for (int x_off = -WEIGHT_SEARCH_SIZE; x_off <= WEIGHT_SEARCH_SIZE; x_off++) {
            for (int y_off = -WEIGHT_SEARCH_SIZE; y_off <= WEIGHT_SEARCH_SIZE; y_off++) {
                float curr_res = search_result.at<float>(mal.x + x_off, mal.y + y_off);
                sum += curr_res;
                max_loc_weighted_x += (mal.x + x_off) * curr_res;
                max_loc_weighted_y += (mal.y + y_off) * curr_res;
            }
        }
        Mat reverse;
        invertAffineTransform(combined, reverse);
        Mat location_global = reverse * Mat(Vec3f(max_loc_weighted_x / sum + img_bv.size[0] / 2.f, max_loc_weighted_y / sum + img_bv.size[1], 1));
        weightedAngleOff += angleOff*sum;
        angleSum += sum;
        Point3f(location_global.at<float>(0,0),location_global.at<float>(0,1), ma);

    }
    ret[0] = weightedXSum/angleSum;
    ret[1] = weightedYSum/angleSum;
    ret[2] = weightedAngleOff/angleSum;
    ret[3] = angleSum/9/angleCnt;
    return ret;
}

void FineLocator::setAngleSearchSpace(float min, float max, int cnt) {
    angleMin = min;
    angleMax = max;
    angleInc = (max - min)/(cnt - 1);
    angleCnt = cnt;
}
