//
// Created by Stefan on 23.08.2018.
//

#include "FineLocator.h"

FineLocator::FineLocator(char *pathToScaledMap) {
    scaledMap = imread(pathToScaledMap);
}

FineLocator::~FineLocator() {
    scaledMap.release();
}

Point2i FineLocator::localize(Mat img_bv, float theta, Point2i pos, int size) {
    // left upper corner of map -> car location
    Mat car_coord_shift = Mat::eye(3,3, CV_32F);
    car_coord_shift.at<float>(0, 2) = -pos.x;
    car_coord_shift.at<float>(1, 2) = -pos.y;
    // map rotation -> car rotation
    Mat car_rot = Mat::eye(3,3, CV_32F);
    Mat rot = getRotationMatrix2D(Point2f(0, 0), -theta, 1.0f);
    rot.copyTo(car_rot(Rect_<int>(0,0,1,2)));
    // car location -> picture location
    Mat offset = Mat::eye(3,3, CV_32F);
    offset.at<float>(0, 2) += img_bv.size[0]/2.f + size/2.f;
    offset.at<float>(1, 2) += img_bv.size[1] + size/2.f;  //TODO check if size[1] == 192 etc.
    // combine in reverse order
    Mat combined = offset*car_rot*car_coord_shift;
    combined = combined(Rect_<int>(0,0,1,2)); // only select the Affine Part of the Transformation
    Mat search_space;
    warpAffine(scaledMap, search_space ,combined, Size(img_bv.size[0] + size, img_bv.size[1] + size), INTER_LINEAR, BORDER_REPLICATE);
    Mat search_result;
    matchTemplate(search_space, img_bv, search_result, TM_CCOEFF_NORMED);
    double mi, ma;
    Point mil, mal;
    minMaxLoc(search_result, &mi, &ma, &mil, &mal);
    Mat reverse;
    invertAffineTransform(combined, reverse);
    Mat location_global = reverse*Mat(Vec3f(mal.x + img_bv.size[0]/2.f, mal.y + img_bv.size[1], 1));
    return Point2i(location_global.at<float>(0,0),location_global.at<float>(0,1));

}

float rad2grad(float x){
    return (float)(x*M_PI/180.0f);
}