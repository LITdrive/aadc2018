//
// Created by Stefan on 23.08.2018.
//

#define _USE_MATH_DEFINES
#include "FineLocator.h"
#include "stdafx.h"
#include <math.h>
#include "PixelMetricTransformer.h"

#define WEIGHT_SEARCH_SIZE 1        //1 for 3x3 2 for 5x5 etc.
#define DEGTORAD M_PI/180
#define RADTODEG 180/M_PI
#define DEBUG_LOC false
#define CORR_THRESH 0.2

FineLocator::FineLocator(){
}

FineLocator::FineLocator(char *pathToScaledMap){
    scaledMap = imread(pathToScaledMap);
}

FineLocator::~FineLocator() {
    scaledMap.release();
}

void FineLocator::setSearchSpace(int sss) {
    searchSpaceSize = sss;
}

void FineLocator::setMap(char* pathToScaledMap){
    scaledMap = imread(pathToScaledMap);
    if(scaledMap.empty()) cout << "\n\n\n\n\n\n\nERROR: no valid Image Supplied \n\n\n\n\n\n\n";
}

void FineLocator::setPixelMetricTransformer(PixelMetricTransformer pixel2metric){
    pmt = pixel2metric;
}

float* FineLocator::localize(Mat img_bv, float theta, float in_x, float in_y, float pictureOffset, bool initial) {
    double angleSum = 0, weightedAngleOff = 0, weightedXSum = 0, weightedYSum = 0, anglemax = CORR_THRESH, angleFaktor = initial ? angleRadiusFaktor : 1;
    for(float x_off = initial ? -posSearchSpace : 0.0f; (initial && (x_off < posSearchSpace)) || (!initial && (x_off < posSearchInc)); x_off += posSearchInc) {
        for (float y_off = initial ? -posSearchSpace : 0.0f; (initial && (y_off < posSearchSpace)) || (!initial && (y_off < posSearchInc)); y_off += posSearchInc) {
            Point2f pos = Point2f(in_x+x_off, in_y + y_off);
            for (double angleOff = angleMin * angleFaktor; angleOff <= angleMax * angleFaktor; angleOff += angleInc) {
                // origin world coordinates -> car location
                double x_pic = pos.x + pictureOffset * cos(theta + angleOff * DEGTORAD), y_pic =
                        pos.y + pictureOffset * sin(theta + angleOff * DEGTORAD);
                Point2d pos_px = pmt.toPixel(x_pic, y_pic);
                //LOG_INFO("pos_px: %4.2f\t%4.2f", pos_px.x, pos_px.y);
                Mat car_coord_shift = Mat::eye(3, 3, CV_64F);
                car_coord_shift.at<double>(0, 2) = -pos_px.x;
                car_coord_shift.at<double>(1, 2) = -pos_px.y;
                // map rotation -> car rotation
                Mat car_rot = Mat::eye(3, 3, CV_64F);
                Mat rot;
                rot = getRotationMatrix2D(Point2f(0, 0), -(theta * RADTODEG + angleOff) + 90, 1.0f);
                rot.convertTo(rot, CV_64F);
                rot(Rect(0, 0, 2, 2)).copyTo(car_rot(Rect(0, 0, 2, 2)));
                // car location -> picture location
                Mat offset = Mat::eye(3, 3, CV_64F);
                offset.at<double>(0, 2) = img_bv.size[1] / 2.f + searchSpaceSize / 2.f;
                offset.at<double>(1, 2) = img_bv.size[0] + searchSpaceSize / 2.f;
                // combine in reverse order
                Mat combined = offset * car_rot * car_coord_shift;
                combined = combined(Rect(0, 0, 3, 2)).clone(); // only select the Affine Part of the Transformation
                Mat search_space;
                warpAffine(scaledMap, search_space, combined,
                           Size(img_bv.size[1] + searchSpaceSize, img_bv.size[0] + searchSpaceSize), INTER_LINEAR,
                           BORDER_REPLICATE);
                Mat search_result;
                matchTemplate(search_space, img_bv, search_result, TM_CCOEFF_NORMED);
                double mi, ma;
                Point mil, mal;
                minMaxLoc(search_result, &mi, &ma, &mil, &mal);
                //--------------------Write-Result----
                if (angleOff * angleOff <= 1e-2 && DEBUG_LOC) {
                    double range = ma - mi;
                    Mat res_pic = search_result.clone();
                    res_pic = (res_pic - mi) * 255 / range;
                    res_pic.convertTo(res_pic, CV_8UC1);
                    cvtColor(res_pic, res_pic, cv::COLOR_GRAY2BGR);
                    imwrite("/home/aadc/share/adtf/data/res.png", res_pic);
                    imwrite("/home/aadc/share/adtf/data/sspace.png", search_space);
                }
                //------------------------------------
                float max_loc_weighted_x = 0, max_loc_weighted_y = 0, sum = 0;
                for (int x_off = -WEIGHT_SEARCH_SIZE; x_off <= WEIGHT_SEARCH_SIZE; x_off++) {
                    for (int y_off = -WEIGHT_SEARCH_SIZE; y_off <= WEIGHT_SEARCH_SIZE; y_off++) {
                        if (mal.x + x_off < 0 || mal.x + x_off >= searchSpaceSize || mal.y + y_off < 0 ||
                            mal.y + y_off >= searchSpaceSize) {
                            continue; //TODO Correct Mathematical model
                        }
                        float curr_res = search_result.at<float>(mal.x + x_off, mal.y + y_off);
                        sum += curr_res;
                        max_loc_weighted_x += (mal.x + x_off) * curr_res;
                        max_loc_weighted_y += (mal.y + y_off) * curr_res;
                    }
                }

                if (sum == 0) sum = 1e-6; //prevent div by 0 error
                Mat reverse;
                invertAffineTransform(combined, reverse);
                Mat location_global = reverse * Mat(Vec3d(max_loc_weighted_x / sum + img_bv.size[1] / 2.0,
                                                          max_loc_weighted_y / sum + img_bv.size[0], 1));
                /*
                weightedAngleOff += angleOff * ma;
                angleSum += ma;
                */
                Point2d pos_m = pmt.toMeter(location_global.at<double>(0), location_global.at<double>(1));
                /*
                weightedXSum += ma*(pos_m.x - pictureOffset*cos(theta + angleOff*DEGTORAD));
                weightedYSum += ma*(pos_m.y - pictureOffset*sin(theta + angleOff*DEGTORAD));
                */
                if (ma > anglemax) {
                    anglemax = ma;
                    weightedXSum = pos_m.x - pictureOffset * cos(theta + angleOff * DEGTORAD);
                    weightedYSum = pos_m.y - pictureOffset * sin(theta + angleOff * DEGTORAD);
                    weightedAngleOff = angleOff;
                    angleSum = 1;
                }
            }
        }
    }

    if (anglemax > CORR_THRESH){
        ret[0] = weightedXSum/angleSum - in_x ;
        ret[1] = weightedYSum/angleSum - in_y;
        ret[2] = (weightedAngleOff/angleSum)*DEGTORAD;
        ret[3] = anglemax;
    } else { // return safe default Values
        cout << "\n No Good Values found\n";
        ret[0] = 0;
        ret[1] = 0;
        ret[2] = 0;
        ret[3] = 0;
    }
    return ret;
}

void FineLocator::setAngleSearchSpace(float min, float max, int cnt) {
    angleMin = min;
    angleMax = max;
    angleInc = (max - min)/(cnt - 1);
    angleCnt = cnt;
}

void FineLocator::setPosSearchSpace(float posSpace, float posInc, float angleFaktor) {
    posSearchSpace = posSpace;
    posSearchInc = posInc;
    angleRadiusFaktor = angleFaktor;
}
