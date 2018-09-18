#pragma once

#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "Eigen/Dense"
#include "map_element.h"


class DrawSimulation
{
public:
    DrawSimulation();
    void setMap(MapElements mapElements);
    std::pair<double,double> getMapSize(){ return m_mapSize;}
    void setPixelPerMeter(double ppm){m_pixelPerMeter = ppm;}
    double getPixelPerMeter() { return m_pixelPerMeter;}

    void drawMap(cv::Mat& img);
    void drawCar(cv::Mat& img, double x, double y, double heading, double steerAngle);

    int m_windowHeight;
    int m_windowWidth;
    double m_pixelPerMeter;
    cv::Point m_offsetWheelRearLeft;
    cv::Point m_offsetWheelRearRight;
    cv::Point m_offsetWheelFrontLeft;
    cv::Point m_offsetWheelFrontRight;
    cv::Point m_offsetOutlineRearLeft;
    cv::Point m_offsetOutlineRearRight;
    cv::Point m_offsetOutlineFrontLeft;
    cv::Point m_offsetOutlineFrontRight;
    std::pair<double,double> m_mapSize;
private:
    MapElements m_elems;
    int m2px(double m);
};

