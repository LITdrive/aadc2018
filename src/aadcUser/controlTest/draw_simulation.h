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
    void getMapSize();
    void setPixelPerMeter();
    void drawMap(cv::Mat img&);
    void drawCar(cv::Mat img&, double x, double y, double heading, double steerAngle);

    double m_windowHeight;
    double m_windowWidth;
    double m_pixelPerMeter;
    cv::Point m_wheelRearLeft;
    cv::Point m_wheelRearRight;
    cv::Point m_wheelFrontLeft;
    cv::Point m_wheelFrontRight;
    std::pair<double,double> m_mapSize;
    double y; //y - coordinate
    double k; //curvature
    double h; //heading
};

