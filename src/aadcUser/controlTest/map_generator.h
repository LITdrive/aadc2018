//------------------- hpp Part

#pragma once

#include <vector>
#include "Eigen/Dense"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "map_element.h"
#include "Pose2d.h"



class MapGenerator
{
  public:
    MapGenerator();
    void generateLine(const Point2d& startPoint, const Point2d& endPoint);
    void generateArc(Orientation orientation, const Point2d& startPoint, const Point2d& centerPoint, const Point2d& endPoint);
    MapElements getMapElements();
    void plot(cv::Mat& img, double pixelPerMeter) const;
  private:
    MapElements m_elements;
};

