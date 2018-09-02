#include <math.h>
//#include <iostream>
#include "map_generator.h"

MapGenerator::MapGenerator()
{
}

void MapGenerator::generateLine(const Point2d& startPoint, const Point2d& endPoint)
{
  MapElement element(MapElementType::LINE, startPoint, endPoint);
  m_elements.push_back(element);
}
void MapGenerator::generateArc(Orientation orientation, const Point2d& startPoint, const Point2d& centerPoint, const Point2d& endPoint)
{
  MapElement element(MapElementType::ARC, orientation, startPoint, centerPoint, endPoint);
  m_elements.push_back(element);
}

//    Points2d generateArc(Point2d centerPoint, double radius, double thetaStart, double thetaEnd);
MapElements MapGenerator::getMapElements()
{
  return m_elements;
}


void MapGenerator::plot(cv::Mat& img, double pixelPerMeter) const
{
  //find minimum and maximum x,y
  double xMin = std::numeric_limits<double>::max();
  double xMax = -std::numeric_limits<double>::max();
  double yMin = std::numeric_limits<double>::max();
  double yMax = -std::numeric_limits<double>::max();

  for(MapElement element : m_elements)
  {
    if(xMin > element.getStartPoint()(0))
    {
      xMin = element.getStartPoint()(0);
    }
    if(xMin > element.getEndPoint()(0))
    {
      xMin = element.getEndPoint()(0);
    }
    if(xMax < element.getStartPoint()(0))
    {
      xMax = element.getStartPoint()(0);
    }
    if(xMax < element.getEndPoint()(0))
    {
      xMax = element.getEndPoint()(0);
    }
    if(yMin > element.getStartPoint()(1))
    {
      yMin = element.getStartPoint()(1);
    }
    if(yMin > element.getEndPoint()(1))
    {
      yMin = element.getEndPoint()(1);
    }
    if(yMax < element.getStartPoint()(1))
    {
      yMax = element.getStartPoint()(1);
    }
    if(yMax < element.getEndPoint()(1))
    {
      yMax = element.getEndPoint()(1);
    }
  }
  double border = 1.0;
  double xOffsetPixel = -xMin * pixelPerMeter + border * pixelPerMeter; //add 1 Meter border
  double yOffsetPixel = -yMin * pixelPerMeter + border * pixelPerMeter;

  double imgSizeX = xOffsetPixel + xMax * pixelPerMeter + border * pixelPerMeter;
  double imgSizeY = yOffsetPixel + yMax * pixelPerMeter + border * pixelPerMeter;

  if(imgSizeY > img.size().height)
  {
    throw std::runtime_error("[MapGenerator::plot] Image height is too low.");
  }
  imgSizeY = img.size().height;

  if(imgSizeX > img.size().width)
  {
    throw std::runtime_error("[MapGenerator::plot] Image width is too low.");
  }
  imgSizeX = img.size().width;

  for(MapElement element : m_elements)
  {
    if(element.getType() == MapElementType::LINE)
    {
//      void line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
      //call yMax - y since image coordinates are wrong defined
      cv::Point startPoint(element.getStartPoint()(0) * pixelPerMeter + xOffsetPixel, imgSizeY - (element.getStartPoint()(1) * pixelPerMeter + yOffsetPixel));
      cv::Point endPoint(element.getEndPoint()(0) * pixelPerMeter + xOffsetPixel, imgSizeY - (element.getEndPoint()(1) * pixelPerMeter + yOffsetPixel));

      cv::line(img, startPoint, endPoint, cv::Scalar::all(0));
    }
    else if(element.getType() == MapElementType::ARC)
    {

      cv::Point centerPoint(element.getCenterPoint()(0) * pixelPerMeter + xOffsetPixel, imgSizeY - (element.getCenterPoint()(1) * pixelPerMeter + yOffsetPixel));
      double rad2degree = 180.0 / M_PI;
      double angle = 0.0 * rad2degree;
      double startAngle = rad2degree * atan2(element.getStartPoint()(1) - element.getCenterPoint()(1), element.getStartPoint()(0) - element.getCenterPoint()(0));
      double endAngle = rad2degree * atan2(element.getEndPoint()(1) - element.getCenterPoint()(1), element.getEndPoint()(0) - element.getCenterPoint()(0));

      double diffAngle = endAngle - startAngle;

      //thats just necessary since strange behavior of arcs in opencv
      while(diffAngle > 180.0) diffAngle -= 360.0;
      while(diffAngle < -180.0) diffAngle += 360.0;
      startAngle = - startAngle;
      if(element.getOrientation() == Orientation::CCW)
      {
        endAngle = startAngle - diffAngle;
      }
      else
      {
        endAngle = startAngle + diffAngle;
      }
      cv::ellipse(img, centerPoint, cv::Size(element.getRadius() * pixelPerMeter, element.getRadius() * pixelPerMeter), angle, startAngle, endAngle,  cv::Scalar::all(0));
    }
  }
}




