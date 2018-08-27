#include <math.h>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "map_generator.h"

//using namespace std;
//using namespace dlib;

//  ----------------------------------------------------------------------------

//void draw_car(matrix<rgb_pixel>& world, double x, double y, double phi, double delta, double width, double lenControlPointToRear, double lenControlPointToFront, double wheelWidth, double steerLen, double wheelLen)
//{
//    const dpoint controlPoint = point(x, y);
//	draw_solid_circle(world, controlPoint, 1, 0);  //draw control point
//
//    std::vector<dlib::point> outline;
//	outline.push_back(dlib::rotate_point(controlPoint, controlPoint + point(-lenControlPointToRear,width/2), -phi));
//	outline.push_back(dlib::rotate_point(controlPoint, controlPoint + point(lenControlPointToFront, width/2), -phi));
//	outline.push_back(dlib::rotate_point(controlPoint, controlPoint + point(lenControlPointToFront, -width/2), -phi));
//	outline.push_back(dlib::rotate_point(controlPoint, controlPoint + point(-lenControlPointToRear, -width/2), -phi));
//
//	for(int i=0; i < outline.size()-1; i++)
//	{
//		draw_line (world, outline[i], outline[i+1], rgb_pixel(100, 100, 100));
//	}
//	draw_line (world, outline[outline.size()-1], outline[0], rgb_pixel(100, 100, 100));
//
//	//plot wheel right
//	draw_line(world, dlib::rotate_point(controlPoint, controlPoint + point(-wheelLen/2.0, wheelWidth/2), -phi),
//			         dlib::rotate_point(controlPoint, controlPoint + point(wheelLen/2.0, wheelWidth/2), -phi), rgb_pixel(100, 100, 100));
//
//	//plot wheel left
//	draw_line(world, dlib::rotate_point(controlPoint, controlPoint + point(-wheelLen/2.0, -wheelWidth/2), -phi),
//			         dlib::rotate_point(controlPoint, controlPoint + point(wheelLen/2.0, -wheelWidth/2), -phi), rgb_pixel(100, 100, 100));
//
//
//	//plot steer wheelrolPoint, rotate_point(steerPointLeft, steerPointLeft + point(-wheelLen/2.0, 0.0), -delta), -phi),
////			         dlib::rotate_point(controlPoint, rotate_point(steerPointLeft, steerPointLeft + point(wheelLen/2.0, 0.0), -delta), -phi), rgb_pixel(100, 100, 100));
////	draw_line(world, dlib::rotate_point(controlPoint, steerPointLeft + point(-wheelLen/2.0, 0.0), -phi),
////			         dlib::rotate_point(controlPoint, steerPointLeft + point(wheelLen/2.0, 0.0), -phi), rgb_pixel(100, 100, 100));
//
//	//plot steer wheel left
//	const dpoint steerPointRight = controlPoint + point(steerLen, -wheelWidth/2);
//	draw_line(world, dlib::rotate_point(controlPoint, rotate_point(steerPointRight, steerPointRight + point(-wheelLen/2.0, 0.0), -delta), -phi),
//			         dlib::rotate_point(controlPoint, rotate_point(steerPointRight, steerPointRight + point(wheelLen/2.0, 0.0), -delta), -phi), rgb_pixel(100, 100, 100));
//
//
//
//}

int main()
{
  std::cout << "started MapGenerator Test.." << std::endl;
  MapGenerator mapGenerator;
  mapGenerator.generateLine(Point2d(1.0, 0.0), Point2d(3.0, 0.0));
  mapGenerator.generateArc(Orientation::CCW, Point2d(3.0, 0.0), Point2d(3.0, 1.0), Point2d(4.0, 1.0));
  mapGenerator.generateLine(Point2d(4.0, 1.0), Point2d(4.0, 4.0));
  mapGenerator.generateArc(Orientation::CCW, Point2d(4.0, 4.0), Point2d(3.0, 4.0), Point2d(3.0, 5.0));
  mapGenerator.generateLine(Point2d(3.0, 5.0), Point2d(1.0, 5.0));
  mapGenerator.generateArc(Orientation::CCW, Point2d(1.0, 5.0), Point2d(1.0, 4.0), Point2d(0.0, 4.0));
  mapGenerator.generateLine(Point2d(0.0, 4.0), Point2d(0.0, 1.0));
  mapGenerator.generateArc(Orientation::CCW, Point2d(0.0, 1.0), Point2d(1.0, 1.0), Point2d(1.0, 0.0));
  cv::Mat img(1000, 1000, CV_8UC3, cv::Scalar::all(255));
  mapGenerator.plot(img, 100.0);



//  cv::ellipse(img, cv::Point(800.0, 800.0), cv::Size(100.0, 100.0), 0, 90.0,  0.0,  cv::Scalar::all(0));
//  cv::ellipse(img, cv::Point(800.0, 200.0), cv::Size(100.0, 100.0), 0, 0.0, -90.0,  cv::Scalar::all(0));
//  cv::ellipse(img, cv::Point(200.0, 200.0), cv::Size(100.0, 100.0), 0, -90.0, -180.0,  cv::Scalar::all(0));
//  cv::ellipse(img, cv::Point(200.0, 800.0), cv::Size(100.0, 100.0), 0, -180.0, -270.0,  cv::Scalar::all(0));



  cv::namedWindow( "Mapgenerator Test", cv::WINDOW_AUTOSIZE );// Create a window for display.

  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);

  try
  {
      cv::imwrite("alpha.png", img, compression_params);
  }
  catch (std::runtime_error& ex)
  {
      printf("Exception converting image to PNG format: %s\n", ex.what());
      return 1;
  }

//  cv::imshow("Mapgenerator Test", img);

  std::cout << "Finished MapGenerator Test.." << std::endl;
  return 0;
}



//  ----------------------------------------------------------------------------
