#include <math.h>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "map_generator.h"
#include "virtual_point_mover.h"
#include "virtual_car.h"

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

int drawMap(){
  MapGenerator mapGenerator;
  mapGenerator.generateLine(Point2d(1.0, 0.0), Point2d(3.0, 0.0));
  mapGenerator.generateArc(Orientation::CCW, Point2d(3.0, 0.0), Point2d(3.0, 1.0), Point2d(4.0, 1.0));
  mapGenerator.generateLine(Point2d(4.0, 1.0), Point2d(4.0, 4.0));
  mapGenerator.generateArc(Orientation::CCW, Point2d(4.0, 4.0), Point2d(3.0, 4.0), Point2d(3.0, 5.0));
  mapGenerator.generateLine(Point2d(3.0, 5.0), Point2d(1.0, 5.0));
  mapGenerator.generateArc(Orientation::CCW, Point2d(1.0, 5.0), Point2d(1.0, 4.0), Point2d(0.0, 4.0));
  mapGenerator.generateLine(Point2d(0.0, 4.0), Point2d(0.0, 1.0));
  mapGenerator.generateArc(Orientation::CCW, Point2d(0.0, 1.0), Point2d(1.0, 1.0), Point2d(1.0, 0.0));
  cv::Mat mapImg(1000, 1000, CV_8UC3, cv::Scalar::all(255));
  mapGenerator.plot(mapImg, pixelPerMeter);
}

int main()
{
  double pixelPerMeter = 100.0;
  double border = 1.0;
  std::cout << "started MapGenerator Test.." << std::endl;
  drawMap();

  VirtualPointMover vpMover;
  vpMover.setMapElements(mapGenerator.getMapElements());

  bool stopSimulation = false;
  double dtime = 0.02;
  cv::namedWindow( "VirtualPointMoverTest", cv::WINDOW_AUTOSIZE );// Create a window for display.

  virtualCar vCar;
  //vCar();

  std::cout << "Car: " << std::endl;
  std::cout << "x: " << vCar.carPosition.x << std::endl;
  std::cout << "y: " << vCar.carPosition.y << std::endl;
   
   //while(1);

  while(!stopSimulation)
  {
    vpMover.updateStep(dtime);
    double speed = vpMover.getSpeed();
    double delta = vpMover.getSteerAngle();
    VirtualPoint vp = vpMover.getVirtualPoint();

    std::cout << "Actual Speed: " << speed << " SteerAngle: " << delta
              << " VP: " << vp.toString() << std::endl;



    cv::Mat img(1000, 1000, CV_8UC3, cv::Scalar::all(255));
    img = mapImg;
    cv::Point virtualPoint(vp.x * pixelPerMeter + border * pixelPerMeter, img.size().height - (vp.y * pixelPerMeter +  border * pixelPerMeter));
    cv::circle(img, virtualPoint, 5, cv::Scalar::all(0));

    

    
    //while(1){
    double rad2degree = 180.0 / M_PI;
    vCar.updateStep(vp, dtime);

    std::cout << "Actual virtualpoint: " << std::endl;
    std::cout << "x: " << vp.x << std::endl;
    std::cout << "y: " << vp.y << std::endl;

    std::cout << "Calculated Parameters: " << std::endl;
    std::cout << "x: " << vCar.carPosition.x << std::endl;
    std::cout << "y: " << vCar.carPosition.y << std::endl;
    std::cout << "h: " << vCar.carPosition.h << "(" << + vCar.carPosition.h* rad2degree << "°)"<< std::endl;

    std::cout << "Distance to point: " << vCar.e << std::endl;
    std::cout << "Theta: " << vCar.theta_c << std::endl;
    std::cout << "Car Speed: " << vCar.carSpeed << std::endl;
    std::cout << "Steering angle: " << vCar.carSteeringAngle << "(" << + vCar.carSteeringAngle* rad2degree << "°)"<< std::endl;

    cv::Point car(vCar.carPosition.x * pixelPerMeter + border * pixelPerMeter, img.size().height - (vCar.carPosition.y * pixelPerMeter +  border * pixelPerMeter));
    cv::circle(img, car, 5, cv::Scalar::all(0), -1);

    cv::imshow("VirtualPointMoverTest", img); 
    while(cv::waitKey()==0); 
                      // Show our image inside it.
   // }
    /*if(cv::waitKey(10) > 0)
    {
      stopSimulation = true;
    }*/

//    usleep(10000);

  }



//  cv::imshow("Mapgenerator Test", img);

  std::cout << "Finished VirtualPointMover Test.." << std::endl;
  return 0;
}



//  ----------------------------------------------------------------------------
