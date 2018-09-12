#include <math.h>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "LITD_VirtualPoint.h"
#include "LITD_Map.h"


#define PIXEL_PER_METER 100

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
  /*
  double pixelPerMeter = 100.0;
  double border = 1.0;
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
  cv::Mat mapImg(1000, 1000, CV_8UC3, cv::Scalar::all(255));
  mapGenerator.plot(mapImg, pixelPerMeter);

  VirtualPointMover vpMover;
  vpMover.setMapElements(mapGenerator.getMapElements());

  bool stopSimulation = false;
  double dtime = 0.02;
  cv::namedWindow( "VirtualPointMoverTest", cv::WINDOW_AUTOSIZE );// Create a window for display.

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
    cv::Point car(vp.x * pixelPerMeter + border * pixelPerMeter, img.size().height - (vp.y * pixelPerMeter +  border * pixelPerMeter));
    cv::circle(img, car, 5, cv::Scalar::all(0));

    cv::imshow("VirtualPointMoverTest", img);                   // Show our image inside it.

    if(cv::waitKey(10) > 0)
    {
      stopSimulation = true;
    }

//    usleep(10000);

  }



//  cv::imshow("Mapgenerator Test", img);

  std::cout << "Finished VirtualPointMover Test.." << std::endl;
  return 0;
}

*/

  LITD_Map map;
  //Vertical straight between x 0->2
  if(map.addStraightElement(0.0, 2.0, -0.5, 0.5, 0.0, false)!=MAP_ENOERR) {
    std::cout << "Error adding straight element 0/0->2/0" << std::endl;
  }
  if(map.addCurveElement(2.0, 3.5, -0.5, 1.0, CURVE_CORNER_UL, 1.0)!=MAP_ENOERR) {
    std::cout << "Error adding curve element 2/0->3/1" << std::endl;
  }
  if(map.addStraightElement(2.5, 3.5, 1.0, 3.0, 3.0, true)!=MAP_ENOERR) {
    std::cout << "Error adding straight element 3/1->3/3" << std::endl;
  }  
  if(map.addCurveElement(2.0, 3.5, 3.0, 4.5, CURVE_CORNER_LL, 1.0)!=MAP_ENOERR) {
    std::cout << "Error adding curve element 3/3->2/4" << std::endl;
  }
  if(map.addStraightElement(0.0, 2.0, 3.5, 4.5, 4.0, false)!=MAP_ENOERR) {
    std::cout << "Error adding straight element 2/4->0/4" << std::endl;
  }
  if(map.addCurveElement(-1.5, 0.0, 3.0, 4.5, CURVE_CORNER_LR, 1.0)!=MAP_ENOERR) {
    std::cout << "Error adding curve element 0/4->-1/3" << std::endl;
  }
  if(map.addStraightElement(-1.5, -0.5, 1.0, 3.0, -1.0, true)!=MAP_ENOERR) {
    std::cout << "Error adding straight element -1/3->-1/1" << std::endl;
  }  
  if(map.addCurveElement(-1.5, 0.0, -0.5, 1.0, CURVE_CORNER_UR, 1.0)!=MAP_ENOERR) {
    std::cout << "Error adding curve element -1/1->0/0" << std::endl;
  }
  std::cout << "Map generation complete!" << std::endl;
  map.selectNextManeuver(aadc::jury::maneuver::maneuver_straight);


  cv::Mat mapImg(800, 800, CV_8UC3, cv::Scalar::all(255));

  cv::namedWindow( "VirtualPointMoverTest", cv::WINDOW_AUTOSIZE );// Create a window for display.

  bool running=true;

  //Point2d cp(0.0,0.0);

  LITD_VirtualPoint cp(0.0,0.0,0.0,0.0);

  double carSpeed=0.1;

  LITD_VirtualPoint vp, vp_old;

  while(running) {
    std::cout << "Loop start: x=" << cp.x << " y=" << cp.y << " h=" << cp.h << std::endl;
    double offset = map.getPointOffset(cp);
    vp_old=vp;
    vp=map.getNormalPoint(cp);
    LITD_map_error_t err = map.getMapState();
    if(err!=MAP_ENOERR) {
      std::cout << "Error while generating new point: " << map.strerr(err) << std::endl;
      running=false;
    }
    std::cout << "Got virtual point: x=" << vp.x << " y=" << vp.y << " h=" << vp.h << "Offset=" << offset << std::endl;

    cv::Point car(mapImg.size().width/2+PIXEL_PER_METER*vp.x, mapImg.size().height/2 - PIXEL_PER_METER*vp.y);
    cv::circle(mapImg, car, 5, cv::Scalar::all(0));

    cv::imshow("VirtualPointMoverTest", mapImg);                   // Show our image inside it.

    if(vp.x>=vp_old.x) {
      cp.x=vp.x+0.1;
    } else {
      cp.x=vp.x-0.1;
    }
    if(vp.y>=vp_old.y) {
      cp.y=vp.y+0.1;
    } else {
      cp.y=vp.y-0.1;
    }
    cp.h=vp.h;

    usleep(10000);

    if(cv::waitKey(10)>0) {
      running=false;
    }
  }
  sleep(100);
}
//  ----------------------------------------------------------------------------
