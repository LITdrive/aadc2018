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
#include "LITD_VirtualCar.h"


#define PIXEL_PER_METER 80
#define DTIME 0.1
#define STRAIGHT_SPEED 0.5



int main()
{
  LITD_Map map;
  LITD_VirtualCar vCar;
  
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

  

  std::cout << "Car: " << std::endl;
  std::cout << "x: " << vCar.carPosition.x << std::endl;
  std::cout << "y: " << vCar.carPosition.y << std::endl;

  bool running=true;


  LITD_VirtualPoint vp;
  
  while(running) {
    std::cout << "Loop start: x=" << vCar.carPosition.x << " y=" << vCar.carPosition.y << " h=" << vCar.carPosition.h << std::endl;
    
    

    //cv::Point street(mapImg.size().width/2+PIXEL_PER_METER*vp.x, mapImg.size().height/2 - PIXEL_PER_METER*vp.y);
    //cv::circle(mapImg, street, 5, cv::Scalar::all(0));

    
    
    vCar.updateStep( DTIME);
  //C++: void circle(InputOutputArray img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=LINE_8, int shift=0 )
  //C++: void line  (InputOutputArray img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=LINE_8, int shift=0 )
    
    cv::Point car(mapImg.size().width/2+PIXEL_PER_METER*vCar.carPosition.x, mapImg.size().height/2 - PIXEL_PER_METER*vCar.carPosition.y);
    cv::circle(mapImg, car, 2, cv::Scalar::all(0), -1);

    cv::Point backPoint(mapImg.size().width/2+PIXEL_PER_METER*vCar.carBackPosition.x, mapImg.size().height/2 - PIXEL_PER_METER*vCar.carBackPosition.y);
    cv::line(mapImg,car, backPoint, cv::Scalar::all(0), 1);

    cv::imshow("VirtualPointMoverTest", mapImg);                   // Show our image inside it.

    usleep(10000);

    while(cv::waitKey() == 0) {
      //running=false;
    }
  }
  while(1) {
    sleep(100);
  }
}
//  ----------------------------------------------------------------------------

void printCar(){
  
}

