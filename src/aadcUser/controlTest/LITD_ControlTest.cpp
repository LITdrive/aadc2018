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


#define PIXEL_PER_METER 100
#define DTIME 0.1



int main()
{
  LITD_Map map;
  //Vertical straight between x 0->2
  /*
  if(map.addStraightElement(0.0, 2.0, -0.5, 0.5, 0.0, false)!=MAP_ENOERR) {
    std::cout << "Error adding straight element 0/0->2/0" << std::endl;
  }
  */
  if(map.addDoubleStraightElement(0.0, 2.0, -0.5, 0.5, -0.2, 0.2, false)!=MAP_ENOERR) {
    std::cout << "Error adding straight element 0/0->2/0" << std::endl;
  }
  /*
  if(map.addCurveElement(2.0, 3.5, -0.5, 1.0, CURVE_CORNER_UL, 1.0)!=MAP_ENOERR) {
    std::cout << "Error adding curve element 2/0->3/1" << std::endl;
  }
  */
  if(map.addDoubleCurveElement(2.0, 3.5, -0.5, 1.0, CURVE_CORNER_UL, 0.8, 1.2)!=MAP_ENOERR) {
    std::cout << "Error adding curve element 2/0->3/1" << std::endl;
  }
  /*
  if(map.addStraightElement(2.5, 3.5, 1.0, 3.0, 3.0, true)!=MAP_ENOERR) {
    std::cout << "Error adding straight element 3/1->3/3" << std::endl;
  }
  */

  if(map.addDoubleStraightElement(2.5, 3.5, 1.0, 3.0, 2.8, 3.2, true)!=MAP_ENOERR) {
    std::cout << "Error adding straight element 3/1->3/3" << std::endl;
  }
  /*
  if(map.addCurveElement(2.0, 3.5, 3.0, 4.5, CURVE_CORNER_LL, 1.0)!=MAP_ENOERR) {
    std::cout << "Error adding curve element 3/3->2/4" << std::endl;
  }
  */
  if(map.addDoubleCurveElement(2.0, 3.5, 3.0, 4.5, CURVE_CORNER_LL, 0.8, 1.2)!=MAP_ENOERR) {
    std::cout << "Error adding curve element 3/3->2/4" << std::endl;
  }
  /*
  if(map.addStraightElement(0.0, 2.0, 3.5, 4.5, 4.0, false)!=MAP_ENOERR) {
    std::cout << "Error adding straight element 2/4->0/4" << std::endl;
  }
  */
  if(map.addDoubleStraightElement(0.0, 2.0, 3.5, 4.5, 3.8, 4.2, false)!=MAP_ENOERR) {
    std::cout << "Error adding straight element 2/4->0/4" << std::endl;
  }
  /*
  if(map.addCurveElement(-1.5, 0.0, 3.0, 4.5, CURVE_CORNER_LR, 1.0)!=MAP_ENOERR) {
    std::cout << "Error adding curve element 0/4->-1/3" << std::endl;
  }
  */
  if(map.addDoubleCurveElement(-1.5, 0.0, 3.0, 4.5, CURVE_CORNER_LR, 0.8, 1.2)!=MAP_ENOERR) {
    std::cout << "Error adding curve element 0/4->-1/3" << std::endl;
  }
  /*
  if(map.addStraightElement(-1.5, -0.5, 1.0, 3.0, -1.0, true)!=MAP_ENOERR) {
    std::cout << "Error adding straight element -1/3->-1/1" << std::endl;
  }  
  */
  if(map.addDoubleStraightElement(-1.5, -0.5, 1.0, 3.0, -1.2, -0.8, true)!=MAP_ENOERR) {
    std::cout << "Error adding straight element -1/3->-1/1" << std::endl;
  }
  /*
  if(map.addCurveElement(-1.5, 0.0, -0.5, 1.0, CURVE_CORNER_UR, 1.0)!=MAP_ENOERR) {
    std::cout << "Error adding curve element -1/1->0/0" << std::endl;
  }
  */
  if(map.addDoubleCurveElement(-1.5, 0.0, -0.5, 1.0, CURVE_CORNER_UR, 0.8, 1.2)!=MAP_ENOERR) {
    std::cout << "Error adding curve element -1/1->0/0" << std::endl;
  }
  std::cout << "Map generation complete!" << std::endl;
  map.selectNextManeuver(aadc::jury::maneuver::maneuver_straight);


  cv::Mat mapImg(800, 800, CV_8UC3, cv::Scalar::all(255));

  cv::namedWindow( "VirtualPointMoverTest", cv::WINDOW_AUTOSIZE );// Create a window for display.

  LITD_VirtualCar vCar;

  std::cout << "Car: " << std::endl;
  std::cout << "x: " << vCar.carPosition.x << std::endl;
  std::cout << "y: " << vCar.carPosition.y << std::endl;

  bool running=true;


  //LITD_VirtualPoint vp(1.0,0.0,0.0,0.0), vp_old, vp_help(0.9,0.0,0.0,0.0);
  LITD_VirtualPoint vp(1.0,0.0,0.0,M_PI), vp_old, vp_help(1.1,0.0,0.0,M_PI);
  while(running) {
    std::cout << "Loop start: x=" << vp.x << " y=" << vp.y << " h=" << vp.h << std::endl;
    vp_old=vp_help;
    vp=map.getNormalPoint(vp);
    LITD_map_error_t err = map.getMapState();
    if(err!=MAP_ENOERR) {
      std::cout << "Error while generating new point: " << map.strerr(err) << std::endl;
      running=false;
      break;
    }
    std::cout << "Got virtual point: x=" << vp.x << " y=" << vp.y << " h=" << vp.h << std::endl;

    cv::Point street(mapImg.size().width/2+PIXEL_PER_METER*vp.x, mapImg.size().height/2 - PIXEL_PER_METER*vp.y);
    cv::circle(mapImg, street, 5, cv::Scalar::all(0));
    cv::Point angle(mapImg.size().width/2+PIXEL_PER_METER*vp.x + PIXEL_PER_METER/4*cos(vp.h), mapImg.size().height/2 - PIXEL_PER_METER*vp.y - PIXEL_PER_METER/4*sin(vp.h));
    cv::arrowedLine(mapImg, street, angle , cv::Scalar::all(0));

    //cv::Point car(mapImg.size().width/2+PIXEL_PER_METER*vCar.carPosition.x, mapImg.size().height/2 - PIXEL_PER_METER*vCar.carPosition.y);
    //cv::circle(mapImg, car, 2, cv::Scalar::all(0), -1);

    //vCar.updateStep(vp, DTIME);
    vp_help=vp;
    if(vp.x<=vp_old.x) {
      vp.x-=0.1;
    } else {
      vp.x += 0.1;
    }
    if(vp.y<=vp_old.y) {
      vp.y-=0.1;
    } else {
      vp.y += 0.1;
    }

    cv::imshow("VirtualPointMoverTest", mapImg);                   // Show our image inside it.

    usleep(50000);

    if(cv::waitKey(10)>0) {
      running=false;
    }
  }
  while(1) {
    sleep(100);
  }
}
//  ----------------------------------------------------------------------------
