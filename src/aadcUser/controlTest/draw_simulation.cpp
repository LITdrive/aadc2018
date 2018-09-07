#include <math.h>
#include <opencv2/opencv.hpp>
#include "constants.h"
#include "draw_simulation.h"


using namespace std;
using namespace cv;

void getRotationMatrix(double angle, Mat2d& out){
    out = Mat2d(2, 2, CV_64FC1);
    out[0][0] = cos(angle);
    out[1][0] = sin(angle);
    out[0][1] = -sin(angle);
    out[1][1] = cos(angle);
}
int DrawSimulation::m2px(double m){
    return int(m_pixelPerMeter*m); //Convert meter to pixel coordinates
}

void draw_point(Mat& img, Point p, int radius = 2, CvScalar color = CV_RGB(100, 100, 100)){
    circle(img, p, radius, color, -1);
}

void draw_point(Mat& img, int x, int y, int radius = 2, CvScalar color = CV_RGB(100, 100, 100)){
    draw_point(img, Point2i(x,y), radius, color);
}

void DrawSimulation::drawMap(Mat& img){
    namedWindow("Display window", WINDOW_AUTOSIZE);// Create a window for display.

    imshow("Display window", img);
}

void DrawSimulation::drawCar(Mat& img, double x, double y, double heading, double steerAngle){
    // Draw Center point of car
    draw_point(img, m2px(x), m2px(y));
    // Calculate car roation in global space
    Mat2d carRot;
    getRotationMatrix(steerAngle, carRot);
    vector<cv::Point2d> outline_rotated;
    outline_rotated.push_back(Point2d(carRot*Mat(m_offsetOutlineFrontLeft)));
    outline_rotated.push_back(carRot*Mat(m_offsetOutlineFrontRight));
    outline_rotated.push_back(carRot*Mat(m_offsetOutlineRearLeft));
    outline_rotated.push_back(carRot*Mat(m_offsetOutlineRearRight));
    // Draw outline of car

}

DrawSimulation::DrawSimulation() {
    //Wheel Points
    Point offsetFrontAxle = cv::Point2d(AXLE_LENGTH, 0);
    Point offsetWheelCenter = cv::Point2d(0, AXLE_WIDTH/2);
    m_offsetWheelRearLeft = offsetWheelCenter;
    m_offsetWheelRearRight = -offsetWheelCenter;
    m_offsetWheelFrontLeft = m_offsetWheelRearLeft + offsetFrontAxle;
    m_offsetWheelFrontRight = m_offsetWheelRearRight + offsetFrontAxle;
    //Car outline Points
    Point offsetRear = cv::Point2d(-BACKTOWHEEL, 0);
    Point offsetFrontRear = cv::Point2d(CAR_LENGTH, 0);
    Point offsetSide = cv::Point2d(0, CAR_WIDTH/2);
    m_offsetOutlineRearLeft = offsetRear + offsetSide;
    m_offsetOutlineRearRight = offsetRear - offsetSide;
    m_offsetOutlineFrontLeft = m_offsetOutlineFrontLeft + offsetFrontRear;
    m_offsetOutlineFrontRight = m_offsetOutlineRearRight + offsetFrontRear;
}

void DrawSimulation::setMap(MapElements mapElements) {
    m_elems = mapElements;
}


/**
outline = np.matrix([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                     [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

fr_wheel = np.matrix([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                      [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

rr_wheel = np.copy(fr_wheel)

fl_wheel = np.copy(fr_wheel)
fl_wheel[1, :] *= -1
rl_wheel = np.copy(rr_wheel)
rl_wheel[1, :] *= -1

Rot1 = np.matrix([[math.cos(yaw), math.sin(yaw)],
                  [-math.sin(yaw), math.cos(yaw)]])
Rot2 = np.matrix([[math.cos(steer), math.sin(steer)],
                  [-math.sin(steer), math.cos(steer)]])

fr_wheel = (fr_wheel.T * Rot2).T
fl_wheel = (fl_wheel.T * Rot2).T
fr_wheel[0, :] += WB
fl_wheel[0, :] += WB

fr_wheel = (fr_wheel.T * Rot1).T
fl_wheel = (fl_wheel.T * Rot1).T

outline = (outline.T * Rot1).T
rr_wheel = (rr_wheel.T * Rot1).T
rl_wheel = (rl_wheel.T * Rot1).T

outline[0, :] += x
outline[1, :] += y
fr_wheel[0, :] += x
fr_wheel[1, :] += y
rr_wheel[0, :] += x
rr_wheel[1, :] += y
fl_wheel[0, :] += x
fl_wheel[1, :] += y
rl_wheel[0, :] += x
rl_wheel[1, :] += y

plt.plot(np.array(outline[0, :]).flatten(),
         np.array(outline[1, :]).flatten(), truckcolor)
plt.plot(np.array(fr_wheel[0, :]).flatten(),
         np.array(fr_wheel[1, :]).flatten(), truckcolor)
plt.plot(np.array(rr_wheel[0, :]).flatten(),
         np.array(rr_wheel[1, :]).flatten(), truckcolor)
plt.plot(np.array(fl_wheel[0, :]).flatten(),
         np.array(fl_wheel[1, :]).flatten(), truckcolor)
plt.plot(np.array(rl_wheel[0, :]).flatten(),
         np.array(rl_wheel[1, :]).flatten(), truckcolor)
plt.plot(x, y, "*")
**/
