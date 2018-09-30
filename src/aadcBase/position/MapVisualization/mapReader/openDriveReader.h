/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/


/*********************************************************************
* This code was provided by HERE
*
* *******************************************************************/

#ifndef _OPEN_DRIVE_PLANNER_
#define _OPEN_DRIVE_PLANNER_
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <algorithm>
#include "tinyxml2.h"
#ifdef DEBUG
#include<iostream>
#endif
#include <locale>


#define BIDIRECTIONAL false
#define SINGLELANE false
#define ALTITUDE 0.0
#define MAP_SCALE 0.125
using namespace tinyxml2;

namespace ODReader{
  struct Quaternion{
	  float x;
	  float y;
	  float z;
	  float w;
	};
	struct Euler{
	  float pitch;
	  float roll;
	  float yaw;
	};
	struct Point{
	  float x;
	  float y;
	  float z;
	};
	struct Pose3D{
	  Point p;
	  Quaternion q;
	};
	struct roadGeometry{
	  float x,y,hdg,s,length;
	  float aU,aV,bU,bV,cU,cV,dU,dV;
	};
  struct polynomial{
	  float a,b,c,d,s;
	};
	struct lane
  {
	  int id;
	  const char *type,*level;
    std::vector<polynomial> width;
    bool operator<(const lane& a) const
    {
        return fabs(id) < fabs(a.id);
    }
	};
	struct node{
	  int id;
	  const char* contactPoint;
	};
	struct roadElement{
	  int id,junction,pointId;
	  std::vector<roadGeometry>  geometry;
    std::vector<polynomial> elevation;
    std::vector<polynomial> laneOffset;
    std::vector<lane> leftLane;
    std::vector<lane> rightLane;
    std::vector<lane> centerLane;
	  std::vector<node>  nodes;
	  const char * predCP,*succCP,*predET,*succET;
	  int predId,succId;
	  int nextRoad;
	  float cost;
	  float altitude;
	  float scale;
    float laneSplit;
	};

	struct MapElement{
    int id;
    std::vector<double> poseV;
    Pose3D pose;
	  std::vector<int> nodes;
	};
  enum RoadPointType{
    CENTER_POINTS=0,
    DRIVING_POINTS=1,
    BORDER_POINTS=2
  };
  enum LaneType{
    LEFT_LANE=0,
    RIGHT_LANE=1,
    ALL_LANES=2
  };

  class openDriveReader{
  public:
    openDriveReader();
    ~openDriveReader();
    /**
    Load map from OpenDRIVE file using FileName
    **/
    openDriveReader(std::string file,int num =10);
    /**
    Load map from OpenDRIVE file using FileName
    **/
    bool LoadMap(std::string file,int num = 10);


      /**
    Load map from OpenDRIVE format from Text
    **/
    bool ParseText(const char* xml, int num = 10);
    /**
    Reload map from stored OpenDRIVE file name.
    **/
    void ReadFile(int num = 10);
    /**
    Get Pose of points from OpenDrive road element.
    **/
    std::vector<Pose3D> GetRoadPoints(roadElement el,int num=10,RoadPointType rType = DRIVING_POINTS,LaneType lane = ALL_LANES);
    /**
    Get Pose of points from a vector of OpenDRIVE road element.
    **/
    std::vector<Pose3D> GetRoadPoints(std::vector<roadElement> el,int num=10,RoadPointType rType = DRIVING_POINTS,LaneType lane = ALL_LANES);
    /**
    Get pose of points from a road vector as a vector of double(x,y,z,pitch,roll,yaw)
    **/
    std::vector<std::vector<double> > GetRoadVector(std::vector<roadElement> el,int num=10,RoadPointType rType = DRIVING_POINTS,LaneType lane = ALL_LANES);
    std::vector<MapElement> MapElList;

    float Altitude,Scale;
    bool BiDirectional,SingleLane;
    /**
    Error message in reading file
    **/
    XMLError FileReadErr;
    /**
    List of roads/ Modified and Unmodified.
    **/
    std::vector<roadElement> RoadListOrg,RoadList;
  private:
    /**
    XML documentation
    **/
    tinyxml2::XMLDocument Doc;
    /**
    List of all the points in the map
    **/
    std::vector<Pose3D> MapPoints;
    /**
    Name of xml document
    **/
    const char* FileName;
    /**
    Get the value of parameteric polynomial from the function
    **/
    float getPolynomialValue(std::vector<polynomial> prof,float ds);
    /**
    Convert euler angles to Quaternion angles
    **/
    Quaternion toQuaternion(double pitch, double roll, double yaw);
    /**
    Convert Quaternion angle to Euler angles
    **/
    Euler toEulerianAngle(Quaternion q);
    /**
    Cubic Parametric Polynomial
    **/
    float CubicPoly(float a1,float b1,float c1, float d1, float ds);
    /**
    Rotation matrix Counter Clockwise X
    **/
    float RotateCCWX(float u2,float v2, float hdg2);
    /**
    Rotation matrix Counter Clockwise Y
    **/
    float RotateCCWY(float u1,float v1, float hdg1);
    /**
    Rotation matrix Clockwise X
    **/
    float RotateCWX(float u2,float v2, float hdg2);
    /**
    Rotation matrix Clockwise Y
    **/
    float RotateCWY(float u1,float v1, float hdg1);
    /**
    Euclidean Distance between two poses
    **/
    float EuclideanDistance(Pose3D pose1,Pose3D pose2);
    /**
    Calculates and Updates the heading of all the points in 2-Dimensions(only yaw)
    **/
    std::vector<Pose3D> UpdatePoseHeading(std::vector<Pose3D> path);
    /**
    Get Map points vector of all points from the road. Creates unique id to all the points and list the neighbour nodes(connections)
    **/
    std::vector<MapElement> GetMapPoints(std::vector<roadElement> roadlist,int num = 10);
    /**
    Normalize angle to centerLane
    **/
    float normalizeAngle(float alpha, float center);
    /**
    Find modulus
    **/
    float mod(float x, float y);
};

}
#endif
