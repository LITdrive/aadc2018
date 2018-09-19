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

#include "openDriveReader.h"

using namespace ODReader;
openDriveReader::openDriveReader(std::string file,int num)
{
#ifndef WIN32
    //Set to always accept .(dot) as decimal separator
    std::locale::global(std::locale("en_US.UTF8"));
#endif
  //Parameters for MapFormat
  BiDirectional = BIDIRECTIONAL;
  SingleLane = SINGLELANE;
  Altitude = ALTITUDE;
  Scale = MAP_SCALE;
  LoadMap(file,num);
}

openDriveReader::openDriveReader()
{

#ifndef WIN32
  //Set to always accept .(dot) as decimal separator
  std::locale::global(std::locale("en_US.UTF8"));
#endif
  //Parameters for MapFormat
  BiDirectional = BIDIRECTIONAL;
  SingleLane = SINGLELANE;
  Altitude = ALTITUDE;
  Scale = MAP_SCALE;
}

openDriveReader::~openDriveReader()
{
}

bool openDriveReader::ParseText(const char* xml, int num)
{
    try
    {
        //Error when parsing text
        FileReadErr = Doc.Parse(xml);
        //Load doc if there is no error
        if (FileReadErr == 0)
        {
            RoadListOrg.clear();
            RoadList.clear();
            ReadFile(num);
        }
        return true;
    }
    catch (std::string& e)
    {
        return false;
    }

    return false;
}

bool openDriveReader::LoadMap(std::string file,int num)
{
  try
  {
    //Error when parsing text
    FileName = file.c_str();
    FileReadErr= Doc.LoadFile(FileName);
    //Load doc if there is no error
    if(FileReadErr == 0)
    {
      RoadListOrg.clear();
      RoadList.clear();
      ReadFile(num);
    }
    return true;
  }
  catch (std::string& e)
  {
      return false;
  }
  return false;;
}


void openDriveReader::ReadFile(int num)
{
  //Load Root element
  XMLNode *pRoot = Doc.FirstChildElement("OpenDRIVE");
  if(pRoot == NULL)
  {
    return;
  }
  int roadId = 1;
  //Load the first road
  XMLElement *pRoad = pRoot->FirstChildElement("road");
  //Loop for all available roads
  while(pRoad != NULL)
  {
    roadElement curRoadElement;
    curRoadElement.predET ="";
    curRoadElement.succET ="";
    pRoad->QueryIntAttribute("id",&curRoadElement.id);
    pRoad->QueryIntAttribute("junction",&curRoadElement.junction);
    //Loop for all available predecessor
    XMLElement *pPredecessor = pRoad->FirstChildElement("link")->FirstChildElement("predecessor");
    while(pPredecessor != NULL)
    {
      curRoadElement.predCP = pPredecessor->Attribute("contactPoint");
      pPredecessor->QueryIntAttribute("elementId",&curRoadElement.predId);
      curRoadElement.predET = pPredecessor->Attribute("elementType");
      pPredecessor = pPredecessor->NextSiblingElement("predecessor");
    }
    //Loop for all available successor
    XMLElement *pSuccessor = pRoad->FirstChildElement("link")->FirstChildElement("successor");
    while(pSuccessor != NULL)
    {
      curRoadElement.succCP = pSuccessor->Attribute("contactPoint");
      pSuccessor->QueryIntAttribute("elementId",&curRoadElement.succId);
      curRoadElement.succET = pSuccessor->Attribute("elementType");
      pSuccessor = pSuccessor->NextSiblingElement("successor");
    }
    //Loop for all available geometry
    XMLElement *pGeometry = pRoad->FirstChildElement("planView")->FirstChildElement("geometry");
    while(pGeometry != NULL)
    {
      roadGeometry geometry;
      pGeometry->QueryFloatAttribute("hdg",&geometry.hdg);
      pGeometry->QueryFloatAttribute("length",&geometry.length);
      pGeometry->QueryFloatAttribute("s",&geometry.s);
      pGeometry->QueryFloatAttribute("x",&geometry.x);
      pGeometry->QueryFloatAttribute("y",&geometry.y);
      XMLElement *pParamPoly = pGeometry->FirstChildElement("paramPoly3");
      //Loop for all available polynomial
      while(pParamPoly != NULL)
      {
        pParamPoly->QueryFloatAttribute("aU",&geometry.aU);
        pParamPoly->QueryFloatAttribute("aV",&geometry.aV);
        pParamPoly->QueryFloatAttribute("bU",&geometry.bU);
        pParamPoly->QueryFloatAttribute("bV",&geometry.bV);
        pParamPoly->QueryFloatAttribute("cU",&geometry.cU);
        pParamPoly->QueryFloatAttribute("cV",&geometry.cV);
        pParamPoly->QueryFloatAttribute("dU",&geometry.dU);
        pParamPoly->QueryFloatAttribute("dV",&geometry.dV);
        curRoadElement.geometry.push_back(geometry);
        pParamPoly = pParamPoly->NextSiblingElement("paramPoly3");
      }
      pGeometry = pGeometry->NextSiblingElement("geometry");
    }
    //Loop for all available elevationProfile
    XMLElement *pElevation = pRoad->FirstChildElement("elevationProfile")->FirstChildElement("elevation");
    while(pElevation != NULL)
    {
      polynomial elevation;
      pElevation->QueryFloatAttribute("s",&elevation.s);
      pElevation->QueryFloatAttribute("a",&elevation.a);
      pElevation->QueryFloatAttribute("b",&elevation.b);
      pElevation->QueryFloatAttribute("c",&elevation.c);
      pElevation->QueryFloatAttribute("d",&elevation.d);
      curRoadElement.elevation.push_back(elevation);
      pElevation = pElevation->NextSiblingElement("elevation");
    }
    //Loop for all available lane offset
    XMLElement *pLaneOffset = pRoad->FirstChildElement("lanes")->FirstChildElement("laneOffset");
    while(pLaneOffset != NULL)
    {
      polynomial laneOffset;
      pLaneOffset->QueryFloatAttribute("s",&laneOffset.s);
      pLaneOffset->QueryFloatAttribute("a",&laneOffset.a);
      pLaneOffset->QueryFloatAttribute("b",&laneOffset.b);
      pLaneOffset->QueryFloatAttribute("c",&laneOffset.c);
      pLaneOffset->QueryFloatAttribute("d",&laneOffset.d);
      curRoadElement.laneOffset.push_back(laneOffset);
      pLaneOffset = pLaneOffset->NextSiblingElement("laneOffset");
    }

    //Loop for all available lane section
    XMLElement *pLaneSection = pRoad->FirstChildElement("lanes")->FirstChildElement("laneSection");
    while(pLaneSection != NULL)
    {
      //Load left lane
      XMLElement *pLeft = pLaneSection->FirstChildElement("left");
      if(pLeft!=NULL)
      {
        XMLElement *pLeftLane = pLeft->FirstChildElement("lane");
        //Loop for all available lanes
        while(pLeftLane != NULL)
        {
          lane leftLane;
          pLeftLane->QueryIntAttribute("id",&leftLane.id);
          leftLane.level=pLeftLane->Attribute("level");
          leftLane.type=pLeftLane->Attribute("type");
          XMLElement *pLaneWidth = pLeftLane->FirstChildElement("width");
          //Loop for all available lane width
          while(pLaneWidth != NULL)
          {
            polynomial poly;
            pLaneWidth->QueryFloatAttribute("sOffset",&poly.s);
            pLaneWidth->QueryFloatAttribute("a",&poly.a);
            pLaneWidth->QueryFloatAttribute("b",&poly.b);
            pLaneWidth->QueryFloatAttribute("c",&poly.c);
            pLaneWidth->QueryFloatAttribute("d",&poly.d);
            leftLane.width.push_back(poly);
            pLaneWidth = pLaneWidth->NextSiblingElement("width");
          }
          curRoadElement.leftLane.push_back(leftLane);
          pLeftLane = pLeftLane->NextSiblingElement("lane");
        }
      }
      //Load Center lane
      XMLElement *pCenter = pLaneSection->FirstChildElement("center");
      if(pCenter!=NULL)
      {
        //Loop for all available lane center
        XMLElement *pCenterLane = pCenter->FirstChildElement("lane");
        while(pCenterLane != NULL)
        {
          lane centerLane;
          pCenterLane->QueryIntAttribute("id",&centerLane.id);
          centerLane.level=pCenterLane->Attribute("level");
          centerLane.type=pCenterLane->Attribute("type");
          curRoadElement.centerLane.push_back(centerLane);
          pCenterLane = pCenterLane->NextSiblingElement("lane");
        }
      }
      //Load Right lane
      XMLElement *pRight = pLaneSection->FirstChildElement("right");
      //Check if there is a left lane
      if(pRight!=NULL)
      {
        //Loop for all available right lane
        XMLElement *pRightLane = pRight->FirstChildElement("lane");
        while(pRightLane != NULL)
        {
          lane rightLane;
          pRightLane->QueryIntAttribute("id",&rightLane.id);
          rightLane.level=pRightLane->Attribute("level");
          rightLane.type=pRightLane->Attribute("type");
          XMLElement *pLaneWidth = pRightLane->FirstChildElement("width");
          //Loop for all available lane width
          while(pLaneWidth != NULL)
          {
            polynomial poly;
            pLaneWidth->QueryFloatAttribute("sOffset",&poly.s);
            pLaneWidth->QueryFloatAttribute("a",&poly.a);
            pLaneWidth->QueryFloatAttribute("b",&poly.b);
            pLaneWidth->QueryFloatAttribute("c",&poly.c);
            pLaneWidth->QueryFloatAttribute("d",&poly.d);
            rightLane.width.push_back(poly);
            pLaneWidth = pLaneWidth->NextSiblingElement("width");
          }
          curRoadElement.rightLane.push_back(rightLane);
          pRightLane = pRightLane->NextSiblingElement("lane");
        }
      }
      pLaneSection = pLaneSection->NextSiblingElement("laneSection");
    }
    curRoadElement.cost = 99999999999;
    curRoadElement.nextRoad = -1;
    curRoadElement.altitude = Altitude;
    curRoadElement.scale = Scale;
    curRoadElement.laneSplit = 0.0;
    curRoadElement.pointId = roadId *10000;
    roadId++;
    RoadListOrg.push_back(curRoadElement);
    pRoad = pRoad->NextSiblingElement("road");
  }
  RoadList = RoadListOrg;
  //Store map pose and points
  MapPoints = GetRoadPoints(RoadList,num);
  MapElList = GetMapPoints(RoadList,num);
  return;
}

std::vector<std::vector<double> > openDriveReader::GetRoadVector(std::vector<roadElement> el,int num,RoadPointType rType,LaneType lane)
{
  //Get Road points from roadElement vector
  std::vector<Pose3D> vect=GetRoadPoints(el,num,rType,lane);
  //Convert to double
  std::vector<std::vector<double> > mapDouble;
  for (unsigned int i = 0; i < vect.size(); i++)
  {
    Pose3D pose = vect[i];
    //To Euler angles
    Euler e = toEulerianAngle(pose.q);
    std::vector<double> p;
    p.push_back(pose.p.x);
    p.push_back(pose.p.y);
    p.push_back(pose.p.z);
    p.push_back(e.pitch);
    p.push_back(e.roll);
    p.push_back(e.yaw);
    mapDouble.push_back(p);
  }
  return mapDouble;
}

std::vector<Pose3D> openDriveReader::GetRoadPoints(std::vector<roadElement> el,int num,RoadPointType rType,LaneType lane)
{
  std::vector<Pose3D> vect;
  //Load road points road Element one by one
  for (int j=0;j<(int)el.size();j++)
  {
    std::vector<Pose3D> points3d= GetRoadPoints(el[j],num,rType,lane);
    vect.insert(vect.end(),points3d.begin(),points3d.end());
  }
  return vect;
}

std::vector<Pose3D> openDriveReader::GetRoadPoints(roadElement el,int num,RoadPointType rType,LaneType lane)
{
  std::vector<Pose3D> lanePoints;
  std::vector<Pose3D> listTemp;
  std::vector<float> s,laneOffset;
  Pose3D pose;
  float dist = 0.0;
  //Find cumulative distance and points for center lane
  for (unsigned int j = 0; j < el.geometry.size(); j++)
  {
    roadGeometry geo = el.geometry[j];
    float curHdg = geo.hdg;
    dist+=0.0001;
    for (int i = 0; i <num; i++)
    {
      float ds = (float)(i+0.01)/(num-1+0.02);
      //Find points from parametric polynomial in local coordinate
      float u = CubicPoly(geo.aU,geo.bU,geo.cU,geo.dU,ds);
      float v = CubicPoly(geo.aV,geo.bV,geo.cV,geo.dV,ds);
      //Rotate to global coordinate
      float newx = RotateCCWX(u,v,geo.hdg);
      float newy = RotateCCWY(u,v,geo.hdg);
      //Shift to global coordinate
      pose.p.x = (geo.x + newx);
      pose.p.y = (geo.y + newy);
      //Get the heading change from last point to current point
      if(i!=0)
      {
        float x0 = listTemp[j*num+i-1].p.x, y0 = listTemp[j*num+i-1].p.y;
        float x1 = pose.p.x, y1 = pose.p.y;
        dist += sqrt( (y1-y0)*(y1-y0) + (x1-x0)*(x1-x0) );
        curHdg = atan2(y1-y0,x1-x0);
      }
      pose.p.z = el.altitude+getPolynomialValue(el.elevation,dist);
      pose.q = toQuaternion(0,0,curHdg);
      //Store line
      listTemp.push_back(pose);
      s.push_back(dist);
    }
  }
  //Update pose of center lane
  listTemp = UpdatePoseHeading(listTemp);
  //Get lane offset from center lane
  for (unsigned int j = 0; j < listTemp.size(); j++)
  {
    laneOffset.push_back(getPolynomialValue(el.laneOffset,s[j]));
  }
  //Get the lane closest to the center lane first
  std::sort(el.leftLane.begin(),el.leftLane.end());
  //Store previous lane width for cumulative width
  std::vector<float> previousLaneWidth (listTemp.size(),0.0);
  for (unsigned int i = 0; i < el.leftLane.size(); i++)
  {
    std::vector<Pose3D> laneP;
    //Get actual lane point from center point
    for (unsigned int j = 0; j < listTemp.size(); j++)
    {
      float currentWidth = getPolynomialValue(el.leftLane[i].width,s[j]);
      //Shifting factor for first lane
      float factor= 0.5;
      //Do not shift when extracting border point
      if(rType == BORDER_POINTS)
      {
        factor = 0;
      }
      //Store the lane width for first lane
      if(i==0)
      {
        previousLaneWidth[j] =laneOffset[j]-currentWidth*factor;
      }
      //Find the cumulative road width
      float widthj= previousLaneWidth[j]+currentWidth;
      //Store the current width
      previousLaneWidth[j] =widthj;
      //Calculate actual point from center(Reference) point
      Pose3D reference = listTemp[j];
      Pose3D actual = reference;
      Euler e =toEulerianAngle(reference.q);
      float hdg = e.yaw;
      //Do not shift when extracting center points
      if(SingleLane || rType == CENTER_POINTS)
      {
        widthj=laneOffset[j];
      }
      actual.p.x = (reference.p.x+RotateCCWX(0,widthj,hdg)) *el.scale;
      actual.p.y = (reference.p.y+RotateCCWY(0,widthj,hdg)) *el.scale;
      actual.p.z *= el.scale;
      laneP.push_back(actual);
    }
    //Update the heading of lane points
    laneP = UpdatePoseHeading(laneP);
    //Store the lane points
    if(lane != RIGHT_LANE)
    {
      lanePoints.insert(lanePoints.end(),laneP.begin(),laneP.end());
    }
  }
  //Get the lane closest to the center lane first
  std::sort(el.rightLane.begin(),el.rightLane.end());
  //Store previous lane width for cumulative width
  std::fill(previousLaneWidth.begin(), previousLaneWidth.end(), 0.0);
  for (unsigned int i = 0; i < el.rightLane.size(); i++)
  {
    std::vector<Pose3D> laneP;
    //Get actual lane point from center point
    for (unsigned int j = 0; j < listTemp.size(); j++)
    {
      float currentWidth = getPolynomialValue(el.rightLane[i].width,s[j]);
      //Shifting factor for first lane
      float factor = 0.5;
      //Do not shift when extracting border point
      if(rType == BORDER_POINTS)
      {
        factor = 0;
      }
      //Store the lane width for first lane
      if(i==0)
      {
        previousLaneWidth[j] =laneOffset[j]+currentWidth*factor;
      }
      //Find the cumulative road width
      float widthj= previousLaneWidth[j]-currentWidth;
      previousLaneWidth[j] = widthj;
      //Calculate actual point from center(Reference) point
      Pose3D reference = listTemp[j];
      Pose3D actual = reference;
      Euler e =toEulerianAngle(reference.q);
      float hdg = e.yaw;
      //Do not shift when extracting center points
      if( SingleLane || rType == CENTER_POINTS)
      {
        widthj=laneOffset[j];
      }
      actual.p.x = (reference.p.x+RotateCCWX(0,widthj,hdg)) *el.scale;
      actual.p.y = (reference.p.y+RotateCCWY(0,widthj,hdg)) *el.scale;
      actual.p.z *= el.scale;
      laneP.push_back(actual);
    }
    //Update the heading of lane points
    laneP = UpdatePoseHeading(laneP);
    //Store the lane points
    if(lane !=LEFT_LANE)
    {
      lanePoints.insert(lanePoints.end(),laneP.begin(),laneP.end());
    }
  }
  return lanePoints;
}


std::vector<MapElement> openDriveReader::GetMapPoints(std::vector<roadElement> roadlist,int num)
{
	std::vector<MapElement> vect;
	int numRoads = roadlist.size();
  for (int j=0;j<numRoads;j++)
  {
		roadElement el = roadlist[j];
    int laneN = 0;
    std::vector<Pose3D> roadPoints = GetRoadPoints(el,num,DRIVING_POINTS,LEFT_LANE);
    #ifdef DEBUG
    std::cout << "Left lane Id: "<<el.id<<'\n';
    #endif
		for (unsigned int i = 0; i < roadPoints.size(); i++)
		{
      int lanePointId = (i)%num;
      int laneNumber = (i)/num;
			MapElement mapEl;
      mapEl.id = el.pointId+i+laneNumber*num;
      #ifdef DEBUG
      std::cout << "Map point id: "<<mapEl.id<< '\n';
      #endif
	    mapEl.poseV.push_back(roadPoints[i].p.x);
      mapEl.poseV.push_back(roadPoints[i].p.y);
      mapEl.poseV.push_back(roadPoints[i].p.z);
      Euler e = toEulerianAngle(roadPoints[i].q);
      e.yaw = normalizeAngle(e.yaw-M_PI,0);
      mapEl.poseV.push_back(e.pitch);
      mapEl.poseV.push_back(e.roll);
      mapEl.poseV.push_back(e.yaw);
			if(lanePointId==0)
			{
        for (int k = 0; k < numRoads; k++)
        {
          roadElement nextEl = roadlist[k];
          if(el.predId == nextEl.id || el.predId ==nextEl.junction)
          {
            if(nextEl.succId == el.id || nextEl.succId ==el.junction ||
               nextEl.predId == el.id || nextEl.predId ==el.junction  )
            {
              int nLeftLanes = nextEl.leftLane.size();
              int nRightLanes = nextEl.rightLane.size();
              #ifdef DEBUG
              std::cout << "checking left lane " <<nextEl.id<< '\n';
              #endif
              const char *endString= "end",*startString= "start";
              if( (*el.predCP==*endString && nextEl.junction==-1) ||
                  (nextEl.succId==el.id && nextEl.junction!=-1 && nLeftLanes>0))
              {
                for (int m = 0; m < nLeftLanes; m++)
                {
                  int nextPoint = nextEl.pointId+num*m+num-1;
                  mapEl.nodes.push_back(nextPoint);
                  #ifdef DEBUG
                  std::cout << "Next id: "<<nextPoint<< '\n';
                  #endif
                }
              }
              if( (*el.predCP==*startString && nextEl.junction==-1) ||
                 (nextEl.predId==el.id && nextEl.junction!=-1 && nRightLanes>0) )
              {
                for (int m = nLeftLanes; m < nLeftLanes+nRightLanes; m++)
                {
                  int nextPoint = nextEl.pointId+num*m;
                  mapEl.nodes.push_back(nextPoint);
                  #ifdef DEBUG
                  std::cout << "Next id: "<<nextPoint<< '\n';
                  #endif
                }
              }
            }
          }
        }
			}
			else
			{
				mapEl.nodes.push_back(mapEl.id-1);
        #ifdef DEBUG
        std::cout << "Next id: "<<mapEl.id-1<< '\n';
        #endif
			}
	    vect.push_back(mapEl);
	  }
    laneN = roadPoints.size()/num;
    roadPoints = GetRoadPoints(el,num,DRIVING_POINTS,RIGHT_LANE);
    #ifdef DEBUG
    std::cout << "Right Lane: "<<el.id<<'\n';
    #endif
    for (unsigned int i = 0; i < roadPoints.size(); i++)
    {
      int lanePointId = i%num;
      int laneNumber = laneN+i/num;
      //std::cout << "Lane Id: "<<lanePointId<<" laneNumber: "<<laneNumber<< '\n';
      MapElement mapEl;
      mapEl.id = el.pointId+i+laneNumber*num;
      #ifdef DEBUG
      std::cout << "Map point id: "<<mapEl.id<< '\n';
      #endif
      mapEl.poseV.push_back(roadPoints[i].p.x);
      mapEl.poseV.push_back(roadPoints[i].p.y);
      mapEl.poseV.push_back(roadPoints[i].p.z);
      Euler e = toEulerianAngle(roadPoints[i].q);
      mapEl.poseV.push_back(e.pitch);
      mapEl.poseV.push_back(e.roll);
      mapEl.poseV.push_back(e.yaw);
	    if(lanePointId==(num-1))
      {
        for (int k = 0; k < numRoads; k++)
        {
          roadElement nextEl = roadlist[k];
          if(el.succId == nextEl.id || el.succId ==nextEl.junction)
          {
            if(nextEl.succId == el.id || nextEl.succId ==el.junction ||
              nextEl.predId == el.id || nextEl.predId ==el.junction  )
            {
              int nLeftLanes = nextEl.leftLane.size();
              int nRightLanes = nextEl.rightLane.size();
              #ifdef DEBUG
              std::cout << "checking right lane " <<nextEl.id<< '\n';
              #endif
              const char *endString= "end",*startString= "start";
              if((*el.succCP==*endString && nextEl.junction==-1) ||
              (nextEl.succId==el.id && nextEl.junction!=-1 && nLeftLanes>0) )
              {
                for (int m = 0; m < nLeftLanes; m++)
                {
                  int nextPoint = nextEl.pointId+num*m+num-1;
                  mapEl.nodes.push_back(nextPoint);
                  #ifdef DEBUG
                  std::cout << "Next id: "<<nextPoint<< '\n';
                  #endif
                }
              }
              if((*el.succCP==*startString && nextEl.junction==-1) ||
              (nextEl.predId==el.id && nextEl.junction!=-1 && nRightLanes>0) )
              {
                for (int m = nLeftLanes; m < nLeftLanes+nRightLanes; m++)
                {
                  int nextPoint = nextEl.pointId+num*m;
                  mapEl.nodes.push_back(nextPoint);
                  #ifdef DEBUG
                  std::cout << "Next id: "<<nextPoint<< '\n';
                  #endif
                }
              }
            }
          }
        }
      }
      else
      {
        mapEl.nodes.push_back(mapEl.id+1);
        #ifdef DEBUG
        std::cout << "Next id: "<<mapEl.id+1<< '\n';
        #endif
      }
      vect.push_back(mapEl);
    }
  }
  return vect;
}

std::vector<Pose3D> openDriveReader::UpdatePoseHeading(std::vector<Pose3D> path)
{
  std::vector<Pose3D> list;
  Pose3D pose;
  int pathLength = path.size();
  float pitch=0.0,roll=0.0,yaw=0.0;
  //Calculate pitch, roll, yaw from previous points
  for (int i = 0; i < pathLength; i++)
  {
    if(i+1!=pathLength)
    {
      float x0 = path[i].p.x, y0 = path[i].p.y,z0 = path[i].p.z;
      float x1 = path[i+1].p.x, y1 = path[i+1].p.y,z1 = path[i+1].p.z;
      pitch = atan2( z1-z0 , x1-x0);
      pitch = 0;
      roll = atan2( z1-z0 , y1-y0);
      roll = 0;
      yaw = atan2( y1-y0 , x1-x0);
    }
    pose = path[i];
    pose.q = toQuaternion(pitch,roll,yaw);
    list.push_back(pose);
  }
  return list;
}

float openDriveReader::getPolynomialValue(std::vector<polynomial> prof,float ds)
{
  for (unsigned int i = 0; i < prof.size(); i++)
  {
    if(i+1<prof.size())
    {
      if(ds>prof[i].s && ds<prof[i+1].s)
      {
        return CubicPoly(prof[i].a,prof[i].b,prof[i].c,prof[i].d,ds-prof[i].s);
      }
    }
    else
    {
      return CubicPoly(prof[i].a,prof[i].b,prof[i].c,prof[i].d,ds-prof[i].s);
    }
  }
  return 0.0;
}

float openDriveReader::CubicPoly(float a1,float b1,float c1, float d1, float ds)
{

    return (a1+b1*ds+c1*pow(ds,2.0)+d1*pow(ds,3.0));
}

float openDriveReader::EuclideanDistance(Pose3D pose1,Pose3D pose2)
{
  float x0 = pose1.p.x;
  float y0 = pose1.p.y;
  float z0 = pose1.p.z;
  float x1 = pose2.p.x;
  float y1 = pose2.p.y;
  float z1 = pose2.p.z;
  float d  = sqrt( (x0-x1)*(x0-x1) + (y0-y1)*(y0-y1) + (z0-z1)*(z0-z1) );
  return d;
}

float openDriveReader::normalizeAngle(float alpha, float center)
{
    return mod(alpha-center+M_PI, 2.0*M_PI) + center-M_PI;
}

float openDriveReader::mod(float x, float y)
{
    float r;
    float b_x;
    if (y == floor(y))
    {
        return x - floor(x / y) * y;
    }
    else
    {
        r = x / y;
        if (r < 0.0)
        {
            b_x = ceil(r - 0.5);
        }
        else
        {
            b_x = floor(r + 0.5);
        }
        if (fabs(r - b_x) <= 2.2204460492503131E-16 * fabs(r))
        {
            return 0.0;
        }
        else
        {
            return (r - floor(r)) * y;
        }
    }
}


Quaternion openDriveReader::toQuaternion(double pitch, double roll, double yaw)
{
	Quaternion q;
	double t0 = cos(yaw * 0.5);
	double t1 = sin(yaw * 0.5);
	double t2 = cos(roll * 0.5);
	double t3 = sin(roll * 0.5);
	double t4 = cos(pitch * 0.5);
	double t5 = sin(pitch * 0.5);

	q.w = t0 * t2 * t4 + t1 * t3 * t5;
	q.x = t0 * t3 * t4 - t1 * t2 * t5;
	q.y = t0 * t2 * t5 + t1 * t3 * t4;
	q.z = t1 * t2 * t4 - t0 * t3 * t5;
	return q;
}


Euler openDriveReader::toEulerianAngle(Quaternion q)
{
	Euler a;
	double ysqr = q.y * q.y;

	// roll (x-axis rotation)
	double t0 = +2.0 * (q.w * q.x + q.y * q.z);
	double t1 = +1.0 - 2.0 * (q.x * q.x + ysqr);
	a.roll = atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (q.w * q.y - q.z * q.x);
	t2 = ((t2 > 1.0) ? 1.0 : t2);
	t2 = ((t2 < -1.0) ? -1.0 : t2);
	a.pitch = asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (q.w * q.z + q.x * q.y);
	double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
	a.yaw = atan2(t3, t4);
	return a;
}




float openDriveReader::RotateCCWX(float u2,float v2, float hdg2)
{
    return (u2*cos(hdg2)-v2*sin(hdg2));
}

float openDriveReader::RotateCCWY(float u1,float v1, float hdg1)
{
    return (u1*sin(hdg1)+v1*cos(hdg1));
}

float openDriveReader::RotateCWX(float u2,float v2, float hdg2)
{
    return (u2*cos(hdg2)+v2*sin(hdg2));
}

float openDriveReader::RotateCWY(float u1,float v1, float hdg1)
{
    return (-u1*sin(hdg1)+v1*cos(hdg1));
}
