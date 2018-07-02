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
openDriveReader::openDriveReader(std::string file)
{

    BiDirectional = BIDIRECTIONAL;
    SingleLane = SINGLELANE;
    Altitude = ALTITUDE;
    Scale = MAP_SCALE;
    LaneWidth = (float)LANEWIDTH;
    LaneWidth = LaneWidth;
    LoadMap(file);
}

openDriveReader::openDriveReader()
{
    BiDirectional = BIDIRECTIONAL;
    SingleLane = SINGLELANE;
    Altitude = ALTITUDE;
    Scale = MAP_SCALE;
    LaneWidth = (float)LANEWIDTH;
}
openDriveReader::~openDriveReader()
{

}


void openDriveReader::LoadMap(std::string file, int num)
{
    
    FileName = file.c_str();

#ifndef WIN32
    std::locale::global(std::locale("en_US.utf8"));
#endif WIN32

    FileReadErr = Doc.LoadFile(FileName);
    if (FileReadErr == 0)
    {
        //Read Vector
        RoadListOrg.clear();
        RoadList.clear();
        ReadFile(num);
    }
    return;
}

void openDriveReader::ReadFile(int num)
{

    
    XMLNode *pRoot = Doc.FirstChildElement("OpenDRIVE");
    if (pRoot == NULL)
    {
        return;
    }
    XMLElement *pRoad = pRoot->FirstChildElement("road");
    while (pRoad != NULL)
    {
        roadElement curRoadElement;
        curRoadElement.predET = "";
        curRoadElement.succET = "";

        pRoad->QueryIntAttribute("id", &curRoadElement.id);
        pRoad->QueryIntAttribute("junction", &curRoadElement.junction);
        XMLElement *pPredecessor = pRoad->FirstChildElement("link")->FirstChildElement("predecessor");
        while (pPredecessor != NULL)
        {
            curRoadElement.predCP = pPredecessor->Attribute("contactPoint");
            pPredecessor->QueryIntAttribute("elementId", &curRoadElement.predId);
            curRoadElement.predET = pPredecessor->Attribute("elementType");
            pPredecessor = pPredecessor->NextSiblingElement("predecessor");
        }
        XMLElement *pSuccessor = pRoad->FirstChildElement("link")->FirstChildElement("successor");
        while (pSuccessor != NULL)
        {
            curRoadElement.succCP = pSuccessor->Attribute("contactPoint");
            pSuccessor->QueryIntAttribute("elementId", &curRoadElement.succId);
            curRoadElement.succET = pSuccessor->Attribute("elementType");
            pSuccessor = pSuccessor->NextSiblingElement("successor");
        }
        XMLElement *pGeometry = pRoad->FirstChildElement("planView")->FirstChildElement("geometry");
        while (pGeometry != NULL)
        {
            roadGeometry geometry;
            pGeometry->QueryFloatAttribute("hdg", &geometry.hdg);
            pGeometry->QueryFloatAttribute("length", &geometry.length);
            pGeometry->QueryFloatAttribute("s", &geometry.s);
            pGeometry->QueryFloatAttribute("x", &geometry.x);
            pGeometry->QueryFloatAttribute("y", &geometry.y);
            XMLElement *pParamPoly = pGeometry->FirstChildElement("paramPoly3");
            while (pParamPoly != NULL)
            {
                pParamPoly->QueryFloatAttribute("aU", &geometry.aU);
                pParamPoly->QueryFloatAttribute("aV", &geometry.aV);
                pParamPoly->QueryFloatAttribute("bU", &geometry.bU);
                pParamPoly->QueryFloatAttribute("bV", &geometry.bV);
                pParamPoly->QueryFloatAttribute("cU", &geometry.cU);
                pParamPoly->QueryFloatAttribute("cV", &geometry.cV);
                pParamPoly->QueryFloatAttribute("dU", &geometry.dU);
                pParamPoly->QueryFloatAttribute("dV", &geometry.dV);
                curRoadElement.geometry.push_back(geometry);
                pParamPoly = pParamPoly->NextSiblingElement("paramPoly3");
            }
            pGeometry = pGeometry->NextSiblingElement("geometry");
        }
        curRoadElement.cost = 99999999999;
        curRoadElement.nextRoad = -1;
        curRoadElement.altitude = Altitude;
        curRoadElement.scale = Scale;
        curRoadElement.laneSplit = 0.0;
        RoadListOrg.push_back(curRoadElement);
        pRoad = pRoad->NextSiblingElement("road");
    }
    if (SingleLane == true)
    {
        MergeLanes();
    }
    else {
        SplitLanes();
    }
    GetNodes();
    MapPoints = GetRoadPoints(RoadList, num);
    MapElList = GetMapPoints(RoadList, num);
    return;
}

void openDriveReader::MergeLanes()
{
    std::vector<int> removelist;
    for (int i = 0; i < (int)RoadListOrg.size(); i++)
    {
        int removeElement = -1;
        for (int j = i + 1; j < (int)RoadListOrg.size(); j++)
        {
            if (RoadListOrg[i].succId == RoadListOrg[j].succId && RoadListOrg[i].predId == RoadListOrg[j].predId)
            {
                removeElement = j;
                removelist.push_back(j);
            }
        }
        //Parallel lane not found,Add the lane
        if (removeElement == -1)
        {
            for (int k = 0; k < (int)removelist.size(); k++) {
                if (removelist[k] == i)
                {
                    removeElement = i;
                }
            }
            if (removeElement == -1)
            {
                RoadList.push_back(RoadListOrg[i]);
            }
        }
        else {
            //if Parallel lane found average the values and add
            roadGeometry geo1 = RoadListOrg[i].geometry[0];
            roadGeometry geo2 = RoadListOrg[removeElement].geometry[0];
            roadElement mergedRoad = RoadListOrg[i];
            mergedRoad.geometry[0].x = (geo1.x + geo2.x) / 2;
            mergedRoad.geometry[0].y = (geo1.y + geo2.y) / 2;
            mergedRoad.geometry[0].hdg = (geo1.hdg + geo2.hdg) / 2;
            mergedRoad.geometry[0].s = (geo1.s + geo2.s) / 2;
            mergedRoad.geometry[0].length = (geo1.length + geo2.length) / 2;
            mergedRoad.geometry[0].aU = (geo1.aU + geo2.aU) / 2;
            mergedRoad.geometry[0].aV = (geo1.aV + geo2.aV) / 2;
            mergedRoad.geometry[0].bU = (geo1.bU + geo2.bU) / 2;
            mergedRoad.geometry[0].bV = (geo1.bV + geo2.bV) / 2;
            mergedRoad.geometry[0].cU = (geo1.cU + geo2.cU) / 2;
            mergedRoad.geometry[0].cV = (geo1.cV + geo2.cV) / 2;
            mergedRoad.geometry[0].dU = (geo1.dU + geo2.dU) / 2;
            mergedRoad.geometry[0].dV = (geo1.dV + geo2.dV) / 2;
            RoadList.push_back(mergedRoad);
        }
    }
    return;
}

void openDriveReader::SplitLanes()
{
    for (int i = 0; i < (int)RoadListOrg.size(); i++)
    {
        roadElement el = RoadListOrg[i];
        if (el.junction != -1)
        {
            if (el.id % 2 == 1)
            {
                //el.laneSplit = JunctionWidth*el.scale/2;
            }
            else {
                //el.laneSplit = -LaneWidth + JunctionWidth*el.scale;
            }
            RoadList.push_back(el);
        }
        else
        {
            roadElement left = el, right = el;
            left.laneSplit = LaneWidth;
            right.laneSplit = -LaneWidth;
            left.predId = el.succId;
            left.succId = el.predId;
            left.predET = el.succET;
            left.succET = el.predET;
            left.predCP = el.succCP;
            left.succCP = el.predCP;
            RoadList.push_back(left);
            RoadList.push_back(right);
        }
    }
}

void openDriveReader::GetNodes()
{

    for (int i = 0; i < (int)RoadList.size(); i++)
    {
        std::vector<node> nodeList;
        int roadid = RoadList[i].id;
        int predId = RoadList[i].predId;
        int succId = RoadList[i].succId;
        const char *succType = RoadList[i].succET;
        const char *predType = RoadList[i].predET;
        const char *succContact = RoadList[i].succCP;
        const char *predContact = RoadList[i].predCP;
        const char *roadString, *startString, *endString;
        roadString = "road";
        startString = "start";
        endString = "end";
        if (*predType == *roadString && BiDirectional == true)
        {
            node curNode;
            curNode.id = predId;
            curNode.contactPoint = predContact;
            nodeList.push_back(curNode);
        }
        else if (BiDirectional == true && (predType) != NULL)
        {
            //Go through all road element to find predecessor
            for (int j = 0; j < (int)RoadList.size(); j++)
            {
                roadElement el = RoadList[j];
                //If not the right junction skip
                if (el.junction != predId)
                {
                    continue;
                }
                if (el.succId == roadid && *el.succCP == *startString)
                {
                    node curNode;
                    curNode.id = el.id;
                    curNode.contactPoint = endString;
                    nodeList.push_back(curNode);
                }
                else if (el.predId == roadid && *el.predCP == *startString)
                {
                    node curNode;
                    curNode.id = el.id;
                    curNode.contactPoint = startString;
                    nodeList.push_back(curNode);
                }
            }
        }

        if (*succType == *roadString)
        {
            node curNode;
            curNode.id = succId;
            curNode.contactPoint = succContact;
            nodeList.push_back(curNode);
        }
        else if (succType != NULL)
        {
            //Go through all road element to find successor
            for (int j = 0; j < (int)RoadList.size(); j++)
            {
                roadElement el = RoadList[j];

                if (el.junction != succId)
                {
                    continue;
                }
                if (el.succId == roadid && *el.succCP != *startString)
                {
                    node curNode;
                    curNode.id = el.id;
                    curNode.contactPoint = startString;
                    nodeList.push_back(curNode);
                }
                else if (el.predId == roadid && *el.predCP != *startString)
                {
                    node curNode;
                    curNode.id = el.id;
                    curNode.contactPoint = startString;
                    nodeList.push_back(curNode);
                }
            }
        }
        RoadList[i].nodes = nodeList;
    }
    return;
}



Quaternion openDriveReader::toQuaternion(double pitch, double roll, double yaw)
{
    Quaternion q;
    double t0 = std::cos(yaw * 0.5);
    double t1 = std::sin(yaw * 0.5);
    double t2 = std::cos(roll * 0.5);
    double t3 = std::sin(roll * 0.5);
    double t4 = std::cos(pitch * 0.5);
    double t5 = std::sin(pitch * 0.5);

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
    a.roll = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    double t2 = +2.0 * (q.w * q.y - q.z * q.x);
    t2 = ((t2 > 1.0) ? 1.0 : t2);
    t2 = ((t2 < -1.0) ? -1.0 : t2);
    a.pitch = std::asin(t2);

    // yaw (z-axis rotation)
    double t3 = +2.0 * (q.w * q.z + q.x * q.y);
    double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
    a.yaw = std::atan2(t3, t4);
    return a;
}


float openDriveReader::CubicPoly(float a1, float b1, float c1, float d1, float ds)
{

    return (a1 + b1*ds + c1*pow(ds, 2.0) + d1*pow(ds, 3.0));
}

float openDriveReader::RotateCCWX(float u2, float v2, float hdg2)
{
    return (u2*cos(hdg2) - v2*sin(hdg2));
}

float openDriveReader::RotateCCWY(float u1, float v1, float hdg1)
{
    return (u1*sin(hdg1) + v1*cos(hdg1));
}

float openDriveReader::EuclideanDistance(Pose3D pose1, Pose3D pose2)
{
    float x0 = pose1.p.x;
    float y0 = pose1.p.y;
    float z0 = pose1.p.z;
    float x1 = pose2.p.x;
    float y1 = pose2.p.y;
    float z1 = pose2.p.z;
    float d = sqrt((x0 - x1)*(x0 - x1) + (y0 - y1)*(y0 - y1) + (z0 - z1)*(z0 - z1));
    return d;
}

std::vector<Pose3D> openDriveReader::UpdatePoseHeading(std::vector<Pose3D> path)
{
    std::vector<Pose3D> list;
    Pose3D pose;
    int pathLength = path.size();
    float h = 0.0;
    for (int i = 0; i < pathLength; i++)
    {
        if (i + 1 != pathLength)
        {
            float x0 = path[i].p.x, y0 = path[i].p.y;
            float x1 = path[i + 1].p.x, y1 = path[i + 1].p.y;
            h = atan2(y1 - y0, x1 - x0);
        }
        pose = path[i];
        pose.q = toQuaternion(0, 0, h);
        list.push_back(pose);
    }
    return list;
}


std::vector<Pose3D> openDriveReader::GetRoadPoints(roadElement el, int num)
{
    std::vector<Pose3D> list;
    Pose3D pose;
    //Finding heading change for lane shifting, Does not affect anything if there is no lane shift
    float curHeading = 0.0, headingChange = 0.0, lastHeading = 0.0;
    for (int i = 0; i < num; i++)
    {
        float ds = (float)(i + 0.1) / (num);
        roadGeometry geo = el.geometry[0];
        //Find points from parametric polynomial in local coordinate
        float tempx = CubicPoly(geo.aU, geo.bU, geo.cU, geo.dU, ds);
        float tempy = CubicPoly(geo.aV, geo.bV, geo.cV, geo.dV, ds);
        //Split to lane point, use heading change to rotate lane shift
        float tempLanex = RotateCCWX(0, el.laneSplit / el.scale, headingChange);
        float tempLaney = RotateCCWY(0, el.laneSplit / el.scale, headingChange);
        //Rotate to global coordinate
        float newx = RotateCCWX(tempx + tempLanex, tempy + tempLaney, geo.hdg);
        float newy = RotateCCWY(tempx + tempLanex, tempy + tempLaney, geo.hdg);
        //Shift to global coordinate
        pose.p.x = (geo.x + newx) * el.scale;
        pose.p.y = (geo.y + newy) * el.scale;
        pose.p.z = el.altitude;
        pose.q = toQuaternion(0, 0, geo.hdg);
        //Store line
        list.push_back(pose);
        //Get the heading change from last point to current point
        if (i != 0)
        {
            float x0 = list[i - 1].p.x, y0 = list[i - 1].p.y;
            float x1 = pose.p.x, y1 = pose.p.y;
            lastHeading = curHeading;
            curHeading = atan2(y1 - y0, x1 - x0);;
            headingChange += normalizeAngle(curHeading - lastHeading, 0);
            headingChange = normalizeAngle(headingChange, 0);
        }
        else
        {
            curHeading = geo.hdg;
        }
    }
    UpdatePoseHeading(list);
    return list;
}



std::vector<Pose3D> openDriveReader::GetRoadPoints(std::vector<roadElement> el, int num)
{
    std::vector<Pose3D> vect;
    for (int j = 0; j < (int)el.size(); j++)
    {
        std::vector<Pose3D> points3d = GetRoadPoints(el[j], num);
        for (int i = 0; i < (int)points3d.size(); i++) {
            vect.push_back(points3d[i]);
        }
    }
    return vect;
}

std::vector<MapElement> openDriveReader::GetMapPoints(std::vector<roadElement> roadlist, int num)
{
    std::vector<MapElement> vect;
    int numRoads = roadlist.size();
    for (int j = 0; j < numRoads; j++)
    {
        roadElement el = roadlist[j];
        for (int i = 0; i < num; i++)
        {
            MapElement mapEl;
            float ds = (float)(i + 1) / (num + 1);
            roadGeometry geo = el.geometry[0];
            float tempx = CubicPoly(geo.aU, geo.bU, geo.cU, geo.dU, ds);
            float tempy = CubicPoly(geo.aV, geo.bV, geo.cV, geo.dV, ds);
            float newx = RotateCCWX(tempx, tempy, geo.hdg);
            float newy = RotateCCWY(tempx, tempy, geo.hdg);
            mapEl.pose.p.x = (geo.x + newx) * el.scale;
            mapEl.pose.p.y = (geo.y + newy) * el.scale;
            mapEl.pose.p.z = el.altitude;
            mapEl.pose.q = toQuaternion(0, 0, geo.hdg);
            mapEl.roadId = el.id;
            if (i == 0)
            {
                mapEl.nodes.push_back(i + 1 + j*num);
                if (el.predId == -1)
                {
                    break;
                }
                for (int k = 0; k < numRoads; k++)
                {
                    roadElement preEl = roadlist[k];
                    if (el.junction != -1 && el.predId == preEl.id)
                    {
                        if (preEl.succId == el.junction)
                        {
                            mapEl.nodes.push_back(k*num + num - 1);
                        }
                        else if (preEl.predId == el.junction)
                        {
                            mapEl.nodes.push_back(k*num);
                        }
                    }
                    else if (el.predId == preEl.id || el.predId == preEl.junction)
                    {
                        if (preEl.succId == el.id)
                        {
                            mapEl.nodes.push_back(k*num + num - 1);
                        }
                        else if (preEl.predId == el.id)
                        {
                            mapEl.nodes.push_back(k*num);
                        }
                    }
                }
            }
            else if (i + 1 == num)
            {
                mapEl.nodes.push_back(i - 1 + j*num);
                if (el.succId == -1)
                {
                    break;
                }
                for (int k = 0; k < numRoads; k++)
                {
                    roadElement preEl = roadlist[k];
                    if (el.junction != -1 && el.succId == preEl.id)
                    {
                        if (preEl.succId == el.junction)
                        {
                            mapEl.nodes.push_back(k*num + num - 1);
                        }
                        else if (preEl.predId == el.junction)
                        {
                            mapEl.nodes.push_back(k*num);
                        }
                    }
                    else if (el.succId == preEl.id || el.succId == preEl.junction)
                    {
                        if (preEl.succId == el.id)
                        {
                            mapEl.nodes.push_back(k*num + num - 1);
                        }
                        else if (preEl.predId == el.id)
                        {
                            mapEl.nodes.push_back(k*num);
                        }
                    }
                }
            }
            else
            {
                mapEl.nodes.push_back(i - 1 + j*num);
                mapEl.nodes.push_back(i + 1 + j*num);
            }
            vect.push_back(mapEl);
        }
    }
    return vect;
}



/*! calculates normalized angle */
tFloat32 openDriveReader::normalizeAngle(tFloat32 alpha, tFloat32 center)
{
    return mod(alpha - center + M_PI, 2.0*M_PI) + center - M_PI;
}

/*! calculates modulus after division */
tFloat32 openDriveReader::mod(tFloat32 x, tFloat32 y)
{
    tFloat32 r;
    tFloat32 b_x;
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
