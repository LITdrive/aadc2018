﻿/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#define _USE_MATH_DEFINES
#include "LITD_StanleyControl.h"
#include <cmath>
#include <cfloat>
#include "math_utilities.h"

/* notes to check
[] actual speed has to be in car-struct
[] is the given point normal to car position?

*/


ADTF_PLUGIN(LABEL_STANLEY_CONTROL_FILTER, cStanleyControl)

tResult cStanleyControl::Init(const tInitStage eStage)
{
	RETURN_IF_FAILED(cFilter::Init(eStage));
	if (eStage == StageFirst)
	{
		// press "Init"
		RETURN_IF_FAILED(Configure());
	}
	RETURN_NOERROR;
}

void cStanleyControl::calcSteeringAngle(){
    double rad2degree = 180.0 / M_PI;
    //vector between car and virtualpoint
	// call getNextVirtualPointOnPoly -> Result is vehicleTargetFrontAxlePosition
    Vector2d diff = vehicleTargetFrontAxlePosition.getVector2d() - vehicleActualFrontAxlePosition.getVector2d();

    //calc sign to steer in direction of road
    int sign = 1;
    double diff_heading_abs = wrapTo2Pi(atan2(diff(1), diff(0)));
    if(wrapTo2Pi(diff_heading_abs - wrapTo2Pi(vehicleTargetFrontAxlePosition.h))> 4.712 ){
        sign = -1;
    }

    //calc normal distance of tangent to car (e)
    double e = (vehicleTargetFrontAxlePosition.getVector2d() - vehicleActualFrontAxlePosition.getVector2d()).norm() * sign;

    //calc angle between car heading and point tangent
    double theta_c =  wrapTo2Pi(vehicleTargetFrontAxlePosition.h) - wrapTo2Pi(vehicleActualFrontAxlePosition.h);

    //calc steering-angle with stanley-approach
    vehicleSteeringAngle = theta_c + atan2(STANLEY_GAIN*e, vehicleSpeed);

    //Debug Messages
    /*std::cout << "-----------------------" << std::endl;
    std::cout << "point heading : " << vp.h << "(" << rad2degree * vp.h << "°)" << std::endl;
    std::cout << "car heading: " << carPosition.h << "(" << rad2degree * carPosition.h << "°)" << std::endl;
    std::cout << "diff heading: " << diff_heading_abs << "(" << rad2degree * diff_heading_abs << "°)" << std::endl;
    std::cout << "e: " << e << std::endl;
    std::cout << "Theta_C: " << theta_c << "(" << rad2degree * theta_c << "°)" << std::endl;
    std::cout << "Steering Angle: " << carSteeringAngle << "(" << rad2degree * carSteeringAngle << "°)" << std::endl;
    std::cout << "-----------------------" << std::endl;*/

}

cStanleyControl::cStanleyControl()
{
	/* tPosition */
	object_ptr<IStreamType> pTypePositionData;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_PositionSampleFactory))
	{
		adtf_ddl::access_element::find_index(m_PositionSampleFactory, cString("f32x"), m_ddlPositionIndex.f32x);
		adtf_ddl::access_element::find_index(m_PositionSampleFactory, cString("f32y"), m_ddlPositionIndex.f32y);
		adtf_ddl::access_element::find_index(m_PositionSampleFactory, cString("f32radius"), m_ddlPositionIndex.f32radius);
		adtf_ddl::access_element::find_index(m_PositionSampleFactory, cString("f32speed"), m_ddlPositionIndex.f32speed);
		adtf_ddl::access_element::find_index(m_PositionSampleFactory, cString("f32heading"), m_ddlPositionIndex.f32heading);
	}
	else
	{
		LOG_WARNING("No mediadescription for tPosition found!");
	}

	/* tTrajectory */
	object_ptr<IStreamType> pTypeTrajectoryData;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tTrajectory", pTypeTrajectoryData, m_TrajectorySampleFactory))
	{
		adtf_ddl::access_element::find_index(m_TrajectorySampleFactory, cString("id"), m_ddlTrajectoryIndex.id);
		adtf_ddl::access_element::find_index(m_TrajectorySampleFactory, cString("ax"), m_ddlTrajectoryIndex.ax);
		adtf_ddl::access_element::find_index(m_TrajectorySampleFactory, cString("bx"), m_ddlTrajectoryIndex.bx);
		adtf_ddl::access_element::find_index(m_TrajectorySampleFactory, cString("cx"), m_ddlTrajectoryIndex.cx);
		adtf_ddl::access_element::find_index(m_TrajectorySampleFactory, cString("dx"), m_ddlTrajectoryIndex.dx);
		adtf_ddl::access_element::find_index(m_TrajectorySampleFactory, cString("ay"), m_ddlTrajectoryIndex.ay);
		adtf_ddl::access_element::find_index(m_TrajectorySampleFactory, cString("by"), m_ddlTrajectoryIndex.by);
		adtf_ddl::access_element::find_index(m_TrajectorySampleFactory, cString("cy"), m_ddlTrajectoryIndex.cy);
		adtf_ddl::access_element::find_index(m_TrajectorySampleFactory, cString("dy"), m_ddlTrajectoryIndex.dy);
		adtf_ddl::access_element::find_index(m_TrajectorySampleFactory, cString("start"), m_ddlTrajectoryIndex.start);
		adtf_ddl::access_element::find_index(m_TrajectorySampleFactory, cString("end"), m_ddlTrajectoryIndex.end);
		adtf_ddl::access_element::find_index(m_TrajectorySampleFactory, cString("backwards"), m_ddlTrajectoryIndex.backwards);
	}
	else
	{
		LOG_WARNING("No mediadescription for tTrajectory found!");
	}

	/* tSignalValue */
	object_ptr<IStreamType> pTypeSignalValue;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
	{
		adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp);
		adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value);
	}
	else
	{
		LOG_INFO("No mediadescription for tSignalValue found!");
	}


    //Register input pins
	create_pin(*this, m_ActualPointReader, "actual_point", pTypePositionData);
	create_pin(*this, m_TrajectoryReader, "inTrajectories", pTypeTrajectoryData);
	//Register output pin
	filter_create_pin(*this, m_SteeringWriter, "steering", pTypeSignalValue);
		
	create_inner_pipe(*this, cString::Format("%s_trigger", "actual_point"), "actual_point", [&](tTimeStamp tmTime) -> tResult
	{
		return ProcessTrajectories(tmTime);
	});

	create_inner_pipe(*this, cString::Format("%s_trigger", "trajectories"), "trajectories", [&](tTimeStamp tmTime) -> tResult
	{
		return ProcessPosition(tmTime);
	});
}


//implement the Configure function to read ALL Properties
tResult cStanleyControl::Configure()
{
    RETURN_NOERROR;
}

tResult cStanleyControl::ProcessPosition(tTimeStamp tmTimeOfTrigger)
{
	::tPosition position;
	object_ptr<const ISample> pReadSample;
	if (IS_OK(m_ActualPointReader.GetNextSample(pReadSample))) {
		auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pReadSample);

		RETURN_IF_FAILED(oDecoder.IsValid());

		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.f32x, &position.f32x));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.f32y, &position.f32y));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.f32heading, &position.f32heading));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.f32radius, &position.f32radius));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.f32speed, &position.f32speed));
	}

	vehicleSpeed = position.f32speed;
	vehicleActualRearAxlePosition.x = position.f32x;
	vehicleActualRearAxlePosition.y = position.f32y;
	vehicleActualRearAxlePosition.h = position.f32heading;

	calcSteeringAngle();

	object_ptr<ISample> pWriteSample;

	if (IS_OK(alloc_sample(pWriteSample)))
	{

		auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSample);

		RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, vehicleSteeringAngle));

	}

	m_SteeringWriter << pWriteSample << flush << trigger;


	RETURN_NOERROR;
}

tResult cStanleyControl::ProcessTrajectories(tTimeStamp tmTimeOfTrigger){
	::tTrajectory trajectory;
	object_ptr<const ISample> pReadSample;
	if (IS_OK(m_TrajectoryReader.GetNextSample(pReadSample))) {
		auto oDecoder = m_TrajectorySampleFactory.MakeDecoderFor(*pReadSample);

		RETURN_IF_FAILED(oDecoder.IsValid());

		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlTrajectoryIndex.id, &trajectory.id));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlTrajectoryIndex.ax, &trajectory.ax));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlTrajectoryIndex.bx, &trajectory.bx));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlTrajectoryIndex.cx, &trajectory.cx));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlTrajectoryIndex.dx, &trajectory.dx));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlTrajectoryIndex.ay, &trajectory.ay));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlTrajectoryIndex.by, &trajectory.by));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlTrajectoryIndex.cy, &trajectory.cy));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlTrajectoryIndex.dy, &trajectory.dy));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlTrajectoryIndex.start, &trajectory.start));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlTrajectoryIndex.end, &trajectory.end));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlTrajectoryIndex.backwards, &trajectory.backwards));
	}

	updatePolyList(trajectory);
	// TODO

	RETURN_NOERROR;
}

void cStanleyControl::calculateFrontAxlePosition(LITD_VirtualPoint rearAxlePosition, LITD_VirtualPoint *frontAxlePosition) {
	double dx = cos(rearAxlePosition.h)*VEHICLE_AXIS_DISTANCE;
	double dy = sin(rearAxlePosition.h)*VEHICLE_AXIS_DISTANCE;

	frontAxlePosition->x = rearAxlePosition.x + dx;
	frontAxlePosition->x = rearAxlePosition.y + dy;
	frontAxlePosition->x = rearAxlePosition.h;
}

void cStanleyControl::updatePolyList(tTrajectory trajectory) {
	//expect always "PLOYLIST_LEN" new polys in each updateStep
	/*if (polyLen > POLYLIST_LEN || polyLen < POLYLIST_LEN) {
		std::cout << "Got wrong number of new polys " << std::endl;
		return;
	}*/

	//update polyList
	/*for (int i = 0; i<TRAJECTORY_ARRAY_LEN; i++) {
		trajectoryArray[i] = trajectory;
	}*/

	trajectoryArray[0] = trajectoryArray[1];
	trajectoryArray[1] = trajectory;
}

/* void cStanleyControl::updateStep(poly_t polys[], uint8_t polyLen, LITD_VirtualPoint actPos) {
	LITD_VirtualPoint frontAxlePosition;
	LITD_VirtualPoint idealPoint;
	polyPoint_t idealPolyPoint;
	double carSpeed = 0;

	// actPos = Position of the rear axle of the vehicle (reference point)
	updatePolyList(polys, polyLen);

	calculateFrontAxlePosition(actPos, &frontAxlePosition);

	getNextVirtualPointOnPoly(polys, polyLen, &idealPolyPoint, &idealPoint, frontAxlePosition);

	//calc Steeringangle
	steeringAngle = calcSteeringAngle(frontAxlePosition, idealPoint, carSpeed);
} */

void cStanleyControl::getNextVirtualPointOnPoly(tTrajectory trajectories[], uint8_t polyLen, tTrajectory* idealPolyPoint, LITD_VirtualPoint* idealPoint, LITD_VirtualPoint carPosition) {

	LITD_VirtualPoint actPoint;
	double min_dist = DBL_MAX;
	int min_poly_index = 0;
	double min_poly_p = 0;
	double min_dist_x = DBL_MAX;
	double min_dist_y = DBL_MAX;
	double min_dist_h = 0;

	for (int i = 0; i<TRAJECTORY_ARRAY_LEN; i++)
	{
		for (int j = 0; j <= POINTS_PER_POLY; j++)
		{
			// p = [0, 1]
			double p = j / POINTS_PER_POLY;
			calcVirtualPointfromPoly(&trajectories[i], p, &actPoint);

			//calc norm to carPosition
			double dist = sqrt(pow(actPoint.x - carPosition.x, 2) + pow(actPoint.y - carPosition.y, 2));

			if (dist < min_dist)
			{
				min_dist = dist;
				min_poly_index = i;
				min_poly_p = p;
				min_dist_x = actPoint.x;
				min_dist_y = actPoint.y;
				min_dist_h = actPoint.h;
			}
		}
	}

	// Function value and ID of Poly with smallest distance to given car point
	idealPolyPoint->id = trajectories[min_poly_index].id;
	// TODO: not compiling, there is no parameter p
	// idealPolyPoint->p = min_poly_p;

	// Point on Poly with smallest distance to given car point
	idealPoint->x = min_dist_x;
	idealPoint->y = min_dist_y;
	idealPoint->h = min_dist_h;
}

void cStanleyControl::calcVirtualPointfromPoly(tTrajectory* poly, double p, LITD_VirtualPoint* vp) {
	double x = poly->ax * pow(p, 3) + poly->bx * pow(p, 2) + poly->cx * p + poly->dx;
	double y = poly->ay * pow(p, 3) + poly->by * pow(p, 2) + poly->cy * p + poly->dy;

	//tangente durch erste ableitung berechnen
	double x_der = poly->ax * pow(p, 2) + poly->bx * p + poly->cx;
	double y_der = poly->ay * pow(p, 2) + poly->by * p + poly->cy;

	double heading = wrapTo2Pi(atan2(y_der, x_der));

	vp->x = x;
	vp->y = y;
	vp->h = heading;
}