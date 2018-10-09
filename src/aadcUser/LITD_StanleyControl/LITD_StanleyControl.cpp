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
#define DEBUG_STANLEY false
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

void cStanleyControl::calcSteeringAngle(){ // double cStanelyControl::calcSteeringAngle(LITD_VirtualPoint vehicleTargetFrontAxlePosition, LITD_VirtualPoint vehicleActualFrontAxlePosition, bool parking) {
	double rad2degree = 180.0 / M_PI;

	
	//if (!parking || parking && !parkingStartPointReached || parking && parkingFinished)
	if (!parking) //parking handling only through trajectories
	{
		// No Parking -> Drive with Stanley
		
		//vector between car and virtualpoint
		// call getNextVirtualPointOnPoly -> Result is vehicleTargetFrontAxlePosition
		Vector2d diff = vehicleTargetFrontAxlePosition.getVector2d() - vehicleActualFrontAxlePosition.getVector2d();

		//calc sign to steer in direction of road
		int sign = 1;
		double diff_heading_abs = wrapTo2Pi(atan2(diff(1), diff(0)));
		if (wrapTo2Pi(diff_heading_abs - wrapTo2Pi(vehicleTargetFrontAxlePosition.h)) > M_PI) {
			sign = -1;
		}

		//calc normal distance of tangent to car (e)
		// double e = (vehicleTargetFrontAxlePosition.getVector2d() - vehicleActualFrontAxlePosition.getVector2d()).norm() * sign;
		double e = diff.norm()*sign;

		//calc angle between car heading and point tangent
		//double theta_c = wrapTo2Pi(vehicleActualFrontAxlePosition.h - vehicleTargetFrontAxlePosition.h);
		double theta_c = wrapTo2Pi(vehicleTargetFrontAxlePosition.h - vehicleActualFrontAxlePosition.h);

		//if (theta_c < 0)
		//{
		//	theta_c = 2 * M_PI + theta_c;
		//}

		//calc steering-angle with stanley-approach
		double dynamicStanleyPart = 0;

		if (vehicleSpeed > 0.02)
		{
			dynamicStanleyPart = atan2(stanleyGain * e, vehicleSpeed);
		}

		vehicleSteeringAngle = theta_c + dynamicStanleyPart;

		if(DEBUG_STANLEY) LOG_INFO("-----------------------");
		if(DEBUG_STANLEY) LOG_INFO("--------Stanley--------");
		if(DEBUG_STANLEY) LOG_INFO("POINT_Heading in °: %f", vehicleTargetFrontAxlePosition.h *rad2degree);
		if(DEBUG_STANLEY) LOG_INFO("Car Heading in °: %f", vehicleActualFrontAxlePosition.h *rad2degree);
		if(DEBUG_STANLEY) LOG_INFO("Diff Heading in °: %f", diff_heading_abs *rad2degree);
		if(DEBUG_STANLEY) LOG_INFO("Stanley e: %f", e);
		if(DEBUG_STANLEY) LOG_INFO("Stanley Theta in °: %f", theta_c *rad2degree);
		if(DEBUG_STANLEY) LOG_INFO("Steering Angle in °: %f", vehicleSteeringAngle *rad2degree);
		if(DEBUG_STANLEY) LOG_INFO("-----------------------");
	}

	else
	{
		// Parking -> Drive with saturated control
		double theta = vehicleActualRearAxlePosition.h - M_PI;
		double v = K*(theta - a_0*vehicleActualRearAxlePosition.y);
		double vehicleSteeringAngleDegree = atan(VEHICLE_AXIS_DISTANCE*u*tanh(K_t*v)); // Degree
		vehicleSteeringAngle = vehicleSteeringAngleDegree*(M_PI / 180.0); // Radians
		// TODO ev. noch *(-1) bzw. theta umdrehen -> Vorzeichen ändern wenn Lenkeinschlag in falsche Richtung

		if(DEBUG_STANLEY) LOG_INFO("-----------------------");
		if(DEBUG_STANLEY) LOG_INFO("---Saturated Control---");
		if(DEBUG_STANLEY) LOG_INFO("Car Heading in °: %f", vehicleActualRearAxlePosition.h *rad2degree);
		if(DEBUG_STANLEY) LOG_INFO("Saturated Control Theta in °: %f", theta *rad2degree);
		if(DEBUG_STANLEY) LOG_INFO("Steering Angle in °: %f", vehicleSteeringAngleDegree);
		if(DEBUG_STANLEY) LOG_INFO("-----------------------");
	}

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
	RegisterPropertyVariable("dynamic properties path", m_properties_file);

	/* tPosition */
	object_ptr<IStreamType> pTypePositionData;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_PositionSampleFactory))
	{
		access_element::find_index(m_PositionSampleFactory, cString("f32x"), m_ddlPositionIndex.f32x);
		access_element::find_index(m_PositionSampleFactory, cString("f32y"), m_ddlPositionIndex.f32y);
		access_element::find_index(m_PositionSampleFactory, cString("f32radius"), m_ddlPositionIndex.f32radius);
		access_element::find_index(m_PositionSampleFactory, cString("f32speed"), m_ddlPositionIndex.f32speed);
		access_element::find_index(m_PositionSampleFactory, cString("f32heading"), m_ddlPositionIndex.f32heading);
	}
	else
	{
		LOG_WARNING("No mediadescription for tPosition found!");
	}

	/* tTrajectoryArray */
	object_ptr<IStreamType> pTypeTrajectoryArrayData;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tTrajectoryArray", pTypeTrajectoryArrayData, m_TrajectoryArraySampleFactory))
	{
		access_element::find_index(m_TrajectoryArraySampleFactory, cString("size"), m_ddlTrajectoryArrayIndex.size);
		access_element::find_array_index(m_TrajectoryArraySampleFactory, cString("trajectories"), m_ddlTrajectoryArrayIndex.trajectories);
	}
	else
	{
		LOG_WARNING("No mediadescription for tTrajectoryArray found!");
	}

	/* tSignalValue */
	object_ptr<IStreamType> pTypeSignalValue;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
	{
		access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp);
		access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value);
	}
	else
	{
		LOG_WARNING("No mediadescription for tSignalValue found!");
	}

	/* tPolynomPoint */
	object_ptr<IStreamType> pTypePolynomPoint;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPolynomPoint", pTypePolynomPoint, m_PolynomPointSampleFactory))
	{
		access_element::find_index(m_PolynomPointSampleFactory, cString("id"), m_ddlPolynomPointIndex.id);
		access_element::find_index(m_PolynomPointSampleFactory, cString("parameter"), m_ddlPolynomPointIndex.parameter);
	}
	else
	{
		LOG_WARNING("No mediadescription for tPolynomPoint found!");
	}

    // register input pins
	create_pin(*this, m_ActualPointReader, "actual_point", pTypePositionData);
	create_pin(*this, m_TrajectoryArrayReader, "trajectories", pTypeTrajectoryArrayData);

	// register output pin
	filter_create_pin(*this, m_SteeringWriter, "steering", pTypeSignalValue);
	filter_create_pin(*this, m_PolyFinishedWriter, "poly_finished", pTypePolynomPoint);
	filter_create_pin(*this, m_PolyTargetPointWriter, "poly_target_point", pTypePolynomPoint);

	create_inner_pipe(*this, cString::Format("%s_trigger", "actual_point"), "actual_point", [&](tTimeStamp tmTime) -> tResult
	{
		return ProcessPosition(tmTime);
	});

	create_inner_pipe(*this, cString::Format("%s_trigger", "trajectories"), "trajectories", [&](tTimeStamp tmTime) -> tResult
	{
		return ProcessTrajectories(tmTime);
	});
}

//implement the Configure function to read ALL Properties
tResult cStanleyControl::Configure()
{
	if(DEBUG_STANLEY) LOG_INFO("Configure is running now!");

	// load properties file for dynamic properties
	cFilename propertiesFileResolved = m_properties_file;
	adtf::services::ant::adtf_resolve_macros(propertiesFileResolved);
	m_properties = new FilePropertiesObserver(propertiesFileResolved.GetPtr());
	m_properties->ReloadProperties();

	// Parking
	/*parkingStartPoint.x = -0.6369718092383394;
	parkingStartPoint.y = 1.0000000000000004;
	parkingStartPoint.h = M_PI / 2;
	parkingTargetPoint.x = 0.702;
	parkingTargetPoint.y = 0.0;
	parkingTargetPoint.h = M_PI;*/

    RETURN_NOERROR;
}

tResult cStanleyControl::ProcessPosition(tTimeStamp tmTimeOfTrigger)
{
    if(trj_list.countTrajectories()<=0){
        LOG_WARNING("Tried Driving without trajectories");
        RETURN_NOERROR;
    }
	::tPosition position;
	//Read Property-File
	m_properties->TriggerPropertiesReload(80); // reload the file every 2 seconds with a 25 msec timer
	stanleyGain = m_properties->GetFloat("stanley_gain");
	maxAngleDegrees = m_properties->GetFloat("max_angle");

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
	//vehicleSpeed = 0;
	vehicleActualRearAxlePosition.x = position.f32x;
	vehicleActualRearAxlePosition.y = position.f32y;
	vehicleActualRearAxlePosition.h = wrapTo2Pi(position.f32heading);
	//vehicleActualRearAxlePosition.h = M_PI/4;


	if(DEBUG_STANLEY) LOG_INFO("Point of BackPosition: x: %f, y: %f, h: %f", vehicleActualRearAxlePosition.x, vehicleActualRearAxlePosition.y, vehicleActualRearAxlePosition.h  * 180.0 / M_PI);

	calculateActualFrontAxlePosition();

	if(DEBUG_STANLEY) LOG_INFO("Point of FrontPosition: x: %f, y: %f, h: %f", vehicleActualFrontAxlePosition.x, vehicleActualFrontAxlePosition.y, vehicleActualFrontAxlePosition.h * 180.0 / M_PI );

	std::tuple<uint32_t, uint32_t, double> list_ret = trj_list.getDistanceToNearestPoint(vehicleActualFrontAxlePosition, vehicleTargetFrontAxlePosition);
	tUInt32 id_finished = std::get<0>(list_ret);
	tUInt32 id_current = std::get<1>(list_ret);
	tFloat32 p_current = std::get<2>(list_ret);
	if(DEBUG_STANLEY) LOG_INFO("Point of SetPoint: x: %f, y: %f, h: %f", vehicleTargetFrontAxlePosition.x, vehicleTargetFrontAxlePosition.y, vehicleTargetFrontAxlePosition.h * 180.0 / M_PI );

	calcSteeringAngle();

	if(DEBUG_STANLEY) LOG_INFO("SteeringAngle in grad: %f", vehicleSteeringAngle * 180.0 / M_PI );

	mapSteeringAngle();
	

	object_ptr<ISample> pWriteSampleSteering;
	if (IS_OK(alloc_sample(pWriteSampleSteering)))
	{
		auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSampleSteering);
		RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, vehicleSteeringAngle));
		m_SteeringWriter << pWriteSampleSteering << flush << trigger;
	}

	//Send the finished polynomial id if the id is > 0.
	object_ptr<ISample> pWriteSamplePolyFinished;

	if (id_finished>0 && IS_OK(alloc_sample(pWriteSamplePolyFinished)))
	{
		auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSamplePolyFinished);
		RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPolynomPointIndex.id, id_finished));
		RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPolynomPointIndex.parameter, 0));
		m_PolyFinishedWriter << pWriteSamplePolyFinished << flush << trigger;
		if(DEBUG_STANLEY) {
			LOG_INFO("STANLEY: submitted %u poly finished!", id_finished);
		}
	}

	// TODO: WRITE SAMPLE WITH OUR TARGET POINT
	// TODO: WRITE SAMPLE WITH OUR TARGET POINT
	object_ptr<ISample> pWriteSampleTargetPoint;
	if (IS_OK(alloc_sample(pWriteSampleTargetPoint)))
	{

		auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSampleTargetPoint);
		RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPolynomPointIndex.id, id_current)); 
		RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPolynomPointIndex.parameter, p_current));
		// TODO: Uncomment if done.
		// m_PolyTargetPointWriter << pWriteSampleTargetPoint << flush << trigger;
	}

	RETURN_NOERROR;
}

tResult cStanleyControl::ProcessTrajectories(tTimeStamp tmTimeOfTrigger)
{

	object_ptr<const ISample> pReadSample;
	if (IS_OK(m_TrajectoryArrayReader.GetLastSample(pReadSample)))
	{
		/* read the tTrajectoryArray sample */
		tTrajectoryArray trajectoryArray;
		auto oDecoder = m_TrajectoryArraySampleFactory.MakeDecoderFor(*pReadSample);
		RETURN_IF_FAILED(oDecoder.IsValid());
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlTrajectoryArrayIndex.size, &trajectoryArray.size));
		const auto* trajectories = static_cast<const tTrajectory*>(oDecoder.GetElementAddress(m_ddlTrajectoryArrayIndex.trajectories));
		memcpy(&trajectoryArray.trajectories, trajectories, sizeof trajectoryArray.trajectories);

		trj_list.addTrajectoryArray(&trajectoryArray);
	}

	RETURN_NOERROR;
}

void cStanleyControl::calculateActualFrontAxlePosition() { //LITD_VirtualPoint cStanleyControl::calculateActualFrontAxlePosition(LITD_VirtualPoint vehicleActualRearAxlePosition) {
	double dx = cos(vehicleActualRearAxlePosition.h)*VEHICLE_AXIS_DISTANCE;
	double dy = sin(vehicleActualRearAxlePosition.h)*VEHICLE_AXIS_DISTANCE;

	vehicleActualFrontAxlePosition.x = vehicleActualRearAxlePosition.x + dx;
	vehicleActualFrontAxlePosition.y = vehicleActualRearAxlePosition.y + dy;
	vehicleActualFrontAxlePosition.h = vehicleActualRearAxlePosition.h;
	//return
}


void cStanleyControl::mapSteeringAngle(){ // double cStanleyControl::mapSteeringAngle(double vehicleSteeringAngle) {

	// it should be regardless for this function if the angle is between -90 and +90 degree or 270 to 90 degree

	tFloat32  rad2degree  = 180.0 / M_PI;

	while(vehicleSteeringAngle < -M_PI)
		vehicleSteeringAngle += 2 * M_PI;

	while(vehicleSteeringAngle > M_PI)
		vehicleSteeringAngle -= 2 * M_PI;

	/*if(vehicleSteeringAngle > 3/2 * M_PI && vehicleSteeringAngle < 2*M_PI){
		vehicleSteeringAngle -= 2 * M_PI; 
	}*/

	 if(vehicleSteeringAngle < -M_PI/4){
            vehicleSteeringAngle = -M_PI/4;
            if(DEBUG_STANLEY) LOG_INFO("Steering angle truncated to -45°!");
    } else if(vehicleSteeringAngle > M_PI/4){
            vehicleSteeringAngle = M_PI/4;
            if(DEBUG_STANLEY) LOG_INFO("Steering angle truncated to 45°!");
    }


    vehicleSteeringAngle = (vehicleSteeringAngle * rad2degree) / maxAngleDegrees * (-100); 
    if(DEBUG_STANLEY) LOG_INFO("SteeringAngle in steps: %f", vehicleSteeringAngle );

}
