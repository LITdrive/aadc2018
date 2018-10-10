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

cStanleyControl::cStanleyControl()
{
	RegisterPropertyVariable("dynamic properties path", m_properties_file);
	RegisterPropertyVariable("show debug log outputs", m_debug_messages);

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
	if(m_debug_messages) LOG_INFO("Configure is running now!");

	// load properties file for dynamic properties
	cFilename propertiesFileResolved = m_properties_file;
	adtf::services::ant::adtf_resolve_macros(propertiesFileResolved);
	m_properties = new FilePropertiesObserver(propertiesFileResolved.GetPtr());
	m_properties->ReloadProperties();

    RETURN_NOERROR;
}

tResult cStanleyControl::ProcessPosition(tTimeStamp tmTimeOfTrigger)
{
    if(trj_list.countTrajectories()<=0){
        LOG_WARNING("Tried Driving without trajectories");
        RETURN_NOERROR;
    }
	::tPosition position, position_front, position_target;
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

	position.f32heading=wrapTo2Pi(position.f32heading);


	if(m_debug_messages) LOG_INFO("Back: x: %f, y: %f, h: %f", position.f32x, position.f32y, position.f32heading  * 180.0 / M_PI);

	calculateActualFrontAxlePosition(position, position_front);

	if(m_debug_messages) LOG_INFO("Front: x: %f, y: %f, h: %f", position_front.f32x, position_front.f32y, position_front.f32heading * 180.0 / M_PI );

	std::tuple<uint32_t, uint32_t, double> list_ret = trj_list.getDistanceToNearestPoint(position_front, position_target);
	tUInt32 id_finished = std::get<0>(list_ret);
	tUInt32 id_current = std::get<1>(list_ret);
	tFloat32 p_current = std::get<2>(list_ret);
	if(m_debug_messages) LOG_INFO("Target: x: %f, y: %f, h: %f; id: %d, p: %f", position_target.f32x, position_target.f32y, position_target.f32heading * 180.0 / M_PI, id_current, p_current);

	tFloat32 steering_angle=calcSteeringAngle(position_target, position_front);

	if(m_debug_messages) LOG_INFO("SteeringAngle in degree: %f", steering_angle * 180.0 / M_PI );

	tFloat32 actual_steering_angle=mapSteeringAngle(steering_angle);
	
	if(m_debug_messages) LOG_INFO("Actual steering angle in degree: %f", steering_angle * 180.0 / M_PI );

	object_ptr<ISample> pWriteSampleSteering;
	if (IS_OK(alloc_sample(pWriteSampleSteering)))
	{
		auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSampleSteering);
		RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, actual_steering_angle));
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
		if(m_debug_messages) {
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

void cStanleyControl::calculateActualFrontAxlePosition(::tPosition& rear, ::tPosition& front) {
	float dx = cos(rear.f32heading)*VEHICLE_AXIS_DISTANCE;
	float dy = sin(rear.f32heading)*VEHICLE_AXIS_DISTANCE;

	front.f32x = rear.f32x + dx;
	front.f32y = rear.f32y + dy;
	front.f32heading = rear.f32heading;
	front.f32speed = rear.f32speed;
}


tFloat32 cStanleyControl::mapSteeringAngle(tFloat32 vehicleSteeringAngle) {

	vehicleSteeringAngle = wrapToPi<tFloat32>(vehicleSteeringAngle);

	if(vehicleSteeringAngle < -M_PI/4){
            vehicleSteeringAngle = -M_PI/4;
            if(m_debug_messages) LOG_INFO("Steering angle truncated to -45°!");
    } else if(vehicleSteeringAngle > M_PI/4){
            vehicleSteeringAngle = M_PI/4;
            if(m_debug_messages) LOG_INFO("Steering angle truncated to 45°!");
    }


    vehicleSteeringAngle = (vehicleSteeringAngle * 180.0 / M_PI) / maxAngleDegrees * (-100); 
    if(m_debug_messages) LOG_INFO("SteeringAngle in steps: %f", vehicleSteeringAngle );
	return vehicleSteeringAngle;
}

tFloat32 cStanleyControl::calcSteeringAngle(::tPosition& target, ::tPosition& front) {
	tFloat32 rad2degree = 180.0 / M_PI;

	
	//vector from car-front-axle to the target point. 
	tFloat32 dx = target.f32x - front.f32x;
	tFloat32 dy = target.f32y - front.f32y;


	//calc sign to steer in direction of road
	int sign = 1;

	//Calculate the angle of the vector.
	tFloat32 diff_heading_abs = wrapTo2Pi(atan2(dy, dx));

	//If the angle is left of the car (difference between car heading and vector-heading is < M_PI), the sign is positive, otherwise negative.
	if (wrapTo2Pi(diff_heading_abs - wrapTo2Pi(target.f32heading)) > M_PI) {
		sign = -1;
	}

	//calc normal distance of tangent to car (e)
	// double e = (target.getVector2d() - front.getVector2d()).norm() * sign;
	tFloat32 e = sqrt(dx*dx+dy*dy)*sign;

	//calc angle between car heading and point tangent
	//double theta_c = wrapTo2Pi(front.f32heading - target.f32heading);
	tFloat32 theta_c = wrapTo2Pi(target.f32heading - front.f32heading);

	//calc steering-angle with stanley-approach
	tFloat32 dynamicStanleyPart = 0;

	if (target.f32speed > 0.02)
	{
		dynamicStanleyPart = atan2(stanleyGain * e, target.f32speed);
	}

	tFloat32 vehicleSteeringAngle = theta_c + dynamicStanleyPart;

	if(m_debug_messages) {
		LOG_INFO("--------Stanley--------");
		LOG_INFO("POINT_Heading in °: %f", target.f32heading *rad2degree);
		LOG_INFO("Car Heading in °: %f", front.f32heading *rad2degree);
		LOG_INFO("Diff Heading in °: %f", diff_heading_abs *rad2degree);
		LOG_INFO("Stanley e: %f", e);
		LOG_INFO("Stanley Theta in °: %f", theta_c *rad2degree);
		LOG_INFO("Steering Angle in °: %f", vehicleSteeringAngle *rad2degree);
		LOG_INFO("-----------------------");
	}

	return vehicleSteeringAngle;

}