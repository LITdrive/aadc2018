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

void cStanleyControl::calcSteeringAngle(){
    double rad2degree = 180.0 / M_PI;
    //vector between car and virtualpoint
	// call getNextVirtualPointOnPoly -> Result is vehicleTargetFrontAxlePosition
    Vector2d diff = vehicleTargetFrontAxlePosition.getVector2d() - vehicleActualFrontAxlePosition.getVector2d();

    //calc sign to steer in direction of road
    int sign = 1;
    double diff_heading_abs = wrapTo2Pi(atan2(diff(1), diff(0)));
    if(wrapTo2Pi(diff_heading_abs - wrapTo2Pi(vehicleTargetFrontAxlePosition.h)) > M_PI){
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

	if(vehicleSpeed > 0.02)
	{
		 dynamicStanleyPart = atan2(stanleyGain * e, vehicleSpeed);
	}

    vehicleSteeringAngle = theta_c + dynamicStanleyPart;

    //Debug Messages
    /*std::cout << "-----------------------" << std::endl;
    std::cout << "point heading : " << vp.h << "(" << rad2degree * vp.h << "°)" << std::endl;
    std::cout << "car heading: " << carPosition.h << "(" << rad2degree * carPosition.h << "°)" << std::endl;
    std::cout << "diff heading: " << diff_heading_abs << "(" << rad2degree * diff_heading_abs << "°)" << std::endl;
    std::cout << "e: " << e << std::endl;
    std::cout << "Theta_C: " << theta_c << "(" << rad2degree * theta_c << "°)" << std::endl;
    std::cout << "Steering Angle: " << carSteeringAngle << "(" << rad2degree * carSteeringAngle << "°)" << std::endl;
    std::cout << "-----------------------" << std::endl;*/
	LOG_INFO("-----------------------");
	LOG_INFO("POINT_Heading in °: %f", vehicleTargetFrontAxlePosition.h *rad2degree);
	LOG_INFO("Car Heading in °: %f", vehicleActualFrontAxlePosition.h *rad2degree);
	LOG_INFO("Diff Heading in °: %f", diff_heading_abs *rad2degree);
	LOG_INFO("Stanley e: %f", e);
	LOG_INFO("Stanley Theta in °: %f", theta_c *rad2degree);
	LOG_INFO("Steering Angle in °: %f", vehicleSteeringAngle *rad2degree);
	LOG_INFO("-----------------------");

}

cStanleyControl::cStanleyControl()
{
	/* tPosition */
	object_ptr<IStreamType> pTypePositionData;

	RegisterPropertyVariable("dynamic properties path", m_properties_file);

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
	/*object_ptr<IStreamType> pTypeTrajectoryData;
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
	}*/

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
	//create_pin(*this, m_TrajectoryReader, "inTrajectories", pTypeTrajectoryData);
	//Register output pin
	filter_create_pin(*this, m_SteeringWriter, "steering", pTypeSignalValue);
		
	create_inner_pipe(*this, cString::Format("%s_trigger", "actual_point"), "actual_point", [&](tTimeStamp tmTime) -> tResult
	{
		LOG_INFO("Hello out of inner pipe" );
		//return ProcessTrajectories(tmTime);
		return ProcessPosition(tmTime);
	});

	create_inner_pipe(*this, cString::Format("%s_trigger", "trajectories"), "trajectories", [&](tTimeStamp tmTime) -> tResult
	{
		return ProcessPosition(tmTime);
	});
}

//implement the Configure function to read ALL Properties
tResult cStanleyControl::Configure()
{
	LOG_INFO("Configure is running now!");

	// load properties file for dynamic properties
	cFilename propertiesFileResolved = m_properties_file;
	adtf::services::ant::adtf_resolve_macros(propertiesFileResolved);
	m_properties = new FilePropertiesObserver(propertiesFileResolved.GetPtr());
	m_properties->ReloadProperties();

	// Fixed Polynomials of a cirle arc and a straight for testing
	// Circle Arc
	// 0.3443 x + 0.3115 x - 1.656 x + 0.363
	trajectoryArray[1].start = 0;
	trajectoryArray[1].end = 1;
	trajectoryArray[1].ax = 0.702;
	trajectoryArray[1].bx = 1.656;
	trajectoryArray[1].cx = -0.3115;
	trajectoryArray[1].dx = 0.3443;
	// -0.3443 x + 1.344 x + 2.22e-16
	trajectoryArray[1].ay = 0;
	trajectoryArray[1].by = 0;
	trajectoryArray[1].cy = 1.344;
	trajectoryArray[1].dy = 0.3443;
	// straight
	// -0.339 x + 0.702
	trajectoryArray[0].start = 0;
	trajectoryArray[0].end = 1;
	trajectoryArray[0].ax = 0.363;
	trajectoryArray[0].bx =  0.339;
	trajectoryArray[0].cx = 0;
	trajectoryArray[0].dx = 0;
	trajectoryArray[0].ay = 0;
	trajectoryArray[0].by = 0;
	trajectoryArray[0].cy = 0;
	trajectoryArray[0].dy = 0;

    RETURN_NOERROR;
}

tResult cStanleyControl::ProcessPosition(tTimeStamp tmTimeOfTrigger)
{
	::tPosition position;
	LOG_INFO("Hellof from ProcessPosition" );

	//Read Property-File
	m_properties->TriggerPropertiesReload(80); // reload the file every 2 seconds with a 25 msec timer
	stanleyGain = m_properties->GetFloat("stanley_gain");
    maxAngle = m_properties->GetFloat("max_angle");

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
	LOG_INFO("Point of BackPosition: x: %f, y: %f, h: %f", vehicleActualRearAxlePosition.x, vehicleActualRearAxlePosition.y, vehicleActualRearAxlePosition.h  * 180.0 / M_PI);


	calculateActualFrontAxlePosition();
	LOG_INFO("Point of FrontPosition: x: %f, y: %f, h: %f", vehicleActualFrontAxlePosition.x, vehicleActualFrontAxlePosition.y, vehicleActualFrontAxlePosition.h * 180.0 / M_PI );
	getNextVirtualPointOnPoly();
	LOG_INFO("Point of SetPoint: x: %f, y: %f, h: %f", vehicleTargetFrontAxlePosition.x, vehicleTargetFrontAxlePosition.y, vehicleTargetFrontAxlePosition.h * 180.0 / M_PI );
	calcSteeringAngle();
	LOG_INFO("SteeringAngle in grad: %f", vehicleSteeringAngle * 180.0 / M_PI );
	mapSteeringAngle();
	

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

	// TODO Not needed for testing, uncomment afterwards
	//updatePolyList(trajectory);
	// TODO trajectory -> trajectories

	RETURN_NOERROR;
}

void cStanleyControl::calculateActualFrontAxlePosition() {
	double dx = cos(vehicleActualRearAxlePosition.h)*VEHICLE_AXIS_DISTANCE;
	double dy = sin(vehicleActualRearAxlePosition.h)*VEHICLE_AXIS_DISTANCE;

	vehicleActualFrontAxlePosition.x = vehicleActualRearAxlePosition.x + dx;
	vehicleActualFrontAxlePosition.y = vehicleActualRearAxlePosition.y + dy;
	vehicleActualFrontAxlePosition.h = vehicleActualRearAxlePosition.h;
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

	if (poly_completed)
	{
		trajectoryArray[0] = trajectoryArray[last_min_dist_poly_index];
		last_min_dist_poly_index = actual_min_dist_poly_index;

		for (int i = 1; i < TRAJECTORY_ARRAY_LEN; i++)
		{
			// TODO: trajectoryArray[i] = trajectories[i]
			trajectoryArray[i] = trajectory;
		}
	}

	else
	{
		for (int i = 0; i < TRAJECTORY_ARRAY_LEN; i++)
		{
			// TODO: trajectoryArray[i] = trajectories[i]
			trajectoryArray[i] = trajectory;
		}
	}	
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

void cStanleyControl::getNextVirtualPointOnPoly() {

	LITD_VirtualPoint actualPoint;
	double min_dist = DBL_MAX;
	//min_dist_poly_index = 0;
	double min_poly_p = 0;
	poly_completed = false;
	double min_dist_x = DBL_MAX;
	double min_dist_y = DBL_MAX;
	double min_dist_h = 0;

	for (int i = 0; i<TRAJECTORY_ARRAY_LEN; i++)
	{
		for (double j = trajectoryArray[i].start; j <= trajectoryArray[i].end; j+=(trajectoryArray[i].end-trajectoryArray[i].start)/POINTS_PER_POLY)
		{
			// p = [0, 1]
			calcVirtualPointfromPoly(trajectoryArray[i], j, &actualPoint);

			// Target Point on Polynom has to be in front of the actual position of the front axle
			//if ((vehicleActualFrontAxlePosition.h == 0.0  && actualPoint.x > vehicleActualFrontAxlePosition.x) || (vehicleActualFrontAxlePosition.h == M_PI && actualPoint.x < vehicleActualFrontAxlePosition.x) || (vehicleActualFrontAxlePosition.h == M_PI/2 && actualPoint.y > vehicleActualFrontAxlePosition.y) || (vehicleActualFrontAxlePosition.h == (3/2)*M_PI && actualPoint.y > vehicleActualFrontAxlePosition.y) || (vehicleActualFrontAxlePosition.h > 0 && vehicleActualFrontAxlePosition.h < M_PI/2 && actualPoint.x > vehicleActualFrontAxlePosition.x && actualPoint.y > vehicleActualFrontAxlePosition.y) || (vehicleActualFrontAxlePosition.h > M_PI/2 && vehicleActualFrontAxlePosition.h < M_PI && actualPoint.x < vehicleActualFrontAxlePosition.x && actualPoint.y > vehicleActualFrontAxlePosition.y) || (vehicleActualFrontAxlePosition.h > M_PI && vehicleActualFrontAxlePosition.h < (3/2)*M_PI && actualPoint.x < vehicleActualFrontAxlePosition.x && actualPoint.y < vehicleActualFrontAxlePosition.y) || (vehicleActualFrontAxlePosition.h > (3/2)*M_PI && vehicleActualFrontAxlePosition.h < 2*M_PI && actualPoint.x > vehicleActualFrontAxlePosition.x && actualPoint.y < vehicleActualFrontAxlePosition.y))
			//if ((vehicleActualFrontAxlePosition.h >= (3/2)*M_PI && vehicleActualFrontAxlePosition.h <= M_PI/2 && actualPoint.x > vehicleActualFrontAxlePosition.x) || (vehicleActualFrontAxlePosition.h >= M_PI/2 && vehicleActualFrontAxlePosition.h <= (3/2)*M_PI && actualPoint.x < vehicleActualFrontAxlePosition.x) || (vehicleActualFrontAxlePosition.h >= 0 && vehicleActualFrontAxlePosition.h <= M_PI && actualPoint.y > vehicleActualFrontAxlePosition.y) || (vehicleActualFrontAxlePosition.h >= M_PI && vehicleActualFrontAxlePosition.h <= 2*M_PI && actualPoint.y < vehicleActualFrontAxlePosition.y))
			//{
			
			//calc vector from carfrontposition to actualpoint
			double x_vec = actualPoint.x - vehicleActualFrontAxlePosition.x;
			double y_vec = actualPoint.y - vehicleActualFrontAxlePosition.y;
			double angleFromCarToActualPoint = atan2(y_vec, x_vec);
			//LOG_INFO("Angle from Car to NextPoint: %f", angleFromCarToActualPoint * 180.0 / M_PI );
			if(angleFromCarToActualPoint <= M_PI/2 || angleFromCarToActualPoint >= 3/2 * M_PI){

				//calc norm to carPosition
				double dist = sqrt(pow(actualPoint.x - vehicleActualFrontAxlePosition.x, 2) + pow(actualPoint.y - vehicleActualFrontAxlePosition.y, 2));
				//LOG_INFO("In fancy logic-if" );
				if (dist < min_dist)
				{
					min_dist = dist;
					actual_min_dist_poly_index = i;
					min_poly_p = j;
					vehicleTargetFrontAxlePosition.x = actualPoint.x;
					vehicleTargetFrontAxlePosition.y = actualPoint.y;
					vehicleTargetFrontAxlePosition.h = actualPoint.h;

					if (trajectoryArray[i].backwards)
					{
						vehicleTargetFrontAxlePosition.h = wrapTo2Pi(vehicleTargetFrontAxlePosition.h + M_PI);
					}
				}
			}
			//}
		}
	}
	LOG_INFO("Point from Poly is: x: %f, y: %f, h: %f", vehicleTargetFrontAxlePosition.x, vehicleTargetFrontAxlePosition.y, vehicleTargetFrontAxlePosition.h * 180.0 / M_PI );
	LOG_INFO("End of trajektorie loop" );

	if (actual_min_dist_poly_index != last_min_dist_poly_index)
	{
		poly_completed = true;
		//last_min_dist_poly_index = actual_min_dist_poly_index;
	}

	// Function value and ID of Poly with smallest distance to given car point
	// idealPolyPoint->id = trajectories[actual_min_dist_poly_index].id;
	// TODO: not compiling, there is no parameter p
	// idealPolyPoint->p = min_poly_p;

	// Point on Poly with smallest distance to given car point
	//idealPoint->x = min_dist_x;
	//idealPoint->y = min_dist_y;
	//idealPoint->h = min_dist_h;
}

void cStanleyControl::calcVirtualPointfromPoly(tTrajectory poly, double p, LITD_VirtualPoint* vp) {
	double heading = 0;

	double x = poly.dx*pow(p, 3) + poly.cx*pow(p, 2) + poly.bx*p + poly.ax;
	double y = poly.dy*pow(p, 3) + poly.cy*pow(p, 2) + poly.by*p + poly.ay;

	// Poly ist Gerade
	if (poly.dx == 0.0 && poly.cx == 0)
	{
		if (poly.bx > 0 && poly.by == 0)
		{
			heading = 0.0;
		}

		else if (poly.bx < 0 && poly.by == 0)
		{
			heading = M_PI;
		}

		else if (poly.by > 0 && poly.bx == 0)
		{
			heading = M_PI / 2;
		}

		else if (poly.by < 0 && poly.bx == 0)
		{
			heading = (3 / 2)*M_PI;
		}
	}

	else
	{
		// Poly ist keine Gerade
		double x_der = 3 * poly.dx*pow(p, 2) + 2 * poly.cx*p + poly.bx;
		double y_der = 3 * poly.dy*pow(p, 2) + 2 * poly.cy*p + poly.by;

		heading = wrapTo2Pi(atan2(y_der, x_der));
	}
	//LOG_INFO("Derivation Points: x_der: %f, y_der: %f from h: %f", x_der, y_der, heading );
	//double heading = wrapTo2Pi(atan2(y_p-y_m, x_p-x_m));

	

	vp->x = x;
	vp->y = y;
	vp->h = heading;
}

void cStanleyControl::mapSteeringAngle(){

	// it should be regardless for this function if the angle is between -90 and +90 degree or 270 to 90 degree

	tFloat32  rad2degree  = 180.0 / M_PI;

	if(vehicleSteeringAngle > 3/2 * M_PI && vehicleSteeringAngle < 2*M_PI){
		vehicleSteeringAngle -= 2 * M_PI; 
	}

	 if(vehicleSteeringAngle < -M_PI/4){
            vehicleSteeringAngle = -M_PI/4;
            LOG_INFO("Steering angle truncated to -45°!");
    } else if(vehicleSteeringAngle > M_PI/4){
            vehicleSteeringAngle = M_PI/4;
            LOG_INFO("Steering angle truncated to 45°!");
    }


    vehicleSteeringAngle = (vehicleSteeringAngle * rad2degree) / maxAngle * (-100); 
    LOG_INFO("SteeringAngle in steps: %f", vehicleSteeringAngle );

}