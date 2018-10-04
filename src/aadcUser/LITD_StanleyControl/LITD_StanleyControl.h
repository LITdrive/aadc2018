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

//TODO:

//punkt zum regeln für stanley controller ein paar cm vor der achse annehmen. Sonst ist es möglich, dass der punkt leihct hinter der vorderachse liegt und somit nicht regelbar ist
//initial heading bei IMU kontrollieren, scheint nicht zu funktionieren.

#pragma once

#include "stdafx.h"
#include "LITD_VirtualPoint.h"
#include <aadc_structs.h>
#include "../utils/properties/FilePropertiesObserver.h"

//*************************************************************************************************
#define CID_STANLEY_CONTROL_FILTER "litd_stanley_control.filter.user.aadc.cid"
#define LABEL_STANLEY_CONTROL_FILTER "LITD StanleyControl"

#define VEHICLE_AXIS_DISTANCE  0.36 // in m 
#define TRAJECTORY_ARRAY_LEN 2
#define POINTS_PER_POLY 100

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class cStanleyControl : public cFilter
{
public:
	ADTF_CLASS_ID_NAME(cStanleyControl, CID_STANLEY_CONTROL_FILTER, LABEL_STANLEY_CONTROL_FILTER);

	// necessary for proper behaviour of the create_inner_pipe call
	using cRuntimeBehaviour::RegisterRunner;
	using cRuntimeBehaviour::RegisterInnerPipe;

private:

    void calcSteeringAngle();

	property_variable<cFilename> m_properties_file = cFilename("/home/aadc/share/adtf/configuration_files/properties/stanleycontrol_pid.ini");
	FilePropertiesObserver* m_properties;
    property_variable<tBool>       m_bShowDebug = tFalse;


	/* tPosition */
	struct
	{
		tSize f32x;
		tSize f32y;
		tSize f32radius;
		tSize f32speed;
		tSize f32heading;
	}  m_ddlPositionIndex;
	cSampleCodecFactory m_PositionSampleFactory;
	cPinReader m_ActualPointReader;

	/* tTrajectoryArray */
	struct
	{
		tSize size;
		tSize trajectories;
	} m_ddlTrajectoryArrayIndex{};

	cSampleCodecFactory m_TrajectoryArraySampleFactory;
	cPinReader m_TrajectoryArrayReader;

	/* tSignalValue */
	struct tSignalValueId
	{
		tSize timeStamp;
		tSize value;
	} m_ddlSignalValueId;
	cSampleCodecFactory m_SignalValueSampleFactory;
	cPinWriter m_SteeringWriter;

	/*// Parameters of Polynomial
	tFloat64 poly_x_a, poly_x_b, poly_x_c, poly_x_d, poly_y_a, poly_y_b, poly_y_c, poly_y_d;
	// Actual vehicle position
    tFloat64 vehicleActualPosition_x, vehicleActualPosition_y, vehicleActualHeading, vehicleActualSpeed;
	// Target vehicle position
	tFloat64 vehicleActualPosition_x, vehicleActualPosition_y, vehicleActualHeading, vehicleActualSpeed;*/
    tFloat64 vehicleSteeringAngle;
	tFloat64 vehicleSpeed;
	int actual_min_dist_poly_index = 0;
	int last_min_dist_poly_index = 0;
	bool poly_completed;
	//::tPosition vehicleActualPosition;
	::tTrajectory trajectoryArray[TRAJECTORY_ARRAY_LEN];
    LITD_VirtualPoint vehicleActualRearAxlePosition, vehicleActualFrontAxlePosition, vehicleTargetFrontAxlePosition;

    //controller params
    double stanleyGain = 1.5;
	double maxAngle = 45;

public:

    /*! Default constructor. */
    cStanleyControl();

    /*! Destructor. */
    virtual ~cStanleyControl() = default;

	tResult Init(tInitStage eStage) override;

    tResult Configure();

    tResult ProcessTrajectories(tTimeStamp tmTimeOfTrigger);

	void calculateActualFrontAxlePosition();

	//void calculateFrontAxlePosition(LITD_VirtualPoint rearAxlePosition, LITD_VirtualPoint* frontAxlePosition);

	tResult ProcessPosition(tTimeStamp tmTimeOfTrigger);

	void updatePolyList(tTrajectory trajectory);

	//void getNextVirtualPointOnPoly(tTrajectory trajectories[], uint8_t polyLen, tTrajectory* idealPolyPoint, LITD_VirtualPoint* idealPoint, LITD_VirtualPoint carPosition);

	//void getNextVirtualPointOnPoly(tTrajectory trajectories[], LITD_VirtualPoint carPosition);

	void getNextVirtualPointOnPoly();

	void calcVirtualPointfromPoly(tTrajectory poly, double p, LITD_VirtualPoint * vp);

	void mapSteeringAngle();

	//void calcVirtualPointfromPoly(tTrajectory * poly, double p, LITD_VirtualPoint * vp);


};


//*************************************************************************************************
