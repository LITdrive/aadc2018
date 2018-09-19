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

#pragma once

#include "LITD_VirtualPoint.h"
#include "../utils/properties/FilePropertiesObserver.h"

//*************************************************************************************************
#define CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER "template_filter.filter.user.aadc.cid"



using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class cStanleyControl : public cTriggerFunction
{
private:

    #define rad2degree  180.0 / M_PI;
    void calcSteeringAngle();
    void mapSteeringAngle();

    /*! Media Descriptions. */
   struct tPositionIndex
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    }m_ddlPositionIndex;

   /*! The template data sample factory */
   cSampleCodecFactory m_VirtualPointSampleFactory;

   struct tSignalValueId
   {
	   tSize timeStamp;
	   tSize value;
   } m_ddlSignalValueId{};

	// signal value
	cSampleCodecFactory m_SignalValueSampleFactory;


    /*! Reader of an InPin. */
    cPinReader m_oVPReaderIst;
    cPinReader m_oVPReaderSoll;
    /*! Writer to an OutPin. */
    cPinWriter m_oWriter;

    tFloat32 carX, carY, carHeading, carSpeed, sollX, sollY, sollHeading, sollSpeed;
    tFloat32 carSteeringAngle, carSteeringValue;

    LITD_VirtualPoint vp;
    LITD_VirtualPoint carPosition;

    //controller params
    double stanleyGain = 1.5;
    double maxAngle = 45;

    property_variable<cFilename> m_properties_file = cFilename("../../../../configuration_files/properties/stanleycontrol_pid.ini");
	FilePropertiesObserver* m_properties;
    property_variable<tBool>       m_bShowDebug = tFalse;


	/*! clock service */
	object_ptr<adtf::services::IReferenceClock> m_pClock;



public:

    /*! Default constructor. */
    cStanleyControl();

    /*! Destructor. */
    virtual ~cStanleyControl() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure() override;
    /**
    * Overwrites the Process
    * You need to implement the Reading and Writing of Samples within this function
    * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
    * This FUnction will be called if the Run() of the TriggerFunction was called.
    */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;

};


//*************************************************************************************************
