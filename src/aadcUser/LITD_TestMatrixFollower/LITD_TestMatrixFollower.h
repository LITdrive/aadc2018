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


#pragma once

//*************************************************************************************************
#define CID_COPENCVTEMPLATE_DATA_TRIGGERED_FILTER "litd_testmatrixfollower_filter.filter.user.aadc.cid"
#define SEARCH_SPACE_SIZE 20
#define CUTOUT_X 50
#define CUTOUT_Y 20

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;


/*! the main class of the open cv template. */
class cLITD_TestMatrixFollower : public cTriggerFunction
{
private:

    //Pins
    /*! Reader of an InPin. */
    cPinReader m_oReader;
    /*! Writer to an OutPin. */
    cPinWriter m_oWriter;
    cPinWriter m_oSteeringWriter;
    cPinWriter m_oSpeedWriter;

    char* PATH_TO_MAP = "/home/aadc/share/adtf/data/scaledMap.png";
    char* PATH_TO_PATH = "/home/aadc/share/adtf/data/scaledMapssw.png";
    
    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

    /*! The signal value sample factory */
    cSampleCodecFactory m_SignalValueSampleFactory;

    
    //Stream Formats
    /*! The input format */
    adtf::streaming::tStreamImageFormat m_sImageFormat;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    //Localization
    FineLocator m_locator = FineLocator(PATH_TO_MAP);

    //Path
    Mat m_Path;

    //Positioning
    int x = 515, y = 583;
    double heading = 0.0;
    double vel = 0.0 ;
    int imgcnt = 0;


public:

    /*! Default constructor. */
    cLITD_TestMatrixFollower();


    /*! Destructor. */
    virtual ~cLITD_TestMatrixFollower() = default;

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
