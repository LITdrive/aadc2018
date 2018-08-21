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
#define CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER "classifypedestrian_filter.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class cClassifyPedestrian : public cTriggerFunction
{
private:
    /*! Media Descriptions. */
    struct ddlTrackletLocation
    {
        tSize StartPoint;
        tSize TrackletDimension;
    } m_ddlTrackletLocationId;

    struct ddlClassifierResult
    {
        tSize ClassifierResult;
    } m_ddlClassifierResultId;

    // LASER SCANNER
    struct ddlLaserScannerDataId
    {
        tSize ui32Size;
        tSize scanArray;
    } m_ddlLSDataId;

    struct ddlPolarCoordinateId
        {
            tSize radius;
            tSize angle;
        } m_ddlPolarCoordinateId;

    /*! The template data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_TrackletLocationSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_polarCoordinateSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_LSStructSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_ClassifierResultSampleFactory;
    /*! Reader of an LS. */
    cPinReader m_oInputLaserScanner;
    /*! Reader of an InPin. */
    cPinReader m_oTrackletLocation;
    /*! Writer to an OutPin. */
    cPinWriter m_oWriter;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    //bool isEngaged;

public:

    /*! Default constructor. */
    cClassifyPedestrian();

    /*! Destructor. */
    virtual ~cClassifyPedestrian() = default;

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
