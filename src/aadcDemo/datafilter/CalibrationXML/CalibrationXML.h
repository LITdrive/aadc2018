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

#include "cCubic.h"

//*************************************************************************************************
#define CID_CALIBRATIONXML_DATA_TRIGGERED_FILTER "calibration_xml.filter.demo.aadc.cid"


using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;



/*! The main class of the calibration xml module */
class cCalibrationXML : public cTriggerFunction
{
private:

    /*! Property for the filename config */
    property_variable<cFilename> m_fileConfig = cFilename(cString("calibrationExampleFile.xml"));
    /*! The debug mode enabled property */
    property_variable<tBool> m_bDebugModeEnabled = tFalse;
    /*! The print initialize to console property */
    property_variable<tBool> m_bPrintInitToConsole = tFalse;
    
    //Pins
    /*! Input Pin reader for wheel struct*/
    cPinReader      m_oInputValue;

    /*! output pin writer for the the speed of the wheels */
    cPinWriter      m_oOutputValue;

    /*! the ddl identifier signalvalue . */
    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

    /*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*! holds the yValues for the supporting points*/
    vector<tFloat32> m_yValues;
    
    /*! holds the xValues for the supporting points*/
    vector<tFloat32> m_xValues;
    
    /*! The calibration mode */
    tInt m_iCalibrationMode;

    /*! The cubic interpolation class */
    Cubic *m_cubicInterpolation;

    /*!
     * Loads configuration data.
     *
     * \return  Standard Result Code.
     */
    tResult LoadConfigurationData();

    /*!
     * Check configuration data.
     *
     * \return  Standard Result Code.
     */
    tResult CheckConfigurationData();

    /*!
     * Gets linear interpolated value.
     *
     * \param   fl32InputValue  The fl 32 input value.
     *
     * \return  The linear interpolated value.
     */
    tFloat32 getLinearInterpolatedValue(tFloat32 fl32InputValue);

    /*!
     * Gets cubic spline interpolated value.
     *
     * \param   fl32InputValue  The fl 32 input value.
     *
     * \return  The cubic spline interpolated value.
     */
    tFloat32 getCubicSplineInterpolatedValue(tFloat32 fl32InputValue);

public:

    /*! Default constructor. */
    cCalibrationXML();
    
    /*! Destructor. */
    virtual ~cCalibrationXML() = default;

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
