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

//otherwise cDOM will cause a deprecated warning, however there is no alternative yet
#define A_UTILS_NO_DEPRECATED_WARNING

#include "stdafx.h"
#include "CalibrationXML.h"
#include "ADTF3_helper.h"

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CALIBRATIONXML_DATA_TRIGGERED_FILTER,
    "Calibration XML",
    cCalibrationXML,
    adtf::filter::pin_trigger({ "input_value" }));

cCalibrationXML::cCalibrationXML()
{
    //Register Properties
    RegisterPropertyVariable("Configuration File For Interpolation", m_fileConfig);
    RegisterPropertyVariable("Border Warnings to Console", m_bDebugModeEnabled);
    RegisterPropertyVariable("Print initial table to Console", m_bPrintInitToConsole);

    //Get Media Descriptions
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    Register(m_oInputValue, "input_value", pTypeSignalValue);
    Register(m_oOutputValue, "output_value", pTypeSignalValue);
}

tResult cCalibrationXML::Configure()
{
    //load xml files for linear interpolation
    RETURN_IF_FAILED(LoadConfigurationData());

    //create class for cubic spline interpolation
    if (m_iCalibrationMode == 2)
    {
        m_cubicInterpolation = new Cubic(tInt(m_xValues.size()), m_xValues, m_yValues);
    }

    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cCalibrationXML::Process(tTimeStamp tmTimeOfTrigger)
{
    // Setpoint value speed
    object_ptr<const ISample> pInputValueSample;

    //write values with zero
    tFloat32 f32Value = 0;
    tUInt32 ui32TimeStamp = 0;

    if (IS_OK(m_oInputValue.GetNextSample(pInputValueSample)))
    {
        {
            auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pInputValueSample);

            RETURN_IF_FAILED(oDecoder.IsValid());

            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &ui32TimeStamp));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value));

            // doing the calibration
            if (m_iCalibrationMode == 1)
                f32Value = getLinearInterpolatedValue(f32Value);
            else if (m_iCalibrationMode == 2)
                f32Value = getCubicSplineInterpolatedValue(f32Value);
            else if (m_iCalibrationMode == 3)    //just for explanation
                f32Value = f32Value;
        }

        RETURN_IF_FAILED(transmitSignalValue(m_oOutputValue, m_pClock->GetStreamTime(), m_SignalValueSampleFactory,
            m_ddlSignalValueId.timeStamp, ui32TimeStamp, m_ddlSignalValueId.value, f32Value));
    } 

    RETURN_NOERROR;
}

tResult cCalibrationXML::LoadConfigurationData()
{
    cFilename fileCalibration = m_fileConfig;
    adtf::services::ant::adtf_resolve_macros(fileCalibration);
    // check if file exits
    if (fileCalibration.IsEmpty())
    {
        LOG_ERROR("cCalibrationXml: Configuration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    // create absolute path
    fileCalibration = fileCalibration.CreateAbsolutePath(".");

    //Load file, parse configuration, print the data
    if (cFileSystem::Exists(m_fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(fileCalibration);
        //load settings for calibration mode
        cDOMElementRefList oElemsSettings;
        if (IS_OK(oDOM.FindNodes("calibration/settings", oElemsSettings)))
        {
            for (cDOMElementRefList::iterator itElem = oElemsSettings.begin(); itElem != oElemsSettings.end(); ++itElem)
            {
                cDOMElement* pConfigElement;
                if (IS_OK((*itElem)->FindNode("mode", pConfigElement)))
                {

                    cString rdMode = pConfigElement->GetData();
                    LOG_INFO(adtf_util::cString::Format("cCalibrationXml: %s", rdMode.GetPtr()));
                    if (rdMode.Compare("linear") == 0)
                        m_iCalibrationMode = 1;
                    else if (rdMode.Compare("cubic") == 0)
                        m_iCalibrationMode = 2;
                    else if (rdMode.Compare("none") == 0)
                        m_iCalibrationMode = 3;
                }
            }
        }
        cString rdMode;
        switch (m_iCalibrationMode)
        {
        case 1:
            rdMode = "linear";
            break;
        case 2:
            rdMode = "cubic";
            break;
        case 3:
            rdMode = "none";
            break;
        }
        LOG_INFO(adtf_util::cString::Format("cCalibrationXml: Calibration mode is %s", rdMode.GetPtr()));
        //load supporting points
        if (m_iCalibrationMode != 3)
        {
            cDOMElementRefList oElems;
            if (IS_OK(oDOM.FindNodes("calibration/supportingPoints/point", oElems)))
            {
                for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
                {

                    cDOMElement* pConfigElement;
                    if (IS_OK((*itElem)->FindNode("xValue", pConfigElement)))
                    {
                        m_xValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
                    }
                    if (IS_OK((*itElem)->FindNode("yValue", pConfigElement)))
                    {
                        m_yValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
                    }
                }
            }
            if (oElems.size() > 0)
            {
                if (m_bPrintInitToConsole)
                {
                    for (tUInt i = 0; i < m_xValues.size(); i++)
                    {
                        if (i > m_yValues.size()) break;
                        LOG_INFO(cString::Format("cCalibrationXml: supportingPoint #%d: (%lf/%lf)", i, m_xValues[i], m_yValues[i]));
                    }
                }
            }
            else
            {
                LOG_ERROR("cCalibrationXml: no supporting points in given file found!");
                RETURN_ERROR(ERR_INVALID_FILE);
            }
            //checks if data are valid
            RETURN_IF_FAILED(CheckConfigurationData());
        }
    }
    else
    {
        LOG_ERROR("cCalibrationXml: Configured configuration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}

tResult cCalibrationXML::CheckConfigurationData()
{
    //checks if the xValues of the calibration table are increasing
    for (vector<tFloat32>::iterator it = m_xValues.begin(); it != m_xValues.end(); it++)
    {
        vector<tFloat32>::iterator it2 = it;
        it2++;
        if (it2 != m_xValues.end())
        {
            // next values is smaller than current value
            if ((tFloat32(*it) > tFloat32(*it2)))
            {
                LOG_ERROR(cString::Format("cCalibrationXml: The xValues in the file %s are not in increasing order. Please reorder the points!", static_cast<cFilename>(m_fileConfig).GetPtr()));
                RETURN_ERROR(ERR_INVALID_FILE);
            }
        }
    }

    RETURN_NOERROR;
}

tFloat32 cCalibrationXML::getLinearInterpolatedValue(tFloat32 fl32InputValue)
{
    // requested value is smaller than smallest value in table
    if (fl32InputValue < m_xValues.front())
    {
        if (m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("cCalibrationXml: requested x-value %f is lower than smallest x-value in calibration table", fl32InputValue));
        return m_yValues.front();
    }
    // requested value is bigger than biggest value in table
    else if (fl32InputValue > m_xValues.back())
    {
        if (m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("cCalibrationXml: requested x-value %f is higher than highes x-value in calibration table", fl32InputValue));
        return m_yValues.back();
    }
    // search in vector for corresponding index (smallest neighbor)
    tUInt iIndex;
    if (m_xValues.size() > 2)
    {
        for (iIndex = 0; iIndex < m_xValues.size(); iIndex++)
        {
            if (m_xValues[iIndex] >= fl32InputValue) break;
        }
        // get smaller neighbor
        if (iIndex != 0) iIndex = iIndex - 1;
    }
    else iIndex = 0;

    if ((m_xValues[iIndex + 1] - m_xValues[iIndex]) != 0)
    {
        // doing the linear interpolation
        tFloat32 f32Value = (fl32InputValue - m_xValues[iIndex])*(m_yValues[iIndex + 1] - m_yValues[iIndex]) / (m_xValues[iIndex + 1] - m_xValues[iIndex]) + m_yValues[iIndex];

        //tFloat32 security check to send only minimum or maximum value of table
        if (f32Value > *max_element(m_yValues.begin(), m_yValues.end()))
            return *max_element(m_yValues.begin(), m_yValues.end());
        else if (f32Value < *min_element(m_yValues.begin(), m_yValues.end()))
            return *min_element(m_yValues.begin(), m_yValues.end());
        else
            return f32Value;
    }
    else
    {
        LOG_ERROR("cCalibrationXml: invalid table in xml!");
        return 0;
    }
}

tFloat32 cCalibrationXML::getCubicSplineInterpolatedValue(tFloat32 fl32InputValue)
{
    // requested value is smaller than smallest value in table
    if (fl32InputValue < m_xValues.front())
    {
        if (m_bDebugModeEnabled) LOG_WARNING("cCalibrationXml: requested x-value lower than smallest x-value in calibration table");
        return m_yValues.front();
    }
    // requested value is bigger than biggest value in table
    else if (fl32InputValue > m_xValues.back())
    {
        if (m_bDebugModeEnabled) LOG_WARNING("cCalibrationXml: requested x-value higher than highes x-value in calibration table");
        return m_yValues.back();
    }
    // doing cubic interpolation
    return tFloat32(m_cubicInterpolation->getValue(tFloat64(fl32InputValue)));
}
