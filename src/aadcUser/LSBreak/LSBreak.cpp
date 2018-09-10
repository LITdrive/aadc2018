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

#include "stdafx.h"
#include "LSBreak.h"
#include <aadc_structs.h>

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
    "LSBreak_cf",
    cLSBreak,
    adtf::filter::pin_trigger({"input"}));


cLSBreak::cLSBreak()
{
    isEngaged = false;
    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeTemplateData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeTemplateData, m_signalDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_signalDataSampleFactory, cString("f32Value"), m_ddlSignalValueId.f32value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tTemplateData found!");
    }


    Register(m_oReader, "input" , pTypeTemplateData);
    Register(m_oWriter, "output", pTypeTemplateData);


    object_ptr<IStreamType> pTypeLSData;
    if (IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaserScannerData", pTypeLSData, m_LSStructSampleFactory)))
    {
        // find the indexes of the element for faster access to the process mtehod
        LOG_INFO("Found mediadescr. for tLaserScannerData!");
        adtf_ddl::access_element::find_index(m_LSStructSampleFactory, "ui32Size", m_ddlLSDataId.ui32Size);
        adtf_ddl::access_element::find_array_index(m_LSStructSampleFactory, "tScanArray", m_ddlLSDataId.scanArray);
    }
    else
    {
        LOG_INFO("No mediadescr. for the tLaserScannerData found :( ");
    }
    object_ptr<IStreamType> pTypePolarCoord;
    if (IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPolarCoordiante", pTypePolarCoord, m_polarCoordinateSampleFactory)))
    {
        //get all the member indices

        adtf_ddl::access_element::find_index(m_polarCoordinateSampleFactory, "f32Radius", m_ddlPolarCoordinateId.radius);
        adtf_ddl::access_element::find_index(m_polarCoordinateSampleFactory, "f32Angle", m_ddlPolarCoordinateId.angle);
    }
    else
    {
        LOG_INFO("No mediadescription for tPolarCoordiante found!");
    }
    Register(m_oInputLaserScanner, "laser_scanner", pTypeLSData);
}


//implement the Configure function to read ALL Properties
tResult cLSBreak::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult cLSBreak::Process(tTimeStamp tmTimeOfTrigger)
{

    cout << "reached1 \n";
    object_ptr<const ISample> pLSSample;
    tSize size = 0;
    float safeDistance = 350.0;

    if (IS_OK(m_oInputLaserScanner.GetLastSample(pLSSample))){
        auto oDecoder2 = m_LSStructSampleFactory.MakeDecoderFor(*pLSSample);

        RETURN_IF_FAILED(oDecoder2.IsValid());

        RETURN_IF_FAILED(oDecoder2.GetElementValue(m_ddlLSDataId.ui32Size, &size));

        const tVoid* ptr = oDecoder2.GetElementAddress(m_ddlLSDataId.scanArray);
        const tPolarCoordiante* pCoordinates = reinterpret_cast<const tPolarCoordiante*>(ptr);

        cout << "reached2 \n";
        //tPolarCoordiante scanPoint;
        tFloat32 rmin = 12000;
        int angle = 50;
        for (tSize i = 0; i < size; i++) {
            //scanPoint = pCoordinates[i];
            if (pCoordinates[i].f32Angle < angle || (pCoordinates[i].f32Angle > (360 - angle))){
                if (pCoordinates[i].f32Radius < rmin && pCoordinates[i].f32Radius > 1){
                    rmin = pCoordinates[i].f32Radius;
                }
            }
        }

        cout << rmin << "\n";
        isEngaged = rmin < safeDistance;
        if (isEngaged){
            LOG_INFO("Saftey break engaged");
        }
    }
    object_ptr<const ISample> pReadSample;

    tFloat32 inputData;
    //tTimeStamp time;
    if (IS_OK(m_oReader.GetLastSample(pReadSample)))
    {
        auto oDecoder = m_signalDataSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.f32value, &inputData));
        LOG_INFO("Got Input");
        //RETURN_IF_FAILED_DESC(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &time),"Failed@retrieving time %ld",time);
        LOG_INFO("Got timestamp");
        // Do the Processing
        tFloat32 outputData = inputData * (!isEngaged);
        cout << "read Input successfully";
        object_ptr<ISample> pWriteSample;

        if (IS_OK(alloc_sample(pWriteSample)))
        {

            auto oCodec = m_signalDataSampleFactory.MakeCodecFor(pWriteSample);

            RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.f32value, outputData));

            //RETURN_IF_FAILED_DESC(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, m_pClock->GetStreamTime()), "Failed@setting time %ld",m_pClock->GetStreamTime());

        }
        m_oWriter << pWriteSample << flush << trigger;
        cout << "Wrote to output: "<< outputData << " @" << m_pClock->GetStreamTime() << "  " << isEngaged << "\n";
    }





    RETURN_NOERROR;
}
