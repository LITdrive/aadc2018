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
#include "ClassifyPedestrian.h"
#include <aadc_structs.h>

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
    "LITD ClassifyPedestrian",
    cClassifyPedestrian,
    adtf::filter::pin_trigger({"input"}));


cClassifyPedestrian::cClassifyPedestrian()
{
    //isEngaged = false;

    object_ptr<IStreamType> pTypeTrackletLocation;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tTrackletLocation", pTypeTrackletLocation, m_TrackletLocationSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_TrackletLocationSampleFactory, "StartPoint", m_ddlTrackletLocationId.StartPoint);
        adtf_ddl::access_element::find_index(m_TrackletLocationSampleFactory, "TrackletDimension", m_ddlTrackletLocationId.TrackletDimension);
    }
    else
    {
        LOG_WARNING("No mediadescription for tTemplateData found!");
    }
    Register(m_oTrackletLocation, "TrackletLocation" , pTypeTrackletLocation);

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
    Register(m_oInputLaserScanner, "laser_scanner", pTypeLSData);

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

    object_ptr<IStreamType> pTypePedestrianClassifierResult;
    if (IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPedestrainClassifierResult", pTypePedestrianClassifierResult, m_ClassifierResultSampleFactory)))
    {
        //get all the member indices

        adtf_ddl::access_element::find_index(m_ClassifierResultSampleFactory, "ClassifierResult", m_ddlClassifierResultId.ClassifierResult);
    }
    else
    {
        LOG_INFO("No mediadescription for tPolarCoordiante found!");
    }

    Register(m_oWriter, "Pedestrian Classifier Result", pTypePedestrianClassifierResult);



}


//implement the Configure function to read ALL Properties
tResult cClassifyPedestrian::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult cClassifyPedestrian::Process(tTimeStamp tmTimeOfTrigger)
{
    tBool OutputResult = 0;
    object_ptr<const ISample> pLSSample;
    tSize sizeofDataPoints = 0;
    object_ptr<const ISample> pTLSample;

    if (IS_OK(m_oTrackletLocation.GetLastSample(pTLSample)))
    {
        auto oDecoder2 = m_TrackletLocationSampleFactory.MakeDecoderFor(*pTLSample);

        RETURN_IF_FAILED(oDecoder2.IsValid());


        const tVoid* ptr1 = oDecoder2.GetElementAddress(m_ddlTrackletLocationId.StartPoint);
        const tFloat32* ptrSP = reinterpret_cast<const tFloat32*>(ptr1);

        const tVoid* ptr2 = oDecoder2.GetElementAddress(m_ddlTrackletLocationId.TrackletDimension);
        const tFloat32* ptrTD = reinterpret_cast<const tFloat32*>(ptr2);
        //to be fixed
        tFloat32 referenceDist = 100;
        tFloat32 referenceTall = 10;
        tFloat32 referenceShort = 3;

        tFloat32 PedestrianDepth = 0;
        tFloat32 PedestrianNoPoints = 0;
        tFloat32 InitCol = ptrSP[0];
        tFloat32 PedestrianHeight = ptrTD[1];
        tFloat32 FinCol = ptrSP[0] + ptrTD[0];
        tFloat32 midCol = 480;
        //init theta is negative if it is left of center
        tFloat32 Inittheta  = 90 - acos((InitCol - midCol)/midCol);
        tFloat32 Fintheta   = 90 - acos((FinCol -  midCol)/midCol);

        if (IS_OK(m_oInputLaserScanner.GetLastSample(pLSSample)))
        {
            auto oDecoder = m_LSStructSampleFactory.MakeDecoderFor(*pLSSample);

            RETURN_IF_FAILED(oDecoder.IsValid());

            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLSDataId.ui32Size, &sizeofDataPoints));

            const tVoid* ptr = oDecoder.GetElementAddress(m_ddlLSDataId.scanArray);
            const tPolarCoordiante* pCoordinates = reinterpret_cast<const tPolarCoordiante*>(ptr);

            if (Inittheta < 0 && Fintheta < 0)
            {
                for (tSize i = 0; i < sizeofDataPoints; i++)
                {
                    if (pCoordinates[i].f32Angle >= (360 - Inittheta) && pCoordinates[i].f32Angle <= (360 - Fintheta))
                    {
                        PedestrianNoPoints += 1;
                        PedestrianDepth += pCoordinates[i].f32Radius;
                    }
                }
                PedestrianDepth /= PedestrianNoPoints;
            }

            else if(Inittheta > 0 && Fintheta > 0)
            {
                for (tSize i = 0; i < sizeofDataPoints; i++)
                {
                    if (pCoordinates[i].f32Angle >= (Inittheta) && pCoordinates[i].f32Angle <= Fintheta)
                    {
                        PedestrianNoPoints += 1;
                        PedestrianDepth += pCoordinates[i].f32Radius;
                    }
                }
                PedestrianDepth /= PedestrianNoPoints;
            }
            else // Inittheta < 0 && Fintheta > 0)
            {
                for (tSize i = 0; i < sizeofDataPoints; i++)
                {

                    {
                        PedestrianNoPoints += 1;
                        PedestrianDepth += pCoordinates[i].f32Radius;
                    }
                }
                PedestrianDepth /= PedestrianNoPoints;
            }

            tFloat32 expectedHeightTall  = (referenceTall*PedestrianDepth)/referenceDist;
            tFloat32 expectedHeightShort = (referenceShort*PedestrianDepth)/referenceDist;
            if (abs(expectedHeightTall - PedestrianHeight) < abs(expectedHeightShort - PedestrianHeight))
            {//tall pedestrian
                OutputResult = 1;
            }

        }
    }


    //write output


    object_ptr<ISample> pClassifierResultSample;

    if (IS_OK(alloc_sample(pClassifierResultSample)))
    {

        auto oCodec = m_ClassifierResultSampleFactory.MakeCodecFor(pClassifierResultSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlClassifierResultId.ClassifierResult, OutputResult));

    }
    m_oWriter << pClassifierResultSample << flush << trigger;

    RETURN_NOERROR;
}
