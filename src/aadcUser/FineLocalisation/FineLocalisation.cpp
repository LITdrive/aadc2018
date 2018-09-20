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
#include "FineLocalisation.h"
#include "ADTF3_OpenCV_helper.h"
#include "FineLocator.h"


#define DEG2RAD M_PI/180

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CBIRDS_EYE_VIEW_DATA_TRIGGERED_FILTER,
                                    "FineLocalisation_cf",
                                    cFineLocalisation,
                                    adtf::filter::pin_trigger({ "inBirdsEye" }));

cFineLocalisation::cFineLocalisation()
{

    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    Register(m_oReader, "inBirdsEye", pType);

    object_ptr<IStreamType> pTypePositionData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_PositionSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32x", m_ddlPositionIndex.x);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32y", m_ddlPositionIndex.y);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32radius", m_ddlPositionIndex.radius);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32speed", m_ddlPositionIndex.speed);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32heading", m_ddlPositionIndex.heading);
    }
    else
    {
        LOG_WARNING("No mediadescription for tPosition found!");
    }

    //Register input pin
    Register(m_oPosReader, "inPosition", pTypePositionData);

    //Register output pin
    Register(m_oPosWriter, "outPosition", pTypePositionData);

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sImageFormat, *pType.Get());
    });

    RegisterPropertyVariable("AffineMat@00", mat00);
    RegisterPropertyVariable("AffineMat@01", mat01);
    RegisterPropertyVariable("AffineMat@02", mat02);
    RegisterPropertyVariable("AffineMat@10", mat10);
    RegisterPropertyVariable("AffineMat@11", mat11);
    RegisterPropertyVariable("AffineMat@12", mat12);
    RegisterPropertyVariable("Path to Map", mapPath);
    RegisterPropertyVariable("search space size", propSearchSpaceSize);
    RegisterPropertyVariable("distance from back axle to Picture bottom [m]", axleToPicture);
    RegisterPropertyVariable("offset for Heading [°]", headingOffset);


}

tResult cFineLocalisation::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    string pathToMap = static_cast<string>(cString(mapPath));
    locator.setMap(const_cast<char*>(pathToMap.c_str()));
    searchSpaceSize = propSearchSpaceSize;
    affineMat[0][0] = mat00;
    affineMat[0][1] = mat01;
    affineMat[0][2] = mat02;
    affineMat[1][0] = mat10;
    affineMat[1][1] = mat11;
    affineMat[1][2] = mat12;
    pmt = PixelMetricTransformer(affineMat);
    RETURN_NOERROR;
}

tResult cFineLocalisation::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample, pVPReadSample;

    if(IS_OK(m_oPosReader.GetLastSample(pVPReadSample))) {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pVPReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &x));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &y));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &heading));
        //adjust heading for inversion of back camera
        heading = heading + headingOffset*DEG2RAD;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.speed, &speed));
    }

    while (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat bvImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height), CV_8UC3, const_cast<unsigned char*>(static_cast<const unsigned char*>(pReadBuffer->GetPtr())));
            float* px_loc = pmt.toPixel(x + axleToPicture*cos(heading), y + axleToPicture*sin(heading));

            float px_x = px_loc[0], px_y = px_loc[1];
            Point3f location = locator.localize(bvImage, heading, Point2i(px_x, px_y), searchSpaceSize);
            object_ptr<ISample> pWriteSample;
            float* m_loc = pmt.toMeter(location.x, location.y);

            RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
            {
                auto oCodec = m_PositionSampleFactory.MakeCodecFor(pWriteSample);

                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.x, m_loc[0] - axleToPicture*cos(heading)));
                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.y, m_loc[1] - axleToPicture*sin(heading)));
                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.heading, heading));
                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.speed, speed));
            }

            m_oPosWriter << pWriteSample << flush << trigger;
        }
    }
    
    RETURN_NOERROR;
}