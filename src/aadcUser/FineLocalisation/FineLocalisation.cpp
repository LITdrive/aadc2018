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


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CBIRDS_EYE_VIEW_DATA_TRIGGERED_FILTER,
                                    "FineLocalisation_cf",
                                    cFineLocalisation,
                                    adtf::filter::pin_trigger({ "input" }));

cFineLocalisation::cFineLocalisation()
{

    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    Register(m_oReader, "inBirdsEye", pType);

    object_ptr<IStreamType> pTypeVirtualPoint;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tVirtualPoint", pTypeVirtualPoint, m_VirtualPointSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, cString("f64x"), m_ddlVirtualPointId.f64x));
        (adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, cString("f64y"), m_ddlVirtualPointId.f64y));
        (adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, cString("f64Heading"), m_ddlVirtualPointId.f64Heading));
        (adtf_ddl::access_element::find_index(m_VirtualPointSampleFactory, cString("f64Speed"), m_ddlVirtualPointId.f64Speed));
    } else {
        LOG_INFO("No mediadescription for tVirtualPoint found!");
    }
    //Register input pin
    Register(m_oVPReader, "inVirtualPoint", pTypeVirtualPoint);

    //Register output pin
    Register(m_oVPWriter, "outVitrtualPoint", pTypeVirtualPoint);

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sImageFormat, *pType.Get());
    });
}

tResult cFineLocalisation::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    
    RETURN_NOERROR;
}

tResult cFineLocalisation::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample, pVPReadSample;

    if(IS_OK(m_oVPReader.GetNextSample(pVPReadSample))) {
        auto oDecoder = m_VirtualPointSampleFactory.MakeDecoderFor(*pVPReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlVirtualPointId.f64x, &x));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlVirtualPointId.f64y, &y));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlVirtualPointId.f64Heading, &heading));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlVirtualPointId.f64Speed, &speed));
    }

    while (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            /*Mat bvImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                   CV_8UC3, const_cast<unsigned char*>(static_cast<const unsigned char*>(pReadBuffer->GetPtr())));*/

            object_ptr<ISample> pWriteSample;

            RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
            {
                auto oCodec = m_VirtualPointSampleFactory.MakeCodecFor(pWriteSample);

                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlVirtualPointId.f64x, x));
                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlVirtualPointId.f64y, y));
                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlVirtualPointId.f64Heading, heading));
                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlVirtualPointId.f64Speed, speed));
            }

            m_oVPWriter << pWriteSample << flush << trigger;
        }
    }
    
    RETURN_NOERROR;
}