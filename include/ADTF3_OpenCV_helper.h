/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra  $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/

#ifndef _ADTF3_OPENCV_HELPER
#define _ADTF3_OPENCV_HELPER

//adtf
#include <adtf_platform_inc.h>
#include <a_utils_platform_inc.h>
#include <adtf_systemsdk.h>


void setTypeFromMat(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tBool keepFormat = false)
{
    adtf::streaming::tStreamImageFormat outputFormat;
    outputFormat.m_ui32Height = outputImage.rows;
    outputFormat.m_ui32Width = outputImage.cols;
    outputFormat.m_szMaxByteSize = outputImage.cols * outputImage.rows * outputImage.channels();

    if (!keepFormat)
    {
        if (outputImage.channels() == 1)
        {
            outputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(GREYSCALE_8);
        }
        if (outputImage.channels() == 3)
        {
            outputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
        }
    }
    outputFormat.m_ui8DataEndianess = PLATFORM_BYTEORDER;

    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, outputFormat);

    writer << pTypeOutput;
}

void writeMatToPin(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tTimeStamp streamTime)
{
    //create write buffer
    object_ptr<ISample> pWriteSample;

    if (IS_OK(alloc_sample(pWriteSample, streamTime)))
    {
        object_ptr_locked<ISampleBuffer> pWriteBuffer;
        if (IS_OK(pWriteSample->WriteLock(pWriteBuffer, outputImage.cols * outputImage.rows * outputImage.channels())))
        {
            pWriteBuffer->Write(adtf_memory_buffer<void, tSize>((void*) outputImage.data, 
                outputImage.cols * outputImage.rows * outputImage.channels()));           
        }
    }

    writer << pWriteSample << flush << trigger;
}

tResult ChangeType(adtf::streaming::cDynamicSampleReader& reader, adtf::streaming::tStreamImageFormat& inputFormat, 
    const adtf::streaming::ant::IStreamType& oType)
{
    if (oType == adtf::streaming::stream_meta_type_image())
    {
        adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pTypeInput;
        // get pType from input reader
        reader >> pTypeInput;
        adtf::streaming::get_stream_type_image_format(inputFormat, *pTypeInput);
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}


tResult ChangeType(adtf::streaming::cDynamicSampleReader& reader, adtf::streaming::tStreamImageFormat& inputFormat, 
    const adtf::streaming::ant::IStreamType& oType, adtf::streaming::cSampleWriter& writer)
{
    if (oType == adtf::streaming::stream_meta_type_image())
    {
        adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pTypeInput;
        // get pType from input reader
        reader >> pTypeInput;
        adtf::streaming::get_stream_type_image_format(inputFormat, *pTypeInput);
        // and set pType also to samplewriter
        writer << pTypeInput;
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}
#endif