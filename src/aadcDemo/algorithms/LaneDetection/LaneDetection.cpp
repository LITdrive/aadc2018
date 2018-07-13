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
#include "LaneDetection.h"
#include "ADTF3_OpenCV_helper.h"

/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataStatistics will be embedded to the Filter
/// and called repeatedly (last parameter of this macro)!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CLANEDETECTION_DATA_TRIGGERED_FILTER,
    "Lane Detection",
    cLaneDetection,
    adtf::filter::pin_trigger({ "in" }));


cLaneDetection::cLaneDetection()
{

    //Register Properties
    RegisterPropertyVariable("ROIOffsetX [Pixel]",      m_ROIOffsetX);
    RegisterPropertyVariable("ROIOffsetY [Pixel]",      m_ROIOffsetY);
    RegisterPropertyVariable("ROIWidth [Pixel]",        m_ROIWidth);
    RegisterPropertyVariable("ROIHeight [Pixel]",       m_ROIHeight);
    RegisterPropertyVariable("detectionLines",  m_detectionLines);
    RegisterPropertyVariable("minLineWidth [Pixel]", m_minLineWidth);
    RegisterPropertyVariable("maxLineWidth [Pixel]", m_maxLineWidth);
    RegisterPropertyVariable("minLineContrast", m_minLineContrast);
    RegisterPropertyVariable("thresholdImageBinarization", m_thresholdImageBinarization);

    //create and set inital input format type
    m_InPinVideoFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pTypeInput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeInput, m_InPinVideoFormat);
    //Register input pin
    Register(m_oReaderVideo, "in", pTypeInput);

    //Register output pins   
    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, m_OutPinVideoFormat);
    Register(m_oWriterVideo, "out", pTypeOutput);

    //register callback for type changes
    m_oReaderVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReaderVideo, *pType.Get());
    });
}

tResult cLaneDetection::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));   
    
    RETURN_NOERROR;
}

tResult cLaneDetection::ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
    const adtf::streaming::ant::IStreamType& oType)
{
    if (oType == adtf::streaming::stream_meta_type_image())
    {
        adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pTypeInput;
        // get pType from input reader
        inputPin >> pTypeInput;
        adtf::streaming::get_stream_type_image_format(m_InPinVideoFormat, *pTypeInput);       
        
        //set also output format 
        adtf::streaming::get_stream_type_image_format(m_OutPinVideoFormat, *pTypeInput);
        //we always have a grayscale output image
        m_OutPinVideoFormat.m_strFormatName = ADTF_IMAGE_FORMAT(GREYSCALE_8);
        // and set pType also to samplewriter
        m_oWriterVideo << pTypeInput;
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}

tResult cLaneDetection::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oReaderVideo.GetNextSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        Mat outputImage;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat inputImage(cv::Size(m_InPinVideoFormat.m_ui32Width, m_InPinVideoFormat.m_ui32Height),
                CV_8UC3, (uchar*)pReadBuffer->GetPtr());


            cvtColor(inputImage, outputImage, COLOR_BGR2GRAY);
            threshold(outputImage, outputImage, m_thresholdImageBinarization, 255, THRESH_BINARY);// Generate Binary Image

            // Detect Lines
            // here we store the pixel lines in the image where we search for lanes
            vector<tInt> detectionLines;
            // here we have all the detected line points 
            vector<Point> detectedLinePoints;

            //calculate the detectionlines in image
            getDetectionLines(detectionLines);

            findLinePoints(detectionLines, outputImage, detectedLinePoints);

            cvtColor(outputImage, outputImage, COLOR_GRAY2RGB);
            //iterate found points for drawing
            for (vector<Point>::size_type i = 0; i != detectedLinePoints.size(); i++)
            {
                circle(outputImage,
                    detectedLinePoints.at(i),
                    32,
                    Scalar(0, 0, 255),
                    -1,
                    8);
            }

            pReadBuffer->Unlock();

        }
        //Write processed Image to Output Pin
        if (!outputImage.empty())
        {
            //update output format if matrix size does not fit to
            if (outputImage.total() * outputImage.elemSize() != m_OutPinVideoFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_oWriterVideo, outputImage);
            }
            // write to pin
            writeMatToPin(m_oWriterVideo, outputImage, m_pClock->GetStreamTime());
        }

    }


    RETURN_NOERROR;
}

tResult cLaneDetection::findLinePoints(const vector<tInt>& detectionLines, const cv::Mat& image, vector<Point>& detectedLinePoints)
{
    //iterate through the calculated horizontal lines
    for (vector<tInt>::const_iterator nline = detectionLines.begin(); nline != detectionLines.end(); nline++)
    {
        uchar ucLastVal = 0;

        // create vector with line data
        const uchar* p = image.ptr<uchar>(*nline, m_ROIOffsetX);
        std::vector<uchar> lineData(p, p + m_ROIWidth);

        tBool detectedStartCornerLine = tFalse;
        tInt columnStartCornerLine = 0;

        for (std::vector<uchar>::iterator lineIterator = lineData.begin(); lineIterator != lineData.end(); lineIterator++)
        {
            uchar ucCurrentVal = *lineIterator;
            tInt currentIndex = tInt(std::distance(lineData.begin(), lineIterator));
            //look for transition from dark to bright -> start of line corner
            if ((ucCurrentVal - ucLastVal) > m_minLineContrast)
            {
                detectedStartCornerLine = tTrue;
                columnStartCornerLine = currentIndex;
            }//look for transition from bright to dark -> end of line 
            else if ((ucLastVal - ucCurrentVal) > m_minLineContrast && detectedStartCornerLine)
            {
                //we already have the start corner of line, so check the width of detected line
                if ((abs(columnStartCornerLine - currentIndex) > m_minLineWidth)
                    && (abs(columnStartCornerLine - currentIndex) < m_maxLineWidth))
                {
                    detectedLinePoints.push_back(Point(tInt(currentIndex - abs(columnStartCornerLine - currentIndex) / 2 +
                        m_ROIOffsetX), *nline));

                    detectedStartCornerLine = tFalse;
                    columnStartCornerLine = 0;
                }
            }
            //we reached maximum line width limit, stop looking for end of line
            if (detectedStartCornerLine &&
                abs(columnStartCornerLine - currentIndex) > m_maxLineWidth)
            {
                detectedStartCornerLine = tFalse;
                columnStartCornerLine = 0;
            }
            ucLastVal = ucCurrentVal;
        }
    }

    RETURN_NOERROR;
}
tResult cLaneDetection::getDetectionLines(vector<tInt>& detectedLines)
{
    tInt distanceBetweenDetectionLines = m_ROIHeight / (m_detectionLines + 1);

    for (int i = 1; i <= m_detectionLines; i++)
    {
        detectedLines.push_back(m_ROIOffsetY + i * distanceBetweenDetectionLines);
    }
    RETURN_NOERROR;
}