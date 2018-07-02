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

using namespace adtf::util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::system;
using namespace adtf::filter;
using namespace adtf::mediadescription;
using namespace adtf_ddl;

using namespace cv;
using namespace cv::dnn;
using namespace std;

#include "ObjectDetection.h"
#include <ADTF3_OpenCV_helper.h>

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_OBJECT_DETECTION_FILTER,
    LABEL_OBJECT_DETECTION_FILTER,
    cObjectDetection,
    adtf::filter::pin_trigger({ "input" }));

void cObjectDetection::getMaxClass(const Mat &probBlob, tUInt64& classId, double& classProb)
{
    Mat probMat = probBlob.reshape(1, 1); //reshape the blob to 1x1000 matrix
    Point classNumber;
    minMaxLoc(probMat, NULL, &classProb, NULL, &classNumber);
    classId = static_cast<tUInt64>(classNumber.x);
}

void cObjectDetection::findMax(Mat &probMat, tUInt64& classId, double& classProb)
{
    Point classNumber;
    minMaxLoc(probMat, NULL, &classProb, NULL, &classNumber);
    classId = static_cast<tUInt64>(classNumber.x);
    probMat.at<float>(classNumber) = 0.0;
}

std::vector<cString> cObjectDetection::readClassNames(const char *filename)
{
    std::vector<cString> classNames;
    std::ifstream fp(filename);
    if (!fp.is_open())
    {
        LOG_ERROR("File with classes labels not found: %s", filename);
        return std::vector<cString>();
    }
    std::string name;
    while (!fp.eof())
    {
        std::getline(fp, name);
        if (name.length())
        {
            classNames.push_back(name.substr(name.find(' ') + 1).c_str());
        }
    }
    fp.close();
    return classNames;
}

cObjectDetection::cObjectDetection()
{
    //create image pins
    m_sCurrentFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    m_sCurrentFormat.m_ui32Width = 1920;
    m_sCurrentFormat.m_ui32Height = 1080;
    m_sCurrentFormat.m_szMaxByteSize = 0;
    m_sCurrentFormat.m_ui8DataEndianess = PLATFORM_BYTEORDER;
    adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());

    set_stream_type_image_format(*pType, m_sCurrentFormat);

    Register(m_oReader, "input", ucom_object_ptr_cast<const IStreamType>(pType));
    Register(m_oWriter, "output", ucom_object_ptr_cast<const IStreamType>(pType));

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sCurrentFormat, *pType.Get(), m_oWriter);
    });

    //get the media description for the classification
    object_ptr<IStreamType> pTypeClassifaction;
    if IS_FAILED(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tClassification",
        pTypeClassifaction, m_oCodecFactory))
    {
        LOG_ERROR("Could not load media description for output pin classification");
    }
    else
    {
        adtf_ddl::access_element::find_array_index(m_oCodecFactory, "className", m_tClassificationIds.classNameId);
        adtf_ddl::access_element::find_index(m_oCodecFactory, "probValue", m_tClassificationIds.probValueId);
        adtf_ddl::access_element::find_index(m_oCodecFactory, "classId", m_tClassificationIds.classIdId);
    }

    //create output pin for classifacation
    Register(m_oClassificationWriter, "classification", ucom_object_ptr_cast<const IStreamType>(pTypeClassifaction));

    //register properties
    RegisterPropertyVariable("opencl", m_bOpenCL);
    RegisterPropertyVariable("proto", m_strProtoTxt);
    RegisterPropertyVariable("model", m_strModel);
    RegisterPropertyVariable("classNames", m_strClassNames);
}

tResult cObjectDetection::Configure()
{
    // check if all necessary files exists
    cFilename filenameProtoTxt = m_strProtoTxt; ;
    adtf::services::ant::adtf_resolve_macros(filenameProtoTxt);
    if (!cFileSystem::Exists(filenameProtoTxt))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, cString::Format("Prototext file could not be loaded from %s", filenameProtoTxt.GetPtr()));
    }

    cFilename filenameModel = m_strModel; ;
    adtf::services::ant::adtf_resolve_macros(filenameModel);
    if (!cFileSystem::Exists(filenameModel))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, cString::Format("Network file could not be loaded from %s", filenameModel.GetPtr()));
    }

    cFilename filenameClassnames = m_strClassNames; ;
    adtf::services::ant::adtf_resolve_macros(filenameClassnames);
    if (!cFileSystem::Exists(filenameClassnames))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, cString::Format("Class names file could not be loaded from %s", filenameClassnames.GetPtr()));
    }
    //read model
    m_oNet = dnn::readNetFromCaffe(filenameProtoTxt.GetPtr(),
        filenameModel.GetPtr());

    if (m_oNet.empty())
    {
        LOG_ERROR("Can't load network");
        RETURN_ERROR_DESC(ERR_INVALID_FILE, cString::Format("Network could not be loaded"));
    }
    else
    {
        LOG_INFO(cString::Format("Loaded caffe model sucessfully from %s", filenameModel.GetPtr()));
    }
    //read class names
    m_classNames = readClassNames(filenameClassnames.GetPtr());

    if (m_bOpenCL)
    {
        m_oNet.setPreferableTarget(DNN_TARGET_OPENCL);
    }

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cObjectDetection::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
 
    //the result for classification
    Mat outputImage;
    cString className;
    double classProb = 0;
    tUInt64 classId = 0;

    if (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat inputImage = Mat(cv::Size(m_sCurrentFormat.m_ui32Width, m_sCurrentFormat.m_ui32Height),
                CV_8UC3, (uchar*)pReadBuffer->GetPtr());

            try
            {
                Mat oBlob = blobFromImage(inputImage, 1.0f, Size(224, 224),
                    Scalar(104, 117, 123), false);

                if (!m_oNet.empty() && oBlob.size > 0)
                {
                    m_oNet.setInput(oBlob, "data");        //set the network input
                    Mat prob = m_oNet.forward("prob");
                    getMaxClass(prob, classId, classProb);//find the best class
                    className = m_classNames.at(classId);
                    //LOG_INFO("Best class: #%d '%s; Prob: %d'", classId, className.GetPtr(), (tUInt32)(classProb * 100));               
                }
            }
            catch (cv::Exception ex)
            {
                const char* err_msg = ex.what();
                LOG_ERROR(cString("OpenCV exception caught: ") + err_msg);
            }

            outputImage = inputImage.clone();
        }
    }

    //send the best result in a media sample
    object_ptr<ISample> pSample;
    if (className.IsNotEmpty() && classProb != 0)
    {
        if IS_OK(alloc_sample(pSample))
        {
            {
                auto oCodec = m_oCodecFactory.MakeCodecFor(pSample);
                //Write the class string
                tChar *pClassNameSample = static_cast<tChar*>(oCodec.GetElementAddress(m_tClassificationIds.classNameId));
                memset(pClassNameSample, 0, 128 * sizeof(tChar));
                if (className.GetLength() < 128 * sizeof(tChar))
                {
                    memcpy(pClassNameSample, className.GetPtr(), className.GetLength());
                }
                else
                {
                    memcpy(pClassNameSample, className.GetPtr(), 128 * sizeof(tChar));
                }
                RETURN_IF_FAILED(oCodec.SetElementValue(m_tClassificationIds.classIdId, &classId));
                RETURN_IF_FAILED(oCodec.SetElementValue(m_tClassificationIds.probValueId, &classProb));
            }
            m_oClassificationWriter << pSample << flush << trigger;

        }
    }

    //Write processed Image to Output Pin
    if (!outputImage.empty())
    {
        cString result = className + ": " + cString::Format("%f", classProb);

        cv::putText(outputImage, result.GetPtr(), Size(30, 400), cv::FONT_HERSHEY_PLAIN, 1.2, Scalar(255, 255, 255), 1, cv::LINE_AA);
        //update output format if matrix size does not fit to
        if (outputImage.total() * outputImage.elemSize() != m_sCurrentFormat.m_szMaxByteSize)
        {
            setTypeFromMat(m_oWriter, outputImage);
        }
        // write to pin
        writeMatToPin(m_oWriter, outputImage, m_pClock->GetStreamTime());
    }

    RETURN_NOERROR;
}
