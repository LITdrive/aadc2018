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
#include "MarkerDetector.h"
#include "ADTF3_OpenCV_helper.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CMARKERDETECTOR_DATA_TRIGGERED_FILTER,
    "Marker Detector",
    cMarkerDetector,
    adtf::filter::pin_trigger({ "input" }));


cMarkerDetector::cMarkerDetector()
{
    //Register Properties
    RegisterPropertyVariable("Calibration File", m_calibFile);
    RegisterPropertyVariable("Detector Parameter File", m_fileDetectorParameter);
    RegisterPropertyVariable("Marker Size [m]", m_f32MarkerSize);
    RegisterPropertyVariable("Marker Roi::x", m_MarkerX);
    RegisterPropertyVariable("Marker Roi::y", m_MarkerY);
    RegisterPropertyVariable("Marker Roi::width", m_MarkerWidth);
    RegisterPropertyVariable("Marker Roi::height", m_MarkerHeight);
    RegisterPropertyVariable("Show Roi", m_ShowMarkerRoi);
    RegisterPropertyVariable("Marker Roi::height", m_MarkerHeight);


    //create and set inital input format type
    m_sInputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sInputFormat);


    //register input pin
    Register(m_oReader, "input", pType);
    //register output pin
    Register(m_oImagePinWriter, "output", pType);

    //get the media description
    object_ptr<IStreamType> pTypePose;
    if IS_FAILED(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tRoadSignExt",
        pTypePose, m_outputPoseSampleFactory))
    {
        LOG_ERROR("Could not load media description for output pin road_sign_ext");
    }
    else
    {
        adtf_ddl::access_element::find_array_index(m_outputPoseSampleFactory, "af32TVec", m_ddlRoadSignExtIds.tVec);
        adtf_ddl::access_element::find_array_index(m_outputPoseSampleFactory, "af32RVec", m_ddlRoadSignExtIds.rVec);
        adtf_ddl::access_element::find_index(m_outputPoseSampleFactory, "i16Identifier", m_ddlRoadSignExtIds.id);
        adtf_ddl::access_element::find_index(m_outputPoseSampleFactory, "f32Imagesize", m_ddlRoadSignExtIds.imageSize);
    }

    //register output pose pin
    Register(m_oPosePinWriter, "road_sign_ext", pTypePose);

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sInputFormat, *pType.Get(), m_oImagePinWriter);
    });
}


tResult cMarkerDetector::Configure()
{
    // Read Camera Calibration File
    cFilename fileCalibration = m_calibFile;
    adtf::services::ant::adtf_resolve_macros(fileCalibration);
    //check if calibration file with camera paramters exits
    if (fileCalibration.IsEmpty())
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, "Calibration File for camera not found");
    }
    if (!(cFileSystem::Exists(fileCalibration)))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, "Calibration File for camera not found");
    }
    else
    {
        // read the calibration file with camera paramters exits and save to member variable
        readCameraParameters(fileCalibration.GetPtr(), m_Intrinsics, m_Distorsion);
        m_bCamaraParamsLoaded = tTrue;
    }


    //Aruco
    //create the detector params
    cFilename fileDetectorParameter = m_fileDetectorParameter;
    m_detectorParams = aruco::DetectorParameters::create();
    adtf::services::ant::adtf_resolve_macros(fileDetectorParameter);
    if (!(readDetectorParameters(fileDetectorParameter.GetPtr(), m_detectorParams)))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, "Detector Parameter file not valid");
    }

    //set marker dictionary
    m_Dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cMarkerDetector::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    
    // the results from aruco detection	
    vector< vector< Point2f > > corners, rejected;
    Mat outputImage;

    vector< Vec3d > rvecs, tvecs;
    vector< int > ids;
    if (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat m_inputImage(cv::Size(m_sInputFormat.m_ui32Width, m_sInputFormat.m_ui32Height),
                CV_8UC3, (uchar*)pReadBuffer->GetPtr());
            


            //Check for ROI settings and create ROI appropriately
            m_MarkerRoiUsed = checkRoi();

            // doing the detection of markers in image
            if (m_MarkerRoiUsed)
            {
                cv::Mat roiImage(m_inputImage, m_MarkerRoi);
                aruco::detectMarkers(roiImage, m_Dictionary, corners, ids, m_detectorParams, rejected);
            }
            else
            {
                aruco::detectMarkers(m_inputImage, m_Dictionary, corners, ids, m_detectorParams, rejected);
            }

            // draw detections
            outputImage = m_inputImage.clone();
            if (m_ShowMarkerRoi)
            {
                rectangle(outputImage, m_MarkerRoi, Scalar(255), 10, 8, 0);
            }
            aruco::drawDetectedMarkers(outputImage, corners, ids);

            // if we have the camera pararmeter available we calculate the pose
            if (m_bCamaraParamsLoaded && ids.size() > 0)
                aruco::estimatePoseSingleMarkers(corners, m_f32MarkerSize, m_Intrinsics, m_Distorsion, rvecs,
                    tvecs);

            if (m_bCamaraParamsLoaded  && ids.size() > 0)
            {
                for (unsigned int i = 0; i < ids.size(); i++)
                {
                    aruco::drawAxis(outputImage, m_Intrinsics, m_Distorsion, rvecs[i], tvecs[i],
                        m_f32MarkerSize * 0.5f);
                }
            }

        }
        //write WITH markers to output pin
        if (!outputImage.empty())
        {
            //update output format if matrix size does not fit to
            if (outputImage.total() * outputImage.elemSize() != m_sInputFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_oImagePinWriter, outputImage);
            }
            // write to pin
            writeMatToPin(m_oImagePinWriter, outputImage, m_pClock->GetStreamTime());
        }
        if (!rvecs.empty() && !tvecs.empty() && !ids.empty())
        {
            int n = 0;
            for (auto id : ids)
            {
                //create write buffer
                object_ptr<ISample> pWriteSample;

                if (IS_OK(alloc_sample(pWriteSample)))
                {
                    auto oCodec = m_outputPoseSampleFactory.MakeCodecFor(pWriteSample);

                    // get pointer to rotation vector in sample
                    tFloat32 rVecFl32[3] = { static_cast<tFloat32>(rvecs[n][0]) , static_cast<tFloat32>(rvecs[n][1]) , static_cast<tFloat32>(rvecs[n][2]) };
                    tFloat32 *rVecSample = static_cast<tFloat32*>(oCodec.GetElementAddress(m_ddlRoadSignExtIds.rVec));
                    memcpy(rVecSample, rVecFl32, sizeof(rVecFl32));

                    tFloat32 imageSize =  getMarkerArea(corners[n]);

                    // get pointer to translation vector in sample
                    tFloat32 tVecFl32[3] = { static_cast<tFloat32>(tvecs[n][0]) , static_cast<tFloat32>(tvecs[n][1]) ,static_cast<tFloat32>(tvecs[n][2]) };
                    tFloat32 *tVecSample = static_cast<tFloat32*>(oCodec.GetElementAddress(m_ddlRoadSignExtIds.tVec));
                    memcpy(tVecSample, tVecFl32, sizeof(tVecFl32));

                    if (IS_OK(oCodec.SetElementValue(m_ddlRoadSignExtIds.id, id)) && 
                        IS_OK(oCodec.SetElementValue(m_ddlRoadSignExtIds.imageSize, imageSize)))
                    {
                        // the sample buffer lock is released in the destructor of oCodec
                        m_oPosePinWriter << pWriteSample << flush << trigger;
                    }
                }
                n++;
            }
        }
    }

    RETURN_NOERROR;
}

tBool cMarkerDetector::checkRoi(void)
{
    tBool res = tTrue;

    // if width or heigt are not set ignore the roi
    if (static_cast<tFloat32>(m_MarkerWidth) == 0 || static_cast<tFloat32>(m_MarkerHeight) == 0) return tFalse;

    //check if we are within the boundaries of the image
    res &= (static_cast<tFloat32>(m_MarkerX) + static_cast<tFloat32>(m_MarkerWidth)) < m_sInputFormat.m_ui32Width;
    res &= (static_cast<tFloat32>(m_MarkerY) + static_cast<tFloat32>(m_MarkerHeight)) < m_sInputFormat.m_ui32Height;

    if (res)
    {
        //create the rectangle
        m_MarkerRoi = cv::Rect2f(static_cast<tFloat32>(m_MarkerX), static_cast<tFloat32>(m_MarkerY), static_cast<tFloat32>(m_MarkerWidth), static_cast<tFloat32>(m_MarkerHeight));
    }

    return res;
}