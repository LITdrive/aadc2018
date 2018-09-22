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


/*********************************************************************
 * This code was provided by HERE
 *
 * *******************************************************************/

#define A_UTILS_NO_DEPRECATED_WARNING

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <adtf3.h>
#include <stdlib.h>

#include "MarkerPositioning.h"

 /*! configuration parameters */

 // road sign distance and pose estimation
#define MP_LIMIT_ALPHA    60.0 // [degrees]
#define MP_LIMIT_YAW      15.0 // [degrees]
#define MP_LIMIT_YAW_INIT  8.0 // [degrees]
#define MP_LIMIT_DISTANCE  2.5 // [m]

// process covariances
#define MP_PROCESS_X                    1e-3
#define MP_PROCESS_Y                    1e-3
#define MP_PROCESS_HEADING              3e-4
#define MP_PROCESS_HEADING_DRIFT        5e-8
#define MP_PROCESS_SPEED                2e-3
#define MP_PROCESS_SPEED_SCALE          1e-6

// initial covariance values
#define MP_PROCESS_INIT_X               10.0
#define MP_PROCESS_INIT_Y               10.0
#define MP_PROCESS_INIT_HEADING         0.55
#define MP_PROCESS_INIT_HEADING_DRIFT   0.25
#define MP_PROCESS_INIT_SPEED           1.0
#define MP_PROCESS_INIT_SPEED_SCALE     0.5

// measurement covariances
#define MP_MEASUREMENT_X                0.5
#define MP_MEASUREMENT_Y                0.5
#define MP_MEASUREMENT_HEADING          1.0 // [radians]

/*! defines a data triggered filter and exposes it via a plugin class factory */
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CMARKERPOS_DATA_TRIGGERED_FILTER,
    "Marker Positioning",
    cMarkerPositioning,
    adtf::filter::pin_trigger({ "imu" , "road_sign_map"}));



/*! initialize the trigger function */
cMarkerPositioning::cMarkerPositioning()
{
    SetName("MarkerPos");
    //register properties
    RegisterPropertyVariable("Speed Scale", m_f32SpeedScale);
    RegisterPropertyVariable("Camera Offset::Lateral", m_f32CameraOffsetLat);
    RegisterPropertyVariable("Camera Offset::Longitudinal", m_f32CameraOffsetLon);
    RegisterPropertyVariable("Optional Roadsign File", m_roadSignFile);


    //the imu struct
    object_ptr<IStreamType> pTypeIMUData;

    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tInerMeasUnitData", pTypeIMUData, m_IMUDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "ui32ArduinoTimestamp", m_ddlInerMeasUnitDataIndex.timeStamp);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_x", m_ddlInerMeasUnitDataIndex.A_x);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_y", m_ddlInerMeasUnitDataIndex.A_y);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_z", m_ddlInerMeasUnitDataIndex.A_z);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_x", m_ddlInerMeasUnitDataIndex.G_x);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_y", m_ddlInerMeasUnitDataIndex.G_y);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_z", m_ddlInerMeasUnitDataIndex.G_z);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_x", m_ddlInerMeasUnitDataIndex.M_x);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_y", m_ddlInerMeasUnitDataIndex.M_y);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_z", m_ddlInerMeasUnitDataIndex.M_z);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32roll", m_ddlInerMeasUnitDataIndex.roll);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32pitch", m_ddlInerMeasUnitDataIndex.pitch);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32yaw", m_ddlInerMeasUnitDataIndex.yaw);
    }
    else
    {
        LOG_WARNING("No mediadescription for tInerMeasUnitData found!");
    }

    //the signal struct
    object_ptr<IStreamType> pTypeSignalData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalData, m_SignalDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_SignalDataSampleFactory, "ui32ArduinoTimestamp", m_ddlSignalDataIndex.timeStamp);
        adtf_ddl::access_element::find_index(m_SignalDataSampleFactory, "f32Value", m_ddlSignalDataIndex.value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalValue found!");
    }

    //the roadsignext struct
    object_ptr<IStreamType> pTypeRoadSignData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tRoadSignExt", pTypeRoadSignData, m_RoadSignSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "i16Identifier", m_ddlRoadSignIndex.id);
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "f32Imagesize", m_ddlRoadSignIndex.size);
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "af32TVec", m_ddlRoadSignIndex.tvec);
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "af32RVec", m_ddlRoadSignIndex.rvec);
    }
    else
    {
        LOG_WARNING("No mediadescription for tRoadSignExt found!");
    }

    //the position struct
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

    // cast to const type for the method calls below
    object_ptr<const IStreamType> pConstTypeRoadSignData = pTypeRoadSignData;
    object_ptr<const IStreamType> pConstTypeSignalData = pTypeSignalData;
    object_ptr<const IStreamType> pConstTypeIMUData = pTypeIMUData;
    object_ptr<const IStreamType> pConstTypePositionData = pTypePositionData;
    object_ptr<IStreamType> pTypeAnonymous = make_object_ptr<cStreamType>(stream_meta_type_anonymous());

    //register input pin(s)
    Register(m_oReaderRoadSign, "road_sign_ext", pConstTypeRoadSignData);
    Register(m_oReaderSpeed, "speed", pConstTypeSignalData);
    Register(m_oReaderIMU, "imu", pConstTypeIMUData);
    Register(m_roadSignMapData, "road_sign_map", pTypeAnonymous);
    //register output pin
    Register(m_oWriter, "position", pConstTypePositionData);
    //Reset Filter Covariances
    ResetFilter();
    // initialize translation and rotation vectors
    m_Tvec = Mat(3, 1, CV_32F, Scalar::all(0));
    m_Rvec = Mat(3, 1, CV_32F, Scalar::all(0));

    // initialize other variables
    m_f32Speed = 0;
    m_f32YawRate = 0;

    m_ui32ArduinoTimestamp = 0;

    m_ui32Cnt = 0;

}


tResult cMarkerPositioning::ParseRoadSignFile(cDOM& oDOM)
{
    m_roadSigns.clear();

    cDOMElementRefList oElems;

    if (IS_OK(oDOM.FindNodes("configuration/roadSign", oElems)))
    {
        for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
        {
            roadSign item;
            item.u16Id = tUInt16((*itElem)->GetAttribute("id", "0").AsInt32());
            item.f32X = tFloat32((*itElem)->GetAttribute("x", "0").AsFloat64());
            item.f32Y = tFloat32((*itElem)->GetAttribute("y", "0").AsFloat64());
            item.f32Radius = tFloat32((*itElem)->GetAttribute("radius", "0").AsFloat64());
            item.f32Direction = tFloat32((*itElem)->GetAttribute("direction", "0").AsFloat64());

            //item.bInit = ((*itElem)->GetAttribute("init","0").AsInt32());
            if ((*itElem)->GetAttribute("init", "0").AsInt32() == 0)
            {
                item.bInit = false;
            }
            else
            {
                item.bInit = true;
            }


            item.u16Cnt = 0;
            item.u32ticks = GetTime();

            item.f32Direction *= static_cast<tFloat32>(DEG2RAD); // convert to radians

            //LOG_INFO(cString::Format("LoadConfiguration::Id %d XY %f %f Radius %f Direction %f",
            //    item.u16Id, item.f32X, item.f32Y, item.f32Radius, item.f32Direction).GetPtr());

            m_roadSigns.push_back(item);

        }
        RETURN_NOERROR;
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}

/*! implements the configure function to read ALL Properties */
tResult cMarkerPositioning::Configure()
{
    // open roadsign configuration file
    cFilename fileRoadSign = m_roadSignFile;
    adtf::services::ant::adtf_resolve_macros(fileRoadSign);


    if (cFileSystem::Exists(fileRoadSign))
    {
        cDOM oDOMFromFile;
        oDOMFromFile.Load(fileRoadSign);
        RETURN_IF_FAILED(ParseRoadSignFile(oDOMFromFile));
        LOG_INFO(cString::Format("Loaded road sign file %s with %d signs", fileRoadSign.GetPtr(), m_roadSigns.size()));
    }

    RETURN_NOERROR;
}

tResult cMarkerPositioning::ProcessRoadSignFile(tInt64 tmTimeOfTrigger, const ISample& sample)
{
    adtf::ucom::ant::object_ptr_shared_locked<const adtf::streaming::ant::ISampleBuffer> pSampleBuffer;
    RETURN_IF_FAILED(sample.Lock(pSampleBuffer));

    adtf_util::cString roadSignFileString;
    roadSignFileString.SetBuffer(pSampleBuffer->GetSize());

    memcpy(roadSignFileString.GetBuffer(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());


    cDOM oDOM;
    RETURN_IF_FAILED(oDOM.FromString(roadSignFileString));
    RETURN_IF_FAILED(ParseRoadSignFile(oDOM));

    LOG_INFO(cString::Format("Recieved road sign file from pin with %d signs", m_roadSigns.size()));
    ResetFilter();
    RETURN_NOERROR;
}

/*! funtion will be executed each time a trigger occured */
tResult cMarkerPositioning::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;

    //LOG_INFO(cString::Format("process: %lu", tmTimeOfTrigger).GetPtr());

    while (IS_OK(m_oReaderSpeed.GetNextSample(pReadSample)))
    {
        // store speed
        auto oDecoder = m_SignalDataSampleFactory.MakeDecoderFor(*pReadSample);
        m_f32Speed = adtf_ddl::access_element::get_value(oDecoder, m_ddlSignalDataIndex.value);
    }

    while (IS_OK(m_oReaderIMU.GetNextSample(pReadSample)))
    {
        // predict
        ProcessInerMeasUnitSample(tmTimeOfTrigger, *pReadSample);
    }

    while (IS_OK(m_oReaderRoadSign.GetNextSample(pReadSample)))
    {
        // update
        ProcessRoadSignStructExt(tmTimeOfTrigger, *pReadSample);
    }

    while (IS_OK(m_roadSignMapData.GetNextSample(pReadSample)))
    {
        // update
        ProcessRoadSignFile(tmTimeOfTrigger, *pReadSample);
    }


    RETURN_NOERROR;
}
tVoid cMarkerPositioning::ResetFilter()
{
  LOG_INFO("Reset Filter Covariances");
  // initialize EKF variables
  m_state = Mat(6,1,CV_64F,Scalar::all(0));
  m_errorCov = Mat(6,6,CV_64F,Scalar::all(0));

  tFloat64 T = 0.1;
  m_errorCov.at<double>(0,0) = MP_PROCESS_INIT_X;
  m_errorCov.at<double>(1,1) = MP_PROCESS_INIT_Y;
  m_errorCov.at<double>(2,2) = MP_PROCESS_INIT_HEADING;
  m_errorCov.at<double>(2,3) = MP_PROCESS_INIT_HEADING/T;
  m_errorCov.at<double>(3,3) = MP_PROCESS_INIT_HEADING_DRIFT;
  m_errorCov.at<double>(4,4) = MP_PROCESS_INIT_SPEED;
  m_errorCov.at<double>(4,5) = MP_PROCESS_INIT_SPEED/T;
  m_errorCov.at<double>(5,5) = MP_PROCESS_INIT_SPEED_SCALE;

  m_transitionMatrix = Mat(6,6,CV_64F,Scalar::all(0));
  setIdentity(m_transitionMatrix);

  m_processCov = Mat(6,6,CV_64F,Scalar::all(0));
  m_processCov.at<double>(0,0) = MP_PROCESS_X;
  m_processCov.at<double>(1,1) = MP_PROCESS_Y;
  m_processCov.at<double>(2,2) = MP_PROCESS_HEADING;
  m_processCov.at<double>(3,3) = MP_PROCESS_HEADING_DRIFT;
  m_processCov.at<double>(4,4) = MP_PROCESS_SPEED;
  m_processCov.at<double>(5,5) = MP_PROCESS_SPEED_SCALE;

  m_isInitialized = tFalse;
  return;
}

/*! calculates normalized angle difference */
tFloat32 cMarkerPositioning::angleDiff(tFloat32 angle1, tFloat32 angle2)
{
    // normalization
    angle1 = normalizeAngle(angle1, static_cast<tFloat32>(M_PI));
    angle2 = normalizeAngle(angle2, static_cast<tFloat32>(M_PI));

    // compute difference and normalize in [-pi pi]
    return normalizeAngle(angle2 - angle1, 0);
}

/*! calculates normalized angle */
tFloat32 cMarkerPositioning::normalizeAngle(tFloat32 alpha, tFloat32 center)
{
    return mod(alpha - center + static_cast<tFloat32>(M_PI), 2.0f*static_cast<tFloat32>(M_PI)) + center - static_cast<tFloat32>(M_PI);
}

/*! calculates modulus after division */
tFloat32 cMarkerPositioning::mod(tFloat32 x, tFloat32 y)
{
    tFloat32 r;
    tFloat32 b_x;
    if (y == floor(y))
    {
        return x - floor(x / y) * y;
    }
    else
    {
        r = x / y;
        if (r < 0.0)
        {
            b_x = ceil(r - 0.5f);
        }
        else
        {
            b_x = floor(r + 0.5f);
        }
        if (fabs(r - b_x) <= 2.2204460492503131E-16 * fabs(r))
        {
            return 0.0;
        }
        else
        {
            return (r - floor(r)) * y;
        }
    }
}

/*! Calculates orientation, distance and pose of the given road sign,
 * and updates the positioning filter accordingly */
tResult cMarkerPositioning::ProcessRoadSignStructExt(tTimeStamp tmTimeOfTrigger, const adtf::streaming::ISample &oSample)
{
    // parse the data
    auto oDecoder = m_RoadSignSampleFactory.MakeDecoderFor(oSample);

    // fetch marker id
    m_i16ID = access_element::get_value(oDecoder, m_ddlRoadSignIndex.id);

    const tVoid* pArray;
    tSize size;

    // fetch marker translation and rotation arrays
    access_element::get_array(oDecoder, "af32TVec", pArray, size);
    m_Tvec.data = const_cast<uchar*>(static_cast<const uchar*>(pArray));

    access_element::get_array(oDecoder, "af32RVec", pArray, size);
    m_Rvec.data = const_cast<uchar*>(static_cast<const uchar*>(pArray));

    // ignore initial noisy markers
    if (m_ui32Cnt < 50)
    {
        m_ui32Cnt++;
        RETURN_NOERROR;
    }

    cv::Mat R;
    cv::Rodrigues(m_Rvec, R); // rot is 3x3

    // calculate translation
    tFloat32 lateral = m_Tvec.at<float>(0);
    tFloat32 longitudinal = m_Tvec.at<float>(2);

    // add camera offset
    lateral += m_f32CameraOffsetLat;
    longitudinal += m_f32CameraOffsetLon;

    tFloat32 d0 = sqrt(lateral*lateral + longitudinal*longitudinal);

    tFloat32 a0 = atan2(lateral, longitudinal);
    a0 = (tFloat32)normalizeAngle(a0, 0)*static_cast<tFloat32>(RAD2DEG); // normalize angle -pi:pi

    a0 *= -1.0; // and change direction

    // calculate pose of the road sign
    float sy = sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) + R.at<float>(1, 0) * R.at<float>(1, 0));

    tFloat32 yawE = atan2(-R.at<float>(2, 0), sy) * static_cast<tFloat32>(RAD2DEG);

    // check angle and distance limit
    if (fabs(a0) > MP_LIMIT_ALPHA || d0 > MP_LIMIT_DISTANCE)
    {
        RETURN_NOERROR;
    }

    //LOG_INFO(cString::Format("ID %d: d0 %f a0 %f yawE %f", m_i16ID, d0, a0, yawE).GetPtr());

    // wait for start-marker, and then initialize the filter
    if (m_isInitialized == tFalse)
    {
        for (unsigned int i = 0; i < m_roadSigns.size(); i++)
        {
            if ((m_roadSigns[i].u16Id == m_i16ID) &&
                (m_roadSigns[i].bInit == tTrue))
            {
                // calculate the vehicle position and heading based on
                // road-sign measurement

                // estimate heading
                tFloat32 heading = m_roadSigns[i].f32Direction + yawE*static_cast<tFloat32>(DEG2RAD);

                heading = normalizeAngle(heading, 0);

                tFloat32 shift = -1.0f*d0;

                tFloat32 correction = heading + a0*static_cast<tFloat32>(DEG2RAD);
                correction = normalizeAngle(correction, 0);

                // estimate location
                tFloat32 x = m_roadSigns[i].f32X + cos(correction)*shift;
                tFloat32 y = m_roadSigns[i].f32Y + sin(correction)*shift;

                //LOG_INFO(cString::Format("initialize e %f n %f h %f x %f y %f", x, y, heading*RAD2DEG,
                    //m_roadSigns[i].f32X, m_roadSigns[i].f32Y).GetPtr());

                // initialize filter state
                m_state.at<double>(0) = x;
                m_state.at<double>(1) = y;
                m_state.at<double>(2) = heading;
                m_state.at<double>(3) = 0;
                m_state.at<double>(4) = 0;
                m_state.at<double>(5) = 0;

                m_isInitialized = tTrue;

            }

        }

        RETURN_NOERROR;

    }

    // find a matching road sign

    tFloat64 dt = 0;

    tInt ind = -1;
    for (unsigned int i = 0; i < m_roadSigns.size(); i++)
    {
        if (m_roadSigns[i].u16Id == m_i16ID)
        {
            // calculate heading wrt marker
            tFloat32 heading = static_cast<tFloat32>(m_state.at<double>(2)) + a0*static_cast<tFloat32>(DEG2RAD);
            heading = normalizeAngle(heading, 0);

            // estimate marker location based on current vehicle location
            // and marker measurement
            tFloat32 x0 = static_cast<tFloat32>(m_state.at<double>(0)) + cos(heading)*d0;
            tFloat32 y0 = static_cast<tFloat32>(m_state.at<double>(1)) + sin(heading)*d0;

            // calculate error distance
            tFloat32 dx = x0 - m_roadSigns[i].f32X;
            tFloat32 dy = y0 - m_roadSigns[i].f32Y;

            tFloat32 distance = sqrt(dx*dx + dy*dy);

            tInt found = tFalse;

            // re-initialize with init-signs
            if (m_roadSigns[i].bInit == tTrue && fabs(m_f32Speed) < 0.01 && fabs(m_f32YawRate) < 1)
            {
                tFloat32 shift = -1.0f*d0;

                // estimate location
                tFloat32 x = m_roadSigns[i].f32X + cos(heading)*shift;
                tFloat32 y = m_roadSigns[i].f32Y + sin(heading)*shift;
                tFloat32 heading = m_roadSigns[i].f32Direction + yawE*static_cast<tFloat32>(DEG2RAD);

                // initialize filter but keep heading states
                m_state.at<double>(0) = x;
                m_state.at<double>(1) = y;
                m_state.at<double>(2) = heading;

                m_state.at<double>(4) = 0;
                m_state.at<double>(5) = 0;
                found = tTrue;

            }
            // marker found within the radius
            else if (distance < m_roadSigns[i].f32Radius)
            {
                found = tTrue;
            }

            // found a suitable marker
            if (found)
            {
                ind = i;

                // calculate time from previous marker measurement
                dt = (GetTime() - m_roadSigns[i].u32ticks)*1e-6;
                m_roadSigns[i].u32ticks = GetTime();

                // reset sample counter when marker reappears
                if (dt > 1.0)
                {
                    m_roadSigns[i].u16Cnt = 0;
                }

                break;
            }

        }

    }

    // update sample counter
    m_roadSigns[ind].u16Cnt++;

    // conditions:
    // #1 no matching marker found
    // #2 too long time from previous marker input
    // #3 dropping samples when a new marker is found
    if (ind < 0 || dt > 0.3 || m_roadSigns[ind].u16Cnt < 10)
    {

        RETURN_NOERROR;
    }

    // EKF update step

    // create pseudo measurement for heading update
    tFloat32 headingUpdate = static_cast<tFloat32>(m_state.at<double>(2));

    tFloat32 shift = -1.0f*d0; // reversed translation direction

    // calculate translation direction
    tFloat32 correction = headingUpdate + a0*static_cast<tFloat32>(DEG2RAD);
    correction = normalizeAngle(correction, 0);

    // update location estimate
    tFloat32 x = m_roadSigns[ind].f32X + cos(correction)*shift;
    tFloat32 y = m_roadSigns[ind].f32Y + sin(correction)*shift;

    Mat measCov = Mat(3, 3, CV_64F, Scalar::all(0));
    measCov.at<double>(0, 0) = MP_MEASUREMENT_X;
    measCov.at<double>(1, 1) = MP_MEASUREMENT_Y;
    measCov.at<double>(2, 2) = MP_MEASUREMENT_HEADING;

    Mat measurementMatrix = Mat(3, 6, CV_64F, Scalar::all(0));

    measurementMatrix.at<double>(0, 0) = 1.0;
    measurementMatrix.at<double>(1, 1) = 1.0;
    measurementMatrix.at<double>(2, 2) = 1.0;

    Mat identity = Mat(6, 6, CV_64F, Scalar::all(0));
    setIdentity(identity);

    Mat measurement = Mat(3, 1, CV_64F, Scalar::all(0));

    measurement.at<double>(0) = x;
    measurement.at<double>(1) = y;
    measurement.at<double>(2) = headingUpdate;

    // propagate covariance
    m_errorCov = m_transitionMatrix*m_errorCov*m_transitionMatrix.t() + m_processCov;

    // calculate innovation
    Mat innovation = measurement - measurementMatrix*m_state;

    // modulo of the heading measurement
    innovation.at<double>(2) = angleDiff(static_cast<tFloat32>(mod(static_cast<tFloat32>(m_state.at<double>(2)), static_cast<tFloat32>(2.0*M_PI)) - M_PI),
        mod(headingUpdate, 2.0f*static_cast<tFloat32>(M_PI)) - static_cast<tFloat32>(M_PI));

    Mat tmp = measurementMatrix*m_errorCov*measurementMatrix.t() + measCov;
    Mat gain = m_errorCov*measurementMatrix.t()*tmp.inv();

    // update state and covariance matrix
    m_state += gain*innovation;
    m_state.at<double>(2) = normalizeAngle(static_cast<tFloat32>(m_state.at<double>(2)), 0.0f);
    m_errorCov = (identity - gain*measurementMatrix)*m_errorCov;

    RETURN_NOERROR;
}


/*! support function for getting time */
tTimeStamp cMarkerPositioning::GetTime()
{
    return adtf_util::cHighResTimer::GetTime();
}

/*! sends position data out */
tResult cMarkerPositioning::sendPositionStruct(const tTimeStamp &timeOfFix, const tFloat32 &f32X, const tFloat32 &f32Y, const tFloat32 &f32Radius,
    const tFloat32 &f32Heading, const tFloat32 &f32Speed)
{
    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample, timeOfFix));

    auto oCodec = m_PositionSampleFactory.MakeCodecFor(pSample);

    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.x, f32X));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.y, f32Y));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.radius, f32Radius));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.speed, f32Speed));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.heading, f32Heading));

    //LOG_INFO(cString::Format("sendPositionStruct: %.3f %.3f %.3f %.3f %.3f", f32X, f32Y,
        //f32Radius, f32Heading, f32Speed).GetPtr());

    // the sample buffer lock is released in the destructor of oCodec
    m_oWriter << pSample << flush << trigger;

    RETURN_NOERROR;
}

/*! processes inertial measurement data sample, and runs EKF prediction
 *  based on heading rate and speed measurements */
tResult cMarkerPositioning::ProcessInerMeasUnitSample(tTimeStamp tmTimeOfTrigger, const adtf::streaming::ISample &oSample)
{
    tFloat64 dt = 0;
    tUInt32 ui32ArduinoTimestamp = 0;

    // parse the data
    auto oDecoder = m_IMUDataSampleFactory.MakeDecoderFor(oSample);

    // yaw-rate gyro measurment
    m_f32YawRate = access_element::get_value(oDecoder, m_ddlInerMeasUnitDataIndex.G_z);

    // fetch timestamp and calculate dt
    ui32ArduinoTimestamp = access_element::get_value(oDecoder, m_ddlInerMeasUnitDataIndex.timeStamp);

    dt = (tFloat64)(ui32ArduinoTimestamp - m_ui32ArduinoTimestamp)*1e-6;

    m_ui32ArduinoTimestamp = ui32ArduinoTimestamp;

    //LOG_INFO(cString::Format("processInertial: %.6f", m_f32YawRate).GetPtr());

    // filter not initialized
    if (m_isInitialized == tFalse)
    {
        RETURN_NOERROR;
    }

    // update heading
    tFloat32 hk = static_cast<tFloat32>(m_state.at<double>(2) + (m_f32YawRate*static_cast<tFloat32>(DEG2RAD) + m_state.at<double>(3))*dt);

    // normalize heading -pi:pi
    hk = normalizeAngle(hk, 0);

    tFloat32 sc = m_f32SpeedScale;

    // update speed and scale
    tFloat32 ak = static_cast<tFloat32>(m_state.at<double>(5));
    tFloat32 vk = m_f32Speed*(sc - ak);

    // update transition matrix; F = I + Fc*dt
    m_transitionMatrix.at<double>(0, 2) = -vk*sin(hk)*dt;
    m_transitionMatrix.at<double>(0, 3) = -vk*sin(hk)*dt;
    m_transitionMatrix.at<double>(0, 4) = cos(hk)*dt;
    m_transitionMatrix.at<double>(0, 5) = -vk / (sc - ak)*cos(hk)*dt;

    m_transitionMatrix.at<double>(1, 2) = vk*cos(hk)*dt;
    m_transitionMatrix.at<double>(1, 3) = vk*cos(hk)*dt;
    m_transitionMatrix.at<double>(1, 4) = sin(hk)*dt;
    m_transitionMatrix.at<double>(1, 5) = -vk / (sc - ak)*sin(hk)*dt;

    m_transitionMatrix.at<double>(2, 3) = dt;

    // propagate state and covariance
    m_state.at<double>(0) += vk*cos(hk)*dt;
    m_state.at<double>(1) += vk*sin(hk)*dt;
    m_state.at<double>(2) = hk;
    m_state.at<double>(4) = vk;

    m_errorCov = m_transitionMatrix*m_errorCov*m_transitionMatrix.t() + m_processCov;

    sendPositionStruct(tmTimeOfTrigger, static_cast<tFloat32>(m_state.at<double>(0)), static_cast<tFloat32>(m_state.at<double>(1)),
        static_cast<tFloat32>(sqrt(m_errorCov.at<double>(0, 0) + m_errorCov.at<double>(1, 1))),
        static_cast<tFloat32>(m_state.at<double>(2)), static_cast<tFloat32>(m_state.at<double>(4)));

    RETURN_NOERROR;
}
