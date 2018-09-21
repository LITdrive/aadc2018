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

#include "LITD_IMULocalization.h"

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
    "LITD_IMULocalization",
    cLITD_IMULocalization,
    adtf::filter::pin_trigger({"imu", "pos_reset"}));



/*! initialize the trigger function */
cLITD_IMULocalization::cLITD_IMULocalization()
{
    SetName("MarkerPos");
    //register properties
    RegisterPropertyVariable("Speed Scale", m_f32SpeedScale);
    RegisterPropertyVariable("Camera Offset::Lateral", m_f32CameraOffsetLat);
    RegisterPropertyVariable("Camera Offset::Longitudinal", m_f32CameraOffsetLon);

    RegisterPropertyVariable("x Startposition in m", m_f32XOffset);
    RegisterPropertyVariable("y Startposition in m", m_f32YOffset); 
    RegisterPropertyVariable("Heading Startposition in degree", m_f32HeadingOffset);


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
    object_ptr<const IStreamType> pConstTypeSignalData = pTypeSignalData;
    object_ptr<const IStreamType> pConstTypeIMUData = pTypeIMUData;
    object_ptr<const IStreamType> pConstTypePositionData = pTypePositionData;

    //register input pin(s)
    Register(m_oReaderSpeed, "speed", pConstTypeSignalData);
    Register(m_oReaderIMU, "imu", pConstTypeIMUData);
    Register(m_oReaderPositionReset, "pos_reset", pConstTypePositionData);
   
    //register output pin
    Register(m_oWriter, "position", pConstTypePositionData);
    //Reset Filter Covariances
    ResetFilter();
    // initialize other variables
    m_f32Speed = 0;
    m_f32YawRate = 0;

    m_ui32ArduinoTimestamp = 0;

    m_ui32Cnt = 0;

}




/*! implements the configure function to read ALL Properties */
tResult cLITD_IMULocalization::Configure()
{
    LOG_INFO("Configure");
    
    m_state.at<double>(0) = m_f32XOffset;
    m_state.at<double>(1) = m_f32YOffset;
    m_state.at<double>(2) = m_f32HeadingOffset;

    RETURN_NOERROR;
}



/*! funtion will be executed each time a trigger occured */
tResult cLITD_IMULocalization::Process(tTimeStamp tmTimeOfTrigger)
{
   // LOG_INFO("Process");
    object_ptr<const ISample> pReadSample;

    //LOG_INFO(cString::Format("process: %lu", tmTimeOfTrigger).GetPtr());

    while(IS_OK(m_oReaderPositionReset.GetNextSample(pReadSample))) {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pReadSample);
        tFloat32 x, y, heading, speed;
        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &x));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &y));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &heading));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.speed, &speed));

        ResetFilter();
        m_state.at<double>(0) = x;
        m_state.at<double>(1) = y;
        m_state.at<double>(2) = heading;
        m_state.at<double>(3) = 0;
        m_state.at<double>(4) = speed;
        m_state.at<double>(5) = 0;
    }

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

    RETURN_NOERROR;
}

tVoid cLITD_IMULocalization::ResetFilter()
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
tFloat32 cLITD_IMULocalization::angleDiff(tFloat32 angle1, tFloat32 angle2)
{
    LOG_INFO("angleDiff");
    // normalization
    angle1 = normalizeAngle(angle1, static_cast<tFloat32>(M_PI));
    angle2 = normalizeAngle(angle2, static_cast<tFloat32>(M_PI));

    // compute difference and normalize in [-pi pi]
    return normalizeAngle(angle2 - angle1, 0);
}

/*! calculates normalized angle */
tFloat32 cLITD_IMULocalization::normalizeAngle(tFloat32 alpha, tFloat32 center)
{
    //LOG_INFO("normalizeAngle");
    return mod(alpha - center + static_cast<tFloat32>(M_PI), 2.0f*static_cast<tFloat32>(M_PI)) + center - static_cast<tFloat32>(M_PI);
}

/*! calculates modulus after division */
tFloat32 cLITD_IMULocalization::mod(tFloat32 x, tFloat32 y)
{
    //LOG_INFO("mod");
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


/*! sends position data out */
tResult cLITD_IMULocalization::sendPositionStruct(const tTimeStamp &timeOfFix, const tFloat32 &f32X, const tFloat32 &f32Y, const tFloat32 &f32Radius, const tFloat32 &f32Heading, const tFloat32 &f32Speed)
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
tResult cLITD_IMULocalization::ProcessInerMeasUnitSample(tTimeStamp tmTimeOfTrigger, const adtf::streaming::ISample &oSample)
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
