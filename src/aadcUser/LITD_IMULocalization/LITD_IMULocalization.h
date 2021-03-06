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

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#define CID_IMULOCALIZATION_FILTER "imulocalization.filter.user.aadc.cid"
#define LABEL_IMULOCALIZATION_FILTER "LITD IMULocalization"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;


#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180


/*! the main class of the marker positioning. */
class cLITD_IMULocalization : public cFilter
{
public:
	ADTF_CLASS_ID_NAME(cLITD_IMULocalization, CID_IMULOCALIZATION_FILTER, LABEL_IMULOCALIZATION_FILTER);

	// necessary for proper behaviour of the create_inner_pipe call
	using cRuntimeBehaviour::RegisterRunner;
	using cRuntimeBehaviour::RegisterInnerPipe;

private:



    /*! speed scale */
    adtf::base::property_variable<tFloat32> m_f32SpeedScale = 1.0f;

    /*! camera offset */
    adtf::base::property_variable<tFloat32> m_f32CameraOffsetLat = 0.0f;
    /*! distance to the rear axle */
    adtf::base::property_variable<tFloat32> m_f32CameraOffsetLon = 0.295f;

    //Positioning offsets
    adtf::base::property_variable<tFloat32> m_f32XOffset = 0.0f;
    adtf::base::property_variable<tFloat32> m_f32YOffset = 0.0f;
    adtf::base::property_variable<tFloat32> m_f32HeadingOffset = 0.0f;


    /*! Reader of an InPin speed. */
    cPinReader m_oReaderSpeed;
    /*! Reader of an InPin IMU. */
    cPinReader m_oReaderIMU;
    /*! Reader of an InPin, Position reset */
    cPinReader m_oReaderPositionReset;

    /*! Writer to an OutPin. */
    cPinWriter m_oWriter;

    /*! The codec factory */
    cSampleCodecFactory m_oCodecFactory;

    /*! The ddl indices for a tInerMeasUnitData */
    struct
    {
        tSize timeStamp;
        tSize A_x;
        tSize A_y;
        tSize A_z;
        tSize G_x;
        tSize G_y;
        tSize G_z;
        tSize M_x;
        tSize M_y;
        tSize M_z;
        tSize roll;
        tSize pitch;
        tSize yaw;
    } m_ddlInerMeasUnitDataIndex;

    /*! The imu data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_IMUDataSampleFactory;

    /*! The ddl indices for a tSignalValue */
    struct
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalDataIndex;

    /*! The signal data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_SignalDataSampleFactory;





    /*! The ddl indices for a tPosition */
    struct
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    } m_ddlPositionIndex;

    /*! The position sample factory */
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;

    /*! speed estimate */
    tFloat32 m_f32Speed;
    /*! The 32 yaw rate */
    tFloat32 m_f32YawRate;

    /*! The 32 arduino timestamp */
    tUInt32 m_ui32ArduinoTimestamp;

    /*! currently processed road-sign */
    tInt16 m_i16ID;
    /*! Size of the 32 marker */
    tFloat32 m_f32MarkerSize;

    /*! The ticks */
    tTimeStamp m_ticks;

    /*! EKF variables */
    Mat m_state; /*! filter state {X}  x, y, heading, radius, speed, accel*/
    Mat m_errorCov; /*! error covariance matrix {P} */
    Mat m_processCov; /*! process covariance matrix {Q} */
    Mat m_transitionMatrix; /*! state transition matrix {F} */

    tBool m_isInitialized; /*! initialization state of the filter */

    tBool m_isFirst = true;

    /*! Number of 32s */
    tInt m_ui32Cnt;

    /*!
     * helper functions.
     *
     * \param   x   A tFloat32 to process.
     * \param   y   A tFloat32 to process.
     *
     * \return  A tFloat32.
     */
    tFloat32 mod(tFloat32 x, tFloat32 y);

    /*!
     * Normalize angle.
     *
     * \param   alpha   The alpha.
     * \param   center  The center.
     *
     * \return  A tFloat32.
     */
    tFloat32 normalizeAngle(tFloat32 alpha, tFloat32 center);

    /*!
     * Angle difference.
     *
     * \param   angle1  The first angle.
     * \param   angle2  The second angle.
     *
     * \return  A tFloat32.
     */
    tFloat32 angleDiff(tFloat32 angle1, tFloat32 angle2);

    /*!
     * Gets the time.
     *
     * \return  The time.
     */
    tTimeStamp GetTime();

    /*!
     * Process the iner meas unit sample.
     *
     * \param   tmTimeOfTrigger The time time of trigger.
     * \param   oSample         The sample.
     *
     * \return  Standard Result Code.
     */
    tResult ProcessInerMeasUnitSample(tTimeStamp tmTimeOfTrigger, const adtf::streaming::ISample &oSample);

    /*!
     * Sends a position structure.
     *
     * \param   timeOfFix   The time of fix.
     * \param   f32X        The 32 x coordinate.
     * \param   f32Y        The 32 y coordinate.
     * \param   f32Radius   The 32 radius.
     * \param   f32Heading  The 32 heading.
     * \param   f32Speed    The 32 speed.
     *
     * \return  Standard Result Code.
     */
    tResult sendPositionStruct(const tTimeStamp &timeOfFix, const tFloat32 &f32X, const tFloat32 &f32Y, const tFloat32 &f32Radius,
        const tFloat32 &f32Heading, const tFloat32 &f32Speed);

public:

    cLITD_IMULocalization();

    virtual ~cLITD_IMULocalization() = default;

	tResult Init(tInitStage eStage) override;

    tResult Configure();
  
    tResult ProcessSpeed(tTimeStamp tmTimeOfTrigger);
	tResult ProcessImu(tTimeStamp tmTimeOfTrigger);
	tResult ProcessReset(tTimeStamp tmTimeOfTrigger);

	tVoid ResetFilter();

};
