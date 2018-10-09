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
#include <zmq.hpp>
#define _USE_MATH_DEFINES
#include "stdafx.h"
#include "FineLocator.h"

//*************************************************************************************************
#define CID_FINE_LOCALISATION_FILTER "finelocalization.filter.user.aadc.cid"
#define LABEL_FINE_LOCALISATION_FILTER "LITD FineLocalization"


/*! the main class of the open cv template. */
class cFineLocalisation : public cFilter
{
public:
    ADTF_CLASS_ID_NAME(cFineLocalisation, CID_FINE_LOCALISATION_FILTER, LABEL_FINE_LOCALISATION_FILTER);
	ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(IZeroMQService),
		REQUIRE_INTERFACE(adtf::services::IReferenceClock));

    // necessary for proper behaviour of the create_inner_pipe call
    using cRuntimeBehaviour::RegisterRunner;
    using cRuntimeBehaviour::RegisterInnerPipe;

private:

    //Pins
    /*! Reader of an InPin. */
    cPinReader m_oReader;
    cPinReader m_oPosReader;
    cPinReader m_oInitalReader;
    /*! Writer to an OutPin. */
    cPinWriter m_oPosWriter;
    cPinWriter m_oConfWriter;

    /*! A position identifier*/
    struct tPositionIndex
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    } m_ddlPositionIndex;

    /*! A signal value identifier. */
    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

	/*! A signal value identifier. */
	struct tBoolSignalValueId
	{
		tSize ui32ArduinoTimestamp;
		tSize bValue;
	} m_ddlBoolSignalValueId;

    /*! The position signal value sample factory */
    cSampleCodecFactory m_PositionSampleFactory;
    /*! The signal value sample factory */
    cSampleCodecFactory m_SignalValueSampleFactory;
	/*! The bool signal value sample factory */
	cSampleCodecFactory m_BoolSignalValueSampleFactory;
	//Stream Formats
    /*! The input format */

    adtf::streaming::tStreamImageFormat m_sImageFormat;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

	/*! ZeroMQ context and socket */
	object_ptr<IZeroMQService> m_pZeroMQService;
	zmq::socket_t* m_sck_pair = nullptr;

	/*! synchronized start signal for the thread */
	std::condition_variable m_runner_cv;
	std::mutex m_runner_mutex;
	bool m_parent_ready = false;

	/* how many messages shall be queued in memory at a maximum (actual limit might be 60 - 70% lower) */
	tInt m_queue_length = 10;

	/* stop signal for the thread */
	std::atomic<bool> m_runner_reset_signal{ false };

	/*! mutex for shared access to position */
	std::mutex m_position_mutex;

	/* the deconfiguration routine can only be done once */
	bool m_deconfigured = false;

    tFloat32 x, y, speed, heading;
	tBool initial = tFalse;
    FineLocator locator;
    // [ 142.8,    0. ,   25. ],
    // [   0. , -139.5,  567. ]
    property_variable<tFloat32> mat00 = 142.8;
    property_variable<tFloat32> mat01 = 0;
    property_variable<tFloat32> mat02 = 25;
    property_variable<tFloat32> mat10 = 0;
    property_variable<tFloat32> mat11 =-139.5;
    property_variable<tFloat32> mat12 = 567;
    property_variable<tFloat32> axleToPicture = 0.73;
    property_variable<tFloat32> headingOffset = 00;
    property_variable<cFilename> mapPath = cFilename("/home/aadc/share/adtf/data/scaledMap.png");
    property_variable<tInt32> propSearchSpaceSize = 20;
    property_variable<tInt32> angleIterCnt = 11;
    property_variable<tFloat32> angleRangeMin = -5;
    property_variable<tFloat32> angleRangeMax =  5;
    property_variable<tFloat32> initialAngleRangeExtensionFaktor =  9;
    property_variable<tFloat32> initialPosSearchRadius =  0.5;
    property_variable<tFloat32> initialPosSearchInc =  0.1;
    property_variable<tInt32> subSampleRate =  30;
    tInt32 sampleCnt = 0;

    double affineMat [2][3] = {{mat00, mat01, mat02}, {mat10, mat11, mat12}};
    bool recievedPosition = false;


public:

    /*! Default constructor. */
    cFineLocalisation();

    /*! Destructor. */
    ~cFineLocalisation() override;

    tResult Init(tInitStage eStage) override;

	tResult Shutdown(tInitStage eStage) override;

	tResult Start() override;

	tResult Stop() override;

    tResult Configure();

	tResult Deconfigure();

private:

	tResult AcceptImage(tTimeStamp tmTimeOfTrigger);

	tResult ProcessPosition(tTimeStamp tmTimeOfTrigger);

	tResult ProcessInitial(tTimeStamp tmTimeOfTrigger);

	tResult ProcessImage(Mat* bvImage);

	tResult TransmitResult(float* location);

	tResult InitializeFineLocalizationThread();

	std::string GetPairSocketAddress() const;

};


//*************************************************************************************************
