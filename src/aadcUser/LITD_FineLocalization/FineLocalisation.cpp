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

#include "FineLocalisation.h"
#include "PixelMetricTransformer.h"
#include "ADTF3_OpenCV_helper.h"
#include "ADTF3_helper.h"


#define DEG2RAD M_PI/180

ADTF_PLUGIN(LABEL_FINE_LOCALISATION_FILTER, cFineLocalisation)

cFineLocalisation::cFineLocalisation()
{
    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    create_pin(*this, m_oReader, "inBirdsEye", pType);

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
    object_ptr<const IStreamType> pConstTypePositionData = pTypePositionData;


    //Register input pin
    create_pin(*this, m_oPosReader, "inPosition", pConstTypePositionData);

    //Register output pin
    filter_create_pin(*this, m_oPosWriter, "outPosition", pConstTypePositionData);

    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }

    object_ptr<const IStreamType> pConstTypeSignalValue = pTypeSignalValue;

    filter_create_pin(*this, m_oConfWriter, "outConfidence", pConstTypeSignalValue);

    object_ptr<IStreamType> pBoolSignalValueStreamType;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeSignalValue, m_BoolSignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue"), m_ddlBoolSignalValueId.bValue));
    }
    else
    {
        LOG_INFO("No mediadescription for tBoolSignalValue found!");
    }

    object_ptr<const IStreamType> pConstBoolSignalValueStreamType = pBoolSignalValueStreamType;

    //Register input pin
    create_pin(*this, m_oInitalReader, "inInitial", pConstBoolSignalValueStreamType);

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
    RegisterPropertyVariable("Angle Iteration Count", angleIterCnt);
    RegisterPropertyVariable("Angle Range Min [°]", angleRangeMin);
    RegisterPropertyVariable("Angle Range Max [°]", angleRangeMax);
    RegisterPropertyVariable("Image subsample rate", subSampleRate);
    RegisterPropertyVariable("Initial angle range extension Faktor", initialAngleRangeExtensionFaktor);
    RegisterPropertyVariable("Initial Pos search radius [m]", initialPosSearchRadius);
    RegisterPropertyVariable("Initial Pos search increment [m]", initialPosSearchInc);



    create_inner_pipe(*this, cString::Format("%s_trigger", "inPosition"), "inPosition", [&](tTimeStamp tmTime) -> tResult
    {
        return ProcessPosition(tmTime);
    });

    create_inner_pipe(*this, cString::Format("%s_trigger", "inInitial"), "inInitial", [&](tTimeStamp tmTime) -> tResult
    {
        return ProcessInitial(tmTime);
    });

    create_inner_pipe(*this, cString::Format("%s_trigger", "inBirdsEye"), "inBirdsEye", [&](tTimeStamp tmTime) -> tResult
    {
        return AcceptImage(tmTime);
    });

	// register the runner which processes the fine localization in the background
	if (IS_FAILED(InitializeFineLocalizationThread()))
		LOG_ERROR("Failed initializing the FineLocalization Thread in the constructor");
}

cFineLocalisation::~cFineLocalisation()
{
	// tell the thread to go home
	m_runner_reset_signal = true;
	Deconfigure();
}

tResult cFineLocalisation::Deconfigure()
{
	// avoid double-free through the destructor
	if (m_deconfigured) RETURN_NOERROR;
	delete m_sck_pair;
	m_deconfigured = true;
	RETURN_NOERROR;
}

tResult cFineLocalisation::Init(const tInitStage eStage)
{
    RETURN_IF_FAILED(cFilter::Init(eStage));
    if (eStage == StageFirst)
    {
        // press "Init"
        RETURN_IF_FAILED(Configure());
    }
    RETURN_NOERROR;
}

tResult cFineLocalisation::Shutdown(const tInitStage eStage)
{
	RETURN_IF_FAILED(cFilter::Shutdown(eStage));
	if (eStage == StageFirst)
	{
		// press "Deinit"
		RETURN_IF_FAILED(Deconfigure());
	}
	RETURN_NOERROR;
}

tResult cFineLocalisation::Start()
{
	m_runner_reset_signal = false;

	// wake up the thread
	std::unique_lock<std::mutex> lk(m_runner_mutex);
	m_parent_ready = true;
	m_runner_cv.notify_one();

	RETURN_NOERROR;
}

tResult cFineLocalisation::Stop()
{
	// tell the thread to stop
	m_runner_reset_signal = true;

	// break any blocking recv calls with an empty message
	auto msg = zmq::message_t(0);
	m_sck_pair->send(msg);

	RETURN_NOERROR;
}

tResult cFineLocalisation::Configure()
{
    //TODO transfer data to new thread and start new thread
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
	RETURN_IF_FAILED(_runtime->GetObject(m_pZeroMQService));

	// configure class
    cFilename mapPathResolved = mapPath;
    adtf::services::ant::adtf_resolve_macros(mapPathResolved);
    locator.setMap(const_cast<char*>(mapPathResolved.GetPtr()));
    locator.setAngleSearchSpace(angleRangeMin, angleRangeMax, angleIterCnt);
    locator.setPosSearchSpace(initialPosSearchRadius, initialPosSearchInc, initialAngleRangeExtensionFaktor);
    affineMat[0][0] = mat00;
    affineMat[0][1] = mat01;
    affineMat[0][2] = mat02;
    affineMat[1][0] = mat10;
    affineMat[1][1] = mat11;
    affineMat[1][2] = mat12;
    locator.setPixelMetricTransformer(PixelMetricTransformer(affineMat));
    locator.setSearchSpace(propSearchSpaceSize);
    sampleCnt = 0;

	// create a pair socket that will distribute messages to the runners socket
	m_sck_pair = new zmq::socket_t(*m_pZeroMQService->GetContext(), ZMQ_PAIR);

	// do not wait at close time
	int linger = 0;
	m_sck_pair->setsockopt(ZMQ_LINGER, &linger, sizeof linger);
	// limit queue length (shared for inproc sockets)
	int hwm = m_queue_length / 2;
	m_sck_pair->setsockopt(ZMQ_SNDHWM, &hwm, sizeof hwm);

	m_sck_pair->bind(GetPairSocketAddress());

    RETURN_NOERROR;
}

tResult cFineLocalisation::ProcessPosition(tTimeStamp tmTimeOfTrigger)
{
	// TODO: potential risk of deadlocks (maybe use https://en.cppreference.com/w/cpp/thread/timed_mutex/try_lock_until)
	// get exclusive access to the position
	std::lock_guard<std::mutex> oGuard(m_position_mutex);

    object_ptr<const ISample> pPosReadSample;

    if(IS_OK(m_oPosReader.GetLastSample(pPosReadSample))) {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pPosReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &x));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &y));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &heading));
        //adjust heading for inversion of back camera
        heading = heading + headingOffset*DEG2RAD;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.speed, &speed));
        recievedPosition = true;

    } else {
        LOG_ERROR("!!Failed to read last Position Sample!!");
    }
    RETURN_NOERROR;
}

tResult cFineLocalisation::ProcessInitial(tTimeStamp tmTimeOfTrigger)
{
    // TODO: potential risk of deadlocks (maybe use https://en.cppreference.com/w/cpp/thread/timed_mutex/try_lock_until)
    // get exclusive access to the position

    object_ptr<const ISample> pInitialReadSample;

    if(IS_OK(m_oInitalReader.GetLastSample(pInitialReadSample))) {
        auto oDecoder = m_BoolSignalValueSampleFactory.MakeDecoderFor(*pInitialReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.bValue, &initial));

    } else {
        LOG_ERROR("!!Failed to read last Bool Sample!!");
    }
    RETURN_NOERROR;
}

tResult cFineLocalisation::AcceptImage(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    while (IS_OK(m_oReader.GetNextSample(pReadSample)) && recievedPosition)
    {
        if(sampleCnt % subSampleRate == 0) {
            object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
            // lock read buffer and send to worker thread
            if (IS_OK(pReadSample->Lock(pReadBuffer))) {
				bool returncode = false;
				const size_t imageSize = 3 * m_sImageFormat.m_ui32Width * m_sImageFormat.m_ui32Height;

				// send width, height and the image buffer (each value will be copied into the message buffer)
				if (m_sck_pair->send(&m_sImageFormat.m_ui32Width, sizeof m_sImageFormat.m_ui32Width, ZMQ_SNDMORE | ZMQ_DONTWAIT))
					if (m_sck_pair->send(&m_sImageFormat.m_ui32Height, sizeof m_sImageFormat.m_ui32Height, ZMQ_SNDMORE | ZMQ_DONTWAIT))
						if (m_sck_pair->send(pReadBuffer->GetPtr(), imageSize, ZMQ_DONTWAIT))
							returncode = true;

				if (!returncode)
					LOG_ERROR("Queue overflow in the fine localization thread. Reduce the fps !!!");
            }
            sampleCnt = 0;
        }
        sampleCnt++;
    }
    RETURN_NOERROR;
}

tResult cFineLocalisation::ProcessImage(Mat* bvImage)
{
#ifdef _DEBUG
	const tTimeStamp start = cHighResTimer::GetTime();
#endif

	//[dx, dy, headingOffset, confidence]l
	float *location = locator.localize(*bvImage, heading, x, y, axleToPicture, initial);

#ifdef _DEBUG
	const tTimeStamp end = cHighResTimer::GetTime();
	LOG_DUMP("Localization run-time: %.2f ms", (end - start) / 1000.0);
#endif

	if ((location[3] > angleRangeMax || location[3] < angleRangeMin) && !initial) LOG_ERROR("caclualted angle offset out of bounds: %f", location[3]);
	LOG_INFO("found deltas: %.2f %.2f %.4f", location[0], location[1], location[2]);
	
	// transmit result (with a lock!)
	return TransmitResult(location);
}

tResult cFineLocalisation::TransmitResult(float* location)
{
	// get exclusive access to the position
	std::lock_guard<std::mutex> oGuard(m_position_mutex);

	object_ptr<ISample> pWriteSample;
	RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
	{
		auto oCodec = m_PositionSampleFactory.MakeCodecFor(pWriteSample);

		RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.x, x + location[0]));
		RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.y, y + location[1]));
		RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.heading, heading - headingOffset*DEG2RAD + location[2]));
		RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.speed, speed));
	}

	transmitSignalValue(m_oConfWriter, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, location[3]);
	m_oPosWriter << pWriteSample << flush << trigger;

	RETURN_NOERROR;
}

tResult cFineLocalisation::InitializeFineLocalizationThread()
{
	// thread for the fine localization, where image data is processed and output samples are sent
	const object_ptr<IRunner> zmq_thread = ::adtf::ucom::make_object_ptr<cRunner>("cv_thread", [&](tTimeStamp /* tmTime */) -> tResult
	{
		/* --- (1) wait for parent initialization --- */

		LOG_DUMP("FineLocalization thread started. Waiting for signal ...");

		// the runner needs to wait until the parent is done with binding the socket and initializing
		std::unique_lock<std::mutex> lk(m_runner_mutex);
		while (!m_parent_ready)
		{
			m_runner_cv.wait(lk);
			if (!m_parent_ready)
				LOG_ERROR("Spurious wake-up in the FineLocalization thread detected.");
		}

		/* --- (2) connect PAIR socket to parent thread --- */

		auto pair_socket = zmq::socket_t(*m_pZeroMQService->GetContext(), ZMQ_PAIR);

		// do not wait at close time
		int linger = 0;
		pair_socket.setsockopt(ZMQ_LINGER, &linger, sizeof linger);
		// limit queue length
		int hwm = m_queue_length / 2;
		pair_socket.setsockopt(ZMQ_RCVHWM, &hwm, sizeof hwm);

		const std::string address = GetPairSocketAddress();
		pair_socket.connect(address);

		LOG_DUMP("FineLocalization thread initialized. Connected to %s", address.c_str());

		while (!m_runner_reset_signal && pair_socket.connected())
		{
			/* --- (3) receive image --- */

			// an empty message from the parent thread signals a break condition
			// message.size() > 0 IMPLIES message.more()

			zmq::message_t message_width;
			pair_socket.recv(&message_width);
			if (message_width.size() == 0 && m_runner_reset_signal) break;
			assert(!(message_width.size() > 0 && !message_width.more()));

			zmq::message_t message_height;
			pair_socket.recv(&message_height);
			if (message_height.size() == 0 && m_runner_reset_signal) break;
			assert(!(message_height.size() > 0 && !message_height.more()));

			zmq::message_t message_image;
			pair_socket.recv(&message_image);
			if (message_image.size() == 0 && m_runner_reset_signal) break;
			assert(!(message_image.size() > 0 && message_image.more()));

			// TODO: check if the queue is really empty! if not, log an error and maybe clear the queue!

			// check for stop signal from parent
			if (m_runner_reset_signal) break;

			/* --- (4) process image --- */

			tUInt32 width = *static_cast<tUInt32*>(message_width.data());
			tUInt32 height = *static_cast<tUInt32*>(message_height.data());
			Mat bvImage = Mat(Size(width, height), CV_8UC3, static_cast<unsigned char *>(message_image.data()));

			ProcessImage(&bvImage);
		}

		pair_socket.close();

		// TODO: allow reconnecting ability
		LOG_WARNING("FineLocalization thread runner stopped. Starting the session again will fail.");
		RETURN_NOERROR;
	});

	RETURN_IF_FAILED(cRuntimeBehaviour::RegisterRunner(zmq_thread));
	RETURN_NOERROR;
}

/**
* \brief Resolve the inproc:// socket address for the PAIR socket between the
* parent thread and the ZeroMQ I/O thread. The address will be inferred from the filter name,
* such that we have a unique address per filter instantiation.
* \return The "inproc://FILTER_NAME" address
*/
std::string cFineLocalisation::GetPairSocketAddress() const
{
	// resolve the filter name
	std::string name;
	GetName(adtf_string<std::string>(&name));

	// replace spaces with underscores
	std::replace(name.begin(), name.end(), ' ', '_');

	// append the inproc prefix
	std::ostringstream name_stream;
	name_stream << "inproc://" << name;

	return name_stream.str();
}