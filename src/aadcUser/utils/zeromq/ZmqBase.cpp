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

#include "ZmqBase.h"

#include <aadc_structs.h>

#include <cstring>
#include <sstream>
#include <algorithm>

// forward declaration
void zmq_free_message(void *data, void *hint);

cZmqBase::cZmqBase()
{
	RegisterPropertyVariable("ZeroMQ Socket Address", m_server_socket_address);
	RegisterPropertyVariable("ZeroMQ Queue Length", m_queue_length);
	RegisterPropertyVariable("ZeroMQ Subsample Factor", m_subsample_factor);

	// register the runner which processes zeromq messages in the background
	if (IS_FAILED(InitializeZeroMQThread()))
		LOG_ERROR("Failed initializing the ZeroMQ Thread in the constructor");
}

cZmqBase::~cZmqBase()
{
	// tell the zeromq thread to stop
	m_runner_reset_signal = true;
	Deconfigure();
}

tResult cZmqBase::Init(const tInitStage eStage)
{
	RETURN_IF_FAILED(cFilter::Init(eStage));
	if (eStage == StageFirst)
	{
		// press "Init"
		RETURN_IF_FAILED(Configure());
	}
	RETURN_NOERROR;
}

tResult cZmqBase::Shutdown(const tInitStage eStage)
{
	RETURN_IF_FAILED(cFilter::Shutdown(eStage));
	if (eStage == StageFirst)
	{
		// press "Deinit"
		RETURN_IF_FAILED(Deconfigure());
	}
	RETURN_NOERROR;
}

tResult cZmqBase::Configure()
{
	RETURN_IF_FAILED(InitializeInputPins());
	RETURN_IF_FAILED(InitializeOutputPins());

	// hey, dependency injector, gimme' my objects!
	RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
	RETURN_IF_FAILED(_runtime->GetObject(m_pZeroMQService));
	
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

tResult cZmqBase::Deconfigure()
{
	// avoid double-free through the destructor
	if (m_deconfigured) RETURN_NOERROR;

	// close and delete socket to our I/O thread
	delete m_sck_pair;
	
	// deallocate pin readers
	for (const auto& pair : m_pinReaders) {
		delete pair.second;
	}

	// deallocate pin writers
	for (const auto& pair : m_pinWriters) {
		delete pair.second;
	}

	m_deconfigured = true;
	RETURN_NOERROR;
}

tResult cZmqBase::Start()
{
	m_runner_reset_signal = false;

	// wake up the zeromq thread
	std::unique_lock<std::mutex> lk(m_runner_mutex);
	m_parent_ready = true;
	m_runner_cv.notify_one();

	RETURN_NOERROR;
}

tResult cZmqBase::Stop()
{
	// tell the zeromq thread to stop
	m_runner_reset_signal = true;

	// break any blocking recv calls with an empty message
	auto msg = zmq::message_t(0);
	m_sck_pair->send(msg);

	RETURN_NOERROR;
}

tResult cZmqBase::InitializePins(std::vector<ZmqPinDef> &pin_definitions, const bool isReader)
{
	for (auto& pin_def : pin_definitions)
	{
		const std::string pinName = std::get<0>(pin_def);
		const eZmqStruct pinType = std::get<1>(pin_def);

		// resolve or create the stream type
		object_ptr<IStreamType>* streamType = GetStreamType(pinType);
		if (!streamType) RETURN_ERROR_DESC(-1, cString::Format("GetStreamType(pinType) for pin %s yielded a nullptr.", pinName.c_str()));

		if (isReader)
		{
			// input pin
			const auto pinReader = new cPinReader();
			m_pinReaders[pinName] = pinReader;
			RETURN_IF_FAILED(create_pin(*this, *pinReader, pinName.c_str(), *streamType));

			if (pinType == Image)
			{
				pinReader->SetAcceptTypeCallback([this, pinName](const adtf::ucom::ant::iobject_ptr<const IStreamType> &pType) -> tResult {
					// expanded: ChangeType(*m_pinReaders[pinName], m_sImageFormat, *pType.Get());
					if (*pType.Get() == stream_meta_type_image())
					{
						object_ptr<const IStreamType> pTypeInput;
						*m_pinReaders[pinName] >> pTypeInput;
						get_stream_type_image_format(m_sImageFormat, *pTypeInput);
					}
					else
					{
						RETURN_ERROR(ERR_INVALID_TYPE);
					}

					RETURN_NOERROR;
				});
			}
		}
		else
		{
			// output pin
			const auto pinWriter = new cPinWriter();
			m_pinWriters[pinName] = pinWriter;
			RETURN_IF_FAILED(filter_create_pin(*this, *pinWriter, pinName.c_str(), *streamType));
		}
	}

	RETURN_NOERROR;
}

/**
 * \brief Register cPinWriters with their respective names
 */
tResult cZmqBase::InitializeOutputPins()
{
	return InitializePins(m_outputs, false);
}

/**
 * \brief Register cPinReaders with their respective names. trigger lambdas on the trigger pins
 */
tResult cZmqBase::InitializeInputPins()
{
	RETURN_IF_FAILED(InitializePins(m_inputs, true));

	// do we also need to create an inner trigger pipe?
	for (auto& trigger_pin : m_triggers)
	{
		const char* trigger_pin_cstr = trigger_pin.c_str();

		// search if we initialized a pin reader for this pin
		auto it = m_pinReaders.find(trigger_pin);
		if (it != m_pinReaders.end())
		{
			// execute this lambda if the pin is triggered
			RETURN_IF_FAILED(create_inner_pipe(*this, cString::Format("%s_trigger", trigger_pin_cstr), trigger_pin_cstr, [&](tTimeStamp tmTime) -> tResult
			{
				if (m_sck_pair && m_sck_pair->connected())
				{
					ProcessInputs(tmTime);
				}
				else
				{
					LOG_ERROR("Socket pair not initialized or not connected. Can not process trigger event.");
				}

				RETURN_NOERROR;
			}));
		}
		else
		{
			LOG_ERROR("Could not create trigger pin on nonexistent pin %s", trigger_pin_cstr);
		}
	}

	RETURN_NOERROR;
}

/**
 * \brief Define and register the ZeroMQ I/O thread, which will actually send the samples and poll for replies
 */
tResult cZmqBase::InitializeZeroMQThread()
{
	// thread for zeromq, where requests are sent and replies are polled
	const object_ptr<IRunner> zmq_thread = ::adtf::ucom::make_object_ptr<cRunner>("zmq_thread", [&](tTimeStamp /* tmTime */) -> tResult
	{
		/* --- (1) wait for parent --- */

		LOG_DUMP("ZMQ I/O thread started. Waiting for signal ...");

		// the runner needs to wait until the parent is done with binding the socket
		std::unique_lock<std::mutex> lk(m_runner_mutex);
		while (!m_parent_ready)
		{
			m_runner_cv.wait(lk);
			if (!m_parent_ready)
				LOG_ERROR("Spurious wake-up in the ZMQ I/O thread detected.");
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

		/* --- (3) connect REQ socket to server --- */

		zmq::socket_t* client_socket = InitializeClientSocket();

		LOG_DUMP("ZMQ I/O thread initialized. Connected to %s", cString(m_server_socket_address).GetPtr());

		const size_t num_outputs = m_outputs.size();
		bool lastConnectionState = true;

		while (!m_runner_reset_signal && pair_socket.connected())
		{
			/* --- (4) receive and forward data --- */

			int pair_flags;
			tTimeStamp start = 0;
			do
			{
				zmq::message_t message;
				pair_socket.recv(&message);

				if (start == 0) start = cHighResTimer::GetTime();

				// an empty message from the parent thread signals a break condition
				if (message.size() == 0 && m_runner_reset_signal)
					break;

				// send to server
				pair_flags = message.more() ? ZMQ_SNDMORE : 0;
				client_socket->send(message, pair_flags);
			} while (pair_flags == ZMQ_SNDMORE);

			// check for stop signal from parent
			if (m_runner_reset_signal) break;

			/* --- (5) wait for server reply (with timeout) --- */

			zmq::pollitem_t items[] = { { *client_socket, 0, ZMQ_POLLIN, 0 } };
			zmq::poll(&items[0], 1, ZMQ_REQUEST_TIMEOUT);

			// polling successful
			if (items[0].revents & ZMQ_POLLIN)
			{
				int server_flags;
				size_t outputIndex = 0;
				bool empty_reply = false;
				do
				{
					zmq::message_t message;
					client_socket->recv(&message);
					server_flags = message.more() ? ZMQ_SNDMORE : 0;
					bool empty = message.size() == 0;

					// is the first message empty and has no ZMQ_SNDMORE flag?
					if (empty && outputIndex == 0 && server_flags == 0)
						empty_reply = true;

					// send output samples
					if (!empty)
						ProcessOutput(&message, outputIndex);

					// next pin
					outputIndex++;
				} while (server_flags == ZMQ_SNDMORE);

				#ifdef _DEBUG
					const tTimeStamp end = cHighResTimer::GetTime();
					LOG_DUMP("Sample latency: %.2f ms", (end - start) / 1000.0);
				#endif

				// sanity checks
				if (!empty_reply)
				{
					if (outputIndex < num_outputs)
						LOG_ERROR("Expected %d output pin structs, but we only received %d messages. Missing pins will not flush any samples.", num_outputs, outputIndex);
					else if (outputIndex > num_outputs)
						LOG_ERROR("Expected %d output pin structs, but we received %d messages. Additional messages will be discarded.", num_outputs, outputIndex);					
				}
				else LOG_DUMP("Received empty reply. No samples will be sent.");

				if (!lastConnectionState)
				{
					LOG_INFO("Successfully reconnected after timeout.");
					lastConnectionState = true;
				}
			}
			else
			{
				LOG_WARNING("Server socket timed out after waiting %d ms for a reply.", ZMQ_REQUEST_TIMEOUT);

				// reopen the socket because we can't send another message on this REQ socket
				delete client_socket;
				client_socket = InitializeClientSocket();
				lastConnectionState = false;
			}
		}

		pair_socket.close();
		delete client_socket;

		// TODO: allow reconnecting ability
		LOG_WARNING("ZeroMQ I/O thread runner stopped. Starting the session again will fail.");
		RETURN_NOERROR;
	});

	RETURN_IF_FAILED(cRuntimeBehaviour::RegisterRunner(zmq_thread));
	RETURN_NOERROR;
}

inline zmq::socket_t* cZmqBase::InitializeClientSocket() const
{
	// create and connect the socket
	auto* client_socket = new zmq::socket_t(*m_pZeroMQService->GetContext(), ZMQ_REQ);
	client_socket->connect(static_cast<string>(cString(m_server_socket_address)));

	// do not wait at close time
	int linger = 0;
	client_socket->setsockopt(ZMQ_LINGER, &linger, sizeof linger);

	return client_socket;
}

#define PROCESS_INPUT_SAMPLE_HELPER(_TYPE_, _CONTENT_) { \
	auto* data = new _TYPE_(); \
	_CONTENT_ \
	zmq::message_t message(data, sizeof(_TYPE_), zmq_free_message, nullptr); \
	returncode = m_sck_pair->send(message, flags); \
	break; \
}

tResult cZmqBase::ProcessInputs(tTimeStamp tmTimeOfTrigger)
{
	// only one thread at a time shall execute this method
	std::lock_guard<std::mutex> oGuard(m_oMutex);

	// subsample the triggers (only take every nth trigger)
	m_num_samples++;
	if (m_num_samples % m_subsample_factor != 0)
		RETURN_NOERROR;

	size_t i = 0;
	for (auto& input : m_inputs)
	{
		i++;

		// send a multipart message by setting the ZMQ_SNDMORE flag on every message except the last
		const bool isLastElement = i == m_inputs.size();
		const int flags = (isLastElement ? 0 : ZMQ_SNDMORE) | ZMQ_DONTWAIT;

		std::string pinName = std::get<0>(input);
		eZmqStruct pinType = std::get<1>(input);
		cPinReader* pinReader = m_pinReaders[pinName];
		cSampleCodecFactory* pinSampleFactory = GetSampleFactory(pinType);

		bool returncode = false;

		// if the pin is not attached or there never have been any samples, the samples might be invalid
		object_ptr<const ISample> pSample;
		if (IS_FAILED(pinReader->GetLastSample(pSample)))
		{
			// send an empty message
			zmq::message_t message(0);
			returncode = m_sck_pair->send(message, flags);
		}
		else
		{
			if (pinType == Image)
			{
				object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
				if (IS_OK(pSample->Lock(pReadBuffer))) {
					const size_t imageSize = 3 * m_sImageFormat.m_ui32Width * m_sImageFormat.m_ui32Height;
					// copy the image to avoid keeping the buffer lock for too long
					returncode = m_sck_pair->send(pReadBuffer->GetPtr(), imageSize);
				}
			}
			else
			{
				cSampleDecoder sampleDecoder = pinSampleFactory->MakeDecoderFor(*pSample);
				RETURN_IF_FAILED(sampleDecoder.IsValid());

				switch (pinType)
				{
				case Jury:
					PROCESS_INPUT_SAMPLE_HELPER(tJuryStruct, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlJuryStructIndex.i16ActionID, &data->i16ActionID));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlJuryStructIndex.i16ManeuverEntry, &data->i16ManeuverEntry));
					});

				case Driver:
					PROCESS_INPUT_SAMPLE_HELPER(tDriverStruct, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlDriverStructIndex.i16StateID, &data->i16StateID));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlDriverStructIndex.i16ManeuverEntry, &data->i16ManeuverEntry));
					});

				case SignalValue:
					PROCESS_INPUT_SAMPLE_HELPER(tSignalValue, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &data->ui32ArduinoTimestamp));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlSignalValueId.value, &data->f32Value));
					});

				case BoolSignalValue:
					PROCESS_INPUT_SAMPLE_HELPER(tBoolSignalValue, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlBoolSignalValueId.ui32ArduinoTimestamp, &data->ui32ArduinoTimestamp));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlBoolSignalValueId.bValue, &data->bValue));
					});

				case WheelData:
					PROCESS_INPUT_SAMPLE_HELPER(tWheelData, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, &data->ui32ArduinoTimestamp));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlWheelDataIndex.WheelTach, &data->ui32WheelTach));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlWheelDataIndex.WheelDir, &data->i8WheelDir));
					});

				case InerMeasUnitData:
					PROCESS_INPUT_SAMPLE_HELPER(tInerMeasUnitData, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.timeStamp, &data->ui32ArduinoTimestamp));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.A_x, &data->f32A_x));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.A_y, &data->f32A_y));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.A_z, &data->f32A_z));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.G_x, &data->f32G_x));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.G_y, &data->f32G_y));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.G_z, &data->f32G_z));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.M_x, &data->f32M_x));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.M_y, &data->f32M_y));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.M_z, &data->f32M_z));
					});

				case RoadSignExt:
					PROCESS_INPUT_SAMPLE_HELPER(tRoadSignExt, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlRoadSignExtIndex.id, &data->i16Identifier));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlRoadSignExtIndex.size, &data->f32Imagesize));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlRoadSignExtIndex.tvec, &data->af32TVec));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlRoadSignExtIndex.rvec, &data->af32RVec));
					});

				case Position:
					PROCESS_INPUT_SAMPLE_HELPER(::tPosition, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlPositionIndex.f32x, &data->f32x));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlPositionIndex.f32y, &data->f32y));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlPositionIndex.f32radius, &data->f32radius));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlPositionIndex.f32speed, &data->f32speed));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlPositionIndex.f32heading, &data->f32heading));
					});

				case Obstacle:
					PROCESS_INPUT_SAMPLE_HELPER(tObstacle, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlObstacleIndex.f32x, &data->f32x));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlObstacleIndex.f32y, &data->f32y));
					});

				case TrafficSign:
					PROCESS_INPUT_SAMPLE_HELPER(tTrafficSign, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrafficSignIndex.i16Identifier, &data->i16Identifier));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrafficSignIndex.f32x, &data->f32x));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrafficSignIndex.f32y, &data->f32y));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrafficSignIndex.f32angle, &data->f32angle));
					});

				case ParkingSpace:
					PROCESS_INPUT_SAMPLE_HELPER(tParkingSpace, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlParkingSpaceIndex.i16Identifier, &data->i16Identifier));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlParkingSpaceIndex.f32x, &data->f32x));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlParkingSpaceIndex.f32y, &data->f32y));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlParkingSpaceIndex.ui16Status, &data->ui16Status));
					});

				case Ultrasonic:
					PROCESS_INPUT_SAMPLE_HELPER(tUltrasonicStruct, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.SideLeft.value, &data->tSideLeft.f32Value));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.SideLeft.timeStamp, &data->tSideLeft.ui32ArduinoTimestamp));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.SideRight.value, &data->tSideRight.f32Value));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.SideRight.timeStamp, &data->tSideRight.ui32ArduinoTimestamp));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.RearLeft.value, &data->tRearLeft.f32Value));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.RearLeft.timeStamp, &data->tRearLeft.ui32ArduinoTimestamp));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.RearCenter.value, &data->tRearCenter.f32Value));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.RearCenter.timeStamp, &data->tRearCenter.ui32ArduinoTimestamp));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.RearRight.value, &data->tRearRight.f32Value));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.RearRight.timeStamp, &data->tRearRight.ui32ArduinoTimestamp));
					});

				case Voltage:
					PROCESS_INPUT_SAMPLE_HELPER(tVoltageStruct, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.ActuatorVoltage.value, &data->tActuatorVoltage.f32Value));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.ActuatorCell1.value, &data->tActuatorCell1.f32Value));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.ActuatorCell2.value, &data->tActuatorCell2.f32Value));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.SensorVoltage.value, &data->tSensorVoltage.f32Value));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.SensorCell1.value, &data->tSensorCell1.f32Value));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.SensorCell2.value, &data->tSensorCell2.f32Value));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.SensorCell3.value, &data->tSensorCell3.f32Value));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.SensorCell4.value, &data->tSensorCell4.f32Value));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.SensorCell5.value, &data->tSensorCell5.f32Value));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.SensorCell6.value, &data->tSensorCell6.f32Value));
					});

				case PolarCoordinate:
					PROCESS_INPUT_SAMPLE_HELPER(tPolarCoordiante, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlPolarCoordianteIndex.f32Radius, &data->f32Radius));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlPolarCoordianteIndex.f32Angle, &data->f32Angle));
					});

				case LaserScanner:
					PROCESS_INPUT_SAMPLE_HELPER(tLaserScannerData, {
						// read the data points into a std::vector<tPolarCoordiante> first
						tSize numOfScanPoints = 0;
						tResult res = sampleDecoder.GetElementValue(m_ddlLSDataId.size, &numOfScanPoints);
						const tPolarCoordiante* pCoordinates = reinterpret_cast<const tPolarCoordiante*>(sampleDecoder.GetElementAddress(m_ddlLSDataId.scanArray));
						std::vector<tPolarCoordiante> scan(numOfScanPoints);
						tPolarCoordiante scanPoint;

						for (tSize i = 0; i < numOfScanPoints; ++i)
						{
							scanPoint.f32Radius = pCoordinates[i].f32Radius;
							scanPoint.f32Angle = pCoordinates[i].f32Angle;
							scan.push_back(scanPoint);
						}

						// set the size
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlLSDataId.size, &data->ui32Size));
						// set the array (and zero it first)
						const tPolarCoordiante* scanArray = static_cast<const tPolarCoordiante*>(sampleDecoder.GetElementAddress(m_ddlLSDataId.scanArray));
						memset(&data->tScanArray, 0, sizeof data->tScanArray);
						memcpy(&data->tScanArray, scanArray, numOfScanPoints * sizeof(tPolarCoordiante));
					});

				case Trajectory:
					PROCESS_INPUT_SAMPLE_HELPER(tTrajectory, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrajectoryIndex.id, &data->id));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrajectoryIndex.ax, &data->ax));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrajectoryIndex.bx, &data->bx));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrajectoryIndex.cx, &data->cx));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrajectoryIndex.dx, &data->dx));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrajectoryIndex.ay, &data->ay));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrajectoryIndex.by, &data->by));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrajectoryIndex.cy, &data->cy));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrajectoryIndex.dy, &data->dy));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrajectoryIndex.start, &data->start));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrajectoryIndex.end, &data->end));
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrajectoryIndex.backwards, &data->backwards));
					});

				case TrajectoryArray:
					PROCESS_INPUT_SAMPLE_HELPER(tTrajectoryArray, {
						RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlTrajectoryArrayIndex.size, &data->size));
						const tTrajectory* trajectories = static_cast<const tTrajectory*>(sampleDecoder.GetElementAddress(m_ddlTrajectoryArrayIndex.trajectories));
						memcpy(&data->trajectories, trajectories, sizeof data->trajectories);
					});
					
				case YoloNetOutput:
					PROCESS_INPUT_SAMPLE_HELPER(tYOLONetOutput, {
						const tFloat32* nodeValues = static_cast<const tFloat32*>(sampleDecoder.GetElementAddress(m_ddlYoloNetOutputIndex.f32NodeValue));
						memcpy(&data->f32NodeValue, nodeValues, sizeof data->f32NodeValue);
					});

				default:
					LOG_ERROR("Unrecognized eZmqStruct %d while processing inputs.", pinType);
				}
			}
		}

		// if the send fails with the ZMQ_DONTWAIT flag set, the queue length limit has been reached
		if (!returncode)
		{
			// did we just switch to this state?
			if (!m_drop_state)
				LOG_WARNING("Queue overflow. Samples will be dropped.");
			m_num_samples_dropped++;
			m_drop_state = true;
			break;
		}
		else if (m_drop_state)
		{
			// we recovered, report number of dropped samples
			LOG_WARNING("Dropped %d samples.", m_num_samples_dropped);
			m_num_samples_dropped = 0;
			m_drop_state = false;
		}
	}

	RETURN_NOERROR;
}

#define PROCESS_OUTPUT_SAMPLE_HELPER(_TYPE_, _CONTENT_) { \
	if (frame->size() != sizeof(_TYPE_)) \
	{ \
		LOG_ERROR("Received %d bytes, but expected %d bytes on pin %s", frame->size(), sizeof(_TYPE_), pinName.c_str()); \
		break; \
	} \
	const auto data = static_cast<_TYPE_*>(frame->data()); \
	{ \
		_CONTENT_ \
	} \
	break; \
}

tResult cZmqBase::ProcessOutput(zmq::message_t* frame, const size_t index)
{
	// ignore invalid outputs
	if (index >= m_outputs.size())
		RETURN_NOERROR;

	ZmqPinDef pinDef = m_outputs.at(index);
	std::string pinName = std::get<0>(pinDef);
	eZmqStruct pinType = std::get<1>(pinDef);
	cPinWriter* pinWriter = m_pinWriters[pinName];
	cSampleCodecFactory* pinSampleFactory = GetSampleFactory(pinType);

	// allocate the sample and build an encoder
	object_ptr<ISample> pWriteSample;
	RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
	cSampleCodec sampleEncoder = pinSampleFactory->MakeCodecFor(pWriteSample);

	switch (pinType)
	{
	case Image:
		LOG_ERROR("eZmqStruct 'Image' is not supported for output pins.");
		break;

	case Jury:
		PROCESS_OUTPUT_SAMPLE_HELPER(tJuryStruct, {
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlJuryStructIndex.i16ActionID, data->i16ActionID));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlJuryStructIndex.i16ManeuverEntry, data->i16ManeuverEntry));
		});

	case Driver:
		LOG_ERROR("eZmqStruct 'Driver' not implemented for output pins.");
		break;

	case SignalValue:
		PROCESS_OUTPUT_SAMPLE_HELPER(tSignalValue, {
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlSignalValueId.timeStamp, data->ui32ArduinoTimestamp));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlSignalValueId.value, data->f32Value));
		});

	case BoolSignalValue:
		PROCESS_OUTPUT_SAMPLE_HELPER(tBoolSignalValue, {
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlBoolSignalValueId.ui32ArduinoTimestamp, data->ui32ArduinoTimestamp));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlBoolSignalValueId.bValue, data->bValue));
		});

	case WheelData:
		PROCESS_OUTPUT_SAMPLE_HELPER(tWheelData, {
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, data->ui32ArduinoTimestamp));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlWheelDataIndex.WheelTach, data->ui32WheelTach));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlWheelDataIndex.WheelDir, data->i8WheelDir));
		});

	case InerMeasUnitData:
		PROCESS_OUTPUT_SAMPLE_HELPER(tInerMeasUnitData, {
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlInerMeasUnitDataIndex.timeStamp, data->ui32ArduinoTimestamp));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlInerMeasUnitDataIndex.A_x, data->f32A_x));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlInerMeasUnitDataIndex.A_y, data->f32A_y));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlInerMeasUnitDataIndex.A_z, data->f32A_z));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlInerMeasUnitDataIndex.G_x, data->f32G_x));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlInerMeasUnitDataIndex.G_y, data->f32G_y));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlInerMeasUnitDataIndex.G_z, data->f32G_z));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlInerMeasUnitDataIndex.M_x, data->f32M_x));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlInerMeasUnitDataIndex.M_y, data->f32M_y));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlInerMeasUnitDataIndex.M_z, data->f32M_z));
		});

	case RoadSignExt:
		LOG_ERROR("eZmqStruct 'RoadSignExt' not implemented for output pins.");
		break;

	case Position:
		PROCESS_OUTPUT_SAMPLE_HELPER(::tPosition, {
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlPositionIndex.f32x, data->f32x));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlPositionIndex.f32y, data->f32y));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlPositionIndex.f32radius, data->f32radius));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlPositionIndex.f32speed, data->f32speed));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlPositionIndex.f32heading, data->f32heading));
		});

	case Obstacle:
		LOG_ERROR("eZmqStruct 'Obstacle' not implemented for output pins.");
		break;
	case TrafficSign:
		LOG_ERROR("eZmqStruct 'TrafficSign' not implemented for output pins.");
		break;
	case ParkingSpace:
		LOG_ERROR("eZmqStruct 'ParkingSpace' not implemented for output pins.");
		break;
	case Ultrasonic:
		LOG_ERROR("eZmqStruct 'Ultrasonic' not implemented for output pins.");
		break;
	case Voltage:
		LOG_ERROR("eZmqStruct 'Voltage' not implemented for output pins.");
		break;
	case PolarCoordinate:
		LOG_ERROR("eZmqStruct 'PolarCoordinate' not implemented for output pins.");
		break;
	case LaserScanner:
		LOG_ERROR("eZmqStruct 'LaserScanner' not implemented for output pins.");
		break;

	case Trajectory:
		PROCESS_OUTPUT_SAMPLE_HELPER(tTrajectory, {
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlTrajectoryIndex.id, data->id));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlTrajectoryIndex.ax, data->ax));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlTrajectoryIndex.bx, data->bx));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlTrajectoryIndex.cx, data->cx));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlTrajectoryIndex.dx, data->dx));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlTrajectoryIndex.ay, data->ay));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlTrajectoryIndex.by, data->by));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlTrajectoryIndex.cy, data->cy));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlTrajectoryIndex.dy, data->dy));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlTrajectoryIndex.start, data->start));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlTrajectoryIndex.end, data->end));
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlTrajectoryIndex.backwards, data->backwards));
		});

	case TrajectoryArray:
		PROCESS_OUTPUT_SAMPLE_HELPER(tTrajectoryArray, {
			RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlTrajectoryArrayIndex.size, data->size));
			tTrajectory* trajectories = static_cast<tTrajectory*>(sampleEncoder.GetElementAddress(m_ddlTrajectoryArrayIndex.trajectories));
			memcpy(trajectories, &data->trajectories, sizeof data->trajectories);
		});

	case YoloNetOutput:
		LOG_ERROR("eZmqStruct 'YoloNetOutput' not implemented for output pins.");

	default:
		LOG_ERROR("Unrecognized eZmqStruct %d while processing outputs", pinType);
	}

	// write the sample
	*pinWriter << pWriteSample << flush << trigger;

	RETURN_NOERROR;
}

#define STREAM_TYPE_DEFINITION_HELPER(_STRUCT_NAME_, _STREAM_TYPE_, _SAMPLE_FACTORY_, _CONTENT_) {\
	if (!_STREAM_TYPE_) \
		if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(_STRUCT_NAME_, _STREAM_TYPE_, _SAMPLE_FACTORY_)) \
		{ \
			_CONTENT_ \
		} \
		else LOG_ERROR("No mediadescription for %s found!", _STRUCT_NAME_); \
	else LOG_DUMP("Skipped initialization for %s, because it is already initialized.", _STRUCT_NAME_); \
	return &_STREAM_TYPE_; \
}

/**
 * \brief Get the IStreamType for a struct type. Create the IStreamType and the SampleFactory if necessary
 * \param sampleType The pin type
 * \return The (created) IStreamType
 */
object_ptr<IStreamType>* cZmqBase::GetStreamType(const eZmqStruct sampleType)
{
	switch (sampleType)
	{
	case Image:
		{
			if (!m_ImageFormatStreamType)
			{
				m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
				m_ImageFormatStreamType = make_object_ptr<cStreamType>(stream_meta_type_image());
				if (!m_ImageFormatStreamType)
					LOG_ERROR("Call make_object_ptr() for the image format yielded a nullptr.");
				else if (IS_FAILED(set_stream_type_image_format(*m_ImageFormatStreamType, m_sImageFormat)))
					LOG_ERROR("Call set_stream_type_image_format() failed during retrieving the image format stream type.");
			}
			else LOG_ERROR("Do not use more than one image input pin !!!");
			return &m_ImageFormatStreamType;
		}

	case Jury:
		STREAM_TYPE_DEFINITION_HELPER("tJuryStruct", m_JuryStructStreamType, m_JuryStructSampleFactory, {
			access_element::find_index(m_JuryStructSampleFactory, cString("i16ActionID"), m_ddlJuryStructIndex.i16ActionID);
			access_element::find_index(m_JuryStructSampleFactory, cString("i16ManeuverEntry"), m_ddlJuryStructIndex.i16ManeuverEntry);
		});

	case Driver:
		STREAM_TYPE_DEFINITION_HELPER("tDriverStruct", m_DriverStructStreamType, m_DriverStructSampleFactory, {
			access_element::find_index(m_DriverStructSampleFactory, cString("i16StateID"), m_ddlDriverStructIndex.i16StateID);
			access_element::find_index(m_DriverStructSampleFactory, cString("i16ManeuverEntry"), m_ddlDriverStructIndex.i16ManeuverEntry);
		});

	case SignalValue:
		STREAM_TYPE_DEFINITION_HELPER("tSignalValue", m_SignalValueStreamType, m_SignalValueSampleFactory, {
			access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp);
			access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value);
		});

	case BoolSignalValue:
		STREAM_TYPE_DEFINITION_HELPER("tBoolSignalValue", m_BoolSignalValueStreamType, m_BoolSignalValueSampleFactory, {
			access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.ui32ArduinoTimestamp);
			access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue"), m_ddlBoolSignalValueId.bValue);
		});

	case WheelData:
		STREAM_TYPE_DEFINITION_HELPER("tWheelData", m_WheelDataStreamType, m_WheelDataSampleFactory, {
			access_element::find_index(m_WheelDataSampleFactory, "ui32ArduinoTimestamp", m_ddlWheelDataIndex.ArduinoTimestamp);
			access_element::find_index(m_WheelDataSampleFactory, "ui32WheelTach", m_ddlWheelDataIndex.WheelTach);
			access_element::find_index(m_WheelDataSampleFactory, "i8WheelDir", m_ddlWheelDataIndex.WheelDir);
		});

	case InerMeasUnitData:
		STREAM_TYPE_DEFINITION_HELPER("tInerMeasUnitData", m_IMUDataStreamType, m_IMUDataSampleFactory, {
			access_element::find_index(m_IMUDataSampleFactory, "ui32ArduinoTimestamp", m_ddlInerMeasUnitDataIndex.timeStamp);
			access_element::find_index(m_IMUDataSampleFactory, "f32A_x", m_ddlInerMeasUnitDataIndex.A_x);
			access_element::find_index(m_IMUDataSampleFactory, "f32A_y", m_ddlInerMeasUnitDataIndex.A_y);
			access_element::find_index(m_IMUDataSampleFactory, "f32A_z", m_ddlInerMeasUnitDataIndex.A_z);
			access_element::find_index(m_IMUDataSampleFactory, "f32G_x", m_ddlInerMeasUnitDataIndex.G_x);
			access_element::find_index(m_IMUDataSampleFactory, "f32G_y", m_ddlInerMeasUnitDataIndex.G_y);
			access_element::find_index(m_IMUDataSampleFactory, "f32G_z", m_ddlInerMeasUnitDataIndex.G_z);
			access_element::find_index(m_IMUDataSampleFactory, "f32M_x", m_ddlInerMeasUnitDataIndex.M_x);
			access_element::find_index(m_IMUDataSampleFactory, "f32M_y", m_ddlInerMeasUnitDataIndex.M_y);
			access_element::find_index(m_IMUDataSampleFactory, "f32M_z", m_ddlInerMeasUnitDataIndex.M_z);
		});

	case RoadSignExt:
		STREAM_TYPE_DEFINITION_HELPER("tRoadSignExt", m_RoadSignExtStreamType, m_RoadSignExtSampleFactory, {
			access_element::find_index(m_RoadSignExtSampleFactory, cString("i16Identifier"), m_ddlRoadSignExtIndex.id);
			access_element::find_index(m_RoadSignExtSampleFactory, cString("f32Imagesize"), m_ddlRoadSignExtIndex.size);
			access_element::find_array_index(m_RoadSignExtSampleFactory, cString("af32TVec"), m_ddlRoadSignExtIndex.tvec);
			access_element::find_array_index(m_RoadSignExtSampleFactory, cString("af32RVec"), m_ddlRoadSignExtIndex.rvec);
		});

	case Position:
		STREAM_TYPE_DEFINITION_HELPER("tPosition", m_PositionStreamType, m_PositionSampleFactory, {
			access_element::find_index(m_PositionSampleFactory, cString("f32x"), m_ddlPositionIndex.f32x);
			access_element::find_index(m_PositionSampleFactory, cString("f32y"), m_ddlPositionIndex.f32y);
			access_element::find_index(m_PositionSampleFactory, cString("f32radius"), m_ddlPositionIndex.f32radius);
			access_element::find_index(m_PositionSampleFactory, cString("f32speed"), m_ddlPositionIndex.f32speed);
			access_element::find_index(m_PositionSampleFactory, cString("f32heading"), m_ddlPositionIndex.f32heading);
		});

	case Obstacle:
		STREAM_TYPE_DEFINITION_HELPER("tObstacle", m_ObstacleStreamType, m_ObstacleSampleFactory, {
			access_element::find_index(m_ObstacleSampleFactory, cString("f32x"), m_ddlObstacleIndex.f32y);
			access_element::find_index(m_ObstacleSampleFactory, cString("f32y"), m_ddlObstacleIndex.f32y);
		});

	case TrafficSign:
		STREAM_TYPE_DEFINITION_HELPER("tTrafficSign", m_TrafficSignStreamType, m_TrafficSignSampleFactory, {
			access_element::find_index(m_TrafficSignSampleFactory, cString("i16Identifier"), m_ddlTrafficSignIndex.i16Identifier);
			access_element::find_index(m_TrafficSignSampleFactory, cString("f32x"), m_ddlTrafficSignIndex.f32x);
			access_element::find_index(m_TrafficSignSampleFactory, cString("f32y"), m_ddlTrafficSignIndex.f32y);
			access_element::find_index(m_TrafficSignSampleFactory, cString("f32angle"), m_ddlTrafficSignIndex.f32angle);
		});

	case ParkingSpace:
		STREAM_TYPE_DEFINITION_HELPER("tParkingSpace", m_ParkingSpaceStreamType, m_ParkingSpaceSampleFactory, {
			access_element::find_index(m_ParkingSpaceSampleFactory, cString("i16Identifier"), m_ddlParkingSpaceIndex.i16Identifier);
			access_element::find_index(m_ParkingSpaceSampleFactory, cString("f32x"), m_ddlParkingSpaceIndex.f32x);
			access_element::find_index(m_ParkingSpaceSampleFactory, cString("f32y"), m_ddlParkingSpaceIndex.f32y);
			access_element::find_index(m_ParkingSpaceSampleFactory, cString("ui16Status"), m_ddlParkingSpaceIndex.ui16Status);
		});

	case Ultrasonic:
		STREAM_TYPE_DEFINITION_HELPER("tUltrasonicStruct", m_USDataStreamType, m_USDataSampleFactory, {
			access_element::find_index(m_USDataSampleFactory, cString("tSideLeft") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.SideLeft.timeStamp);
			access_element::find_index(m_USDataSampleFactory, cString("tSideLeft") + cString(".f32Value"), m_ddlUltrasonicStructIndex.SideLeft.value);
			access_element::find_index(m_USDataSampleFactory, cString("tSideRight") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.SideRight.timeStamp);
			access_element::find_index(m_USDataSampleFactory, cString("tSideRight") + cString(".f32Value"), m_ddlUltrasonicStructIndex.SideRight.value);
			access_element::find_index(m_USDataSampleFactory, cString("tRearLeft") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.RearLeft.timeStamp);
			access_element::find_index(m_USDataSampleFactory, cString("tRearLeft") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearLeft.value);
			access_element::find_index(m_USDataSampleFactory, cString("tRearCenter") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.RearCenter.timeStamp);
			access_element::find_index(m_USDataSampleFactory, cString("tRearCenter") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearCenter.value);
			access_element::find_index(m_USDataSampleFactory, cString("tRearRight") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.RearRight.timeStamp);
			access_element::find_index(m_USDataSampleFactory, cString("tRearRight") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearRight.value);
		});

	case Voltage:
		STREAM_TYPE_DEFINITION_HELPER("tVoltageStruct", m_VoltageStructStreamType, m_VoltageStructSampleFactory, {
			access_element::find_index(m_VoltageStructSampleFactory, cString("tActuatorVoltage") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.ActuatorVoltage.timeStamp);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tActuatorCell1") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.ActuatorCell1.timeStamp);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tActuatorCell2") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.ActuatorCell2.timeStamp);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorVoltage") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.SensorVoltage.timeStamp);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell1") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.SensorCell1.timeStamp);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell2") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.SensorCell2.timeStamp);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell3") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.SensorCell3.timeStamp);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell4") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.SensorCell4.timeStamp);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell5") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.SensorCell5.timeStamp);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell6") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.SensorCell6.timeStamp);

			access_element::find_index(m_VoltageStructSampleFactory, cString("tActuatorVoltage") + cString(".f32Value"), m_ddlVoltageStructIndex.ActuatorVoltage.value);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tActuatorCell1") + cString(".f32Value"), m_ddlVoltageStructIndex.ActuatorCell1.value);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tActuatorCell2") + cString(".f32Value"), m_ddlVoltageStructIndex.ActuatorCell2.value);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorVoltage") + cString(".f32Value"), m_ddlVoltageStructIndex.SensorVoltage.value);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell1") + cString(".f32Value"), m_ddlVoltageStructIndex.SensorCell1.value);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell2") + cString(".f32Value"), m_ddlVoltageStructIndex.SensorCell2.value);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell3") + cString(".f32Value"), m_ddlVoltageStructIndex.SensorCell3.value);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell4") + cString(".f32Value"), m_ddlVoltageStructIndex.SensorCell4.value);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell5") + cString(".f32Value"), m_ddlVoltageStructIndex.SensorCell5.value);
			access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell6") + cString(".f32Value"), m_ddlVoltageStructIndex.SensorCell6.value);
		});

	case PolarCoordinate:
		STREAM_TYPE_DEFINITION_HELPER("tPolarCoordiante", m_PolarCoordianteStreamType, m_PolarCoordianteSampleFactory, {
			access_element::find_index(m_PolarCoordianteSampleFactory, cString("f32Radius"), m_ddlPolarCoordianteIndex.f32Radius);
			access_element::find_index(m_PolarCoordianteSampleFactory, cString("f32Angle"), m_ddlPolarCoordianteIndex.f32Angle);
		});

	case LaserScanner:
		STREAM_TYPE_DEFINITION_HELPER("tLaserScannerData", m_LSStructStreamType, m_LSStructSampleFactory, {
			access_element::find_index(m_LSStructSampleFactory, "ui32Size", m_ddlLSDataId.size);
			access_element::find_array_index(m_LSStructSampleFactory, "tScanArray", m_ddlLSDataId.scanArray);
		});

	case Trajectory:
		STREAM_TYPE_DEFINITION_HELPER("tTrajectory", m_TrajectoryStreamType, m_TrajectorySampleFactory, {
			access_element::find_index(m_TrajectorySampleFactory, cString("id"), m_ddlTrajectoryIndex.id);
			access_element::find_index(m_TrajectorySampleFactory, cString("ax"), m_ddlTrajectoryIndex.ax);
			access_element::find_index(m_TrajectorySampleFactory, cString("bx"), m_ddlTrajectoryIndex.bx);
			access_element::find_index(m_TrajectorySampleFactory, cString("cx"), m_ddlTrajectoryIndex.cx);
			access_element::find_index(m_TrajectorySampleFactory, cString("dx"), m_ddlTrajectoryIndex.dx);
			access_element::find_index(m_TrajectorySampleFactory, cString("ay"), m_ddlTrajectoryIndex.ay);
			access_element::find_index(m_TrajectorySampleFactory, cString("by"), m_ddlTrajectoryIndex.by);
			access_element::find_index(m_TrajectorySampleFactory, cString("cy"), m_ddlTrajectoryIndex.cy);
			access_element::find_index(m_TrajectorySampleFactory, cString("dy"), m_ddlTrajectoryIndex.dy);
			access_element::find_index(m_TrajectorySampleFactory, cString("start"), m_ddlTrajectoryIndex.start);
			access_element::find_index(m_TrajectorySampleFactory, cString("end"), m_ddlTrajectoryIndex.end);
			access_element::find_index(m_TrajectorySampleFactory, cString("backwards"), m_ddlTrajectoryIndex.backwards);
		});

	case TrajectoryArray:
		STREAM_TYPE_DEFINITION_HELPER("tTrajectoryArray", m_TrajectoryArrayStreamType, m_TrajectoryArraySampleFactory, {
			access_element::find_index(m_TrajectoryArraySampleFactory, "size", m_ddlTrajectoryArrayIndex.size);
			access_element::find_array_index(m_TrajectoryArraySampleFactory, "trajectories", m_ddlTrajectoryArrayIndex.trajectories);
		});

	case YoloNetOutput:
		STREAM_TYPE_DEFINITION_HELPER("tYOLONetOutput", m_YoloNetOutputStreamType, m_YoloNetOutputSampleFactory, {
			access_element::find_array_index(m_YoloNetOutputSampleFactory, "f32NodeValue", m_ddlYoloNetOutputIndex.f32NodeValue);
		});

	default:
		LOG_ERROR("Could not get or create stream type for unrecognized eZmqStruct %d", sampleType);
	}

	return nullptr;
}

size_t cZmqBase::GetStructSize(const eZmqStruct sampleType) const
{
	switch (sampleType)
	{
	case Image:					return 3 * m_sImageFormat.m_ui32Width * m_sImageFormat.m_ui32Height;
	case Jury:					return sizeof(tJuryStruct);
	case Driver:				return sizeof(tDriverStruct);
	case SignalValue:			return sizeof(tSignalValue);
	case BoolSignalValue:		return sizeof(tBoolSignalValue);
	case WheelData:				return sizeof(tWheelData);
	case InerMeasUnitData:		return sizeof(tInerMeasUnitData);
	case RoadSignExt:			return sizeof(tRoadSignExt);
	case Position:				return sizeof(::tPosition);
	case Obstacle:				return sizeof(tObstacle);
	case TrafficSign:			return sizeof(tTrafficSign);
	case ParkingSpace:			return sizeof(tParkingSpace);
	case Ultrasonic:			return sizeof(tUltrasonicStruct);
	case Voltage:				return sizeof(tVoltageStruct);
	case PolarCoordinate:		return sizeof(tPolarCoordiante);
	case LaserScanner:			return sizeof(tLaserScannerData);
	case Trajectory:			return sizeof(tTrajectory);
	case TrajectoryArray:		return sizeof(tTrajectoryArray);
	case YoloNetOutput:			return sizeof(tYOLONetOutput);
	default:					LOG_ERROR("Could not request size for unrecognized eZmqStruct %d", sampleType);
	}

	return 0;
}

cSampleCodecFactory* cZmqBase::GetSampleFactory(const eZmqStruct sampleType)
{
	switch (sampleType)
	{
	case Image:					return nullptr;
	case Jury:					return &m_JuryStructSampleFactory;
	case Driver:				return &m_DriverStructSampleFactory;
	case SignalValue:			return &m_SignalValueSampleFactory;
	case BoolSignalValue:		return &m_BoolSignalValueSampleFactory;
	case WheelData:				return &m_WheelDataSampleFactory;
	case InerMeasUnitData:		return &m_IMUDataSampleFactory;
	case RoadSignExt:			return &m_RoadSignExtSampleFactory;
	case Position:				return &m_PositionSampleFactory;
	case Obstacle:				return &m_ObstacleSampleFactory;
	case TrafficSign:			return &m_TrafficSignSampleFactory;
	case ParkingSpace:			return &m_ParkingSpaceSampleFactory;
	case Ultrasonic:			return &m_USDataSampleFactory;
	case Voltage:				return &m_VoltageStructSampleFactory;
	case PolarCoordinate:		return &m_PolarCoordianteSampleFactory;
	case LaserScanner:			return &m_LSStructSampleFactory;
	case Trajectory:			return &m_TrajectorySampleFactory;
	case TrajectoryArray:		return &m_TrajectoryArraySampleFactory;
	case YoloNetOutput:			return &m_YoloNetOutputSampleFactory;
	default:					LOG_ERROR("Could not get sample factory for unrecognized eZmqStruct %d", sampleType);
	}

	return nullptr;
}

/**
* \brief Resolve the inproc:// socket address for the PAIR socket between the
* parent thread and the ZeroMQ I/O thread. The address will be inferred from the filter name,
* such that we have a unique address per filter instantiation.
* \return The "inproc://FILTER_NAME" address
*/
std::string cZmqBase::GetPairSocketAddress() const
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

inline bool cZmqBase::SendString(zmq::socket_t& socket, const std::string& string)
{
	zmq::message_t message(string.size());
	memcpy(message.data(), string.data(), string.size());

	const bool result = socket.send(message);
	return result;
}

inline std::string cZmqBase::ReceiveString(zmq::socket_t& socket)
{
	zmq::message_t message;
	socket.recv(&message);

	return std::string(static_cast<char*>(message.data()), message.size());
}

void zmq_free_message(void *data, void *hint)
{
	free(data);
}
