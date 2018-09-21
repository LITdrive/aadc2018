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
#include "../../services/zeromq/zeromq_service_intf.h"

#include <aadc_structs.h>

#include <sstream>
#include <algorithm>
#include <boost/thread/thread.hpp>

// forward declaration
void zmq_free_message(void *data, void *hint);

cZmqBase::cZmqBase()
{
	RegisterPropertyVariable("ZeroMQ Socket Address", m_server_socket_address);
	RegisterPropertyVariable("ZeroMQ Queue Length", m_queue_length);
	RegisterPropertyVariable("ZeroMQ Subsample Factor", m_subsample_factor);

	// we need some null bytes for empty messages, fill them!
	memset(m_nullbytes, 0, sizeof m_nullbytes / sizeof *m_nullbytes);

	// register the runner which processes zeromq messages in the background
	InitializeZeroMQThread();
}

tResult cZmqBase::Init(const tInitStage eStage)
{
	RETURN_IF_FAILED(cFilter::Init(eStage));
	if (eStage == StageFirst)
	{
		RETURN_IF_FAILED(Configure());
	}
	RETURN_NOERROR;
}

cZmqBase::~cZmqBase()
{
	// signal the thread to stop
	m_runner_stop = true;

	delete m_sck_pair;

	// deallocate pin readers
	for (const auto& pair : m_pinReaders) {
		delete pair.second;
	}

	// deallocate pin writers
	for (const auto& pair : m_pinWriters) {
		delete pair.second;
	}
}

tResult cZmqBase::Configure()
{
	InitializeInputPins();
	InitializeOutputPins();

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

	// wake up the zeromq thread
	std::unique_lock<std::mutex> lk(m_runner_mutex);
	m_runner_ready = true;
	m_runner_cv.notify_one();

	RETURN_NOERROR;
}

void cZmqBase::InitializePins(std::vector<ZmqPinDef> &pin_definitions, const bool isReader)
{
	for (auto& pin_def : pin_definitions)
	{
		std::string pinName = std::get<0>(pin_def);
		eZmqStruct pinType = std::get<1>(pin_def);

		// resolve or create the stream type
		object_ptr<IStreamType>* streamType = GetStreamType(pinType);

		if (isReader)
		{
			// input pin
			const auto pinReader = new cPinReader();
			m_pinReaders[pinName] = pinReader;
			create_pin(*this, *pinReader, pinName.c_str(), *streamType);
		}
		else
		{
			// output pin
			const auto pinWriter = new cPinWriter();
			m_pinWriters[pinName] = pinWriter;
			filter_create_pin(*this, *pinWriter, pinName.c_str(), *streamType);
		}
	}
}

/**
 * \brief Register cPinWriters with their respective names
 */
void cZmqBase::InitializeOutputPins()
{
	InitializePins(m_outputs, false);
}

/**
 * \brief Register cPinReaders with their respective names. trigger lambdas on the trigger pins
 */
void cZmqBase::InitializeInputPins()
{
	InitializePins(m_inputs, true);

	// do we also need to create an inner trigger pipe?
	for (auto& trigger_pin : m_triggers)
	{
		const char* trigger_pin_cstr = trigger_pin.c_str();

		// search if we initialized a pin reader for this pin
		auto it = m_pinReaders.find(trigger_pin);
		if (it != m_pinReaders.end())
		{
			// execute this lambda if the pin is triggered
			create_inner_pipe(*this, cString::Format("%s_trigger", trigger_pin_cstr), trigger_pin_cstr, [&](tTimeStamp tmTime) -> tResult
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
			});
		}
		else
		{
			LOG_ERROR("Could not create trigger pin on nonexistent pin %s", trigger_pin_cstr);
		}
	}
}

/**
 * \brief Define and register the ZeroMQ I/O thread, which will actually send the samples and poll for replies
 */
void cZmqBase::InitializeZeroMQThread()
{
	// thread for zeromq, where requests are sent and replies are polled
	const object_ptr<IRunner> zmq_thread = ::adtf::ucom::make_object_ptr<cRunner>("zmq_thread", [&](tTimeStamp /* tmTime */) -> tResult
	{
		/* --- wait for parent --- */

		LOG_DUMP("ZMQ I/O thread waits for mutex ...");

		// the runner needs to wait until the parent is done with binding the socket
		std::unique_lock<std::mutex> lk(m_runner_mutex);
		while (!m_runner_ready)
		{
			m_runner_cv.wait(lk);
			if (!m_runner_ready)
			{
				LOG_WARNING("Spurious wake-up in the ZMQ I/O thread detected.");
			}
		}

		/* --- connect sockets --- */

		// connect a PAIR socket to the parent thread
		auto pair_socket = zmq::socket_t(*m_pZeroMQService->GetContext(), ZMQ_PAIR);

		// do not wait at close time
		int linger = 0;
		pair_socket.setsockopt(ZMQ_LINGER, &linger, sizeof linger);

		// limit queue length
		int hwm = m_queue_length / 2;
		pair_socket.setsockopt(ZMQ_RCVHWM, &hwm, sizeof hwm);

		pair_socket.connect(GetPairSocketAddress());

		// connect a REQ socket to the server
		zmq::socket_t* client_socket = InitializeClientSocket();

		LOG_DUMP("ZMQ I/O thread and client sockets initialized.");

		const size_t num_outputs = m_outputs.size();
		bool lastConnectionState = true;

		// TODO: explicitly stop this loop in the destructor or use zmq heartbeating
		while (!m_runner_stop && pair_socket.connected())
		{
#ifdef _DEBUG
			// latency measurement in debug mode
			tTimeStamp start = cHighResTimer::GetTime();
#endif

			/* --- receive and forward data --- */

			int pair_flags;
			do
			{
				// receive from parent thread
				zmq::message_t message;
				pair_socket.recv(&message);
				pair_flags = message.more() ? ZMQ_SNDMORE : 0;

				// send to server
				client_socket->send(message, pair_flags);
			}
			while (pair_flags == ZMQ_SNDMORE);

			// simulate load
			//boost::this_thread::sleep(boost::posix_time::seconds(2));
			
			/* --- wait for server reply (with timeout) --- */

			zmq::pollitem_t items[] = { { *client_socket, 0, ZMQ_POLLIN, 0 } };
			zmq::poll(&items[0], 1, ZMQ_REQUEST_TIMEOUT);

			if (items[0].revents & ZMQ_POLLIN)
			{
				int server_flags;
				size_t outputIndex = 0;
				do
				{
					// receive from server
					zmq::message_t message;
					client_socket->recv(&message);
					server_flags = message.more() ? ZMQ_SNDMORE : 0;

					// process message
					ProcessOutput(&message, outputIndex);
					outputIndex++;
				}
				while (server_flags == ZMQ_SNDMORE);

				// sanity checks
				if (outputIndex < num_outputs)
					LOG_ERROR("Expected %d output pin structs, but we only received %d messages. Missing pins will not flush any samples.", num_outputs, outputIndex);
				else if (outputIndex > num_outputs)
					LOG_ERROR("Expected %d output pin structs, but we received %d messages. Additional messages will be discarded.", num_outputs, outputIndex);

				if (!lastConnectionState)
				{
					LOG_INFO("Successfully reconnected after timeout.");
					lastConnectionState = true;
				}

#ifdef _DEBUG
				const tTimeStamp end = cHighResTimer::GetTime();
				LOG_DUMP("Sample latency: %.2f ms", (end - start) / 1000.0);
#endif
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
		LOG_DUMP("Gracefully closed ZeroMQ I/O thread.");

		RETURN_NOERROR;
	});

	cRuntimeBehaviour::RegisterRunner(zmq_thread);
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
			// send an empty struct (just null bytes) if the samples is invalid
			zmq::message_t message(m_nullbytes, GetStructSize(pinType), nullptr, nullptr);
			returncode = m_sck_pair->send(message, flags);
		}
		else
		{
			// properly decode the sample
			cSampleDecoder sampleDecoder = pinSampleFactory->MakeDecoderFor(*pSample);
			RETURN_IF_FAILED(sampleDecoder.IsValid());

			switch (pinType)
			{
			case Jury:
				LOG_ERROR("eZmqStruct 'Jury' not implemented.");
				break;

			case Driver:
				LOG_ERROR("eZmqStruct 'Driver' not implemented.");
				break;

			case SignalValue:
				{
					auto* signalValue = new tSignalValue();
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &signalValue->ui32ArduinoTimestamp));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlSignalValueId.value, &signalValue->f32Value));

					zmq::message_t message(signalValue, sizeof(tSignalValue), zmq_free_message, nullptr);
					returncode = m_sck_pair->send(message, flags);
				}
				break;

			case BoolSignalValue:
				{
					auto* boolSignalValue = new tBoolSignalValue();
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlBoolSignalValueId.ui32ArduinoTimestamp, &boolSignalValue->ui32ArduinoTimestamp));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlBoolSignalValueId.bValue, &boolSignalValue->bValue));

					zmq::message_t message(boolSignalValue, sizeof(tBoolSignalValue), zmq_free_message, nullptr);
					returncode = m_sck_pair->send(message, flags);
				}
				break;

			case WheelData:
				{
					auto* wheelData = new tWheelData();
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, &wheelData->ui32ArduinoTimestamp));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlWheelDataIndex.WheelTach, &wheelData->ui32WheelTach));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlWheelDataIndex.WheelDir, &wheelData->i8WheelDir));

					zmq::message_t message(wheelData, sizeof(tWheelData), zmq_free_message, nullptr);
					returncode = m_sck_pair->send(message, flags);
				}
				break;

			case InerMeasUnitData:
				{
					auto* IMU_data = new tInerMeasUnitData();
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.timeStamp, &IMU_data->ui32ArduinoTimestamp));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.A_x, &IMU_data->f32A_x));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.A_y, &IMU_data->f32A_y));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.A_z, &IMU_data->f32A_z));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.G_x, &IMU_data->f32G_x));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.G_y, &IMU_data->f32G_y));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.G_z, &IMU_data->f32G_z));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.M_x, &IMU_data->f32M_x));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.M_y, &IMU_data->f32M_y));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlInerMeasUnitDataIndex.M_z, &IMU_data->f32M_z));

					zmq::message_t message(IMU_data, sizeof(tInerMeasUnitData), zmq_free_message, nullptr);
					returncode = m_sck_pair->send(message, flags);
				}
				break;

			case RoadSignExt:
				LOG_ERROR("eZmqStruct 'RoadSignExt' not implemented.");
				break;

			case Position:
				LOG_ERROR("eZmqStruct 'Position' not implemented.");
				break;

			case Obstacle:
				LOG_ERROR("eZmqStruct 'Obstacle' not implemented.");
				break;

			case TrafficSign:
				LOG_ERROR("eZmqStruct 'TrafficSign' not implemented.");
				break;

			case ParkingSpace:
				LOG_ERROR("eZmqStruct 'ParkingSpace' not implemented.");
				break;

			case Ultrasonic:
				{
					// TODO: remove timestamps
					auto* US_data = new tUltrasonicStruct();
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.SideLeft.value, &US_data->tSideLeft.f32Value));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.SideLeft.timeStamp, &US_data->tSideLeft.ui32ArduinoTimestamp));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.SideRight.value, &US_data->tSideRight.f32Value));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.SideRight.timeStamp, &US_data->tSideRight.ui32ArduinoTimestamp));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.RearLeft.value, &US_data->tRearLeft.f32Value));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.RearLeft.timeStamp, &US_data->tRearLeft.ui32ArduinoTimestamp));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.RearCenter.value, &US_data->tRearCenter.f32Value));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.RearCenter.timeStamp, &US_data->tRearCenter.ui32ArduinoTimestamp));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.RearRight.value, &US_data->tRearRight.f32Value));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlUltrasonicStructIndex.RearRight.timeStamp, &US_data->tRearRight.ui32ArduinoTimestamp));

					zmq::message_t message(US_data, sizeof(tUltrasonicStruct), zmq_free_message, nullptr);
					returncode = m_sck_pair->send(message, flags);
				}
				break;

			case Voltage:
				{
					auto* voltData = new tVoltageStruct();
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.ActuatorVoltage.value, &voltData->tActuatorVoltage.f32Value));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.ActuatorCell1.value, &voltData->tActuatorCell1.f32Value));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.ActuatorCell2.value, &voltData->tActuatorCell2.f32Value));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.SensorVoltage.value, &voltData->tSensorVoltage.f32Value));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.SensorCell1.value, &voltData->tSensorCell1.f32Value));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.SensorCell2.value, &voltData->tSensorCell2.f32Value));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.SensorCell3.value, &voltData->tSensorCell3.f32Value));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.SensorCell4.value, &voltData->tSensorCell4.f32Value));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.SensorCell5.value, &voltData->tSensorCell5.f32Value));
					RETURN_IF_FAILED(sampleDecoder.GetElementValue(m_ddlVoltageStructIndex.SensorCell6.value, &voltData->tSensorCell6.f32Value));

					zmq::message_t message(voltData, sizeof(tVoltageStruct), zmq_free_message, nullptr);
					returncode = m_sck_pair->send(message, flags);
				}
				break;

			case PolarCoordinate:
				LOG_ERROR("eZmqStruct 'PolarCoordinate' not implemented.");
				break;

			case LaserScanner:
				{
					// TODO: use zeromq zero-copy here!

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

					returncode = m_sck_pair->send(&scan[0], numOfScanPoints * sizeof(tPolarCoordiante));
				}
				break;

			default:
				LOG_ERROR("Unrecognized eZmqStruct %d while processing inputs.", pinType);
			}
		}

		// if the send fails with the ZMQ_DONTWAIT flag set, the queue length limit has been reached
		if (!returncode)
		{
			// did we just switch to this state?
			if (!m_drop_state)
				LOG_WARNING("Senders queue overflow. We will drop samples from now on!");
			m_num_samples_dropped++;
			m_drop_state = true;
			break;
		}
		else if (m_drop_state)
		{
			// we recovered, report number of dropped samples
			LOG_WARNING("Dropped %d samples because too many samples are in the senders queue.", m_num_samples_dropped);
			m_num_samples_dropped = 0;
			m_drop_state = false;
		}
	}

	RETURN_NOERROR;
}

tResult cZmqBase::ProcessOutput(zmq::message_t* frame, const size_t index)
{
	if (index >= m_outputs.size())
	{
		// ignore invalid outputs
		RETURN_NOERROR;
	}

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
	case Jury:
		LOG_ERROR("eZmqStruct 'Jury' not implemented.");
		break;
	case Driver:
		LOG_ERROR("eZmqStruct 'Driver' not implemented.");
		break;
	case SignalValue:
		{
			if (frame->size() != sizeof(tSignalValue))
			{
				LOG_ERROR("Received %d bytes, but expected %d bytes for tSignalValue struct on pin %s", frame->size(), sizeof(tSignalValue), pinName.c_str());
			}
			else
			{
				const auto signalValue = static_cast<tSignalValue*>(frame->data());
				RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlSignalValueId.timeStamp, signalValue->ui32ArduinoTimestamp));
				RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlSignalValueId.value, signalValue->f32Value));
			}
		}
		break;

	case BoolSignalValue:
		{
			if (frame->size() != sizeof(tBoolSignalValue))
			{
				LOG_ERROR("Received %d bytes, but expected %d bytes for tBoolSignalValue struct on pin %s", frame->size(), sizeof(tBoolSignalValue), pinName.c_str());
			}
			else
			{
				const auto boolSignalValue = static_cast<tBoolSignalValue*>(frame->data());
				RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlBoolSignalValueId.ui32ArduinoTimestamp, boolSignalValue->ui32ArduinoTimestamp));
				RETURN_IF_FAILED(sampleEncoder.SetElementValue(m_ddlBoolSignalValueId.bValue, boolSignalValue->bValue));
			}
		}
		break;

	case WheelData:
		break;

	case InerMeasUnitData:
		break;

	case RoadSignExt:
		LOG_ERROR("eZmqStruct 'RoadSignExt' not implemented.");
		break;
	case Position:
		LOG_ERROR("eZmqStruct 'Position' not implemented.");
		break;
	case Obstacle:
		LOG_ERROR("eZmqStruct 'Obstacle' not implemented.");
		break;
	case TrafficSign:
		LOG_ERROR("eZmqStruct 'TrafficSign' not implemented.");
		break;
	case ParkingSpace:
		LOG_ERROR("eZmqStruct 'ParkingSpace' not implemented.");
		break;
	case Ultrasonic:
		LOG_ERROR("eZmqStruct 'Ultrasonic' not implemented.");
		break;
	case Voltage:
		LOG_ERROR("eZmqStruct 'Voltage' not implemented.");
		break;
	case PolarCoordinate:
		LOG_ERROR("eZmqStruct 'PolarCoordinate' not implemented.");
		break;
	case LaserScanner:
		LOG_ERROR("eZmqStruct 'LaserScanner' not implemented.");
		break;
	default:
		LOG_ERROR("Unrecognized eZmqStruct %d while processing outputs", pinType);
	}

	// write the sample
	*pinWriter << pWriteSample << flush << trigger;

	RETURN_NOERROR;
}

size_t cZmqBase::GetStructSize(const eZmqStruct sampleType) const
{
	switch (sampleType)
	{
	case Jury:
		return sizeof(tJuryStruct);
	case Driver:
		return sizeof(tDriverStruct);
	case SignalValue:
		return sizeof(tSignalValue);
	case BoolSignalValue:
		return sizeof(tBoolSignalValue);
	case WheelData:
		return sizeof(tWheelData);
	case InerMeasUnitData:
		return sizeof(tInerMeasUnitData);
	case RoadSignExt:
		return sizeof(tRoadSignExt);
	case Position:
		// use global namespace because tPosition is also defined by fucking ADTF somewhere
		return sizeof(::tPosition);
	case Obstacle:
		return sizeof(tObstacle);
	case TrafficSign:
		return sizeof(tTrafficSign);
	case ParkingSpace:
		return sizeof(tParkingSpace);
	case Ultrasonic:
		return sizeof(tUltrasonicStruct);
	case Voltage:
		return sizeof(tVoltageStruct);
	case PolarCoordinate:
		return sizeof(tPolarCoordiante);
	case LaserScanner:
		return sizeof(tLaserScannerData);
	default:
		LOG_ERROR("Could not request size for unrecognized eZmqStruct %d", sampleType);
	}

	return 0;
}

/**
 * \brief Get the cSampleCodecFactory for a struct type.
 * \param sampleType The pin type
 * \return The cSampleCodecFactory
 */
cSampleCodecFactory* cZmqBase::GetSampleFactory(const eZmqStruct sampleType)
{
	switch (sampleType)
	{
	case Jury:
		LOG_ERROR("eZmqStruct 'Jury' not implemented.");
		break;
	case Driver:
		LOG_ERROR("eZmqStruct 'Driver' not implemented.");
		break;
	case SignalValue:
		return &m_SignalValueSampleFactory;
	case BoolSignalValue:
		return &m_BoolSignalValueSampleFactory;
	case WheelData:
		return &m_WheelDataSampleFactory;
	case InerMeasUnitData:
		return &m_IMUDataSampleFactory;
	case RoadSignExt:
		LOG_ERROR("eZmqStruct 'RoadSignExt' not implemented.");
		break;
	case Position:
		LOG_ERROR("eZmqStruct 'Position' not implemented.");
		break;
	case Obstacle:
		LOG_ERROR("eZmqStruct 'Obstacle' not implemented.");
		break;
	case TrafficSign:
		LOG_ERROR("eZmqStruct 'TrafficSign' not implemented.");
		break;
	case ParkingSpace:
		LOG_ERROR("eZmqStruct 'ParkingSpace' not implemented.");
		break;
	case Ultrasonic:
		return &m_USDataSampleFactory;
	case Voltage:
		return &m_VoltageStructSampleFactory;
	case PolarCoordinate:
		LOG_ERROR("eZmqStruct 'PolarCoordinate' not implemented.");
		break;
	case LaserScanner:
		return &m_LSStructSampleFactory;
	default:
		LOG_ERROR("Could not get sample factory for unrecognized eZmqStruct %d", sampleType);
	}

	return nullptr;
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
	case Jury:
		LOG_ERROR("eZmqStruct 'Jury' not implemented.");
		break;

	case Driver:
		LOG_ERROR("eZmqStruct 'Driver' not implemented.");
		break;

	case SignalValue:
		if (!m_SignalValueStreamType)
		{
			if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", m_SignalValueStreamType, m_SignalValueSampleFactory))
			{
				access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp);
				access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value);
			}
			else
			{
				LOG_ERROR("No mediadescription for tSignalValue found!");
			}
		}
		else
		{
			LOG_DUMP("Skipped initialization for tSignalValue, because it is already initialized.");
		}

		return &m_SignalValueStreamType;

	case BoolSignalValue:
		if (!m_BoolSignalValueStreamType)
		{
			if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", m_BoolSignalValueStreamType, m_BoolSignalValueSampleFactory))
			{
				access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.ui32ArduinoTimestamp);
				access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue"), m_ddlBoolSignalValueId.bValue);
			}
			else
			{
				LOG_ERROR("No mediadescription for tBoolSignalValue found!");
			}
		}
		else
		{
			LOG_DUMP("Skipped initialization for tBoolSignalValue, because it is already initialized.");
		}

		return &m_BoolSignalValueStreamType;

	case WheelData:
		if (!m_WheelDataStreamType)
		{
			if IS_OK(create_adtf_default_stream_type_from_service("tWheelData", m_WheelDataStreamType, m_WheelDataSampleFactory))
			{
				access_element::find_index(m_WheelDataSampleFactory, "ui32ArduinoTimestamp", m_ddlWheelDataIndex.ArduinoTimestamp);
				access_element::find_index(m_WheelDataSampleFactory, "ui32WheelTach", m_ddlWheelDataIndex.WheelTach);
				access_element::find_index(m_WheelDataSampleFactory, "i8WheelDir", m_ddlWheelDataIndex.WheelDir);
			}
			else
			{
				LOG_ERROR("No mediadescription for tWheelData found!");
			}
		}
		else
		{
			LOG_DUMP("Skipped initialization for tWheelData, because it is already initialized.");
		}

		return &m_WheelDataStreamType;

	case InerMeasUnitData:
		if (!m_IMUDataStreamType)
		{
			if IS_OK(create_adtf_default_stream_type_from_service("tInerMeasUnitData", m_IMUDataStreamType, m_IMUDataSampleFactory))
			{
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
			}
			else
			{
				LOG_ERROR("No mediadescription for tInerMeasUnitData found!");
			}
		}
		else
		{
			LOG_DUMP("Skipped initialization for tInerMeasUnitData, because it is already initialized.");
		}

		return &m_IMUDataStreamType;

	case RoadSignExt:
		LOG_ERROR("eZmqStruct 'RoadSignExt' not implemented.");
		break;

	case Position:
		LOG_ERROR("eZmqStruct 'Position' not implemented.");
		break;

	case Obstacle:
		LOG_ERROR("eZmqStruct 'Obstacle' not implemented.");
		break;

	case TrafficSign:
		LOG_ERROR("eZmqStruct 'TrafficSign' not implemented.");
		break;

	case ParkingSpace:
		LOG_ERROR("eZmqStruct 'ParkingSpace' not implemented.");
		break;

	case Ultrasonic:
		if (!m_USDataStreamType)
		{
			if IS_OK(create_adtf_default_stream_type_from_service("tUltrasonicStruct", m_USDataStreamType, m_USDataSampleFactory))
			{
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
			}
			else
			{
				LOG_ERROR("No mediadescription for tUltrasonicStruct found!");
			}
		}
		else
		{
			LOG_DUMP("Skipped initialization for tUltrasonicStruct, because it is already initialized.");
		}

		return &m_USDataStreamType;

	case Voltage:
		if (!m_VoltageStructStreamType)
		{
			if IS_OK(create_adtf_default_stream_type_from_service("tVoltageStruct", m_VoltageStructStreamType, m_VoltageStructSampleFactory))
			{
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
			}
			else
			{
				LOG_ERROR("No mediadescription for tVoltageStruct found!");
			}
		}
		else
		{
			LOG_DUMP("Skipped initialization for tVoltageStruct, because it is already initialized.");
		}

		return &m_VoltageStructStreamType;

	case PolarCoordinate:
		LOG_ERROR("eZmqStruct 'PolarCoordinate' not implemented.");
		break;

	case LaserScanner:
		if (!m_LSStructStreamType)
		{
			if (ERR_NOERROR == create_adtf_default_stream_type_from_service("tLaserScannerData", m_LSStructStreamType, m_LSStructSampleFactory))
			{
				access_element::find_index(m_LSStructSampleFactory, "ui32Size", m_ddlLSDataId.size);
				access_element::find_array_index(m_LSStructSampleFactory, "tScanArray", m_ddlLSDataId.scanArray);
			}
			else
			{
				LOG_ERROR("No mediadescription for tLaserScannerData found!");
			}
		}
		else
		{
			LOG_DUMP("Skipped initialization for tLaserScannerData, because it is already initialized.");
		}

		return &m_LSStructStreamType;

	default:
		LOG_ERROR("Could not get or create stream type for unrecognized eZmqStruct %d", sampleType);
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
