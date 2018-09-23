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

#include "stdafx.h"

// how many milliseconds we wait for a request before dropping the sample (must be > 1000 ms)
#define ZMQ_REQUEST_TIMEOUT		3000

/*! input and output pin types (as specified in "aadc_structs.h") */
enum eZmqStruct
{
	Jury,
	Driver,
	SignalValue,
	BoolSignalValue,
	WheelData,
	InerMeasUnitData,
	RoadSignExt,
	Position,
	Obstacle,
	TrafficSign,
	ParkingSpace,
	Ultrasonic,
	Voltage,
	PolarCoordinate,
	LaserScanner
};

/*! pin name and type tuple */
typedef std::tuple<std::string, eZmqStruct> ZmqPinDef;

class cZmqBase : public cFilter
{
public:

	cZmqBase();

	virtual ~cZmqBase() = 0;

	tResult Init(tInitStage eStage) override;

	tResult Shutdown(tInitStage eStage) override;

	tResult Start() override;

	tResult Stop() override;
	
	tResult Configure();

	tResult Deconfigure();

	tResult ProcessInputs(tTimeStamp tmTimeOfTrigger);

	tResult ProcessOutput(zmq::message_t* frame, size_t index);

	// necessary for proper behaviour of the create_inner_pipe call
	using cRuntimeBehaviour::RegisterRunner;
	using cRuntimeBehaviour::RegisterInnerPipe;

protected:

	/*! desired input pin definitions (set by subclasses) */
	std::vector<ZmqPinDef> m_inputs;

	/*! desired output pin definitions (set by subclasses) */
	std::vector<ZmqPinDef> m_outputs;

	/*! new samples on these pins will trigger the process method (set by subclasses) */
	std::vector<std::string> m_triggers;

private:

	/*! server socket (tcp://ip:port OR ipc:///tmp/endpoint/0 OR inproc://endpoint) */
	property_variable<cString> m_server_socket_address = cString("tcp://127.0.0.1:5555");

	/* how many messages shall be queued in memory at a maximum (actual limit might be 60 - 70% lower) */
	property_variable<tInt> m_queue_length = 10;

	/* only take every nth trigger */
	property_variable<tInt> m_subsample_factor = 1;

	/*! clock service */
	object_ptr<adtf::services::IReferenceClock> m_pClock;

	/*! ZeroMQ context service */
	object_ptr<IZeroMQService> m_pZeroMQService;

	/*! mutex for process method synchronization */
	std::mutex m_oMutex;

	/*! ZeroMQ pair socket for asynchronous reply polling */
	zmq::socket_t* m_sck_pair = nullptr;

	/*! signals the ZeroMQ thread, that it can start connecting to the socket */
	bool m_parent_ready = false;

	/* signals the ZeroMQ thread, that it should stop */
	std::atomic<bool> m_runner_reset_signal { false };

	/*! synchronization of the m_parent_ready bool between the ZeroMQ thread and the parent */
	std::condition_variable m_runner_cv;

	/*! mutex for the condition variable */
	std::mutex m_runner_mutex;

	/* input pin readers with the pin name as its key */
	std::map<std::string, cPinReader*> m_pinReaders;

	/* output pin writers with the pin name as its key */
	std::map<std::string, cPinWriter*> m_pinWriters;

	/* some nullbytes, which will be sent for empty pins */
	uint8_t m_nullbytes[1024]{};

	/* count number of samples for applying the subsample factor */
	int m_num_samples = 0;

	/* number of dropped samples */
	int m_num_samples_dropped = 0;

	/* is the queue full and we are only dropping samples from now on? */
	bool m_drop_state = false;

	/* the deconfiguration routine can only be done once */
	bool m_deconfigured = false;

private:

	zmq::socket_t* InitializeClientSocket() const;

	void InitializeZeroMQThread();

	void InitializeInputPins();

	void InitializeOutputPins();

	void InitializePins(std::vector<ZmqPinDef>& pin_definitions, bool isReader);

	object_ptr<IStreamType>* GetStreamType(eZmqStruct sampleType);

	cSampleCodecFactory* GetSampleFactory(eZmqStruct sampleType);

	size_t GetStructSize(eZmqStruct sampleType) const;

	std::string GetPairSocketAddress() const;

private:

	static bool SendString(zmq::socket_t& socket, const std::string& string);

	static std::string ReceiveString(zmq::socket_t& socket);

private:

	// signal value
	cSampleCodecFactory m_SignalValueSampleFactory;
	object_ptr<IStreamType> m_SignalValueStreamType = nullptr;

	struct tSignalValueId
	{
		tSize timeStamp;
		tSize value;
	} m_ddlSignalValueId{};

	// bool signal value
	cSampleCodecFactory m_BoolSignalValueSampleFactory;
	object_ptr<IStreamType> m_BoolSignalValueStreamType = nullptr;

	struct tBoolSignalValueId
	{
		tSize ui32ArduinoTimestamp;
		tSize bValue;
	} m_ddlBoolSignalValueId{};

	// imu
	cSampleCodecFactory m_IMUDataSampleFactory;
	object_ptr<IStreamType> m_IMUDataStreamType = nullptr;

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
	} m_ddlInerMeasUnitDataIndex{};

	// ultrasonic
	cSampleCodecFactory m_USDataSampleFactory;
	object_ptr<IStreamType> m_USDataStreamType = nullptr;

	struct
	{
		tSignalValueId SideLeft;
		tSignalValueId SideRight;
		tSignalValueId RearLeft;
		tSignalValueId RearCenter;
		tSignalValueId RearRight;
	} m_ddlUltrasonicStructIndex{};

	// wheel encoders
	cSampleCodecFactory m_WheelDataSampleFactory;
	object_ptr<IStreamType> m_WheelDataStreamType = nullptr;

	struct
	{
		tSize ArduinoTimestamp;
		tSize WheelTach;
		tSize WheelDir;
	} m_ddlWheelDataIndex{};

	// voltages
	cSampleCodecFactory m_VoltageStructSampleFactory;
	object_ptr<IStreamType> m_VoltageStructStreamType = nullptr;

	struct
	{
		tSignalValueId ActuatorVoltage;
		tSignalValueId ActuatorCell1;
		tSignalValueId ActuatorCell2;
		tSignalValueId SensorVoltage;
		tSignalValueId SensorCell1;
		tSignalValueId SensorCell2;
		tSignalValueId SensorCell3;
		tSignalValueId SensorCell4;
		tSignalValueId SensorCell5;
		tSignalValueId SensorCell6;
	} m_ddlVoltageStructIndex{};

	// lidar
	cSampleCodecFactory m_LSStructSampleFactory;
	object_ptr<IStreamType> m_LSStructStreamType = nullptr;

	struct ddlLaserScannerDataId
	{
		tSize size;
		tSize scanArray;
	} m_ddlLSDataId{};
};
