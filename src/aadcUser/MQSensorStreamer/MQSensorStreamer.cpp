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
#include "MQSensorStreamer.h"
#include "ADTF3_OpenCV_helper.h"

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_MQSENSORSTREAMER_DATA_TRIGGERED_FILTER,
                                    "MQSensorStreamer",
                                    cMQSensorStreamer,
                                    adtf::filter::pin_trigger({"input"}));

cMQSensorStreamer::cMQSensorStreamer() {

    // create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(
            stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    // register input pin
    Register(m_oReader, "input", pType);
    // register output pin
    Register(m_oWriter, "output", pType);

    // register callback for type changes
    m_oReader.SetAcceptTypeCallback(
            [this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType> &pType) -> tResult {
                return ChangeType(m_oReader, m_sImageFormat, *pType.Get(), m_oWriter);
            });
}

//implement the Configure function to read ALL Properties
tResult cMQSensorStreamer::Configure() {
	RETURN_IF_FAILED(_runtime->GetObject(m_zeromq_service));

	// create and configure the socket
	m_publisher = new zmq::socket_t(*m_zeromq_service->GetContext(), ZMQ_PUB);
	const int v_true = 1;
	m_publisher->setsockopt(ZMQ_CONFLATE, &v_true, sizeof(v_true));
    m_publisher->bind("tcp://*:5556");
    //m_publisher->bind("ipc://aadc_camera.ipc");

    RETURN_NOERROR;

    //m_publisher->close();
}

tResult cMQSensorStreamer::Process(tTimeStamp tmTimeOfTrigger) {

    object_ptr<const ISample> pReadSample;
    while (IS_OK(m_oReader.GetNextSample(pReadSample))) {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;

        // lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer))) {

            tUInt32 size = 3 * m_sImageFormat.m_ui32Width * m_sImageFormat.m_ui32Height;
            m_publisher->send(pReadBuffer->GetPtr(), size);
            LOG_INFO(cString::Format("Published %d bytes from the image buffer", size));
        }
    }

    RETURN_NOERROR;
}