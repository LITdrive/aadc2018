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

#include "ArduinoBase.h"


cArduinoBase::cArduinoBase(ARDUINO_ID id) : m_arduinoID(id),
    m_serialDevicePrefix(SERIAL_DEVICE_PREFIX)
{
    RegisterPropertyVariable("Enable console log", m_propEnableConsoleOutput);
}


tResult cArduinoBase::Construct()
{
    RETURN_IF_FAILED(cSampleStreamingSource::Construct());

    RETURN_NOERROR;
}

tResult cArduinoBase::Destruct()
{
    return cSampleStreamingSource::Destruct();
}

tResult cArduinoBase::StartStreaming()
{
    RETURN_IF_FAILED(cSampleStreamingSource::StartStreaming());

    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    //create the timer
    m_oTimer = kernel_timer(cString(get_named_graph_object_full_name(*this) + "::generation_timer"),
                            m_nFrameDelay, 0, &cArduinoBase::TimerFunc, this);
    
    if (!m_oTimer.Stoppable())
    {
        RETURN_ERROR_DESC(ERR_UNEXPECTED, "Unable to create kernel timer");
    }

    RETURN_NOERROR;
}

tResult cArduinoBase::StopStreaming()
{
    m_oTimer.Stop();    

    return cSampleStreamingSource::StopStreaming();
}

tResult cArduinoBase::Init()
{
    RETURN_IF_FAILED(cSampleStreamingSource::Init());
    //InitArduino();
    if (m_serialDevice.init(m_arduinoID, m_serialDevicePrefix, NUM_ARDUINO))
    {
        LOG_INFO(cString::Format("Connected to arduino on port: %d\t ID: %d\t Software version: %d",
                         m_serialDevice.get_port_num(), m_serialDevice.get_id(), m_serialDevice.get_software_version()));
    }
    else
    {
        LOG_ERROR(cString::Format("Could not find an arduino with correct id: %d ", m_arduinoID));
        RETURN_ERROR_DESC(ERR_DEVICE_IO, cString::Format("Could not find an arduino with correct id: %d ", m_arduinoID));

    }
    RETURN_NOERROR;
}

tResult cArduinoBase::Shutdown()
{
    m_serialDevice.stop_reading();
    m_serialDevice.end();
    return cSampleStreamingSource::Shutdown();
}


tVoid cArduinoBase::TimerFunc()
{
    std::vector<uint8_t> frame;

    if (m_serialDevice.get_next_frame(frame))
    {
        tArduinoHeader header;
        memcpy(&header, frame.data(), sizeof(tArduinoHeader));
        tDataUnion data;
        memcpy(&data, frame.data() + sizeof(tArduinoHeader), header.ui8DataLength);
        const SENSOR_ID id = static_cast<SENSOR_ID>(header.ui8ID);
        const tUInt32 timestamp = header.ui32Timestamp;

        switch (id)
        {
            case ID_ARD_SENSOR_INFO:
                LOG_INFO(cString::Format("Info frame received.\tID: %d\tSoftware version: %d", data.info.ui8ArduinoAddress, data.info.ui16ArduinoVersion));
                break;
            case ID_ARD_SENS_ERROR:
                LOG_INFO(cString::Format("Error frame received from port: %s%d", m_serialDevicePrefix.c_str(), m_serialDevice.get_id()));
                break;            
            default:
            //this calls the child classes
                Transmit(id, timestamp, data);
                break;
        }
    }
}

