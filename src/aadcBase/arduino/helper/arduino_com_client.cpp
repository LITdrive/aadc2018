/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra  $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/


#include "arduino_com_client.h"
#include "SimpleTimer.h"
#include "arduino_com_helpers.h"

arduino_com_client::arduino_com_client() : _arduino_id(-1), _software_version(-1), _port_num(-1), _running(false) {}

arduino_com_client::~arduino_com_client() {}

bool arduino_com_client::begin(std::string device)
{
    return _port.open(device.c_str(), 115200);
}

void arduino_com_client::end()
{
    _port.close();
}

bool arduino_com_client::init(const int arduinoId, const std::string& serialDevicePrefix, const int maxDeviceNum)
{
    //check all ports
    for (int i = 0; i < maxDeviceNum; i++)
    {
        const std::string deviceString = serialDevicePrefix + std::to_string(i);

        if (begin(deviceString))
        {
            //LogNamedMessage(cString::Format("Device found on port: %s", deviceString.c_str()));
            if (start_reading(arduinoId))
            {
                _port_num = i;
                return true;
            }
            else
            {
                //close this port
                end();
            }

        }
    }

    return false;
}

bool arduino_com_client::start_reading(int arduino_id)
{
    if (_running)
    {
        stop_reading();
    }

    _running = true;
    _thread = boost::thread(&arduino_com_client::_thread_func, this);

    std::vector<uint8_t> frame;
    SimpleTimer timer;

    do
    {
        send_request();
        for (;;)
        {
            frame.clear();
            const bool info_available = get_next_info_frame(frame);

            if (info_available)
            {
                tInfoData info;
                memcpy(&info, frame.data() + sizeof(tArduinoHeader), sizeof(tInfoData));
                _arduino_id = info.ui8ArduinoAddress;
                _software_version = info.ui16ArduinoVersion;

                if (_arduino_id != arduino_id)
                {
                    return false;
                }
                break;
            }

            if (timer.elapsed() > TIMEOUT)
            {
                //end();
                return false;
            }
        }

        if (timer.elapsed() > TIMEOUT)
        {
            //end();
            return false;
        }
    }
    while (_arduino_id < 0);

    boost::lock_guard<boost::mutex> lock(_buffer_mutex);
    _frame_buffer.clear();
    _port.flush();


    return true;
}

void arduino_com_client::stop_reading()
{
    _running = false;
    _thread.join();
    _frame_buffer.clear();
}

void arduino_com_client::clear_buffer()
{
    boost::lock_guard<boost::mutex> lock(_buffer_mutex);
    _frame_buffer.clear();
}

void arduino_com_client::send_request()
{
    std::vector<uint8_t> frame;
    tArduinoHeader header;
    header.ui8ID = ID_ARD_ACT_REQUEST;
    header.ui8DataLength = 0;
    header.ui32Timestamp = 0;

    uint8_t* p = reinterpret_cast<uint8_t*>(&header);
    for (size_t i = 0; i < sizeof(tArduinoHeader); i++)
        frame.push_back(*p++);

    uint16_t CRC = fletcher16(frame.data(), uint8_t(frame.size()));

    p = reinterpret_cast<uint8_t*>(&CRC);
    for (size_t i = 0; i < sizeof(uint16_t); i++)
        frame.push_back(*p++);

    std::vector<uint8_t> stuffed_frame = stuff_frame(frame);

    boost::lock_guard<boost::mutex> lock(_serial_mutex);
    _port.write(stuffed_frame.data(), int(stuffed_frame.size()));
}

void arduino_com_client::send_steering(float angle)
{
    // remap angle
    angle = angle > 100.f ? 100.f : angle;
    angle = angle < -100.f ? -100.f : angle;
    angle += 100.f;
    angle *= 0.9f;

    tArduinoHeader header;
    header.ui8ID = ID_ARD_ACT_STEER_SERVO;
    header.ui8DataLength = sizeof(tServoData);
    header.ui32Timestamp = 0;

    tServoData data;
    data.ui8Angle = (uint8_t)angle;

    std::vector<uint8_t> stuffed_frame = _pack_and_stuff_data(header, data);

    boost::lock_guard<boost::mutex> lock(_serial_mutex);
    _port.write(stuffed_frame.data(), int(stuffed_frame.size()));

}

void arduino_com_client::send_speed(float speed)
{
    // remap speed
    speed = speed > 100.f ? 100.f : speed;
    speed = speed < -100.f ? -100.f : speed;
    speed += 100.f;
    speed *= 0.9f;

    tArduinoHeader header;
    header.ui8ID = ID_ARD_ACT_SPEED_CONTR;
    header.ui8DataLength = sizeof(tServoData);
    header.ui32Timestamp = 0;

    tServoData data;
    data.ui8Angle = (uint8_t)speed;

    std::vector<uint8_t> stuffed_frame = _pack_and_stuff_data(header, data);

    boost::lock_guard<boost::mutex> lock(_serial_mutex);
    _port.write(stuffed_frame.data(), int(stuffed_frame.size()));

}

void arduino_com_client::send_light(uint8_t light_mask)
{
    tArduinoHeader header;
    header.ui8ID = ID_ARD_ACT_LIGHT;
    header.ui8DataLength = sizeof(tLightData);
    header.ui32Timestamp = 0;

    tLightData data;
    data.ui8LightMask = light_mask;

    std::vector<uint8_t> stuffed_frame = _pack_and_stuff_data(header, data);

    boost::lock_guard<boost::mutex> lock(_serial_mutex);
    _port.write(stuffed_frame.data(), int(stuffed_frame.size()));

}

void arduino_com_client::send_emergency_stop()
{
    tArduinoHeader header;
    header.ui8ID = ID_ARD_ACT_EMERGENCY_STOP;
    header.ui8DataLength = sizeof(tEmergencyStopData);
    header.ui32Timestamp = 0;

    tEmergencyStopData data;
    data.ui8IsTriggerd = 1;

    std::vector<uint8_t> stuffed_frame = _pack_and_stuff_data(header, data);

    boost::lock_guard<boost::mutex> lock(_serial_mutex);
    _port.write(stuffed_frame.data(), int(stuffed_frame.size()));

}

void arduino_com_client::send_watchdog_trigger()
{
    tArduinoHeader header;
    header.ui8ID = ID_ARD_ACT_WATCHDOG;
    header.ui8DataLength = sizeof(tWatchdogData);
    header.ui32Timestamp = 0;

    tWatchdogData data;
    data.ui8IsTriggerd = 1;

    std::vector<uint8_t> stuffed_frame = _pack_and_stuff_data(header, data);

    boost::lock_guard<boost::mutex> lock(_serial_mutex);
    _port.write(stuffed_frame.data(), int(stuffed_frame.size()));

}

void arduino_com_client::send_disable_uss()
{
    std::vector<uint8_t> frame;
    tArduinoHeader header;
    header.ui8ID = ID_ARD_DISABLE_USS;
    header.ui8DataLength = 0;
    header.ui32Timestamp = 0;

    uint8_t* p = reinterpret_cast<uint8_t*>(&header);
    for (size_t i = 0; i < sizeof(tArduinoHeader); i++)
        frame.push_back(*p++);

    uint16_t CRC = fletcher16(frame.data(), uint8_t(frame.size()));

    p = reinterpret_cast<uint8_t*>(&CRC);
    for (size_t i = 0; i < sizeof(uint16_t); i++)
        frame.push_back(*p++);

    std::vector<uint8_t> stuffed_frame = stuff_frame(frame);

    boost::lock_guard<boost::mutex> lock(_serial_mutex);
    _port.write(stuffed_frame.data(), int(stuffed_frame.size()));
}

void arduino_com_client::send_enable_uss()
{
    std::vector<uint8_t> frame;
    tArduinoHeader header;
    header.ui8ID = ID_ARD_ENABLE_USS;
    header.ui8DataLength = 0;
    header.ui32Timestamp = 0;

    uint8_t* p = reinterpret_cast<uint8_t*>(&header);
    for (size_t i = 0; i < sizeof(tArduinoHeader); i++)
        frame.push_back(*p++);

    uint16_t CRC = fletcher16(frame.data(), uint8_t(frame.size()));

    p = reinterpret_cast<uint8_t*>(&CRC);
    for (size_t i = 0; i < sizeof(uint16_t); i++)
        frame.push_back(*p++);

    std::vector<uint8_t> stuffed_frame = stuff_frame(frame);

    boost::lock_guard<boost::mutex> lock(_serial_mutex);
    _port.write(stuffed_frame.data(), int(stuffed_frame.size()));

}

bool arduino_com_client::get_next_frame(std::vector<uint8_t>& frame)
{
    boost::lock_guard<boost::mutex> lock(_buffer_mutex);
    if (_frame_buffer.size() > 0)
    {
        frame = _frame_buffer.front();
        _frame_buffer.pop_front();
    }
    else
    {
        return false;
    }

    return true;
}

bool arduino_com_client::get_next_info_frame(std::vector<uint8_t>& frame)
{
    boost::lock_guard<boost::mutex> lock(_buffer_mutex);
    if (_frame_buffer.size() > 0)
    {
        frame = _frame_buffer.front();
        _frame_buffer.pop_front();
        if (frame.size() > 0 && frame[0] == ID_ARD_SENSOR_INFO)
            return true;
    }

    return false;
}

void arduino_com_client::_thread_func()
{
    std::vector<uint8_t> frame;
    while (_running)
    {
        int result = _read_and_destuff_frame(frame);
        switch (result)
        {
        case ERROR_STX_NOT_FOUND:// STX NOT FOUND
        case ERROR_ESC_BYTE_BROKEN:// ESCAPE BYTE FAIL
        case ERROR_ETX_NOT_FOUND:// ETX NOT FOUND
        case ERROR_NO_FRAME_DATA:// No more data available
        case ERROR_CRC_INVALID:// INVALID CRC
        case ERROR_NO_BYTES_AVAILABLE:// NO data available in general
            break;
        default:
            _push_frame(frame);           
        }
    }
}

int arduino_com_client::_read_and_destuff_frame(std::vector<uint8_t>& frame)
{
    frame.clear();
    uint8_t c = 0, last = 0;
    bool read_success = false;
    bool frame_received = false;
    int destlen = 0;
    SimpleTimer timer;

    // Sync to STX
    // timeout function
    //boost::lock_guard<boost::mutex> lock(_serial_mutex);
    do
    {
        if (read_success)
        {
            last = c;
            read_success = false;
        }
        else
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(5));
        }

        if (_port.read(c))
        {
            read_success = true;
        }

        if (timer.elapsed() > TIMEOUT)
        {
            // NO STX FOUND
            return ERROR_STX_NOT_FOUND;
        }

    }
    while (!(c == START_BYTE && last != ESCAPE_BYTE));

    // read incoming frame
    // timeout function
    timer.restart();
    while (_running)
    {
        if (_port.read(c))
        {
            if (c == END_BYTE)
            {
                frame_received = true;
                break;
            }

            if (c == ESCAPE_BYTE)
            {
                while (!_port.read(c))
                {
                    boost::this_thread::sleep(boost::posix_time::milliseconds(5));
                    if (timer.elapsed() > TIMEOUT)
                    {
                        // ESCAPE BYTE BREAK
                        return ERROR_ESC_BYTE_BROKEN;
                    }
                }
                frame.push_back(c);
            }
            else
            {
                frame.push_back(c);
                destlen++;
            }

            if (destlen > MAX_VECTOR_BUFFER_SIZE)
            {
                // NO ETX FOUND
                return ERROR_ETX_NOT_FOUND;
            }
        }

        if (timer.elapsed() > TIMEOUT)
        {
            // NO BYTES AVAILABLE
            return ERROR_NO_BYTES_AVAILABLE;
        }
    }

    if (frame_received)
    {
        uint16_t calculated_CRC = fletcher16(frame.data(), sizeof(tArduinoHeader) + frame[1]);
        uint16_t frame_CRC;
        memcpy(&frame_CRC, frame.data() + sizeof(tArduinoHeader) + frame[1], sizeof(uint16_t));

        if (calculated_CRC != frame_CRC)
        {
            // CRC INVALID
            return ERROR_CRC_INVALID;
        }

        return destlen;
    }

    // POTATO DATA RECEIVED
    return ERROR_NO_FRAME_DATA;
}

void arduino_com_client::_push_frame(std::vector<uint8_t> frame)
{
    boost::lock_guard<boost::mutex> lock(_buffer_mutex);
    _frame_buffer.push_back(frame);
    while (_frame_buffer.size() > BUFFERSIZE)
    {
        std::vector<uint8_t> popped = _frame_buffer.front();
        _frame_buffer.pop_front();
    }
}

template<typename T>
std::vector<uint8_t> arduino_com_client::_pack_and_stuff_data(tArduinoHeader& header, T& data)
{
    std::vector<uint8_t> frame;
    uint8_t* p = NULL;

    p = reinterpret_cast<uint8_t*>(&header);
    for (size_t i = 0; i < sizeof(tArduinoHeader); i++)
        frame.push_back(*p++);

    p = reinterpret_cast<uint8_t*>(&data);
    for (size_t i = 0; i < sizeof(data); i++)
        frame.push_back(*p++);

    uint16_t CRC = fletcher16(frame.data(), sizeof(tArduinoHeader) + sizeof(data));

    p = reinterpret_cast<uint8_t*>(&CRC);
    for (size_t i = 0; i < sizeof(uint16_t); i++)
        frame.push_back(*p++);

    std::vector<uint8_t> stuffed_frame = stuff_frame(frame);

    return stuffed_frame;
}