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

#ifndef ARDUINO_COM_CLIENT_ASYNC_H
#define ARDUINO_COM_CLIENT_ASYNC_H

#include <boost/thread.hpp>
#include <deque>

#include "SerialDeviceBoost.h"

#include "arduino_protocol.h"

/*!
 * The Buffersize defines the software buffer size, where the arduino frames are temporarily stored. Be aware that setting it too small some messages
 * like info frames could be dropped. a buffer size of 32 - 64 is recommended.
 */
#define BUFFERSIZE 64
/*! A macro that defines the timeout of the serial port if no arduino protocol is recognized. */
#define TIMEOUT 3.0

#if WIN32 //check all com ports...
#define NUM_ARDUINO 100
#define SERIAL_DEVICE_PREFIX "\\\\.\\com"
#else
#define NUM_ARDUINO 5
#define SERIAL_DEVICE_PREFIX "/dev/ttyACM"
#endif


/*!
 * This is the implementation of the arduino protocol on linux host.
 *
 * The implementation on a linux system. It functions like the arduino communication in the
 * arduino aadc library. Normally there is no need to inspect this class further as all stuff
 * gets done in cArduinoCommunication class.
 *
 */
class arduino_com_client
{
public:
    /*! Default constructor. */
    arduino_com_client();
    /*! Destructor. */
    virtual ~arduino_com_client();

    /*!
     * Opens the given device. On linux this could be e.g. /dev/ttyACM0
     *
     * \param   device  The device name (on linux e.g. /dev/ttyACM0)
     *
     * \return  True if it succeeds (the port was opened), false if it fails.
     */
    bool begin(std::string device);
    
    /*! Ends the serial connection. */
    void end();

    /*!
     * Initializes the device with the given id. Loops through
     * all serial devices until maxDeviceNum and opens the device with 
     * the arduinoId
     *
     * \param   arduinoId           Identifier for the arduino.
     * \param   serialDevicePrefix  The serial device prefix (OS dependend).
     * \param   maxDeviceNum        The maximum device number.
     *
     * \return  True if it succeeds, false if it fails.
     */
    bool init(int arduinoId, const std::string& serialDevicePrefix, int maxDeviceNum);

    /*!
     * Gets the arduino identifier. (have a look in arduino_protocol.h)
     *
     * \return  The identifier.
     */
    int get_id()
    {
        return _arduino_id;
    }

    /*!
     * Gets port number of the device.
     *
     * \return  The port number.
     */
    int get_port_num()
    {
        return _port_num;
    }
    /*!
     * Gets the current arduino software version.
     *
     * \return  The software arduino version.
     */
    int get_software_version()
    {
        return _software_version;
    }

    /*!
     * Sets the arduino identifier.
     *
     * \param   id  The arduino identifier.
     */
    void set_id(int id)
    {
        _arduino_id = id;
    }

    /*!
     * Starts a reading.
     *
     * \return  True if it succeeds, false if it fails.
     */
    bool start_reading(int arduino_id);
    /*! Stops a reading. */
    void stop_reading();

    /*! Clears the buffer. */
    void clear_buffer();

    /*! Sends a request to the arduino. Then an info frame will be received. */
    void send_request();

    /*!
     * Sends steering.
     *
     * \param   angle   The angle value. this value will be mapped in the range of [-100, 100]. On
     * the current vehicle. this should translate from -30° left to 30° right
     */
    void send_steering(float angle);    // -100 = most left, 100 = most right

    /*!
     * Sends speed.
     *
     * \param   speed   The speed. This value is mapped in the range [-100, 100]
     */
    void send_speed(float speed);       // -100 = reverse,   100 = full gas

    /*!
     * Sends a light.
     *
     * \param   light_mask  The light mask. You can add different options  in this argument. habe a look in
     * arduino_protocol.h for the enums. Then combine them with the " | " - operator.
     */
    void send_light(uint8_t light_mask);

    /*! Sends the emergency stop. */
    void send_emergency_stop();

    /*! Sends the watchdog trigger. */
    void send_watchdog_trigger();

    /*! Sends the disable uss. */
    void send_disable_uss();

    /*! Sends the enable uss. */
    void send_enable_uss();

    /*!
     * Gets the next frame.
     *
     * \param [in,out]  frame   The frame.
     *
     * \return  True if it succeeds, false if it fails.
     */
    bool get_next_frame(std::vector<uint8_t>& frame);       // non blocking, returns true if a frame is available

    /*!
     * Gets the next information frame. This function must be called in a loop until an info frame is found.
     *
     * \param [in,out]  frame   The frame.
     *
     * \return  True if it succeeds, false if it fails.
     */
    bool get_next_info_frame(std::vector<uint8_t>& frame);  // non blocking, returns true if the next frame is a info frame

private:
    /*! Identifier for the arduino */
    int     _arduino_id;
    /*! The software version */
    int     _software_version;

    /*! The port number */
    int _port_num;

    /*! The serial port */
    SerialDeviceBoost                       _port;
    /*! The serial thread */
    boost::thread                           _thread;
    /*! The buffer mutex */
    boost::mutex                            _buffer_mutex;
    /*! The serial mutex */
    boost::mutex                            _serial_mutex;
 
    /*! Buffer for frame data */
    std::deque<std::vector<uint8_t>>      _frame_buffer;
    /*! True to running */
    bool                                    _running;

    template<typename T>
    /*!
     * Pack and stuff data. This is a preparation for sending a frame.
     *
     * \param [in]  header  The header.
     * \param [in]  data    The data.
     *
     * \return  A std::vector<uint8_t> where the stuffed data frame is stored
     */
    std::vector<uint8_t>    _pack_and_stuff_data(tArduinoHeader& header, T& data);

    /*!
     * Reads and destuff a frame. This is a preparation
     *
     * \param [in,out]  frame   The frame.
     *
     * \return  The size of the frame, otherwise a negative number. For error codes have a look in arduino_protocol.h
     */
    int                     _read_and_destuff_frame(std::vector<uint8_t>& frame);

    /*!
     * Pushes a frame into the buffer.
     *
     * \param   frame   The frame to push.
     */
    void                    _push_frame(std::vector<uint8_t> frame);
    /*! Thread function. */
    void                    _thread_func();


};




#endif
