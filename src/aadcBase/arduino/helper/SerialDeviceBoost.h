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

#ifndef SERIALDEVICE_BOOST_HPP
#define SERIALDEVICE_BOOST_HPP

#include <boost/asio.hpp>
#include <boost/exception/get_error_info.hpp>
#include <iostream>
#include <boost/exception_ptr.hpp>

/*!
 * The serial communication implementation via boost.
 *
 * This class is an abstraction for the serial port using the boost libraray. It provides
 * minimal functionality to keep the interface clean.
 *
 */
class SerialDeviceBoost
{
public:

    /*! Default constructor. */
    SerialDeviceBoost();

    /*! Destructor. */
    ~SerialDeviceBoost() {};

    /*!
     * Opens.
     *
     * \param   device      The device.
     * \param   baudrate    The baudrate.
     *
     * \return  True if it succeeds, false if it fails.
     */
    bool    open(const char* device, unsigned long baudrate);


    /*! Closes this object. */
    void    close();

    /*!
     * Writes.
     *
     * \param [in,out]  data    If non-null, the data.
     * \param           len     The length.
     *
     * \return  An int.
     */
    int     write(void *data, int len);

    /*!
     * Gets the read.
     *
     * \return  An uint8_t.
     */
    uint8_t read();

    /*!
     * Reads the given c.
     *
     * \param [in,out]  c   The c to read.
     *
     * \return  True if it succeeds, false if it fails.
     */
    bool    read(uint8_t& c);

    /*!
     * Gets the available.
     *
     * \return  An int.
     */
    int     available();

    /*! Flushes this object. */
    void    flush();

private:

    /*! The i/o */
    boost::asio::io_service        m_io;

    /*! The port */
    boost::asio::serial_port       m_port;
};

#endif
