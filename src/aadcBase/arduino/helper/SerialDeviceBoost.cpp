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


#include "SerialDeviceBoost.h"


SerialDeviceBoost::SerialDeviceBoost() : m_port(m_io)
{

}



bool SerialDeviceBoost::open(const char* device, unsigned long baudrate)
{
    boost::asio::serial_port_base::baud_rate
    BAUD(baudrate);
    // how big is each "packet" of data (default is 8 bits)
    boost::asio::serial_port_base::character_size
    C_SIZE(8);
    // what flow control is used (default is none)
    boost::asio::serial_port_base::flow_control
    FLOW(boost::asio::serial_port_base::flow_control::none);
    // what parity is used (default is none)
    boost::asio::serial_port_base::parity
    PARITY(boost::asio::serial_port_base::parity::none);
    // how many stop bits are used (default is one)
    boost::asio::serial_port_base::stop_bits
    STOP(boost::asio::serial_port_base::stop_bits::one);
    try
    {
        m_port.open(device);
        m_port.set_option(C_SIZE);
        m_port.set_option(FLOW);
        m_port.set_option(PARITY);
        m_port.set_option(STOP);
        m_port.set_option(BAUD);

    }
    catch (boost::exception&  e)
    {        
        std::cerr << boost::diagnostic_information(e);
        return false;
    }

    return true;
}

void SerialDeviceBoost::close()
{
    m_port.close();
}

int SerialDeviceBoost::write(void *data, int length)
{
    // if the buffer is full, write_some will not return
    // so receiver has to poll data
    int written = 0;
    try
    {
        written = int(m_port.write_some(boost::asio::buffer((uint8_t*)data, size_t(length))));
    }
    catch (boost::exception& e)
    {
        std::cerr << boost::diagnostic_information(e);
        return false;
    }

    return written;
}

uint8_t SerialDeviceBoost::read()
{
    uint8_t c = 0;
    try
    {
        boost::asio::read(m_port, boost::asio::buffer(&c, 1)); // available should be called before
        return c;
    }
    catch (boost::exception& e)
    {
        std::cerr << boost::diagnostic_information(e);
        return false;
    }

    return c;
}

bool SerialDeviceBoost::read(uint8_t& c)
{
    if (available())
    {
        try
        {
            boost::asio::read(m_port, boost::asio::buffer(&c, 1));
            return true;
        }
        catch (boost::exception& e)
        {

            std::cerr << boost::diagnostic_information(e);
            return false;
        }
    }

    return false;
}

int SerialDeviceBoost::available()
{
    int bytes = 0;

#if defined(BOOST_ASIO_WINDOWS) || defined(__CYGWIN__)
    COMSTAT status;
    if (::ClearCommError(m_port.lowest_layer().native_handle(), NULL, &status) != 0)
    {
        bytes = status.cbInQue;
    }
#else
    if (::ioctl(m_port.lowest_layer().native_handle(), FIONREAD, &bytes) == 0)
    {
        // error
    }
#endif
    return bytes;
}

void SerialDeviceBoost::flush()
{
    boost::system::error_code error;
#if defined(BOOST_ASIO_WINDOWS) || defined(__CYGWIN__)
#else
    // TCIOFLUSH flush input/output
//    if (0 == ::tcflush(_port.lowest_layer().native_handle(), TCIOFLUSH))
//    {
//        error = boost::system::error_code();
//    }
//    else
//    {
//        error = boost::system::error_code(errno,
//            boost::asio::error::get_system_category());
//    }

    // 0 = rx, 1 = tx, 2 = both
    ::ioctl(m_port.lowest_layer().native_handle(), TCIOFLUSH, 2);
#endif

    return;
}
