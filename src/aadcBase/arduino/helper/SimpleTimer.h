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

#ifndef SIMPLETIMER_HPP
#define SIMPLETIMER_HPP

/*!
 * A helper class which measures time in seconds.
 *
 * A small helper class to measure time. It uses the boost libraray. The timer starts when the
 * object is created. It is used to timeout the serial communication.
 *
 */
class SimpleTimer
{
public:

    /*! Default constructor. */
    SimpleTimer() : start(clock::now()) {}

    /*! Destructor. */
    ~SimpleTimer() {}

    /*!
     * Restarts the timer.
     *
     * \return  returns the elapsed time since start as double.     *
    */
    double restart()
    {
        timepoint current = clock::now();
        double elapsed = boost::chrono::duration_cast<second>(current - start).count();
        start = current;
        return elapsed;
    }

    /*!
     * Returns the elapsed time.
     *
     * \return  Returns the elapsed time since start as double.     *
     *
     */
    double elapsed()
    {
        timepoint current = clock::now();
        double elapsed = boost::chrono::duration_cast<second>(current - start).count();
        return elapsed;
    }

private:

    /*! Defines an alias representing the clock. */
    typedef boost::chrono::high_resolution_clock clock;

    /*! Defines an alias representing the second. */
    typedef boost::chrono::duration< double, boost::ratio<1> > second;

    /*! Defines an alias representing the timepoint. */
    typedef boost::chrono::high_resolution_clock::time_point timepoint;

    /*! The start */
    boost::chrono::high_resolution_clock::time_point start;
};

#endif
