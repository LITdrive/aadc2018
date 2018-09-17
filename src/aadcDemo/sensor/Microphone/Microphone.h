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

#include <QAudioInput>
#include "AudioIODevice.h"

#pragma once

#define CID_MICROPHONE_STREAMING_SOURCE "microphone.streaming_source.demo.aadc.cid"
#define LABEL_MICROPHONE_STREAMING_SOURCE "Microphone"


/*! A the main class of the microphone. */
class cMicrophone : public adtf::streaming::cSampleStreamingSource, public AudioIODevice
{

public:
    ADTF_CLASS_ID_NAME(cMicrophone,
        CID_MICROPHONE_STREAMING_SOURCE,
        LABEL_MICROPHONE_STREAMING_SOURCE);



    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock), REQUIRE_INTERFACE(adtf::services::IKernel));

private:

    /*! The frame delay */
    property_variable<tInt>   m_samplingRate = 8000;

    //sample pin writer
    cSampleWriter m_oOut;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    //QAudioInput
    QAudioInput *m_audioInputDevice;
    
    /*! Name of the audio input */
    property_variable<tInt> m_audioInputDeviceId = 0;


public:
    /*! Default constructor. */
    cMicrophone();

    tResult Construct() override;
    tResult StartStreaming() override;
    tResult StopStreaming() override;

    //overwrite IO Device
    qint64 writeData(const char *data, qint64 maxSize) override;


};
