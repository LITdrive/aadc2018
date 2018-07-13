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


#include <adtf_systemsdk.h>

using namespace adtf::util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::system;

#include "SignalValueGenerator.h"

ADTF_PLUGIN(LABEL_SIGNALVALUEGENERATOR_STREAMING_SOURCE, cSignalValueGenerator);

cSignalValueGenerator::cSignalValueGenerator()
{
    RegisterPropertyVariable("Value", m_value);
    RegisterPropertyVariable("Timer Interval [Hz]", m_timerInterval);
}

tResult cSignalValueGenerator::Construct()
{
    RETURN_IF_FAILED(cSampleStreamingSource::Construct());

    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_signalValueSampleFactory))
    {
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_signalValueSampleFactory, "ui32ArduinoTimestamp", m_ddlSignalValueId.timeStamp));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_signalValueSampleFactory, "f32Value", m_ddlSignalValueId.value));

    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }

    //Register Output Pin
    RETURN_IF_FAILED(create_pin(*this, m_oOut, "output", pTypeSignalValue));

    RETURN_NOERROR;
}

tResult cSignalValueGenerator::StartStreaming()
{
    RETURN_IF_FAILED(cSampleStreamingSource::StartStreaming());
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    m_oTimer = kernel_timer(cString(get_named_graph_object_full_name(*this) + "::frame_timer"), m_timerInterval, 0,
        &cSignalValueGenerator::TimerFunc,
        this);
    
    if (!m_oTimer.Stoppable())
    {
        RETURN_ERROR_DESC(ERR_UNEXPECTED, "Unable to create kernel timer");
    }

    RETURN_NOERROR;
}

tResult cSignalValueGenerator::StopStreaming()
{
    m_oTimer.Stop();

    return cSampleStreamingSource::StopStreaming();
}

tVoid cSignalValueGenerator::TimerFunc()
{
    TransmitNewSample();;
}

tResult cSignalValueGenerator::TransmitNewSample()
{
    object_ptr<ISample> pSample;
    if IS_OK(alloc_sample(pSample, m_pClock->GetStreamTime()))
    {
        auto oCodec = m_signalValueSampleFactory.MakeCodecFor(pSample);
        tFloat32 value = static_cast<tFloat32>(m_value);
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, &value));

    }
    if (pSample)
    {
        m_oOut << pSample << trigger;
    }

    RETURN_NOERROR;

}