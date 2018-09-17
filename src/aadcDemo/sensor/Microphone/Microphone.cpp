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

using namespace adtf::util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::system;

#include "Microphone.h"

ADTF_PLUGIN(LABEL_MICROPHONE_STREAMING_SOURCE, cMicrophone);

cMicrophone::cMicrophone() : m_audioInputDevice(nullptr)
{
    //Register Properties
    RegisterPropertyVariable("Sampling Rate [Hz]", m_samplingRate);

    RegisterPropertyVariable("Device Id", m_audioInputDeviceId);
}

tResult cMicrophone::Construct()
{
    RETURN_IF_FAILED(cSampleStreamingSource::Construct());

    //Define stream output format
    object_ptr<IStreamType> pType = make_object_ptr<cStreamType>(stream_meta_type_audio());

    //Register Output Pin
    RETURN_IF_FAILED(create_pin(*this, m_oOut, "audio", pType));

    QAudioFormat format;
    // Set up the desired format, for example:
    format.setSampleRate(m_samplingRate);

    format.setChannelCount(1);
    format.setSampleSize(8);
    format.setCodec("audio/pcm");
    format.setByteOrder(QAudioFormat::LittleEndian);
    format.setSampleType(QAudioFormat::UnSignedInt);

    
    //get all the available devices
    QList<QAudioDeviceInfo> listAvalaibleDevices = QAudioDeviceInfo::availableDevices(QAudio::AudioInput);

    if (listAvalaibleDevices.empty())
    {
        RETURN_ERROR_DESC(ERR_BAD_DEVICE, "Found no avalaible audio devices.");
    }

    // print the devices to the console
    for (int i = 0; i< listAvalaibleDevices.count(); i++)
    {
        LOG_INFO(cString::Format("Found available Audio devices: # %d: %s", i, listAvalaibleDevices[i].deviceName().toStdString().c_str()));
    }

    //check if set id is available
    if (m_audioInputDeviceId >= listAvalaibleDevices.count())
    {
        RETURN_ERROR_DESC(ERR_BAD_DEVICE, cString::Format("Invalid ID for audio device was set: %d (avalaible devices count: %d)", m_audioInputDeviceId, listAvalaibleDevices.count()));
    }
         
    QAudioDeviceInfo info = listAvalaibleDevices[m_audioInputDeviceId];

    if (!info.isFormatSupported(format))
    {
        RETURN_ERROR_DESC(ERR_BAD_DEVICE, "Format not supported by current device");
    }

    m_audioInputDevice = new QAudioInput(info, format, this);
    
    LOG_INFO(cString::Format("Opened Audio Device %s with %d Hz", info.deviceName().toStdString().c_str(), format.sampleRate()));

    RETURN_NOERROR;
}

tResult cMicrophone::StartStreaming()
{
    startDevice();

    m_audioInputDevice->start(this);

    RETURN_IF_FAILED(cSampleStreamingSource::StartStreaming());
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}


tResult cMicrophone::StopStreaming()
{
    m_audioInputDevice->stop();

    stopDevice();

    return cSampleStreamingSource::StopStreaming();
}


qint64 cMicrophone::writeData(const char *data, qint64 maxSize)
{
    object_ptr<ISample> pSample;

    if (IS_OK(alloc_sample(pSample)))
    {
        object_ptr_locked<ISampleBuffer> pBuffer;
        pSample->SetTime(m_pClock->GetStreamTime());
        if (IS_OK(pSample->WriteLock(pBuffer, maxSize)))
        {
            pBuffer->Write(adtf_memory_buffer<const tVoid>(data, maxSize));
            //Not needed as in demo_virtual_clock?
            //pBuffer->Unlock();
        }
    }

    m_oOut << pSample << trigger;
    return maxSize;
}
