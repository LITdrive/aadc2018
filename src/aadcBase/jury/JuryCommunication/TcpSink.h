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
#pragma once

#include "TcpSource.h"
#include <adtf_systemsdk.h>

#define CID_JURY_COM_TCP_STREAMING_SINK "jury_communication_tcp.streaming_sink.base.aadc.cid"
#define LBL_JURY_COM_TCP_STREAMING_SINK "Jury Communication TCP Sender"

class cTcpSink: public adtf::streaming::cSampleStreamingSink
{
    public:
        ADTF_CLASS_ID_NAME(cTcpSink,
                           CID_JURY_COM_TCP_STREAMING_SINK,
                           LBL_JURY_COM_TCP_STREAMING_SINK);
    private:
    struct tDriverStructId
    {
        tSize stateId;
        tSize maneuverEntry;
    } m_ddlDriverStructId;

    /*! The driver structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_driverStructSampleFactory;

    /*! The property enable console output */
    adtf::base::property_variable<tBool> m_propEnableConsoleOutput = tFalse;

    public:
        cTcpSink();

        tResult Construct() override;
        tResult Init() override;
        tResult StartStreaming() override;
        tResult StopStreaming() override;

    private:
        tResult ProcessSamples(tTimeStamp tmTrigger);
        tResult ForwardDriverStruct(const tDriverStruct& driverStruct);

        /*! The input driver structure */
        adtf::streaming::cDynamicSampleReader m_oInputDriverStruct;

        /*! The socket */
        adtf::ucom::object_ptr<ISocket> m_pSocket;
};
