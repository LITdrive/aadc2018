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
#include "TcpSink.h"

using namespace adtf::util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::system;

ADTF_PLUGIN("ADTF TCP Source and Sink Plugin",
            cTcpSource, cTcpSink);

cTcpSink::cTcpSink()
{
    RegisterPropertyVariable("Enable console log", m_propEnableConsoleOutput);
}

tResult cTcpSink::Construct()
{
    RETURN_IF_FAILED(cSampleStreamingSink::Construct());

    object_ptr<IStreamType> pTypeDriverStruct;
    cString structName = "tDriverStruct";
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(structName.GetPtr(), pTypeDriverStruct, m_driverStructSampleFactory))
    {
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16StateID"), m_ddlDriverStructId.stateId));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16ManeuverEntry"), m_ddlDriverStructId.maneuverEntry));
    }
    else
    {
        LOG_WARNING(cString::Format("No mediadescription for %s found!", structName.GetPtr()));
    }
    const cString inputDriverStructName = "driver_struct";
    object_ptr<IStreamType> pType = make_object_ptr<cStreamType>(stream_meta_type_anonymous());
    RETURN_IF_FAILED(create_pin(*this, m_oInputDriverStruct, inputDriverStructName.GetPtr(), pType));

    RETURN_IF_FAILED(create_inner_pipe(*this, "input_data_trigger_handler", inputDriverStructName,
                                       std::bind(&cTcpSink::ProcessSamples, this, std::placeholders::_1)));

    RETURN_IF_FAILED(create_client<ISocket>(*this, "socket"));

    RETURN_NOERROR;
}

tResult cTcpSink::Init()
{
    RETURN_IF_FAILED(cSampleStreamingSink::Init());
    object_ptr<IBindingClient> pClient;
    if (IS_OK(FindBindingObject("socket", pClient)))
    {
        pClient->GetServerObject(m_pSocket);
    }

    if (!m_pSocket)
    {
        RETURN_ERROR_DESC(ERR_NOT_CONNECTED, "No binding object found, which is needed for socket interface.");
    }

    RETURN_NOERROR;
}

tResult cTcpSink::StartStreaming()
{
    RETURN_IF_FAILED(cSampleStreamingSink::StartStreaming());



    RETURN_NOERROR;
}

tResult cTcpSink::StopStreaming()
{
    m_pSocket.Reset();
    return cSampleStreamingSink::StopStreaming();
}

tResult cTcpSink::ProcessSamples(tTimeStamp /* tmTrigger */)
{
    object_ptr<const ISample> pSample;
    while (IS_OK(m_oInputDriverStruct.GetNextSample(pSample)))
    {
        tDriverStruct driverStruct;
        {
            auto oDecoder = m_driverStructSampleFactory.MakeDecoderFor(*pSample);
            RETURN_IF_FAILED(oDecoder.IsValid());
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriverStructId.stateId, &driverStruct.i16StateID));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriverStructId.maneuverEntry, &driverStruct.i16ManeuverEntry));
        }
        RETURN_IF_FAILED(ForwardDriverStruct(driverStruct));
    }

    RETURN_NOERROR;
}

tResult cTcpSink::ForwardDriverStruct(const tDriverStruct& driverStruct)
{

    return m_pSocket->Send(&driverStruct, sizeof driverStruct);
}
