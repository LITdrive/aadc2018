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

#include <adtf_systemsdk.h>
#include <aadc_structs.h>

#define CID_JURY_COM_TCP_STREAMING_SOURCE "jury_communication_tcp.streaming_source.base.aadc.cid"
#define LBL_JURY_COM_TCP_STREAMING_SOURCE "Jury Communication TCP Receiver"
#define IID_TCP_SOCKET "socket.tcp.adtf"

 /**
  * This interface is used by the sink to access the socket of the source, if requested
  */
class ISocket : public adtf::ucom::ant::IObject
{
public:
    ADTF_IID(ISocket, IID_TCP_SOCKET);

public:
    virtual tResult Send(const tVoid* pData, tSize nDataSize) = 0;
};

/**
 * Implements a UDP socket that provides ISocket.
 */
class cSocketWrapper : public adtf::ucom::object<adtf::util::cStreamSocket, ISocket>
{
public:
    tResult Send(const tVoid* pData, tSize nDataSize) override
    {
        tInt nDataSent = 0;
        RETURN_IF_FAILED(Write(pData, static_cast<tInt>(nDataSize), &nDataSent));
        if (static_cast<tSize>(nDataSent) != nDataSize)
        {
            RETURN_ERROR_DESC(ERR_INVALID_ARG, "Unable to send data via TCP, data size is too large: %d", nDataSize);
        }

        RETURN_NOERROR;
    }

    tResult Read(tVoid* pDestination, tSize nBytes, tInt* pBytesRead)
    {
        return adtf::util::cStreamSocket::Read(pDestination, static_cast<tInt>(nBytes), pBytesRead);
    }

private:
    tUInt32 m_nRemoteAddress = 0;
    tUInt16 m_nRemotePort = 0;
};

class cTcpSource : public adtf::streaming::cSampleStreamingSource
{
public:
    ADTF_CLASS_ID_NAME(cTcpSource,
                       CID_JURY_COM_TCP_STREAMING_SOURCE,
                       LBL_JURY_COM_TCP_STREAMING_SOURCE);

    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock));

public:
    cTcpSource();

    tResult Construct() override;
    tResult StartStreaming() override;
    tResult Init() override;
    tResult Shutdown() override;
    tResult StopStreaming() override;

private:
    tVoid ReadThread();
    tResult ProcessJuryContainer();
    tResult ReadAndForwardPacket();
    tResult TransmitFileData(adtf::streaming::cSampleWriter& output, void* data, tSize length);
    tResult TransmitJuryStruct(tJuryStruct& juryStruct);

    //Media Descriptions
    struct tJuryStructId
    {
        tSize actionId;
        tSize maneuverEntry;
    } m_ddlJuryStructId;

    /*! The jury structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_juryStructSampleFactory;
    adtf::streaming::cSampleWriter m_oOutputJuryStruct;

    /*! The property enable console output */
    adtf::base::property_variable<tBool> m_propEnableConsoleOutput = tFalse;
    /*! The property TCP port */
    adtf::base::property_variable<tUInt> m_propTCPPort = 54321;

    adtf::ucom::object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*! output pin for maneuvers list*/
    adtf::streaming::cSampleWriter m_oOutputManeuverList;
    /*! output pin for xodr map */
    adtf::streaming::cSampleWriter m_oOutputOpenDriveMap;
    /*! output pin for trafficsign map */
    adtf::streaming::cSampleWriter m_oOutputTrafficSignMap;
    /*! The read thread */
    adtf::system::kernel_thread_looper m_oReadThread;

    /*! Buffer for read data */
    std::vector<tUInt8> m_oReadBuffer;

    /*! The client connection established */
    tBool m_clientConnectionEstablished;
    /*! The server socket */
    adtf_util::cServerSocket m_serverSocket;

    /*! The socket */
    adtf::ucom::object_ptr<cSocketWrapper> m_pSocket;
    /*! The total bytes read */
    tSize m_totalBytesRead = 0;
    /*! Number of packets */
    tSize m_packetCnt = 0;

    /*! Number of maximum packets */
    static const tSize MaxPacketCount = 1000;
};
