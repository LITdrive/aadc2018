#pragma once

#include <adtf_systemsdk.h>
//#include "stdafx.h"

using namespace adtf::util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::system;
using namespace adtf::mediadescription;

const tChar* strAudioSampleStructDescription =
"<struct alignment=\"4\" name=\"tSimple\" version=\"2\">"
"<element name=\"cValue\" type=\"tFloat64\" arraysize=\"1\">"
"<deserialized alignment=\"4\"/>"
"<serialized byteorder=\"LE\" bytepos=\"0\"/>"
"</element>"
"</struct>";

#define CID_AUDIO_SAMPLE_STREAMING_SOURCE "audio_sample_streaming_source.adtf.cid"

/**
* A very simple streaming source that generates characters in a given interval
*/
class cAudioSampleStreamingSource : public adtf::streaming::cSampleStreamingSource
{
public:
	ADTF_CLASS_ID_NAME(cAudioSampleStreamingSource, CID_AUDIO_SAMPLE_STREAMING_SOURCE, "Audio Sample Streaming Source");

	ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IKernel), REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:
	// Microseconds
	property_variable<tInt64> m_nSpeed = 22; //1000000;	// 1 second (1/44100 = 22 µs sampling time)
	// tChar alphabet[26] = { 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z' };
	object_ptr<adtf::services::IReferenceClock> m_pClock;
	kernel_timer m_oTimer;
	tSize m_nValueElementIndex;
	cSampleCodecFactory m_oCodecFactorySimple;
	cSampleWriter m_oOutputWriter;
	IStreamingService::tStreamingState m_eState;

public:
	cAudioSampleStreamingSource();

	tResult Construct() override;
	tResult StartStreaming() override;
	tResult StopStreaming() override;
	tResult RequestPin(const tChar* strName, const iobject_ptr<const IStreamType>& pType, iobject_ptr<IOutPin>& pOutPin);

private:
	tVoid TimerFunc();
};