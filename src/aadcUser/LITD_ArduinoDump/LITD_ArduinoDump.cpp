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
#include "LITD_ArduinoDump.h"
#include <fstream>
#include <sstream>
#include <ADTF3_helper.h>

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ARDUINO_DUMP_FILTER,
	"LITD ArduinoDump",
	cArduinoDump,
	adtf::filter::pin_trigger({"wheel_left"}));


cArduinoDump::cArduinoDump()
{
	RegisterPropertyVariable("CSV WheelLeft", m_dump_path_left);
	RegisterPropertyVariable("CSV WheelRight", m_dump_path_right);

	object_ptr<IStreamType> pTypeWheelData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tWheelData", pTypeWheelData, m_WheelDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "ui32ArduinoTimestamp", m_ddlWheelDataIndex.ArduinoTimestamp);
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "ui32WheelTach", m_ddlWheelDataIndex.WheelTach);
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "i8WheelDir", m_ddlWheelDataIndex.WheelDir);
    }
    else
    {
        LOG_INFO("No mediadescription for tWheelData found!");
    }
    Register(m_oInputWheelLeft, "wheel_left", pTypeWheelData);
    Register(m_oInputWheelRight, "wheel_right", pTypeWheelData);
}

tResult cArduinoDump::Configure()
{
	cFilename dumpFileLeft = m_dump_path_left;
	adtf_resolve_macros(dumpFileLeft);
	m_dumpWriterLeft = new std::ofstream(dumpFileLeft);

	cFilename dumpFileRight = m_dump_path_right;
	adtf_resolve_macros(dumpFileRight);
	m_dumpWriterRight = new std::ofstream(dumpFileRight);

    RETURN_NOERROR;
}

tResult cArduinoDump::Process(tTimeStamp tmTimeOfTrigger)
{
	object_ptr<const ISample> pSampleFromWheelLeft;

    while (IS_OK(m_oInputWheelLeft.GetNextSample(pSampleFromWheelLeft)))
	{
		tUInt32 tach = 0;
		tInt8 dir = 0;
		tUInt32 timestamp = 0;

		auto oDecoder = m_WheelDataSampleFactory.MakeDecoderFor(*pSampleFromWheelLeft);

		RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, &timestamp));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelTach, &tach));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelDir, &dir));

		std::ostringstream row;
  		row << timestamp << ',' << (int)dir << ',' << tach << '\n';
		*m_dumpWriterLeft << row.str();
	}

	object_ptr<const ISample> pSampleFromWheelRight;

	while (IS_OK(m_oInputWheelRight.GetNextSample(pSampleFromWheelRight)))
	{
		tUInt32 tach = 0;
		tInt8 dir = 0;
		tUInt32 timestamp = 0;

		auto oDecoder = m_WheelDataSampleFactory.MakeDecoderFor(*pSampleFromWheelRight);

		RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, &timestamp));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelTach, &tach));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelDir, &dir));

		std::ostringstream row;
  		row << timestamp << ',' << (int)dir << ',' << tach << '\n';
		*m_dumpWriterRight << row.str();
	}
	
	m_dumpWriterLeft->flush();
	m_dumpWriterRight->flush();

    RETURN_NOERROR;
}
