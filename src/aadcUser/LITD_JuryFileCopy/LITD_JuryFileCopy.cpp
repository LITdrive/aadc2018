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
#include "LITD_JuryFileCopy.h"
#include <fstream>
#include <ADTF3_helper.h>

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_JURY_FILE_COPY_FILTER,
	"LITD JuryFileCopy",
	cJuryFileCopy,
	adtf::filter::pin_trigger({"opendrive", "roadsigns", "maneuver"}));


cJuryFileCopy::cJuryFileCopy()
{
	RegisterPropertyVariable("Jury Dump Path (opendrive.xml)", m_jury_dump_opendrive);
	RegisterPropertyVariable("Jury Dump Path (roadsigns.xml)", m_jury_dump_roadsigns);
	RegisterPropertyVariable("Jury Dump Path (maneuver.xml)", m_jury_dump_maneuver);

	object_ptr<IStreamType> pTypeBoolSignalValue;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory))
	{
		access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.ui32ArduinoTimestamp);
		access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue"), m_ddlBoolSignalValueId.bValue);
	}
	else
	{
		LOG_INFO("No mediadescription for tBoolSignalValue found!");
	}

	object_ptr<IStreamType> pTypeAnonymous = make_object_ptr<cStreamType>(stream_meta_type_anonymous());

	Register(m_opendrive_reader, "opendrive", pTypeAnonymous);
	Register(m_roadsigns_reader, "roadsigns", pTypeAnonymous);
	Register(m_maneuver_reader, "maneuver", pTypeAnonymous);

	// update signal
	Register(m_update_writer, "update", pTypeBoolSignalValue);
}

tResult cJuryFileCopy::Configure()
{
    RETURN_NOERROR;
}

tResult cJuryFileCopy::Process(tTimeStamp tmTimeOfTrigger)
{
	object_ptr<const ISample> pReadSample;

	while (IS_OK(m_opendrive_reader.GetNextSample(pReadSample)))
	{
		object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
		RETURN_IF_FAILED(pReadSample->Lock(pSampleBuffer));

		// read file
		cString fileString;
		fileString.SetBuffer(pSampleBuffer->GetSize());
		memcpy(fileString.GetBuffer(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());

		// dump file
		cFilename juryDumpFile = m_jury_dump_opendrive;
		adtf_resolve_macros(juryDumpFile);
		std::ofstream juryDumpWriter(juryDumpFile.GetPtr());
		juryDumpWriter.write(fileString.GetBuffer(), pSampleBuffer->GetSize());
		juryDumpWriter.close();
		LOG_INFO("Dumped opendrive.xodr map to '%s'", juryDumpFile.GetPtr());

		transmitBoolSignalValue(m_update_writer, tmTimeOfTrigger, m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, true);
	}

	while (IS_OK(m_roadsigns_reader.GetNextSample(pReadSample)))
	{
		object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
		RETURN_IF_FAILED(pReadSample->Lock(pSampleBuffer));

		// read file
		cString fileString;
		fileString.SetBuffer(pSampleBuffer->GetSize());
		memcpy(fileString.GetBuffer(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());

		// dump file
		cFilename juryDumpFile = m_jury_dump_roadsigns;
		adtf_resolve_macros(juryDumpFile);
		std::ofstream juryDumpWriter(juryDumpFile.GetPtr());
		juryDumpWriter.write(fileString.GetBuffer(), pSampleBuffer->GetSize());
		juryDumpWriter.close();
		LOG_INFO("Dumped roadsigns.xml to '%s'", juryDumpFile.GetPtr());

		transmitBoolSignalValue(m_update_writer, tmTimeOfTrigger, m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, true);
	}

	while (IS_OK(m_maneuver_reader.GetNextSample(pReadSample)))
	{
		object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
		RETURN_IF_FAILED(pReadSample->Lock(pSampleBuffer));

		// read file
		cString fileString;
		fileString.SetBuffer(pSampleBuffer->GetSize());
		memcpy(fileString.GetBuffer(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());

		// dump file
		cFilename juryDumpFile = m_jury_dump_maneuver;
		adtf_resolve_macros(juryDumpFile);
		std::ofstream juryDumpWriter(juryDumpFile.GetPtr());
		juryDumpWriter.write(fileString.GetBuffer(), pSampleBuffer->GetSize());
		juryDumpWriter.close();
		LOG_INFO("Dumped maneuver.xml to '%s'", juryDumpFile.GetPtr());

		transmitBoolSignalValue(m_update_writer, tmTimeOfTrigger, m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, true);
	}

    RETURN_NOERROR;
}
