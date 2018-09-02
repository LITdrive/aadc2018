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

#include "LIT_CarControllerFilter.h"
#include "ADTF3_helper.h"
//*************************************************************************************************

ADTF_PLUGIN("LIT Car Controller Plugin", cCarControllerFilter);

cCarControllerFilter::cCarControllerFilter()
{
	RegisterPropertyVariable("update interval [msec]", m_f64updateInterval);

	object_ptr<IStreamType> pTypeSignalValue;
	if
	IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue,
		m_SignalValueSampleFactory))
	{
		(adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"),
		                                      m_ddlSignalValueId.timeStamp));
		(adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value)
		);
	}
	else
	{
		LOG_INFO("No mediadescription for tUltrasonicStruct found!");
	}

	create_pin(*this, m_oOutputSteeringController, "steering", pTypeSignalValue);
	create_pin(*this, m_oOutputSpeedController, "speed", pTypeSignalValue);
}

QWidget* cCarControllerFilter::CreateView()
{
	m_pUiFileWidget = new cCarControllerWidget(nullptr, m_f64updateInterval);

	connect(m_pUiFileWidget, SIGNAL(sendSteering(tFloat32)), this, SLOT(SendSteering(tFloat32)));
	connect(m_pUiFileWidget, SIGNAL(sendSpeed(tFloat32)), this, SLOT(SendSpeed(tFloat32)));

	return m_pUiFileWidget;
}

tVoid cCarControllerFilter::ReleaseView()
{
	delete m_pUiFileWidget;
	m_pUiFileWidget = nullptr;
}

tResult cCarControllerFilter::OnIdle()
{
	std::lock_guard<std::mutex> oGuard(m_oMutex);
	RETURN_NOERROR;
}

tResult cCarControllerFilter::OnTimer()
{
	std::lock_guard<std::mutex> oGuard(m_oMutex);
	RETURN_NOERROR;
}

tResult cCarControllerFilter::Init(tInitStage eStage)
{
	RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));
	RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
	RETURN_NOERROR;
}

/* MOVEMENT */

tResult cCarControllerFilter::SendSteering(tFloat32 value)
{
	std::lock_guard<std::mutex> oGuard(m_oMutex);
	transmitSignalValue(m_oOutputSteeringController, m_pClock->GetStreamTime(), m_SignalValueSampleFactory,
	                    m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, value);
	RETURN_NOERROR;
}

tResult cCarControllerFilter::SendSpeed(tFloat32 value)
{
	std::lock_guard<std::mutex> oGuard(m_oMutex);
	transmitSignalValue(m_oOutputSpeedController, m_pClock->GetStreamTime(), m_SignalValueSampleFactory,
	                    m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, value);
	RETURN_NOERROR;
}
