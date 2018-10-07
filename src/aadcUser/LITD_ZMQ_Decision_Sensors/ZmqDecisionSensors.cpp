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

#include "ZmqDecisionSensors.h"

ADTF_PLUGIN(LABEL_LITD_ZMQ_DECISION_SENSORS, cZmqDecisionSensors)

cZmqDecisionSensors::cZmqDecisionSensors()
{
	// input pin names and types
	m_inputs.emplace_back("timer", BoolSignalValue);

	m_inputs.emplace_back("position", Position);
	m_inputs.emplace_back("measured_speed", SignalValue);

	m_inputs.emplace_back("signs", RoadSignExt);
	m_inputs.emplace_back("lidar", LaserScanner);
	m_inputs.emplace_back("ultrasonic", Ultrasonic);
	m_inputs.emplace_back("imu", InerMeasUnitData);

	m_inputs.emplace_back("controller_leverage", PolynomPoint);
	m_inputs.emplace_back("controller_feedback", PolynomPoint);

	m_inputs.emplace_back("siren", BoolSignalValue);
	m_inputs.emplace_back("lidar_break", BoolSignalValue);

	// output pin names and types
	m_outputs.emplace_back("desired_speed", SignalValue);
	m_outputs.emplace_back("trajectories", TrajectoryArray);

	m_outputs.emplace_back("turn_signal_right", BoolSignalValue);
	m_outputs.emplace_back("turn_signal_left", BoolSignalValue);
	m_outputs.emplace_back("hazard_light", BoolSignalValue);
	m_outputs.emplace_back("brake_light", BoolSignalValue);
	m_outputs.emplace_back("reverse_light", BoolSignalValue);

	// pipe out the data whenever there are new samples on these pins
	m_triggers.emplace_back("timer");
}
