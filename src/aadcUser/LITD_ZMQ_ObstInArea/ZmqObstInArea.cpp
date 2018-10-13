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

#include "ZmqObstInArea.h"

ADTF_PLUGIN(LABEL_LITD_ZMQ_Obst_InArea, cZmqObstInArea)

cZmqObstInArea::cZmqObstInArea()
{
	// input pin names and types
	m_inputs.emplace_back("Lidar", LaserScanner);
	m_inputs.emplace_back("ultrasonic", Ultrasonic);
	m_inputs.emplace_back("position", Position);

	// output pin names and types #adjust based on Areas
	m_outputs.emplace_back("ObstInArea0", SignalValue);
	m_outputs.emplace_back("ObstInArea1", SignalValue);
	m_outputs.emplace_back("ObstInArea2", SignalValue);
	m_outputs.emplace_back("ObstInArea3", SignalValue);
	m_outputs.emplace_back("ObstInArea4", SignalValue);
	m_outputs.emplace_back("ObstInArea5", SignalValue);
	m_outputs.emplace_back("ObstInArea6", SignalValue);
	m_outputs.emplace_back("ObstInArea7", SignalValue);
	m_outputs.emplace_back("ObstInArea8", SignalValue);
	m_outputs.emplace_back("ObstInArea9", SignalValue);
	m_outputs.emplace_back("ObstInArea10", SignalValue);
	m_outputs.emplace_back("ObstInArea11", SignalValue);
	m_outputs.emplace_back("ObstInArea12", SignalValue);
	m_outputs.emplace_back("ObstInArea13", SignalValue);
	m_outputs.emplace_back("ObstInArea14", SignalValue);
	m_outputs.emplace_back("ObstInArea15", SignalValue);
	m_outputs.emplace_back("ObstInArea16", SignalValue);
	m_outputs.emplace_back("ObstInArea17", SignalValue);
	m_outputs.emplace_back("ObstInArea18", SignalValue);
	m_outputs.emplace_back("ObstInArea19", SignalValue);

	// pipe out the data whenever there are new samples on these pins
	m_triggers.emplace_back("Lidar");
}
