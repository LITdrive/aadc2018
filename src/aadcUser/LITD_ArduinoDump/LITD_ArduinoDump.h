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

#define CID_ARDUINO_DUMP_FILTER "litd_arduino_dump.filter.user.aadc.cid"


using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class cArduinoDump : public cTriggerFunction
{
private:

	property_variable<cFilename> m_dump_path_left = cFilename("../../../../data/wheel_left.csv");
	property_variable<cFilename> m_dump_path_right = cFilename("../../../../data/wheel_right.csv");

    cPinReader m_oInputWheelLeft;
    cPinReader m_oInputWheelRight;

    std::ofstream* m_dumpWriterLeft = nullptr;
    std::ofstream* m_dumpWriterRight = nullptr;

    struct
    {
        tSize ArduinoTimestamp;
        tSize WheelTach;
        tSize WheelDir;
    } m_ddlWheelDataIndex;

    adtf::mediadescription::cSampleCodecFactory m_WheelDataSampleFactory;

public:

	cArduinoDump();

    virtual ~cArduinoDump() = default;

    tResult Configure() override;
    
    tResult Process(tTimeStamp tmTimeOfTrigger) override;

};


//*************************************************************************************************
