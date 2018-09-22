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

#include "stdafx.h"

#define CID_MICROPHONE_VISUALIZATION  "audio_visualization.filter.demo.aadc.cid"

/*! the main class for the car controller filter. */
class cAudioVisualization : virtual public cQtUIFilter
{

public:
    ADTF_CLASS_ID_NAME(cAudioVisualization, CID_MICROPHONE_VISUALIZATION, "Audio Visualization");
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::ui::ant::IQtXSystem),
        REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

    /*! The output speed controller */
    cPinReader m_inAudio;

    /*! The chart view */
    QChartView* m_chartView;

    /*! The series used in qchart */
    QLineSeries* m_series;

    /*! The buffer for audio samples */
    QVector<QPointF> m_buffer;

    /*! The nr samples to display */
    const int m_nrSamplesToDisplay = 2000;

    /*! The codec package size in bytes */
    const int m_codecPackageSizeInBytes = 4;

public:

    /*! Default constructor. */
    cAudioVisualization();

    /*! Destructor. */
    virtual ~cAudioVisualization();

protected: // Implement cBaseQtFilter
    QWidget * CreateView() override;
    tVoid    ReleaseView() override;
    tResult  OnTimer() override;
    tResult  Init(tInitStage eStage) override;
};