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

#include "AudioVisualization.h"


//*************************************************************************************************

ADTF_PLUGIN("Audio Visualization Plugin", cAudioVisualization)

cAudioVisualization::cAudioVisualization() : m_chartView(nullptr), m_series(nullptr)
{
    object_ptr<IStreamType> pType = make_object_ptr<cStreamType>(stream_meta_type_audio());

    create_pin(*this, m_inAudio, "audio", pType);
}


cAudioVisualization::~cAudioVisualization()
{
}

QWidget* cAudioVisualization::CreateView()
{
    m_series = new QLineSeries();

    QChart *chart = new QChart();
    chart->legend()->hide();
    chart->addSeries(m_series);
    chart->createDefaultAxes();
    chart->setTitle("Audio Input");

    QValueAxis* axisX = new QValueAxis;
    axisX->setLabelFormat("%g");
    axisX->setTitleText("Samples");
    axisX->setRange(0, m_nrSamplesToDisplay);

    QValueAxis *axisY = new QValueAxis;
    axisY->setRange(-1, 1);
    axisY->setTitleText("Audio level");

    chart->setAxisX(axisX, m_series);
    chart->setAxisY(axisY, m_series);

    m_chartView = new QChartView(chart);
    m_chartView->setRenderHint(QPainter::Antialiasing);

    return m_chartView;
}

tVoid cAudioVisualization::ReleaseView()
{
    delete m_chartView;
    m_chartView = nullptr;
    m_series = nullptr;
}


tResult cAudioVisualization::OnTimer()
{

    //this will empty the Reader queue and return the last sample received.
    //If no sample was during the time between the last execution of Process the "old sample" is return.
    object_ptr<const ISample> pSampleAudio;

    while (IS_OK(m_inAudio.GetNextSample(pSampleAudio)))
    {
        adtf::ucom::object_ptr_shared_locked<const adtf::streaming::ISampleBuffer> pBuffer;
        pSampleAudio->Lock(pBuffer);
        //get number of audio samples in media samples
        const int availableSamples = int(pBuffer->GetSize()) / m_codecPackageSizeInBytes;

        //if we have nothing displayed yet
        if (m_buffer.isEmpty())
        {
            m_buffer.reserve(m_nrSamplesToDisplay);
            for (int i = 0; i < m_nrSamplesToDisplay; ++i)
                m_buffer.append(QPointF(i, 0));
        }

        //find start index in current buffer
        int start = 0;
        if (availableSamples < m_nrSamplesToDisplay)
        {
            start = m_nrSamplesToDisplay - availableSamples;
            for (int s = 0; s < start; ++s)
                m_buffer[s].setY(m_buffer.at(s + availableSamples).y()); // shift y values by size availableSamples 
        }

        //copy data to local variabe to get non-const char pointer
        std::vector<uchar> audioData;
        audioData.resize(pBuffer->GetSize() / sizeof(uchar));
        memcpy(audioData.data(), pBuffer->GetPtr(), audioData.size());

        uchar* data = static_cast<uchar*>(audioData.data());

        for (int s = start; s < m_nrSamplesToDisplay; ++s, data += m_codecPackageSizeInBytes)
        {
            m_buffer[s].setY(qreal(*data - 128) / qreal(128));
        }

        //update data for ui
        m_series->replace(m_buffer);

        pBuffer->Unlock();


    }

    RETURN_NOERROR;
}

tResult cAudioVisualization::Init(tInitStage eStage)
{
    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));

    RETURN_NOERROR;
}

