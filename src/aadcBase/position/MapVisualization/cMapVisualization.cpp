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

ADTF_PLUGIN(CID_MAP_VISUALIZATION_FILTER, cMapVisualization)


cMapVisualization::cMapVisualization() : adtf::ui::cQtUIFilter(), m_pDisplayWidget(0)
{
    //Register OpenDrive file location property
    RegisterPropertyVariable("OpenDrive Map File", m_mapFile);
    //Add position pin from mediadescription
    adtf::ucom::object_ptr<adtf::streaming::IStreamType> pTypePositionData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_PositionSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32x", m_ddlPositionIndex.x);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32y", m_ddlPositionIndex.y);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32radius", m_ddlPositionIndex.radius);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32speed", m_ddlPositionIndex.speed);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32heading", m_ddlPositionIndex.heading);
    }
    else
    {
        LOG_WARNING("No mediadescription for tPosition found!");
    }
    //Create Pin
    adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pConstTypePositionData = pTypePositionData;
    adtf::streaming::create_pin(*this, m_oReaderPos, "position", pConstTypePositionData);
}

cMapVisualization::~cMapVisualization()
{
}

tResult cMapVisualization::Init(tInitStage eStage)
{
    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));
    if (eStage == StageFirst)
    {
    }
    RETURN_NOERROR;
}

QWidget* cMapVisualization::CreateView()
{
    //Create Widget
    m_pDisplayWidget = new DisplayWidget(nullptr);
    //Draw map to widget
    ShowMap(m_mapFile);
    return m_pDisplayWidget;
}

tVoid cMapVisualization::ReleaseView()
{
    m_pDisplayWidget = nullptr;
}

tResult cMapVisualization::OnTimer()
{
    //Read Position values
    tFloat32 f32x, f32y, f32heading;
    //Get Sample
    adtf::ucom::object_ptr<const adtf::streaming::ISample> pReadSample;
    while (IS_OK(m_oReaderPos.GetNextSample(pReadSample)))
    {
        //Get Position values
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(pReadSample);
        f32x = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.x);
        f32y = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.y);
        f32heading = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.heading);
        //Convert to Pixel Coordinates
        tFloat32 pixX = m_pixelScaleX *(f32x - m_minX);
        tFloat32 pixY = GRAPHICSSCENE_HEIGHT - m_pixelScaleY * (f32y - m_minY);
        //Plot if within the graphicsscene
        if (pixX > 0 && pixX < GRAPHICSSCENE_WIDTH && pixY>0 && pixY < GRAPHICSSCENE_HEIGHT)
        {
            m_pDisplayWidget->PlotPosition(pixX, pixY, f32heading);
        }
    }
    RETURN_NOERROR;
}


tResult cMapVisualization::ShowMap(adtf_util::cFilename fileMap)
{
    //Create Absolute path and convert to string
    fileMap = fileMap.CreateAbsolutePath(".");
    std::string file = (std::string)fileMap;
    //Display file name
    LOG_INFO("File %s", file.c_str());
    //Create Open Drive reader and read file
    m_odReader = new ODReader::openDriveReader(file);
    //Check if reader is success
    LOG_INFO("File Read Error: %d", m_odReader->FileReadErr);
    //Check the number of roads
    LOG_INFO("Number of roads: %lu", m_odReader->RoadList.size());
    //Find the minimum and maximum X and y points
    m_minX = 99999999.f, m_maxX = -99999999.f;
    m_minY = 99999999.f, m_maxY = -99999999.f;
    for (unsigned int i = 0; i < m_odReader->MapPoints.size(); i++)
    {
        float x = m_odReader->MapPoints[i].p.x;
        float y = m_odReader->MapPoints[i].p.y;
        if (x < m_minX)
        {
            m_minX = x;
        }
        else if (x > m_maxX)
        {
            m_maxX = x;
        }
        if (y < m_minY)
        {
            m_minY = y;
        }
        else if (y > m_maxY)
        {
            m_maxY = y;
        }
    }
    //Extend the map on either side for displaying points outside the map
    m_minX -= 0.3f;
    m_minY -= 0.3f;
    m_maxX += 0.3f;
    m_maxY += 0.3f;
    //Calculate scale from the fixed width and height
    m_pixelScaleX = GRAPHICSSCENE_WIDTH / fabs(m_maxX - m_minX);
    m_pixelScaleY = GRAPHICSSCENE_HEIGHT / fabs(m_maxY - m_minY);
    //Use the minimum scale of the two
    if (m_pixelScaleX < m_pixelScaleY)
    {
        m_pixelScaleY = m_pixelScaleX;
    }
    else {
        m_pixelScaleX = m_pixelScaleY;
    }
    LOG_INFO("Min x %.3f Max X %.3f Scale X %.3f", m_minX, m_maxX, m_pixelScaleX);
    LOG_INFO("Min Y %.3f Max Y %.3f Scale Y %.3f", m_minY, m_maxY, m_pixelScaleY);
    //Draw individual roads one by one
    for (unsigned int i = 0; i < m_odReader->RoadList.size(); i++)
    {
        //Get the points from each road
        std::vector<ODReader::Pose3D> path = m_odReader->GetRoadPoints(m_odReader->RoadList[i]);
        //Draw line for each point
        for (unsigned int j = 0; j < path.size() - 1; j++)
        {//
          //Convert map coordinates in -x1 to x2 to Pixel 0 to GRAPHICS_WIDTH
          //Convert map coordinates in y2 to -y1 to Pixel 0 to GRAPHICS_HEIGHT
          //Note Y direction is flipped as map and pixel is opposite in y-direction
          //Pixel coordinates
            float pixX1 = (path[j].p.x - m_minX)*m_pixelScaleX;
            float pixX2 = (path[j + 1].p.x - m_minX)*m_pixelScaleX;
            float pixY1 = GRAPHICSSCENE_HEIGHT - (path[j].p.y - m_minY)*m_pixelScaleY;
            float pixY2 = GRAPHICSSCENE_HEIGHT - (path[j + 1].p.y - m_minY)*m_pixelScaleY;
            //Send pixel coordinates to draw
            m_pDisplayWidget->DrawLine(pixX1, pixY1, pixX2, pixY2);
        }
    }
    RETURN_NOERROR;
}
