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

    adtf::ucom::object_ptr<adtf::streaming::ant::IStreamType> pTypeDefault = adtf::ucom::make_object_ptr<adtf::streaming::ant::cStreamType>(adtf::streaming::ant::stream_meta_type_anonymous());
    create_pin(*this, m_oInputOpenDrive, "open_drive", pTypeDefault);
    m_ShowTrace = tFalse;
    m_ShowLanes = tFalse;

    //Register Properties
    RegisterPropertyVariable("Show Driving Path", m_ShowLanes);
    RegisterPropertyVariable("Show Position Trace", m_ShowTrace);
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

    //Create Open Drive reader and read file
    m_odReader = new ODReader::openDriveReader();


    //Draw map to widget
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
            m_pDisplayWidget->PlotPosition(pixX, pixY, f32heading,m_ShowTrace);
        }
    }

    adtf::ucom::object_ptr<const adtf::streaming::ISample> pSampleAnonymous;
    while (IS_OK(m_oInputOpenDrive.GetNextSample(pSampleAnonymous)))
    {

        adtf::ucom::ant::object_ptr_shared_locked<const adtf::streaming::ant::ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pSampleAnonymous->Lock(pSampleBuffer));

        adtf_util::cString openDriveMapFileString;
        openDriveMapFileString.SetBuffer(pSampleBuffer->GetSize());
        memcpy(openDriveMapFileString.GetBuffer(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());

        if (openDriveMapFileString.GetBufferSize() > 0)
        {
            if (m_odReader->ParseText(openDriveMapFileString.GetBuffer()))
            {
                m_pDisplayWidget->ResetScene();
                ShowMap();
            }
        }
    }
    RETURN_NOERROR;
}


tResult cMapVisualization::ShowMap()
{
    //Check if reader is success
    if(m_odReader->FileReadErr>0)
    {
      LOG_WARNING("File Read Error: %d", m_odReader->FileReadErr);
      RETURN_NOERROR;
    }
    //Check the number of roads
    LOG_INFO("Number of roads: %lu", m_odReader->RoadList.size());
    //Find the minimum and maximum X and y points
    m_minX = 99999999.0f, m_maxX = -99999999.0f;
    m_minY = 99999999.0f, m_maxY = -99999999.0f;
    float m_minZ = 99999999.0f, m_maxZ = -99999999.0f;
    int num = 5;//Number of points
    std::vector<ODReader::Pose3D> mapPoints = m_odReader->GetRoadPoints(m_odReader->RoadList,num);
    std::vector<ODReader::Pose3D> centerPoints = m_odReader->GetRoadPoints(m_odReader->RoadList,num,ODReader::CENTER_POINTS);
    std::vector<ODReader::Pose3D> borderPoints = m_odReader->GetRoadPoints(m_odReader->RoadList,num,ODReader::BORDER_POINTS);
    for (unsigned int i = 0; i < borderPoints.size(); i++)
    {
        float x = borderPoints[i].p.x;
        float y = borderPoints[i].p.y;
        float z = borderPoints[i].p.z;
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
        if (z < m_minZ)
        {
            m_minZ = z;
        }
        else if (z > m_maxZ)
        {
            m_maxZ = z;
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
    LOG_INFO("Min Z %.3f Max Z %.3f", m_minZ, m_maxZ);
    for (unsigned int j = 0; j < mapPoints.size() - 1; j++)
    {
      //Convert map coordinates in -x1 to x2 to Pixel 0 to GRAPHICS_WIDTH
      //Convert map coordinates in y2 to -y1 to Pixel 0 to GRAPHICS_HEIGHT
      //Note Y direction is flipped as map and pixel is opposite in y-direction
      //Pixel coordinates
      float pixX1 = (mapPoints[j].p.x - m_minX)*m_pixelScaleX;
      float pixX2 = (mapPoints[j + 1].p.x - m_minX)*m_pixelScaleX;
      float pixY1 = GRAPHICSSCENE_HEIGHT - (mapPoints[j].p.y - m_minY)*m_pixelScaleY;
      float pixY2 = GRAPHICSSCENE_HEIGHT - (mapPoints[j + 1].p.y - m_minY)*m_pixelScaleY;
      float zScale = 0;
      if( fabs(m_maxZ-m_minZ)>0.01 )
      {
        zScale=255*(mapPoints[j].p.z - m_minZ)/fabs(m_maxZ-m_minZ);
      }
      //Send pixel coordinates to draw
      if( (j+1)%num !=0 && m_ShowLanes)
      {
        m_pDisplayWidget->DrawLine(pixX1, pixY1, pixX2, pixY2,zScale);
      }
    }
    for (unsigned int j = 0; j < centerPoints.size() - 1; j++)
    {
      //Convert map coordinates in -x1 to x2 to Pixel 0 to GRAPHICS_WIDTH
      //Convert map coordinates in y2 to -y1 to Pixel 0 to GRAPHICS_HEIGHT
      //Note Y direction is flipped as map and pixel is opposite in y-direction
      //Pixel coordinates
      float pixX1 = (centerPoints[j].p.x - m_minX)*m_pixelScaleX;
      float pixX2 = (centerPoints[j + 1].p.x - m_minX)*m_pixelScaleX;
      float pixY1 = GRAPHICSSCENE_HEIGHT - (centerPoints[j].p.y - m_minY)*m_pixelScaleY;
      float pixY2 = GRAPHICSSCENE_HEIGHT - (centerPoints[j + 1].p.y - m_minY)*m_pixelScaleY;
      //Send pixel coordinates to draw
      if( (j+1)%num !=0)
      {
        m_pDisplayWidget->DrawLine(pixX1, pixY1, pixX2, pixY2,0,CENTER_LANE);
      }
    }
    for (unsigned int j = 0; j < borderPoints.size() - 1; j++)
    {
      //Convert map coordinates in -x1 to x2 to Pixel 0 to GRAPHICS_WIDTH
      //Convert map coordinates in y2 to -y1 to Pixel 0 to GRAPHICS_HEIGHT
      //Note Y direction is flipped as map and pixel is opposite in y-direction
      //Pixel coordinates
      float pixX1 = (borderPoints[j].p.x - m_minX)*m_pixelScaleX;
      float pixX2 = (borderPoints[j + 1].p.x - m_minX)*m_pixelScaleX;
      float pixY1 = GRAPHICSSCENE_HEIGHT - (borderPoints[j].p.y - m_minY)*m_pixelScaleY;
      float pixY2 = GRAPHICSSCENE_HEIGHT - (borderPoints[j + 1].p.y - m_minY)*m_pixelScaleY;
      float zScale = 0;
      if( fabs(m_maxZ-m_minZ)>0.01 )
      {
        zScale=255*(borderPoints[j].p.z - m_minZ)/fabs(m_maxZ-m_minZ);
      }
      //Send pixel coordinates to draw
      if( (j+1)%num !=0)
      {
        //LOG_INFO("Road %d Point %d Mod %d",m_odReader->RoadList[i].id,j,(j+1)%num);
        //LOG_INFO("x1 %.3f y1 %.3f x2 %.3f y2 %.3f",mapPoints[j].p.x,mapPoints[j].p.y,mapPoints[j+1].p.x,mapPoints[j+1].p.y);
        m_pDisplayWidget->DrawLine(pixX1, pixY1, pixX2, pixY2,zScale,BORDER_LANE);
      }
    }

    RETURN_NOERROR;
}
