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

#include "mapReader/openDriveReader.h"

#ifndef _MAP_VISUALIZATION_FILTER_HEADER_
#define _MAP_VISUALIZATION_FILTER_HEADER_

#define CID_MAP_VISUALIZATION_FILTER  "map_visualization.filter.base.aadc.cid"

/*! this is the main class for the map visualization. */
class cMapVisualization : public adtf::ui::cQtUIFilter, QObject
{
public:
    ADTF_CLASS_ID_NAME(cMapVisualization, CID_MAP_VISUALIZATION_FILTER, "Map Visualization");
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::ui::IQtXSystem));

private:

    /*! The invalidated */
    std::atomic_bool  m_bInvalidated;


    /*! Scale for map in x and y direction. */
    tFloat32 m_pixelScaleX;
    /*! The pixel scale y coordinate */
    tFloat32 m_pixelScaleY;
    //Minimum and maximum map coordinates
    tFloat32 m_minX, m_minY, m_maxX, m_maxY;
    /*! The ddl indices for a tPosition */
    struct
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    }  m_ddlPositionIndex;


    /*! The reader position */
    adtf::filter::cPinReader m_oReaderPos;
    /*! sample reader List of input maneuvers */
    adtf::filter::cPinReader     m_oInputOpenDrive;
    /*! The position sample factory */
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;

    /*! The display widget */
    DisplayWidget* m_pDisplayWidget;
    /*! The od reader */
    ODReader::openDriveReader *m_odReader;




    /*! The show lanes */
    adtf::base::property_variable<tBool> m_ShowLanes;

    /*! The show trace */
    adtf::base::property_variable<tBool> m_ShowTrace;


public:
    /*! Default constructor. */
    cMapVisualization();
    /*! Destructor. */
    virtual ~cMapVisualization();

protected:
    QWidget* CreateView() override;
    tVoid    ReleaseView() override;
    tResult  OnTimer() override;
    tResult  Init(tInitStage eStage)override;

private:

    /*!
     * Shows the map.
     *
     * \return  Standard Result Code.
     */
    tResult ShowMap();


signals:

    /*!
     * Sends a map data.
     *
     * \param   x1  The first x value.
     * \param   y1  The first y value.
     * \param   x2  The second x value.
     * \param   y2  The second y value.
     */
    void SendMapData(float x1, float y1, float x2, float y2);


};

#endif //_ADTF_MEDIA_DESCDISP_FILTER_CLASS_HEADER_
