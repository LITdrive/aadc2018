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


/*********************************************************************
* This code was provided by HERE
*
* *******************************************************************/


#ifndef _DISPLAY_WIDGET_
#define _DISPLAY_WIDGET_

#define GRAPHICSSCENE_WIDTH 500
#define GRAPHICSSCENE_HEIGHT 500

enum RoadType{
  CENTER_LANE =0,
  DRIVING_LANE = 1,
  BORDER_LANE = 2
};
class DisplayWidget : public QWidget
{
    public:
        DisplayWidget(QWidget* pParent);
        virtual ~DisplayWidget();
        void ResetScene();
        void DrawLine(float x1, float y1, float x2, float y2,float zScale,RoadType rType=DRIVING_LANE);
        void PlotPosition(float x, float y,float h,bool showTrace = false);

      private:

          /*! the main widget */
          QWidget* m_pWidget;

          /*! the main font for the widget */
          QFont* m_mainFont;

          /*! the smaller main font for the tableviews etc */
          QFont* m_mainFontSmall;

          /*! the x coordinate of the car in the graphicsscene */
          const qreal m_qPCarCenter_x;

          /*! the y coordinate of the car in the graphicsscene */
          const qreal m_qPCarCenter_y;

          QGraphicsScene* scene;
          /*! the main layout for the widget*/
          QVBoxLayout *m_mainLayout;

          //Graphics View for widget
          QGraphicsView* view;

          //Circles for Position
          QGraphicsEllipseItem *pos,*pos1;

          //Circles for Marker
          QGraphicsEllipseItem *marker1,*marker2;

          //Heading Line
          QGraphicsLineItem *head;

          //Text for Marker
          QGraphicsTextItem *text;

          QGraphicsSceneWheelEvent *wheel;
};

#endif //_ADTF_QT_VIDEO_WIDGET_CLASS_HEADER_
