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

#include "stdafx.h"

DisplayWidget::DisplayWidget(QWidget* pParent) :
    QWidget(pParent), m_qPCarCenter_x(GRAPHICSSCENE_WIDTH), m_qPCarCenter_y(GRAPHICSSCENE_HEIGHT)
{
  // initialize the main widget
  m_pWidget = new QWidget(this);
  m_pWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  m_pWidget->setFixedSize(GRAPHICSSCENE_WIDTH,GRAPHICSSCENE_HEIGHT);

  //initialize the fonts
  m_mainFont = new QFont("Arial",12);
  //Set Font
  m_mainFontSmall = new QFont("Arial",10);
  setFont(*m_mainFont);
  // create new QGraphicsScene
  scene=new QGraphicsScene(this);
  //Set Backgroud Coloe
  scene->setBackgroundBrush(QColor(43,43,43));
  //Create New view
  view= new QGraphicsView(scene,this);
  //Create Marker Small Cirle
  marker1=scene->addEllipse(0,0,0,0,QPen(QColor(0,0,0)),QBrush(QColor(0,0,0)));
  //Create Marker Bight circle
  marker2=scene->addEllipse(0,0,0,0,QPen(QColor(0,0,0)),QBrush(QColor(0,0,0)));
  text=new QGraphicsTextItem("");
  scene->addItem(text);
  //Add view layout
  view->setMaximumSize(600,800);
  //Map scene to view
  view->mapToScene(0,0);
  //Create View
  m_mainLayout = new QVBoxLayout();
  //Add view to the layout
  m_mainLayout->addWidget(view);
  setLayout(m_mainLayout);
  ResetScene();
}

/**
 * Destructor.
 * Mind: A destructor within adtf should always be virtual.
 */
DisplayWidget::~DisplayWidget()
{
}

void DisplayWidget::ResetScene()
{
  scene->clear();
  //Create Small Circle for Position
  pos=scene->addEllipse(0,0,0,0,QPen(QColor(0,0,0)),QBrush(QColor(0,0,0)));
  //Create Big Circle for Position
  pos1=scene->addEllipse(400,400,0,0,QPen(QColor(0,0,0)),QBrush(QColor(0,0,0)));
  //Create heading line
  head=scene->addLine(0,0,0,0);
  return;
}


//Plot Map points
void DisplayWidget::DrawLine(float x1, float y1, float x2, float y2,float zScale,RoadType rType)
{
  //Create Right Lane
  QGraphicsLineItem *line1=scene->addLine(x1,y1,x2,y2);
  QPen pen1;
  pen1.setWidth(1);
  if(rType == BORDER_LANE)
  {
    pen1.setColor(QColor(zScale,255-zScale,100));
  }
  else if (rType == CENTER_LANE)
  {
    pen1.setStyle(Qt::DashDotLine);
    pen1.setColor(QColor(255,255,255));
  }
  else{
    pen1.setColor(QColor(zScale,255-zScale,190));
  }
  line1->setPen(pen1);
  //Update Scene
  scene->update();
}

//Position Date Received Create Position
void DisplayWidget::PlotPosition(float x1, float y1, float h1,bool showTrace)
{
  //x1=m_scalex*x1;
  //y1=m_ymax-m_scaley*y1;
  scene->removeItem(pos);
  scene->removeItem(pos1);
  scene->removeItem(head);

  //Add Position Circles
  pos1=scene->addEllipse(x1-12.5,y1-12.5,25,25,QPen(QColor(0,0,0)),QBrush(QColor(255,0,0,50)));
  pos=scene->addEllipse(x1-5,y1-5,10,10,QPen(QColor(0,0,0)),QBrush(QColor(247,0,0)));
  if(showTrace)
  {
    scene->addEllipse(x1-2.5,y1-2.5,5,5,QPen(QColor(0,0,0)),QBrush(QColor(76,247,202)));
  }

  //Add Heading Line
  head=scene->addLine(x1,y1,x1+cos(h1)*10,y1-sin(h1)*10);
  //Set Heading Line Color
  QPen pen2;
  pen2.setWidth(1.5);
  pen2.setColor(QColor(255,255,255));
  head->setPen(pen2);

  //Update Scene
  scene->update();
}
