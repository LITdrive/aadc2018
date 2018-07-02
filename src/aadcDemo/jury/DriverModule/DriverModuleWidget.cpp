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
#include "DriverModuleWidget.h"
using namespace aadc::jury;

DisplayWidgetDriver::DisplayWidgetDriver(QWidget* parent) : QWidget(parent)
{
    m_pWidget = new QWidget(this);
    m_pWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    // create the elements
    QFont font;
    font.setPointSize(20);


    QLabel *toplabel = new QLabel(this);
    toplabel->setText("AADC - Driver Module");
    toplabel->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
    toplabel->setFont(font);

    QHBoxLayout *hboxManeuverList = new QHBoxLayout();
    m_comboBox = new QComboBox(this);
    m_comboBox->setMaximumWidth(150);

    hboxManeuverList->addWidget(new QLabel(QString("Maneuver Index")));
    hboxManeuverList->addWidget(m_comboBox);


    m_pSendStartupButton = new QPushButton(this);
    m_pSendStartupButton->setText("Startup");
    m_pSendStartupButton->setFixedWidth(100);

    m_pSendReadyResponseButton = new QPushButton(this);
    m_pSendReadyResponseButton->setText("Ready to Start");
    m_pSendReadyResponseButton->setFixedWidth(100);

    m_pSendStateErrorButton = new QPushButton(this);
    m_pSendStateErrorButton->setText("Error");
    m_pSendStateErrorButton->setFixedWidth(100);

    m_pSendStateRunButton = new QPushButton(this);
    m_pSendStateRunButton->setText("Running");
    m_pSendStateRunButton->setFixedWidth(100);

    m_pSendStateCompleteButton = new QPushButton(this);
    m_pSendStateCompleteButton->setText("Complete");
    m_pSendStateCompleteButton->setFixedWidth(100);

    m_logField = new QTextEdit(this);
    m_logField->setReadOnly(true);
    m_logField->setFixedSize(240,180);

    //create the controls for the sections

    QVBoxLayout *vboxManeuverControl = new QVBoxLayout();
    vboxManeuverControl->addLayout(hboxManeuverList);
    vboxManeuverControl->addWidget(m_pSendStartupButton,0, Qt::AlignCenter);
    vboxManeuverControl->addWidget(m_pSendReadyResponseButton,0, Qt::AlignCenter);
    vboxManeuverControl->addWidget(m_pSendStateRunButton,0, Qt::AlignCenter);
    vboxManeuverControl->addWidget(m_pSendStateErrorButton,0, Qt::AlignCenter);
    vboxManeuverControl->addWidget(m_pSendStateCompleteButton,0, Qt::AlignCenter);

    m_pManeuverGroupBox = new QGroupBox(tr("Maneuver Control"),this);
    m_pManeuverGroupBox->setLayout(vboxManeuverControl);
    m_pManeuverGroupBox->setFixedWidth(280);

    QLabel *jurylabel = new QLabel(this);
    jurylabel->setText("Jury Module:");
    jurylabel->setAlignment(Qt::AlignCenter);

    m_JuryInfo = new QLabel(this);
    m_JuryInfo->setFixedSize(100,30);
    m_JuryInfo->setAutoFillBackground(true);
    m_JuryInfo->setAlignment(Qt::AlignCenter);

    QHBoxLayout *vboxJuryInfo = new QHBoxLayout();
    vboxJuryInfo->addWidget(jurylabel);
    vboxJuryInfo->addWidget(m_JuryInfo);


    // set the main layout
    m_mainLayout = new QVBoxLayout();
    m_mainLayout->addWidget(toplabel,0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_pManeuverGroupBox, 0,Qt::AlignCenter);
    m_mainLayout->addLayout(vboxJuryInfo);
    m_mainLayout->addWidget(m_logField,0, Qt::AlignCenter);
    setLayout(m_mainLayout);


    //connect the buttons to the slots
    connect(m_pSendStateErrorButton,  SIGNAL(clicked()), this, SLOT(OnStateErrorClicked()));
    connect(m_pSendStateRunButton, SIGNAL(clicked()), this, SLOT(OnStateRunClicked()));
    connect(m_pSendStartupButton, SIGNAL(clicked()), this, SLOT(OnStartupClicked()));
    connect(m_pSendReadyResponseButton, SIGNAL(clicked()), this, SLOT(OnResponseReadyClicked()));
    connect(m_pSendStateCompleteButton, SIGNAL(clicked()), this, SLOT(OnStateCompleteClicked()));
}


void DisplayWidgetDriver::OnStateRunClicked()
{
    emit sendStruct(statecar_running,tInt16(m_comboBox->currentIndex()));
}

void DisplayWidgetDriver::OnStateErrorClicked()
{
    emit sendStruct(statecar_error,tInt16(m_comboBox->currentIndex()));
}

void DisplayWidgetDriver::OnResponseReadyClicked()
{
    emit sendStruct(statecar_ready,tInt16(m_comboBox->currentIndex()));
}

void DisplayWidgetDriver::OnStateCompleteClicked()
{
    emit sendStruct(statecar_complete,tInt16(m_comboBox->currentIndex()));
}

void DisplayWidgetDriver::OnStartupClicked()
{
    emit sendStruct(statecar_startup,tInt16(m_comboBox->currentIndex()));
}


void DisplayWidgetDriver::OnDriverGo(int entryId)
{
    m_JuryInfo->setStyleSheet("background-color: rgb(0,255,0);");
    m_JuryInfo->setText(QString("Start ") + QString::number(entryId));
}

void DisplayWidgetDriver::OnDriverStop(int entryId)
{
    m_JuryInfo->setStyleSheet("background-color: rgb(255,0,0);");
    m_JuryInfo->setText(QString("Stop ") + QString::number(entryId));;
}

void DisplayWidgetDriver::OnDriverRequestReady(int entryId)
{
    m_JuryInfo->setStyleSheet("background-color: rgb(255,255,0);");
    m_JuryInfo->setText(QString("Request Ready ") + QString::number(entryId) + QString(" ?"));
}

void DisplayWidgetDriver::OnAppendText(QString text)
{
    m_logField->append(text);
}

void DisplayWidgetDriver::SetManeuverList(std::vector<tSector>& sectorList)
{
    m_sectorList = sectorList;
}

void DisplayWidgetDriver::EnableManeuverGroupBox(bool bEnable)
{
    m_pManeuverGroupBox->setEnabled(bEnable);
}

void DisplayWidgetDriver::ShowManeuverList()
{
    for(unsigned int i = 0; i < m_sectorList.size(); i++)
    {
        OnAppendText(QString(cString::Format("Driver Module: Sector ID %d",m_sectorList[i].id).GetPtr()));

        for(unsigned int j = 0; j < m_sectorList[i].sector.size(); j++)
        {
            OnAppendText(QString(cString::Format("    Maneuver ID: %d", m_sectorList[i].sector.at(j).id).GetPtr()));
            OnAppendText(QString(cString::Format("    Maneuver action: %s", maneuverToString(m_sectorList[i].sector.at(j).action).c_str()).GetPtr()));
        }
    }
}

void DisplayWidgetDriver::FillComboBox()
{
    m_comboBox->clear();

    int lastActionId = 0;
    std::vector<cString> vecManeuverNames;

    //get the last action id
    for(unsigned int i = 0; i < m_sectorList.size(); i++)
    {
        for(unsigned int j = 0; j < m_sectorList[i].sector.size(); j++)
        {
            lastActionId++;
            vecManeuverNames.push_back(maneuverToString(m_sectorList[i].sector[j].action).c_str());
        }

    }

    for(int i = 0; i < lastActionId; i++)
    {
        m_comboBox->addItem(QString::number(i) + QString(" - ") + QString(vecManeuverNames[i]));
    }
}





