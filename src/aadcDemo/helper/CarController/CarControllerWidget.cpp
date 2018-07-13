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

#include "CarControllerWidget.h"

cCarControllerWidget::cCarControllerWidget(QWidget *parent) : QWidget(parent), m_ui(new Ui_CarControllerUi)
{
    m_ui->setupUi(this);

    // Rearrange buttons id 0...5 for lights
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(1), 0); // Head
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(2), 1); // Brake
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(4), 2); // Reverse
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(5), 3); // Hazard
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(3), 4); // TurnLeft
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(0), 5); // TurnRight
    
    m_timer.setInterval(50);
    connect(&m_timer, SIGNAL(timeout()), this, SLOT(update()));
    m_timer.start();
}


cCarControllerWidget::~cCarControllerWidget()
{
    delete m_ui;
}

void cCarControllerWidget::setSpeed(int value)
{
    m_ui->lcdNumber_throttle->display(value);
    emit throttleReceived((float) value);
}

void cCarControllerWidget::setSteering(int value)
{
    m_ui->lcdNumber_steering->display(value);
    emit steeringReceived(value);
}

QButtonGroup* cCarControllerWidget::getLightButtonGroup()
{
    return m_ui->buttonGroup_lights;
}

void cCarControllerWidget::update()
{
    if (m_ui->radioButton_enable_resetting_slider->isChecked())
    {
        int steering = m_ui->horizontalSlider_steering->value();
        int speed = m_ui->verticalSlider_throttle->value();

        if (steering > 0)
            steering--;
        if (steering < 0)
            steering++;

        if (speed > 0)
            speed--;
        if (speed < 0)
            speed++;

        m_ui->horizontalSlider_steering->setValue(steering);
        m_ui->verticalSlider_throttle->setValue(speed);
    }

    setSteering(m_ui->horizontalSlider_steering->value());
    setSpeed(m_ui->verticalSlider_throttle->value());
}