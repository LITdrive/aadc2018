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

#include "LIT_CarControllerWidget.h"

cCarControllerWidget::cCarControllerWidget(QWidget *parent) : QWidget(parent), m_ui(new Ui_CarControllerUi)
{
    m_ui->setupUi(this);

    // Rearrange buttons id 0...5 for lights
    m_ui->buttonGroup_RC->setId(m_ui->buttonGroup_RC->buttons().at(0), 0); // Head
    m_ui->buttonGroup_RC->setId(m_ui->buttonGroup_RC->buttons().at(1), 1); // Brake
    
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

}

void cCarControllerWidget::setSteering(int value)
{

}

void cCarControllerWidget::keyPressEvent(QKeyEvent* event) {
    if (!event->isAutoRepeat()) {
        emit keyReceived((int)event->key());

        int key = event->key();
        if (key == 16777235) {
            // UP
            emit sendpressUp();
            LOG_INFO("KeyboardRemote: Up Press");
        } else if (key == 16777234) {
            // LEFT
            emit sendpressLeft();
            LOG_INFO("KeyboardRemote: Left Press");
        } else if (key == 16777236) {
            // Right
            emit sendpressRight();
            LOG_INFO("KeyboardRemote: Right Press");
        } else if (key == 16777237) {
            // DOWN
            emit sendpressDown();
            LOG_INFO("KeyboardRemote: Down Press");
        }
    }
}

void cCarControllerWidget::keyReleaseEvent(QKeyEvent* event) {
    if (!event->isAutoRepeat()) {
        //        emit keyReceived((int)event->key());

        int key = event->key();
        if (key == 16777235) {
            // UP
            emit sendreleaseUp();
            LOG_INFO("KeyboardRemote: Up Realease");
        } else if (key == 16777234) {
            // LEFT
            emit sendreleaseLeft();
            LOG_INFO("KeyboardRemote: Left Release");
        } else if (key == 16777236) {
            // Right
            emit sendreleaseRight();
            LOG_INFO("KeyboardRemote: Right Release");
        } else if (key == 16777237) {
            // DOWN
            emit sendreleaseDown();
            LOG_INFO("KeyboardRemote: Down Release");
        }
    }
}

QButtonGroup* cCarControllerWidget::getRCButtonGroup()
{
    return m_ui->buttonGroup_RC;
}

void cCarControllerWidget::update()
{

}

void cCarControllerWidget::mousePressEvent(QMouseEvent* event) {
    // printf("\nMouse in board");
    setFocus();
}

void cCarControllerWidget::focusOutEvent(QFocusEvent* event){
    emit sendreleaseRight(); // reset steering = 0
    emit sendreleaseDown(); // reset speed = 0
}
