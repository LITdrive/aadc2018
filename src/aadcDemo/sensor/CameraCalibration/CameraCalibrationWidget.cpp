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
#include "CameraCalibrationWidget.h"
#include "calibrationSettings.h"

cCameraCalibrationWidget::cCameraCalibrationWidget(QWidget* parent) : QWidget(parent)
{
    m_pWidget = new QWidget(this);
    m_pWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    m_lblVideoPixmap = new QLabel();
    m_lblVideoPixmap->setFixedSize(320, 240);

    m_btSaveAs = new QPushButton(this);
    m_btSaveAs->setText("Save Calibration File");
    m_btSaveAs->setFixedSize(200, 50);

    m_btStart = new QPushButton(this);
    m_btStart->setText("Start Calibration");
    m_btStart->setFixedSize(200, 50);

    m_btStartFisheye = new QPushButton(this);
    m_btStartFisheye->setText("Start Calibration Fisheye");
    m_btStartFisheye->setFixedSize(200, 50);

    m_btCancel = new QPushButton(this);
    m_btCancel->setText("Cancel Calibration");
    m_btCancel->setFixedSize(200, 50);

    m_lblStatus = new QLabel();
    m_lblStatus->setFixedSize(200, 50);
    m_lblStatus->setText("Waiting for action");

    m_lblMessage = new QLabel();
    m_lblMessage->setFixedSize(200, 50);


    QVBoxLayout *buttonsLayout = new QVBoxLayout();
    buttonsLayout->addWidget(m_lblStatus);
    buttonsLayout->addWidget(m_lblMessage);
    buttonsLayout->addWidget(m_btStart);
    buttonsLayout->addWidget(m_btStartFisheye);
    buttonsLayout->addWidget(m_btSaveAs);
    buttonsLayout->addWidget(m_btCancel);


    m_mainLayout = new QHBoxLayout();
    m_mainLayout->addWidget(m_lblVideoPixmap, 0, Qt::AlignCenter);
    m_mainLayout->addLayout(buttonsLayout, 0);
    setLayout(m_mainLayout);

    connect(m_btSaveAs, SIGNAL(clicked()), this, SLOT(OnSaveAs()));

    m_btSaveAs->setEnabled(false);
    m_btCancel->setEnabled(false);
}


void cCameraCalibrationWidget::OnNewImage(const QImage &newImage)
{
    m_lblVideoPixmap->setPixmap(QPixmap::fromImage(newImage));
}

void cCameraCalibrationWidget::OnSaveAs()
{
    QString filenameToSaveTo = QFileDialog::getSaveFileName(this, tr("Save Calibration As.."), "camara_intr_calibration.yml");
    emit SendSaveAs(filenameToSaveTo);
}

void cCameraCalibrationWidget::OnSetState(int state)
{
    switch (state)
    {
        case CAPTURING_FINISHED:
            m_btSaveAs->setEnabled(true);
            m_btStart->setEnabled(true);
            m_btCancel->setEnabled(false);
            m_lblStatus->setText("Capturing Finished");
            break;
        case WAITING:
            m_btSaveAs->setEnabled(false);
            m_btStart->setEnabled(true);
            m_lblStatus->setText("Waiting for action");
            m_btCancel->setEnabled(false);
            break;
        case CAPTURING:
            m_btSaveAs->setEnabled(false);
            m_btStart->setEnabled(false);
            m_btCancel->setEnabled(true);
            m_lblStatus->setText("Capturing Chessboard");
            break;
        default:
            break;
    }

}

void cCameraCalibrationWidget::OnSendMessage(cString message)
{
    m_lblMessage->setText(QString(message));
}