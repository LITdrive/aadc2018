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

#include "SensorVisualizationWidget.h"

QString format_number(int distance)
{
    std::stringstream ss;
    if (distance >= 0)
        ss << std::setw(4) << std::setfill(' ') << std::right << distance;
    else
        ss << "-" << std::setw(3) << std::setfill(' ') << abs(distance);

    return QString::fromStdString(ss.str());
}

cSensorVisualizationWidget::cSensorVisualizationWidget(QWidget *parent) : QWidget(parent), ui(new Ui_SensorVisualizationUi)
{
    ui->setupUi(this);
    memset(&m_usData, 0, sizeof(tUltrasonicStructSimple));
    memset(&m_imuData, 0, sizeof(tInerMeasUnitData));
    memset(&m_wheelData, 0, sizeof(tWheelDataSimple));
    memset(&m_batteryData, 0, sizeof(tVoltageStructSimple));
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update_gui()));

    connect(ui->doubleSpinBox_ls_max_dist, SIGNAL(valueChanged(double)), ui->us_widget, SLOT(setLSMaxDist(double)));
    timer->start(50);

    emit ui->doubleSpinBox_ls_max_dist->setValue(5.0);
}


cSensorVisualizationWidget::~cSensorVisualizationWidget()
{
    delete ui;
}

void cSensorVisualizationWidget::setUs(tUltrasonicStruct usData)
{
    m_usData.us_rear_center = usData.tRearCenter.f32Value;
    m_usData.us_rear_left = usData.tRearLeft.f32Value;
    m_usData.us_rear_right = usData.tRearRight.f32Value;
    m_usData.us_side_left = usData.tSideLeft.f32Value;
    m_usData.us_side_right = usData.tSideRight.f32Value;

    m_frameRates[aadc::US_SIDELEFT].calc();
    m_frameRates[aadc::US_REARCENTERLEFT].calc();
    m_frameRates[aadc::US_REARCENTER].calc();
    m_frameRates[aadc::US_REARCENTERRIGHT].calc();
    m_frameRates[aadc::US_SIDERIGHT].calc();
}

void cSensorVisualizationWidget::setImu(tInerMeasUnitData imuData)
{
    m_frameRates[aadc::IMU].calc();
    m_imuData.f32A_x = imuData.f32A_x;
    m_imuData.f32A_y = imuData.f32A_y;
    m_imuData.f32A_z = imuData.f32A_z;
    m_imuData.f32G_x = imuData.f32G_x;
    m_imuData.f32G_y = imuData.f32G_y;
    m_imuData.f32G_z = imuData.f32G_z;
    m_imuData.f32M_x = imuData.f32M_x;
    m_imuData.f32M_y = imuData.f32M_y;
    m_imuData.f32M_z = imuData.f32M_z;
}

void cSensorVisualizationWidget::setWheelLeft(int count, int direction)
{
    m_frameRates[aadc::WHEEL_LEFT].calc();
    m_wheelData.count_left = count;
    m_wheelData.dir_left = direction;
}

void cSensorVisualizationWidget::setWheelRight(int count, int direction)
{
    m_frameRates[aadc::WHEEL_RIGHT].calc();
    m_wheelData.count_right = count;
    m_wheelData.dir_right = direction;
}

void cSensorVisualizationWidget::setVoltage(tVoltageStruct voltData)
{
    m_frameRates[aadc::VOLTAGE_ACTUATORS].calc();

    m_batteryData.actuator_overall = voltData.tActuatorVoltage.f32Value;
    m_batteryData.actuator_cell1 = voltData.tActuatorCell1.f32Value;
    m_batteryData.actuator_cell2 = voltData.tActuatorCell2.f32Value;
    m_batteryData.sensors_overall = voltData.tSensorVoltage.f32Value;
    m_batteryData.sensors_cell1 = voltData.tSensorCell1.f32Value;
    m_batteryData.sensors_cell2 = voltData.tSensorCell2.f32Value;
    m_batteryData.sensors_cell3 = voltData.tSensorCell3.f32Value;
    m_batteryData.sensors_cell4 = voltData.tSensorCell4.f32Value;
    m_batteryData.sensors_cell5 = voltData.tSensorCell5.f32Value;
    m_batteryData.sensors_cell6 = voltData.tSensorCell6.f32Value;
}

void cSensorVisualizationWidget::setLaserScan(aadc::laserscanner::tLaserScan& scan)
{
    m_laserScannerData.clear();
    m_laserScannerData = scan;
}

void cSensorVisualizationWidget::update_gui()
{
    ui->us_widget->setDistances(m_usData.us_side_left,
        m_usData.us_side_right,
        m_usData.us_rear_left,
        m_usData.us_rear_center,
        m_usData.us_rear_right);

    ui->us_widget->setLSData(m_laserScannerData);

    // US
    ui->lcdNumber_us_update_sl->display(m_frameRates[aadc::US_SIDELEFT].get_frame_rate().c_str());
    ui->lcdNumber_us_sl->display(format_number(m_usData.us_side_left));

    ui->lcdNumber_us_update_rcl->display(m_frameRates[aadc::US_REARCENTERLEFT].get_frame_rate().c_str());
    ui->lcdNumber_us_rcl->display(format_number(m_usData.us_rear_left));

    ui->lcdNumber_us_update_rc->display(m_frameRates[aadc::US_REARCENTER].get_frame_rate().c_str());
    ui->lcdNumber_us_rc->display(format_number(m_usData.us_rear_center));

    ui->lcdNumber_us_update_rcr->display(m_frameRates[aadc::US_REARCENTERRIGHT].get_frame_rate().c_str());
    ui->lcdNumber_us_rcr->display(format_number(m_usData.us_rear_right));

    ui->lcdNumber_us_update_sr->display(m_frameRates[aadc::US_SIDERIGHT].get_frame_rate().c_str());
    ui->lcdNumber_us_sr->display(format_number(m_usData.us_side_right));

    // WHEEL
    ui->lcdNumber_wheel_update_left->display(m_frameRates[aadc::WHEEL_LEFT].get_frame_rate().c_str());

    ui->lcdNumber_wheel_left_count->display(m_wheelData.count_left);
    ui->lcdNumber_wheel_left_direction->display(m_wheelData.dir_left);

    ui->lcdNumber_wheel_update_right->display(m_frameRates[aadc::WHEEL_RIGHT].get_frame_rate().c_str());

    ui->lcdNumber_wheel_right_count->display(m_wheelData.count_right);
    ui->lcdNumber_wheel_right_direction->display(m_wheelData.dir_right);

    // IMU
    ui->lcdNumber_imu_update->display(m_frameRates[aadc::IMU].get_frame_rate().c_str());

    ui->lcdNumber_imu_acc_x->display(m_imuData.f32A_x);
    ui->lcdNumber_imu_acc_y->display(m_imuData.f32A_y);
    ui->lcdNumber_imu_acc_z->display(m_imuData.f32A_z);

    ui->lcdNumber_imu_angular_x->display(m_imuData.f32G_x);
    ui->lcdNumber_imu_angular_y->display(m_imuData.f32G_y);
    ui->lcdNumber_imu_angular_z->display(m_imuData.f32G_z);

    ui->lcdNumber_imu_mag_x->display(m_imuData.f32M_x);
    ui->lcdNumber_imu_mag_y->display(m_imuData.f32M_y);
    ui->lcdNumber_imu_mag_z->display(m_imuData.f32M_z);

    //BATTERY
    ui->lcdNumber_volt_update->display(m_frameRates[aadc::VOLTAGE].get_frame_rate().c_str());

    ui->lcdNumber_volt_actuator_overall->display(m_batteryData.actuator_overall);
    ui->lcdNumber_volt_sensors_overall->display(m_batteryData.sensors_overall);

    ui->lcdNumber_actuator_cell1->display(m_batteryData.actuator_cell1);
    ui->lcdNumber_actuator_cell2->display(m_batteryData.actuator_cell2);
    ui->lcdNumber_sensors_cell1->display(m_batteryData.sensors_cell1);
    ui->lcdNumber_sensors_cell2->display(m_batteryData.sensors_cell2);
    ui->lcdNumber_sensors_cell3->display(m_batteryData.sensors_cell3);
    ui->lcdNumber_sensors_cell4->display(m_batteryData.sensors_cell4);
    ui->lcdNumber_sensors_cell5->display(m_batteryData.sensors_cell5);
    ui->lcdNumber_sensors_cell6->display(m_batteryData.sensors_cell6);

    //LS
    ui->lcdNumber_ls_samples->display(m_laserScannerData.size());

}
