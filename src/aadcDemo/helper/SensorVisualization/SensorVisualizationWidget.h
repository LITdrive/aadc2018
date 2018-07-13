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

#pragma once

#include "stdafx.h"
#include "ui_sensor_visualization.h"

#include "FrameCounter.h"
#include "aadc_structs.h"
#include "aadc_enums.h"
#include "aadc_laserscanner.h"
#include "aadc_simple_structs.h"

/*! A sensor visualization widget. */
class cSensorVisualizationWidget: public QWidget, public Ui_SensorVisualizationUi
{
        
    Q_OBJECT

    public slots:

        /*!
         * Sets the us.
         *
         * \param   usData  The data.
         */
        void setUs                  (tUltrasonicStruct usData);

        /*!
         * Sets an imu.
         *
         * \param   imuData Information describing the imu.
         */
        void setImu                 (tInerMeasUnitData imuData);

        /*!
         * Sets wheel left.
         *
         * \param   count       Number of.
         * \param   direction   The direction.
         */
		void setWheelLeft           (int count, int direction);

        /*!
         * Sets wheel right.
         *
         * \param   count       Number of.
         * \param   direction   The direction.
         */
		void setWheelRight          (int count, int direction);

        /*!
         * Sets a voltage.
         *
         * \param   voltData    Information describing the volt.
         */
        void setVoltage             (tVoltageStruct voltData);

        /*!
         * Sets laser scan.
         *
         * \param [in,out]  scan    The scan.
         */
        void setLaserScan           (aadc::laserscanner::tLaserScan& scan);

		/*! Updates the graphical user interface. */
		void update_gui();

	private:
		/*! The user interface */
        Ui_SensorVisualizationUi *ui;

		/*! The ultrasonic data */
		tUltrasonicStructSimple m_usData;

        /*! data from the the laser scanner */
        aadc::laserscanner::tLaserScan m_laserScannerData;

		/*! the imu data */
		tInerMeasUnitData m_imuData;

		/*! A wheel data. */
		tWheelDataSimple m_wheelData;

		/*! holds the battery data */
		tVoltageStructSimple m_batteryData;

		/*! The frame rates[ size] */
		FrameCounter m_frameRates[aadc::NUM_SENSORS];

	public:

        /*!
         * Constructor.
         *
         * \param [in,out]  parent  (Optional) If non-null, the parent.
         */
		cSensorVisualizationWidget(QWidget *parent = 0);
		/*! Destructor. */
		~cSensorVisualizationWidget();

};

