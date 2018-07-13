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

#ifndef DISPWIDGET_DRIVER_FILTER
#define DISPWIDGET_DRIVER_FILTER

#include "stdafx.h"

#include "aadc_jury.h"

/*!
This class it the QWidget for the prototyp of the driver filter
*/
class DisplayWidgetDriver : public QWidget
{
    Q_OBJECT

public:
    /*! constructor for the widget
    * \param parent the parent widget
    */
    DisplayWidgetDriver(QWidget* parent);
    /*! Destructor. */
    ~DisplayWidgetDriver() {};

public slots:

    /*! slot for receiving a new go command with a maneuver number
    * \param entryId the current maneuver id
    */
    void OnDriverGo(int entryId);
    /*! slot for receiving a stop command
    * \param entryId the current maneuver id
    */
    void OnDriverStop(int entryId);
    /*! slot for receiving a request ready command
    * \param entryId the current maneuver id
    */
    void OnDriverRequestReady(int entryId);

private slots:
    /*!  slot for sending the startup state button*/
    void OnStartupClicked();
    /*!  slot for sending the run state button*/
    void OnStateRunClicked();
    /*!  slot for sending the error state button*/
    void OnStateErrorClicked();
    /*! slot for response ready*/
    void OnResponseReadyClicked();
    /*! slot for state complete button*/
    void OnStateCompleteClicked();
    /*! slot for adding new text line to log field
    * \param text to be added as new line
    */
    void OnAppendText(QString text);

public:
    /*! this function sets the given sectorList to m_sectorList of the Widget
    * \param sectorList sectorList to be printed
    */
    void SetManeuverList(std::vector<aadc::jury::tSector>& sectorList);

    /*! this function sets the given sectorList to the log field
    */
    void ShowManeuverList();
    /*! this functions fills the combobox of the gui */
    void FillComboBox();
    /*! this method enables or disables the maneuver group box
    * \param bEnable if true, the maneuver group box will be enabled, disabled otherwise
    */
    void EnableManeuverGroupBox(bool bEnable);

signals:

    /*! signal for sending the state
    * \param i8StateID state to be sent; -1: error, 0: Ready, 1: Running
    * \param i16ManeuverEntry current entry to be sent
    */
    void sendStruct(aadc::jury::stateCar i8StateID, tInt16 i16ManeuverEntry);

private:
    /*! the main widget */
    QWidget* m_pWidget;
    /*! the button to send the startup response */
    QPushButton *m_pSendStartupButton;

    /*! the button to send the response to the ready request in the given the section */
    QPushButton *m_pSendReadyResponseButton;

    /*! the button to send the error state */
    QPushButton *m_pSendStateErrorButton;

    /*! the button to send the run state */
    QPushButton *m_pSendStateRunButton;

    /*! the button to send the complete state */
    QPushButton *m_pSendStateCompleteButton;

    /*! combo box for selection the maneuver id */
    QComboBox *m_comboBox;

    /*! The maneuver group box */
    QGroupBox *m_pManeuverGroupBox;

    /*! label for current maneuver id of jury*/
    QLabel *m_JuryInfo;

    /*! the main layout of the widget */
    QVBoxLayout* m_mainLayout;

    /*! field to show the parsed maneuver actions*/
    QTextEdit *m_logField;

    /*! a list with all the sector entries */
    aadc::jury::maneuverList m_sectorList;

};

#endif
