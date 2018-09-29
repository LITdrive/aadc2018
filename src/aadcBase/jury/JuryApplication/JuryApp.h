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

#ifndef JURYAPP_H
#define JURYAPP_H

#include <QMainWindow>
#include <QFileDialog>
#include <QtNetwork/QTcpSocket>
#include <QtXml/QtXml>
#include <qmessagebox.h>
#include <QString>

#include <aadc_jury.h>
#include <thread>
#include <QGraphicsView>

//#include <aadc_structs.h>
// use own defintion, otherwise we have to include ADTF
#pragma pack(push,1)
typedef struct
{
    int16_t i16ActionID;
    int16_t i16ManeuverEntry;
} tJuryStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    int16_t i16StateID;
    int16_t i16ManeuverEntry;
} tDriverStruct;
#pragma pack(pop)

//forward declaration of main windows
namespace Ui {
    class MainWindow;
}

/*! This is the main class of the jury application. It is a Qt5-standalone application and
 *  is used by the jury to control the car during the competition */
class JuryApp : public QMainWindow
{
    Q_OBJECT

public:

    /*!
     * Constructor.
     *
     * \param [in,out]  parent  (Optional) If non-null, the parent.
     */
    explicit JuryApp(QWidget *parent = 0);
    /*! Destructor. */
    ~JuryApp();

    void closeEvent(QCloseEvent *event) override;
private slots:

    /*! Handles action exit triggered signals. */
    void on_actionExit_triggered();

    /*! Handles action clear all triggered signals. */
    void on_actionClear_all_triggered();

    /*! Handles set maneuver file triggered signals. */
    void onSetManeuverFileTriggered();
    /*! Handles set open drive map triggered signals. */
    void onSetOpenDriveMapTriggered();
    /*! Handles set traffic sign map triggered signals. */
    void onSetRoadSignMapTriggered();

    /*! Handles load maneuver triggered signals. */
    void onLoadManeuverTriggered();

    /*!
     * Transmit file.
     *
     * \param [in,out]  pFile       If non-null, the file.
     * \param           fileType    Type of the file.
     *
     * \return  True if it succeeds, false if it fails.
     */
    bool TransmitFile(QFile* pFile, aadc::jury::juryContainerId fileType);

    /*! Handles send maneuver triggered signals. */
    void onSendManeuverTriggered();
    
    /*! Handles send open drive map triggered signals. */
    void onSendOpenDriveMapTriggered();
    /*! Handles send traffic sign map triggered signals. */
    void onSendRoadSignMapTriggered();

    /*! Handles connect signals. */
    void on_connect();
    
    /*! Handles disconnect signals. */
    void on_disconnect();

    /*! Handles connection established signals. */
    void on_socketConnectionEstablished();

    /*! Handles IP address entered signals. */
    void on_ipAddressEntered();


    /*! Handles socket disconnected signals. */
    void on_socketDisconnected();

    /*!
     * Handles socket error signals.
     *
     * \param   socketError The socket error.
     */
    void onSocketError(QAbstractSocket::SocketError socketError);

    /*!
     * Handles section index changed signals.
     *
     * \param   index   Zero-based index of the.
     */
    void on_sectionIndexChanged(int index);

    /*! Handles read socket signals. */
    void on_readSocket();

    /*!
     * Transmit jury structure.
     *
     * \param   actionID        action id for struct.
     * \param   maneuverEntry   maneuver entry for transmit.
     */
    void transmitJuryStruct(int8_t actionID, int16_t maneuverEntry);

    /*! Handles get ready signals. */
    void on_GetReady();

    /*! Handles start signals. */
    void on_Start();

    /*! Handles stop signals. */
    void on_Stop();

    

private:

    /*! The user interface */
    Ui::MainWindow *ui;    

    /*! The TCP socket */
    QTcpSocket *m_tcpSocket;
    
    //if application is connected to car
    bool m_isConnected;

    //if we have a valid maneuver file
    bool m_hasValidManeuverFile;

    // the loaded manveuver file
    QFile* m_maneuverFile;

   
    /*! Sets initial values. */
    void setInitValues(bool deleteUserEntries = true);

    void WriteLog(const QString log) const;
    void BrushGraphicsView(QGraphicsView* view, QColor color, QString text = "");


    /*!
     * Verify maneuver file.
     *
     * \param   document    The document.
     *
     * \return  True if it succeeds, false if it fails.
     */
    bool verifyManeuverFile(QDomDocument document);

    /*! The parsed maneuver file */
    std::vector<std::vector<aadc::jury::tManeuver> > m_parsedManeuverFile;

    /*!
     * Process the XML file described by file.
     *
     * \param   file    The file.
     *
     * \return  True if it succeeds, false if it fails.
     */
    bool processXmlFile(QFile* file);

    //thread for reading data from socket
    QTimer *m_readTimer;

    QSettings m_settings;
};

#endif // MAINWINDOW_H
