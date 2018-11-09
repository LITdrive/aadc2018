#include <QString>

#include "JuryApp.h"
#include "ui_mainwindow.h"
#include <QGraphicsWidget>


JuryApp::JuryApp(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_settings("AADC", "jury_application")
{
    ui->setupUi(this);

    QRegExp ipRegEx("^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$");
    ui->lEdIpAddress->setValidator( new QRegExpValidator(ipRegEx,this));

    m_tcpSocket = new QTcpSocket(this);

    // do the qt connections
    connect(ui->btSetManeuverFile, SIGNAL(pressed()), this, SLOT(onSetManeuverFileTriggered()));
    connect(ui->btSetOpenDriveMap, SIGNAL(pressed()), this, SLOT(onSetOpenDriveMapTriggered()));
    connect(ui->btSetRoadSignMap, SIGNAL(pressed()), this, SLOT(onSetRoadSignMapTriggered()));

    connect(ui->btLoadManeuver, SIGNAL(pressed()), this, SLOT(onLoadManeuverTriggered()));
    connect(ui->btSendManeuver, SIGNAL(pressed()), this, SLOT(onSendManeuverTriggered()));
    connect(ui->btSendOpenDriveMap, SIGNAL(pressed()), this, SLOT(onSendOpenDriveMapTriggered()));
    connect(ui->btSendRoadSignMap, SIGNAL(pressed()), this, SLOT(onSendRoadSignMapTriggered()));

    connect(ui->btConnect, SIGNAL(pressed()), this, SLOT(on_connect()));
    connect(ui->btDisconnect, SIGNAL(pressed()), this, SLOT(on_disconnect()));

    connect(ui->btGetReady, SIGNAL(pressed()), this, SLOT(on_GetReady()));
    connect(ui->btStart, SIGNAL(pressed()), this, SLOT(on_Start()));
    connect(ui->btStop, SIGNAL(pressed()), this, SLOT(on_Stop()));
    connect(this->m_tcpSocket, SIGNAL(connected()), this, SLOT(on_socketConnectionEstablished()));
    connect(this->m_tcpSocket, SIGNAL(disconnected()), this, SLOT(on_socketDisconnected()));
    connect(this->m_tcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(onSocketError(QAbstractSocket::SocketError)));
    connect(ui->lEdIpAddress, SIGNAL(editingFinished()), this, SLOT(on_ipAddressEntered()));
    connect(ui->cmBoxSections, SIGNAL(currentIndexChanged(int)), this, SLOT(on_sectionIndexChanged(int)));


    m_readTimer = nullptr;

    setInitValues();

}

JuryApp::~JuryApp()
{
    delete ui;
}

void JuryApp::closeEvent(QCloseEvent *event)
{
    const char* val = ui->lEdIpAddress->text().toStdString().c_str();
    QString ip = ui->lEdIpAddress->text();
    //read settings here
    m_settings.setValue("maneuverfile/path", ui->lEdManeuverFile->text());
    m_settings.setValue("opendrivemap/path", ui->lEdOpenDriveMap->text());
    m_settings.setValue("trafficsignmap/path", ui->lEdRoadSignMap->text());

    m_settings.setValue("connection/ip", ui->lEdIpAddress->text());
    m_settings.setValue("connection/port", ui->lEdPort->text());
    QMainWindow::closeEvent(event);
}

void JuryApp::onLoadManeuverTriggered()
{
    m_maneuverFile = new QFile(ui->lEdManeuverFile->text(), this);

    //process and verify file
    if (!processXmlFile(m_maneuverFile))
    {
        m_hasValidManeuverFile = false;
        return;
    }
    m_hasValidManeuverFile = true;

	ui->cmBoxSections->clear();
	ui->cmBoxManeuvers->clear();

    //write possible sections to combo list 
    for (int i = 0; i < m_parsedManeuverFile.size(); i++)
    {
        ui->cmBoxSections->addItem(QString::number(i));
    }

    if (m_isConnected)
    {
        ui->btSendManeuver->setEnabled(true);
        ui->btGetReady->setEnabled(true);
        ui->btStart->setEnabled(true);
        ui->btStop->setEnabled(true);
    }

    QGraphicsScene* newScene = new QGraphicsScene(this);;
    ui->viewStateFile->setScene(newScene);
    newScene->setBackgroundBrush(QBrush(Qt::green, Qt::SolidPattern));

}

bool JuryApp::TransmitFile(QFile* pFile, const aadc::jury::juryContainerId fileType)
{
    if (pFile == nullptr)
    {
        return false;
    }
    if (pFile->open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QByteArray ba = pFile->readAll();

        aadc::jury::tJuryContainer container;
        container.id = fileType;
        container.dataSize = ba.size();
        container.data = ba.data();

        std::vector<char> serialized;
        aadc::jury::serializeContainer(container, serialized);
        m_tcpSocket->write(serialized.data(), serialized.size());

        //wait until at least some bytes are written
        if (!m_tcpSocket->waitForBytesWritten())
        {
            WriteLog("Data could not be sent to client");
        }
    }
    else
    {
        WriteLog("File \"" + pFile->fileName() + "\" could not be opened for sending");
        return false;
    }
    pFile->close();
    return true;
}

void JuryApp::onSendManeuverTriggered()
{
    if (m_hasValidManeuverFile)
    {
        WriteLog("Sending maneuver file (" + m_maneuverFile->fileName() + ").");
        if (TransmitFile(m_maneuverFile, aadc::jury::container_id_maneuverlist))
        {
            BrushGraphicsView(ui->viewStateManeuverFileSent, Qt::green);            
        }

    }
}

void JuryApp::onSendOpenDriveMapTriggered()
{
    /*! The open drive map file */
    QFile* openDriveMapFile;


    openDriveMapFile = new QFile(ui->lEdOpenDriveMap->text(), this);

    WriteLog("Sending OpenDrive map (" + openDriveMapFile->fileName() + ").");
    if (TransmitFile(openDriveMapFile, aadc::jury::container_id_opendrive_map))
    {
        BrushGraphicsView(ui->viewStateOpenDriveMapSent, Qt::green);
    }


}

void JuryApp::onSendRoadSignMapTriggered()
{
    QFile* roadSignMapFile;

    roadSignMapFile = new QFile(ui->lEdRoadSignMap->text(), this);

    WriteLog("Sending road sign map (" + roadSignMapFile->fileName() + ").");
    if (TransmitFile(roadSignMapFile, aadc::jury::container_id_traffic_sign_map))
    {
        BrushGraphicsView(ui->viewStateRoadSignMapSent, Qt::green);        
    }
}

void JuryApp::on_connect()
{
    //check necessary inputs
    if (ui->lEdIpAddress->text().isEmpty())
    {
        WriteLog("Please set a valid client IP address!");
        return;
    }

    if (ui->lEdPort->text().isEmpty())
    {
        WriteLog("Please set a valid client port!");
        return;
    }
    //try to connect
    m_tcpSocket->connectToHost(ui->lEdIpAddress->text(), ui->lEdPort->text().toUInt());

    //wait for connect

    if (!m_tcpSocket->waitForConnected(3000))
    {
        WriteLog("Could not connect to host!");
        return;
    }
    ui->btConnect->setDisabled(true);
    ui->btDisconnect->setDisabled(false);
}

void JuryApp::on_disconnect()
{
    m_tcpSocket->disconnectFromHost();

    if (m_readTimer)
    {
        if (m_readTimer->isActive()) m_readTimer->stop();
    }
    m_isConnected = false;
    ui->btConnect->setDisabled(false);
    ui->btDisconnect->setDisabled(true);
    BrushGraphicsView(ui->viewStateManeuverFileSent, Qt::red);
    BrushGraphicsView(ui->viewStateRoadSignMapSent, Qt::red);
    BrushGraphicsView(ui->viewStateOpenDriveMapSent, Qt::red);

}

void JuryApp::on_socketConnectionEstablished()
{
    ui->btConnect->setEnabled(false);
    ui->lEdPort->setEnabled(false);
    ui->lEdIpAddress->setEnabled(false);

    m_isConnected = true;

    QGraphicsScene* newScene = new QGraphicsScene(this);;
    ui->viewStateConnection->setScene(newScene);
    newScene->setBackgroundBrush(QBrush(Qt::green, Qt::SolidPattern));

    WriteLog("Connection to Server " + ui->lEdIpAddress->text() + ":" + ui->lEdPort->text() + " established.");

    if (m_hasValidManeuverFile)
    {
        ui->btSendManeuver->setEnabled(true);
        ui->btGetReady->setEnabled(true);
        ui->btStart->setEnabled(true);
        ui->btStop->setEnabled(true);
    }

    ui->btSendRoadSignMap->setEnabled(true);
    ui->btSendOpenDriveMap->setEnabled(true);

    m_readTimer = new QTimer(this);
    connect(m_readTimer, SIGNAL(timeout()), this, SLOT(on_readSocket()));
    m_readTimer->start(100);
}

void JuryApp::on_ipAddressEntered()
{
    //check necessary inputs
    if (!ui->lEdIpAddress->hasAcceptableInput())
    {
        WriteLog("Please set a valid client IP address!");
        return;
    }
}

void JuryApp::on_socketDisconnected()
{
    WriteLog("Connection to client was lost!");

    BrushGraphicsView(ui->viewStateConnection, Qt::red);
    BrushGraphicsView(ui->viewStateCar, Qt::red);
    ui->lblCarState->setText("no state");

    ui->lEdPort->setEnabled(true);
    ui->lEdIpAddress->setEnabled(true);

    ui->btSendManeuver->setDisabled(true);
    ui->btGetReady->setDisabled(true);
    ui->btStart->setDisabled(true);
    ui->btStop->setDisabled(true);

    ui->btSendRoadSignMap->setDisabled(true);
    ui->btSendOpenDriveMap->setDisabled(true);

    ui->btDisconnect->setDisabled(true);
    ui->btConnect->setEnabled(true);

    if (m_readTimer)
    {
        if (m_readTimer->isActive()) m_readTimer->stop();
    }
    return;
}

void JuryApp::onSocketError(QAbstractSocket::SocketError socketError)
{
    WriteLog("Error in socket: " + QString::number(socketError));
    //setInitValues(false);

    if (m_readTimer)
    {
        if (m_readTimer->isActive())
            m_readTimer->stop();
    }
    return;
}

void JuryApp::on_sectionIndexChanged(int index)
{
    //clear old list
    ui->cmBoxManeuvers->clear();

    if (index >= 0 && index < m_parsedManeuverFile.size())
    {
        //write possible maneuvers to combo list 
        for (std::vector<aadc::jury::tManeuver>::iterator it = m_parsedManeuverFile[index].begin(); it != m_parsedManeuverFile[index].end(); ++it)
        {
			if(it->action == aadc::jury::manuever_undefined)
			{
				continue;
			}

            ui->cmBoxManeuvers->addItem(QString::number(it->id));
        }
    }
}

void JuryApp::BrushGraphicsView(QGraphicsView* view, QColor color, QString text)
{
    QGraphicsScene* newScene1 = new QGraphicsScene(this);;
    view->setScene(newScene1);
    newScene1->setBackgroundBrush(QBrush(color, Qt::SolidPattern));

    QGraphicsTextItem *textItem = newScene1->addText(text);
    textItem->setPos(100, 200);
}

void JuryApp::on_readSocket()
{
    if (m_tcpSocket->bytesAvailable() >= sizeof(tDriverStruct))
    {
        tDriverStruct newStruct;
        qint64 readBytes = m_tcpSocket->read(reinterpret_cast<char*>(&newStruct), static_cast<qint64>(sizeof(tDriverStruct)));

        if (readBytes == sizeof(newStruct))
        {
            const aadc::jury::stateCar carState = static_cast<aadc::jury::stateCar>(newStruct.i16StateID);
            const QString carStateStr = aadc::jury::stateCarToString(carState).c_str();
            ui->lblCarState->setText(carStateStr);
            WriteLog("state: \t" + carStateStr + "\t maneuver id: " + QString::number(newStruct.i16ManeuverEntry));
            //use some colors
            std::map < aadc::jury::stateCar, QColor> careState2Color = {
                { aadc::jury::statecar_error ,Qt::red },
                { aadc::jury::statecar_ready ,QColor(255, 153, 0) /*orange*/ },
                { aadc::jury::statecar_running ,Qt::green },
                { aadc::jury::statecar_complete ,Qt::darkGreen },
                { aadc::jury::statecar_startup ,Qt::yellow },
            };
            BrushGraphicsView(ui->viewStateCar, careState2Color[carState], "id: " + QString::number(newStruct.i16ManeuverEntry));

        }
    }
}

void JuryApp::transmitJuryStruct(int8_t actionID, int16_t maneuverEntry)
{
    //declare a jury struct
    tJuryStruct newStruct;
    newStruct.i16ActionID = actionID;
    newStruct.i16ManeuverEntry = maneuverEntry;
    //fill it into a container
    aadc::jury::tJuryContainer container;
    container.id = aadc::jury::container_id_juryStruct;
    container.dataSize = sizeof(tJuryStruct);
    container.data = reinterpret_cast<char*>(&newStruct);
    //serialize the container
    std::vector<char> data;
    aadc::jury::serializeContainer(container, data);
    //transmit via socket
    m_tcpSocket->write(data.data(), data.size());

    //wait until at least some bytes are written
    if (!m_tcpSocket->waitForBytesWritten())
    {
        WriteLog("Data could not be sent to client");
    }
}

void JuryApp::on_GetReady()
{
    transmitJuryStruct(aadc::jury::action_getready, ui->cmBoxManeuvers->currentText().toInt());
}

void JuryApp::on_Start()
{
    transmitJuryStruct(aadc::jury::action_start, ui->cmBoxManeuvers->currentText().toInt());
}

void JuryApp::on_Stop()
{
    transmitJuryStruct(aadc::jury::action_stop, ui->cmBoxManeuvers->currentText().toInt());
}

void JuryApp::setInitValues(bool deleteUserEntries)
{
    m_maneuverFile = nullptr;

    ui->btConnect->setEnabled(true);
    ui->btDisconnect->setDisabled(true);
    ui->lEdPort->setEnabled(true);
    ui->lEdIpAddress->setEnabled(true);
    ui->btSendManeuver->setEnabled(false);
    ui->btGetReady->setEnabled(false);
    ui->btStart->setEnabled(false);
    ui->btStop->setEnabled(false);
    ui->cmBoxManeuvers->clear();
    ui->cmBoxSections->clear();

    ui->btSendRoadSignMap->setDisabled(true);
    ui->btSendOpenDriveMap->setDisabled(true);

    m_hasValidManeuverFile = false;
    m_isConnected = false;
    if (deleteUserEntries) ui->lEdManeuverFile->setText("");

    BrushGraphicsView(ui->viewStateConnection, Qt::red);
    BrushGraphicsView(ui->viewStateCar, Qt::red);
    BrushGraphicsView(ui->viewStateFile, Qt::red);
    BrushGraphicsView(ui->viewStateOpenDriveMapSent, Qt::red);
    BrushGraphicsView(ui->viewStateRoadSignMapSent, Qt::red);
    BrushGraphicsView(ui->viewStateManeuverFileSent, Qt::red);


    //fill settings here
    ui->lEdManeuverFile->setText(m_settings.value("maneuverfile/path", "").toString());
    ui->lEdOpenDriveMap->setText(m_settings.value("opendrivemap/path", "").toString());
    ui->lEdRoadSignMap->setText(m_settings.value("trafficsignmap/path", "").toString());
    ui->lEdIpAddress->setText(m_settings.value("connection/ip", "127.0.0.1").toString());
    ui->lEdPort->setText(m_settings.value("connection/port", "54321").toString());

}

bool JuryApp::verifyManeuverFile(QDomDocument document)
{
    // Getting root element
    QDomElement root = document.firstChildElement();
    std::string test = root.tagName().toStdString();

    //check root tag name
    if (root.tagName().compare("AADC-Maneuver-List", Qt::CaseInsensitive) != 0)
    {
        return false;
    }

    //load all the sector nodes
    QDomNodeList sectorNodes = root.elementsByTagName("AADC-Sector");

    //check if the maneuvers are increasing
    int maneuverIndex = 0;

    for (int n = 0; n < sectorNodes.count(); n++)
    {
        //get maneuvers in sector
        QDomNodeList maneuverNodes = sectorNodes.at(n).childNodes();

        std::vector<aadc::jury::tManeuver> maneuversInSector;

        for (int nManeuvers = 0; nManeuvers < maneuverNodes.count(); nManeuvers++)
        {
            QDomNamedNodeMap attributes = maneuverNodes.at(nManeuvers).attributes();

            aadc::jury::tManeuver newManeuver;
            for (int nAttributes = 0; nAttributes < attributes.count(); nAttributes++)
            {
                QDomAttr newAttribute = attributes.item(nAttributes).toAttr();
                QString name = newAttribute.name();
                QString value = newAttribute.value();
                if (newAttribute.name() == "id")
                {
                    if (maneuverIndex != newAttribute.value().toInt())
                    {
                        //found not increasing maneuver index, return false
                        return false;
                    }
                    else
                    {
                        newManeuver.id = newAttribute.value().toInt();
                        maneuverIndex++;
                    }
                }
                else if (newAttribute.name() == "action")
                {
                    newManeuver.action = aadc::jury::maneuverFromString(newAttribute.value().toStdString());
                }
                else if (newAttribute.name() == "extra")
                {
                    newManeuver.extra = newAttribute.value().toInt();
                }
            }
            maneuversInSector.push_back(newManeuver);
        }
        //push maneuvers in sector list
        m_parsedManeuverFile.push_back(maneuversInSector);
    }

    return true;
}

bool JuryApp::processXmlFile(QFile* file)
{
    // Create a document to write XML
    QDomDocument document;

    if (file->open(QIODevice::ReadOnly | QIODevice::Text))
    {
        // loading
        if (document.setContent(file))
        {
            if (!verifyManeuverFile(document))
            {
                WriteLog("Content of maneuever file is not valid");
            }
            else
            {
                WriteLog("Correctly loaded maneuverfile " + file->fileName());
                file->close();
                return true;
            }

        }
        else
        {
            WriteLog("Selected file is not a valid XML file");
        }
    }
    else
    {
        WriteLog("Selected file cannot be opened");
    }

    file->close();
    return false;
}

void JuryApp::on_actionExit_triggered()
{

    if (m_readTimer)
    {
        if (m_readTimer->isActive()) m_readTimer->stop();
    }
    this->close();
}

void JuryApp::on_actionClear_all_triggered()
{

    if (m_readTimer)
    {
        if (m_readTimer->isActive()) m_readTimer->stop();
    }
    setInitValues();
    m_tcpSocket->disconnectFromHost();
}

void JuryApp::onSetManeuverFileTriggered()
{
    //open filepicker
    QString stringFileName = QFileDialog::getOpenFileName(this,
        tr("Load maneuver file"), "", tr("XML Files (*.xml)"));

    ui->lEdManeuverFile->setText(stringFileName);
}

void JuryApp::onSetOpenDriveMapTriggered()
{
    //open filepicker
    QString stringFileName = QFileDialog::getOpenFileName(this,
        tr("Load OpenDrive map file"), "", tr("XODR Files (*.xodr)"));

    ui->lEdOpenDriveMap->setText(stringFileName);
}

void JuryApp::onSetRoadSignMapTriggered()
{
    //open filepicker
    QString stringFileName = QFileDialog::getOpenFileName(this,
        tr("Load road sign map file"), "", tr("XML Files (*.xml)"));

    ui->lEdRoadSignMap->setText(stringFileName);
}

void JuryApp::WriteLog(const QString log) const
{
    ui->logField->moveCursor(QTextCursor::End);
    ui->logField->insertPlainText(log + "\n");
    ui->logField->moveCursor(QTextCursor::End);
}
