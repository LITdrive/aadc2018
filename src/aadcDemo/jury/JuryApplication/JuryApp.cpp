#include <QString>

#include "JuryApp.h"
#include "ui_mainwindow.h"


JuryApp::JuryApp(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QRegExpValidator ipRegEx(QRegExp("[0 - 9]{ 1,3 }\\.[0 - 9]{ 1,3 }\\.[0 - 9]{ 1,3 }\\.[0 - 9]{ 1,3 }"), this);
    ui->lEdIpAddress->setValidator(&ipRegEx);

    m_tcpSocket = new QTcpSocket(this);

    // do the qt connections

    connect(ui->btSetManeuverFile, SIGNAL(pressed()), this, SLOT(on_setManeuverFile_triggered()));
    connect(ui->btLoadManeuver, SIGNAL(pressed()), this, SLOT(on_loadManeuver_triggered()));
    connect(ui->btSendManeuver, SIGNAL(pressed()), this, SLOT(on_sendManeuver_triggered()));
    connect(ui->btConnect, SIGNAL(pressed()), this, SLOT(on_connect()));

    connect(ui->btGetReady, SIGNAL(pressed()), this, SLOT(on_GetReady()));
    connect(ui->btStart, SIGNAL(pressed()), this, SLOT(on_Start()));
    connect(ui->btStop, SIGNAL(pressed()), this, SLOT(on_Stop()));
    connect(this->m_tcpSocket, SIGNAL(connected()), this, SLOT(on_socketConnectionEstablished()));
    connect(this->m_tcpSocket, SIGNAL(disconnected()), this, SLOT(on_socketDisconnected()));
    connect(this->m_tcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(on_socketError(QAbstractSocket::SocketError())));
    connect(ui->lEdIpAddress, SIGNAL(editingFinished()), this, SLOT(on_ipAddressEntered()));
    connect(ui->cmBoxSections, SIGNAL(currentIndexChanged(int)), this, SLOT(on_sectionIndexChanged(int)));

    setInitValues();

}

JuryApp::~JuryApp()
{
    delete ui;
}

void JuryApp::on_loadManeuver_triggered()
{
    m_maneuverFile = new QFile(ui->lEdManeuverFile->text(), this);

    //process and verify file
    if (!processXmlFile(m_maneuverFile))
    {
        return;
    }

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
    
    m_hasValidManeuverFile = true;
}


void JuryApp::on_sendManeuver_triggered()
{
    if (m_hasValidManeuverFile)
    {
        if (m_maneuverFile->open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QByteArray ba = m_maneuverFile->readAll();
            m_tcpSocket->write(ba.data(), ba.size());

            //wait until at least some bytes are written
            if (!m_tcpSocket->waitForBytesWritten())
            {
                QMessageBox msgBox;
                msgBox.setText("Data could not be sent to client");
                msgBox.exec();
            }
        }
        else
        {
            QMessageBox msgBox;
            msgBox.setText("File could not be opened for sending");
            msgBox.exec();
        }
        m_maneuverFile->close();
    }
}

void JuryApp::on_connect()
{
    //check necessary inputs
    if (ui->lEdIpAddress->text().isEmpty())
    {
        QMessageBox msgBox;
        msgBox.setText("Please set a valid client IP address!");
        msgBox.exec();
        return;
    }

    if (ui->lEdPort->text().isEmpty())
    {
        QMessageBox msgBox;
        msgBox.setText("Please set a valid client port!");
        msgBox.exec();
        return;
    }
    //try to connect
    m_tcpSocket->connectToHost(ui->lEdIpAddress->text(), ui->lEdPort->text().toUInt());

    //wait for connect
    
    if (!m_tcpSocket->waitForConnected(3000))
    {
        QMessageBox msgBox;
        msgBox.setText("Could not connect to host!");
        msgBox.exec();
        return;
    }
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

    if (m_hasValidManeuverFile)
    {
        ui->btSendManeuver->setEnabled(true);
        ui->btGetReady->setEnabled(true);
        ui->btStart->setEnabled(true);
        ui->btStop->setEnabled(true);
    }

    m_readTimer = new QTimer(this);
    connect(m_readTimer, SIGNAL(timeout()), this, SLOT(on_readSocket()));
    m_readTimer->start(100);
}

void JuryApp::on_ipAddressEntered()
{
    //check necessary inputs
    if (ui->lEdIpAddress->hasAcceptableInput())
    {
        QMessageBox msgBox;
        msgBox.setText("Please set a valid client IP address!");
        msgBox.exec();
        return;
    }
}

void JuryApp::on_socketDisconnected()
{
    QMessageBox msgBox;
    msgBox.setText("Connection to client was lost!");
    msgBox.exec();
    setInitValues(false);

    if (m_readTimer->isActive()) m_readTimer->stop();
    return;
}

void JuryApp::on_socketError(QAbstractSocket::SocketError socketError)
{
    QMessageBox msgBox;
    msgBox.setText("Error in socket: " + QString::number(socketError));
    msgBox.exec();
    setInitValues(false);

    if (m_readTimer->isActive()) m_readTimer->stop();
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
            ui->cmBoxManeuvers->addItem(QString::number(it->id));
        }
    }
}

void JuryApp::on_readSocket()
{
    if (m_tcpSocket->bytesAvailable() >= sizeof(tDriverStruct))
    {
        tDriverStruct newStruct;
        qint64 readBytes = m_tcpSocket->read(reinterpret_cast<char*>(&newStruct), static_cast<qint64>(sizeof(tDriverStruct)));

        if (readBytes == sizeof(newStruct))
        {
            ui->lblCarState->setText(aadc::jury::stateCarToString(static_cast<aadc::jury::stateCar>(newStruct.i16StateID)).c_str());

            if (!m_hasReceivedCarState)
            {
                m_hasReceivedCarState = true;
                QGraphicsScene* newScene1 = new QGraphicsScene(this);;
                ui->viewStateCar->setScene(newScene1);
                newScene1->setBackgroundBrush(QBrush(Qt::green, Qt::SolidPattern));
            }
        }


    }
}

void JuryApp::transmitJuryStruct(int8_t actionID, int16_t maneuverEntry)
{
    tJuryStruct newStruct;
    newStruct.i16ActionID = actionID;
    newStruct.i16ManeuverEntry = maneuverEntry;
    
    m_tcpSocket->write(reinterpret_cast<const char*>(&newStruct), sizeof(tJuryStruct));

    //wait until at least some bytes are written
    if (!m_tcpSocket->waitForBytesWritten())
    {
        QMessageBox msgBox;
        msgBox.setText("Data could not be sent to client");
        msgBox.exec();
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
    ui->btConnect->setEnabled(true);
    ui->lEdPort->setEnabled(true);
    ui->lEdIpAddress->setEnabled(true);
    ui->btSendManeuver->setEnabled(false);
    ui->btGetReady->setEnabled(false);
    ui->btStart->setEnabled(false);
    ui->btStop->setEnabled(false);
    ui->cmBoxManeuvers->clear();
    ui->cmBoxSections->clear();

    m_hasValidManeuverFile = false;
    m_isConnected = false;
    m_hasReceivedCarState = false;
    if (deleteUserEntries) ui->lEdManeuverFile->setText("");

    QGraphicsScene* newScene = new QGraphicsScene(this);;
    ui->viewStateConnection->setScene(newScene);
    newScene->setBackgroundBrush(QBrush(Qt::red, Qt::SolidPattern));

    QGraphicsScene* newScene1 = new QGraphicsScene(this);;
    ui->viewStateCar->setScene(newScene1);
    newScene1->setBackgroundBrush(QBrush(Qt::red, Qt::SolidPattern));

    QGraphicsScene* newScene2 = new QGraphicsScene(this);;
    ui->viewStateFile->setScene(newScene2);
    newScene2->setBackgroundBrush(QBrush(Qt::red, Qt::SolidPattern));
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
                QMessageBox msgBox;
                msgBox.setText("Content of maneuever file is not valid");
                msgBox.exec();
            }
            else
            {
                file->close();
                return true;
            }

        }
        else
        {
            QMessageBox msgBox;
            msgBox.setText("Selected file is not a valid XML file");
            msgBox.exec();
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("Selected file cannot be opened");
        msgBox.exec();
    }

    file->close();
    return false;
}

void JuryApp::on_actionExit_triggered()
{
    if (m_readTimer->isActive()) m_readTimer->stop();
    this->close();
}

void JuryApp::on_actionClear_all_triggered()
{
    if (m_readTimer->isActive()) m_readTimer->stop();
    setInitValues();
    m_tcpSocket->disconnectFromHost();
}

void JuryApp::on_setManeuverFile_triggered()
{
    //open filepicker
    QString stringFileName = QFileDialog::getOpenFileName(this,
        tr("Load maneuver file"), "", tr("XML Files (*.xml)"));

    ui->lEdManeuverFile->setText(stringFileName);
}
