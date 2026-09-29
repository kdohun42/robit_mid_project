/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui->
 *
 * @date August 2024
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/gamecontroller/main_window.hpp"

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) : QMainWindow(parent), ui(new Ui::RobocupController)
{
    ui->setupUi(this);

    qnode = new QNode(argc, argv);
    myTeam = qnode->teamRobit;

    bool isInit = initAddrAndPort();
    if (isInit)
    {
        std::cout << "===== GAME CONTROLLER OPEN =====" << std::endl;
    }
    else
    {
        std::cout << "======= INITIALIZE FAIL ========" << std::endl;
        exit(0);
    }

    QIcon icon("://ros-icon.png");
    this->setWindowIcon(icon);

    QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));

    pub_timer = new QTimer(this);
    QObject::connect(pub_timer, SIGNAL(timeout()), this, SLOT(Pub_msg()));
    pub_timer->start(1000);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete m_pReadSocket;
    delete m_pSendSocket;
}

bool MainWindow::initAddrAndPort()
{
    bool isOpen_Network = false;
    bool isTimeOut = false;

    clock_t timeoutCnt_start = clock();
    clock_t timeoutCnt_end = clock();

    while (!isOpen_Network)
    {
        timeoutCnt_end = clock();
        if (double(timeoutCnt_end - timeoutCnt_start) / CLOCKS_PER_SEC > 1)
        {
            isTimeOut = true;
        }

        QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();

        for (int i = 0; i < ipAddressesList.size(); i++)
        {
            //            std::cout << i << " " << ipAddressesList.at(i).toString().toStdString() << std::endl;
            if (ipAddressesList.at(i) != QHostAddress::LocalHost && ipAddressesList.at(i).toIPv4Address())
            {
                if (isTimeOut)
                {
                    if (ipAddressesList.at(i).toString().toStdString().find("172") != std::string::npos) // 192
                    {
                        m_qstrIp = ipAddressesList.at(i).toString();
                        isOpen_Network = true;
                        break;
                    }
                }
                else
                {
                    if (ipAddressesList.at(i).toString().toStdString().find("192") != std::string::npos) // 192
                    {
                        m_qstrIp = ipAddressesList.at(i).toString();
                        isOpen_Network = true;
                        break;
                    }
                }
            }
        }

        if (m_qstrIp.isEmpty())
        {
            m_qstrIp = QHostAddress(QHostAddress::LocalHost).toString();
        }

        if (isTimeOut)
        {
            isOpen_Network = true;
        }
    }

    m_iPort = qnode->dataPort;

    return isOpen_Network;
}

void MainWindow::initSocket()
{
    cout << "initSocket" << endl;
    if (m_pReadSocket == nullptr)
    {
        m_pReadSocket = new QUdpSocket(this);
    }

    if (m_pReadSocket != nullptr && m_pReadSocket->bind(m_iPort, QUdpSocket::ShareAddress))
    {
        m_bIsServerOpen = true;
        ui->checkBox_side->setEnabled(false);
        ui->checkBox_kickoff->setEnabled(false);
        ui->comboBox_number->setEnabled(false);
        ui->comboBox_position->setEnabled(false);
        ui->comboBox_team->setEnabled(false);

        ui->Server_Open->setText("CLOSE");

        connect(m_pReadSocket, SIGNAL(readyRead()), this, SLOT(readData()));
        readData();
    }
}

void MainWindow::closeSocket()
{
    ui->checkBox_side->setEnabled(true);
    ui->checkBox_kickoff->setEnabled(true);
    ui->comboBox_number->setEnabled(true);
    if (ui->comboBox_position->currentIndex() != POSITION_TECHNICAL)
    {
        ui->comboBox_position->setEnabled(true);
    }
    ui->comboBox_state->setEnabled(true);
    ui->comboBox_team->setEnabled(true);
    ui->checkBox_side->setTristate(false);
    ui->checkBox_side->setText("Left (first half)");

    disconnect(m_pReadSocket, SIGNAL(readyRead()), this, SLOT(readData()));
    ui->textEdit->clear();

    ui->Server_Open->setText("OPEN");

    m_pReadSocket->close();

    m_bIsServerOpen = false;
}

void MainWindow::on_comboBox_position_currentIndexChanged(int index)
{
    if (index == POSITION_TECHNICAL)
    {
        ChangeTechnicalMode();
        ui->Server_Open->setEnabled(false);

        return;
    }

    if (index != POSITION)
    {
        ui->Server_Open->setEnabled(true);
    }
    else
    {
        ui->Server_Open->setEnabled(false);
    }
}

void MainWindow::on_comboBox_state_currentIndexChanged(int index)
{
    qnode->gameControlData.robotnum = playerNum + 1;
    qnode->gameControlData.myside = initSide;
    qnode->gameControlData.position = position;
    qnode->gameControlData.state = index;

    if (index == 1) // state: ready
    {
        qnode->gameControlData.readytime = 45;
    }
    else
    {
        qnode->gameControlData.readytime = 0;
    }

    switch (index)
    {
    case 0:
        qnode->gameControlData.state_name = "INITIAL";
        break;
    case 1:
        qnode->gameControlData.state_name = "READY";
        break;
    case 2:
        qnode->gameControlData.state_name = "SET";
        break;
    case 3:
        qnode->gameControlData.state_name = "PLAYING";
        break;
    case 4:
        qnode->gameControlData.state_name = "FINISHED";
        break;

    default:
        break;
    }

    qnode->gamecontrollerPub->publish(qnode->gameControlData);
}

void MainWindow::on_Server_Open_clicked()
{
    if (m_bIsServerOpen == false)
    {
        playerNum = ui->comboBox_number->currentIndex();

        initSide = ui->checkBox_side->isChecked() ? LEFT : RIGHT;

        position = ui->comboBox_position->currentIndex();

        switch (ui->comboBox_team->currentIndex())
        {
        case 0: // ROBIT
            myTeam = qnode->teamRobit;
            break;
        case 1: // ROBIT_RED
            myTeam = qnode->teamRobitRed;
            break;
        case 2: // ROBIT_BLUE
            myTeam = qnode->teamRobitBlue;
            break;
        default:
            myTeam = qnode->teamRobit;
            ui->comboBox_team->setCurrentIndex(0);
            break;
        }
        qnode->gameControlData.myteam = myTeam;

        initSocket();

        qnode->gameControlData.robotnum = playerNum + 1;
        qnode->gameControlData.myside = initSide;
        qnode->gameControlData.iskickoff = ui->checkBox_kickoff->isChecked() ? true : false;
        qnode->gameControlData.position = position;
        qnode->gameControlData.state = ui->comboBox_state->currentIndex();
        qnode->gamecontrollerPub->publish(qnode->gameControlData);

        g_timer = new QTimer(this);
        QObject::connect(g_timer, SIGNAL(timeout()), this, SLOT(GameController_callback()));
        g_timer->start(500);
    }
    else
    {
        closeSocket();
        delete g_timer;
    }
}

void MainWindow::ChangeTechnicalMode()
{
    cout << "!!TECHNICAL MODE ON!!" << endl;

    playerNum = ui->comboBox_number->currentIndex();
    position = ui->comboBox_position->currentIndex();

    ui->comboBox_position->setEnabled(false);
    ui->comboBox_number->setEnabled(false);
    ui->comboBox_team->setEnabled(false);
    ui->checkBox_side->setEnabled(false);
    ui->checkBox_kickoff->setEnabled(false);

    TechnicalDialog dlg(qnode, playerNum, initSide, position, this);
    dlg.exec();

    // 팝업 닫히면 설정 컨트롤 다시 활성화
    ui->comboBox_position->setEnabled(true);
    ui->comboBox_number->setEnabled(true);
    ui->comboBox_team->setEnabled(true);
    ui->checkBox_side->setEnabled(true);
    ui->checkBox_kickoff->setEnabled(true);
}

void MainWindow::readData()
{
    QByteArray str;
    str.resize(m_pReadSocket->bytesAvailable());

    m_pReadSocket->readDatagram(str.data(), str.size(), &senderAddress, &senderPort);

    qDebug() << "From : " << senderAddress.toString();
    qDebug() << "Port : " << senderPort;
    qDebug() << "Message : " << str.size();

    cout << "============= GAME INFO =============" << endl;
    cout << "SIDE : " << mySide << endl;
    cout << "playerNum = " << playerNum + 1 << endl;
    cout << "position  = " << position << endl;

    if (str.size() == sizeof(RoboCupGameControlData))
    {
        ui->textEdit->clear();

        memmove(&robocupData, str.data(), sizeof(RoboCupGameControlData));
        cout << "state = "       << (int)robocupData.state << endl;
        cout << "gamePhase = "   << (int)robocupData.gamePhase << endl;
        cout << "setPlay = "     << (int)robocupData.setPlay << endl;
        cout << "half = "        << (int)robocupData.firstHalf << endl;
        cout << "kickingTeam = " << (int)robocupData.kickingTeam << endl;
        cout << "Team 0 Number " << (int)robocupData.teams[0].teamNumber << endl;
        cout << "Team 1 Number " << (int)robocupData.teams[1].teamNumber << endl;

        mySide = robocupData.teams[0].teamNumber == myTeam ? 0 : 1;

        if (robocupData.teams[0].teamNumber != myTeam &&
            robocupData.teams[1].teamNumber != myTeam)
        {
            cout << "!!NOT OUR GAME!!" << endl;
            return;
        }

        // gamePhase / setPlay → secondstate 변환
        int secondstate = STATE2_NORMAL;
        if (robocupData.gamePhase == GAME_PHASE_PENALTY_SHOOT_OUT)
            secondstate = STATE2_PENALTYSHOOT;
        else if (robocupData.gamePhase == GAME_PHASE_EXTRA_TIME)
            secondstate = STATE2_OVERTIME;
        else if (robocupData.gamePhase == GAME_PHASE_TIMEOUT)
            secondstate = STATE2_TIMEOUT;
        else if (robocupData.setPlay == SET_PLAY_DIRECT_FREE_KICK)
            secondstate = STATE2_DIRECT_FREEKICK;
        else if (robocupData.setPlay == SET_PLAY_INDIRECT_FREE_KICK)
            secondstate = STATE2_INDIRECT_FREEKICK;
        else if (robocupData.setPlay == SET_PLAY_PENALTY_KICK)
            secondstate = STATE2_PENALTYKICK;
        else if (robocupData.setPlay == SET_PLAY_THROW_IN)
            secondstate = STATE2_THROW_IN;
        else if (robocupData.setPlay == SET_PLAY_GOAL_KICK)
            secondstate = STATE2_GOAL_KICK;
        else if (robocupData.setPlay == SET_PLAY_CORNER_KICK)
            secondstate = STATE2_CORNER_KICK;

        qnode->gameControlData.robotnum    = playerNum + 1;
        qnode->gameControlData.position    = position;
        qnode->gameControlData.state       = (int)robocupData.state;
        qnode->gameControlData.stopped     = (int)robocupData.stopped;
        qnode->gameControlData.myside      = mySide;
        qnode->gameControlData.iskickoff   = (robocupData.kickingTeam == myTeam) ? true : false;
        qnode->gameControlData.secondstate = secondstate;
        qnode->gameControlData.readytime   = (int)robocupData.secondaryTime;
        qnode->gameControlData.penalty     = (int)robocupData.teams[mySide].players[playerNum].penalty;
        qnode->gameControlData.message_budget = (int)robocupData.teams[mySide].messageBudget;
        ui->label_msg_budget_value->setText(
            QString::number(qnode->gameControlData.message_budget));
        qnode->gameControlData.secondinfo.clear();

        mySide = qnode->gameControlData.myside;

        std::cout << endl
                  << endl;

        qnode->gamecontrollerPub->publish(qnode->gameControlData);

        uiUpdate();
    }
    qnode->gameControlData.robotnum = playerNum + 1;
}

void MainWindow::Pub_msg()
{
    qnode->gamecontrollerPub->publish(qnode->gameControlData);
}

void MainWindow::uiUpdate()
{
    switch (robocupData.competitionType)
    {
    case COMPETITION_TYPE_SMALL:
        ui->textEdit->append("SMALL SIZE");
        break;
    case COMPETITION_TYPE_MIDDLE:
        ui->textEdit->append("MIDDLE SIZE");
        break;
    case COMPETITION_TYPE_LARGE:
        ui->textEdit->append("LARGE SIZE");
        break;
    default:
        break;
    }

    switch (robocupData.firstHalf)
    {
    case 1:
        ui->textEdit->append("1st Half");

        break;

    case 0:
        ui->textEdit->append("2nd Half");
        break;
    default:
        break;
    }

    ui->textEdit->append("secs remaining: " + QString::number((int)robocupData.secsRemaining) + "\n");

    if (qnode->gameControlData.myside == LEFT)
    {
        ui->textEdit->append("side: LEFTSIDE");
    }
    else
    {
        ui->textEdit->append("side: RIGHTSIDE");
    }
    ui->checkBox_side->setCheckState(Qt::CheckState::PartiallyChecked);
    ui->checkBox_side->setText("AUTO");

    switch (robocupData.state)
    {
    case STATE_INITIAL:
        ui->textEdit->append("state: INITIAL");
        break;
    case STATE_READY:
        ui->textEdit->append("state: READY");
        ui->textEdit->append("ready time: " + QString::number(qnode->gameControlData.readytime));
        break;
    case STATE_SET:
        ui->textEdit->append("state: SET");
        break;
    case STATE_PLAYING:
        ui->textEdit->append("state: PLAY");
        if (qnode->gameControlData.readytime)
        {
            ui->textEdit->append("kick-off time: " + QString::number(qnode->gameControlData.readytime));
        }
        break;
    case STATE_FINISHED:
        ui->textEdit->append("state: FINISH");
        if (qnode->gameControlData.readytime)
        {
            ui->textEdit->append("half time: " + QString::number(qnode->gameControlData.readytime));
        }
        break;
    default:
        break;
    }
    ui->comboBox_state->setEnabled(false);
    ui->comboBox_state->setCurrentIndex(robocupData.state);

    if (qnode->gameControlData.iskickoff)
        ui->textEdit->append("kickoff: YES");
    else
        ui->textEdit->append("kickoff: NO");

    switch (qnode->gameControlData.penalty)
    {
    case PENALTY_NONE:
        ui->textEdit->append("penalty: NONE");
        break;
    case SUBSTITUTE:
        ui->textEdit->append("penalty: SUBSTITUTE");
        break;
    default:
        ui->textEdit->append("penalty: " + QString::number(qnode->gameControlData.penalty));
        break;
    }
    if (qnode->gameControlData.penalty != NONE)
    {
        ui->textEdit->append("secsTillUnpenalised: " + QString::number((int)robocupData.teams[mySide].players[playerNum].secsTillUnpenalised));
    }

    switch (qnode->gameControlData.secondstate)
    {
    case STATE2_PENALTYSHOOT:
        ui->textEdit->append("secondary: PENALTYSHOOT");
        if (qnode->gameControlData.secondinfo.size() >= 2)
            ui->textEdit->append("info : " + QString::number(qnode->gameControlData.secondinfo[0]) + " " + QString::number(qnode->gameControlData.secondinfo[1]));
        break;

    case STATE2_OVERTIME:
        ui->textEdit->append("secondary: OVERTIME");
        break;

    case STATE2_TIMEOUT:
        ui->textEdit->append("secondary: TIMEOUT");
        break;

    case STATE2_DIRECT_FREEKICK:
        ui->textEdit->append("secondary: DIRECT_FREEKICK");
        if (qnode->gameControlData.secondinfo.size() >= 2)
            ui->textEdit->append("info: " + QString::number(qnode->gameControlData.secondinfo[0]) + " " + QString::number(qnode->gameControlData.secondinfo[1]));
        break;

    case STATE2_INDIRECT_FREEKICK:
        ui->textEdit->append("secondary: INDIRECT_FREEKICK");
        if (qnode->gameControlData.secondinfo.size() >= 2)
            ui->textEdit->append("info: " + QString::number(qnode->gameControlData.secondinfo[0]) + " " + QString::number(qnode->gameControlData.secondinfo[1]));
        break;

    case STATE2_PENALTYKICK:
        ui->textEdit->append("secondary: PENALTYKICK");
        if (qnode->gameControlData.secondinfo.size() >= 2)
            ui->textEdit->append("info: " + QString::number(qnode->gameControlData.secondinfo[0]) + " " + QString::number(qnode->gameControlData.secondinfo[1]));
        break;

    case STATE2_CORNER_KICK:
        ui->textEdit->append("secondary: CORNERKICK");
        if (qnode->gameControlData.secondinfo.size() >= 2)
            ui->textEdit->append("info: " + QString::number(qnode->gameControlData.secondinfo[0]) + " " + QString::number(qnode->gameControlData.secondinfo[1]));
        break;

    case STATE2_GOAL_KICK:
        ui->textEdit->append("secondary: GOALKICK");
        if (qnode->gameControlData.secondinfo.size() >= 2)
            ui->textEdit->append("info : " + QString::number(qnode->gameControlData.secondinfo[0]) + " " + QString::number(qnode->gameControlData.secondinfo[1]));
        break;

    case STATE2_THROW_IN:
        ui->textEdit->append("secondary: THROWIN");
        if (qnode->gameControlData.secondinfo.size() >= 2)
            ui->textEdit->append("info: " + QString::number(qnode->gameControlData.secondinfo[0]) + " " + QString::number(qnode->gameControlData.secondinfo[1]));
        break;

    default:
        ui->textEdit->append("secondary: NORMAL");
        break;
    }
}

void MainWindow::GameController_callback()
{
  if (m_pSendSocket == nullptr)
  {
    m_pSendSocket = new QUdpSocket(this);
  }

  RoboCupGameControlReturnData returnData;
  returnData.playerNum = static_cast<uint8_t>(playerNum + 1);
  returnData.teamNum = static_cast<uint8_t>(myTeam);
  returnData.fallen = 0;
  returnData.pose[0] = 0.0f;
  returnData.pose[1] = 0.0f;
  returnData.pose[2] = 0.0f;
  returnData.ballAge = -1.0f;
  returnData.ball[0] = 0.0f;
  returnData.ball[1] = 0.0f;

  QByteArray Data(reinterpret_cast<const char *>(&returnData), sizeof(returnData));

  if (!senderAddress.isNull())
  {
    m_pSendSocket->writeDatagram(Data, senderAddress, qnode->returnPort);
  }
}


