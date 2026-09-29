#include "gamecontroller/technical_dialog.hpp"

TechnicalDialog::TechnicalDialog(QNode *node, int playerNum, int initSide, int position, QWidget *parent)
    : QDialog(parent),
      m_qnode(node),
      m_playerNum(playerNum),
      m_initSide(initSide),
      m_position(position)
{
    setWindowTitle("TECHNICAL CONTROLLER");
    setMinimumWidth(340);
    setWindowFlags(Qt::Dialog | Qt::WindowCloseButtonHint);

    QVBoxLayout *layout = new QVBoxLayout(this);

    QLabel *titleLabel = new QLabel("TECHNICAL CONTROLLER", this);
    titleLabel->setStyleSheet("font-size: 16pt; font-weight: bold;");
    titleLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(titleLabel);

    QLabel *infoLabel = new QLabel("Robot No. " + QString::number(m_playerNum + 1), this);
    infoLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(infoLabel);

    layout->addSpacing(10);

    QLabel *challengeLabel = new QLabel("Challenge:", this);
    layout->addWidget(challengeLabel);

    m_comboChallenge = new QComboBox(this);
    m_comboChallenge->addItem("TECHNICAL");
    m_comboChallenge->addItem("LOCAL KICKER");
    m_comboChallenge->addItem("LOCAL HELPER");
    m_comboChallenge->addItem("DYNAMIC KICK");
    m_comboChallenge->addItem("HIGH KICK");
    m_comboChallenge->addItem("PARKOUR");
    m_comboChallenge->addItem("OBSTACLE");
    m_comboChallenge->addItem("CORNER KICK RECEIVER");
    layout->addWidget(m_comboChallenge);

    layout->addSpacing(10);

    QPushButton *closeBtn = new QPushButton("Close", this);
    layout->addWidget(closeBtn);

    setLayout(layout);

    connect(m_comboChallenge, SIGNAL(currentIndexChanged(int)), this, SLOT(onChallengeChanged(int)));
    connect(closeBtn, SIGNAL(clicked()), this, SLOT(accept()));

    // 초기 상태 publish
    onChallengeChanged(0);
}

void TechnicalDialog::onChallengeChanged(int index)
{
    m_qnode->gameControlData.robotnum  = m_playerNum + 1;
    m_qnode->gameControlData.myside    = m_initSide;
    m_qnode->gameControlData.position  = m_position;
    m_qnode->gameControlData.state     = index;
    m_qnode->gameControlData.readytime = 0;

    switch (index)
    {
    case 1:
        m_qnode->gameControlData.state_name = "LOCAL KICKER";
        break;
    case 2:
        m_qnode->gameControlData.state_name = "LOCAL HELPER";
        break;
    case 3:
        m_qnode->gameControlData.state_name = "DYNAMIC KICK";
        break;
    case 4:
        m_qnode->gameControlData.state_name = "HIGH KICK";
        break;
    case 5:
        m_qnode->gameControlData.state_name = "PARKOUR";
        break;
    case 6:
        m_qnode->gameControlData.state_name = "OBSTACLE";
        break;
    case 7:
        m_qnode->gameControlData.state_name = "CORNER KICK RECEIVER";
        break;

    default:
        break;
    }

    m_qnode->gamecontrollerPub->publish(m_qnode->gameControlData);
}
