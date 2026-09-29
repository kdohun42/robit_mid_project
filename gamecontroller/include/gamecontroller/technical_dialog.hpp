#ifndef TECHNICAL_DIALOG_HPP
#define TECHNICAL_DIALOG_HPP

#include <QDialog>
#include <QComboBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QPushButton>

#include "qnode.hpp"

class TechnicalDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TechnicalDialog(QNode *node, int playerNum, int initSide, int position, QWidget *parent = nullptr);

private slots:
    void onChallengeChanged(int index);

private:
    QNode   *m_qnode;
    int      m_playerNum;
    int      m_initSide;
    int      m_position;
    QComboBox *m_comboChallenge;
};

#endif // TECHNICAL_DIALOG_HPP
