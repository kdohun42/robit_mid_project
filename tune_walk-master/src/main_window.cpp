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

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/tune_walk/main_window.hpp"
#include "../include/tune_walk/qnode.hpp"
#include <QKeyEvent>
#include <QDebug>
#include <QStatusBar>

using namespace std;

namespace tune_walk
{
  
  extern double pitch;
  extern double roll;
  extern double yaw;
  extern double left_x_zmp;
  extern double left_y_zmp;
  extern double right_x_zmp;
  extern double right_y_zmp;
  extern double total_x_zmp;
  extern double total_y_zmp;
  extern bool Left_Foot, Right_Foot, Both_Feet;

  bool IMU_graph_flag = false;
  bool Pattern_graph_flag = false;

  struct Set_Data
  {
    double Swing_Right_Leg_Box = 0;
    double Swing_Left_Leg_Box = 0;
    double Swing_Right_Shoulder_Box = 0;
    double Swing_Left_Shoulder_Box = 0;
    double Rise_Right_Leg_Box = 0;
    double Rise_Left_Leg_Box = 0;
    double Start_Swing_Box = 0;
    double Start_Rise_Box = 0;
    double End_Swing_Box = 0;
    double End_Rise_Box = 0;
    double Test_X_Box = 0;
    double Test_Side_Box = 0;
    double Test_Yaw_Box = 0;
    double Tuning_X_Box = 0;
    double Tuning_Side_Box = 0;
    double Tuning_Yaw_Box = 0;
  };
  Set_Data Tuning_Data[3];

  MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
  {
    ui->setupUi(this);

    QIcon icon(":/images/973761776.png");
    this->setWindowIcon(icon);
    qnode = new QNode();
    QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
    QObject::connect(qnode, SIGNAL(Landing_callback()),this, SLOT(LandingCallback()));

    QTimer *timer = new QTimer(this);
    QObject::connect(ui->Angle_horizontalScrollBar, &QScrollBar::valueChanged, this, &MainWindow::on_Angle_horizontalScrollBar_valueChanged);
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(IMUrealtimeDataSlot()));

    timer->start(10);
    int index = 0;
    makeIMUPlot();

    Click_Value = 3;
    Offset_Angle_Limit = 500;
    tune2walk.ik_flag = false;
    tune2walk.entire_time = 80;
    tune2walk.frequency = 1;
    tune2walk.ratio_check_flag = false;
    tune2walk.center2leg = 100;
    tune2walk.link2link = 150;
    tune2walk.init_z_up = 10;
    tune2walk.default_x_right = 0;
    tune2walk.default_x_left = 0;
    tune2walk.default_y_right = -tune2walk.center2leg;
    tune2walk.default_y_left = tune2walk.center2leg;
    tune2walk.default_z_right = -(tune2walk.link2link + tune2walk.link2link - tune2walk.init_z_up);
    tune2walk.default_z_left = -(tune2walk.link2link + tune2walk.link2link - tune2walk.init_z_up);
    Offset_Scroll_Value = 0;

    for (int i = 0; i < 22; i++)
    {
      offset_past_motor[i] = 0;
      tune2walk.percentage_of_ik_motor.push_back(1);
      tune2walk.offset_motor.push_back(0);
    }
    tune2walk.swing_leg_right = 10;
    tune2walk.swing_leg_left = 10;
    tune2walk.swing_shoulder_right = 0;
    tune2walk.swing_shoulder_left = 0;
    tune2walk.rise_leg_left = 10;
    tune2walk.rise_leg_right = 10;
    tune2walk.start_entire_time = 60;
    tune2walk.start_swing = 5;
    tune2walk.start_rise = 5;
    tune2walk.end_entire_time = 60;
    tune2walk.end_swing = 5;
    tune2walk.end_rise = 5;
    tune2walk.test_x = 0;
    tune2walk.test_side = 0;
    tune2walk.test_yaw = 0;
    tune2walk.tuning_x = 0;
    tune2walk.tuning_side = 0;
    tune2walk.tuning_yaw = 0;

    tune2walk.balance_value_0 = 0;
    tune2walk.balance_value_1 = 0;
    tune2walk.balance_value_2 = 0;
    tune2walk.balance_value_3 = 0;

    tune2walk.balance_pitch_gp = 0;
    tune2walk.balance_pitch_gi = 0;
    tune2walk.balance_pitch_gd = 0;
    tune2walk.balance_pitch_elimit = 0;
    tune2walk.balance_pitch_olimit = 0;
    tune2walk.balance_pitch_neg_target = 0;
    tune2walk.balance_pitch_pos_target = 0;

    tune2walk.balance_angle_pitch_gp = 0;
    tune2walk.balance_angle_pitch_gi = 0;
    tune2walk.balance_angle_pitch_gd = 0;
    tune2walk.balance_angle_pitch_elimit = 0;
    tune2walk.balance_angle_pitch_olimit = 0;
    tune2walk.balance_angle_pitch_neg_target = 0;
    tune2walk.balance_angle_pitch_pos_target = 0;

    tune2walk.balance_roll_gp = 0;
    tune2walk.balance_roll_gi = 0;
    tune2walk.balance_roll_gd = 0;
    tune2walk.balance_roll_elimit = 0;
    tune2walk.balance_roll_olimit = 0;
    tune2walk.balance_roll_neg_target = 0;
    tune2walk.balance_roll_pos_target = 0;

    tune2walk.balance_pitch_flag = false;
    tune2walk.balance_roll_flag = false;
    tune2walk.balance_ankle_pitch_flag = false;

    //////////////////imu_parameter////////////////////////////////
    tune2walk.balance_value_4 = 0;
    tune2walk.balance_value_5 = 0;
    tune2walk.balance_pitch_flag_imu = false;
    tune2walk.balance_pitch_gp_imu = 0;
    tune2walk.balance_pitch_gi_imu = 0;
    tune2walk.balance_pitch_gd_imu = 0;
    tune2walk.balance_pitch_elimit_imu = 0;
    tune2walk.balance_pitch_olimit_imu = 0;
    tune2walk.balance_pitch_neg_target_imu = 0;
    tune2walk.balance_pitch_pos_target_imu = 0;

    ui->Balance_Value_0_Box_2->setValue(tune2walk.balance_value_4);
    ui->Balance_Value_1_Box_2->setValue(tune2walk.balance_value_5);
    ui->Balance_Pitch_GP_IMU_Box->setValue(tune2walk.balance_pitch_gp_imu);
    ui->Balance_Pitch_GI_IMU_Box->setValue(tune2walk.balance_pitch_gi_imu);
    ui->Balance_Pitch_GD_IMU_Box->setValue(tune2walk.balance_pitch_gd_imu);
    ui->Balance_Pitch_ELIMIT_IMU_Box->setValue(tune2walk.balance_pitch_elimit_imu);
    ui->Balance_Pitch_OLIMIT_IMU_Box->setValue(tune2walk.balance_pitch_olimit_imu);
    ui->Balance_Pitch_Neg_Target_IMU_Box->setValue(tune2walk.balance_pitch_neg_target_imu);
    ui->Balance_Pitch_Pos_Target_IMU_Box->setValue(tune2walk.balance_pitch_pos_target_imu);

    tune2walk.balance_roll_flag_imu = false;
    tune2walk.balance_roll_gp_imu = 0;
    tune2walk.balance_roll_gi_imu = 0;
    tune2walk.balance_roll_gd_imu = 0;
    tune2walk.balance_roll_elimit_imu = 0;
    tune2walk.balance_roll_olimit_imu = 0;
    tune2walk.balance_roll_neg_target_imu = 0;
    tune2walk.balance_roll_pos_target_imu = 0;

    ui->Balance_Roll_GP_IMU_Box->setValue(tune2walk.balance_roll_gp_imu);
    ui->Balance_Roll_GI_IMU_Box->setValue(tune2walk.balance_roll_gi_imu);
    ui->Balance_Roll_GD_IMU_Box->setValue(tune2walk.balance_roll_gd_imu);
    ui->Balance_Roll_ELIMIT_IMU_Box->setValue(tune2walk.balance_roll_elimit_imu);
    ui->Balance_Roll_OLIMIT_IMU_Box->setValue(tune2walk.balance_roll_olimit_imu);
    ui->Balance_Roll_Neg_Target_IMU_Box->setValue(tune2walk.balance_roll_neg_target_imu);
    ui->Balance_Roll_Pos_Target_IMU_Box->setValue(tune2walk.balance_roll_pos_target_imu);

    ///////////////////////K_Value////////////////////////////////
    tune2walk.first_pos_xr = 0;
    tune2walk.first_neg_xr = 0;
    tune2walk.first_pos_side_r = 0;
    tune2walk.first_pos_side_r_swing_minus = 0;
    tune2walk.first_neg_side_r = 0;
    tune2walk.first_neg_side_r_swing_minus = 0;
    tune2walk.first_pos_yaw_r = 0;
    tune2walk.first_neg_yaw_r = 0;
    tune2walk.first_pos_xl = 0;
    tune2walk.first_neg_xl = 0;
    tune2walk.first_pos_side_l = 0;
    tune2walk.first_pos_side_l_swing_minus = 0;
    tune2walk.first_neg_side_l = 0;
    tune2walk.first_neg_side_l_swing_minus = 0;
    tune2walk.first_pos_yaw_l = 0;
    tune2walk.first_neg_yaw_l = 0;
    tune2walk.first_min = 0;
    tune2walk.first_max = 15;

    tune2walk.second_pos_xr = 0;
    tune2walk.second_neg_xr = 0;
    tune2walk.second_pos_side_r = 0;
    tune2walk.second_pos_side_r_swing_minus = 0;
    tune2walk.second_neg_side_r = 0;
    tune2walk.second_neg_side_r_swing_minus = 0;
    tune2walk.second_pos_yaw_r = 0;
    tune2walk.second_neg_yaw_r = 0;
    tune2walk.second_pos_xl = 0;
    tune2walk.second_neg_xl = 0;
    tune2walk.second_pos_side_l = 0;
    tune2walk.second_pos_side_l_swing_minus = 0;
    tune2walk.second_neg_side_l = 0;
    tune2walk.second_neg_side_l_swing_minus = 0;
    tune2walk.second_pos_yaw_l = 0;
    tune2walk.second_neg_yaw_l = 0;
    tune2walk.second_min = 15;
    tune2walk.second_max = 30;

    tune2walk.third_pos_xr = 0;
    tune2walk.third_neg_xr = 0;
    tune2walk.third_pos_side_r = 0;
    tune2walk.third_pos_side_r_swing_minus = 0;
    tune2walk.third_neg_side_r = 0;
    tune2walk.third_neg_side_r_swing_minus = 0;
    tune2walk.third_pos_yaw_r = 0;
    tune2walk.third_neg_yaw_r = 0;
    tune2walk.third_pos_xl = 0;
    tune2walk.third_neg_xl = 0;
    tune2walk.third_pos_side_l = 0;
    tune2walk.third_pos_side_l_swing_minus = 0;
    tune2walk.third_neg_side_l = 0;
    tune2walk.third_neg_side_l_swing_minus = 0;
    tune2walk.third_pos_yaw_l = 0;
    tune2walk.third_neg_yaw_l = 0;
    tune2walk.third_min = 30;
    tune2walk.third_max = 40;

    tune2walk.fourth_pos_xr = 0;
    tune2walk.fourth_neg_xr = 0;
    tune2walk.fourth_pos_side_r = 0;
    tune2walk.fourth_pos_side_r_swing_minus = 0;
    tune2walk.fourth_neg_side_r = 0;
    tune2walk.fourth_neg_side_r_swing_minus = 0;
    tune2walk.fourth_pos_yaw_r = 0;
    tune2walk.fourth_neg_yaw_r = 0;
    tune2walk.fourth_pos_xl = 0;
    tune2walk.fourth_neg_xl = 0;
    tune2walk.fourth_pos_side_l = 0;
    tune2walk.fourth_pos_side_l_swing_minus = 0;
    tune2walk.fourth_neg_side_l = 0;
    tune2walk.fourth_neg_side_l_swing_minus = 0;
    tune2walk.fourth_pos_yaw_l = 0;
    tune2walk.fourth_neg_yaw_l = 0;
    tune2walk.fourth_min = 40;
    tune2walk.fourth_max = 50;

    tune2walk.landing_time_control_flag = 0;

    tune2walk.control_system_flag = 0;

    ui->Pos_XRSpinBox->setValue(tune2walk.first_pos_xr);
    ui->Neg_XRSpinBox->setValue(tune2walk.first_neg_xr);
    ui->Pos_SideRSpinBox->setValue(tune2walk.first_pos_side_r);
    ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.first_pos_side_r_swing_minus);
    ui->Neg_SideRSpinBox->setValue(tune2walk.first_neg_side_r);
    ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.first_neg_side_r_swing_minus);
    ui->Pos_YawRSpinBox->setValue(tune2walk.first_pos_yaw_r);
    ui->Neg_YawRSpinBox->setValue(tune2walk.first_neg_yaw_r);
    ui->Pos_XLSpinBox->setValue(tune2walk.first_pos_xl);
    ui->Neg_XLSpinBox->setValue(tune2walk.first_neg_xl);
    ui->Pos_SideLSpinBox->setValue(tune2walk.first_pos_side_l);
    ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.first_pos_side_l_swing_minus);
    ui->Neg_SideLSpinBox->setValue(tune2walk.first_neg_side_l);
    ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.first_neg_side_l_swing_minus);
    ui->Pos_YawLSpinBox->setValue(tune2walk.first_pos_yaw_l);
    ui->Neg_YawLSpinBox->setValue(tune2walk.first_neg_yaw_l);
    ui->First_min->setValue(tune2walk.first_min);
    ui->First_max->setValue(tune2walk.first_max);
    ui->Second_min->setValue(tune2walk.second_min);
    ui->Second_max->setValue(tune2walk.second_max);
    ui->Third_min->setValue(tune2walk.third_min);
    ui->Third_max->setValue(tune2walk.third_max);
    ui->Fourth_min->setValue(tune2walk.fourth_min);
    ui->Fourth_max->setValue(tune2walk.fourth_max);
    ///////////////////////////////////////////////////////////////

    ui->Entire_Time_Box->setValue(tune2walk.entire_time);
    ui->Frequency_Box->setValue(tune2walk.frequency);
    ui->Center2Leg_Box->setValue(tune2walk.center2leg);
    ui->Link2Link_Box->setValue(tune2walk.link2link);
    ui->Init_Z_Up_Box->setValue(tune2walk.init_z_up);
    ui->Default_X_Right_Box->setValue(tune2walk.default_x_right);
    ui->Default_X_Left_Box->setValue(tune2walk.default_x_left);
    ui->Default_Y_Right_Box->setValue(tune2walk.default_y_right);
    ui->Default_Y_Left_Box->setValue(tune2walk.default_y_left);
    ui->Default_Z_Right_Box->setValue(tune2walk.default_z_right);
    ui->Default_Z_Left_Box->setValue(tune2walk.default_z_left);
    ui->Motor_10_Minus->setText(QString::number(tune2walk.offset_motor[10]));
    ui->Motor_11_Minus->setText(QString::number(tune2walk.offset_motor[11]));
    ui->Motor_12_Minus->setText(QString::number(tune2walk.offset_motor[12]));
    ui->Motor_13_Minus->setText(QString::number(tune2walk.offset_motor[13]));
    ui->Motor_14_Minus->setText(QString::number(tune2walk.offset_motor[14]));
    ui->Motor_15_Minus->setText(QString::number(tune2walk.offset_motor[15]));
    ui->Motor_16_Minus->setText(QString::number(tune2walk.offset_motor[16]));
    ui->Motor_17_Minus->setText(QString::number(tune2walk.offset_motor[17]));
    ui->Motor_18_Minus->setText(QString::number(tune2walk.offset_motor[18]));
    ui->Motor_19_Minus->setText(QString::number(tune2walk.offset_motor[19]));
    ui->Motor_20_Minus->setText(QString::number(tune2walk.offset_motor[20]));
    ui->Motor_21_Minus->setText(QString::number(tune2walk.offset_motor[21]));
    ui->Percentage_of_10_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[10]);
    ui->Percentage_of_11_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[11]);
    ui->Percentage_of_12_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[12]);
    ui->Percentage_of_13_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[13]);
    ui->Percentage_of_14_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[14]);
    ui->Percentage_of_15_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[15]);
    ui->Percentage_of_16_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[16]);
    ui->Percentage_of_17_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[17]);
    ui->Percentage_of_18_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[18]);
    ui->Percentage_of_19_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[19]);
    ui->Percentage_of_20_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[20]);
    ui->Percentage_of_21_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[21]);
    ui->Swing_Right_Leg_Box->setValue(tune2walk.swing_leg_right);
    ui->Swing_Left_Leg_Box->setValue(tune2walk.swing_leg_left);
    ui->Swing_Right_Shoulder_Box->setValue(tune2walk.swing_shoulder_right);
    ui->Swing_Left_Shoulder_Box->setValue(tune2walk.swing_shoulder_left);
    ui->Rise_Right_Leg_Box->setValue(tune2walk.rise_leg_right);
    ui->Rise_Left_Leg_Box->setValue(tune2walk.rise_leg_left);
    ui->Start_Entire_Time_Box->setValue(tune2walk.start_entire_time);
    ui->Start_Swing_Box->setValue(tune2walk.start_swing);
    ui->Start_Rise_Box->setValue(tune2walk.start_rise);
    ui->End_Entire_Time_Box->setValue(tune2walk.end_entire_time);
    ui->End_Swing_Box->setValue(tune2walk.end_swing);
    ui->End_Rise_Box->setValue(tune2walk.end_rise);

    ui->Test_X_Box->setValue(tune2walk.test_x);
    ui->Test_Side_Box->setValue(tune2walk.test_side);
    ui->Test_Yaw_Box->setValue(tune2walk.test_yaw);
    ui->Tuning_X_Box->setValue(tune2walk.tuning_x);
    ui->Tuning_Side_Box->setValue(tune2walk.tuning_side);
    ui->Tuning_Yaw_Box->setValue(tune2walk.tuning_yaw);

    ui->Balance_Value_0_Box->setValue(tune2walk.balance_value_0);

    ui->Balance_Pitch_GP_Box->setValue(tune2walk.balance_pitch_gp);
    ui->Balance_Pitch_GI_Box->setValue(tune2walk.balance_pitch_gi);
    ui->Balance_Pitch_GD_Box->setValue(tune2walk.balance_pitch_gd);
    ui->Balance_Pitch_ELIMIT_Box->setValue(tune2walk.balance_pitch_elimit);
    ui->Balance_Pitch_OLIMIT_Box->setValue(tune2walk.balance_pitch_olimit);
    ui->Balance_Pitch_Neg_Target_Box->setValue(tune2walk.balance_pitch_neg_target);
    ui->Balance_Pitch_Pos_Target_Box->setValue(tune2walk.balance_pitch_pos_target);

    ui->Balance_Value_1_Box->setValue(tune2walk.balance_value_1);

    ui->Balance_Ankle_Pitch_GP_Box->setValue(tune2walk.balance_angle_pitch_gp);
    ui->Balance_Ankle_Pitch_GI_Box->setValue(tune2walk.balance_angle_pitch_gi);
    ui->Balance_Ankle_Pitch_GD_Box->setValue(tune2walk.balance_angle_pitch_gd);
    ui->Balance_Ankle_Pitch_ELIMIT_Box->setValue(tune2walk.balance_angle_pitch_elimit);
    ui->Balance_Ankle_Pitch_OLIMIT_Box->setValue(tune2walk.balance_angle_pitch_olimit);
    ui->Balance_Ankle_Pitch_Neg_Target_Box->setValue(tune2walk.balance_angle_pitch_neg_target);
    ui->Balance_Ankle_Pitch_Pos_Target_Box->setValue(tune2walk.balance_angle_pitch_pos_target);

    ui->Balance_Value_2_Box->setValue(tune2walk.balance_value_2);
    ui->Balance_Value_3_Box->setValue(tune2walk.balance_value_3);

    ui->Balance_Roll_GP_Box->setValue(tune2walk.balance_roll_gp);
    ui->Balance_Roll_GI_Box->setValue(tune2walk.balance_roll_gi);
    ui->Balance_Roll_GD_Box->setValue(tune2walk.balance_roll_gd);
    ui->Balance_Roll_ELIMIT_Box->setValue(tune2walk.balance_roll_elimit);
    ui->Balance_Roll_OLIMIT_Box->setValue(tune2walk.balance_roll_olimit);
    ui->Balance_Roll_Neg_Target_Box->setValue(tune2walk.balance_roll_neg_target);
    ui->Balance_Roll_Pos_Target_Box->setValue(tune2walk.balance_roll_pos_target);

    ui->IMU_graph->setChecked(0);
    ui->Pattern_graph->setChecked(0);
  }

  void MainWindow::closeEvent(QCloseEvent *event)
  {
    QMainWindow::closeEvent(event);
  }

  MainWindow::~MainWindow()
  {
    delete ui;
  }

void MainWindow::keyPressEvent(QKeyEvent *e)
{
  if (e->key() == Qt::Key_W)
  {
    tune2walk.test_x += 1;
  }
  else if (e->key() == Qt::Key_A)
  {
    tune2walk.test_side += 1;
  }
  else if (e->key() == Qt::Key_S)
  {
    tune2walk.test_x -= 1;
  }
  else if (e->key() == Qt::Key_D)
  {
    tune2walk.test_side -= 1;
  }
  else if (e->key() == Qt::Key_Comma)
  {
    tune2walk.test_yaw += 1;
  }
  else if (e->key() == Qt::Key_Period)
  {
    tune2walk.test_yaw -= 1;
  }
  else if (e->key() == Qt::Key_1)
  {
    MainWindow::on_Save_Data1_Button_clicked();
  }
  else if (e->key() == Qt::Key_2)
  {
    MainWindow::on_Save_Data2_Button_clicked();
  }
  else if (e->key() == Qt::Key_3)
  {
    MainWindow::on_Save_Data3_Button_clicked();
  }
  else if (e->key() == Qt::Key_4)
  {
    MainWindow::on_Set_Data1_Button_clicked();
  }
  else if (e->key() == Qt::Key_5)
  {
    MainWindow::on_Set_Data2_Button_clicked();
  }
  else if (e->key() == Qt::Key_6)
  {
    MainWindow::on_Set_Data3_Button_clicked();
  }
  else if (e->key() == Qt::Key_0)
  {
    MainWindow::on_Set_Zero_Button_clicked();
  }
  else if (e->key() == Qt::Key_P)
  {
    static bool Control_Flag = true;
    if (Control_Flag)
    {
      tune2walk.ik_flag = 1;
      ui->IK_Flag_Button->setText("STOP");
      Control_Flag = false;
      cout << "STOP" << endl;
    }
    else if (!Control_Flag)
    {
      ui->IK_Flag_Button->setText("WALK");
      tune2walk.ik_flag = 0;
      Control_Flag = true;
      cout << "WALK" << endl;
    }
  }

  if (tune2walk.test_x >= 150)
    tune2walk.test_x = 150;
  if (tune2walk.test_x <= -50)
    tune2walk.test_x = -50;
  if (tune2walk.test_side >= 50)
    tune2walk.test_side = 50;
  if (tune2walk.test_side <= -50)
    tune2walk.test_side = -50;
  if (tune2walk.test_yaw >= 50)
    tune2walk.test_yaw = 50;
  if (tune2walk.test_yaw <= -50)
    tune2walk.test_yaw = -50;

  ui->Test_X_Box->setValue(tune2walk.test_x);
  ui->Test_Side_Box->setValue(tune2walk.test_side);
  ui->Test_Yaw_Box->setValue(tune2walk.test_yaw);

  qnode->tune2walk_Pub->publish(tune2walk);
}

//-------------------------------------------------------------IMU_graph

void MainWindow::makeIMUPlot()
{
  ui->IMU_plot->addGraph(); // blue line
  ui->IMU_plot->graph(0)->setPen(QPen(QColor(40, 110, 255)));//pitch
  ui->IMU_plot->addGraph(); // red line
  ui->IMU_plot->graph(1)->setPen(QPen(QColor(255, 110, 40)));//roll

  QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
  timeTicker->setTimeFormat("%h:%m:%s");
  ui->IMU_plot->xAxis->setTicker(timeTicker);
  ui->IMU_plot->axisRect()->setupFullAxesBox();
  ui->IMU_plot->yAxis->setRange(-20, 20);

  // make left and bottom axes transfer their ranges to right and top axes:
  connect(ui->IMU_plot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->IMU_plot->xAxis2, SLOT(setRange(QCPRange)));
  connect(ui->IMU_plot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->IMU_plot->yAxis2, SLOT(setRange(QCPRange)));

}

void MainWindow::IMUrealtimeDataSlot()
{
  if(IMU_graph_flag)
  {
      static QTime time(QTime::currentTime());
      // calculate two new data points:
      double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
      static double lastPointKey = 0;
      if (key-lastPointKey > 0.0001) // at most add point every 1s
      {
        // add data to lines:
        ui->IMU_plot->graph(0)->addData(key, pitch);
        ui->IMU_plot->graph(1)->addData(key, roll);
        // rescale value (vertical) axis to fit the current data:
        //ui->IMU_plot->graph(0)->rescaleValueAxis();
        //ui->IMU_plot->graph(1)->rescaleValueAxis(true);
        lastPointKey = key;
      }
      // make key axis range scroll with the data (at a constant range size of 8):
      ui->IMU_plot->xAxis->setRange(key, 8, Qt::AlignRight);
      ui->IMU_plot->replot();
  }
}

//---------------------------Parameter--------------------------------------//
void MainWindow::on_Entire_Time_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.entire_time = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Entire_Time_Plus_Button_clicked()
{
  tune2walk.entire_time += 10;
  if (tune2walk.entire_time >= 500)
    tune2walk.entire_time = 500;
  ui->Entire_Time_Box->setValue(tune2walk.entire_time);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Entire_Time_Minus_Button_clicked()
{
  tune2walk.entire_time -= 10;
  if (tune2walk.entire_time <= 0)
    tune2walk.entire_time = 0;
  ui->Entire_Time_Box->setValue(tune2walk.entire_time);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Frequency_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.frequency = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Frequency_Plus_Button_clicked()
{
  tune2walk.frequency += 1;
  if (tune2walk.frequency >= 100)
    tune2walk.frequency = 100;
  ui->Frequency_Box->setValue(tune2walk.frequency);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Frequency_Minus_Button_clicked()
{
  tune2walk.frequency -= 1;
  if (tune2walk.frequency <= 0)
    tune2walk.frequency = 0;
  ui->Frequency_Box->setValue(tune2walk.frequency);
  qnode->tune2walk_Pub->publish(tune2walk);
}

// Default_X_Right
void MainWindow::on_Default_X_Right_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.default_x_right = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Default_X_Right_Plus_Button_clicked()
{
  tune2walk.default_x_right += 1;
  if (tune2walk.default_x_right >= 100)
    tune2walk.default_x_right = 100;
  ui->Default_X_Right_Box->setValue(tune2walk.default_x_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Default_X_Right_Minus_Button_clicked()
{
  tune2walk.default_x_right -= 1;
  if (tune2walk.default_x_right <= -100)
    tune2walk.default_x_right = -100;
  ui->Default_X_Right_Box->setValue(tune2walk.default_x_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Default_X_Left_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.default_x_left = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Default_X_Left_Plus_Button_clicked()
{
  tune2walk.default_x_left += 1;
  if (tune2walk.default_x_left >= 100)
    tune2walk.default_x_left = 100;
  ui->Default_X_Left_Box->setValue(tune2walk.default_x_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Default_X_Left_Minus_Button_clicked()
{
  tune2walk.default_x_left -= 1;
  if (tune2walk.default_x_left <= -100)
    tune2walk.default_x_left = -100;
  ui->Default_X_Left_Box->setValue(tune2walk.default_x_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}
// Default_Y_Right
void MainWindow::on_Default_Y_Right_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.default_y_right = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Default_Y_Right_Plus_Button_clicked()
{
  tune2walk.default_y_right += 1;
  if (tune2walk.default_y_right >= 100)
    tune2walk.default_y_right = 100;
  ui->Default_Y_Right_Box->setValue(tune2walk.default_y_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Default_Y_Right_Minus_Button_clicked()
{
  tune2walk.default_y_right -= 1;
  if (tune2walk.default_y_right <= -100)
    tune2walk.default_y_right = -100;
  ui->Default_Y_Right_Box->setValue(tune2walk.default_y_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}
// Default_Y_Left
void MainWindow::on_Default_Y_Left_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.default_y_left = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Default_Y_Left_Plus_Button_clicked()
{
  tune2walk.default_y_left += 1;
  if (tune2walk.default_y_left >= 100)
    tune2walk.default_y_left = 100;
  ui->Default_Y_Left_Box->setValue(tune2walk.default_y_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Default_Y_Left_Minus_Button_clicked()
{
  tune2walk.default_y_left -= 1;
  if (tune2walk.default_y_left <= -100)
    tune2walk.default_y_left = -100;
  ui->Default_Y_Left_Box->setValue(tune2walk.default_y_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}
// Default_Z_Right
void MainWindow::on_Default_Z_Right_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.default_z_right = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Default_Z_Right_Plus_Button_clicked()
{
  tune2walk.default_z_right += 1;
  if (tune2walk.default_z_right >= 100)
    tune2walk.default_z_right = 100;
  ui->Default_Z_Right_Box->setValue(tune2walk.default_z_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Default_Z_Right_Minus_Button_clicked()
{
  tune2walk.default_z_right -= 1;
  if (tune2walk.default_z_right <= -(2 * tune2walk.link2link))
    tune2walk.default_z_right = -(2 * tune2walk.link2link);
  ui->Default_Z_Right_Box->setValue(tune2walk.default_z_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}
// Default_Z_Left
void MainWindow::on_Default_Z_Left_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.default_z_left = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Default_Z_Left_Plus_Button_clicked()
{
  tune2walk.default_z_left += 1;
  if (tune2walk.default_z_left >= 100)
    tune2walk.default_z_left = 100;
  ui->Default_Z_Left_Box->setValue(tune2walk.default_z_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Default_Z_Left_Minus_Button_clicked()
{
  tune2walk.default_z_left -= 1;
  if (tune2walk.default_z_left <= -(2 * tune2walk.link2link))
    tune2walk.default_z_left = -(2 * tune2walk.link2link);
  ui->Default_Z_Left_Box->setValue(tune2walk.default_z_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Center2Leg_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.center2leg = arg1;
  tune2walk.default_y_right = -tune2walk.center2leg;
  tune2walk.default_y_left = tune2walk.center2leg;
  ui->Default_Y_Right_Box->setValue(tune2walk.default_y_right);
  ui->Default_Y_Left_Box->setValue(tune2walk.default_y_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Link2Link_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.link2link = arg1;
  tune2walk.default_z_right = -(2 * tune2walk.link2link - tune2walk.init_z_up);
  tune2walk.default_z_left = -(2 * tune2walk.link2link - tune2walk.init_z_up);
  ui->Default_Z_Right_Box->setValue(tune2walk.default_z_right);
  ui->Default_Z_Left_Box->setValue(tune2walk.default_z_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Init_Z_Up_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.init_z_up = arg1;
  tune2walk.default_z_right = -(2 * tune2walk.link2link - tune2walk.init_z_up);
  tune2walk.default_z_left = -(2 * tune2walk.link2link - tune2walk.init_z_up);
  ui->Default_Z_Right_Box->setValue(tune2walk.default_z_right);
  ui->Default_Z_Left_Box->setValue(tune2walk.default_z_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Init_Z_Up_Plus_Button_clicked()
{
  tune2walk.init_z_up += 1;
  if (tune2walk.init_z_up >= 75)
    tune2walk.init_z_up = 75;
  ui->Init_Z_Up_Box->setValue(tune2walk.init_z_up);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Init_Z_Up_Minus_Button_clicked()
{
  tune2walk.init_z_up -= 1;
  if (tune2walk.init_z_up <= 1)
    tune2walk.init_z_up = 1;
  ui->Init_Z_Up_Box->setValue(tune2walk.init_z_up);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Ratio_Check_Box_clicked()
{
  if (tune2walk.ratio_check_flag == 0)
  {
    tune2walk.ratio_check_flag = 1;
    cout << "Double" << endl;
  }
  else if (tune2walk.ratio_check_flag == 1)
  {
    tune2walk.ratio_check_flag = 0;
    cout << "Single" << endl;
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}

//------------------------------Offset--------------------------------------//
void MainWindow::on_All_Offset_Zero_Button_clicked()
{
  QMessageBox Reset_Check_Box;
  Reset_Check_Box.setText("Do you really want to reset it?");
  Reset_Check_Box.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
  Reset_Check_Box.setDefaultButton(QMessageBox::Cancel);

  if (Reset_Check_Box.exec() == QMessageBox::Ok)
  {
    for (int i = 10; i < 22; i++)
    {
      tune2walk.offset_motor[i] = 0;
    }
    angle_update();
    past_motor_update();
    Offset_scroll_reset(0);

    arr_reset();
  }
}
void MainWindow::past_motor_update()
{
  for (int i = 10; i < 22; i++)
  {
    offset_past_motor[i] = tune2walk.offset_motor[i];
  }
}

void MainWindow::angle_update()
{
  ui->Motor_10_Minus->setText(QString::number(tune2walk.offset_motor[10]));
  ui->Motor_11_Minus->setText(QString::number(tune2walk.offset_motor[11]));
  ui->Motor_12_Minus->setText(QString::number(tune2walk.offset_motor[12]));
  ui->Motor_13_Minus->setText(QString::number(tune2walk.offset_motor[13]));
  ui->Motor_14_Minus->setText(QString::number(tune2walk.offset_motor[14]));
  ui->Motor_15_Minus->setText(QString::number(tune2walk.offset_motor[15]));
  ui->Motor_16_Minus->setText(QString::number(tune2walk.offset_motor[16]));
  ui->Motor_17_Minus->setText(QString::number(tune2walk.offset_motor[17]));
  ui->Motor_18_Minus->setText(QString::number(tune2walk.offset_motor[18]));
  ui->Motor_19_Minus->setText(QString::number(tune2walk.offset_motor[19]));
  ui->Motor_20_Minus->setText(QString::number(tune2walk.offset_motor[20]));
  ui->Motor_21_Minus->setText(QString::number(tune2walk.offset_motor[21]));

  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::arr_update()
{
  if (motor_plus_arr[0] != 0)
    ui->plus_Button_0->setText(QString::number(motor_plus_arr[0]));
  else
    ui->plus_Button_0->setText(" ");
  if (motor_plus_arr[1] != 0)
    ui->plus_Button_1->setText(QString::number(motor_plus_arr[1]));
  else
    ui->plus_Button_1->setText(" ");
  if (motor_plus_arr[2] != 0)
    ui->plus_Button_2->setText(QString::number(motor_plus_arr[2]));
  else
    ui->plus_Button_2->setText(" ");
  if (motor_plus_arr[3] != 0)
    ui->plus_Button_3->setText(QString::number(motor_plus_arr[3]));
  else
    ui->plus_Button_3->setText(" ");

  if (motor_minus_arr[0] != 0)
    ui->minus_Button_0->setText(QString::number(motor_minus_arr[0]));
  else
    ui->minus_Button_0->setText(" ");
  if (motor_minus_arr[1] != 0)
    ui->minus_Button_1->setText(QString::number(motor_minus_arr[1]));
  else
    ui->minus_Button_1->setText(" ");
  if (motor_minus_arr[2] != 0)
    ui->minus_Button_2->setText(QString::number(motor_minus_arr[2]));
  else
    ui->minus_Button_2->setText(" ");
  if (motor_minus_arr[3] != 0)
    ui->minus_Button_3->setText(QString::number(motor_minus_arr[3]));
  else
    ui->minus_Button_3->setText(" ");
}

void MainWindow::arr_reset()
{
  for (int i = 0; i < 4; i++)
  {
    motor_plus_arr[i] = 0;
    motor_minus_arr[i] = 0;
  }
  motor_plus_cnt = 0;
  motor_minus_cnt = 0;
  arr_update();
}

int MainWindow::arr_check(int arr[], int motor_num)
{
  for (int i = 3; i >= 0; i--)
  {
    if (arr[i] != motor_num && i == 0)
      return -1;
    else if (arr[i] == motor_num)
    {
      return i;
      break;
    }
  }
}

int MainWindow::arr_num_cnt(int num, int arr[], int size)
{
  int cnt = 0;
  for (int i = 0; i < size; i++)
    if (arr[i] == num)
      cnt++;
  return cnt;
}

void MainWindow::Motor_Num_Toss(int Pos_or_Neg, int motor_num)
{
  cout << "motor_num_toss" << endl;
  if (Pos_or_Neg >= 0)
  {
    arr_check_num = arr_check(motor_minus_arr, motor_num);
    if (arr_check_num != -1)
    {
      motor_minus_arr[arr_check_num] = 0;
      for (int i = arr_check_num; i < 3; i++)
      {
        motor_minus_arr[i] = motor_minus_arr[i + 1];
        motor_minus_arr[i + 1] = 0;
      }
      if (motor_minus_cnt > 0)
        motor_minus_cnt--;
    }
    else if (motor_plus_cnt < 4)
    {
      if (motor_plus_arr[motor_plus_cnt] == 0)
        motor_plus_arr[motor_plus_cnt] = motor_num;
      if (motor_plus_cnt != 3)
        motor_plus_cnt++;
    }
  }
  else
  {
    arr_check_num = arr_check(motor_plus_arr, motor_num);
    if (arr_check_num != -1)
    {
      motor_plus_arr[arr_check_num] = 0;
      for (int i = arr_check_num; i < 3; i++)
      {
        motor_plus_arr[i] = motor_plus_arr[i + 1];
        motor_plus_arr[i + 1] = 0;
      }
      if (motor_plus_cnt > 0)
        motor_plus_cnt--;
    }
    else if (motor_minus_cnt < 4)
    {
      if (motor_minus_arr[motor_minus_cnt] == 0)
        motor_minus_arr[motor_minus_cnt] = motor_num;
      if (motor_minus_cnt != 3)
        motor_minus_cnt++;
    }
  }
  arr_update();
}

void MainWindow::Offset_scroll_reset(int button_flag)
{
  if (Offset_Scroll_Value != 0)
  {
    Offset_Scroll_Value = 0;
    ui->Angle_Control_Box->setValue(Offset_Scroll_Value);
    ui->Angle_horizontalScrollBar->setValue(Offset_Scroll_Value);
  }
  if (button_flag == 1)
  {
    if (arr_check(motor_plus_arr, 10) == -1)
      tune2walk.offset_motor[10] = offset_past_motor[10];
    if (arr_check(motor_plus_arr, 11) == -1)
      tune2walk.offset_motor[11] = offset_past_motor[11];
    if (arr_check(motor_plus_arr, 12) == -1)
      tune2walk.offset_motor[12] = offset_past_motor[12];
    if (arr_check(motor_plus_arr, 13) == -1)
      tune2walk.offset_motor[13] = offset_past_motor[13];
    if (arr_check(motor_plus_arr, 14) == -1)
      tune2walk.offset_motor[14] = offset_past_motor[14];
    if (arr_check(motor_plus_arr, 15) == -1)
      tune2walk.offset_motor[15] = offset_past_motor[15];
    if (arr_check(motor_plus_arr, 16) == -1)
      tune2walk.offset_motor[16] = offset_past_motor[16];
    if (arr_check(motor_plus_arr, 17) == -1)
      tune2walk.offset_motor[17] = offset_past_motor[17];
    if (arr_check(motor_plus_arr, 18) == -1)
      tune2walk.offset_motor[18] = offset_past_motor[18];
    if (arr_check(motor_plus_arr, 19) == -1)
      tune2walk.offset_motor[19] = offset_past_motor[19];
    if (arr_check(motor_plus_arr, 20) == -1)
      tune2walk.offset_motor[20] = offset_past_motor[20];
    if (arr_check(motor_plus_arr, 21) == -1)
      tune2walk.offset_motor[21] = offset_past_motor[21];

    if (arr_check(motor_minus_arr, 10) == -1)
      tune2walk.offset_motor[10] = offset_past_motor[10];
    if (arr_check(motor_minus_arr, 11) == -1)
      tune2walk.offset_motor[11] = offset_past_motor[11];
    if (arr_check(motor_minus_arr, 12) == -1)
      tune2walk.offset_motor[12] = offset_past_motor[12];
    if (arr_check(motor_minus_arr, 13) == -1)
      tune2walk.offset_motor[13] = offset_past_motor[13];
    if (arr_check(motor_minus_arr, 14) == -1)
      tune2walk.offset_motor[14] = offset_past_motor[14];
    if (arr_check(motor_minus_arr, 15) == -1)
      tune2walk.offset_motor[15] = offset_past_motor[15];
    if (arr_check(motor_minus_arr, 16) == -1)
      tune2walk.offset_motor[16] = offset_past_motor[16];
    if (arr_check(motor_minus_arr, 17) == -1)
      tune2walk.offset_motor[17] = offset_past_motor[17];
    if (arr_check(motor_minus_arr, 18) == -1)
      tune2walk.offset_motor[18] = offset_past_motor[18];
    if (arr_check(motor_minus_arr, 19) == -1)
      tune2walk.offset_motor[19] = offset_past_motor[19];
    if (arr_check(motor_minus_arr, 20) == -1)
      tune2walk.offset_motor[20] = offset_past_motor[20];
    if (arr_check(motor_minus_arr, 21) == -1)
      tune2walk.offset_motor[21] = offset_past_motor[21];
    angle_update();
  }
}
void MainWindow::motor_limit_control()
{
  if (tune2walk.offset_motor[10] > Offset_Angle_Limit)
    tune2walk.offset_motor[10] = +Offset_Angle_Limit;
  else if (tune2walk.offset_motor[10] < -Offset_Angle_Limit)
    tune2walk.offset_motor[10] = -Offset_Angle_Limit;
  if (tune2walk.offset_motor[11] > Offset_Angle_Limit)
    tune2walk.offset_motor[11] = +Offset_Angle_Limit;
  else if (tune2walk.offset_motor[11] < -Offset_Angle_Limit)
    tune2walk.offset_motor[11] = -Offset_Angle_Limit;
  if (tune2walk.offset_motor[12] > Offset_Angle_Limit)
    tune2walk.offset_motor[12] = +Offset_Angle_Limit;
  else if (tune2walk.offset_motor[12] < -Offset_Angle_Limit)
    tune2walk.offset_motor[12] = -Offset_Angle_Limit;
  if (tune2walk.offset_motor[13] > Offset_Angle_Limit)
    tune2walk.offset_motor[13] = +Offset_Angle_Limit;
  else if (tune2walk.offset_motor[13] < -Offset_Angle_Limit)
    tune2walk.offset_motor[13] = -Offset_Angle_Limit;
  if (tune2walk.offset_motor[14] > Offset_Angle_Limit)
    tune2walk.offset_motor[14] = +Offset_Angle_Limit;
  else if (tune2walk.offset_motor[14] < -Offset_Angle_Limit)
    tune2walk.offset_motor[14] = -Offset_Angle_Limit;
  if (tune2walk.offset_motor[15] > Offset_Angle_Limit)
    tune2walk.offset_motor[15] = +Offset_Angle_Limit;
  else if (tune2walk.offset_motor[15] < -Offset_Angle_Limit)
    tune2walk.offset_motor[15] = -Offset_Angle_Limit;
  if (tune2walk.offset_motor[16] > Offset_Angle_Limit)
    tune2walk.offset_motor[16] = +Offset_Angle_Limit;
  else if (tune2walk.offset_motor[16] < -Offset_Angle_Limit)
    tune2walk.offset_motor[16] = -Offset_Angle_Limit;
  if (tune2walk.offset_motor[17] > Offset_Angle_Limit)
    tune2walk.offset_motor[17] = +Offset_Angle_Limit;
  else if (tune2walk.offset_motor[17] < -Offset_Angle_Limit)
    tune2walk.offset_motor[17] = -Offset_Angle_Limit;
  if (tune2walk.offset_motor[18] > Offset_Angle_Limit)
    tune2walk.offset_motor[18] = +Offset_Angle_Limit;
  else if (tune2walk.offset_motor[18] < -Offset_Angle_Limit)
    tune2walk.offset_motor[18] = -Offset_Angle_Limit;
  if (tune2walk.offset_motor[19] > Offset_Angle_Limit)
    tune2walk.offset_motor[19] = +Offset_Angle_Limit;
  else if (tune2walk.offset_motor[19] < -Offset_Angle_Limit)
    tune2walk.offset_motor[19] = -Offset_Angle_Limit;
  if (tune2walk.offset_motor[20] > Offset_Angle_Limit)
    tune2walk.offset_motor[20] = +Offset_Angle_Limit;
  else if (tune2walk.offset_motor[20] < -Offset_Angle_Limit)
    tune2walk.offset_motor[20] = -Offset_Angle_Limit;
  if (tune2walk.offset_motor[21] > Offset_Angle_Limit)
    tune2walk.offset_motor[21] = +Offset_Angle_Limit;
  else if (tune2walk.offset_motor[21] < -Offset_Angle_Limit)
    tune2walk.offset_motor[21] = -Offset_Angle_Limit;
}
void MainWindow::on_Motor_10_Plus_clicked()
{
  cout << "10plus" << endl;
  Motor_Num_Toss(+1, 10);
  Offset_scroll_reset(1);
  cout << "10plusend" << endl;
}
void MainWindow::on_Motor_11_Plus_clicked()
{
  Motor_Num_Toss(+1, 11);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_12_Plus_clicked()
{
  Motor_Num_Toss(+1, 12);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_13_Plus_clicked()
{
  Motor_Num_Toss(+1, 13);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_14_Plus_clicked()
{
  Motor_Num_Toss(+1, 14);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_15_Plus_clicked()
{
  Motor_Num_Toss(+1, 15);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_16_Plus_clicked()
{
  Motor_Num_Toss(+1, 16);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_17_Plus_clicked()
{
  Motor_Num_Toss(+1, 17);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_18_Plus_clicked()
{
  Motor_Num_Toss(+1, 18);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_19_Plus_clicked()
{
  Motor_Num_Toss(+1, 19);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_20_Plus_clicked()
{
  Motor_Num_Toss(+1, 20);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_21_Plus_clicked()
{
  Motor_Num_Toss(+1, 21);
  Offset_scroll_reset(1);
}

void MainWindow::on_Motor_10_Minus_clicked()
{
  Motor_Num_Toss(-1, 10);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_11_Minus_clicked()
{
  Motor_Num_Toss(-1, 11);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_12_Minus_clicked()
{
  Motor_Num_Toss(-1, 12);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_13_Minus_clicked()
{
  Motor_Num_Toss(-1, 13);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_14_Minus_clicked()
{
  Motor_Num_Toss(-1, 14);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_15_Minus_clicked()
{
  Motor_Num_Toss(-1, 15);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_16_Minus_clicked()
{
  Motor_Num_Toss(-1, 16);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_17_Minus_clicked()
{
  Motor_Num_Toss(-1, 17);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_18_Minus_clicked()
{
  Motor_Num_Toss(-1, 18);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_19_Minus_clicked()
{
  Motor_Num_Toss(-1, 19);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_20_Minus_clicked()
{
  Motor_Num_Toss(-1, 20);
  Offset_scroll_reset(1);
}
void MainWindow::on_Motor_21_Minus_clicked()
{
  Motor_Num_Toss(-1, 21);
  Offset_scroll_reset(1);
}

void MainWindow::on_plus_Button_0_clicked()
{
  if (motor_plus_arr[0] != 0)
  {
    for (int i = 0; i < 7; i++)
    {
      motor_plus_arr[i] = motor_plus_arr[i + 1];
      motor_plus_arr[i + 1] = 0;
    }
    if (motor_plus_cnt > 0)
      motor_plus_cnt--;
  }
  arr_update();
  Offset_scroll_reset(1);
}
void MainWindow::on_plus_Button_1_clicked()
{
  if (motor_plus_arr[1] != 0)
  {
    for (int i = 1; i < 7; i++)
    {
      motor_plus_arr[i] = motor_plus_arr[i + 1];
      motor_plus_arr[i + 1] = 0;
    }
    motor_plus_cnt--;
  }
  arr_update();
  Offset_scroll_reset(1);
}
void MainWindow::on_plus_Button_2_clicked()
{
  if (motor_plus_arr[2] != 0)
  {
    for (int i = 2; i < 7; i++)
    {
      motor_plus_arr[i] = motor_plus_arr[i + 1];
      motor_plus_arr[i + 1] = 0;
    }
    motor_plus_cnt--;
  }
  arr_update();
  Offset_scroll_reset(1);
}
void MainWindow::on_plus_Button_3_clicked()
{
  if (motor_plus_arr[3] != 0)
  {
    for (int i = 3; i < 7; i++)
    {
      motor_plus_arr[i] = motor_plus_arr[i + 1];
      motor_plus_arr[i + 1] = 0;
    }
    motor_plus_cnt--;
  }
  arr_update();
  Offset_scroll_reset(1);
}

void MainWindow::on_minus_Button_0_clicked()
{
  if (motor_minus_arr[0] != 0)
  {
    for (int i = 0; i < 7; i++)
    {
      motor_minus_arr[i] = motor_minus_arr[i + 1];
      motor_minus_arr[i + 1] = 0;
    }
    if (motor_minus_cnt > 0)
      motor_minus_cnt--;
  }
  arr_update();
  Offset_scroll_reset(1);
}
void MainWindow::on_minus_Button_1_clicked()
{
  if (motor_minus_arr[1] != 0)
  {
    for (int i = 1; i < 7; i++)
    {
      motor_minus_arr[i] = motor_minus_arr[i + 1];
      motor_minus_arr[i + 1] = 0;
    }
    motor_minus_cnt--;
  }
  arr_update();
  Offset_scroll_reset(1);
}
void MainWindow::on_minus_Button_2_clicked()
{
  if (motor_minus_arr[2] != 0)
  {
    for (int i = 2; i < 7; i++)
    {
      motor_minus_arr[i] = motor_minus_arr[i + 1];
      motor_minus_arr[i + 1] = 0;
    }
    motor_minus_cnt--;
  }
  arr_update();
  Offset_scroll_reset(1);
}
void MainWindow::on_minus_Button_3_clicked()
{
  if (motor_minus_arr[3] != 0)
  {
    for (int i = 3; i < 7; i++)
    {
      motor_minus_arr[i] = motor_minus_arr[i + 1];
      motor_minus_arr[i + 1] = 0;
    }
    motor_minus_cnt--;
  }
  arr_update();
  Offset_scroll_reset(1);
}

void MainWindow::on_Angle_horizontalScrollBar_valueChanged(int value)
{
  Offset_Scroll_Value = value;
  cout << "value" << value << endl;
  ui->Angle_Control_Box->setValue(Offset_Scroll_Value);
}
void MainWindow::on_Angle_Control_Box_valueChanged(double arg1)
{
  cout << "111111111" << endl;
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  Offset_Scroll_Value = arg1;
  ui->Angle_horizontalScrollBar->setValue(Offset_Scroll_Value);

  for (int i = 0; i < 3; i++)
  {
    if (motor_plus_arr[i] == 10)
      tune2walk.offset_motor[10] = offset_past_motor[10] + (arr_num_cnt(10, motor_plus_arr, 8) * Offset_Scroll_Value);
    else if (motor_plus_arr[i] == 11)
      tune2walk.offset_motor[11] = offset_past_motor[11] + (arr_num_cnt(11, motor_plus_arr, 8) * Offset_Scroll_Value);
    else if (motor_plus_arr[i] == 12)
      tune2walk.offset_motor[12] = offset_past_motor[12] + (arr_num_cnt(12, motor_plus_arr, 8) * Offset_Scroll_Value);
    else if (motor_plus_arr[i] == 13)
      tune2walk.offset_motor[13] = offset_past_motor[13] + (arr_num_cnt(13, motor_plus_arr, 8) * Offset_Scroll_Value);
    else if (motor_plus_arr[i] == 14)
      tune2walk.offset_motor[14] = offset_past_motor[14] + (arr_num_cnt(14, motor_plus_arr, 8) * Offset_Scroll_Value);
    else if (motor_plus_arr[i] == 15)
      tune2walk.offset_motor[15] = offset_past_motor[15] + (arr_num_cnt(15, motor_plus_arr, 8) * Offset_Scroll_Value);
    else if (motor_plus_arr[i] == 16)
      tune2walk.offset_motor[16] = offset_past_motor[16] + (arr_num_cnt(16, motor_plus_arr, 8) * Offset_Scroll_Value);
    else if (motor_plus_arr[i] == 17)
      tune2walk.offset_motor[17] = offset_past_motor[17] + (arr_num_cnt(17, motor_plus_arr, 8) * Offset_Scroll_Value);
    else if (motor_plus_arr[i] == 18)
      tune2walk.offset_motor[18] = offset_past_motor[18] + (arr_num_cnt(18, motor_plus_arr, 8) * Offset_Scroll_Value);
    else if (motor_plus_arr[i] == 19)
      tune2walk.offset_motor[19] = offset_past_motor[19] + (arr_num_cnt(19, motor_plus_arr, 8) * Offset_Scroll_Value);
    else if (motor_plus_arr[i] == 20)
      tune2walk.offset_motor[20] = offset_past_motor[20] + (arr_num_cnt(20, motor_plus_arr, 8) * Offset_Scroll_Value);
    else if (motor_plus_arr[i] == 21)
      tune2walk.offset_motor[21] = offset_past_motor[21] + (arr_num_cnt(21, motor_plus_arr, 8) * Offset_Scroll_Value);

    if (motor_minus_arr[i] == 10)
      tune2walk.offset_motor[10] = offset_past_motor[10] - (arr_num_cnt(10, motor_minus_arr, 8) * Offset_Scroll_Value);
    else if (motor_minus_arr[i] == 11)
      tune2walk.offset_motor[11] = offset_past_motor[11] - (arr_num_cnt(11, motor_minus_arr, 8) * Offset_Scroll_Value);
    else if (motor_minus_arr[i] == 12)
      tune2walk.offset_motor[12] = offset_past_motor[12] - (arr_num_cnt(12, motor_minus_arr, 8) * Offset_Scroll_Value);
    else if (motor_minus_arr[i] == 13)
      tune2walk.offset_motor[13] = offset_past_motor[13] - (arr_num_cnt(13, motor_minus_arr, 8) * Offset_Scroll_Value);
    else if (motor_minus_arr[i] == 14)
      tune2walk.offset_motor[14] = offset_past_motor[14] - (arr_num_cnt(14, motor_minus_arr, 8) * Offset_Scroll_Value);
    else if (motor_minus_arr[i] == 15)
      tune2walk.offset_motor[15] = offset_past_motor[15] - (arr_num_cnt(15, motor_minus_arr, 8) * Offset_Scroll_Value);
    else if (motor_minus_arr[i] == 16)
      tune2walk.offset_motor[16] = offset_past_motor[16] - (arr_num_cnt(16, motor_minus_arr, 8) * Offset_Scroll_Value);
    else if (motor_minus_arr[i] == 17)
      tune2walk.offset_motor[17] = offset_past_motor[17] - (arr_num_cnt(17, motor_minus_arr, 8) * Offset_Scroll_Value);
    else if (motor_minus_arr[i] == 18)
      tune2walk.offset_motor[18] = offset_past_motor[18] - (arr_num_cnt(18, motor_minus_arr, 8) * Offset_Scroll_Value);
    else if (motor_minus_arr[i] == 19)
      tune2walk.offset_motor[19] = offset_past_motor[19] - (arr_num_cnt(19, motor_minus_arr, 8) * Offset_Scroll_Value);
    else if (motor_minus_arr[i] == 20)
      tune2walk.offset_motor[20] = offset_past_motor[20] - (arr_num_cnt(20, motor_minus_arr, 8) * Offset_Scroll_Value);
    else if (motor_minus_arr[i] == 21)
      tune2walk.offset_motor[21] = offset_past_motor[21] - (arr_num_cnt(21, motor_minus_arr, 8) * Offset_Scroll_Value);
    // if (motor_plus_arr[i] == i)
    //   tune2walk.offset_motor[i] = offset_past_motor[i] + (arr_num_cnt(i, motor_plus_arr, 8)*Offset_Scroll_Value);
    // if (motor_minus_arr[i] == i)
    //   tune2walk.offset_motor[i] = offset_past_motor[i] - (arr_num_cnt(i, motor_minus_arr, 8) * Offset_Scroll_Value);
  }
  cout << "kkkkk" << endl; // offset_past_motor[i] + (arr_num_cnt(i, motor_plus_arr, 8)*Offset_Scroll_Value)<<endl;
  motor_limit_control();
  angle_update();
}
void MainWindow::on_offset_O_Button_clicked()
{
  cout << "0 bt" << endl;
  for (int i = 0; i < 3; i++)
  {
    for (int j = 10; j < 22; j++)
    {
      if (motor_plus_arr[i] == i)
        tune2walk.offset_motor[i] = offset_past_motor[i] + (arr_num_cnt(i, motor_plus_arr, 8) * Offset_Scroll_Value);

      if (motor_minus_arr[i] == 10)
        tune2walk.offset_motor[i] = offset_past_motor[i] - (arr_num_cnt(i, motor_minus_arr, 8) * Offset_Scroll_Value);
    }
  }
  motor_limit_control();
  angle_update();
  past_motor_update();

  Offset_scroll_reset(0);

  arr_reset();
}
void MainWindow::on_offset_X_Button_clicked()
{
  cout << "x bt" << endl;
  for (int i = 0; i < 3; i++)
  {
    for (int j = 10; j < 22; j++)
    {
      if (motor_plus_arr[i] == i)
        tune2walk.offset_motor[i] = offset_past_motor[i];
      if (motor_minus_arr[i] == i)
        tune2walk.offset_motor[i] = offset_past_motor[i];
    }
  }
  motor_limit_control();
  angle_update();

  Offset_Scroll_Value = 0;
  ui->Angle_Control_Box->setValue(Offset_Scroll_Value);
  ui->Angle_horizontalScrollBar->setValue(Offset_Scroll_Value);

  arr_reset();
}

void MainWindow::on_Stand_Button_clicked()
{
  arr_reset();
  Motor_Num_Toss(+1, 14);
  Motor_Num_Toss(-1, 16);
  Motor_Num_Toss(-1, 16);
  Motor_Num_Toss(+1, 18);

  Motor_Num_Toss(+1, 15);
  Motor_Num_Toss(-1, 17);
  Motor_Num_Toss(-1, 17);
  Motor_Num_Toss(+1, 19);
  Offset_scroll_reset(1);
}
void MainWindow::on_Stand_Left_Button_clicked()
{
  arr_reset();
  Motor_Num_Toss(+1, 14);
  Motor_Num_Toss(-1, 16);
  Motor_Num_Toss(-1, 16);
  Motor_Num_Toss(+1, 18);
  Offset_scroll_reset(1);
}
void MainWindow::on_Stand_Right_Button_clicked()
{
  arr_reset();
  Motor_Num_Toss(+1, 15);
  Motor_Num_Toss(-1, 17);
  Motor_Num_Toss(-1, 17);
  Motor_Num_Toss(+1, 19);
  Offset_scroll_reset(1);
}
void MainWindow::on_Stretch_Button_clicked()
{
  arr_reset();
  Motor_Num_Toss(+1, 10);
  Motor_Num_Toss(-1, 11);

  Motor_Num_Toss(-1, 20);
  Motor_Num_Toss(+1, 21);
  Offset_scroll_reset(1);
}
void MainWindow::on_Percentage_of_10_Motor_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > 0)
    arg1 = 0.1;
  if (arg1 == 0.0)
    arg1 = 1.0;
  tune2walk.percentage_of_ik_motor[10] = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Percentage_of_11_Motor_Box_valueChanged(double arg1)
{

  if (arg1 < 0.1 && arg1 > 0)
    arg1 = 0.1;
  if (arg1 == 0.0)
    arg1 = 1.0;
  tune2walk.percentage_of_ik_motor[11] = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Percentage_of_12_Motor_Box_valueChanged(double arg1)
{

  if (arg1 < 0.1 && arg1 > 0)
    arg1 = 0.1;
  if (arg1 == 0.0)
    arg1 = 1.0;
  tune2walk.percentage_of_ik_motor[12] = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Percentage_of_13_Motor_Box_valueChanged(double arg1)
{

  if (arg1 < 0.1 && arg1 > 0)
    arg1 = 0.1;
  if (arg1 == 0.0)
    arg1 = 1.0;
  tune2walk.percentage_of_ik_motor[13] = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Percentage_of_14_Motor_Box_valueChanged(double arg1)
{

  if (arg1 < 0.1 && arg1 > 0)
    arg1 = 0.1;
  if (arg1 == 0.0)
    arg1 = 1.0;
  tune2walk.percentage_of_ik_motor[14] = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Percentage_of_15_Motor_Box_valueChanged(double arg1)
{

  if (arg1 < 0.1 && arg1 > 0)
    arg1 = 0.1;
  if (arg1 == 0.0)
    arg1 = 1.0;
  tune2walk.percentage_of_ik_motor[15] = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Percentage_of_16_Motor_Box_valueChanged(double arg1)
{

  if (arg1 < 0.1 && arg1 > 0)
    arg1 = 0.1;
  if (arg1 == 0.0)
    arg1 = 1.0;
  tune2walk.percentage_of_ik_motor[16] = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Percentage_of_17_Motor_Box_valueChanged(double arg1)
{

  if (arg1 < 0.1 && arg1 > 0)
    arg1 = 0.1;
  if (arg1 == 0.0)
    arg1 = 1.0;
  tune2walk.percentage_of_ik_motor[17] = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Percentage_of_18_Motor_Box_valueChanged(double arg1)
{

  if (arg1 < 0.1 && arg1 > 0)
    arg1 = 0.1;
  if (arg1 == 0.0)
    arg1 = 1.0;
  tune2walk.percentage_of_ik_motor[18] = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Percentage_of_19_Motor_Box_valueChanged(double arg1)
{

  if (arg1 < 0.1 && arg1 > 0)
    arg1 = 0.1;
  if (arg1 == 0.0)
    arg1 = 1.0;
  tune2walk.percentage_of_ik_motor[19] = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Percentage_of_20_Motor_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > 0)
    arg1 = 0.1;
  if (arg1 == 0.0)
    arg1 = 1.0;
  tune2walk.percentage_of_ik_motor[20] = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Percentage_of_21_Motor_Box_valueChanged(double arg1)
{

  if (arg1 < 0.1 && arg1 > 0)
    arg1 = 0.1;
  if (arg1 == 0.0)
    arg1 = 1.0;
  tune2walk.percentage_of_ik_motor[21] = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Right_Leg_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.swing_leg_right = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Right_Leg_Plus_Button_clicked()
{
  tune2walk.swing_leg_right += 1;
  if (tune2walk.swing_leg_right >= 100)
    tune2walk.swing_leg_right = 100;
  ui->Swing_Right_Leg_Box->setValue(tune2walk.swing_leg_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Right_Leg_Minus_Button_clicked()
{
  tune2walk.swing_leg_right -= 1;
  if (tune2walk.swing_leg_right <= -100)
    tune2walk.swing_leg_right = -100;
  ui->Swing_Right_Leg_Box->setValue(tune2walk.swing_leg_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Left_Leg_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.swing_leg_left = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Left_Leg_Plus_Button_clicked()
{
  tune2walk.swing_leg_left += 1;
  if (tune2walk.swing_leg_left >= 100)
    tune2walk.swing_leg_left = 100;
  ui->Swing_Left_Leg_Box->setValue(tune2walk.swing_leg_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Left_Leg_Minus_Button_clicked()
{
  tune2walk.swing_leg_left -= 1;
  if (tune2walk.swing_leg_left <= -100)
    tune2walk.swing_leg_left = -100;
  ui->Swing_Left_Leg_Box->setValue(tune2walk.swing_leg_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Both_Leg_Plus5_Button_clicked()
{
  tune2walk.swing_leg_left += 5;
  tune2walk.swing_leg_right += 5;
  if (tune2walk.swing_leg_left >= 100)
    tune2walk.swing_leg_left = 100;
  if (tune2walk.swing_leg_right >= 100)
    tune2walk.swing_leg_right = 100;
  ui->Swing_Left_Leg_Box->setValue(tune2walk.swing_leg_left);
  ui->Swing_Right_Leg_Box->setValue(tune2walk.swing_leg_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Both_Leg_Minus5_Button_clicked()
{
  tune2walk.swing_leg_left -= 5;
  tune2walk.swing_leg_right -= 5;
  if (tune2walk.swing_leg_left <= -100)
    tune2walk.swing_leg_left = -100;
  if (tune2walk.swing_leg_right <= -100)
    tune2walk.swing_leg_right = -100;
  ui->Swing_Left_Leg_Box->setValue(tune2walk.swing_leg_left);
  ui->Swing_Right_Leg_Box->setValue(tune2walk.swing_leg_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Both_Leg_Plus_Button_clicked()
{
  tune2walk.swing_leg_left += 1;
  tune2walk.swing_leg_right += 1;
  if (tune2walk.swing_leg_left >= 100)
    tune2walk.swing_leg_left = 100;
  if (tune2walk.swing_leg_right >= 100)
    tune2walk.swing_leg_right = 100;
  ui->Swing_Left_Leg_Box->setValue(tune2walk.swing_leg_left);
  ui->Swing_Right_Leg_Box->setValue(tune2walk.swing_leg_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Both_Leg_Minus_Button_clicked()
{
  tune2walk.swing_leg_left -= 1;
  tune2walk.swing_leg_right -= 1;
  if (tune2walk.swing_leg_left <= -100)
    tune2walk.swing_leg_left = -100;
  if (tune2walk.swing_leg_right <= -100)
    tune2walk.swing_leg_right = -100;
  ui->Swing_Left_Leg_Box->setValue(tune2walk.swing_leg_left);
  ui->Swing_Right_Leg_Box->setValue(tune2walk.swing_leg_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Right_Shoulder_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.swing_shoulder_right = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Right_Shoulder_Plus_Button_clicked()
{
  tune2walk.swing_shoulder_right += 1;
  if (tune2walk.swing_shoulder_right >= 100)
    tune2walk.swing_shoulder_right = 100;
  ui->Swing_Right_Shoulder_Box->setValue(tune2walk.swing_shoulder_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Right_Shoulder_Minus_Button_clicked()
{
  tune2walk.swing_shoulder_right -= 1;
  if (tune2walk.swing_shoulder_right <= -100)
    tune2walk.swing_shoulder_right = -100;
  ui->Swing_Right_Shoulder_Box->setValue(tune2walk.swing_shoulder_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Left_Shoulder_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.swing_shoulder_left = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Left_Shoulder_Plus_Button_clicked()
{
  tune2walk.swing_shoulder_left += 1;
  if (tune2walk.swing_shoulder_left >= 100)
    tune2walk.swing_shoulder_left = 100;
  ui->Swing_Left_Shoulder_Box->setValue(tune2walk.swing_shoulder_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Left_Shoulder_Minus_Button_clicked()
{
  tune2walk.swing_shoulder_left -= 1;
  if (tune2walk.swing_shoulder_left <= -100)
    tune2walk.swing_shoulder_left = -100;
  ui->Swing_Left_Shoulder_Box->setValue(tune2walk.swing_shoulder_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Swing_Both_Shoulder_Plus5_Button_clicked()
{
  tune2walk.swing_shoulder_left += 5;
  tune2walk.swing_shoulder_right += 5;
  if (tune2walk.swing_shoulder_left >= 100)
    tune2walk.swing_shoulder_left = 100;
  if (tune2walk.swing_shoulder_right >= 100)
    tune2walk.swing_shoulder_right = 100;
  ui->Swing_Left_Shoulder_Box->setValue(tune2walk.swing_shoulder_left);
  ui->Swing_Right_Shoulder_Box->setValue(tune2walk.swing_shoulder_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Swing_Both_Shoulder_Minus5_Button_clicked()
{
  tune2walk.swing_shoulder_left -= 5;
  tune2walk.swing_shoulder_right -= 5;
  if (tune2walk.swing_shoulder_left <= -100)
    tune2walk.swing_shoulder_left = -100;
  if (tune2walk.swing_shoulder_right <= -100)
    tune2walk.swing_shoulder_right = -100;
  ui->Swing_Left_Shoulder_Box->setValue(tune2walk.swing_shoulder_left);
  ui->Swing_Right_Shoulder_Box->setValue(tune2walk.swing_shoulder_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Rise_Right_Leg_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.rise_leg_right = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Rise_Right_Leg_Plus_Button_clicked()
{
  tune2walk.rise_leg_right += 1;
  if (tune2walk.rise_leg_right >= 100)
    tune2walk.rise_leg_right = 100;
  ui->Rise_Right_Leg_Box->setValue(tune2walk.rise_leg_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Rise_Right_Leg_Minus_Button_clicked()
{
  tune2walk.rise_leg_right -= 1;
  if (tune2walk.rise_leg_right <= -100)
    tune2walk.rise_leg_right = -100;
  ui->Rise_Right_Leg_Box->setValue(tune2walk.rise_leg_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Rise_Left_Leg_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.rise_leg_left = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Rise_Left_Leg_Plus_Button_clicked()
{
  tune2walk.rise_leg_left += 1;
  if (tune2walk.rise_leg_left >= 100)
    tune2walk.rise_leg_left = 100;
  ui->Rise_Left_Leg_Box->setValue(tune2walk.rise_leg_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Rise_Left_Leg_Minus_Button_clicked()
{
  tune2walk.rise_leg_left -= 1;
  if (tune2walk.rise_leg_left <= -100)
    tune2walk.rise_leg_left = -100;
  ui->Rise_Left_Leg_Box->setValue(tune2walk.rise_leg_left);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Rise_All_Leg_Plus_Button_clicked()
{
  tune2walk.rise_leg_left += 1;
  tune2walk.rise_leg_right += 1;
  if (tune2walk.rise_leg_left >= 100)
    tune2walk.rise_leg_left = 100;
  if (tune2walk.rise_leg_right >= 100)
    tune2walk.rise_leg_right = 100;
  ui->Rise_Left_Leg_Box->setValue(tune2walk.rise_leg_left);
  ui->Rise_Right_Leg_Box->setValue(tune2walk.rise_leg_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Rise_All_Leg_Minus_Button_clicked()
{
  tune2walk.rise_leg_left -= 1;
  tune2walk.rise_leg_right -= 1;
  if (tune2walk.rise_leg_left <= 0)
    tune2walk.rise_leg_left = 0;
  if (tune2walk.rise_leg_right <= 0)
    tune2walk.rise_leg_right = 0;
  ui->Rise_Left_Leg_Box->setValue(tune2walk.rise_leg_left);
  ui->Rise_Right_Leg_Box->setValue(tune2walk.rise_leg_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Rise_All_Leg_Plus5_Button_clicked()
{
  tune2walk.rise_leg_left += 5;
  tune2walk.rise_leg_right += 5;
  if (tune2walk.rise_leg_left >= 100)
    tune2walk.rise_leg_left = 100;
  if (tune2walk.rise_leg_right >= 100)
    tune2walk.rise_leg_right = 100;
  ui->Rise_Left_Leg_Box->setValue(tune2walk.rise_leg_left);
  ui->Rise_Right_Leg_Box->setValue(tune2walk.rise_leg_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Rise_All_Leg_Minus5_Button_clicked()
{
  tune2walk.rise_leg_left -= 5;
  tune2walk.rise_leg_right -= 5;
  if (tune2walk.rise_leg_left <= 0)
    tune2walk.rise_leg_left = 0;
  if (tune2walk.rise_leg_right <= 0)
    tune2walk.rise_leg_right = 0;
  ui->Rise_Left_Leg_Box->setValue(tune2walk.rise_leg_left);
  ui->Rise_Right_Leg_Box->setValue(tune2walk.rise_leg_right);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Start_Entire_Time_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.start_entire_time = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Start_Entire_Time_Plus_Button_clicked()
{
  tune2walk.start_entire_time += 10;
  if (tune2walk.start_entire_time >= 200)
    tune2walk.start_entire_time = 200;
  ui->Start_Entire_Time_Box->setValue(tune2walk.start_entire_time);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Start_Entire_Time_Minus_Button_clicked()
{
  tune2walk.start_entire_time -= 10;
  if (tune2walk.start_entire_time <= 0)
    tune2walk.start_entire_time = 0;
  ui->Start_Entire_Time_Box->setValue(tune2walk.start_entire_time);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Start_Swing_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.start_swing = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Start_Swing_Plus_Button_clicked()
{
  tune2walk.start_swing += 1;
  if (tune2walk.start_swing >= 100)
    tune2walk.start_swing = 100;
  ui->Start_Swing_Box->setValue(tune2walk.start_swing);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Start_Swing_Minus_Button_clicked()
{
  tune2walk.start_swing -= 1;
  if (tune2walk.start_swing <= -100)
    tune2walk.start_swing = -100;
  ui->Start_Swing_Box->setValue(tune2walk.start_swing);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Start_Rise_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.start_rise = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Start_Rise_Plus_Button_clicked()
{
  tune2walk.start_rise += 1;
  if (tune2walk.start_rise >= 100)
    tune2walk.start_rise = 100;
  ui->Start_Rise_Box->setValue(tune2walk.start_rise);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Start_Rise_Minus_Button_clicked()
{
  tune2walk.start_rise -= 1;
  if (tune2walk.start_rise <= 0)
    tune2walk.start_rise = 0;
  ui->Start_Rise_Box->setValue(tune2walk.start_rise);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_End_Entire_Time_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.end_entire_time = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_End_Entire_Time_Plus_Button_clicked()
{
  tune2walk.end_entire_time += 10;
  if (tune2walk.end_entire_time >= 200)
    tune2walk.end_entire_time = 200;
  ui->End_Entire_Time_Box->setValue(tune2walk.end_entire_time);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_End_Entire_Time_Minus_Button_clicked()
{
  tune2walk.end_entire_time -= 10;
  if (tune2walk.end_entire_time <= 0)
    tune2walk.end_entire_time = 0;
  ui->End_Entire_Time_Box->setValue(tune2walk.end_entire_time);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_End_Swing_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.end_swing = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_End_Swing_Plus_Button_clicked()
{
  tune2walk.end_swing += 1;
  if (tune2walk.end_swing >= 100)
    tune2walk.end_swing = 100;
  ui->End_Swing_Box->setValue(tune2walk.end_swing);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_End_Swing_Minus_Button_clicked()
{
  tune2walk.end_swing -= 1;
  if (tune2walk.end_swing <= -100)
    tune2walk.end_swing = -100;
  ui->End_Swing_Box->setValue(tune2walk.end_swing);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_End_Rise_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.end_rise = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_End_Rise_Plus_Button_clicked()
{
  tune2walk.end_rise += 1;
  if (tune2walk.end_rise >= 100)
    tune2walk.end_rise = 100;
  ui->End_Rise_Box->setValue(tune2walk.end_rise);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_End_Rise_Minus_Button_clicked()
{
  tune2walk.end_rise -= 1;
  if (tune2walk.end_rise <= 0)
    tune2walk.end_rise = 0;
  ui->End_Rise_Box->setValue(tune2walk.end_rise);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_IK_Flag_Button_clicked()
{
  static bool Flag = true;
  if (Flag)
  {
    ui->IK_Flag_Button->setText("STOP");
    tune2walk.ik_flag = 1;
    Flag = false;
    cout << "WALK" << endl;
  }
  else if (!Flag)
  {
    ui->IK_Flag_Button->setText("WALK");
    tune2walk.ik_flag = 0;
    Flag = true;
    cout << "STOP" << endl;
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_CP_Flag_Button_clicked()
{
}
void MainWindow::on_Tuning_X_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.tuning_x = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Tuning_Side_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.tuning_side = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Tuning_Yaw_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.tuning_yaw = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Test_X_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.test_x = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Test_Side_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.test_side = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Test_Yaw_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.test_yaw = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Value_0_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_value_0 = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Blance_Value_0_Plus_Button_clicked()
{
  tune2walk.balance_value_0 += 1;
  if (tune2walk.balance_value_0 >= 100)
    tune2walk.balance_value_0 = 100;
  ui->Balance_Value_0_Box->setValue(tune2walk.balance_value_0);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Blance_Value_0_Minus_Button_clicked()
{
  tune2walk.balance_value_0 -= 1;
  if (tune2walk.balance_value_0 <= -100)
    tune2walk.balance_value_0 = -100;
  ui->Balance_Value_0_Box->setValue(tune2walk.balance_value_0);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Balance_Pitch_GP_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_pitch_gp = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Pitch_GI_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_pitch_gi = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Pitch_GD_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_pitch_gd = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Pitch_Neg_Target_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_pitch_neg_target = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Pitch_Pos_Target_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_pitch_pos_target = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Pitch_ELIMIT_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_pitch_elimit = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Pitch_OLIMIT_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_pitch_olimit = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_checkBox_Pitch_ZMP_clicked()
{
  if (tune2walk.balance_pitch_flag == 0)
  {
    tune2walk.balance_pitch_flag = 1;
    cout << "PID running" << endl;
  }
  else
  {
    tune2walk.balance_pitch_flag = 0;
    cout << "PID OFF" << endl;
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Ankle_Pitch_GP_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_angle_pitch_gp = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Ankle_Pitch_GI_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_angle_pitch_gi = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Ankle_Pitch_GD_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_angle_pitch_gd = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Ankle_Pitch_ELIMIT_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_angle_pitch_elimit = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Ankle_Pitch_OLIMIT_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_angle_pitch_olimit = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Ankle_Pitch_Neg_Target_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_angle_pitch_neg_target = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Balance_Ankle_Pitch_Pos_Target_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_angle_pitch_pos_target = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_checkBox_Ankle_Pitch_ZMP_clicked()
{
  if (tune2walk.balance_ankle_pitch_flag == 0)
  {
    tune2walk.balance_ankle_pitch_flag = 1;
    cout << "Ankle_PID running" << endl;
  }
  else
  {
    tune2walk.balance_ankle_pitch_flag = 0;
    cout << "Ankle_PID OFF" << endl;
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Value_1_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_value_1 = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Blance_Value_1_Plus_Button_clicked()
{
  tune2walk.balance_value_1 += 1;
  if (tune2walk.balance_value_1 >= 100)
    tune2walk.balance_value_1 = 100;
  ui->Balance_Value_1_Box->setValue(tune2walk.balance_value_1);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Blance_Value_1_Minus_Button_clicked()
{
  tune2walk.balance_value_1 -= 1;
  if (tune2walk.balance_value_1 <= -100)
    tune2walk.balance_value_1 = -100;
  ui->Balance_Value_1_Box->setValue(tune2walk.balance_value_1);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Roll_GP_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_roll_gp = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Roll_GI_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_roll_gi = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Roll_GD_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_roll_gd = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Roll_ELIMIT_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_roll_elimit = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Roll_OLIMIT_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_roll_olimit = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Roll_Neg_Target_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_roll_neg_target = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Roll_Pos_Target_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_roll_pos_target = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_checkBox_Ankle_Roll_ZMP_clicked()
{
  if (tune2walk.balance_roll_flag == 0)
  {
    tune2walk.balance_roll_flag = 1;
    cout << "Roll_PID running" << endl;
  }
  else
  {
    tune2walk.balance_roll_flag = 0;
    cout << "Roll_PID OFF" << endl;
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Value_2_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_value_2 = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Blance_Value_2_Plus_Button_clicked()
{
  tune2walk.balance_value_2 += 1;
  if (tune2walk.balance_value_2 >= 100)
    tune2walk.balance_value_2 = 100;
  ui->Balance_Value_2_Box->setValue(tune2walk.balance_value_2);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Blance_Value_2_Minus_Button_clicked()
{
  tune2walk.balance_value_2 -= 1;
  if (tune2walk.balance_value_2 <= -100)
    tune2walk.balance_value_2 = -100;
  ui->Balance_Value_2_Box->setValue(tune2walk.balance_value_2);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Value_3_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_value_3 = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Blance_Value_3_Plus_Button_clicked()
{
  tune2walk.balance_value_3 += 1;
  if (tune2walk.balance_value_3 >= 100)
    tune2walk.balance_value_3 = 100;
  ui->Balance_Value_3_Box->setValue(tune2walk.balance_value_3);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Blance_Value_3_Minus_Button_clicked()
{
  tune2walk.balance_value_3 -= 1;
  if (tune2walk.balance_value_3 <= -100)
    tune2walk.balance_value_3 = -100;
  ui->Balance_Value_3_Box->setValue(tune2walk.balance_value_3);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_L_Step_Pattern_Check_Box_clicked()
{
  if (L_Step_Pattern_Check_Flag == 0)
    L_Step_Pattern_Check_Flag = 1;

  else if (L_Step_Pattern_Check_Flag == 1)
    L_Step_Pattern_Check_Flag = 0;

  cout << "L_Step_Pattern_Check_Flag = " << L_Step_Pattern_Check_Flag << endl;
}
void MainWindow::on_L_COM_Pattern_Check_Box_clicked()
{
  if (L_COM_Pattern_Check_Flag == 0)
    L_COM_Pattern_Check_Flag = 1;

  else if (L_COM_Pattern_Check_Flag == 1)
    L_COM_Pattern_Check_Flag = 0;

  cout << "L_COM_Pattern_Check_Flag = " << L_COM_Pattern_Check_Flag << endl;
}
void MainWindow::on_L_Rise_Pattern_Check_Box_clicked()
{
  if (L_Rise_Pattern_Check_Flag == 0)
    L_Rise_Pattern_Check_Flag = 1;

  else if (L_Rise_Pattern_Check_Flag == 1)
    L_Rise_Pattern_Check_Flag = 0;

  cout << "L_Rise_Pattern_Check_Flag = " << L_Rise_Pattern_Check_Flag << endl;
}
void MainWindow::on_L_Side_Pattern_Check_Box_clicked()
{
  if (L_Side_Pattern_Check_Flag == 0)
    L_Side_Pattern_Check_Flag = 1;

  else if (L_Side_Pattern_Check_Flag == 1)
    L_Side_Pattern_Check_Flag = 0;

  cout << "L_Side_Pattern_Check_Flag = " << L_Side_Pattern_Check_Flag << endl;
}

void MainWindow::on_L_Turn_Pattern_Check_Box_clicked()
{
  if (L_Turn_Pattern_Check_Flag == 0)
    L_Turn_Pattern_Check_Flag = 1;

  else if (L_Turn_Pattern_Check_Flag == 1)
    L_Turn_Pattern_Check_Flag = 0;

  cout << "L_Turn_Pattern_Check_Flag = " << L_Turn_Pattern_Check_Flag << endl;
}

void MainWindow::on_R_Step_Pattern_Check_Box_clicked()
{
  if (R_Step_Pattern_Check_Flag == 0)
    R_Step_Pattern_Check_Flag = 1;

  else if (R_Step_Pattern_Check_Flag == 1)
    R_Step_Pattern_Check_Flag = 0;

  cout << "R_Step_Pattern_Check_Flag = " << R_Step_Pattern_Check_Flag << endl;
}
void MainWindow::on_R_COM_Pattern_Check_Box_clicked()
{
  if (R_COM_Pattern_Check_Flag == 0)
    R_COM_Pattern_Check_Flag = 1;

  else if (R_COM_Pattern_Check_Flag == 1)
    R_COM_Pattern_Check_Flag = 0;

  cout << "R_COM_Pattern_Check_Flag = " << R_COM_Pattern_Check_Flag << endl;
}
void MainWindow::on_R_Rise_Pattern_Check_Box_clicked()
{
  if (R_Rise_Pattern_Check_Flag == 0)
    R_Rise_Pattern_Check_Flag = 1;

  else if (R_Rise_Pattern_Check_Flag == 1)
    R_Rise_Pattern_Check_Flag = 0;

  cout << "R_Rise_Pattern_Check_Flag = " << R_Rise_Pattern_Check_Flag << endl;
}
void MainWindow::on_R_Side_Pattern_Check_Box_clicked()
{
  if (R_Side_Pattern_Check_Flag == 0)
    R_Side_Pattern_Check_Flag = 1;

  else if (R_Side_Pattern_Check_Flag == 1)
    R_Side_Pattern_Check_Flag = 0;

  cout << "R_Side_Pattern_Check_Flag = " << R_Side_Pattern_Check_Flag << endl;
}

void MainWindow::on_R_Turn_Pattern_Check_Box_clicked()
{
  if (R_Turn_Pattern_Check_Flag == 0)
    R_Turn_Pattern_Check_Flag = 1;

  else if (R_Turn_Pattern_Check_Flag == 1)
    R_Turn_Pattern_Check_Flag = 0;

  cout << "R_Turn_Pattern_Check_Flag = " << R_Turn_Pattern_Check_Flag << endl;
}

void MainWindow::on_Reset_Button_clicked()
{
  QMessageBox Reset_Check_Box;
  Reset_Check_Box.setText("Do you really want to reset it?");
  Reset_Check_Box.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
  Reset_Check_Box.setDefaultButton(QMessageBox::Cancel);

  if (Reset_Check_Box.exec() == QMessageBox::Ok)
  {
    ui->Entire_Time_Box->setValue(tune2walk.entire_time);
    ui->Frequency_Box->setValue(tune2walk.frequency);
    ui->Center2Leg_Box->setValue(tune2walk.center2leg);
    ui->Link2Link_Box->setValue(tune2walk.link2link);
    ui->Init_Z_Up_Box->setValue(tune2walk.init_z_up);
    ui->Default_X_Right_Box->setValue(tune2walk.default_x_right);
    ui->Default_X_Left_Box->setValue(tune2walk.default_x_left);
    ui->Default_Y_Right_Box->setValue(tune2walk.default_y_right);
    ui->Default_Y_Left_Box->setValue(tune2walk.default_y_left);
    ui->Default_Z_Right_Box->setValue(tune2walk.default_z_right);
    ui->Default_Z_Left_Box->setValue(tune2walk.default_z_left);
    for (int i = 10; i < 22; i++)
    {
      tune2walk.offset_motor[i] = 0.0;
    }

    angle_update();
    past_motor_update();
    arr_reset();
    Offset_scroll_reset(0);
    ui->Percentage_of_10_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[10]);
    ui->Percentage_of_11_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[11]);
    ui->Percentage_of_12_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[12]);
    ui->Percentage_of_13_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[13]);
    ui->Percentage_of_14_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[14]);
    ui->Percentage_of_15_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[15]);
    ui->Percentage_of_16_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[16]);
    ui->Percentage_of_17_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[17]);
    ui->Percentage_of_18_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[18]);
    ui->Percentage_of_19_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[19]);
    ui->Percentage_of_20_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[20]);
    ui->Percentage_of_21_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[21]);
    ui->Swing_Right_Leg_Box->setValue(tune2walk.swing_leg_right);
    ui->Swing_Left_Leg_Box->setValue(tune2walk.swing_leg_left);
    ui->Swing_Right_Shoulder_Box->setValue(tune2walk.swing_shoulder_right);
    ui->Swing_Left_Shoulder_Box->setValue(tune2walk.swing_shoulder_left);
    ui->Rise_Right_Leg_Box->setValue(tune2walk.rise_leg_right);
    ui->Rise_Left_Leg_Box->setValue(tune2walk.rise_leg_left);
    ui->Start_Entire_Time_Box->setValue(tune2walk.start_entire_time);
    ui->Start_Swing_Box->setValue(tune2walk.start_swing);
    ui->Start_Rise_Box->setValue(tune2walk.start_rise);
    ui->End_Entire_Time_Box->setValue(tune2walk.end_entire_time);
    ui->End_Swing_Box->setValue(tune2walk.end_swing);
    ui->End_Rise_Box->setValue(tune2walk.end_rise);
    ui->Test_X_Box->setValue(tune2walk.test_x);
    ui->Test_Side_Box->setValue(tune2walk.test_side);
    ui->Test_Yaw_Box->setValue(tune2walk.test_yaw);
    ui->Tuning_X_Box->setValue(tune2walk.tuning_x);
    ui->Tuning_Side_Box->setValue(tune2walk.tuning_side);
    ui->Tuning_Yaw_Box->setValue(tune2walk.tuning_yaw);

    ui->Balance_Value_0_Box->setValue(tune2walk.balance_value_0);
    ui->Balance_Pitch_GP_Box->setValue(tune2walk.balance_pitch_gp);
    ui->Balance_Pitch_GI_Box->setValue(tune2walk.balance_pitch_gi);
    ui->Balance_Pitch_GD_Box->setValue(tune2walk.balance_pitch_gd);
    ui->Balance_Pitch_ELIMIT_Box->setValue(tune2walk.balance_pitch_elimit);
    ui->Balance_Pitch_OLIMIT_Box->setValue(tune2walk.balance_pitch_olimit);
    ui->Balance_Pitch_Neg_Target_Box->setValue(tune2walk.balance_pitch_neg_target);
    ui->Balance_Pitch_Pos_Target_Box->setValue(tune2walk.balance_pitch_pos_target);

    ui->Balance_Value_1_Box->setValue(tune2walk.balance_value_1);
    ui->Balance_Ankle_Pitch_GP_Box->setValue(tune2walk.balance_angle_pitch_gp);
    ui->Balance_Ankle_Pitch_GI_Box->setValue(tune2walk.balance_angle_pitch_gi);
    ui->Balance_Ankle_Pitch_GD_Box->setValue(tune2walk.balance_angle_pitch_gd);
    ui->Balance_Ankle_Pitch_ELIMIT_Box->setValue(tune2walk.balance_angle_pitch_elimit);
    ui->Balance_Ankle_Pitch_OLIMIT_Box->setValue(tune2walk.balance_angle_pitch_olimit);
    ui->Balance_Ankle_Pitch_Neg_Target_Box->setValue(tune2walk.balance_angle_pitch_neg_target);
    ui->Balance_Ankle_Pitch_Pos_Target_Box->setValue(tune2walk.balance_angle_pitch_pos_target);

    ui->Balance_Value_2_Box->setValue(tune2walk.balance_value_2);
    ui->Balance_Value_3_Box->setValue(tune2walk.balance_value_3);
    ui->Balance_Roll_GP_Box->setValue(tune2walk.balance_roll_gp);
    ui->Balance_Roll_GI_Box->setValue(tune2walk.balance_roll_gi);
    ui->Balance_Roll_GD_Box->setValue(tune2walk.balance_roll_gd);
    ui->Balance_Roll_ELIMIT_Box->setValue(tune2walk.balance_roll_elimit);
    ui->Balance_Roll_OLIMIT_Box->setValue(tune2walk.balance_roll_olimit);
    ui->Balance_Roll_Neg_Target_Box->setValue(tune2walk.balance_roll_neg_target);
    ui->Balance_Roll_Pos_Target_Box->setValue(tune2walk.balance_roll_pos_target);

    //////////////////imu_parameter////////////////////////////////
    ui->Balance_Value_0_Box_2->setValue(tune2walk.balance_value_4);
    ui->Balance_Value_1_Box_2->setValue(tune2walk.balance_value_5);
    ui->Balance_Pitch_GP_IMU_Box->setValue(tune2walk.balance_pitch_gp_imu);
    ui->Balance_Pitch_GI_IMU_Box->setValue(tune2walk.balance_pitch_gi_imu);
    ui->Balance_Pitch_GD_IMU_Box->setValue(tune2walk.balance_pitch_gd_imu);
    ui->Balance_Pitch_ELIMIT_IMU_Box->setValue(tune2walk.balance_pitch_elimit_imu);
    ui->Balance_Pitch_OLIMIT_IMU_Box->setValue(tune2walk.balance_pitch_olimit_imu);
    ui->Balance_Pitch_Neg_Target_IMU_Box->setValue(tune2walk.balance_pitch_neg_target_imu);
    ui->Balance_Pitch_Pos_Target_IMU_Box->setValue(tune2walk.balance_pitch_pos_target_imu);

    ui->Balance_Roll_GP_IMU_Box->setValue(tune2walk.balance_roll_gp_imu);
    ui->Balance_Roll_GI_IMU_Box->setValue(tune2walk.balance_roll_gi_imu);
    ui->Balance_Roll_GD_IMU_Box->setValue(tune2walk.balance_roll_gd_imu);
    ui->Balance_Roll_ELIMIT_IMU_Box->setValue(tune2walk.balance_roll_elimit_imu);
    ui->Balance_Roll_OLIMIT_IMU_Box->setValue(tune2walk.balance_roll_olimit_imu);
    ui->Balance_Roll_Neg_Target_IMU_Box->setValue(tune2walk.balance_roll_neg_target_imu);
    ui->Balance_Roll_Pos_Target_IMU_Box->setValue(tune2walk.balance_roll_pos_target_imu);
    //////////////////////////////////K_Value//////////////////////////////////////////////////
    ui->Pos_XRSpinBox->setValue(tune2walk.first_pos_xr);
    ui->Neg_XRSpinBox->setValue(tune2walk.first_neg_xr);
    ui->Pos_SideRSpinBox->setValue(tune2walk.first_pos_side_r);
    ui->Neg_SideRSpinBox->setValue(tune2walk.first_neg_side_r);
    ui->Pos_YawRSpinBox->setValue(tune2walk.first_pos_yaw_r);
    ui->Neg_YawRSpinBox->setValue(tune2walk.first_neg_yaw_r);
    ui->Pos_XLSpinBox->setValue(tune2walk.first_pos_xl);
    ui->Neg_XLSpinBox->setValue(tune2walk.first_neg_xl);
    ui->Pos_SideLSpinBox->setValue(tune2walk.first_pos_side_l);
    ui->Neg_SideLSpinBox->setValue(tune2walk.first_neg_side_l);
    ui->Pos_YawLSpinBox->setValue(tune2walk.first_pos_yaw_l);
    ui->Neg_YawLSpinBox->setValue(tune2walk.first_neg_yaw_l);
    ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.first_pos_side_r_swing_minus);
    ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.first_neg_side_r_swing_minus);
    ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.first_pos_side_l_swing_minus);
    ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.first_neg_side_l_swing_minus);

    ui->First_min->setValue(tune2walk.first_min);
    ui->First_max->setValue(tune2walk.first_max);
    ui->Second_min->setValue(tune2walk.second_min);
    ui->Second_max->setValue(tune2walk.second_max);
    ui->Third_min->setValue(tune2walk.third_min);
    ui->Third_max->setValue(tune2walk.third_max);
    ui->Fourth_min->setValue(tune2walk.fourth_min);
    ui->Fourth_max->setValue(tune2walk.fourth_max);

    ui->Landing_Time_Control_flag->setChecked(tune2walk.landing_time_control_flag);

    cout << "Reset::" << endl
         << endl;
  }
}

void MainWindow::on_Open_Button_clicked()
{
  QString fileName = QFileDialog::getOpenFileName(this, tr("Open file"), "/home/robit/colcon_ws/src/tune_walk/work");

  if (fileName.isEmpty() == true)
  {
    qDebug() << "Load Cancel";
  }

  else
  {
    for (int i = 0; i < 2; i++)
    {
      std::ifstream is;
      is.open(fileName.toStdString().c_str());

      is >> tune2walk.entire_time;
      is >> tune2walk.frequency;
      is >> tune2walk.ratio_check_flag;
      is >> tune2walk.default_x_right;
      is >> tune2walk.default_x_left;
      is >> tune2walk.default_y_right;
      is >> tune2walk.default_y_left;
      is >> tune2walk.default_z_right;
      is >> tune2walk.default_z_left;
      for (int i = 10; i < 22; i++)
      {
        is >> tune2walk.offset_motor[i];
      }
      is >> tune2walk.swing_leg_right;
      is >> tune2walk.swing_leg_left;
      is >> tune2walk.swing_shoulder_right;
      is >> tune2walk.swing_shoulder_left;
      is >> tune2walk.rise_leg_right;
      is >> tune2walk.rise_leg_left;
      is >> tune2walk.start_entire_time;
      is >> tune2walk.start_swing;
      is >> tune2walk.start_rise;
      is >> tune2walk.end_entire_time;
      is >> tune2walk.end_swing;
      is >> tune2walk.end_rise;
      is >> tune2walk.tuning_x;
      is >> tune2walk.tuning_side;
      is >> tune2walk.tuning_yaw;

      is >> tune2walk.balance_value_0;
      is >> tune2walk.balance_pitch_gp;
      is >> tune2walk.balance_pitch_gi;
      is >> tune2walk.balance_pitch_gd;
      is >> tune2walk.balance_pitch_elimit;
      is >> tune2walk.balance_pitch_olimit;
      is >> tune2walk.balance_pitch_neg_target;
      is >> tune2walk.balance_pitch_pos_target;

      is >> tune2walk.balance_value_1;
      is >> tune2walk.balance_angle_pitch_gp;
      is >> tune2walk.balance_angle_pitch_gi;
      is >> tune2walk.balance_angle_pitch_gd;
      is >> tune2walk.balance_angle_pitch_elimit;
      is >> tune2walk.balance_angle_pitch_olimit;
      is >> tune2walk.balance_angle_pitch_neg_target;
      is >> tune2walk.balance_angle_pitch_pos_target;

      is >> tune2walk.balance_value_2;
      is >> tune2walk.balance_roll_gp;
      is >> tune2walk.balance_roll_gi;
      is >> tune2walk.balance_roll_gd;
      is >> tune2walk.balance_roll_elimit;
      is >> tune2walk.balance_roll_olimit;
      is >> tune2walk.balance_roll_neg_target;
      is >> tune2walk.balance_roll_pos_target;

      is >> tune2walk.balance_value_3;

      //////////////////imu_parameter////////////////////////////////

      is >> tune2walk.balance_value_4;
      is >> tune2walk.balance_value_5;
      is >> tune2walk.balance_pitch_gp_imu;
      is >> tune2walk.balance_pitch_gi_imu;
      is >> tune2walk.balance_pitch_gd_imu;
      is >> tune2walk.balance_pitch_elimit_imu;
      is >> tune2walk.balance_pitch_olimit_imu;
      is >> tune2walk.balance_pitch_neg_target_imu;
      is >> tune2walk.balance_pitch_pos_target_imu;

      is >> tune2walk.balance_roll_gp_imu;
      is >> tune2walk.balance_roll_gi_imu;
      is >> tune2walk.balance_roll_gd_imu;
      is >> tune2walk.balance_roll_elimit_imu;
      is >> tune2walk.balance_roll_olimit_imu;
      is >> tune2walk.balance_roll_neg_target_imu;
      is >> tune2walk.balance_roll_pos_target_imu;
      ///////////////////////////////////////////////////////////////
      is >> tune2walk.center2leg;
      is >> tune2walk.link2link;
      is >> tune2walk.init_z_up;
      is >> tune2walk.percentage_of_ik_motor[10];
      is >> tune2walk.percentage_of_ik_motor[11];
      is >> tune2walk.percentage_of_ik_motor[12];
      is >> tune2walk.percentage_of_ik_motor[13];
      is >> tune2walk.percentage_of_ik_motor[14];
      is >> tune2walk.percentage_of_ik_motor[15];
      is >> tune2walk.percentage_of_ik_motor[16];
      is >> tune2walk.percentage_of_ik_motor[17];
      is >> tune2walk.percentage_of_ik_motor[18];
      is >> tune2walk.percentage_of_ik_motor[19];
      is >> tune2walk.percentage_of_ik_motor[20];
      is >> tune2walk.percentage_of_ik_motor[21];

      ////////////////////////////////K_Value///////////////////////////
      is >> tune2walk.first_pos_xr;
      is >> tune2walk.first_neg_xr;
      is >> tune2walk.first_pos_side_r;
      is >> tune2walk.first_neg_side_r;
      is >> tune2walk.first_pos_yaw_r;
      is >> tune2walk.first_neg_yaw_r;
      is >> tune2walk.first_pos_xl;
      is >> tune2walk.first_neg_xl;
      is >> tune2walk.first_pos_side_l;
      is >> tune2walk.first_neg_side_l;
      is >> tune2walk.first_pos_yaw_l;
      is >> tune2walk.first_neg_yaw_l;
      is >> tune2walk.first_pos_side_r_swing_minus;
      is >> tune2walk.first_neg_side_r_swing_minus;
      is >> tune2walk.first_pos_side_l_swing_minus;
      is >> tune2walk.first_neg_side_l_swing_minus;
      is >> tune2walk.first_min;
      is >> tune2walk.first_max;

      is >> tune2walk.second_pos_xr;
      is >> tune2walk.second_neg_xr;
      is >> tune2walk.second_pos_side_r;
      is >> tune2walk.second_neg_side_r;
      is >> tune2walk.second_pos_yaw_r;
      is >> tune2walk.second_neg_yaw_r;
      is >> tune2walk.second_pos_xl;
      is >> tune2walk.second_neg_xl;
      is >> tune2walk.second_pos_side_l;
      is >> tune2walk.second_neg_side_l;
      is >> tune2walk.second_pos_yaw_l;
      is >> tune2walk.second_neg_yaw_l;
      is >> tune2walk.second_pos_side_r_swing_minus;
      is >> tune2walk.second_neg_side_r_swing_minus;
      is >> tune2walk.second_pos_side_l_swing_minus;
      is >> tune2walk.second_neg_side_l_swing_minus;
      is >> tune2walk.second_min;
      is >> tune2walk.second_max;

      is >> tune2walk.third_pos_xr;
      is >> tune2walk.third_neg_xr;
      is >> tune2walk.third_pos_side_r;
      is >> tune2walk.third_neg_side_r;
      is >> tune2walk.third_pos_yaw_r;
      is >> tune2walk.third_neg_yaw_r;
      is >> tune2walk.third_pos_xl;
      is >> tune2walk.third_neg_xl;
      is >> tune2walk.third_pos_side_l;
      is >> tune2walk.third_neg_side_l;
      is >> tune2walk.third_pos_yaw_l;
      is >> tune2walk.third_neg_yaw_l;
      is >> tune2walk.third_pos_side_r_swing_minus;
      is >> tune2walk.third_neg_side_r_swing_minus;
      is >> tune2walk.third_pos_side_l_swing_minus;
      is >> tune2walk.third_neg_side_l_swing_minus;
      is >> tune2walk.third_min;
      is >> tune2walk.third_max;

      is >> tune2walk.fourth_pos_xr;
      is >> tune2walk.fourth_neg_xr;
      is >> tune2walk.fourth_pos_side_r;
      is >> tune2walk.fourth_neg_side_r;
      is >> tune2walk.fourth_pos_yaw_r;
      is >> tune2walk.fourth_neg_yaw_r;
      is >> tune2walk.fourth_pos_xl;
      is >> tune2walk.fourth_neg_xl;
      is >> tune2walk.fourth_pos_side_l;
      is >> tune2walk.fourth_neg_side_l;
      is >> tune2walk.fourth_pos_yaw_l;
      is >> tune2walk.fourth_neg_yaw_l;
      is >> tune2walk.fourth_pos_side_r_swing_minus;
      is >> tune2walk.fourth_neg_side_r_swing_minus;
      is >> tune2walk.fourth_pos_side_l_swing_minus;
      is >> tune2walk.fourth_neg_side_l_swing_minus;
      is >> tune2walk.fourth_min;
      is >> tune2walk.fourth_max;

      is >> tune2walk.landing_time_control_flag;

      is >> tune2walk.balance_pitch_flag_imu;
      is >> tune2walk.balance_roll_flag_imu;
      is >> tune2walk.balance_pitch_flag;
      is >> tune2walk.balance_ankle_pitch_flag;
      is >> tune2walk.balance_roll_flag;

      is.close();

      ui->Entire_Time_Box->setValue(tune2walk.entire_time);
      ui->Frequency_Box->setValue(tune2walk.frequency);
      ui->Center2Leg_Box->setValue(tune2walk.center2leg);
      ui->Link2Link_Box->setValue(tune2walk.link2link);
      ui->Init_Z_Up_Box->setValue(tune2walk.init_z_up);
      ui->Default_X_Right_Box->setValue(tune2walk.default_x_right);
      ui->Default_X_Left_Box->setValue(tune2walk.default_x_left);
      ui->Default_Y_Right_Box->setValue(tune2walk.default_y_right);
      ui->Default_Y_Left_Box->setValue(tune2walk.default_y_left);
      ui->Default_Z_Right_Box->setValue(tune2walk.default_z_right);
      ui->Default_Z_Left_Box->setValue(tune2walk.default_z_left);
      angle_update();
      past_motor_update();
      arr_reset();
      Offset_scroll_reset(0);
      ui->Percentage_of_10_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[10]);
      ui->Percentage_of_11_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[11]);
      ui->Percentage_of_12_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[12]);
      ui->Percentage_of_13_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[13]);
      ui->Percentage_of_14_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[14]);
      ui->Percentage_of_15_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[15]);
      ui->Percentage_of_16_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[16]);
      ui->Percentage_of_17_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[17]);
      ui->Percentage_of_18_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[18]);
      ui->Percentage_of_19_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[19]);
      ui->Percentage_of_20_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[20]);
      ui->Percentage_of_21_Motor_Box->setValue(tune2walk.percentage_of_ik_motor[21]);

      ui->Swing_Right_Leg_Box->setValue(tune2walk.swing_leg_right);
      ui->Swing_Left_Leg_Box->setValue(tune2walk.swing_leg_left);
      ui->Swing_Right_Shoulder_Box->setValue(tune2walk.swing_shoulder_right);
      ui->Swing_Left_Shoulder_Box->setValue(tune2walk.swing_shoulder_left);
      ui->Rise_Right_Leg_Box->setValue(tune2walk.rise_leg_right);
      ui->Rise_Left_Leg_Box->setValue(tune2walk.rise_leg_left);
      ui->Start_Entire_Time_Box->setValue(tune2walk.start_entire_time);
      ui->Start_Swing_Box->setValue(tune2walk.start_swing);
      ui->Start_Rise_Box->setValue(tune2walk.start_rise);
      ui->End_Entire_Time_Box->setValue(tune2walk.end_entire_time);
      ui->End_Swing_Box->setValue(tune2walk.end_swing);
      ui->End_Rise_Box->setValue(tune2walk.end_rise);
      ui->Test_X_Box->setValue(tune2walk.test_x);
      ui->Test_Side_Box->setValue(tune2walk.test_side);
      ui->Test_Yaw_Box->setValue(tune2walk.test_yaw);
      ui->Tuning_X_Box->setValue(tune2walk.tuning_x);
      ui->Tuning_Side_Box->setValue(tune2walk.tuning_side);
      ui->Tuning_Yaw_Box->setValue(tune2walk.tuning_yaw);

      ui->Balance_Value_0_Box->setValue(tune2walk.balance_value_0);
      ui->Balance_Pitch_GP_Box->setValue(tune2walk.balance_pitch_gp);
      ui->Balance_Pitch_GI_Box->setValue(tune2walk.balance_pitch_gi);
      ui->Balance_Pitch_GD_Box->setValue(tune2walk.balance_pitch_gd);
      ui->Balance_Pitch_ELIMIT_Box->setValue(tune2walk.balance_pitch_elimit);
      ui->Balance_Pitch_OLIMIT_Box->setValue(tune2walk.balance_pitch_olimit);
      ui->Balance_Pitch_Neg_Target_Box->setValue(tune2walk.balance_pitch_neg_target);
      ui->Balance_Pitch_Pos_Target_Box->setValue(tune2walk.balance_pitch_pos_target);

      ui->Balance_Value_1_Box->setValue(tune2walk.balance_value_1);
      ui->Balance_Ankle_Pitch_GP_Box->setValue(tune2walk.balance_angle_pitch_gp);
      ui->Balance_Ankle_Pitch_GI_Box->setValue(tune2walk.balance_angle_pitch_gi);
      ui->Balance_Ankle_Pitch_GD_Box->setValue(tune2walk.balance_angle_pitch_gd);
      ui->Balance_Ankle_Pitch_ELIMIT_Box->setValue(tune2walk.balance_angle_pitch_elimit);
      ui->Balance_Ankle_Pitch_OLIMIT_Box->setValue(tune2walk.balance_angle_pitch_olimit);
      ui->Balance_Ankle_Pitch_Neg_Target_Box->setValue(tune2walk.balance_angle_pitch_neg_target);
      ui->Balance_Ankle_Pitch_Pos_Target_Box->setValue(tune2walk.balance_angle_pitch_pos_target);

      ui->Balance_Value_2_Box->setValue(tune2walk.balance_value_2);

      ui->Balance_Value_3_Box->setValue(tune2walk.balance_value_3);

      ui->Balance_Roll_GP_Box->setValue(tune2walk.balance_roll_gp);
      ui->Balance_Roll_GI_Box->setValue(tune2walk.balance_roll_gi);
      ui->Balance_Roll_GD_Box->setValue(tune2walk.balance_roll_gd);
      ui->Balance_Roll_ELIMIT_Box->setValue(tune2walk.balance_roll_elimit);
      ui->Balance_Roll_OLIMIT_Box->setValue(tune2walk.balance_roll_olimit);
      ui->Balance_Roll_Neg_Target_Box->setValue(tune2walk.balance_roll_neg_target);
      ui->Balance_Roll_Pos_Target_Box->setValue(tune2walk.balance_roll_pos_target);

      ////////////////////////////PID_IMU_PARAMETER/////////////////////////////////////////
      ui->Balance_Value_0_Box_2->setValue(tune2walk.balance_value_4);
      ui->Balance_Value_1_Box_2->setValue(tune2walk.balance_value_5);
      ui->Balance_Pitch_GP_IMU_Box->setValue(tune2walk.balance_pitch_gp_imu);
      ui->Balance_Pitch_GI_IMU_Box->setValue(tune2walk.balance_pitch_gi_imu);
      ui->Balance_Pitch_GD_IMU_Box->setValue(tune2walk.balance_pitch_gd_imu);
      ui->Balance_Pitch_ELIMIT_IMU_Box->setValue(tune2walk.balance_pitch_elimit_imu);
      ui->Balance_Pitch_OLIMIT_IMU_Box->setValue(tune2walk.balance_pitch_olimit_imu);
      ui->Balance_Pitch_Neg_Target_IMU_Box->setValue(tune2walk.balance_pitch_neg_target_imu);
      ui->Balance_Pitch_Pos_Target_IMU_Box->setValue(tune2walk.balance_pitch_pos_target_imu);

      ui->Balance_Roll_GP_IMU_Box->setValue(tune2walk.balance_roll_gp_imu);
      ui->Balance_Roll_GI_IMU_Box->setValue(tune2walk.balance_roll_gi_imu);
      ui->Balance_Roll_GD_IMU_Box->setValue(tune2walk.balance_roll_gd_imu);
      ui->Balance_Roll_ELIMIT_IMU_Box->setValue(tune2walk.balance_roll_elimit_imu);
      ui->Balance_Roll_OLIMIT_IMU_Box->setValue(tune2walk.balance_roll_olimit_imu);
      ui->Balance_Roll_Neg_Target_IMU_Box->setValue(tune2walk.balance_roll_neg_target_imu);
      ui->Balance_Roll_Pos_Target_IMU_Box->setValue(tune2walk.balance_roll_pos_target_imu);
      //////////////////////////////////K_Value//////////////////////////////////////////////////
      ui->Pos_XRSpinBox->setValue(tune2walk.first_pos_xr);
      ui->Neg_XRSpinBox->setValue(tune2walk.first_neg_xr);
      ui->Pos_SideRSpinBox->setValue(tune2walk.first_pos_side_r);
      ui->Neg_SideRSpinBox->setValue(tune2walk.first_neg_side_r);
      ui->Pos_YawRSpinBox->setValue(tune2walk.first_pos_yaw_r);
      ui->Neg_YawRSpinBox->setValue(tune2walk.first_neg_yaw_r);
      ui->Pos_XLSpinBox->setValue(tune2walk.first_pos_xl);
      ui->Neg_XLSpinBox->setValue(tune2walk.first_neg_xl);
      ui->Pos_SideLSpinBox->setValue(tune2walk.first_pos_side_l);
      ui->Neg_SideLSpinBox->setValue(tune2walk.first_neg_side_l);
      ui->Pos_YawLSpinBox->setValue(tune2walk.first_pos_yaw_l);
      ui->Neg_YawLSpinBox->setValue(tune2walk.first_neg_yaw_l);
      ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.first_pos_side_r_swing_minus);
      ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.first_neg_side_r_swing_minus);
      ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.first_pos_side_l_swing_minus);
      ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.first_neg_side_l_swing_minus);

      ui->First_min->setValue(tune2walk.first_min);
      ui->First_max->setValue(tune2walk.first_max);
      ui->Second_min->setValue(tune2walk.second_min);
      ui->Second_max->setValue(tune2walk.second_max);
      ui->Third_min->setValue(tune2walk.third_min);
      ui->Third_max->setValue(tune2walk.third_max);
      ui->Fourth_min->setValue(tune2walk.fourth_min);
      ui->Fourth_max->setValue(tune2walk.fourth_max);

      tune2walk.control_system_flag = 1;
      ui->checkBox_Control_System->setChecked(1);
    }

    if (tune2walk.ratio_check_flag == 1)
    {
      ui->Ratio_Check_Box->setChecked(1);
      cout << "Double" << endl;
    }
    else if (tune2walk.ratio_check_flag == 0)
    {
      ui->Ratio_Check_Box->setChecked(0);
      cout << "Single" << endl;
    }

    if (tune2walk.landing_time_control_flag == 1)
    {
      ui->Landing_Time_Control_flag->setChecked(1);
      cout << "LTC_ON" << endl;
    }
    else if (tune2walk.landing_time_control_flag == 0)
    {
      ui->Landing_Time_Control_flag->setChecked(0);
      cout << "LTC_OFF" << endl;
    }

    if (tune2walk.balance_pitch_flag_imu == 1)
    {
      ui->checkBox_Pitch_IMU->setChecked(1);
      cout << "IMU_Pitch_PID_ON" << endl;
    }
    else if (tune2walk.balance_pitch_flag_imu == 0)
    {
      ui->checkBox_Pitch_IMU->setChecked(0);
      cout << "IMU_Pitch_PID_OFF" << endl;
    }

    if (tune2walk.balance_roll_flag_imu == 1)
    {
      ui->checkBox_Roll_IMU->setChecked(1);
      cout << "IMU_Roll_PID_ON" << endl;
    }
    else if (tune2walk.balance_roll_flag_imu == 0)
    {
      ui->checkBox_Roll_IMU->setChecked(0);
      cout << "IMU_Roll_PID_OFF" << endl;
    }

    if (tune2walk.balance_pitch_flag == 1)
    {
      ui->checkBox_Pitch_ZMP->setChecked(1);
      cout << "ZMP_Pitch_PID_ON" << endl;
    }
    else if (tune2walk.balance_pitch_flag == 0)
    {
      ui->checkBox_Pitch_ZMP->setChecked(0);
      cout << "ZMP_Pitch_PID_OFF" << endl;
    }

    if (tune2walk.balance_ankle_pitch_flag == 1)
    {
      ui->checkBox_Ankle_Pitch_ZMP->setChecked(1);
      cout << "ZMP_Ankle_PID_ON" << endl;
    }
    else if (tune2walk.balance_ankle_pitch_flag == 0)
    {
      ui->checkBox_Ankle_Pitch_ZMP->setChecked(0);
      cout << "ZMP_Ankle_PID_OFF" << endl;
    }

    if (tune2walk.balance_roll_flag == 1)
    {
      ui->checkBox_Ankle_Roll_ZMP->setChecked(1);
      cout << "ZMP_Roll_PID_ON" << endl;
    }
    else if (tune2walk.balance_roll_flag == 0)
    {
      ui->checkBox_Ankle_Roll_ZMP->setChecked(0);
      cout << "ZMP_Roll_PID_OFF" << endl;
    }
  }
}

void MainWindow::on_Save_Button_clicked()
{
  QString fileName = QFileDialog::getSaveFileName(this,
                                                  tr("Save file"), "src/ik_walk/work/");

  if (fileName.isEmpty() == true)
    qDebug() << "Save Cancel";

  else
  {
    QFile *file = new QFile;
    file->setFileName(fileName);
    file->open(QIODevice::WriteOnly);
    QTextStream out(file);

    out
        << tune2walk.entire_time << endl
        << tune2walk.frequency << endl
        << tune2walk.ratio_check_flag << endl
        << tune2walk.default_x_right << endl
        << tune2walk.default_x_left << endl
        << tune2walk.default_y_right << endl
        << tune2walk.default_y_left << endl
        << tune2walk.default_z_right << endl
        << tune2walk.default_z_left << endl
        << tune2walk.offset_motor[10] << endl
        << tune2walk.offset_motor[11] << endl
        << tune2walk.offset_motor[12] << endl
        << tune2walk.offset_motor[13] << endl
        << tune2walk.offset_motor[14] << endl
        << tune2walk.offset_motor[15] << endl
        << tune2walk.offset_motor[16] << endl
        << tune2walk.offset_motor[17] << endl
        << tune2walk.offset_motor[18] << endl
        << tune2walk.offset_motor[19] << endl
        << tune2walk.offset_motor[20] << endl
        << tune2walk.offset_motor[21] << endl
        << tune2walk.swing_leg_right << endl
        << tune2walk.swing_leg_left << endl
        << tune2walk.swing_shoulder_right << endl
        << tune2walk.swing_shoulder_left << endl
        << tune2walk.rise_leg_right << endl
        << tune2walk.rise_leg_left << endl
        << tune2walk.start_entire_time << endl
        << tune2walk.start_swing << endl
        << tune2walk.start_rise << endl
        << tune2walk.end_entire_time << endl
        << tune2walk.end_swing << endl
        << tune2walk.end_rise << endl
        << tune2walk.tuning_x << endl
        << tune2walk.tuning_side << endl
        << tune2walk.tuning_yaw << endl

        << tune2walk.balance_value_0 << endl
        << tune2walk.balance_pitch_gp << endl
        << tune2walk.balance_pitch_gi << endl
        << tune2walk.balance_pitch_gd << endl
        << tune2walk.balance_pitch_elimit << endl
        << tune2walk.balance_pitch_olimit << endl
        << tune2walk.balance_pitch_neg_target << endl
        << tune2walk.balance_pitch_pos_target << endl

        << tune2walk.balance_value_1 << endl
        << tune2walk.balance_angle_pitch_gp << endl
        << tune2walk.balance_angle_pitch_gi << endl
        << tune2walk.balance_angle_pitch_gd << endl
        << tune2walk.balance_angle_pitch_elimit << endl
        << tune2walk.balance_angle_pitch_olimit << endl
        << tune2walk.balance_angle_pitch_neg_target << endl
        << tune2walk.balance_angle_pitch_pos_target << endl

        << tune2walk.balance_value_2 << endl
        << tune2walk.balance_roll_gp << endl
        << tune2walk.balance_roll_gi << endl
        << tune2walk.balance_roll_gd << endl
        << tune2walk.balance_roll_elimit << endl
        << tune2walk.balance_roll_olimit << endl
        << tune2walk.balance_roll_neg_target << endl
        << tune2walk.balance_roll_pos_target << endl

        << tune2walk.balance_value_3 << endl

        ////////////////PID_IMU_PARAMETER/////////////////////
        << tune2walk.balance_value_4 << endl
        << tune2walk.balance_value_5 << endl
        << tune2walk.balance_pitch_gp_imu << endl
        << tune2walk.balance_pitch_gi_imu << endl
        << tune2walk.balance_pitch_gd_imu << endl
        << tune2walk.balance_pitch_elimit_imu << endl
        << tune2walk.balance_pitch_olimit_imu << endl
        << tune2walk.balance_pitch_neg_target_imu << endl
        << tune2walk.balance_pitch_pos_target_imu << endl

        << tune2walk.balance_roll_gp_imu << endl
        << tune2walk.balance_roll_gi_imu << endl
        << tune2walk.balance_roll_gd_imu << endl
        << tune2walk.balance_roll_elimit_imu << endl
        << tune2walk.balance_roll_olimit_imu << endl
        << tune2walk.balance_roll_neg_target_imu << endl
        << tune2walk.balance_roll_pos_target_imu << endl
        << tune2walk.center2leg << endl
        << tune2walk.link2link << endl
        << tune2walk.init_z_up << endl
        ///////////////////////////////////////////////////////
        << tune2walk.percentage_of_ik_motor[10] << endl
        << tune2walk.percentage_of_ik_motor[11] << endl
        << tune2walk.percentage_of_ik_motor[12] << endl
        << tune2walk.percentage_of_ik_motor[13] << endl
        << tune2walk.percentage_of_ik_motor[14] << endl
        << tune2walk.percentage_of_ik_motor[15] << endl
        << tune2walk.percentage_of_ik_motor[16] << endl
        << tune2walk.percentage_of_ik_motor[17] << endl
        << tune2walk.percentage_of_ik_motor[18] << endl
        << tune2walk.percentage_of_ik_motor[19] << endl
        << tune2walk.percentage_of_ik_motor[20] << endl
        << tune2walk.percentage_of_ik_motor[21] << endl

        /////////////////////K_Value///////////////////////////////
        << tune2walk.first_pos_xr << endl
        << tune2walk.first_neg_xr << endl
        << tune2walk.first_pos_side_r << endl
        << tune2walk.first_neg_side_r << endl
        << tune2walk.first_pos_yaw_r << endl
        << tune2walk.first_neg_yaw_r << endl
        << tune2walk.first_pos_xl << endl
        << tune2walk.first_neg_xl << endl
        << tune2walk.first_pos_side_l << endl
        << tune2walk.first_neg_side_l << endl
        << tune2walk.first_pos_yaw_l << endl
        << tune2walk.first_neg_yaw_l << endl
        << tune2walk.first_pos_side_r_swing_minus << endl
        << tune2walk.first_neg_side_r_swing_minus << endl
        << tune2walk.first_pos_side_l_swing_minus << endl
        << tune2walk.first_neg_side_l_swing_minus << endl
        << tune2walk.first_min << endl
        << tune2walk.first_max << endl

        << tune2walk.second_pos_xr << endl
        << tune2walk.second_neg_xr << endl
        << tune2walk.second_pos_side_r << endl
        << tune2walk.second_neg_side_r << endl
        << tune2walk.second_pos_yaw_r << endl
        << tune2walk.second_neg_yaw_r << endl
        << tune2walk.second_pos_xl << endl
        << tune2walk.second_neg_xl << endl
        << tune2walk.second_pos_side_l << endl
        << tune2walk.second_neg_side_l << endl
        << tune2walk.second_pos_yaw_l << endl
        << tune2walk.second_neg_yaw_l << endl
        << tune2walk.second_pos_side_r_swing_minus << endl
        << tune2walk.second_neg_side_r_swing_minus << endl
        << tune2walk.second_pos_side_l_swing_minus << endl
        << tune2walk.second_neg_side_l_swing_minus << endl
        << tune2walk.second_min << endl
        << tune2walk.second_max << endl

        << tune2walk.third_pos_xr << endl
        << tune2walk.third_neg_xr << endl
        << tune2walk.third_pos_side_r << endl
        << tune2walk.third_neg_side_r << endl
        << tune2walk.third_pos_yaw_r << endl
        << tune2walk.third_neg_yaw_r << endl
        << tune2walk.third_pos_xl << endl
        << tune2walk.third_neg_xl << endl
        << tune2walk.third_pos_side_l << endl
        << tune2walk.third_neg_side_l << endl
        << tune2walk.third_pos_yaw_l << endl
        << tune2walk.third_neg_yaw_l << endl
        << tune2walk.third_pos_side_r_swing_minus << endl
        << tune2walk.third_neg_side_r_swing_minus << endl
        << tune2walk.third_pos_side_l_swing_minus << endl
        << tune2walk.third_neg_side_l_swing_minus << endl
        << tune2walk.third_min << endl
        << tune2walk.third_max << endl

        << tune2walk.fourth_pos_xr << endl
        << tune2walk.fourth_neg_xr << endl
        << tune2walk.fourth_pos_side_r << endl
        << tune2walk.fourth_neg_side_r << endl
        << tune2walk.fourth_pos_yaw_r << endl
        << tune2walk.fourth_neg_yaw_r << endl
        << tune2walk.fourth_pos_xl << endl
        << tune2walk.fourth_neg_xl << endl
        << tune2walk.fourth_pos_side_l << endl
        << tune2walk.fourth_neg_side_l << endl
        << tune2walk.fourth_pos_yaw_l << endl
        << tune2walk.fourth_neg_yaw_l << endl
        << tune2walk.fourth_pos_side_r_swing_minus << endl
        << tune2walk.fourth_neg_side_r_swing_minus << endl
        << tune2walk.fourth_pos_side_l_swing_minus << endl
        << tune2walk.fourth_neg_side_l_swing_minus << endl
        << tune2walk.fourth_min << endl
        << tune2walk.fourth_max << endl

        << tune2walk.landing_time_control_flag << endl

        << tune2walk.balance_pitch_flag_imu << endl
        << tune2walk.balance_roll_flag_imu << endl
        << tune2walk.balance_pitch_flag << endl
        << tune2walk.balance_ankle_pitch_flag << endl
        << tune2walk.balance_roll_flag << endl
        << ",";
    file->close();
  }
  arr_reset();
  Offset_scroll_reset(0);
}

void MainWindow::on_checkBox_Pitch_IMU_clicked()
{
  if (tune2walk.balance_pitch_flag_imu == 0)
  {
    tune2walk.balance_pitch_flag_imu = 1;
    cout << "imu_pid_pitch_running" << endl;
  }
  else
  {
    tune2walk.balance_pitch_flag_imu = 0;
    cout << "imu_pid_pitch_OFF" << endl;
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Value_0_Plus_Button_2_clicked()
{
  tune2walk.balance_value_4 += 1;
  if (tune2walk.balance_value_4 >= 100)
    tune2walk.balance_value_4 = 100;
  ui->Balance_Value_0_Box_2->setValue(tune2walk.balance_value_4);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Balance_Value_0_Minus_Button_2_clicked()
{
  tune2walk.balance_value_4 -= 1;
  if (tune2walk.balance_value_4 <= -100)
    tune2walk.balance_value_4 = -100;
  ui->Balance_Value_0_Box_2->setValue(tune2walk.balance_value_4);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Pitch_GP_IMU_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_pitch_gp_imu = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Pitch_GI_IMU_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_pitch_gi_imu = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Pitch_GD_IMU_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_pitch_gd_imu = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Pitch_ELIMIT_IMU_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_pitch_elimit_imu = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Pitch_OLIMIT_IMU_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_pitch_olimit_imu = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Pitch_Neg_Target_IMU_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_pitch_neg_target_imu = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Pitch_Pos_Target_IMU_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_pitch_pos_target_imu = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_checkBox_Roll_IMU_clicked()
{
  if (tune2walk.balance_roll_flag_imu == 0)
  {
    tune2walk.balance_roll_flag_imu = 1;
    cout << "imu_pid_roll_running" << endl;
  }
  else
  {
    tune2walk.balance_roll_flag_imu = 0;
    cout << "imu_pid_roll_OFF" << endl;
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Value_1_Plus_Button_2_clicked()
{
  tune2walk.balance_value_5 += 1;
  if (tune2walk.balance_value_5 >= 100)
    tune2walk.balance_value_5 = 100;
  ui->Balance_Value_1_Box_2->setValue(tune2walk.balance_value_5);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Balance_Value_1_Minus_Button_2_clicked()
{
  tune2walk.balance_value_5 -= 1;
  if (tune2walk.balance_value_5 <= -100)
    tune2walk.balance_value_5 = -100;
  ui->Balance_Value_1_Box_2->setValue(tune2walk.balance_value_5);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Roll_GP_IMU_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_roll_gp_imu = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Roll_GI_IMU_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_roll_gi_imu = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Roll_GD_IMU_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_roll_gd_imu = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Roll_ELIMIT_IMU_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_roll_elimit_imu = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Roll_OLIMIT_IMU_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_roll_olimit_imu = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Roll_Neg_Target_IMU_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_roll_neg_target_imu = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Balance_Roll_Pos_Target_IMU_Box_valueChanged(double arg1)
{
  if (arg1 < 0.1 && arg1 > -0.1)
    arg1 = 0.0;
  tune2walk.balance_roll_pos_target_imu = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}

//////////////////////K_Value///////////////////////////
void MainWindow::on_Pos_XRSpinBox_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_pos_xr = arg1;
  else if (index == 1)
    tune2walk.second_pos_xr = arg1;
  else if (index == 2)
    tune2walk.third_pos_xr = arg1;
  else if (index == 3)
    tune2walk.fourth_pos_xr = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_XRSpinBox_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_neg_xr = arg1;
  else if (index == 1)
    tune2walk.second_neg_xr = arg1;
  else if (index == 2)
    tune2walk.third_neg_xr = arg1;
  else if (index == 3)
    tune2walk.fourth_neg_xr = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_SideRSpinBox_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_pos_side_r = arg1;
  else if (index == 1)
    tune2walk.second_pos_side_r = arg1;
  else if (index == 2)
    tune2walk.third_pos_side_r = arg1;
  else if (index == 3)
    tune2walk.fourth_pos_side_r = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_SideRSpinBox_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_neg_side_r = arg1;
  else if (index == 1)
    tune2walk.second_neg_side_r = arg1;
  else if (index == 2)
    tune2walk.third_neg_side_r = arg1;
  else if (index == 3)
    tune2walk.fourth_neg_side_r = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_YawRSpinBox_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_pos_yaw_r = arg1;
  else if (index == 1)
    tune2walk.second_pos_yaw_r = arg1;
  else if (index == 2)
    tune2walk.third_pos_yaw_r = arg1;
  else if (index == 3)
    tune2walk.fourth_pos_yaw_r = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_YawRSpinBox_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_neg_yaw_r = arg1;
  else if (index == 1)
    tune2walk.second_neg_yaw_r = arg1;
  else if (index == 2)
    tune2walk.third_neg_yaw_r = arg1;
  else if (index == 3)
    tune2walk.fourth_neg_yaw_r = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_XLSpinBox_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_pos_xl = arg1;
  else if (index == 1)
    tune2walk.second_pos_xl = arg1;
  else if (index == 2)
    tune2walk.third_pos_xl = arg1;
  else if (index == 3)
    tune2walk.fourth_pos_xl = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_XLSpinBox_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_neg_xl = arg1;
  else if (index == 1)
    tune2walk.second_neg_xl = arg1;
  else if (index == 2)
    tune2walk.third_neg_xl = arg1;
  else if (index == 3)
    tune2walk.fourth_neg_xl = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_SideLSpinBox_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_pos_side_l = arg1;
  else if (index == 1)
    tune2walk.second_pos_side_l = arg1;
  else if (index == 2)
    tune2walk.third_pos_side_l = arg1;
  else if (index == 3)
    tune2walk.fourth_pos_side_l = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_SideLSpinBox_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_neg_side_l = arg1;
  else if (index == 1)
    tune2walk.second_neg_side_l = arg1;
  else if (index == 2)
    tune2walk.third_neg_side_l = arg1;
  else if (index == 3)
    tune2walk.fourth_neg_side_l = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_YawLSpinBox_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_pos_yaw_l = arg1;
  else if (index == 1)
    tune2walk.second_pos_yaw_l = arg1;
  else if (index == 2)
    tune2walk.third_pos_yaw_l = arg1;
  else if (index == 3)
    tune2walk.fourth_pos_yaw_l = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_YawLSpinBox_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_neg_yaw_l = arg1;
  else if (index == 1)
    tune2walk.second_neg_yaw_l = arg1;
  else if (index == 2)
    tune2walk.third_neg_yaw_l = arg1;
  else if (index == 3)
    tune2walk.fourth_neg_yaw_l = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_XR_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_xr += 0.01;
    ui->Pos_XRSpinBox->setValue(tune2walk.first_pos_xr);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_xr += 0.01;
    ui->Pos_XRSpinBox->setValue(tune2walk.second_pos_xr);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_xr += 0.01;
    ui->Pos_XRSpinBox->setValue(tune2walk.third_pos_xr);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_xr += 0.01;
    ui->Pos_XRSpinBox->setValue(tune2walk.fourth_pos_xr);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Neg_XR_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_xr += 0.01;
    ui->Neg_XRSpinBox->setValue(tune2walk.first_neg_xr);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_xr += 0.01;
    ui->Neg_XRSpinBox->setValue(tune2walk.second_neg_xr);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_xr += 0.01;
    ui->Neg_XRSpinBox->setValue(tune2walk.third_neg_xr);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_xr += 0.01;
    ui->Neg_XRSpinBox->setValue(tune2walk.fourth_neg_xr);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_SideR_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_side_r += 0.01;
    ui->Pos_SideRSpinBox->setValue(tune2walk.first_pos_side_r);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_side_r += 0.01;
    ui->Pos_SideRSpinBox->setValue(tune2walk.second_pos_side_r);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_side_r += 0.01;
    ui->Pos_SideRSpinBox->setValue(tune2walk.third_pos_side_r);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_side_r += 0.01;
    ui->Pos_SideRSpinBox->setValue(tune2walk.fourth_pos_side_r);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_SideR_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_side_r += 0.01;
    ui->Neg_SideRSpinBox->setValue(tune2walk.first_neg_side_r);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_side_r += 0.01;
    ui->Neg_SideRSpinBox->setValue(tune2walk.second_neg_side_r);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_side_r += 0.01;
    ui->Neg_SideRSpinBox->setValue(tune2walk.third_neg_side_r);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_side_r += 0.01;
    ui->Neg_SideRSpinBox->setValue(tune2walk.fourth_neg_side_r);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_YawR_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_yaw_r += 0.01;
    ui->Pos_YawRSpinBox->setValue(tune2walk.first_pos_yaw_r);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_yaw_r += 0.01;
    ui->Pos_YawRSpinBox->setValue(tune2walk.second_pos_yaw_r);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_yaw_r += 0.01;
    ui->Pos_YawRSpinBox->setValue(tune2walk.third_pos_yaw_r);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_yaw_r += 0.01;
    ui->Pos_YawRSpinBox->setValue(tune2walk.fourth_pos_yaw_r);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_YawR_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_yaw_r += 0.01;
    ui->Neg_YawRSpinBox->setValue(tune2walk.first_neg_yaw_r);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_yaw_r += 0.01;
    ui->Neg_YawRSpinBox->setValue(tune2walk.second_neg_yaw_r);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_yaw_r += 0.01;
    ui->Neg_YawRSpinBox->setValue(tune2walk.third_neg_yaw_r);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_yaw_r += 0.01;
    ui->Neg_YawRSpinBox->setValue(tune2walk.fourth_neg_yaw_r);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_XL_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_xl += 0.01;
    ui->Pos_XLSpinBox->setValue(tune2walk.first_pos_xl);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_xl += 0.01;
    ui->Pos_XLSpinBox->setValue(tune2walk.second_pos_xl);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_xl += 0.01;
    ui->Pos_XLSpinBox->setValue(tune2walk.third_pos_xl);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_xl += 0.01;
    ui->Pos_XLSpinBox->setValue(tune2walk.fourth_pos_xl);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Neg_XL_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_xl += 0.01;
    ui->Neg_XLSpinBox->setValue(tune2walk.first_neg_xl);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_xl += 0.01;
    ui->Neg_XLSpinBox->setValue(tune2walk.second_neg_xl);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_xl += 0.01;
    ui->Neg_XLSpinBox->setValue(tune2walk.third_neg_xl);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_xl += 0.01;
    ui->Neg_XLSpinBox->setValue(tune2walk.fourth_neg_xl);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_SideL_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_side_l += 0.01;
    ui->Pos_SideLSpinBox->setValue(tune2walk.first_pos_side_l);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_side_l += 0.01;
    ui->Pos_SideLSpinBox->setValue(tune2walk.second_pos_side_l);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_side_l += 0.01;
    ui->Pos_SideLSpinBox->setValue(tune2walk.third_pos_side_l);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_side_l += 0.01;
    ui->Pos_SideLSpinBox->setValue(tune2walk.fourth_pos_side_l);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_SideL_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_side_l += 0.01;
    ui->Neg_SideLSpinBox->setValue(tune2walk.first_neg_side_l);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_side_l += 0.01;
    ui->Neg_SideLSpinBox->setValue(tune2walk.second_neg_side_l);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_side_l += 0.01;
    ui->Neg_SideLSpinBox->setValue(tune2walk.third_neg_side_l);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_side_l += 0.01;
    ui->Neg_SideLSpinBox->setValue(tune2walk.fourth_neg_side_l);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_YawL_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_yaw_l += 0.01;
    ui->Pos_YawLSpinBox->setValue(tune2walk.first_pos_yaw_l);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_yaw_l += 0.01;
    ui->Pos_YawLSpinBox->setValue(tune2walk.second_pos_yaw_l);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_yaw_l += 0.01;
    ui->Pos_YawLSpinBox->setValue(tune2walk.third_pos_yaw_l);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_yaw_l += 0.01;
    ui->Pos_YawLSpinBox->setValue(tune2walk.fourth_pos_yaw_l);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_YawL_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_yaw_l += 0.01;
    ui->Neg_YawLSpinBox->setValue(tune2walk.first_neg_yaw_l);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_yaw_l += 0.01;
    ui->Neg_YawLSpinBox->setValue(tune2walk.second_neg_yaw_l);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_yaw_l += 0.01;
    ui->Neg_YawLSpinBox->setValue(tune2walk.third_neg_yaw_l);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_yaw_l += 0.01;
    ui->Neg_YawLSpinBox->setValue(tune2walk.fourth_neg_yaw_l);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Pos_XR_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_xr -= 0.01;
    ui->Pos_XRSpinBox->setValue(tune2walk.first_pos_xr);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_xr -= 0.01;
    ui->Pos_XRSpinBox->setValue(tune2walk.second_pos_xr);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_xr -= 0.01;
    ui->Pos_XRSpinBox->setValue(tune2walk.third_pos_xr);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_xr -= 0.01;
    ui->Pos_XRSpinBox->setValue(tune2walk.fourth_pos_xr);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Neg_XR_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_xr -= 0.01;
    ui->Neg_XRSpinBox->setValue(tune2walk.first_neg_xr);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_xr -= 0.01;
    ui->Neg_XRSpinBox->setValue(tune2walk.second_neg_xr);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_xr -= 0.01;
    ui->Neg_XRSpinBox->setValue(tune2walk.third_neg_xr);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_xr -= 0.01;
    ui->Neg_XRSpinBox->setValue(tune2walk.fourth_neg_xr);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_SideR_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_side_r -= 0.01;
    ui->Pos_SideRSpinBox->setValue(tune2walk.first_pos_side_r);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_side_r -= 0.01;
    ui->Pos_SideRSpinBox->setValue(tune2walk.second_pos_side_r);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_side_r -= 0.01;
    ui->Pos_SideRSpinBox->setValue(tune2walk.third_pos_side_r);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_side_r -= 0.01;
    ui->Pos_SideRSpinBox->setValue(tune2walk.fourth_pos_side_r);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_SideR_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_side_r -= 0.01;
    ui->Neg_SideRSpinBox->setValue(tune2walk.first_neg_side_r);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_side_r -= 0.01;
    ui->Neg_SideRSpinBox->setValue(tune2walk.second_neg_side_r);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_side_r -= 0.01;
    ui->Neg_SideRSpinBox->setValue(tune2walk.third_neg_side_r);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_side_r -= 0.01;
    ui->Neg_SideRSpinBox->setValue(tune2walk.fourth_neg_side_r);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_YawR_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_yaw_r -= 0.01;
    ui->Pos_YawRSpinBox->setValue(tune2walk.first_pos_yaw_r);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_yaw_r -= 0.01;
    ui->Pos_YawRSpinBox->setValue(tune2walk.second_pos_yaw_r);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_yaw_r -= 0.01;
    ui->Pos_YawRSpinBox->setValue(tune2walk.third_pos_yaw_r);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_yaw_r -= 0.01;
    ui->Pos_YawRSpinBox->setValue(tune2walk.fourth_pos_yaw_r);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_YawR_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_yaw_r -= 0.01;
    ui->Neg_YawRSpinBox->setValue(tune2walk.first_neg_yaw_r);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_yaw_r -= 0.01;
    ui->Neg_YawRSpinBox->setValue(tune2walk.second_neg_yaw_r);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_yaw_r -= 0.01;
    ui->Neg_YawRSpinBox->setValue(tune2walk.third_neg_yaw_r);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_yaw_r -= 0.01;
    ui->Neg_YawRSpinBox->setValue(tune2walk.fourth_neg_yaw_r);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_XL_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_xl -= 0.01;
    ui->Pos_XLSpinBox->setValue(tune2walk.first_pos_xl);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_xl -= 0.01;
    ui->Pos_XLSpinBox->setValue(tune2walk.second_pos_xl);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_xl -= 0.01;
    ui->Pos_XLSpinBox->setValue(tune2walk.third_pos_xl);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_xl -= 0.01;
    ui->Pos_XLSpinBox->setValue(tune2walk.fourth_pos_xl);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Neg_XL_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_xl -= 0.01;
    ui->Neg_XLSpinBox->setValue(tune2walk.first_neg_xl);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_xl -= 0.01;
    ui->Neg_XLSpinBox->setValue(tune2walk.second_neg_xl);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_xl -= 0.01;
    ui->Neg_XLSpinBox->setValue(tune2walk.third_neg_xl);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_xl -= 0.01;
    ui->Neg_XLSpinBox->setValue(tune2walk.fourth_neg_xl);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_SideL_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_side_l -= 0.01;
    ui->Pos_SideLSpinBox->setValue(tune2walk.first_pos_side_l);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_side_l -= 0.01;
    ui->Pos_SideLSpinBox->setValue(tune2walk.second_pos_side_l);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_side_l -= 0.01;
    ui->Pos_SideLSpinBox->setValue(tune2walk.third_pos_side_l);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_side_l -= 0.01;
    ui->Pos_SideLSpinBox->setValue(tune2walk.fourth_pos_side_l);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_SideL_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_side_l -= 0.01;
    ui->Neg_SideLSpinBox->setValue(tune2walk.first_neg_side_l);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_side_l -= 0.01;
    ui->Neg_SideLSpinBox->setValue(tune2walk.second_neg_side_l);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_side_l -= 0.01;
    ui->Neg_SideLSpinBox->setValue(tune2walk.third_neg_side_l);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_side_l -= 0.01;
    ui->Neg_SideLSpinBox->setValue(tune2walk.fourth_neg_side_l);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_YawL_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_yaw_l -= 0.01;
    ui->Pos_YawLSpinBox->setValue(tune2walk.first_pos_yaw_l);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_yaw_l -= 0.01;
    ui->Pos_YawLSpinBox->setValue(tune2walk.second_pos_yaw_l);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_yaw_l -= 0.01;
    ui->Pos_YawLSpinBox->setValue(tune2walk.third_pos_yaw_l);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_yaw_l -= 0.01;
    ui->Pos_YawLSpinBox->setValue(tune2walk.fourth_pos_yaw_l);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_YawL_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_yaw_l -= 0.01;
    ui->Neg_YawLSpinBox->setValue(tune2walk.first_neg_yaw_l);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_yaw_l -= 0.01;
    ui->Neg_YawLSpinBox->setValue(tune2walk.second_neg_yaw_l);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_yaw_l -= 0.01;
    ui->Neg_YawLSpinBox->setValue(tune2walk.third_neg_yaw_l);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_yaw_l -= 0.01;
    ui->Neg_YawLSpinBox->setValue(tune2walk.fourth_neg_yaw_l);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_SideRSpinBox_SwingMinus_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_pos_side_r_swing_minus = arg1;
  else if (index == 1)
    tune2walk.second_pos_side_r_swing_minus = arg1;
  else if (index == 2)
    tune2walk.third_pos_side_r_swing_minus = arg1;
  else if (index == 3)
    tune2walk.fourth_pos_side_r_swing_minus = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_SideRSpinBox_SwingMinus_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_neg_side_r_swing_minus = arg1;
  else if (index == 1)
    tune2walk.second_neg_side_r_swing_minus = arg1;
  else if (index == 2)
    tune2walk.third_neg_side_r_swing_minus = arg1;
  else if (index == 3)
    tune2walk.fourth_neg_side_r_swing_minus = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_SideLSpinBox_SwingMinus_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_pos_side_l_swing_minus = arg1;
  else if (index == 1)
    tune2walk.second_pos_side_l_swing_minus = arg1;
  else if (index == 2)
    tune2walk.third_pos_side_l_swing_minus = arg1;
  else if (index == 3)
    tune2walk.fourth_pos_side_l_swing_minus = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_SideLSpinBox_SwingMinus_valueChanged(double arg1)
{
  if (index == 0)
    tune2walk.first_neg_side_l_swing_minus = arg1;
  else if (index == 1)
    tune2walk.second_neg_side_l_swing_minus = arg1;
  else if (index == 2)
    tune2walk.third_neg_side_l_swing_minus = arg1;
  else if (index == 3)
    tune2walk.fourth_neg_side_l_swing_minus = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_SideR_SwingMinus_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_side_r_swing_minus += 0.1;
    ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.first_pos_side_r_swing_minus);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_side_r_swing_minus += 0.1;
    ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.second_pos_side_r_swing_minus);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_side_r_swing_minus += 0.1;
    ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.third_pos_side_r_swing_minus);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_side_r_swing_minus += 0.1;
    ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.fourth_pos_side_r_swing_minus);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_SideR_SwingMinus_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_side_r_swing_minus += 0.1;
    ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.first_neg_side_r_swing_minus);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_side_r_swing_minus += 0.1;
    ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.second_neg_side_r_swing_minus);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_side_r_swing_minus += 0.1;
    ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.third_neg_side_r_swing_minus);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_side_r_swing_minus += 0.1;
    ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.fourth_neg_side_r_swing_minus);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_SideL_SwingMinus_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_side_l_swing_minus += 0.1;
    ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.first_pos_side_l_swing_minus);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_side_l_swing_minus += 0.1;
    ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.second_pos_side_l_swing_minus);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_side_l_swing_minus += 0.1;
    ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.third_pos_side_l_swing_minus);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_side_l_swing_minus += 0.1;
    ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.fourth_pos_side_l_swing_minus);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_SideL_SwingMinus_Plus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_side_l_swing_minus += 0.1;
    ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.first_neg_side_l_swing_minus);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_side_l_swing_minus += 0.1;
    ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.second_neg_side_l_swing_minus);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_side_l_swing_minus += 0.1;
    ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.third_neg_side_l_swing_minus);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_side_l_swing_minus += 0.1;
    ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.fourth_neg_side_l_swing_minus);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_SideR_SwingMinus_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_side_r_swing_minus -= 0.1;
    ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.first_pos_side_r_swing_minus);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_side_r_swing_minus -= 0.1;
    ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.second_pos_side_r_swing_minus);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_side_r_swing_minus -= 0.1;
    ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.third_pos_side_r_swing_minus);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_side_r_swing_minus -= 0.1;
    ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.fourth_pos_side_r_swing_minus);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_SideR_SwingMinus_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_side_r_swing_minus -= 0.1;
    ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.first_neg_side_r_swing_minus);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_side_r_swing_minus -= 0.1;
    ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.second_neg_side_r_swing_minus);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_side_r_swing_minus -= 0.1;
    ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.third_neg_side_r_swing_minus);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_side_r_swing_minus -= 0.1;
    ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.fourth_neg_side_r_swing_minus);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Pos_SideL_SwingMinus_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_pos_side_l_swing_minus -= 0.1;
    ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.first_pos_side_l_swing_minus);
  }
  else if (index == 1)
  {
    tune2walk.second_pos_side_l_swing_minus -= 0.1;
    ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.second_pos_side_l_swing_minus);
  }
  else if (index == 2)
  {
    tune2walk.third_pos_side_l_swing_minus -= 0.1;
    ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.third_pos_side_l_swing_minus);
  }
  else if (index == 3)
  {
    tune2walk.fourth_pos_side_l_swing_minus -= 0.1;
    ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.fourth_pos_side_l_swing_minus);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Neg_SideL_SwingMinus_Minus_clicked()
{
  if (index == 0)
  {
    tune2walk.first_neg_side_l_swing_minus -= 0.1;
    ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.first_neg_side_l_swing_minus);
  }
  else if (index == 1)
  {
    tune2walk.second_neg_side_l_swing_minus -= 0.1;
    ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.second_neg_side_l_swing_minus);
  }
  else if (index == 2)
  {
    tune2walk.third_neg_side_l_swing_minus -= 0.1;
    ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.third_neg_side_l_swing_minus);
  }
  else if (index == 3)
  {
    tune2walk.fourth_neg_side_l_swing_minus -= 0.1;
    ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.fourth_neg_side_l_swing_minus);
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_First_min_valueChanged(int arg1)
{
  tune2walk.first_min = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_First_max_valueChanged(int arg1)
{
  tune2walk.first_max = arg1;
  ui->Second_min->setValue(tune2walk.first_max);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Second_min_valueChanged(int arg1)
{
  tune2walk.second_min = arg1;
  ui->First_max->setValue(tune2walk.second_min);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Second_max_valueChanged(int arg1)
{
  tune2walk.second_max = arg1;
  ui->Third_min->setValue(tune2walk.second_max);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Third_min_valueChanged(int arg1)
{
  tune2walk.third_min = arg1;
  ui->Second_max->setValue(tune2walk.third_min);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Third_max_valueChanged(int arg1)
{
  tune2walk.third_max = arg1;
  ui->Fourth_min->setValue(tune2walk.third_max);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Fourth_min_valueChanged(int arg1)
{
  tune2walk.fourth_min = arg1;
  ui->Third_max->setValue(tune2walk.fourth_min);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Fourth_max_valueChanged(int arg1)
{
  tune2walk.fourth_max = arg1;
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_First_load_clicked()
{
  index = 0;
  ui->Pos_XRSpinBox->setValue(tune2walk.first_pos_xr);
  ui->Neg_XRSpinBox->setValue(tune2walk.first_neg_xr);
  ui->Pos_SideRSpinBox->setValue(tune2walk.first_pos_side_r);
  ui->Neg_SideRSpinBox->setValue(tune2walk.first_neg_side_r);
  ui->Pos_YawRSpinBox->setValue(tune2walk.first_pos_yaw_r);
  ui->Neg_YawRSpinBox->setValue(tune2walk.first_neg_yaw_r);
  ui->Pos_XLSpinBox->setValue(tune2walk.first_pos_xl);
  ui->Neg_XLSpinBox->setValue(tune2walk.first_neg_xl);
  ui->Pos_SideLSpinBox->setValue(tune2walk.first_pos_side_l);
  ui->Neg_SideLSpinBox->setValue(tune2walk.first_neg_side_l);
  ui->Pos_YawLSpinBox->setValue(tune2walk.first_pos_yaw_l);
  ui->Neg_YawLSpinBox->setValue(tune2walk.first_neg_yaw_l);
  ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.first_pos_side_r_swing_minus);
  ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.first_neg_side_r_swing_minus);
  ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.first_pos_side_l_swing_minus);
  ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.first_neg_side_l_swing_minus);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Second_load_clicked()
{
  index = 1;
  ui->Pos_XRSpinBox->setValue(tune2walk.second_pos_xr);
  ui->Neg_XRSpinBox->setValue(tune2walk.second_neg_xr);
  ui->Pos_SideRSpinBox->setValue(tune2walk.second_pos_side_r);
  ui->Neg_SideRSpinBox->setValue(tune2walk.second_neg_side_r);
  ui->Pos_YawRSpinBox->setValue(tune2walk.second_pos_yaw_r);
  ui->Neg_YawRSpinBox->setValue(tune2walk.second_neg_yaw_r);
  ui->Pos_XLSpinBox->setValue(tune2walk.second_pos_xl);
  ui->Neg_XLSpinBox->setValue(tune2walk.second_neg_xl);
  ui->Pos_SideLSpinBox->setValue(tune2walk.second_pos_side_l);
  ui->Neg_SideLSpinBox->setValue(tune2walk.second_neg_side_l);
  ui->Pos_YawLSpinBox->setValue(tune2walk.second_pos_yaw_l);
  ui->Neg_YawLSpinBox->setValue(tune2walk.second_neg_yaw_l);
  ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.second_pos_side_r_swing_minus);
  ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.second_neg_side_r_swing_minus);
  ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.second_pos_side_l_swing_minus);
  ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.second_neg_side_l_swing_minus);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Third_load_clicked()
{
  index = 2;
  ui->Pos_XRSpinBox->setValue(tune2walk.third_pos_xr);
  ui->Neg_XRSpinBox->setValue(tune2walk.third_neg_xr);
  ui->Pos_SideRSpinBox->setValue(tune2walk.third_pos_side_r);
  ui->Neg_SideRSpinBox->setValue(tune2walk.third_neg_side_r);
  ui->Pos_YawRSpinBox->setValue(tune2walk.third_pos_yaw_r);
  ui->Neg_YawRSpinBox->setValue(tune2walk.third_neg_yaw_r);
  ui->Pos_XLSpinBox->setValue(tune2walk.third_pos_xl);
  ui->Neg_XLSpinBox->setValue(tune2walk.third_neg_xl);
  ui->Pos_SideLSpinBox->setValue(tune2walk.third_pos_side_l);
  ui->Neg_SideLSpinBox->setValue(tune2walk.third_neg_side_l);
  ui->Pos_YawLSpinBox->setValue(tune2walk.third_pos_yaw_l);
  ui->Neg_YawLSpinBox->setValue(tune2walk.third_neg_yaw_l);
  ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.third_pos_side_r_swing_minus);
  ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.third_neg_side_r_swing_minus);
  ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.third_pos_side_l_swing_minus);
  ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.third_neg_side_l_swing_minus);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Fourth_load_clicked()
{
  index = 3;
  ui->Pos_XRSpinBox->setValue(tune2walk.fourth_pos_xr);
  ui->Neg_XRSpinBox->setValue(tune2walk.fourth_neg_xr);
  ui->Pos_SideRSpinBox->setValue(tune2walk.fourth_pos_side_r);
  ui->Neg_SideRSpinBox->setValue(tune2walk.fourth_neg_side_r);
  ui->Pos_YawRSpinBox->setValue(tune2walk.fourth_pos_yaw_r);
  ui->Neg_YawRSpinBox->setValue(tune2walk.fourth_neg_yaw_r);
  ui->Pos_XLSpinBox->setValue(tune2walk.fourth_pos_xl);
  ui->Neg_XLSpinBox->setValue(tune2walk.fourth_neg_xl);
  ui->Pos_SideLSpinBox->setValue(tune2walk.fourth_pos_side_l);
  ui->Neg_SideLSpinBox->setValue(tune2walk.fourth_neg_side_l);
  ui->Pos_YawLSpinBox->setValue(tune2walk.fourth_pos_yaw_l);
  ui->Neg_YawLSpinBox->setValue(tune2walk.fourth_neg_yaw_l);
  ui->Pos_SideRSpinBox_SwingMinus->setValue(tune2walk.fourth_pos_side_r_swing_minus);
  ui->Neg_SideRSpinBox_SwingMinus->setValue(tune2walk.fourth_neg_side_r_swing_minus);
  ui->Pos_SideLSpinBox_SwingMinus->setValue(tune2walk.fourth_pos_side_l_swing_minus);
  ui->Neg_SideLSpinBox_SwingMinus->setValue(tune2walk.fourth_neg_side_l_swing_minus);
  qnode->tune2walk_Pub->publish(tune2walk);
}

//////////////////////Set (0 or Data)///////////////////////////
void MainWindow::on_Set_Zero_Button_clicked()
{
  tune2walk.swing_leg_right = 0;
  tune2walk.swing_leg_left = 0;
  tune2walk.swing_shoulder_right = 0;
  tune2walk.swing_shoulder_left = 0;
  tune2walk.rise_leg_right = 0;
  tune2walk.rise_leg_left = 0;
  tune2walk.start_rise = 0;
  tune2walk.start_swing = 0;
  tune2walk.end_rise = 0;
  tune2walk.end_swing = 0;
  tune2walk.test_x = 0;
  tune2walk.test_side = 0;
  tune2walk.test_yaw = 0;
  tune2walk.tuning_x = 0;
  tune2walk.tuning_side = 0;
  tune2walk.tuning_yaw = 0;

  ui->Swing_Right_Leg_Box->setValue(0);
  ui->Swing_Left_Leg_Box->setValue(0);
  ui->Swing_Right_Shoulder_Box->setValue(0);
  ui->Swing_Left_Shoulder_Box->setValue(0);
  ui->Rise_Right_Leg_Box->setValue(0);
  ui->Rise_Left_Leg_Box->setValue(0);
  ui->Start_Swing_Box->setValue(0);
  ui->Start_Rise_Box->setValue(0);
  ui->End_Swing_Box->setValue(0);
  ui->End_Rise_Box->setValue(0);
  ui->Test_X_Box->setValue(0);
  ui->Test_Side_Box->setValue(0);
  ui->Test_Yaw_Box->setValue(0);
  ui->Tuning_X_Box->setValue(0);
  ui->Tuning_Side_Box->setValue(0);
  ui->Tuning_Yaw_Box->setValue(0);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Set_Data1_Button_clicked()
{
  tune2walk.swing_leg_right = Tuning_Data[0].Swing_Right_Leg_Box;
  tune2walk.swing_leg_left = Tuning_Data[0].Swing_Left_Leg_Box;
  tune2walk.swing_shoulder_right = Tuning_Data[0].Swing_Right_Shoulder_Box;
  tune2walk.swing_shoulder_left = Tuning_Data[0].Swing_Left_Shoulder_Box;
  tune2walk.rise_leg_right = Tuning_Data[0].Rise_Right_Leg_Box;
  tune2walk.rise_leg_left = Tuning_Data[0].Rise_Left_Leg_Box;
  tune2walk.start_rise = Tuning_Data[0].Start_Rise_Box;
  tune2walk.start_swing = Tuning_Data[0].Start_Swing_Box;
  tune2walk.end_rise = Tuning_Data[0].End_Rise_Box;
  tune2walk.end_swing = Tuning_Data[0].End_Swing_Box;
  tune2walk.test_x = Tuning_Data[0].Test_X_Box;
  tune2walk.test_side = Tuning_Data[0].Test_Side_Box;
  tune2walk.test_yaw = Tuning_Data[0].Test_Yaw_Box;
  tune2walk.tuning_x = Tuning_Data[0].Tuning_X_Box;
  tune2walk.tuning_side = Tuning_Data[0].Tuning_Side_Box;
  tune2walk.tuning_yaw = Tuning_Data[0].Tuning_Yaw_Box;

  ui->Swing_Right_Leg_Box->setValue(Tuning_Data[0].Swing_Right_Leg_Box);
  ui->Swing_Left_Leg_Box->setValue(Tuning_Data[0].Swing_Left_Leg_Box);
  ui->Swing_Right_Shoulder_Box->setValue(Tuning_Data[0].Swing_Right_Shoulder_Box);
  ui->Swing_Left_Shoulder_Box->setValue(Tuning_Data[0].Swing_Left_Shoulder_Box);
  ui->Rise_Right_Leg_Box->setValue(Tuning_Data[0].Rise_Right_Leg_Box);
  ui->Rise_Left_Leg_Box->setValue(Tuning_Data[0].Rise_Left_Leg_Box);
  ui->Start_Swing_Box->setValue(Tuning_Data[0].Start_Swing_Box);
  ui->Start_Rise_Box->setValue(Tuning_Data[0].Start_Rise_Box);
  ui->End_Swing_Box->setValue(Tuning_Data[0].End_Swing_Box);
  ui->End_Rise_Box->setValue(Tuning_Data[0].End_Rise_Box);
  ui->Test_X_Box->setValue(Tuning_Data[0].Test_X_Box);
  ui->Test_Side_Box->setValue(Tuning_Data[0].Test_Side_Box);
  ui->Test_Yaw_Box->setValue(Tuning_Data[0].Test_Yaw_Box);
  ui->Tuning_X_Box->setValue(Tuning_Data[0].Tuning_X_Box);
  ui->Tuning_Side_Box->setValue(Tuning_Data[0].Tuning_Side_Box);
  ui->Tuning_Yaw_Box->setValue(Tuning_Data[0].Tuning_Yaw_Box);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Set_Data2_Button_clicked()
{
  tune2walk.swing_leg_right = Tuning_Data[1].Swing_Right_Leg_Box;
  tune2walk.swing_leg_left = Tuning_Data[1].Swing_Left_Leg_Box;
  tune2walk.swing_shoulder_right = Tuning_Data[1].Swing_Right_Shoulder_Box;
  tune2walk.swing_shoulder_left = Tuning_Data[1].Swing_Left_Shoulder_Box;
  tune2walk.rise_leg_right = Tuning_Data[1].Rise_Right_Leg_Box;
  tune2walk.rise_leg_left = Tuning_Data[1].Rise_Left_Leg_Box;
  tune2walk.start_rise = Tuning_Data[1].Start_Rise_Box;
  tune2walk.start_swing = Tuning_Data[1].Start_Swing_Box;
  tune2walk.end_rise = Tuning_Data[1].End_Rise_Box;
  tune2walk.end_swing = Tuning_Data[1].End_Swing_Box;
  tune2walk.test_x = Tuning_Data[1].Test_X_Box;
  tune2walk.test_side = Tuning_Data[1].Test_Side_Box;
  tune2walk.test_yaw = Tuning_Data[1].Test_Yaw_Box;
  tune2walk.tuning_x = Tuning_Data[1].Tuning_X_Box;
  tune2walk.tuning_side = Tuning_Data[1].Tuning_Side_Box;
  tune2walk.tuning_yaw = Tuning_Data[1].Tuning_Yaw_Box;

  ui->Swing_Right_Leg_Box->setValue(Tuning_Data[1].Swing_Right_Leg_Box);
  ui->Swing_Left_Leg_Box->setValue(Tuning_Data[1].Swing_Left_Leg_Box);
  ui->Swing_Right_Shoulder_Box->setValue(Tuning_Data[1].Swing_Right_Shoulder_Box);
  ui->Swing_Left_Shoulder_Box->setValue(Tuning_Data[1].Swing_Left_Shoulder_Box);
  ui->Rise_Right_Leg_Box->setValue(Tuning_Data[1].Rise_Right_Leg_Box);
  ui->Rise_Left_Leg_Box->setValue(Tuning_Data[1].Rise_Left_Leg_Box);
  ui->Start_Swing_Box->setValue(Tuning_Data[1].Start_Swing_Box);
  ui->Start_Rise_Box->setValue(Tuning_Data[1].Start_Rise_Box);
  ui->End_Swing_Box->setValue(Tuning_Data[1].End_Swing_Box);
  ui->End_Rise_Box->setValue(Tuning_Data[1].End_Rise_Box);
  ui->Test_X_Box->setValue(Tuning_Data[1].Test_X_Box);
  ui->Test_Side_Box->setValue(Tuning_Data[1].Test_Side_Box);
  ui->Test_Yaw_Box->setValue(Tuning_Data[1].Test_Yaw_Box);
  ui->Tuning_X_Box->setValue(Tuning_Data[1].Tuning_X_Box);
  ui->Tuning_Side_Box->setValue(Tuning_Data[1].Tuning_Side_Box);
  ui->Tuning_Yaw_Box->setValue(Tuning_Data[1].Tuning_Yaw_Box);
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_Set_Data3_Button_clicked()
{
  tune2walk.swing_leg_right = Tuning_Data[2].Swing_Right_Leg_Box;
  tune2walk.swing_leg_left = Tuning_Data[2].Swing_Left_Leg_Box;
  tune2walk.swing_shoulder_right = Tuning_Data[2].Swing_Right_Shoulder_Box;
  tune2walk.swing_shoulder_left = Tuning_Data[2].Swing_Left_Shoulder_Box;
  tune2walk.rise_leg_right = Tuning_Data[2].Rise_Right_Leg_Box;
  tune2walk.rise_leg_left = Tuning_Data[2].Rise_Left_Leg_Box;
  tune2walk.start_rise = Tuning_Data[2].Start_Rise_Box;
  tune2walk.start_swing = Tuning_Data[2].Start_Swing_Box;
  tune2walk.end_rise = Tuning_Data[2].End_Rise_Box;
  tune2walk.end_swing = Tuning_Data[2].End_Swing_Box;
  tune2walk.test_x = Tuning_Data[2].Test_X_Box;
  tune2walk.test_side = Tuning_Data[2].Test_Side_Box;
  tune2walk.test_yaw = Tuning_Data[2].Test_Yaw_Box;
  tune2walk.tuning_x = Tuning_Data[2].Tuning_X_Box;
  tune2walk.tuning_side = Tuning_Data[2].Tuning_Side_Box;
  tune2walk.tuning_yaw = Tuning_Data[2].Tuning_Yaw_Box;

  ui->Swing_Right_Leg_Box->setValue(Tuning_Data[2].Swing_Right_Leg_Box);
  ui->Swing_Left_Leg_Box->setValue(Tuning_Data[2].Swing_Left_Leg_Box);
  ui->Swing_Right_Shoulder_Box->setValue(Tuning_Data[2].Swing_Right_Shoulder_Box);
  ui->Swing_Left_Shoulder_Box->setValue(Tuning_Data[2].Swing_Left_Shoulder_Box);
  ui->Rise_Right_Leg_Box->setValue(Tuning_Data[2].Rise_Right_Leg_Box);
  ui->Rise_Left_Leg_Box->setValue(Tuning_Data[2].Rise_Left_Leg_Box);
  ui->Start_Swing_Box->setValue(Tuning_Data[2].Start_Swing_Box);
  ui->Start_Rise_Box->setValue(Tuning_Data[2].Start_Rise_Box);
  ui->End_Swing_Box->setValue(Tuning_Data[2].End_Swing_Box);
  ui->End_Rise_Box->setValue(Tuning_Data[2].End_Rise_Box);
  ui->Test_X_Box->setValue(Tuning_Data[2].Test_X_Box);
  ui->Test_Side_Box->setValue(Tuning_Data[2].Test_Side_Box);
  ui->Test_Yaw_Box->setValue(Tuning_Data[2].Test_Yaw_Box);
  ui->Tuning_X_Box->setValue(Tuning_Data[2].Tuning_X_Box);
  ui->Tuning_Side_Box->setValue(Tuning_Data[2].Tuning_Side_Box);
  ui->Tuning_Yaw_Box->setValue(Tuning_Data[2].Tuning_Yaw_Box);
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::on_Save_Data1_Button_clicked()
{
  Tuning_Data[0].Swing_Right_Leg_Box = tune2walk.swing_leg_right;
  Tuning_Data[0].Swing_Left_Leg_Box = tune2walk.swing_leg_left;
  Tuning_Data[0].Swing_Right_Shoulder_Box = tune2walk.swing_shoulder_right;
  Tuning_Data[0].Swing_Left_Shoulder_Box = tune2walk.swing_shoulder_left;
  Tuning_Data[0].Rise_Right_Leg_Box = tune2walk.rise_leg_right;
  Tuning_Data[0].Rise_Left_Leg_Box = tune2walk.rise_leg_left;
  Tuning_Data[0].Start_Rise_Box = tune2walk.start_rise;
  Tuning_Data[0].Start_Swing_Box = tune2walk.start_swing;
  Tuning_Data[0].End_Rise_Box = tune2walk.end_rise;
  Tuning_Data[0].End_Swing_Box = tune2walk.end_swing;
  Tuning_Data[0].Test_X_Box = tune2walk.test_x;
  Tuning_Data[0].Test_Side_Box = tune2walk.test_side;
  Tuning_Data[0].Test_Yaw_Box = tune2walk.test_yaw;
  Tuning_Data[0].Tuning_X_Box = tune2walk.tuning_x;
  Tuning_Data[0].Tuning_Side_Box = tune2walk.tuning_side;
  Tuning_Data[0].Tuning_Yaw_Box = tune2walk.tuning_yaw;
}

void MainWindow::on_Save_Data2_Button_clicked()
{
  Tuning_Data[1].Swing_Right_Leg_Box = tune2walk.swing_leg_right;
  Tuning_Data[1].Swing_Left_Leg_Box = tune2walk.swing_leg_left;
  Tuning_Data[1].Swing_Right_Shoulder_Box = tune2walk.swing_shoulder_right;
  Tuning_Data[1].Swing_Left_Shoulder_Box = tune2walk.swing_shoulder_left;
  Tuning_Data[1].Rise_Right_Leg_Box = tune2walk.rise_leg_right;
  Tuning_Data[1].Rise_Left_Leg_Box = tune2walk.rise_leg_left;
  Tuning_Data[1].Start_Rise_Box = tune2walk.start_rise;
  Tuning_Data[1].Start_Swing_Box = tune2walk.start_swing;
  Tuning_Data[1].End_Rise_Box = tune2walk.end_rise;
  Tuning_Data[1].End_Swing_Box = tune2walk.end_swing;
  Tuning_Data[1].Test_X_Box = tune2walk.test_x;
  Tuning_Data[1].Test_Side_Box = tune2walk.test_side;
  Tuning_Data[1].Test_Yaw_Box = tune2walk.test_yaw;
  Tuning_Data[1].Tuning_X_Box = tune2walk.tuning_x;
  Tuning_Data[1].Tuning_Side_Box = tune2walk.tuning_side;
  Tuning_Data[1].Tuning_Yaw_Box = tune2walk.tuning_yaw;
}

void MainWindow::on_Save_Data3_Button_clicked()
{
  Tuning_Data[2].Swing_Right_Leg_Box = tune2walk.swing_leg_right;
  Tuning_Data[2].Swing_Left_Leg_Box = tune2walk.swing_leg_left;
  Tuning_Data[2].Swing_Right_Shoulder_Box = tune2walk.swing_shoulder_right;
  Tuning_Data[2].Swing_Left_Shoulder_Box = tune2walk.swing_shoulder_left;
  Tuning_Data[2].Rise_Right_Leg_Box = tune2walk.rise_leg_right;
  Tuning_Data[2].Rise_Left_Leg_Box = tune2walk.rise_leg_left;
  Tuning_Data[2].Start_Rise_Box = tune2walk.start_rise;
  Tuning_Data[2].Start_Swing_Box = tune2walk.start_swing;
  Tuning_Data[2].End_Rise_Box = tune2walk.end_rise;
  Tuning_Data[2].End_Swing_Box = tune2walk.end_swing;
  Tuning_Data[2].Test_X_Box = tune2walk.test_x;
  Tuning_Data[2].Test_Side_Box = tune2walk.test_side;
  Tuning_Data[2].Test_Yaw_Box = tune2walk.test_yaw;
  Tuning_Data[2].Tuning_X_Box = tune2walk.tuning_x;
  Tuning_Data[2].Tuning_Side_Box = tune2walk.tuning_side;
  Tuning_Data[2].Tuning_Yaw_Box = tune2walk.tuning_yaw;
}

void MainWindow::on_Landing_Time_Control_flag_clicked()
{
  if (tune2walk.landing_time_control_flag == 0)
  {
    tune2walk.landing_time_control_flag = 1;
    cout << "LTC ON" << endl;
  }
  else if (tune2walk.landing_time_control_flag == 1)
  {
    tune2walk.landing_time_control_flag = 0;
    cout << "LTC OFF" << endl;
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}

void MainWindow::on_checkBox_Control_System_clicked()
{
  static int Control_flag[6];
  if (tune2walk.control_system_flag == 1)
  {
    Control_flag[0] = tune2walk.balance_pitch_flag_imu;
    Control_flag[1] = tune2walk.balance_roll_flag_imu;
    Control_flag[2] = tune2walk.balance_pitch_flag;
    Control_flag[3] = tune2walk.balance_ankle_pitch_flag;
    Control_flag[4] = tune2walk.balance_roll_flag;
    Control_flag[5] = tune2walk.landing_time_control_flag;
    tune2walk.control_system_flag = 0;
    tune2walk.balance_pitch_flag_imu = 0;
    tune2walk.balance_roll_flag_imu = 0;
    tune2walk.balance_pitch_flag = 0;
    tune2walk.balance_ankle_pitch_flag = 0;
    tune2walk.balance_roll_flag = 0;
    tune2walk.landing_time_control_flag = 0;
    ui->checkBox_Pitch_IMU->setChecked(0);
    ui->checkBox_Roll_IMU->setChecked(0);
    ui->checkBox_Pitch_ZMP->setChecked(0);
    ui->checkBox_Ankle_Pitch_ZMP->setChecked(0);
    ui->checkBox_Ankle_Roll_ZMP->setChecked(0);
    ui->Landing_Time_Control_flag->setChecked(0);
  }
  else
  {

    tune2walk.control_system_flag = 1;
    if (Control_flag[0])
    {
      tune2walk.balance_pitch_flag_imu = 1;
      ui->checkBox_Pitch_IMU->setChecked(1);
    }
    if (Control_flag[1])
    {
      tune2walk.balance_roll_flag_imu = 1;
      ui->checkBox_Roll_IMU->setChecked(1);
    }
    if (Control_flag[2])
    {
      tune2walk.balance_pitch_flag = 1;
      ui->checkBox_Pitch_ZMP->setChecked(1);
    }
    if (Control_flag[3])
    {
      tune2walk.balance_ankle_pitch_flag = 1;
      ui->checkBox_Ankle_Pitch_ZMP->setChecked(1);
    }
    if (Control_flag[4])
    {
      tune2walk.balance_roll_flag = 1;
      ui->checkBox_Ankle_Roll_ZMP->setChecked(1);
    }
    if (Control_flag[5])
    {
      tune2walk.landing_time_control_flag = 1;
      ui->Landing_Time_Control_flag->setChecked(1);
    }
  }
  qnode->tune2walk_Pub->publish(tune2walk);
}
void MainWindow::LandingCallback()
{
  ui->Entire_Time->setText(QString::number(qnode->Landing_info.entire_time));
  ui->Swing_Gain_L->setText(QString::number(qnode->Landing_info.swing_gain_l));
  ui->Swing_Gain_R->setText(QString::number(qnode->Landing_info.swing_gain_r));
  ui->Slider_Warning->setValue(qnode->Landing_info.warning);
  ui->Slider_Safe->setValue(qnode->Landing_info.safe);

  ui->Landing_Time_L->setText(QString::number(qnode->Landing_info.landing_time_l));
  ui->Landing_Time_R->setText(QString::number(qnode->Landing_info.landing_time_r));

  ui->Landing_Error_L->setText(QString::number(qnode->Landing_info.landing_error_l));
  ui->Landing_Error_R->setText(QString::number(qnode->Landing_info.landing_error_r));
}
void MainWindow::on_IMU_graph_clicked(){
    if(IMU_graph_flag == false)
    {
        IMU_graph_flag = true;
    }
    else
    {
        IMU_graph_flag = false;
    }
}


}

