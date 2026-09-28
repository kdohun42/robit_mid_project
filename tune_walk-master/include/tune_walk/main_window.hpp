/**
 * @file /include/tune_walk/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2024
 **/

#ifndef tune_walk_MAIN_WINDOW_H
#define tune_walk_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <QKeyEvent>
#include <QDebug>
#include <QTimer>
#include <QFileDialog>

  using namespace std;

namespace tune_walk
{
  extern rclcpp::Publisher<humanoid_interfaces::msg::Tune2IkMsg>::SharedPtr tune2walk_Pub;

  /*****************************************************************************
  ** Interface [MainWindow]
  *****************************************************************************/
  /**
   * @brief Qt central, all operations relating to the view part here.
   */

  class MainWindow : public QMainWindow
  {
    Q_OBJECT

  public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    QNode *qnode;

    humanoid_interfaces::msg::Tune2IkMsg tune2walk;

    double Click_Value;
    double Offset_Scroll_Value;
    double Offset_Angle_Limit;
    int index = 0;
    void past_motor_update();
    void angle_update();
    void arr_update();
    void arr_reset();
    int arr_check(int arr[], int motor_num);
    int arr_check_num = 0;
    int arr_num_cnt(int num, int arr[], int size);

    int motor_plus_cnt = 0;
    int motor_minus_cnt = 0;
    int motor_plus_arr[8] = {
        0,
    };
    int motor_minus_arr[8] = {
        0,
    };

    void Motor_Num_Toss(int Pos_or_Neg, int motor_num);
    void Offset_scroll_reset(int button_flag);

    double offset_past_motor[22] = {
        0,
    };

    void motor_limit_control();

    void MakePlot();

    int L_Step_Pattern_Check_Flag = 0;
    int L_COM_Pattern_Check_Flag = 0;
    int L_Rise_Pattern_Check_Flag = 0;
    int L_Side_Pattern_Check_Flag = 0;
    int L_Turn_Pattern_Check_Flag = 0;
    int R_Step_Pattern_Check_Flag = 0;
    int R_COM_Pattern_Check_Flag = 0;
    int R_Rise_Pattern_Check_Flag = 0;
    int R_Side_Pattern_Check_Flag = 0;
    int R_Turn_Pattern_Check_Flag = 0;

  public slots:
    void LandingCallback();

  private slots:
    void keyPressEvent(QKeyEvent *e);
    void on_Entire_Time_Box_valueChanged(double arg1);
    void on_Entire_Time_Plus_Button_clicked();
    void on_Entire_Time_Minus_Button_clicked();

    void on_Frequency_Box_valueChanged(double arg1);
    void on_Frequency_Plus_Button_clicked();
    void on_Frequency_Minus_Button_clicked();

    void on_Default_X_Right_Box_valueChanged(double arg1);
    void on_Default_X_Right_Plus_Button_clicked();
    void on_Default_X_Right_Minus_Button_clicked();

    void on_Default_X_Left_Box_valueChanged(double arg1);
    void on_Default_X_Left_Plus_Button_clicked();
    void on_Default_X_Left_Minus_Button_clicked();

    void on_Default_Y_Right_Box_valueChanged(double arg1);
    void on_Default_Y_Right_Plus_Button_clicked();
    void on_Default_Y_Right_Minus_Button_clicked();

    void on_Default_Y_Left_Box_valueChanged(double arg1);
    void on_Default_Y_Left_Plus_Button_clicked();
    void on_Default_Y_Left_Minus_Button_clicked();

    void on_Default_Z_Right_Box_valueChanged(double arg1);
    void on_Default_Z_Right_Plus_Button_clicked();
    void on_Default_Z_Right_Minus_Button_clicked();

    void on_Default_Z_Left_Box_valueChanged(double arg1);
    void on_Default_Z_Left_Plus_Button_clicked();
    void on_Default_Z_Left_Minus_Button_clicked();

    void on_All_Offset_Zero_Button_clicked();
    void on_Motor_10_Plus_clicked();
    void on_Motor_11_Plus_clicked();
    void on_Motor_12_Plus_clicked();
    void on_Motor_13_Plus_clicked();
    void on_Motor_14_Plus_clicked();
    void on_Motor_15_Plus_clicked();
    void on_Motor_16_Plus_clicked();
    void on_Motor_17_Plus_clicked();
    void on_Motor_18_Plus_clicked();
    void on_Motor_19_Plus_clicked();
    void on_Motor_20_Plus_clicked();
    void on_Motor_21_Plus_clicked();
    void on_Motor_10_Minus_clicked();
    void on_Motor_11_Minus_clicked();
    void on_Motor_12_Minus_clicked();
    void on_Motor_13_Minus_clicked();
    void on_Motor_14_Minus_clicked();
    void on_Motor_15_Minus_clicked();
    void on_Motor_16_Minus_clicked();
    void on_Motor_17_Minus_clicked();
    void on_Motor_18_Minus_clicked();
    void on_Motor_19_Minus_clicked();
    void on_Motor_20_Minus_clicked();
    void on_Motor_21_Minus_clicked();

    void on_plus_Button_0_clicked();
    void on_plus_Button_1_clicked();
    void on_plus_Button_2_clicked();
    void on_plus_Button_3_clicked();
    void on_minus_Button_0_clicked();
    void on_minus_Button_1_clicked();
    void on_minus_Button_2_clicked();
    void on_minus_Button_3_clicked();

    void on_Angle_horizontalScrollBar_valueChanged(int value);
    void on_Angle_Control_Box_valueChanged(double arg1);
    void on_offset_O_Button_clicked();
    void on_offset_X_Button_clicked();

    void on_Stand_Button_clicked();
    void on_Stand_Left_Button_clicked();
    void on_Stand_Right_Button_clicked();
    void on_Stretch_Button_clicked();

    void on_Swing_Right_Leg_Box_valueChanged(double arg1);
    void on_Swing_Right_Leg_Plus_Button_clicked();
    void on_Swing_Right_Leg_Minus_Button_clicked();

    void on_Swing_Left_Leg_Box_valueChanged(double arg1);
    void on_Swing_Left_Leg_Plus_Button_clicked();
    void on_Swing_Left_Leg_Minus_Button_clicked();

    void on_Swing_Right_Shoulder_Box_valueChanged(double arg1);
    void on_Swing_Right_Shoulder_Plus_Button_clicked();
    void on_Swing_Right_Shoulder_Minus_Button_clicked();

    void on_Swing_Left_Shoulder_Box_valueChanged(double arg1);
    void on_Swing_Left_Shoulder_Plus_Button_clicked();
    void on_Swing_Left_Shoulder_Minus_Button_clicked();

    void on_Rise_Right_Leg_Box_valueChanged(double arg1);
    void on_Rise_Right_Leg_Plus_Button_clicked();
    void on_Rise_Right_Leg_Minus_Button_clicked();

    void on_Rise_Left_Leg_Box_valueChanged(double arg1);
    void on_Rise_Left_Leg_Plus_Button_clicked();
    void on_Rise_Left_Leg_Minus_Button_clicked();

    void on_Rise_All_Leg_Plus_Button_clicked();
    void on_Rise_All_Leg_Minus_Button_clicked();
    void on_Rise_All_Leg_Plus5_Button_clicked();
    void on_Rise_All_Leg_Minus5_Button_clicked();

    void on_Start_Entire_Time_Box_valueChanged(double arg1);
    void on_Start_Entire_Time_Plus_Button_clicked();
    void on_Start_Entire_Time_Minus_Button_clicked();

    void on_Start_Swing_Box_valueChanged(double arg1);
    void on_Start_Swing_Plus_Button_clicked();
    void on_Start_Swing_Minus_Button_clicked();

    void on_Start_Rise_Box_valueChanged(double arg1);
    void on_Start_Rise_Plus_Button_clicked();
    void on_Start_Rise_Minus_Button_clicked();

    void on_End_Entire_Time_Box_valueChanged(double arg1);
    void on_End_Entire_Time_Plus_Button_clicked();
    void on_End_Entire_Time_Minus_Button_clicked();

    void on_End_Swing_Box_valueChanged(double arg1);
    void on_End_Swing_Plus_Button_clicked();
    void on_End_Swing_Minus_Button_clicked();

    void on_End_Rise_Box_valueChanged(double arg1);
    void on_End_Rise_Plus_Button_clicked();
    void on_End_Rise_Minus_Button_clicked();

    void on_IK_Flag_Button_clicked();
    void on_CP_Flag_Button_clicked();

    void on_Reset_Button_clicked();
    void on_Open_Button_clicked();
    void on_Save_Button_clicked();

    void on_Tuning_X_Box_valueChanged(double arg1);
    void on_Tuning_Side_Box_valueChanged(double arg1);
    void on_Tuning_Yaw_Box_valueChanged(double arg1);
    void on_Test_X_Box_valueChanged(double arg1);
    void on_Test_Side_Box_valueChanged(double arg1);
    void on_Test_Yaw_Box_valueChanged(double arg1);

    void on_Balance_Value_0_Box_valueChanged(double arg1);
    void on_Blance_Value_0_Plus_Button_clicked();
    void on_Blance_Value_0_Minus_Button_clicked();

    void on_Balance_Pitch_GP_Box_valueChanged(double arg1);
    void on_Balance_Pitch_GI_Box_valueChanged(double arg1);
    void on_Balance_Pitch_GD_Box_valueChanged(double arg1);
    void on_Balance_Pitch_ELIMIT_Box_valueChanged(double arg1);
    void on_Balance_Pitch_OLIMIT_Box_valueChanged(double arg1);
    void on_Balance_Pitch_Neg_Target_Box_valueChanged(double arg1);
    void on_Balance_Pitch_Pos_Target_Box_valueChanged(double arg1);

    void on_checkBox_Pitch_ZMP_clicked();

    void on_checkBox_Control_System_clicked();

    void on_IMU_graph_clicked();

    void on_Balance_Ankle_Pitch_GP_Box_valueChanged(double arg1);

    void on_Balance_Ankle_Pitch_GI_Box_valueChanged(double arg1);

    void on_Balance_Ankle_Pitch_GD_Box_valueChanged(double arg1);

    void on_Balance_Ankle_Pitch_ELIMIT_Box_valueChanged(double arg1);

    void on_Balance_Ankle_Pitch_OLIMIT_Box_valueChanged(double arg1);

    void on_Balance_Ankle_Pitch_Neg_Target_Box_valueChanged(double arg1);
    void on_Balance_Ankle_Pitch_Pos_Target_Box_valueChanged(double arg1);
    void on_checkBox_Ankle_Pitch_ZMP_clicked();

    void on_Balance_Value_1_Box_valueChanged(double arg1);

    void on_Blance_Value_1_Plus_Button_clicked();

    void on_Blance_Value_1_Minus_Button_clicked();

    void on_checkBox_Ankle_Roll_ZMP_clicked();

    void on_Balance_Roll_GP_Box_valueChanged(double arg1);

    void on_Balance_Roll_GI_Box_valueChanged(double arg1);

    void on_Balance_Roll_GD_Box_valueChanged(double arg1);

    void on_Balance_Roll_ELIMIT_Box_valueChanged(double arg1);

    void on_Balance_Roll_OLIMIT_Box_valueChanged(double arg1);

    void on_Balance_Roll_Neg_Target_Box_valueChanged(double arg1);

    void on_Balance_Roll_Pos_Target_Box_valueChanged(double arg1);

    void on_Balance_Value_2_Box_valueChanged(double arg1);

    void on_Blance_Value_2_Plus_Button_clicked();

    void on_Blance_Value_2_Minus_Button_clicked();

    void on_Swing_Both_Leg_Plus5_Button_clicked();

    void on_Swing_Both_Leg_Minus5_Button_clicked();

    void on_Swing_Both_Leg_Plus_Button_clicked();

    void on_Swing_Both_Leg_Minus_Button_clicked();

    void on_Swing_Both_Shoulder_Plus5_Button_clicked();

    void on_Swing_Both_Shoulder_Minus5_Button_clicked();

    void on_Ratio_Check_Box_clicked();

    void on_Balance_Value_3_Box_valueChanged(double arg1);

    void on_Blance_Value_3_Plus_Button_clicked();

    void on_Blance_Value_3_Minus_Button_clicked();

    void on_Balance_Value_0_Plus_Button_2_clicked();

    void on_Balance_Value_0_Minus_Button_2_clicked();

    void on_Balance_Value_1_Plus_Button_2_clicked();

    void on_Balance_Value_1_Minus_Button_2_clicked();

    void on_checkBox_Pitch_IMU_clicked();

    void on_Balance_Pitch_GP_IMU_Box_valueChanged(double arg1);

    void on_Balance_Pitch_GI_IMU_Box_valueChanged(double arg1);

    void on_Balance_Pitch_GD_IMU_Box_valueChanged(double arg1);

    void on_Balance_Pitch_ELIMIT_IMU_Box_valueChanged(double arg1);

    void on_Balance_Pitch_OLIMIT_IMU_Box_valueChanged(double arg1);

    void on_Balance_Pitch_Neg_Target_IMU_Box_valueChanged(double arg1);

    void on_Balance_Pitch_Pos_Target_IMU_Box_valueChanged(double arg1);

    void on_checkBox_Roll_IMU_clicked();

    void on_Balance_Roll_GP_IMU_Box_valueChanged(double arg1);

    void on_Balance_Roll_GI_IMU_Box_valueChanged(double arg1);

    void on_Balance_Roll_GD_IMU_Box_valueChanged(double arg1);

    void on_Balance_Roll_ELIMIT_IMU_Box_valueChanged(double arg1);

    void on_Balance_Roll_OLIMIT_IMU_Box_valueChanged(double arg1);

    void on_Balance_Roll_Neg_Target_IMU_Box_valueChanged(double arg1);

    void on_Balance_Roll_Pos_Target_IMU_Box_valueChanged(double arg1);

    void makeIMUPlot();
    void IMUrealtimeDataSlot();
    void on_Center2Leg_Box_valueChanged(double arg1);

    void on_Link2Link_Box_valueChanged(double arg1);

    void on_Init_Z_Up_Box_valueChanged(double arg1);

    void on_Init_Z_Up_Plus_Button_clicked();

    void on_Init_Z_Up_Minus_Button_clicked();

    void on_Percentage_of_10_Motor_Box_valueChanged(double arg1);

    void on_Percentage_of_11_Motor_Box_valueChanged(double arg1);

    void on_Percentage_of_12_Motor_Box_valueChanged(double arg1);

    void on_Percentage_of_13_Motor_Box_valueChanged(double arg1);

    void on_Percentage_of_14_Motor_Box_valueChanged(double arg1);

    void on_Percentage_of_15_Motor_Box_valueChanged(double arg1);

    void on_Percentage_of_16_Motor_Box_valueChanged(double arg1);

    void on_Percentage_of_17_Motor_Box_valueChanged(double arg1);

    void on_Percentage_of_18_Motor_Box_valueChanged(double arg1);

    void on_Percentage_of_19_Motor_Box_valueChanged(double arg1);

    void on_Percentage_of_20_Motor_Box_valueChanged(double arg1);

    void on_Percentage_of_21_Motor_Box_valueChanged(double arg1);

    void on_Pos_XRSpinBox_valueChanged(double arg1);

    //////K_Value//////

    void on_Neg_XRSpinBox_valueChanged(double arg1);
    void on_Pos_SideRSpinBox_valueChanged(double arg1);
    void on_Neg_SideRSpinBox_valueChanged(double arg1);
    void on_Pos_YawRSpinBox_valueChanged(double arg1);
    void on_Neg_YawRSpinBox_valueChanged(double arg1);
    void on_Pos_XLSpinBox_valueChanged(double arg1);
    void on_Neg_XLSpinBox_valueChanged(double arg1);
    void on_Pos_SideLSpinBox_valueChanged(double arg1);
    void on_Neg_SideLSpinBox_valueChanged(double arg1);
    void on_Pos_YawLSpinBox_valueChanged(double arg1);
    void on_Neg_YawLSpinBox_valueChanged(double arg1);

    void on_Pos_XR_Plus_clicked();
    void on_Neg_XR_Plus_clicked();
    void on_Pos_SideR_Plus_clicked();
    void on_Neg_SideR_Plus_clicked();
    void on_Pos_SideR_SwingMinus_Plus_clicked();
    void on_Neg_SideR_SwingMinus_Minus_clicked();
    void on_Pos_SideR_SwingMinus_Minus_clicked();
    void on_Neg_SideR_SwingMinus_Plus_clicked();
    void on_Pos_YawR_Plus_clicked();
    void on_Neg_YawR_Plus_clicked();
    void on_Pos_XL_Plus_clicked();
    void on_Neg_XL_Plus_clicked();
    void on_Pos_SideL_Plus_clicked();
    void on_Neg_SideL_Plus_clicked();

    void on_Pos_SideRSpinBox_SwingMinus_valueChanged(double arg1);
    void on_Neg_SideRSpinBox_SwingMinus_valueChanged(double arg1);
    void on_Pos_SideLSpinBox_SwingMinus_valueChanged(double arg1);
    void on_Neg_SideLSpinBox_SwingMinus_valueChanged(double arg1);

    void on_Pos_SideL_SwingMinus_Plus_clicked();
    void on_Neg_SideL_SwingMinus_Minus_clicked();
    void on_Pos_SideL_SwingMinus_Minus_clicked();
    void on_Neg_SideL_SwingMinus_Plus_clicked();
    void on_Pos_YawL_Plus_clicked();
    void on_Neg_YawL_Plus_clicked();
    void on_Pos_XR_Minus_clicked();
    void on_Neg_XR_Minus_clicked();
    void on_Pos_SideR_Minus_clicked();
    void on_Neg_SideR_Minus_clicked();
    void on_Pos_YawR_Minus_clicked();
    void on_Neg_YawR_Minus_clicked();
    void on_Pos_XL_Minus_clicked();
    void on_Neg_XL_Minus_clicked();
    void on_Pos_SideL_Minus_clicked();
    void on_Neg_SideL_Minus_clicked();
    void on_Pos_YawL_Minus_clicked();
    void on_Neg_YawL_Minus_clicked();

    void on_First_min_valueChanged(int arg1);
    void on_First_max_valueChanged(int arg1);
    void on_Second_min_valueChanged(int arg1);
    void on_Second_max_valueChanged(int arg1);
    void on_Third_min_valueChanged(int arg1);
    void on_Third_max_valueChanged(int arg1);
    void on_Fourth_min_valueChanged(int arg1);
    void on_Fourth_max_valueChanged(int arg1);

    void on_First_load_clicked();
    void on_Second_load_clicked();
    void on_Third_load_clicked();
    void on_Fourth_load_clicked();

    void on_Set_Zero_Button_clicked();
    void on_Set_Data1_Button_clicked();
    void on_Set_Data2_Button_clicked();
    void on_Set_Data3_Button_clicked();
    void on_Save_Data1_Button_clicked();
    void on_Save_Data2_Button_clicked();
    void on_Save_Data3_Button_clicked();

    /// Landing_Time_Control///
    void on_Landing_Time_Control_flag_clicked();

    void on_L_Step_Pattern_Check_Box_clicked();
    void on_L_COM_Pattern_Check_Box_clicked();
    void on_L_Rise_Pattern_Check_Box_clicked();
    void on_L_Side_Pattern_Check_Box_clicked();
    void on_L_Turn_Pattern_Check_Box_clicked();
    void on_R_Step_Pattern_Check_Box_clicked();
    void on_R_COM_Pattern_Check_Box_clicked();
    void on_R_Rise_Pattern_Check_Box_clicked();
    void on_R_Side_Pattern_Check_Box_clicked();
    void on_R_Turn_Pattern_Check_Box_clicked();

  private:
    Ui::MainWindowDesign *ui;
    void closeEvent(QCloseEvent *event);
    int value = 0;
    int Condition_Data[6];
  };

}
#endif // tune_walk_MAIN_WINDOW_H
