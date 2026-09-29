#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "ik_walk.hpp"

std::shared_ptr<IKwalk> node;

IKwalk::Robit_Humanoid Model_Data;
IKwalk::K_Value K_value[4];
IKwalk::Adjust_Yaw Adjust; // <-Compansate

IKwalk::Timer CNT;
IKwalk::Step_Acc Acc;
IKwalk::Calc Cal;

IKwalk::Walk_Param Now_Param;
IKwalk::Walk_Param Past_Param;

IKwalk::Balancing Balance;

/// For RUN_MODE ///
bool Run_Flag = false;
double Run_Add = 0.0;

/// For ANKLE_MODE ///
bool Ankle_Flag = true;

/// For WAIST_TURN (골반 회전: 스윙(나가는) 발 방향으로 허리(22번)를 돌려 보행 틀어짐 보완) ///
// 왼발/오른발의 전후 변위차에 비례해 허리를 회전. 보폭(X)이 클수록 진폭이 커지고,
// 제자리/정지 시 0으로 수렴. 부호로 회전 방향을 바꾸며, 0이면 기능 비활성.
double Waist_Swing_Gain = 0.2;   // (deg per mm) 허리 회전 세기/부호. 첫 테스트 시 부호 먼저 확인할 것
double Waist_Yaw_Limit  = 25.0;  // 허리 회전 각도 제한(deg) - 하드웨어 보호

/// for imu pitch > walk stop ///
int imu_safe_cnt = 0;

bool Start_Flag = false;
bool Normal_first_Flag = false;
bool End_Flag = false;

IK_zmp_pos_control Zmp_pos;
IK_imu_pos_control Imu_Balance;

double filter_data[FILTERDATA];
double temp;

void IKwalk::get_parameters()
{

  std::string addr;
  addr = "/home/kdh/Desktop/colcon_ws/src/robit_mid_project/tune_walk-master/work/RCKO"; // ******************************************

  std::ifstream is(addr.c_str());

  if (!(is.is_open())) // nuc 6
  {
    cout << "---------------Warning!---------------" << endl;
    cout << "------Not Defined TUNE_WALK File------" << endl;
    cout << "--------------------------------------" << endl;
    std::exit(0);
  }

  is >> Past_Param.Entire_Time;
  is >> Past_Param.Frequency;
  is >> Balance.Ratio_Check_Flag;

  is >> Past_Param.X.Default_X_Right;
  is >> Past_Param.X.Default_X_Left;
  is >> Past_Param.Y.Default_Y_Right;
  is >> Past_Param.Y.Default_Y_Left;
  is >> Past_Param.Z.Default_Z_Right;
  is >> Past_Param.Z.Default_Z_Left;
  is >> IK.Past_Motor_Angle.Motor_Angle_10;
  is >> IK.Past_Motor_Angle.Motor_Angle_11;
  is >> IK.Past_Motor_Angle.Motor_Angle_12;
  is >> IK.Past_Motor_Angle.Motor_Angle_13;
  is >> IK.Past_Motor_Angle.Motor_Angle_14;
  is >> IK.Past_Motor_Angle.Motor_Angle_15;
  is >> IK.Past_Motor_Angle.Motor_Angle_16;
  is >> IK.Past_Motor_Angle.Motor_Angle_17;
  is >> IK.Past_Motor_Angle.Motor_Angle_18;
  is >> IK.Past_Motor_Angle.Motor_Angle_19;
  is >> IK.Past_Motor_Angle.Motor_Angle_20;
  is >> IK.Past_Motor_Angle.Motor_Angle_21;
  is >> Past_Param.Y.Swing_Leg_Right;
  is >> Past_Param.Y.Swing_Leg_Left;
  is >> Past_Param.Shoulder.Swing_Shoulder_Right;
  is >> Past_Param.Shoulder.Swing_Shoulder_Left;
  is >> Past_Param.Z.Rise_Leg_Right;
  is >> Past_Param.Z.Rise_Leg_Left;
  is >> Past_Param.Start_Entire_Time;
  is >> Past_Param.Y.Start_Swing;
  is >> Past_Param.Z.Start_Rise;
  is >> Now_Param.End_Entire_Time;
  is >> Now_Param.Y.End_Swing;
  is >> Now_Param.Z.End_Rise;
  is >> Past_Param.X.Tuning_X;
  is >> Past_Param.Y.Tuning_Side;
  is >> Past_Param.Yaw_R.Tuning_Yaw;

  is >> Balance.Balance_Value_0;
  is >> Balance.Balance_Pitch_GP;
  is >> Balance.Balance_Pitch_GI;
  is >> Balance.Balance_Pitch_GD;
  is >> Balance.Balance_Pitch_ELIMIT;
  is >> Balance.Balance_Pitch_OLIMIT;
  is >> Balance.Balance_Pitch_Neg_Target;
  is >> Balance.Balance_Pitch_Pos_Target;

  is >> Balance.Balance_Value_1;
  is >> Balance.Balance_Angle_Pitch_GP;
  is >> Balance.Balance_Angle_Pitch_GI;
  is >> Balance.Balance_Angle_Pitch_GD;
  is >> Balance.Balance_Angle_Pitch_ELIMIT;
  is >> Balance.Balance_Angle_Pitch_OLIMIT;
  is >> Balance.Balance_Angle_Pitch_Neg_Target;
  is >> Balance.Balance_Angle_Pitch_Pos_Target;

  is >> Balance.Balance_Value_2;
  is >> Balance.Balance_Roll_GP;
  is >> Balance.Balance_Roll_GI;
  is >> Balance.Balance_Roll_GD;
  is >> Balance.Balance_Roll_ELIMIT;
  is >> Balance.Balance_Roll_OLIMIT;
  is >> Balance.Balance_Roll_Neg_Target;
  is >> Balance.Balance_Roll_Pos_Target;

  is >> Balance.Balance_Value_3;

  is >> Balance.Balance_Value_4;
  is >> Balance.Balance_Value_5;
  is >> Balance.Balance_Pitch_GP_imu;
  is >> Balance.Balance_Pitch_GI_imu;
  is >> Balance.Balance_Pitch_GD_imu;
  is >> Balance.Balance_Pitch_ELIMIT_imu;
  is >> Balance.Balance_Pitch_OLIMIT_imu;
  is >> Balance.Balance_Pitch_Neg_Target_imu;
  is >> Balance.Balance_Pitch_Pos_Target_imu;

  is >> Balance.Balance_Roll_GP_imu;
  is >> Balance.Balance_Roll_GI_imu;
  is >> Balance.Balance_Roll_GD_imu;
  is >> Balance.Balance_Roll_ELIMIT_imu;
  is >> Balance.Balance_Roll_OLIMIT_imu;
  is >> Balance.Balance_Roll_Neg_Target_imu;
  is >> Balance.Balance_Roll_Pos_Target_imu;
  is >> Model_Data.Center2Leg;
  is >> Model_Data.Link2Link;
  is >> Model_Data.Init_Z_Up;

  is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_10;
  is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_11;
  is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_12;
  is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_13;
  is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_14;
  is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_15;
  is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_16;
  is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_17;
  is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_18;
  is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_19;
  is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_20;
  is >> IK.Past_Percentage_of_IK_Motor.Motor_Multiple_21;

  // K_value //
  is >> K_value[0].Pos_XR;
  is >> K_value[0].Neg_XR;
  is >> K_value[0].Pos_SideR;
  is >> K_value[0].Neg_SideR;
  is >> K_value[0].Pos_YawR;
  is >> K_value[0].Neg_YawR;
  is >> K_value[0].Pos_XL;
  is >> K_value[0].Neg_XL;
  is >> K_value[0].Pos_SideL;
  is >> K_value[0].Neg_SideL;
  is >> K_value[0].Pos_YawL;
  is >> K_value[0].Neg_YawL;
  is >> K_value[0].Pos_SideR_SwingMinus;
  is >> K_value[0].Neg_SideR_SwingMinus;
  is >> K_value[0].Pos_SideL_SwingMinus;
  is >> K_value[0].Neg_SideL_SwingMinus;
  is >> K_value[0].min;
  is >> K_value[0].max;
  is >> K_value[1].Pos_XR;
  is >> K_value[1].Neg_XR;
  is >> K_value[1].Pos_SideR;
  is >> K_value[1].Neg_SideR;
  is >> K_value[1].Pos_YawR;
  is >> K_value[1].Neg_YawR;
  is >> K_value[1].Pos_XL;
  is >> K_value[1].Neg_XL;
  is >> K_value[1].Pos_SideL;
  is >> K_value[1].Neg_SideL;
  is >> K_value[1].Pos_YawL;
  is >> K_value[1].Neg_YawL;
  is >> K_value[1].Pos_SideR_SwingMinus;
  is >> K_value[1].Neg_SideR_SwingMinus;
  is >> K_value[1].Pos_SideL_SwingMinus;
  is >> K_value[1].Neg_SideL_SwingMinus;
  is >> K_value[1].min;
  is >> K_value[1].max;
  is >> K_value[2].Pos_XR;
  is >> K_value[2].Neg_XR;
  is >> K_value[2].Pos_SideR;
  is >> K_value[2].Neg_SideR;
  is >> K_value[2].Pos_YawR;
  is >> K_value[2].Neg_YawR;
  is >> K_value[2].Pos_XL;
  is >> K_value[2].Neg_XL;
  is >> K_value[2].Pos_SideL;
  is >> K_value[2].Neg_SideL;
  is >> K_value[2].Pos_YawL;
  is >> K_value[2].Neg_YawL;
  is >> K_value[2].Pos_SideR_SwingMinus;
  is >> K_value[2].Neg_SideR_SwingMinus;
  is >> K_value[2].Pos_SideL_SwingMinus;
  is >> K_value[2].Neg_SideL_SwingMinus;
  is >> K_value[2].min;
  is >> K_value[2].max;
  is >> K_value[3].Pos_XR;
  is >> K_value[3].Neg_XR;
  is >> K_value[3].Pos_SideR;
  is >> K_value[3].Neg_SideR;
  is >> K_value[3].Pos_YawR;
  is >> K_value[3].Neg_YawR;
  is >> K_value[3].Pos_XL;
  is >> K_value[3].Neg_XL;
  is >> K_value[3].Pos_SideL;
  is >> K_value[3].Neg_SideL;
  is >> K_value[3].Pos_YawL;
  is >> K_value[3].Neg_YawL;
  is >> K_value[3].Pos_SideR_SwingMinus;
  is >> K_value[3].Neg_SideR_SwingMinus;
  is >> K_value[3].Pos_SideL_SwingMinus;
  is >> K_value[3].Neg_SideL_SwingMinus;
  is >> K_value[3].min;
  is >> K_value[3].max;

  is >> Balance.Landing_Time_Control_flag;

  is >> Balance.Balance_Pitch_Flag_imu;
  is >> Balance.Balance_Roll_Flag_imu;
  is >> Balance.Balance_Pitch_Flag;
  is >> Balance.Balance_Ankle_Pitch_Flag;
  is >> Balance.Balance_Roll_Flag;

  // Balance flag on/off
  Balance.Balance_Ankle_Pitch_Flag = false;
  Balance.Balance_Pitch_Flag = true;
  Balance.Balance_Roll_Flag = true;
  Balance.Balance_Pitch_Flag_imu = true;
  Balance.Balance_Roll_Flag_imu = false;

  // Default//
  Now_Param.Y.Default_Y_Right = -(Model_Data.Center2Leg);
  Now_Param.Y.Default_Y_Left = Model_Data.Center2Leg;
  Now_Param.Z.Default_Z_Right = -(2 * Model_Data.Link2Link - Model_Data.Init_Z_Up);
  Now_Param.Z.Default_Z_Left = -(2 * Model_Data.Link2Link - Model_Data.Init_Z_Up);

  // Offset & % of IK init//
  IK.Now_Motor_Angle = IK.Past_Motor_Angle;
  IK.Now_Percentage_of_IK_Motor = IK.Past_Percentage_of_IK_Motor;
}

void IKwalk::timer_callback()
{
  imu_count++;
  static int Milli_Second = 0;
  Milli_Second++;
 if(Entire_time_flag ==1) Past_Param.Entire_Time -=0.07;
  else if(Entire_time_flag ==0) Past_Param.Entire_Time +=0.07;
  if(Past_Param.Entire_Time <= 43 ) Past_Param.Entire_Time = 43;
  else if(Past_Param.Entire_Time >= 50) Past_Param.Entire_Time = 50;
  if (Milli_Second >= Now_Param.Frequency)
  {
    Milli_Second = 0;

    Timer_Time += Now_Param.Frequency;
    Timer_Time_Start += Now_Param.Frequency;

    if (End_Flag)
      Timer_Time_End += Now_Param.Frequency;

    if (Timer_Time >= Now_Param.Entire_Time)
    {
      Timer_Time = 0;
    }
    if (Timer_Time_Start >= Now_Param.Sink_Entire_Time)
    {
      Timer_Time_Start = 0;
    }
    if (Timer_Time_End >= Now_Param.End_Entire_Time)
    {
      Timer_Time_End = 0;
    }
    if(kick_time)  Kick_Return.generate_kick_trajectory(Now_Param.Z.Default_Z_Right);
    Walk_Start_End(Now_Param, Past_Param);
    if (imu_count >= 10)
    {
      Adjust.desire_yaw = first_yaw;
      // cout << "desire_yaw: " << Adjust.desire_yaw << endl;
      imu_count = 0;
    }

    if (Now_Param.Check_ratio.Ratio_Flag != Past_Param.Check_ratio.Ratio_Flag)
    {
      Now_Param.Check_ratio.Ratio_Flag = Past_Param.Check_ratio.Ratio_Flag;
      Generate_Pattern(Now_Param);
    }

    Now_Param.Z.Z_com = 325.0 - Model_Data.Init_Z_Up;
    IkCOM.x_com = (Now_Param.Z.Z_com / 9.81) * ((Amount_of_Change_X) / (Now_Param.Entire_Time * 0.01)) + ZMP.total_y_zmp;
    IkCOM.y_com = (Now_Param.Z.Z_com / 9.81) * ((Amount_of_Change_Y) / (Now_Param.Entire_Time * 0.01)) + ZMP.total_x_zmp;
    COM_Pub->publish(IkCOM);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKwalk>("ik_walk");

  node->IK.init_save();
  std::cout << "[DEBUG] init_save done" << std::endl;
  rclcpp::sleep_for(std::chrono::seconds(1));

  node->get_parameters();
  std::cout << "[DEBUG] get_parameters done" << std::endl;

  Balance.Landing_Error_L = true;
  Balance.Landing_flag_R = true;

  node->Generate_Pattern(Now_Param);
  std::cout << "[DEBUG] Generate_Pattern done" << std::endl;

  node->IK.Balance_Control_Body_Upright(0, Now_Param.Z.Default_Z_Right, Init_Position_Time, Init_Rise_Condition, Init_Position_Pitch, Init_Position_Balance_Msg, Init_Position_Balance_Msg, 0, 0, 0, 0, 0, Model_Data.Link2Link, 0, 0, 99);
  std::cout << "[DEBUG] Balance_Control_Body_Upright done" << std::endl;

  Now_Param.Y.Default_Y_Right = -(Model_Data.Center2Leg);
  Now_Param.Y.Default_Y_Left = Model_Data.Center2Leg;
  Now_Param.Z.Default_Z_Right = -(2 * Model_Data.Link2Link - Model_Data.Init_Z_Up);
  Now_Param.Z.Default_Z_Left = -(2 * Model_Data.Link2Link - Model_Data.Init_Z_Up);
  node->IK.vel = 1;
  std::cout << "[DEBUG] Before solve" << std::endl;
  node->IK.solve(0.0, Now_Param.Y.Default_Y_Right, Now_Param.Z.Default_Z_Right, 0.0, 0.0, Now_Param.Y.Default_Y_Left, Now_Param.Z.Default_Z_Left, 0.0, Leg, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  std::cout << "[DEBUG] solve done" << std::endl;

  rclcpp::sleep_for(std::chrono::seconds(3));
  node->IK.vel = 0;

  std::cout << "[DEBUG] Starting spin..." << std::endl;
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

void IKwalk::Walk_Start_End(Walk_Param &Now_Param, Walk_Param &Past_Param)
{
  humanoid_interfaces::msg::IkEndMsg IkEnd;
  humanoid_interfaces::msg::IkLTCMsg IkLTC;

  if (Past_Param.IK_Flag != Ik_Flag_Past)
  {
    if (Past_Param.IK_Flag)
    {
      Start_Flag = true;
      // cout << "Start_Flag" << endl;
    }
    else if (!Past_Param.IK_Flag && Timer_Time == 0)
    {
      End_Flag = true;
      // cout << "End_Flag" << endl;
    }
  }
  if (Start_Flag)
  {
    IK.Now_Motor_Angle = IK.Past_Motor_Angle;
    IK.Now_Percentage_of_IK_Motor = IK.Past_Percentage_of_IK_Motor;
    if (Timer_Time_Start == 0.0)
    {
      Now_Param.IK_Flag = true;

      cout << "%%%%%%%%%%%%%%%%%%" << Start_Cnt << "%%%%%%%%%%%%\n";
      Start_Cnt++;
    }
    // cout << "Repeat_Start_Cycle: " << Repeat_Start_Cycle << endl;

    if (Start_Cnt >= Repeat_Start_Cycle)
    {
      Timer_Time = Timer_Time_Start;
      if (fabs(Timer_Time_Start - Timer_Time) < 3)
      {
        Start_Cnt = 1;
        Start_Flag = false;
        IkEnd.ikend = 0;
        Ikend_Pub->publish(IkEnd);
      }
    }
  }

  else if (End_Flag)
  {
    if (Timer_Time_End == 0)
    {
      End_Cnt--;
    }
    if (End_Cnt < 1 && Timer_Time_End == 0)
    {
      End_Cnt = Repeat_End_Cycle;
      End_Flag = false;
      Now_Param.IK_Flag = false;
      Ik_Flag_Past = false;
      Timer_Time_End = 0.0;

      Balance.Swing_Control_L = 1.0;
      Balance.Swing_Control_R = 1.0;
      Balance.Swing_Control_add_L = 1.0;
      Balance.Swing_Control_add_R = 1.0;
      Balance.Swing_Control_Safe_cnt = 0;
      Balance.Swing_Control_Warning_cnt = 0;
      IkLTC.swing_gain_l = Balance.Swing_Control_L;
      IkLTC.swing_gain_r = Balance.Swing_Control_R;
      IkLTC.landing_time_l = 0;
      IkLTC.landing_time_r = 0;
      IkLTC.landing_error_l = 0;
      IkLTC.landing_error_r = 0;
      Landing_Pub->publish(IkLTC);

      IK.Now_Motor_Angle = IK.Past_Motor_Angle;
      IK.Now_Percentage_of_IK_Motor = IK.Past_Percentage_of_IK_Motor;
      IK.solve(0.0, Now_Param.Y.Default_Y_Right, Now_Param.Z.Default_Z_Right, 0.0, 0.0, Now_Param.Y.Default_Y_Left, Now_Param.Z.Default_Z_Left, 0.0, Leg, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      IkEnd.ikend = 1;
      Ikend_Pub->publish(IkEnd);
    }
  }

  if (Now_Param.IK_Flag && (Start_Flag || End_Flag))
  {
    // cout << "result" << endl;
    Result_Pattern(Now_Param);
    if (Start_Flag)
      End_Cnt = Repeat_End_Cycle;
    else if (End_Flag)
      Start_Cnt = 1;
  }
  else if (Now_Param.IK_Flag && (!Start_Flag && !End_Flag))
  {
    // cout << "normal\n\n";
    Result_Pattern(Now_Param);
  }
}

void IKwalk::Generate_Pattern(Walk_Param &Now_Param)
{
  //-------------------------Single_OR_Double_Pattern-------------------------//

  if (Balance.Ratio_Check_Flag == 1)
  {
    //-------------------------Start_Pattern-------------------------//
    Start_COM_Pattern.put_point(0.0, 0.0, 7.0, 0.0);
    Start_COM_Pattern.put_point(0.1, 0.7, 0.0, 0.0);
    Start_COM_Pattern.put_point(0.2, 1.0, 0.0, 0.0);
    Start_COM_Pattern.put_point(0.3, 1.0, 0.0, 0.0);
    Start_COM_Pattern.put_point(0.4, 0.7, -7.0, 0.0);
    Start_COM_Pattern.put_point(0.5, 0.0, -7.0, 0.0);
    Start_COM_Pattern.put_point(0.6, -0.7, -7.0, 0.0);
    Start_COM_Pattern.put_point(0.7, -1.0, 0.0, 0.0);
    Start_COM_Pattern.put_point(0.8, -1.0, 0.0, 0.0);
    Start_COM_Pattern.put_point(0.9, -0.7, 0.0, 0.0);
    Start_COM_Pattern.put_point(1.0, 0.0, 7.0, 0.0);

    Start_Rise_Pattern.put_point(0.0, 0.0, 0.0, 0.0);
    Start_Rise_Pattern.put_point(0.1, 0.0, 0.0, 0.0);
    Start_Rise_Pattern.put_point(0.2, 0.7, 0.0, 0.0);
    Start_Rise_Pattern.put_point(0.3, 1.0, 0.0, 0.0);
    Start_Rise_Pattern.put_point(0.5, 0.0, 0.0, 0.0);
    Start_Rise_Pattern.put_point(1.0, 0.0, 0.0, 0.0);

    //-------------------------Normal Pattern---------------------------//
    Side_Pattern.put_point(0.0, 0.0, 0.0, 0.0);
    Side_Pattern.put_point(0.1, 0.0, 0.0, 0.0);
    Side_Pattern.put_point(0.2, 1.0, 0.0, 0.0);
    Side_Pattern.put_point(0.3, 1.0, 0.0, 0.0);
    Side_Pattern.put_point(0.4, 1.0, 0.0, 0.0);
    Side_Pattern.put_point(0.5, 0.0, 0.0, 0.0);
    Side_Pattern.put_point(0.7, 0.0, 0.0, 0.0);
    Side_Pattern.put_point(1.0, 0.0, 0.0, 0.0);

    Step_Pattern.put_point(0.0, -0.5, 0.0, 0.0);
    // Step_Pattern.put_point(0.1,-0.5, 0.0,0.0);
    // Step_Pattern.put_point(0.4, 0.5, 0.0,0.0);
    Step_Pattern.put_point(0.5, 0.5, 0.0, 0.0);
    // Step_Pattern.put_point(0.6, 0.5, 0.0,0.0);
    // Step_Pattern.put_point(0.9,-0.5, 0.0,0.0);
    Step_Pattern.put_point(1.0, -0.5, 0.0, 0.0);

    // Step_Pattern.put_point(0.0,-0.5, 0.0,0.0);
    // Step_Pattern.put_point(0.375, 0.5, 0.0,0.0);
    // Step_Pattern.put_point(1.0,-0.5, 0.0,0.0);

    COM_Pattern.put_point(0.0, -0.5, 0.0, 0.0);
    COM_Pattern.put_point(0.2, 0.5, 0.0, 0.0);
    COM_Pattern.put_point(0.5, 0.5, 0.0, 0.0);
    COM_Pattern.put_point(0.7, -0.5, 0.0, 0.0);
    COM_Pattern.put_point(1.0, -0.5, 0.0, 0.0);

    Rise_Pattern.put_point(0.0, 0.0, 0.0, 0.0);
    Rise_Pattern.put_point(0.1, 0.0, 0.0, 0.0);
    Rise_Pattern.put_point(0.2, 0.7, 0.0, 0.0);
    Rise_Pattern.put_point(0.3, 1.0, 0.0, 0.0);
    Rise_Pattern.put_point(0.5, 0.0, 0.0, 0.0);
    Rise_Pattern.put_point(0.6, 0.0, 0.0, 0.0);
    Rise_Pattern.put_point(1.0, 0.0, 0.0, 0.0);

    Turn_Pattern.put_point(0.0, 0.0, 0.0, 0.0);
    Turn_Pattern.put_point(0.1, 0.0, 0.0, 0.0);
    Turn_Pattern.put_point(0.3, 1.0, 0.0, 0.0);
    Turn_Pattern.put_point(0.6, 1.0, 0.0, 0.0);
    Turn_Pattern.put_point(0.7, 0.4, 0.0, 0.0);
    Turn_Pattern.put_point(0.9, 0.0, 0.0, 0.0);
    Turn_Pattern.put_point(1.0, 0.0, 0.0, 0.0);
  }
  else if (Balance.Ratio_Check_Flag == 0)
  {
    // if you need more pattern, add pattern.
  }
}

void IKwalk::Result_Pattern(Walk_Param &Now_Param)
{
  humanoid_interfaces::msg::IkCoordMsg IkCoord;
  humanoid_interfaces::msg::IkPatternMsg IkPattern;

  Time_Right_Leg_Start = Timer_Time_Start / Now_Param.Sink_Entire_Time + 0.5;

  Time_Left_Leg_Start = Timer_Time_Start / Now_Param.Sink_Entire_Time;

  Now_Param.Entire_Time = Past_Param.Entire_Time;

  Time_Right_Leg = Timer_Time / Now_Param.Entire_Time + 0.5;
  Time_Left_Leg = Timer_Time / Now_Param.Entire_Time;

  Time_Right_Leg_End = Timer_Time_End / Now_Param.End_Entire_Time + 0.5;
  Time_Left_Leg_End = Timer_Time_End / Now_Param.End_Entire_Time;

  if (Time_Right_Leg_Start >= 1.0)
    Time_Right_Leg_Start -= 1.0;
  if (Time_Right_Leg >= 1.0)
    {Time_Right_Leg -= 1.0; cycle_flag++;}
  if (Time_Right_Leg_End >= 1.0)
    Time_Right_Leg_End -= 1.0;

  if((Time_Right_Leg - 0.25 >= -0.02 && Time_Right_Leg - 0.75 <= 0.02)|| (Time_Left_Leg - 0.25 >= -0.02 && Time_Left_Leg - 0.75 <= 0.02)) kick_interval = true;
  else {kick_interval = false; kick_time = false; }

  if(kick_flag_ && kick_flag_msg_ == "left" && fabs(Time_Left_Leg - 0.25) < 0.05)
      kick_l_armed_ = true;

  if(kick_flag_ && kick_flag_msg_ == "right" && fabs(Time_Right_Leg - 0.25) < 0.05)
      kick_r_armed_ = true;

  if (Balance.Ratio_Check_Flag == 1)
    Past_Param.Check_ratio.Ratio_Flag = 1;
  else if (Balance.Ratio_Check_Flag == 0)
    Past_Param.Check_ratio.Ratio_Flag = 0;

  // Run_flag 기준 설정(Run 상태에 진입시 기구식 조건 일부 해제)
  if (Now_Param.X.X >= 1)
    Run_Flag = false;
  else
    Run_Flag = false;

  // Ankle_Flag 설정(뒤로 가거나 Side시 발목을 사용하여 걷지 않도록)
  if ((Now_Param.X.X <= 0) || (Now_Param.Y.Side != 0))
    Ankle_Flag = false;
  else
    Ankle_Flag = true;

  if (Time_Right_Leg >= 0 && Time_Right_Leg <= 0.02)
  {


    Now_Param.Check_ratio = Past_Param.Check_ratio;
    Now_Param.Y.Default_Y_Right = Past_Param.Y.Default_Y_Right;
    Now_Param.Y.Default_Y_Left = Past_Param.Y.Default_Y_Left;
    Now_Param.Z.Default_Z_Right = Past_Param.Z.Default_Z_Right;
    Now_Param.Z.Default_Z_Left = Past_Param.Z.Default_Z_Left;
    Now_Param.Shoulder.Swing_Shoulder_Right = Past_Param.Shoulder.Swing_Shoulder_Right;
    Now_Param.Shoulder.Swing_Shoulder_Left = Past_Param.Shoulder.Swing_Shoulder_Left;
    Now_Param.Frequency = Past_Param.Frequency;
    Now_Param.Y.Swing_Leg_Right = Past_Param.Y.Swing_Leg_Right;
    Now_Param.Y.Swing_Leg_Left = Past_Param.Y.Swing_Leg_Left;
    Now_Param.Z.Rise_Leg_Right = Past_Param.Z.Rise_Leg_Right;
    Now_Param.Z.Rise_Leg_Left = Past_Param.Z.Rise_Leg_Left;
    Now_Param.X = Past_Param.X;
    Now_Param.Yaw_R = Past_Param.Yaw_R;
    Now_Param.Yaw_L = Past_Param.Yaw_L;
    Now_Param.Yaw_R.Tuning_Yaw = Past_Param.Yaw_R.Tuning_Yaw;
    Now_Param.Yaw_L.Tuning_Yaw = Past_Param.Yaw_L.Tuning_Yaw;

    if (Acc.accel_pos_x_cnt >= 0 && Acc.accel_pos_x_cnt <= Acc.accel_pos_x_num)
      Acc.accel_pos_x_cnt++;
    if (Acc.accel_neg_x_cnt >= 0 && Acc.accel_neg_x_cnt <= Acc.accel_neg_x_num)
      Acc.accel_neg_x_cnt++;
    if (Acc.accel_pos_y_cnt >= 0 && Acc.accel_pos_y_cnt <= Acc.accel_pos_y_num)
      Acc.accel_pos_y_cnt++;
    if (Acc.accel_neg_y_cnt >= 0 && Acc.accel_neg_y_cnt <= Acc.accel_neg_y_num)
      Acc.accel_neg_y_cnt++;
    if (Acc.accel_pos_z_cnt >= 0 && Acc.accel_pos_z_cnt <= Acc.accel_pos_z_num)
      Acc.accel_pos_z_cnt++;
    if (Acc.accel_neg_z_cnt >= 0 && Acc.accel_neg_z_cnt <= Acc.accel_neg_z_num)
      Acc.accel_neg_z_cnt++;
  }
  else if((kick_l_armed_|| kick_time) && kick_flag_msg_ == "left" && Time_Left_Leg - 0.25 >= -0.02 && Time_Left_Leg - 0.55 <= 0.02){
    IK.solve_kick(0.0, Now_Param.Y.Default_Y_Right, Now_Param.Z.Default_Z_Right, 0.0, 0.0, Now_Param.Y.Default_Y_Left, Now_Param.Z.Default_Z_Left-20.0, 0.0, Leg, IMU.pitch, Balance.Balance_Pitch_Pos_Target_imu, Balance.Balance_Pitch_Neg_Target_imu, IMU.roll, Now_Param.X.X, -10.0, true);
    cout<<"Kick!"<<endl;
    kick_time = true;
    kick_flag_ = false;
    kick_l_armed_ = false;
    ik_1 = true;
    return;
  }
  else if(ik_1 && kick_flag_msg_ == "left" && Time_Left_Leg - 0.55  >= -0.02 && Time_Left_Leg - 1.0 <= 0.02){
    cout<<"1"<<endl;
    kick_time = false;
    IK.solve_kick(0.0, Now_Param.Y.Default_Y_Right, Now_Param.Z.Default_Z_Right, 0.0, 150.0, Now_Param.Y.Default_Y_Left, Now_Param.Z.Default_Z_Left+25.0, 0.0, Leg, IMU.pitch, Balance.Balance_Pitch_Pos_Target_imu, Balance.Balance_Pitch_Neg_Target_imu, IMU.roll, Now_Param.X.X, -10.0, true);
    cout<<"Kick shoot!"<<endl;
    ik_2 = true;
    return;
  }
  else if(kick_flag_msg_ == "left" && ik_2) {
     cout<<"2"<<endl;
    if(ik_3 && fabs(Time_Left_Leg-0.2) <= 0.02 && cycle_flag >=2) {{ std_msgs::msg::Bool _b; _b.data = true; kick_flag_pub_->publish(_b); } ik_2 = false; ik_3= false; kick_return_t = 0.0; if(kick_pending_){
      if(kick_pending_start_time_ < 0.0) { kick_pending_start_time_ = Timer_Time; kick_pending_wait_cnt_ = 0; }
      double dt = Timer_Time - kick_pending_start_time_;
      if(dt < 0) dt += Now_Param.Entire_Time;
      kick_pending_wait_cnt_ += (int)dt;
      kick_pending_start_time_ = Timer_Time;
      ik_2 = true; ik_3 = true;
      if(kick_pending_wait_cnt_ < 150) { return; }
      else { ik_2 = false; ik_3 = false; kick_pending_start_time_ = -1.0; kick_pending_wait_cnt_ = 0; kick_flag_ = true; kick_pending_ = false; kick_flag_msg_ = kick_flag_msg_pending; }
    } else{ Start_Flag = true; } Acc.basic_x = 0.0;
double target = Past_Param.X.X;
if(target > 0){
    Acc.accel_pos_x = target;
    Acc.accel_pos_x_cnt = 0;
    Acc.accel_neg_x_cnt = -1;
} else if(target < 0){
    Acc.accel_neg_x = target;
    Acc.accel_neg_x_cnt = 0;
    Acc.accel_pos_x_cnt = -1;
} else {
    Acc.accel_pos_x_cnt = -1;
    Acc.accel_neg_x_cnt = -1;
} return; }
    else if(ik_3) return;
     cout<<"Kick return!"<<endl;
    double ex, ez;
    ik_1 = false;
    Kick_Return.result(kick_return_t, ex, ez);
    IK.solve_kick(0.0, Now_Param.Y.Default_Y_Right, Now_Param.Z.Default_Z_Right, 0.0,
                  ex, Now_Param.Y.Default_Y_Left, ez, 0.0,
                  Leg,
                  IMU.pitch, Balance.Balance_Pitch_Pos_Target_imu,
                  Balance.Balance_Pitch_Neg_Target_imu,
                  IMU.roll, Now_Param.X.X, -10.0, true);
    kick_return_t += 0.01;
    if(kick_return_t >= Kick_Return.get_duration()) {
      kick_return_t = 0.0;
      IK.solve_kick(0.0, Now_Param.Y.Default_Y_Right, Now_Param.Z.Default_Z_Right, 0.0,
                  ex, Now_Param.Y.Default_Y_Left, ez, 0.0,
                  Leg,
                  IMU.pitch, Balance.Balance_Pitch_Pos_Target_imu,
                  Balance.Balance_Pitch_Neg_Target_imu,
                  IMU.roll, Now_Param.X.X, 0.0, true);
      cout << "kick return done" << endl;
      ik_3 = true;
      kick_time = false;
      cycle_flag=0;
    }
    return;
  }
  
  else if((kick_r_armed_|| kick_time) && kick_flag_msg_ == "right" && Time_Right_Leg - 0.25 >= -0.02 && Time_Right_Leg - 0.55 <= 0.02){
    IK.solve_kick(0.0, Now_Param.Y.Default_Y_Right, Now_Param.Z.Default_Z_Right-20.0, 0.0, 0.0, Now_Param.Y.Default_Y_Left, Now_Param.Z.Default_Z_Left, 0.0, Leg, IMU.pitch, Balance.Balance_Pitch_Pos_Target_imu, Balance.Balance_Pitch_Neg_Target_imu, IMU.roll, Now_Param.X.X, -10.0, false);
    cout<<"Kick!"<<endl;
    kick_time = true;
    kick_flag_ = false;
    ik_1 = true;
    kick_r_armed_ = false;
  
    return;
  }
  else if(ik_1 && kick_flag_msg_ == "right" && Time_Right_Leg - 0.55 >= -0.02 && Time_Right_Leg - 1.0 <= 0.02){
    kick_time = false;
    IK.solve_kick(150.0, Now_Param.Y.Default_Y_Right, Now_Param.Z.Default_Z_Right+25.0, 0.0, 0.0, Now_Param.Y.Default_Y_Left, Now_Param.Z.Default_Z_Left, 0.0, Leg, IMU.pitch, Balance.Balance_Pitch_Pos_Target_imu, Balance.Balance_Pitch_Neg_Target_imu, IMU.roll, Now_Param.X.X, -10.0, false);
    cout<<"Kick shoot!"<<endl;
    ik_2 = true;
    return;
  }
  else if(kick_flag_msg_ == "right" && ik_2) {
     if(ik_3 && fabs(Time_Right_Leg-0.2) <= 0.02 && cycle_flag >=2) {{ std_msgs::msg::Bool _b; _b.data = true; kick_flag_pub_->publish(_b); } ik_2 = false; ik_3= false; kick_return_t = 0.0; if(kick_pending_){
      if(kick_pending_start_time_ < 0.0) { kick_pending_start_time_ = Timer_Time; kick_pending_wait_cnt_ = 0; }
      double dt = Timer_Time - kick_pending_start_time_;
      if(dt < 0) dt += Now_Param.Entire_Time;
      kick_pending_wait_cnt_ += (int)dt;
      kick_pending_start_time_ = Timer_Time;
      ik_2 = true; ik_3 = true;
      if(kick_pending_wait_cnt_ < 150) { return; }
      else { ik_2 = false; ik_3 = false; kick_pending_start_time_ = -1.0; kick_pending_wait_cnt_ = 0; kick_flag_ = true; kick_pending_ = false; kick_flag_msg_ = kick_flag_msg_pending; }
    } else{ Start_Flag = true; } Acc.basic_x = 0.0;
double target = Past_Param.X.X;
if(target > 0){
    Acc.accel_pos_x = target;
    Acc.accel_pos_x_cnt = 0;
    Acc.accel_neg_x_cnt = -1;
} else if(target < 0){
    Acc.accel_neg_x = target;
    Acc.accel_neg_x_cnt = 0;
    Acc.accel_pos_x_cnt = -1;
} else {
    Acc.accel_pos_x_cnt = -1;
    Acc.accel_neg_x_cnt = -1;
} return;}
    else if(ik_3) return;
     cout<<"Kick return!"<<endl;
    double ex, ez;
    ik_1 = false;
    Kick_Return.result(kick_return_t, ex, ez);
    IK.solve_kick(ex, Now_Param.Y.Default_Y_Right, ez, 0.0,
                  0.0, Now_Param.Y.Default_Y_Left, Now_Param.Z.Default_Z_Left, 0.0,
                  Leg,
                  IMU.pitch, Balance.Balance_Pitch_Pos_Target_imu,
                  Balance.Balance_Pitch_Neg_Target_imu,
                  IMU.roll, Now_Param.X.X, -10.0, false);
    kick_return_t += 0.01;
    if(kick_return_t >= Kick_Return.get_duration()) {
      kick_return_t = 0.0;
      IK.solve_kick(ex, Now_Param.Y.Default_Y_Right, ez, 0.0,
                  0.0, Now_Param.Y.Default_Y_Left, Now_Param.Z.Default_Z_Left, 0.0,
                  Leg,
                  IMU.pitch, Balance.Balance_Pitch_Pos_Target_imu,
                  Balance.Balance_Pitch_Neg_Target_imu,
                  IMU.roll, Now_Param.X.X, 0.0, false);
      cout << "kick return done" << endl;
      ik_3 = true;
      kick_time = false;
      cycle_flag=0;
    }
    return;
  }
  else if (fabs(Time_Right_Leg - 0.5) <= 0.02 || fabs(Time_Right_Leg - 0.25) <= 0.02 ||
           fabs(Time_Right_Leg - 0.75) <= 0.02)
  {


    Now_Param.X = Past_Param.X;
    Now_Param.Y = Past_Param.Y;
    Now_Param.Yaw_R = Past_Param.Yaw_R;
    Now_Param.Yaw_L = Past_Param.Yaw_L;

    if (Now_Param.X.X != Past_Param.X.X)
    {
      Temp_Param_X = Now_Param.X.X;
      X_Change = true;
    }
    else
    {
      X_Change = false;
      Time_X = 0.0;
    }

    if (Now_Param.Y.Side != Past_Param.Y.Side)
    {
      Temp_Param_Side = Now_Param.Y.Side;
      Side_Change = true;
    }
    else
    {
      Side_Change = false;
      Time_Side = 0.0;
    }

    if (Now_Param.Yaw_L.Yaw != Past_Param.Yaw_L.Yaw)
    {
      Temp_Param_Yaw_L = Now_Param.Yaw_L.Yaw;
      Yaw_L_Change = true;
    }
    else
    {
      Yaw_L_Change = false;
      Time_Yaw_L = 0.0;
    }

    if (Now_Param.Yaw_R.Yaw != Past_Param.Yaw_R.Yaw)
    {
      Temp_Param_Yaw_R = Now_Param.Yaw_R.Yaw;
      Yaw_R_Change = true;
    }
    else
    {
      Yaw_R_Change = false;
      Time_Yaw_R = 0.0;
    }
  }

  else if ((Time_Right_Leg - 1) <= 0 && (Time_Right_Leg - 1) >= -0.02)
  {
    if (End_Cnt > 1 && End_Flag)
    {
      Now_Param.Y.Side = 0.0;
      Now_Param.X.X = 0.0;
      Now_Param.Yaw_R.Yaw = 0.0;
      Now_Param.Yaw_L.Yaw = 0.0;
      Side_Change = false;
      X_Change = false;
      Yaw_L_Change = false;
      Yaw_R_Change = false;
    }
  }

  if (Side_Change)
  {
    Time_Side++;

    foot_trajectory Param_Pattern;
    Param_Pattern.put_point(0.0, Temp_Param_Side, 0.0, 0.0);
    Param_Pattern.put_point(Now_Param.Entire_Time, Past_Param.Y.Side, 0.0, 0.0);

    Now_Param.Y.Side = Param_Pattern.result(Time_Side);
    if (Now_Param.Y.Side < 0.1 && Now_Param.Y.Side > -0.1)
      Now_Param.Y.Side = 0;

    if (Time_Side >= Now_Param.Entire_Time)
      Time_Side = 0.0;
  }
  if (X_Change)
  {
    Time_X++;

    foot_trajectory Param_Pattern;
    Param_Pattern.put_point(0.0, Temp_Param_X, 0.0, 0.0);
    Param_Pattern.put_point(Now_Param.Entire_Time, Past_Param.X.X, 0.0, 0.0);

    Now_Param.X.X = Param_Pattern.result(Time_X);
    if (Now_Param.X.X < 0.1 && Now_Param.X.X > -0.1)
      Now_Param.X.X = 0;

    if (Time_X >= Now_Param.Entire_Time)
      Time_X = 0.0;
  }
  if (Yaw_L_Change)
  {
    Time_Yaw_L++;
    foot_trajectory Param_Pattern;
    Param_Pattern.put_point(0.0, Temp_Param_Yaw_L, 0.0, 0.0);
    Param_Pattern.put_point(Now_Param.Entire_Time, Past_Param.Yaw_L.Yaw, 0.0, 0.0);

    Now_Param.Yaw_L.Yaw = Param_Pattern.result(Time_Yaw_L);
    if (Now_Param.Yaw_L.Yaw < 0.1 && Now_Param.Yaw_L.Yaw > -0.1)
      Now_Param.Yaw_L.Yaw = 0;

    if (Time_Yaw_L >= Now_Param.Entire_Time)
      Time_Yaw_L = 0.0;
  }
  if (Yaw_R_Change)
  {
    Time_Yaw_R++;
    foot_trajectory Param_Pattern;
    Param_Pattern.put_point(0.0, Temp_Param_Yaw_R, 0.0, 0.0);
    Param_Pattern.put_point(Now_Param.Entire_Time, Past_Param.Yaw_R.Yaw, 0.0, 0.0);

    Now_Param.Yaw_R.Yaw = Param_Pattern.result(Time_Yaw_R);
    if (Now_Param.Yaw_R.Yaw < 0.1 && Now_Param.Yaw_R.Yaw > -0.1)
      Now_Param.Yaw_R.Yaw = 0;

    if (Time_Yaw_R >= Now_Param.Entire_Time)
      Time_Yaw_R = 0.0;
  }
  if (Start_Flag)
  {
    // cout << "\n**************Start_Walking*****************\n\n";
    Now_Param.X.X = 0.0;
    Now_Param.Y.Side = 0.0;
    Now_Param.Yaw_R.Yaw = 0.0;
    Now_Param.Yaw_L.Yaw = 0.0;
    Accel_Rise_R = ((Now_Param.Z.Rise_Leg_Right - Now_Param.Z.Start_Rise) / Repeat_Start_Cycle) * Start_Cnt;
    Accel_Rise_L = ((Now_Param.Z.Rise_Leg_Left - Now_Param.Z.Start_Rise) / Repeat_Start_Cycle) * Start_Cnt;
    Accel_Swing_R = ((Now_Param.Y.Swing_Leg_Right - Now_Param.Y.Start_Swing) / Repeat_Start_Cycle) * Start_Cnt;
    Accel_Swing_L = ((Now_Param.Y.Swing_Leg_Left - Now_Param.Y.Start_Swing) / Repeat_Start_Cycle) * Start_Cnt;
    Accel_Entire_Time = ((Now_Param.Entire_Time - Now_Param.Start_Entire_Time) / Repeat_Start_Cycle) * Start_Cnt;

    Now_Param.Sink_Entire_Time = Now_Param.Start_Entire_Time + Accel_Entire_Time;

    if (Accel_Rise_R > fabs(Now_Param.Z.Rise_Leg_Right - Now_Param.Z.Start_Rise))
      Accel_Rise_R = fabs(Now_Param.Z.Rise_Leg_Right - Now_Param.Z.Start_Rise);
    if (Accel_Rise_L > fabs(Now_Param.Z.Rise_Leg_Left - Now_Param.Z.Start_Rise))
      Accel_Rise_L = fabs(Now_Param.Z.Rise_Leg_Left - Now_Param.Z.Start_Rise);
    if (Accel_Swing_R > fabs(Now_Param.Y.Swing_Leg_Right - Now_Param.Y.Start_Swing))
      Accel_Swing_R = fabs(Now_Param.Y.Swing_Leg_Right - Now_Param.Y.Start_Swing);
    if (Accel_Swing_L > fabs(Now_Param.Y.Swing_Leg_Left - Now_Param.Y.Start_Swing))
      Accel_Swing_L = fabs(Now_Param.Y.Swing_Leg_Left - Now_Param.Y.Start_Swing);
    if (Accel_Entire_Time > fabs(Now_Param.Entire_Time - Now_Param.Start_Entire_Time))
      Accel_Entire_Time = fabs(Now_Param.Entire_Time - Now_Param.Start_Entire_Time);

    if (Acc.accel_pos_x_cnt != -1)
      Acc.accel_pos_x_cnt = 0;
    if (Acc.accel_neg_x_cnt != -1)
      Acc.accel_neg_x_cnt = 0;
    if (Acc.accel_pos_y_cnt != -1)
      Acc.accel_pos_y_cnt = 0;
    if (Acc.accel_pos_z_cnt != -1)
      Acc.accel_pos_z_cnt = 0;

    Normal_first_Flag = true;

    // RIGHT--------------------------------------------------------------------------------------
    Kinetic_X_R = Step_Pattern.result(Time_Right_Leg_Start) * (Now_Param.X.Tuning_X) + Now_Param.X.Default_X_Right;

    Kinetic_Y_R = -Start_COM_Pattern.result(Time_Right_Leg_Start) * (Now_Param.Y.Start_Swing + Accel_Swing_R) + Now_Param.Y.Default_Y_Right;
    Kinetic_Z_R = Start_Rise_Pattern.result(Time_Right_Leg_Start) * (Now_Param.Z.Start_Rise + Accel_Rise_R) + Now_Param.Z.Default_Z_Right;
    Kinetic_Yaw_R = Turn_Pattern.result(Time_Right_Leg_Start) * (Now_Param.Yaw_R.Tuning_Yaw);

    // LEFT--------------------------------------------------------------------------------------
    Kinetic_X_L = Step_Pattern.result(Time_Left_Leg_Start) * (Now_Param.X.Tuning_X) + Now_Param.X.Default_X_Left;
    Kinetic_Y_L = Start_COM_Pattern.result(Time_Left_Leg_Start) * (Now_Param.Y.Start_Swing + Accel_Swing_L) + Now_Param.Y.Default_Y_Left;
    Kinetic_Z_L = Start_Rise_Pattern.result(Time_Left_Leg_Start) * (Now_Param.Z.Start_Rise + Accel_Rise_L) + Now_Param.Z.Default_Z_Left;
    Kinetic_Yaw_L = Turn_Pattern.result(Time_Left_Leg_Start) * (Now_Param.Yaw_L.Tuning_Yaw);
  }

  else if (End_Flag)
  {
    // cout << endl
    //      << "**************End_Walking*****************" << endl
    //      << endl;
    Acc.check_old_x = 0;
    Now_Param.X.X = 0;

    Acc.check_old_y = 0;
    Now_Param.Y.Side = 0;

    Acc.check_old_z = 0;
    Now_Param.Yaw_R.Yaw = 0.0;
    Now_Param.Yaw_L.Yaw = 0.0;

    Acc.accel_pos_x_cnt = -1;
    Acc.accel_neg_x_cnt = -1;
    Acc.accel_pos_y_cnt = -1;
    Acc.accel_pos_z_cnt = -1;

    // RIGHT-------------------------------------------------------------------------------------
    Kinetic_X_R = Now_Param.X.Default_X_Right;
    Kinetic_Y_R = -COM_Pattern.result(Time_Right_Leg_End) * (Now_Param.Y.End_Swing) + Now_Param.Y.Default_Y_Right;
    Kinetic_Z_R = Rise_Pattern.result(Time_Right_Leg_End) * (Now_Param.Z.End_Rise) + Now_Param.Z.Default_Z_Right;
    Kinetic_Yaw_R = 0.0;

    // LEFT--------------------------------------------------------------------------------------
    Kinetic_X_L = Now_Param.X.Default_X_Left;
    Kinetic_Y_L = COM_Pattern.result(Time_Left_Leg_End) * (Now_Param.Y.End_Swing) + Now_Param.Y.Default_Y_Left;
    Kinetic_Z_L = Rise_Pattern.result(Time_Left_Leg_End) * (Now_Param.Z.End_Rise) + Now_Param.Z.Default_Z_Left;
    Kinetic_Yaw_L = 0.0;

    //------------------------------------------------------------------------------------------
    Amount_of_Change_X = Kinetic_X_R - Past_Kinetic_X_R;
    Past_Kinetic_X_R = Kinetic_X_R;
    Amount_of_Change_Y = Kinetic_Y_R - Past_Kinetic_Y_R;
    Past_Kinetic_Y_R = Kinetic_Y_R;
  }
  else
  {
    // cout << "\n**************Normal_Walking*****************\n\n";
    // cout<<-COM_Pattern.result(Time_Right_Leg)<<endl<<Balance.Swing_Control_R<<endl<<Now_Param.Y.Swing_Leg_Right<<endl;//(Cal.negative_position(Now_Param.Yaw_R.Yaw * K_value[0].Neg_YawR) - Cal.positive_position(Now_Param.Yaw_R.Yaw * K_value[0].Pos_YawR)) - (Cal.negative_position(Now_Param.X.X * K_value[0].Neg_XR) +
    // cout<<Cal.positive_position(Now_Param.X.X * K_value[0].Pos_XR)<<endl<<Now_Param.Y.Default_Y_Right<<endl;
    if (Normal_first_Flag == true)
    {
      Now_Param.Sink_Entire_Time = Now_Param.Entire_Time;
      Normal_first_Flag = false;
    }
    // 원하는 값까지 단계적 변화
    if ((Acc.accel_pos_x_cnt >= 0 && Acc.accel_pos_x_cnt <= Acc.accel_pos_x_num) && (Acc.accel_neg_x_cnt == -1))
    {
      Now_Param.X.X = Acc.step_acc_func(Acc.basic_x, Acc.accel_pos_x, Acc.accel_pos_x_num, Acc.accel_pos_x_cnt);
    }
    if ((Acc.accel_neg_x_cnt >= 0 && Acc.accel_neg_x_cnt <= Acc.accel_neg_x_num) && (Acc.accel_pos_x_cnt == -1))
    {
      Now_Param.X.X = Acc.step_acc_func(Acc.basic_x, Acc.accel_neg_x, Acc.accel_neg_x_num, Acc.accel_neg_x_cnt);
    }
    if ((Acc.accel_pos_y_cnt >= 0 && Acc.accel_pos_y_cnt <= Acc.accel_pos_y_num) && (Acc.accel_neg_y_cnt == -1))
    {
      Now_Param.Y.Side = Acc.step_acc_func(Acc.basic_y, Acc.accel_pos_y, Acc.accel_pos_y_num, Acc.accel_pos_y_cnt);
    }
    if ((Acc.accel_neg_y_cnt >= 0 && Acc.accel_neg_y_cnt <= Acc.accel_neg_y_num) && (Acc.accel_pos_y_cnt == -1))
    {
      Now_Param.Y.Side = Acc.step_acc_func(Acc.basic_y, Acc.accel_neg_y, Acc.accel_neg_y_num, Acc.accel_neg_y_cnt);
    }
    if ((Acc.accel_pos_z_cnt >= 0 && Acc.accel_pos_z_cnt <= Acc.accel_pos_z_num) && (Acc.accel_neg_z_cnt == -1))
    {
      Now_Param.Yaw_R.Yaw = Acc.step_acc_func(Acc.basic_z, Acc.accel_pos_z, Acc.accel_pos_z_num, Acc.accel_pos_z_cnt);
      Now_Param.Yaw_L.Yaw = Acc.step_acc_func(Acc.basic_z, Acc.accel_pos_z, Acc.accel_pos_z_num, Acc.accel_pos_z_cnt);
    }
    if ((Acc.accel_neg_z_cnt >= 0 && Acc.accel_neg_z_cnt <= Acc.accel_neg_z_num) && (Acc.accel_pos_z_cnt == -1))
    {
      Now_Param.Yaw_R.Yaw = Acc.step_acc_func(Acc.basic_z, Acc.accel_neg_z, Acc.accel_neg_z_num, Acc.accel_neg_z_cnt);
      Now_Param.Yaw_L.Yaw = Acc.step_acc_func(Acc.basic_z, Acc.accel_neg_z, Acc.accel_neg_z_num, Acc.accel_neg_z_cnt);
    }

    // 외란에 대한 단계적 회복(감속구간>제어구간>회복구간)
    if (imu_safe_cnt || (IK.Now_Balance_Theta.Theta3 != 0 || fabs(IMU.pitch - (Balance.Balance_Pitch_Pos_Target_imu + Balance.Balance_Pitch_Neg_Target_imu) / 2) > 6))
    {
      imu_safe_cnt++;
      if (imu_safe_cnt <= 150)
        Now_Param.X.X = Past_Param.X.X * 0.9;
      else if (imu_safe_cnt <= 200)
        Now_Param.X.X = Past_Param.X.X * 0.6;
      else if (imu_safe_cnt < 500)
        Now_Param.X.X = Past_Param.X.X * ((0.4 * (imu_safe_cnt - 200)) / 300 + 0.6);
      if (imu_safe_cnt == 500)
        imu_safe_cnt = 0;
    }

    if (Balance.Swing_Control_add_L > 1.5)
      Balance.Swing_Control_add_L = 1.5;
    if (Balance.Swing_Control_add_R > 1.5)
      Balance.Swing_Control_add_R = 1.5;
    if (Balance.Swing_Control_add_L < -0.5)
      Balance.Swing_Control_add_L = -0.5;
    if (Balance.Swing_Control_add_R < -0.5)
      Balance.Swing_Control_add_R = -0.5;
    if (abs(Balance.Swing_Control_add_L) < 0.05)
      Balance.Swing_Control_add_L = 0;
    if (abs(Balance.Swing_Control_add_R) < 0.05)
      Balance.Swing_Control_add_R = 0;

    // master node debug line sytart ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // adjust swing

    if (Now_Param.X.X >= K_value[3].min && Now_Param.X.X <= K_value[3].max)
    {
      // RIGHT-------------------------------------------------------------------------------------
      Kinetic_Y_R = -COM_Pattern.result(Time_Right_Leg) * Balance.Swing_Control_R * (Now_Param.Y.Swing_Leg_Right - (Cal.negative_position(Now_Param.Yaw_R.Yaw * K_value[3].Neg_YawR) - Cal.positive_position(Now_Param.Yaw_R.Yaw * K_value[3].Pos_YawR)) - (Cal.negative_position(Now_Param.X.X * K_value[3].Neg_XR) + Cal.positive_position(Now_Param.X.X * K_value[3].Pos_XR))) + Now_Param.Y.Default_Y_Right;

      if (-COM_Pattern.result(Time_Right_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_R -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[3].Neg_SideR_SwingMinus;
      else if (-COM_Pattern.result(Time_Right_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_R -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[3].Neg_SideR;
      else if (-COM_Pattern.result(Time_Right_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_R += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[3].Pos_SideR_SwingMinus;
      else if (-COM_Pattern.result(Time_Right_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_R += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[3].Pos_SideR;
      Kinetic_Y_R += ((Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * Side_Pattern.result(Time_Right_Leg));
      // LEFT--------------------------------------------------------------------------------------
      Kinetic_Y_L = COM_Pattern.result(Time_Left_Leg) * Balance.Swing_Control_L * (Now_Param.Y.Swing_Leg_Left - (Cal.negative_position(Now_Param.Yaw_L.Yaw * K_value[3].Neg_YawL) - Cal.positive_position(Now_Param.Yaw_L.Yaw * K_value[3].Pos_YawL)) - (Cal.negative_position(Now_Param.X.X * K_value[3].Neg_XL) + Cal.positive_position(Now_Param.X.X * K_value[3].Pos_XL))) + Now_Param.Y.Default_Y_Left;

      if (COM_Pattern.result(Time_Left_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_L += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[3].Pos_SideL_SwingMinus;
      else if (COM_Pattern.result(Time_Left_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_L += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[3].Pos_SideL;
      else if (COM_Pattern.result(Time_Left_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_L -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[3].Neg_SideL_SwingMinus;
      else if (COM_Pattern.result(Time_Left_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_L -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[3].Neg_SideL;

      Kinetic_Y_L += ((Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * Side_Pattern.result(Time_Left_Leg));
    }
    else if (Now_Param.X.X >= K_value[2].min && Now_Param.X.X < K_value[2].max)
    {
      // RIGHT-------------------------------------------------------------------------------------
      Kinetic_Y_R = -COM_Pattern.result(Time_Right_Leg) * Balance.Swing_Control_R * (Now_Param.Y.Swing_Leg_Right - (Cal.negative_position(Now_Param.Yaw_R.Yaw * K_value[2].Neg_YawR) - Cal.positive_position(Now_Param.Yaw_R.Yaw * K_value[2].Pos_YawR)) - (Cal.negative_position(Now_Param.X.X * K_value[2].Neg_XR) + Cal.positive_position(Now_Param.X.X * K_value[2].Pos_XR))) + Now_Param.Y.Default_Y_Right;

      if (-COM_Pattern.result(Time_Right_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_R -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[0].Neg_SideR_SwingMinus;
      else if (-COM_Pattern.result(Time_Right_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_R -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[0].Neg_SideR;
      else if (-COM_Pattern.result(Time_Right_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_R += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[0].Pos_SideR_SwingMinus;
      else if (-COM_Pattern.result(Time_Right_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_R += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[0].Pos_SideR;
      Kinetic_Y_R += ((Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * Side_Pattern.result(Time_Right_Leg));
      // LEFT--------------------------------------------------------------------------------------
      Kinetic_Y_L = COM_Pattern.result(Time_Left_Leg) * Balance.Swing_Control_L * (Now_Param.Y.Swing_Leg_Left - (Cal.negative_position(Now_Param.Yaw_L.Yaw * K_value[2].Neg_YawL) - Cal.positive_position(Now_Param.Yaw_L.Yaw * K_value[2].Pos_YawL)) - (Cal.negative_position(Now_Param.X.X * K_value[2].Neg_XL) + Cal.positive_position(Now_Param.X.X * K_value[2].Pos_XL))) + Now_Param.Y.Default_Y_Left;

      if (COM_Pattern.result(Time_Left_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_L += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[2].Pos_SideL_SwingMinus;
      else if (COM_Pattern.result(Time_Left_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_L += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[2].Pos_SideL;
      else if (COM_Pattern.result(Time_Left_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_L -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[2].Neg_SideL_SwingMinus;
      else if (COM_Pattern.result(Time_Left_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_L -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[2].Neg_SideL;

      Kinetic_Y_L += ((Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * Side_Pattern.result(Time_Left_Leg));
    }
    else if (Now_Param.X.X >= K_value[1].min && Now_Param.X.X < K_value[1].max)
    {
      // RIGHT-------------------------------------------------------------------------------------
      Kinetic_Y_R = -COM_Pattern.result(Time_Right_Leg) * Balance.Swing_Control_R * (Now_Param.Y.Swing_Leg_Right - (Cal.negative_position(Now_Param.Yaw_R.Yaw * K_value[1].Neg_YawR) - Cal.positive_position(Now_Param.Yaw_R.Yaw * K_value[1].Pos_YawR)) - (Cal.negative_position(Now_Param.X.X * K_value[1].Neg_XR) + Cal.positive_position(Now_Param.X.X * K_value[1].Pos_XR))) + Now_Param.Y.Default_Y_Right;

      if (-COM_Pattern.result(Time_Right_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_R -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[1].Neg_SideR_SwingMinus;
      else if (-COM_Pattern.result(Time_Right_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_R -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[1].Neg_SideR;
      else if (-COM_Pattern.result(Time_Right_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_R += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[1].Pos_SideR_SwingMinus;
      else if (-COM_Pattern.result(Time_Right_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_R += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[1].Pos_SideR;
      Kinetic_Y_R += ((Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * Side_Pattern.result(Time_Right_Leg));
      // LEFT--------------------------------------------------------------------------------------
      Kinetic_Y_L = COM_Pattern.result(Time_Left_Leg) * Balance.Swing_Control_L * (Now_Param.Y.Swing_Leg_Left - (Cal.negative_position(Now_Param.Yaw_L.Yaw * K_value[1].Neg_YawL) - Cal.positive_position(Now_Param.Yaw_L.Yaw * K_value[1].Pos_YawL)) - (Cal.negative_position(Now_Param.X.X * K_value[1].Neg_XL) + Cal.positive_position(Now_Param.X.X * K_value[1].Pos_XL))) + Now_Param.Y.Default_Y_Left;

      if (COM_Pattern.result(Time_Left_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_L += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[1].Pos_SideL_SwingMinus;
      else if (COM_Pattern.result(Time_Left_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_L += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[1].Pos_SideL;
      else if (COM_Pattern.result(Time_Left_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_L -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[1].Neg_SideL_SwingMinus;
      else if (COM_Pattern.result(Time_Left_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_L -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[1].Neg_SideL;

      Kinetic_Y_L += ((Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * Side_Pattern.result(Time_Left_Leg));
    }
    else if (Now_Param.X.X >= K_value[0].min && Now_Param.X.X < K_value[0].max)
    {
      // RIGHT-------------------------------------------------------------------------------------
      Kinetic_Y_R = -COM_Pattern.result(Time_Right_Leg) * Balance.Swing_Control_R * (Now_Param.Y.Swing_Leg_Right - (Cal.negative_position(Now_Param.Yaw_R.Yaw * K_value[0].Neg_YawR) - Cal.positive_position(Now_Param.Yaw_R.Yaw * K_value[0].Pos_YawR)) - (Cal.negative_position(Now_Param.X.X * K_value[0].Neg_XR) + Cal.positive_position(Now_Param.X.X * K_value[0].Pos_XR))) + Now_Param.Y.Default_Y_Right;

      if (-COM_Pattern.result(Time_Right_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_R -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[0].Neg_SideR_SwingMinus;
      else if (-COM_Pattern.result(Time_Right_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_R -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[0].Neg_SideR;
      else if (-COM_Pattern.result(Time_Right_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_R += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[0].Pos_SideR_SwingMinus;
      else if (-COM_Pattern.result(Time_Right_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_R += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[0].Pos_SideR;
      Kinetic_Y_R += ((Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * Side_Pattern.result(Time_Right_Leg));
      // LEFT--------------------------------------------------------------------------------------
      Kinetic_Y_L = COM_Pattern.result(Time_Left_Leg) * Balance.Swing_Control_L * (Now_Param.Y.Swing_Leg_Left - (Cal.negative_position(Now_Param.Yaw_L.Yaw * K_value[0].Neg_YawL) - Cal.positive_position(Now_Param.Yaw_L.Yaw * K_value[0].Pos_YawL)) - (Cal.negative_position(Now_Param.X.X * K_value[0].Neg_XL) + Cal.positive_position(Now_Param.X.X * K_value[0].Pos_XL))) + Now_Param.Y.Default_Y_Left;

      if (COM_Pattern.result(Time_Left_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_L += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[0].Pos_SideL_SwingMinus;
      else if (COM_Pattern.result(Time_Left_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) > 0)
        Kinetic_Y_L += (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[0].Pos_SideL;
      else if (COM_Pattern.result(Time_Left_Leg) <= 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_L -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[0].Neg_SideL_SwingMinus;
      else if (COM_Pattern.result(Time_Left_Leg) > 0 && (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) <= 0)
        Kinetic_Y_L -= (Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * K_value[0].Neg_SideL;

      Kinetic_Y_L += ((Now_Param.Y.Tuning_Side + Now_Param.Y.Side) * Side_Pattern.result(Time_Left_Leg));
    }

    if (Now_Param.Y.Side > 10.0 &&
        Time_Right_Leg >= 0.1 && Time_Right_Leg <= 0.5)
    {
      Kinetic_Y_L += Now_Param.Y.Side * 0.667;
    }
    else if (Now_Param.Y.Side < -10.0 &&
        Time_Left_Leg >= 0.1 && Time_Left_Leg <= 0.5)
    {
      Kinetic_Y_R -= abs(Now_Param.Y.Side * 0.667);
    }
    // master node debug line end ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // RIGHT-------------------------------------------------------------------------------------
    Kinetic_X_R = Step_Pattern.result(Time_Right_Leg) * (Now_Param.X.Tuning_X + Now_Param.X.X) + Now_Param.X.Default_X_Right;
    Kinetic_Z_R = Rise_Pattern.result(Time_Right_Leg) * (Now_Param.Z.Rise_Leg_Right) + Now_Param.Z.Default_Z_Right;

    // LEFT--------------------------------------------------------------------------------------
    Kinetic_X_L = Step_Pattern.result(Time_Left_Leg) * (Now_Param.X.Tuning_X + Now_Param.X.X) + Now_Param.X.Default_X_Left;
    Kinetic_Z_L = Rise_Pattern.result(Time_Left_Leg) * (Now_Param.Z.Rise_Leg_Left) + Now_Param.Z.Default_Z_Left;

    // Shoulder----------------------------------------------------------------------------------
    Kinetic_Shoulder_X_R = Step_Pattern.result(Time_Right_Leg) * (Now_Param.Shoulder.Swing_Shoulder_Right);
    Kinetic_Shoulder_X_L = Step_Pattern.result(Time_Left_Leg) * (Now_Param.Shoulder.Swing_Shoulder_Left);

    Amount_of_Change_Y = Kinetic_Y_R - Past_Kinetic_Y_R;
    Past_Kinetic_Y_R = Kinetic_Y_R;

    Amount_of_Change_X = Kinetic_X_R - Past_Kinetic_X_R;
    Past_Kinetic_X_R = Kinetic_X_R;

    // X or Y값만 사용할 경우 yaw값 유지 기능
    // first_yaw 기준 yaw 보정량 계산
    // x/y 이동 명령이 있고, 외부 yaw 명령은 없을 때만 yaw hold
    double move_cmd =
        fabs(Past_Param.X.X) +
        fabs(Past_Param.Y.Side) +
        fabs(Now_Param.X.X) +
        fabs(Now_Param.Y.Side);

    double external_yaw_cmd =
        fabs(Past_Param.Yaw_R.Yaw) +
        fabs(Past_Param.Yaw_L.Yaw);

    if (move_cmd > 0.01 && external_yaw_cmd < 0.01)
    {
      Adjust.desire_yaw = first_yaw;
      Adjust.Yaw_flag = true;

      double yaw_error = Adjust.desire_yaw - IMU.yaw;

      if (yaw_error >= 2.0)
      {
        Adjust.Yaw_gain = 5.0;
      }
      else if (yaw_error <= -3.0)
      {
        Adjust.Yaw_gain = -5.0;
      }
      else
      {
        Adjust.Yaw_gain = 0.0;
      }

      std::cout << "[YAW HOLD ON] "
                << "gain=" << Adjust.Yaw_gain
                << " target=" << Adjust.desire_yaw
                << " imu=" << IMU.yaw
                << " err=" << yaw_error
                << " move_cmd=" << move_cmd
                << " ext_yaw=" << external_yaw_cmd
                << std::endl;
    }
    else
    {
      Adjust.Yaw_flag = false;
      Adjust.Yaw_gain = 0.0;

      std::cout << "[YAW HOLD OFF] "
                << "move_cmd=" << move_cmd
                << " ext_yaw=" << external_yaw_cmd
                << " PastX=" << Past_Param.X.X
                << " PastY=" << Past_Param.Y.Side
                << " PastYawR=" << Past_Param.Yaw_R.Yaw
                << " PastYawL=" << Past_Param.Yaw_L.Yaw
                << " NowX=" << Now_Param.X.X
                << " NowY=" << Now_Param.Y.Side
                << " NowYawR=" << Now_Param.Yaw_R.Yaw
                << " NowYawL=" << Now_Param.Yaw_L.Yaw
                << std::endl;
    }

    // 마스터에서 공 보정시에 Side와 Yaw를 동시에 줄 때 공 주위로 돌도록(즉, Yaw가 내회전)
    if ((Now_Param.Y.Side != 0 && Now_Param.Yaw_R.Yaw != 0))
    {
      if ((Now_Param.Yaw_R.Tuning_Yaw + Now_Param.Yaw_R.Yaw) >= 0)
      {
        Kinetic_Yaw_R = +Turn_Pattern.result(Time_Right_Leg) * (Now_Param.Yaw_R.Tuning_Yaw + Now_Param.Yaw_R.Yaw);
        Kinetic_Yaw_L = -Turn_Pattern.result(Time_Right_Leg) * (Now_Param.Yaw_L.Tuning_Yaw + Now_Param.Yaw_L.Yaw);
      }
      else if ((Now_Param.Yaw_R.Tuning_Yaw + Now_Param.Yaw_R.Yaw) < 0)
      {
        Kinetic_Yaw_R = -Turn_Pattern.result(Time_Left_Leg) * (Now_Param.Yaw_R.Tuning_Yaw + Now_Param.Yaw_R.Yaw);
        Kinetic_Yaw_L = +Turn_Pattern.result(Time_Left_Leg) * (Now_Param.Yaw_L.Tuning_Yaw + Now_Param.Yaw_L.Yaw);
      }
    }
    else
    {
      if ((Now_Param.Yaw_R.Tuning_Yaw + Now_Param.Yaw_R.Yaw) >= 0)
      {
        Kinetic_Yaw_R = -Turn_Pattern.result(Time_Left_Leg) * (Now_Param.Yaw_R.Tuning_Yaw + Now_Param.Yaw_R.Yaw + Adjust.Yaw_gain);
        Kinetic_Yaw_L = +Turn_Pattern.result(Time_Left_Leg) * (Now_Param.Yaw_L.Tuning_Yaw + Now_Param.Yaw_L.Yaw + Adjust.Yaw_gain);
      }
      else if ((Now_Param.Yaw_R.Tuning_Yaw + Now_Param.Yaw_R.Yaw) < 0)
      {
        Kinetic_Yaw_R = +Turn_Pattern.result(Time_Right_Leg) * (Now_Param.Yaw_R.Tuning_Yaw + Now_Param.Yaw_R.Yaw + Adjust.Yaw_gain);
        Kinetic_Yaw_L = -Turn_Pattern.result(Time_Right_Leg) * (Now_Param.Yaw_L.Tuning_Yaw + Now_Param.Yaw_L.Yaw + Adjust.Yaw_gain);
      }
    }

    if (Time_Right_Leg <= 0.02)
    {
      IkCoord.x = (int)Now_Param.X.X;
      IkCoord.y = (int)Now_Param.Y.Side;
      Ikcoordinate_Pub->publish(IkCoord);
    }
  }
  if (Balance.Balance_Pitch_Flag)
  {
    if (ZMP.right_foot)
    {
      Zmp_pos.PID_Zmp_Pitch_Balancing(
          ZMP.right_y_zmp, Balance.Balance_Pitch_GP, Balance.Balance_Pitch_GI,
          Balance.Balance_Pitch_GD, Balance.Balance_Pitch_ELIMIT,
          Balance.Balance_Pitch_OLIMIT, Balance.Balance_Pitch_Neg_Target,
          Balance.Balance_Pitch_Pos_Target);
      Balance.Support_Con = 1;
    }
    if (ZMP.left_foot)
    {
      Zmp_pos.PID_Zmp_Pitch_Balancing(
          ZMP.left_y_zmp, Balance.Balance_Pitch_GP, Balance.Balance_Pitch_GI,
          Balance.Balance_Pitch_GD, Balance.Balance_Pitch_ELIMIT,
          Balance.Balance_Pitch_OLIMIT, Balance.Balance_Pitch_Neg_Target,
          Balance.Balance_Pitch_Pos_Target);
      Balance.Support_Con = -1;
    }
    if (ZMP.both_feet)
    {
      Zmp_pos.PID_Zmp_Pitch_Balancing(
          ZMP.total_y_zmp, Balance.Balance_Pitch_GP, Balance.Balance_Pitch_GI,
          Balance.Balance_Pitch_GD, Balance.Balance_Pitch_ELIMIT,
          Balance.Balance_Pitch_OLIMIT, Balance.Balance_Pitch_Neg_Target,
          Balance.Balance_Pitch_Pos_Target);
      Balance.Support_Con = 0;
    }
  }

  if (Balance.Balance_Roll_Flag)
  {
    if (ZMP.right_foot)
    {
      Zmp_pos.PID_Zmp_Roll_Balancing(
          ZMP.right_x_zmp, Balance.Balance_Roll_GP, Balance.Balance_Roll_GI,
          Balance.Balance_Roll_GD, Balance.Balance_Roll_ELIMIT,
          Balance.Balance_Roll_OLIMIT, Balance.Balance_Roll_Neg_Target,
          Balance.Balance_Roll_Pos_Target);

      Balance.Support_Con = 1;
    }
    else if (ZMP.left_foot)
    {
      Zmp_pos.PID_Zmp_Roll_Balancing(
          -ZMP.left_x_zmp, Balance.Balance_Roll_GP, Balance.Balance_Roll_GI,
          Balance.Balance_Roll_GD, Balance.Balance_Roll_ELIMIT,
          Balance.Balance_Roll_OLIMIT, Balance.Balance_Roll_Neg_Target,
          Balance.Balance_Roll_Pos_Target);

      Balance.Support_Con = -1;
    }
    else if (ZMP.both_feet)
    {
      Zmp_pos.PID_Zmp_Roll_Balancing(
          ZMP.total_x_zmp, Balance.Balance_Roll_GP, Balance.Balance_Roll_GI,
          Balance.Balance_Roll_GD, Balance.Balance_Roll_ELIMIT,
          Balance.Balance_Roll_OLIMIT, 0, 0);
      Balance.Support_Con = 0;
    }
  }
  if (!Balance.Balance_Ankle_Pitch_Flag)
  {
    Zmp_pos.Target_Pitch_Angle = 0;
  }
  if (!Balance.Balance_Pitch_Flag)
  {
    Zmp_pos.Target_X = 0;
  }
  if (!Balance.Balance_Roll_Flag)
  {
    Zmp_pos.Target_Roll_Angle = 0;
  }
  if (End_Flag)
  {
    Zmp_pos.Target_Pitch_Angle = 0;
    Zmp_pos.Target_X = 0;
    Zmp_pos.Target_Roll_Angle = 0;
  }
  if ((!Balance.Balance_Ankle_Pitch_Flag && !Balance.Balance_Pitch_Flag &&
       !Balance.Balance_Roll_Flag) ||
      (!ZMP.both_feet && !ZMP.right_foot && !ZMP.left_foot))
  {
    Balance.Support_Con = 99;
  }
  if (Balance.Balance_Pitch_Flag_imu)
  {
    Imu_Balance.PD_Pitch_control(
        IMU.pitch, Balance.Balance_Pitch_GP_imu, Balance.Balance_Pitch_GI_imu,
        Balance.Balance_Pitch_GD_imu, Balance.Balance_Pitch_ELIMIT_imu,
        Balance.Balance_Pitch_OLIMIT_imu, Balance.Balance_Pitch_Neg_Target_imu,
        Balance.Balance_Pitch_Pos_Target_imu);
    if (End_Flag)
    {
      Imu_Balance.Pitch_ADD_Angle = 0;
      Imu_Balance.Roll_ADD_Angle = 0;
    }
  }
  if (!Balance.Balance_Pitch_Flag_imu)
  {
    Imu_Balance.Pitch_ADD_Angle = 0;
  }
  if (Balance.Balance_Roll_Flag_imu)
  {
    Imu_Balance.PD_Roll_control(
        IMU.roll, Balance.Balance_Roll_GP_imu, Balance.Balance_Roll_GI_imu,
        Balance.Balance_Roll_GD_imu, Balance.Balance_Roll_ELIMIT_imu,
        Balance.Balance_Roll_OLIMIT_imu, Balance.Balance_Roll_Neg_Target_imu,
        Balance.Balance_Roll_Pos_Target_imu);
  }
  if (!Balance.Balance_Roll_Flag_imu)
  {
    Imu_Balance.Roll_ADD_Angle = 0;
  }

  if (Now_Param.IK_Flag)
  {
    Now_Param.Z.R_Rise_Condition = Rise_Pattern.result(Time_Right_Leg) * (Now_Param.Z.Rise_Leg_Right);
    Now_Param.Z.L_Rise_Condition = Rise_Pattern.result(Time_Left_Leg) * (Now_Param.Z.Rise_Leg_Left);

    IK.Balance_Control_Body_Upright(Zmp_pos.Target_X, Now_Param.Z.Default_Z_Right, Time_Right_Leg, Now_Param.Z.R_Rise_Condition, Zmp_pos.Target_Pitch_Angle, Balance.Balance_Value_0, Balance.Balance_Value_1, ZMP.left_y_zmp, ZMP.right_y_zmp, Zmp_pos.Target_Roll_Angle, Balance.Balance_Value_2, Balance.Balance_Value_3, Model_Data.Link2Link, ZMP.left_x_zmp, ZMP.right_x_zmp, Balance.Support_Con);

    //--------Landing_Time_Control(LTC)--------//
    if (Balance.Landing_Time_Control_flag)
    {
      if (Balance.Support_Con == 1 && Balance.Landing_flag_R)
      {
        Balance.Landing_Time_R = Timer_Time;
        Balance.Landing_Error_R = (Now_Param.Entire_Time / 2) - Balance.Landing_Time_R;
        Balance.Landing_flag_R = false;
        Balance.Landing_flag_L = true;
        Balance.Swing_Moment_Gain_L = 0.0;
        Balance.Swing_Moment_Gain_R = 0.5;
      }
      else if (Balance.Support_Con == -1 && Balance.Landing_flag_L)
      {
        Balance.Landing_Time_L = Timer_Time;
        Balance.Landing_Error_L = (Now_Param.Entire_Time - 1) - Balance.Landing_Time_L;
        Balance.Landing_flag_L = false;
        Balance.Landing_flag_R = true;
        Balance.Swing_Moment_Gain_L = 0.5;
        Balance.Swing_Moment_Gain_R = 0.0;
      }
      else if (Balance.Support_Con == 0)
      {
        Balance.Swing_Control_L = Balance.Swing_Control_add_L;
        Balance.Swing_Control_R = Balance.Swing_Control_add_R;
      }
      else if (Balance.Support_Con == 99)
      {
        Balance.Swing_Control_Warning_cnt = 0;
        Balance.Swing_Control_Safe_cnt = 0;
      }

      if (Balance.Landing_Error_L >= 17 && Balance.Landing_Error_L <= (Now_Param.Entire_Time / 2))
      {
        if (Balance.Support_Con != 0)
          Balance.Swing_Control_Warning_cnt++;
        IkLTC.warning = Balance.Swing_Control_Warning_cnt;
        Balance.Swing_Control_L = Balance.Swing_Control_add_L + Balance.Swing_Moment_Gain_L;
        if (Balance.Swing_Control_Warning_cnt >= 100)
        {
          Balance.Swing_Control_Warning_cnt = 0;
          Balance.Swing_Control_add_L += 0.05;
          Balance.Swing_Control_add_R += 0.05;
        }
      }
      else if (Balance.Landing_Error_R >= 17)
      {
        if (Balance.Support_Con != 0)
          Balance.Swing_Control_Warning_cnt++;
        IkLTC.warning = Balance.Swing_Control_Warning_cnt;
        Balance.Swing_Control_R = Balance.Swing_Control_add_R + Balance.Swing_Moment_Gain_R;
        if (Balance.Swing_Control_Warning_cnt >= 100)
        {
          Balance.Swing_Control_Warning_cnt = 0;
          Balance.Swing_Control_add_L += 0.05;
          Balance.Swing_Control_add_R += 0.05;
        }
      }
      else if (Balance.Landing_Error_L >= (Now_Param.Entire_Time - 10))
      {
        if (Balance.Support_Con != 0)
          Balance.Swing_Control_Safe_cnt++;
        IkLTC.safe = Balance.Swing_Control_Safe_cnt;
        Balance.Swing_Control_L = Balance.Swing_Control_add_L - Balance.Swing_Moment_Gain_L;
        if (Balance.Swing_Control_Safe_cnt >= 100)
        {
          Balance.Swing_Control_Safe_cnt = 0;
          Balance.Swing_Control_add_L -= 0.05;
          Balance.Swing_Control_add_R -= 0.05;
        }
      }
      else if (Balance.Landing_Error_R <= -10)
      {
        if (Balance.Support_Con != 0)
          Balance.Swing_Control_Safe_cnt++;
        IkLTC.safe = Balance.Swing_Control_Safe_cnt;
        Balance.Swing_Control_R = Balance.Swing_Control_add_R - Balance.Swing_Moment_Gain_R;
        if (Balance.Swing_Control_Safe_cnt >= 100)
        {
          Balance.Swing_Control_Safe_cnt = 0;
          Balance.Swing_Control_add_L -= 0.05;
          Balance.Swing_Control_add_R -= 0.05;
        }
      }
      IkLTC.swing_gain_l = Balance.Swing_Control_L;
      IkLTC.swing_gain_r = Balance.Swing_Control_R;
      IkLTC.entire_time = Now_Param.Entire_Time;
      IkLTC.landing_time_l = Balance.Landing_Time_L;
      IkLTC.landing_time_r = Balance.Landing_Time_R;
      IkLTC.landing_error_l = Balance.Landing_Error_L;
      IkLTC.landing_error_r = Balance.Landing_Error_R;
      Landing_Pub->publish(IkLTC);
    }
    else
    {
      Balance.Swing_Control_L = 1.0;
      Balance.Swing_Control_R = 1.0;
      Balance.Swing_Control_add_L = 1.0;
      Balance.Swing_Control_add_R = 1.0;
      Balance.Swing_Control_Safe_cnt = 0;
      Balance.Swing_Control_Warning_cnt = 0;
      IkLTC.swing_gain_l = Balance.Swing_Control_L;
      IkLTC.swing_gain_r = Balance.Swing_Control_R;
      IkLTC.landing_time_l = 0;
      IkLTC.landing_time_r = 0;
      IkLTC.landing_error_l = 0;
      IkLTC.landing_error_r = 0;
      Landing_Pub->publish(IkLTC);
    }

    IkPattern.xrpattern = Step_Pattern.result(Time_Right_Leg);
    IkPattern.yrpattern = -COM_Pattern.result(Time_Right_Leg);
    IkPattern.zrpattern = Rise_Pattern.result(Time_Right_Leg);
    IkPattern.trpattern = Turn_Pattern.result(Time_Right_Leg);

    IkPattern.xlpattern = Step_Pattern.result(Time_Left_Leg);
    IkPattern.ylpattern = -COM_Pattern.result(Time_Left_Leg);
    IkPattern.zlpattern = Rise_Pattern.result(Time_Left_Leg);
    IkPattern.tlpattern = Turn_Pattern.result(Time_Left_Leg);

    walk_pattern_Pub->publish(IkPattern);

    // === 허리(골반) 회전: 스윙(나가는) 발 방향으로 허리를 돌려 보행 틀어짐 보완 ===
    // 각 발의 전후 변위(기본 위치 대비)를 구해 그 차이에 비례해 허리를 회전시킨다.
    // 왼발이 앞으로 나갈 때(+) 허리를 한쪽으로, 오른발이 나갈 때(-) 반대로 돌리며,
    // 보폭(Tuning_X+X)이 클수록 진폭이 커지고 제자리/정지(End) 시 0으로 수렴한다.
    double Left_Step_Fwd  = Kinetic_X_L - Now_Param.X.Default_X_Left;
    double Right_Step_Fwd = Kinetic_X_R - Now_Param.X.Default_X_Right;
    double Kinetic_Waist_Yaw = Waist_Swing_Gain * (Left_Step_Fwd - Right_Step_Fwd);
    if (Kinetic_Waist_Yaw >  Waist_Yaw_Limit) Kinetic_Waist_Yaw =  Waist_Yaw_Limit;
    if (Kinetic_Waist_Yaw < -Waist_Yaw_Limit) Kinetic_Waist_Yaw = -Waist_Yaw_Limit;

    IK.solve(Kinetic_X_R, Kinetic_Y_R, Kinetic_Z_R, Kinetic_Yaw_R, Kinetic_X_L, Kinetic_Y_L, Kinetic_Z_L, Kinetic_Yaw_L, Leg, Kinetic_Shoulder_X_R, Kinetic_Shoulder_X_L, Kinetic_Shoulder_Y_R, Kinetic_Shoulder_Y_L, IK.Now_Balance_Theta.Theta1, IK.Now_Balance_Theta.Theta2, IK.Now_Balance_Theta.Theta3, Now_Param.Z.R_Rise_Condition, Now_Param.Z.L_Rise_Condition, Now_Param.Z.Rise_Leg_Right, Now_Param.Z.Rise_Leg_Left, IMU.pitch, Balance.Balance_Pitch_Pos_Target_imu, Balance.Balance_Pitch_Neg_Target_imu, IMU.roll, Now_Param.X.X, Kinetic_Waist_Yaw);
    // cout << "===============IKflag_true====================" << endl;
    // cout<< "timer : "<<Time_Right_Leg<<endl;
    // cout<<" end_timer : "<<Time_Right_Leg_End<<endl;

    // cout << "Now_Param.X.X >> " << Now_Param.X.X << endl;
    // cout << "Now_Param.Y.Side >> " << Now_Param.Y.Side << endl;
    // cout << "Now_Param.Z.YawR >> " << Now_Param.Yaw_R.Yaw << endl;
    // cout << "Past_Param.X.X >> " << Past_Param.X.X << endl;
    // cout << "Past_Param.Y.Side >> " << Past_Param.Y.Side << endl;
    // cout << "Past_Param.Z.YawR >> " << Past_Param.Yaw_R.Yaw << endl;
  }
  Ik_Flag_Past = Now_Param.IK_Flag;
}

void IKwalk::master2ik_callback(const humanoid_interfaces::msg::Master2IkMsg::SharedPtr msg)
{
  Past_Param.IK_Flag = msg->flag;
  //-----------X_Accel-----------//
  if (fabs(msg->x_length - Acc.check_old_x) >= 10) // 기존 x 값과 수신 받은 x 값의 차이가 10 보다 큰 상황 일때 >> 유의미하게 큰 X 값을 받을 떄
  {
    Acc.check_old_x = msg->x_length;
    Acc.basic_x = Now_Param.X.X;
    if (msg->x_length - Now_Param.X.X >= 0) // 메세지 x 값이 더 클때 >> 전진
    {
      Acc.accel_pos_x = msg->x_length - Now_Param.X.X;
      Acc.accel_pos_x_cnt = 0;
      Acc.accel_neg_x_cnt = -1;
      acc_flag = true;
    }
    else // 후진 일때
    {
      Acc.accel_neg_x = msg->x_length - Now_Param.X.X;
      Acc.accel_pos_x_cnt = -1;
      Acc.accel_neg_x_cnt = 0;
      acc_flag =false;
    }
    Past_Param.X.X = msg->x_length;
  }
  else // 기존 x 값과 수신 받은 x 값의 차이가 10 이내 일때 >> X 값의 크기가 유의미하지 않을때
  {
    Acc.check_old_x = msg->x_length;
    Past_Param.X.X = msg->x_length;
  }
  //-----------Y_Accel-----------//
  if (fabs(msg->y_length - Acc.check_old_y) >= 5) // 유의미하게 큰 Y 값을 받을때 기준 값 크기 5
  {
    Acc.check_old_y = msg->y_length;
    Acc.basic_y = Now_Param.Y.Side;

    Acc.accel_pos_y = msg->y_length - Now_Param.Y.Side;

    if (msg->y_length - Now_Param.Y.Side >= 0) // 좌측으로 이동
    {
      Acc.accel_pos_y = msg->y_length - Now_Param.Y.Side;
      Acc.accel_pos_y_cnt = 0;
      Acc.accel_neg_y_cnt = -1;
    }
    else // 우측으로 이동
    {
      Acc.accel_neg_y = msg->y_length - Now_Param.Y.Side;
      Acc.accel_pos_y_cnt = -1;
      Acc.accel_neg_y_cnt = 0;
    }
    Past_Param.Y.Side = msg->y_length;


  }
  else // 유의미하지 않은 Y 값을 수신 받음
  {

    Acc.check_old_y = msg->y_length;
    Past_Param.Y.Side = msg->y_length;
  }
  if(fabs(msg->y_length) >=10) Entire_time_flag = 1;
  else if(fabs(msg->y_length) <10) Entire_time_flag = 0;
  
  
  //---------Z(Yaw)_Accel---------//
  if (fabs(msg->yaw - Acc.check_old_z) >= 3) // z 값 : 제자리 회전 값이 3 보다 크다 >> 유의미 하게 큰 YAW 값을 수신 받음
  {
    Acc.check_old_z = msg->yaw;
    Acc.basic_z = Now_Param.Yaw_R.Yaw;
    Acc.accel_pos_z = msg->yaw - Now_Param.Yaw_R.Yaw;

    if (msg->yaw - Now_Param.Yaw_R.Yaw >= 0) // 좌측으로의 회전
    {
      Acc.accel_pos_z = msg->yaw - Now_Param.Yaw_R.Yaw;
      Acc.accel_pos_z_cnt = 0;
      Acc.accel_neg_z_cnt = -1;
    }
    else
    {
      Acc.accel_neg_z = msg->yaw - Now_Param.Yaw_R.Yaw;
      Acc.accel_pos_z_cnt = -1;
      Acc.accel_neg_z_cnt = 0;
    }
    Past_Param.Yaw_R.Yaw = msg->yaw;

    if (msg->yaw - Now_Param.Yaw_L.Yaw >= 0)
    {
      Acc.accel_pos_z = msg->yaw - Now_Param.Yaw_L.Yaw;
      Acc.accel_pos_z_cnt = 0;
      Acc.accel_neg_z_cnt = -1;
    }
    else
    {
      Acc.accel_neg_z = msg->yaw - Now_Param.Yaw_L.Yaw;
      Acc.accel_pos_z_cnt = -1;
      Acc.accel_neg_z_cnt = 0;
    }
    Past_Param.Yaw_L.Yaw = msg->yaw;
  }
  else
  {
    Acc.check_old_z = msg->yaw;
    Past_Param.Yaw_R.Yaw = msg->yaw;
    Past_Param.Yaw_L.Yaw = msg->yaw;
  }

  if (Past_Param.X.X >= X_LIMIT)
    Past_Param.X.X = X_LIMIT;
  else if (Past_Param.X.X <= -X_LIMIT)
    Past_Param.X.X = -X_LIMIT;

  if (Past_Param.Y.Side >= Y_LIMIT)
    Past_Param.Y.Side = Y_LIMIT;
  else if (Past_Param.Y.Side <= -Y_LIMIT)
    Past_Param.Y.Side = -Y_LIMIT;

  if (Past_Param.Yaw_R.Yaw >= YAW_LIMIT)
    Past_Param.Yaw_R.Yaw = YAW_LIMIT;
  else if (Past_Param.Yaw_R.Yaw <= -YAW_LIMIT)
    Past_Param.Yaw_R.Yaw = -YAW_LIMIT;

  if (Past_Param.Yaw_L.Yaw >= YAW_LIMIT)
    Past_Param.Yaw_L.Yaw = YAW_LIMIT;
  else if (Past_Param.Yaw_L.Yaw <= -YAW_LIMIT)
    Past_Param.Yaw_L.Yaw = -YAW_LIMIT;
}

void IKwalk::imu_callback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg)
{
  IMU.pitch = msg->pitch;//Cal.MAF(msg->pitch);
  IMU.roll = msg->roll;
  IMU.yaw = msg->yaw;
  if (first_count == 0)
  {
    first_yaw = IMU.yaw;
    first_count++;
  }
}

void IKwalk::tune2ik_callback(const humanoid_interfaces::msg::Tune2IkMsg::SharedPtr msg)
{
  Past_Param.IK_Flag = msg->ik_flag;
  //Past_Param.Entire_Time = msg->entire_time;
  Past_Param.Frequency = msg->frequency;
  Balance.Ratio_Check_Flag = msg->ratio_check_flag;

  //-----------X_Accel-----------//
  if (fabs(msg->test_x - Acc.check_old_x) >= 10)
  {
    Acc.check_old_x = msg->test_x;
    Acc.basic_x = Now_Param.X.X;
    if (msg->test_x - Now_Param.X.X >= 0)
    {
      Acc.accel_pos_x = msg->test_x - Now_Param.X.X;
      Acc.accel_pos_x_cnt = 0;
      Acc.accel_neg_x_cnt = -1;
      acc_flag = true;
    }
    else
    {
      Acc.accel_neg_x = msg->test_x - Now_Param.X.X;
      Acc.accel_pos_x_cnt = -1;
      Acc.accel_neg_x_cnt = 0;
      acc_flag = false;
    }
    Past_Param.X.X = msg->test_x;
  }
  else
  {
    Acc.check_old_x = msg->test_x;
    Past_Param.X.X = msg->test_x;
  }
  //-----------Y_Accel-----------//
  if (fabs(msg->test_side - Acc.check_old_y) >= 5)
  {
    Acc.check_old_y = msg->test_side;
    Acc.basic_y = Now_Param.Y.Side;
    if (msg->test_side - Now_Param.Y.Side >= 0)
    {
      Acc.accel_pos_y = msg->test_side - Now_Param.Y.Side;
      Acc.accel_pos_y_cnt = 0;
      Acc.accel_neg_y_cnt = -1;
    }
    else
    {
      Acc.accel_neg_y = msg->test_side - Now_Param.Y.Side;
      Acc.accel_pos_y_cnt = -1;
      Acc.accel_neg_y_cnt = 0;
    }
    Past_Param.Y.Side = msg->test_side;
  }
  else
  {
    Acc.check_old_y = msg->test_side;
    Past_Param.Y.Side = msg->test_side;
  }
    if(fabs(msg->test_side) >=10) Entire_time_flag = 1;
  else if(fabs(msg->test_side) <10) Entire_time_flag = 0;
  //--------Z(Yaw)_Accel---------//
  if (fabs(msg->test_yaw - Acc.check_old_z) >= 3)
  {
    Acc.check_old_z = msg->test_yaw;
    Acc.basic_z = Now_Param.Yaw_R.Yaw;
    if (msg->test_yaw - Now_Param.Yaw_R.Yaw >= 0)
    {
      Acc.accel_pos_z = msg->test_yaw - Now_Param.Yaw_R.Yaw;
      Acc.accel_pos_z_cnt = 0;
      Acc.accel_neg_z_cnt = -1;
    }
    else
    {
      Acc.accel_neg_z = msg->test_yaw - Now_Param.Yaw_R.Yaw;
      Acc.accel_pos_z_cnt = -1;
      Acc.accel_neg_z_cnt = 0;
    }
    Past_Param.Yaw_R.Yaw = msg->test_yaw;

    if (msg->test_yaw - Now_Param.Yaw_L.Yaw >= 0)
    {
      Acc.accel_pos_z = msg->test_yaw - Now_Param.Yaw_L.Yaw;
      Acc.accel_pos_z_cnt = 0;
      Acc.accel_neg_z_cnt = -1;
    }
    else
    {
      Acc.accel_neg_z = msg->test_yaw - Now_Param.Yaw_L.Yaw;
      Acc.accel_pos_z_cnt = -1;
      Acc.accel_neg_z_cnt = 0;
    }
    Past_Param.Yaw_L.Yaw = msg->test_yaw;
  }
  else
  {
    Acc.check_old_z = msg->test_yaw;
    Past_Param.Yaw_R.Yaw = msg->test_yaw;
    Past_Param.Yaw_L.Yaw = msg->test_yaw;
  }

  Past_Param.X.Tuning_X = msg->tuning_x;
  Past_Param.X.Default_X_Right = msg->default_x_right;
  Past_Param.X.Default_X_Left = msg->default_x_left;

  Past_Param.Y.Tuning_Side = msg->tuning_side;
  Past_Param.Y.Default_Y_Right = msg->default_y_right;
  Past_Param.Y.Default_Y_Left = msg->default_y_left;
  Past_Param.Y.Swing_Leg_Right = msg->swing_leg_right;
  Past_Param.Y.Swing_Leg_Left = msg->swing_leg_left;

  Past_Param.Z.Default_Z_Right = msg->default_z_right;
  Past_Param.Z.Default_Z_Left = msg->default_z_left;
  Past_Param.Z.Rise_Leg_Right = msg->rise_leg_right;
  Past_Param.Z.Rise_Leg_Left = msg->rise_leg_left;
  Past_Param.Shoulder.Swing_Shoulder_Right = msg->swing_shoulder_right;
  Past_Param.Shoulder.Swing_Shoulder_Left = msg->swing_shoulder_left;

  Past_Param.Yaw_R.Tuning_Yaw = msg->tuning_yaw;
  Past_Param.Yaw_L.Tuning_Yaw = msg->tuning_yaw;

  IK.Past_Motor_Angle.Motor_Angle_10 = msg->offset_motor[10];
  IK.Past_Motor_Angle.Motor_Angle_11 = msg->offset_motor[11];
  IK.Past_Motor_Angle.Motor_Angle_12 = msg->offset_motor[12];
  IK.Past_Motor_Angle.Motor_Angle_13 = msg->offset_motor[13];
  IK.Past_Motor_Angle.Motor_Angle_14 = msg->offset_motor[14];
  IK.Past_Motor_Angle.Motor_Angle_15 = msg->offset_motor[15];
  IK.Past_Motor_Angle.Motor_Angle_16 = msg->offset_motor[16];
  IK.Past_Motor_Angle.Motor_Angle_17 = msg->offset_motor[17];
  IK.Past_Motor_Angle.Motor_Angle_18 = msg->offset_motor[18];
  IK.Past_Motor_Angle.Motor_Angle_19 = msg->offset_motor[19];
  IK.Past_Motor_Angle.Motor_Angle_20 = msg->offset_motor[20];
  IK.Past_Motor_Angle.Motor_Angle_21 = msg->offset_motor[21];

  IK.Now_Motor_Angle = IK.Past_Motor_Angle;

  Past_Param.Start_Entire_Time = msg->start_entire_time;

  Past_Param.Y.Start_Swing = msg->start_swing;
  Past_Param.Z.Start_Rise = msg->start_rise;
  Now_Param.End_Entire_Time = msg->end_entire_time;
  Now_Param.Y.End_Swing = msg->end_swing;
  Now_Param.Z.End_Rise = msg->end_rise;
  Balance.Balance_Value_0 = msg->balance_value_0;
  Balance.Balance_Value_1 = msg->balance_value_1;
  Balance.Balance_Value_2 = msg->balance_value_2;
  Balance.Balance_Value_3 = msg->balance_value_3;
  Balance.Balance_Pitch_GP = msg->balance_pitch_gp;
  Balance.Balance_Pitch_GI = msg->balance_pitch_gi;
  Balance.Balance_Pitch_GD = msg->balance_pitch_gd;
  Balance.Balance_Pitch_ELIMIT = msg->balance_pitch_elimit;
  Balance.Balance_Pitch_OLIMIT = msg->balance_pitch_olimit;
  Balance.Balance_Pitch_Neg_Target = msg->balance_pitch_neg_target;
  Balance.Balance_Pitch_Pos_Target = msg->balance_pitch_pos_target;

  Balance.Balance_Angle_Pitch_GP = msg->balance_angle_pitch_gp;
  Balance.Balance_Angle_Pitch_GI = msg->balance_angle_pitch_gi;
  Balance.Balance_Angle_Pitch_GD = msg->balance_angle_pitch_gd;
  Balance.Balance_Angle_Pitch_ELIMIT = msg->balance_angle_pitch_elimit;
  Balance.Balance_Angle_Pitch_OLIMIT = msg->balance_angle_pitch_olimit;
  Balance.Balance_Angle_Pitch_Neg_Target = msg->balance_angle_pitch_neg_target;
  Balance.Balance_Angle_Pitch_Pos_Target = msg->balance_angle_pitch_pos_target;
  Balance.Balance_Roll_GP = msg->balance_roll_gp;
  Balance.Balance_Roll_GI = msg->balance_roll_gi;
  Balance.Balance_Roll_GD = msg->balance_roll_gd;
  Balance.Balance_Roll_ELIMIT = msg->balance_roll_elimit;
  Balance.Balance_Roll_OLIMIT = msg->balance_roll_olimit;
  Balance.Balance_Roll_Neg_Target = msg->balance_roll_neg_target;
  Balance.Balance_Roll_Pos_Target = msg->balance_roll_pos_target;

  Balance.Balance_Pitch_GP_imu = msg->balance_pitch_gp_imu;
  Balance.Balance_Pitch_GI_imu = msg->balance_pitch_gi_imu;
  Balance.Balance_Pitch_GD_imu = msg->balance_pitch_gd_imu;
  Balance.Balance_Pitch_ELIMIT_imu = msg->balance_pitch_elimit_imu;
  Balance.Balance_Pitch_OLIMIT_imu = msg->balance_pitch_olimit_imu;
  Balance.Balance_Pitch_Neg_Target_imu = msg->balance_pitch_neg_target_imu;
  Balance.Balance_Pitch_Pos_Target_imu = msg->balance_pitch_pos_target_imu;

  Balance.Balance_Roll_GP_imu = msg->balance_roll_gp_imu;
  Balance.Balance_Roll_GI_imu = msg->balance_roll_gi_imu;
  Balance.Balance_Roll_GD_imu = msg->balance_roll_gd_imu;
  Balance.Balance_Roll_ELIMIT_imu = msg->balance_roll_elimit_imu;
  Balance.Balance_Roll_OLIMIT_imu = msg->balance_roll_olimit_imu;
  Balance.Balance_Roll_Neg_Target_imu = msg->balance_roll_neg_target_imu;
  Balance.Balance_Roll_Pos_Target_imu = msg->balance_roll_pos_target_imu;

  Balance.Balance_Pitch_Flag = msg->balance_pitch_flag;
  Balance.Balance_Ankle_Pitch_Flag = msg->balance_ankle_pitch_flag;
  Balance.Balance_Roll_Flag = msg->balance_roll_flag;
  Balance.Balance_Pitch_Flag_imu = msg->balance_pitch_flag_imu;
  Balance.Balance_Roll_Flag_imu = msg->balance_roll_flag_imu;

  IK.Past_Percentage_of_IK_Motor.Motor_Multiple_10 = msg->percentage_of_ik_motor[10];
  IK.Past_Percentage_of_IK_Motor.Motor_Multiple_11 = msg->percentage_of_ik_motor[11];
  IK.Past_Percentage_of_IK_Motor.Motor_Multiple_12 = msg->percentage_of_ik_motor[12];
  IK.Past_Percentage_of_IK_Motor.Motor_Multiple_13 = msg->percentage_of_ik_motor[13];
  IK.Past_Percentage_of_IK_Motor.Motor_Multiple_14 = msg->percentage_of_ik_motor[14];
  IK.Past_Percentage_of_IK_Motor.Motor_Multiple_15 = msg->percentage_of_ik_motor[15];
  IK.Past_Percentage_of_IK_Motor.Motor_Multiple_16 = msg->percentage_of_ik_motor[16];
  IK.Past_Percentage_of_IK_Motor.Motor_Multiple_17 = msg->percentage_of_ik_motor[17];
  IK.Past_Percentage_of_IK_Motor.Motor_Multiple_18 = msg->percentage_of_ik_motor[18];
  IK.Past_Percentage_of_IK_Motor.Motor_Multiple_19 = msg->percentage_of_ik_motor[19];
  IK.Past_Percentage_of_IK_Motor.Motor_Multiple_20 = msg->percentage_of_ik_motor[20];
  IK.Past_Percentage_of_IK_Motor.Motor_Multiple_21 = msg->percentage_of_ik_motor[21];

  IK.Now_Percentage_of_IK_Motor = IK.Past_Percentage_of_IK_Motor;

  //-------------------Kinetic_value-------------------//
  // tune2ik msg call back

  // 18
  K_value[0].Pos_XR = msg->first_pos_xr;
  K_value[0].Neg_XR = msg->first_neg_xr;
  K_value[0].Pos_SideR = msg->first_pos_side_r;
  K_value[0].Neg_SideR = msg->first_neg_side_r;
  K_value[0].Pos_YawR = msg->first_pos_yaw_r;
  K_value[0].Neg_YawR = msg->first_neg_yaw_r;
  K_value[0].Pos_XL = msg->first_pos_xl;
  K_value[0].Neg_XL = msg->first_neg_xl;
  K_value[0].Pos_SideL = msg->first_pos_side_l;
  K_value[0].Neg_SideL = msg->first_neg_side_l;
  K_value[0].Pos_YawL = msg->first_pos_yaw_l;
  K_value[0].Neg_YawL = msg->first_neg_yaw_l;
  K_value[0].Pos_SideR_SwingMinus = msg->first_pos_side_r_swing_minus;
  K_value[0].Neg_SideR_SwingMinus = msg->first_neg_side_r_swing_minus;
  K_value[0].Pos_SideL_SwingMinus = msg->first_pos_side_l_swing_minus;
  K_value[0].Neg_SideL_SwingMinus = msg->first_neg_side_l_swing_minus;
  K_value[0].min = msg->first_min;
  K_value[0].max = msg->first_max;

  // 18
  K_value[1].Pos_XR = msg->second_pos_xr;
  K_value[1].Neg_XR = msg->second_neg_xr;
  K_value[1].Pos_SideR = msg->second_pos_side_r;
  K_value[1].Neg_SideR = msg->second_neg_side_r;
  K_value[1].Pos_YawR = msg->second_pos_yaw_r;
  K_value[1].Neg_YawR = msg->second_neg_yaw_r;
  K_value[1].Pos_XL = msg->second_pos_xl;
  K_value[1].Neg_XL = msg->second_neg_xl;
  K_value[1].Pos_SideL = msg->second_pos_side_l;
  K_value[1].Neg_SideL = msg->second_neg_side_l;
  K_value[1].Pos_YawL = msg->second_pos_yaw_l;
  K_value[1].Neg_YawL = msg->second_neg_yaw_l;
  K_value[1].Pos_SideR_SwingMinus = msg->second_pos_side_r_swing_minus;
  K_value[1].Neg_SideR_SwingMinus = msg->second_neg_side_r_swing_minus;
  K_value[1].Pos_SideL_SwingMinus = msg->second_pos_side_l_swing_minus;
  K_value[1].Neg_SideL_SwingMinus = msg->second_neg_side_l_swing_minus;
  K_value[1].min = msg->second_min;
  K_value[1].max = msg->second_max;

  // 18
  K_value[2].Pos_XR = msg->third_pos_xr;
  K_value[2].Neg_XR = msg->third_neg_xr;
  K_value[2].Pos_SideR = msg->third_pos_side_r;
  K_value[2].Neg_SideR = msg->third_neg_side_r;
  K_value[2].Pos_YawR = msg->third_pos_yaw_r;
  K_value[2].Neg_YawR = msg->third_neg_yaw_r;
  K_value[2].Pos_XL = msg->third_pos_xl;
  K_value[2].Neg_XL = msg->third_neg_xl;
  K_value[2].Pos_SideL = msg->third_pos_side_l;
  K_value[2].Neg_SideL = msg->third_neg_side_l;
  K_value[2].Pos_YawL = msg->third_pos_yaw_l;
  K_value[2].Neg_YawL = msg->third_neg_yaw_l;
  K_value[2].Pos_SideR_SwingMinus = msg->third_pos_side_r_swing_minus;
  K_value[2].Neg_SideR_SwingMinus = msg->third_neg_side_r_swing_minus;
  K_value[2].Pos_SideL_SwingMinus = msg->third_pos_side_l_swing_minus;
  K_value[2].Neg_SideL_SwingMinus = msg->third_neg_side_l_swing_minus;
  K_value[2].min = msg->third_min;
  K_value[2].max = msg->third_max;

  // 18개
  K_value[3].Pos_XR = msg->fourth_pos_xr;
  K_value[3].Neg_XR = msg->fourth_neg_xr;
  K_value[3].Pos_SideR = msg->fourth_pos_side_r;
  K_value[3].Neg_SideR = msg->fourth_neg_side_r;
  K_value[3].Pos_YawR = msg->fourth_pos_yaw_r;
  K_value[3].Neg_YawR = msg->fourth_neg_yaw_r;
  K_value[3].Pos_XL = msg->fourth_pos_xl;
  K_value[3].Neg_XL = msg->fourth_neg_xl;
  K_value[3].Pos_SideL = msg->fourth_pos_side_l;
  K_value[3].Neg_SideL = msg->fourth_neg_side_l;
  K_value[3].Pos_YawL = msg->fourth_pos_yaw_l;
  K_value[3].Neg_YawL = msg->fourth_neg_yaw_l;
  K_value[3].Pos_SideR_SwingMinus = msg->fourth_pos_side_r_swing_minus;
  K_value[3].Neg_SideR_SwingMinus = msg->fourth_neg_side_r_swing_minus;
  K_value[3].Pos_SideL_SwingMinus = msg->fourth_pos_side_l_swing_minus;
  K_value[3].Neg_SideL_SwingMinus = msg->fourth_neg_side_l_swing_minus;
  K_value[3].min = msg->fourth_min;
  K_value[3].max = msg->fourth_max;

  Balance.Landing_Time_Control_flag = msg->landing_time_control_flag;

  Balance.Balance_Pitch_Flag_imu = msg->balance_pitch_flag_imu;
  Balance.Balance_Roll_Flag_imu = msg->balance_roll_flag_imu;
  Balance.Balance_Pitch_Flag = msg->balance_pitch_flag;
  Balance.Balance_Ankle_Pitch_Flag = msg->balance_ankle_pitch_flag;
  Balance.Balance_Roll_Flag = msg->balance_roll_flag;
}

double IKwalk::Step_Acc::step_acc_func(double basic, double acc, int num, int &cnt)
{
  if (num == cnt)
  {
    cnt = -1;
    return basic + acc;
  }
  else
  {
    return basic + (acc / num) * cnt;
  }
}
double IKwalk::Calc::positive_position(double x)
{
  if (x >= 0.0)
    return x;
  else
    return 0;
}
double IKwalk::Calc::negative_position(double x)
{
  if (x <= 0.0)
    return x;
  else
    return 0;
}
double IKwalk::Calc::MAF(double x)
{
  double sum = 0, average;
  for (unsigned char i = 0; i < FILTERDATA - 1; i++)
    filter_data[i] = filter_data[i + 1];

  filter_data[FILTERDATA - 1] = x;

  for (unsigned char i = 0; i < FILTERDATA; i++)
    sum += filter_data[i];

  average = sum / FILTERDATA;
  // sort(filter_data,filter_data+FILTERDATA);//
  return /*data[4];*/ average;
}
void IKwalk::kick_flag_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if(msg->data != "left" && msg->data != "right") return;
  if(kick_pending_) return;


  if(kick_interval&&( ik_1 || ik_2 || ik_3)){
    kick_pending_ = true;
    kick_flag_msg_pending = msg->data;
  }
  else{
    kick_flag_ = true;
    kick_flag_msg_ = msg->data;
  }
  
}
