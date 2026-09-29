#ifndef IK_WALK_HPP
#define IK_WALK_HPP

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <cmath>
#include <memory>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <string>
#include <utility>
#include <functional>
#include <mutex>

#include "imu_position_control.hpp"
#include "pid_control_float.hpp"
#include "solve_kinematics.hpp"
#include "walk_pattern.hpp"
#include "kick_module.hpp"
#include "zmp_position_control.hpp"

// MSG_HEADER //
#include "irc_humanoid_interfaces/msg/ik_com_msg.hpp"
#include "irc_humanoid_interfaces/msg/ik_coord_msg.hpp"
#include "irc_humanoid_interfaces/msg/ik_end_msg.hpp"
#include "irc_humanoid_interfaces/msg/ik_ltc_msg.hpp"
#include "irc_humanoid_interfaces/msg/ik_pattern_msg.hpp"
#include "irc_humanoid_interfaces/msg/motion_operator.hpp"

#include "irc_humanoid_interfaces/msg/imu_msg.hpp"
#include "irc_humanoid_interfaces/msg/master2_ik_msg.hpp"
#include "irc_humanoid_interfaces/msg/tune2_ik_msg.hpp"
#include "irc_humanoid_interfaces/msg/zmp_msg.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#define X_LIMIT 50
#define Y_LIMIT 50
#define YAW_LIMIT 50
#define Init_Position_Time -1
#define Init_Position_Pitch 0
#define Init_Position_Balance_Msg 0
#define Init_IK_End 1
#define Init_Rise_Condition 0
#define FILTERDATA 10
#define EXP 2.7182818284590452354

static double Timer_Time = 0.0;
static double Timer_Time_Start = 0.0;
static double Timer_Time_End = 0.0;

using namespace std::chrono_literals;



class IKwalk : public rclcpp::Node
{
public:
  IK_Solve IK;
  IK_zmp_pos_control Zmp_pos;
  int imu_count=0;
  int first_yaw=0;
  int first_count=0;

  explicit IKwalk(const std::string &node_name)
      : Node(node_name)
  {
    this->declare_parameter("qos_depth", 10);
    rclcpp::QoS qos_profile(10);
    int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
    Motor_Pub = this->create_publisher<dynamixel_hardware_msgs::msg::DynamixelControlMsgs>("dynamixel_control", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort());
    Ikend_Pub = this->create_publisher<irc_humanoid_interfaces::msg::IkEndMsg>("ikend", 10);                   // QOS_RKL10V);
    Ikcoordinate_Pub = this->create_publisher<irc_humanoid_interfaces::msg::IkCoordMsg>("ikcoordinate", 10);   // QOS_RKL10V);
    walk_pattern_Pub = this->create_publisher<irc_humanoid_interfaces::msg::IkPatternMsg>("walk_pattern", 10); // QOS_RKL10V);
    COM_Pub = this->create_publisher<irc_humanoid_interfaces::msg::IkComMsg>("COM", 10);                       // QOS_RKL10V);
    Landing_Pub = this->create_publisher<irc_humanoid_interfaces::msg::IkLTCMsg>("Landing_Time_Control", 10);  // QOS_RKL10V);

    Imu_Sub = this->create_subscription<irc_humanoid_interfaces::msg::ImuMsg>(
        "Imu",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort(),
        std::bind(&IKwalk::imu_callback, this, std::placeholders::_1));
    Master2ik_Sub = this->create_subscription<irc_humanoid_interfaces::msg::Master2IkMsg>(
        "master2ik",
        qos_profile,
        std::bind(&IKwalk::master2ik_callback, this, std::placeholders::_1));
    Tune2ik_Sub = this->create_subscription<irc_humanoid_interfaces::msg::Tune2IkMsg>(
        "tune2walk",
        qos_profile,
        std::bind(&IKwalk::tune2ik_callback, this, std::placeholders::_1));
    // Zmp_Sub = this->create_subscription<irc_humanoid_interfaces::msg::ZmpMsg>(
    //     "zmp",
    //     qos_profile,
    //     std::bind(&IKwalk::zmp_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(10ms, std::bind(&IKwalk::timer_callback, this));
    kick_flag_sub_ = create_subscription<std_msgs::msg::String>(
        "kick_flag", 10,
        std::bind(&IKwalk::kick_flag_callback, this, std::placeholders::_1));
    kick_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("kick_flag_end", 10);

    const auto motion_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    motion_start_sub_ = create_subscription<irc_humanoid_interfaces::msg::MotionOperator>(
        "motion_operator", motion_qos,
        [this](const irc_humanoid_interfaces::msg::MotionOperator::SharedPtr msg) {
          active_motion_num_ = msg->motion_num;
          IK.set_motor_publish_enabled(false);
          RCLCPP_INFO(get_logger(),
                      "Paused IK motor commands for motion %d", msg->motion_num);
        });
    motion_end_sub_ = create_subscription<irc_humanoid_interfaces::msg::MotionOperator>(
        "motion_end", motion_qos,
        [this](const irc_humanoid_interfaces::msg::MotionOperator::SharedPtr msg) {
          if (msg->motion_end == 0 || msg->motion_num != active_motion_num_)
            return;
          IK.set_motor_publish_enabled(true);
          RCLCPP_INFO(get_logger(),
                      "Resumed IK motor commands after motion %d", msg->motion_num);
          active_motion_num_ = -1;
        });
  }

  // MSG //
  irc_humanoid_interfaces::msg::IkEndMsg IkEnd;
  irc_humanoid_interfaces::msg::IkCoordMsg IkCoord;
  irc_humanoid_interfaces::msg::IkPatternMsg IkPattern;
  irc_humanoid_interfaces::msg::IkComMsg IkCOM;
  irc_humanoid_interfaces::msg::IkLTCMsg IkLTC;

  irc_humanoid_interfaces::msg::ImuMsg IMU;
  irc_humanoid_interfaces::msg::Master2IkMsg Master2ik;
  irc_humanoid_interfaces::msg::Tune2IkMsg Tune2ik;
  irc_humanoid_interfaces::msg::ZmpMsg ZMP;

  // callback function //
  void get_parameters();
  void master2ik_callback(const irc_humanoid_interfaces::msg::Master2IkMsg::SharedPtr msg);
  void imu_callback(const irc_humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
  void tune2ik_callback(const irc_humanoid_interfaces::msg::Tune2IkMsg::SharedPtr msg);
  void zmp_callback(const irc_humanoid_interfaces::msg::ZmpMsg::SharedPtr msg)
  {
  }
  void kick_flag_callback(const std_msgs::msg::String::SharedPtr msg);
  struct K_Value
  {
    double Pos_XR = 0;
    double Neg_XR = 0;
    double Pos_SideR = 0;
    double Neg_SideR = 0;
    double Pos_YawR = 0;
    double Neg_YawR = 0;

    double Pos_XL = 0;
    double Neg_XL = 0;
    double Pos_SideL = 0;
    double Neg_SideL = 0;
    double Pos_YawL = 0;
    double Neg_YawL = 0;

    double Pos_SideR_SwingMinus = 0;
    double Neg_SideR_SwingMinus = 0;
    double Pos_SideL_SwingMinus = 0;
    double Neg_SideL_SwingMinus = 0;

    int min = 0;
    int max = 0;
  };

  struct Adjust_Yaw // <-Compansate
  {
    /// for Compansate Yaw ///
    bool Yaw_flag = false;
    double desire_yaw = 0;
    double Yaw_gain = 0;
    int Yaw_cnt = 0;
  };

  struct Robit_Humanoid
  {
    double Center2Leg = 0;
    double Link2Link = 0;
    double Init_Z_Up = 0;
  };
  // Robit_Humanoid Model_Data;

  class Walk_Param
  {
  public:
    bool IK_Flag = false;

    double Entire_Time = 80.0;
    double Frequency = 1.0;

    double Start_Entire_Time = 30.0;
    double End_Entire_Time = 0.0;

    double Sink_Entire_Time = 30.0;

    struct X_Value
    {
      double X = 0.0;
      double Tuning_X = 0.0;
      double Default_X_Right = 0.0;
      double Default_X_Left = 0.0;
    };
    struct Y_Value
    {
      Robit_Humanoid Model_Data;
      double Side = 0.0;
      double Tuning_Side = 0.0;

      double Swing_Side_Right = 0.0;
      double Swing_Side_Left = 0.0;

      double Default_Y_Right = -(Model_Data.Center2Leg);
      double Default_Y_Left = Model_Data.Center2Leg;

      double Swing_Leg_Right = 0.0;
      double Swing_Leg_Left = 0.0;
      double Start_Swing = 0.0;
      double End_Swing = 0.0;
    };
    struct Z_Value
    {
      Robit_Humanoid Model_Data;
      double Default_Z_Right = -(2 * Model_Data.Link2Link - Model_Data.Init_Z_Up);
      double Default_Z_Left = -(2 * Model_Data.Link2Link - Model_Data.Init_Z_Up);

      double Rise_Leg_Right = 0.0;
      double Rise_Leg_Left = 0.0;

      double Start_Rise = 0.0;
      double End_Rise = 0.0;
      double Z_com = 0.0;

      double R_Rise_Condition = 0.0;
      double L_Rise_Condition = 0.0;
    };
    struct Yaw_Value
    {
      double Yaw = 0.0;
      double Tuning_Yaw = 0.0;
    };
    struct Shoulder_Value
    {
      double Swing_Shoulder_Right = 0.0;
      double Swing_Shoulder_Left = 0.0;
    };

    struct Ratio_Flag
    {
      int Ratio_Flag;
    };

    X_Value X;
    Y_Value Y;
    Z_Value Z;
    Yaw_Value Yaw_R;
    Yaw_Value Yaw_L;
    Shoulder_Value Shoulder;
    Ratio_Flag Check_ratio;
  };

  Walk_Param Now_Param;
  Walk_Param Past_Param;

  class Balancing
  {
  public:
    double Target_Time = 0.0;

    /// PID control ///
    double Balance_Value_0 = 0.0;
    double Balance_Value_1 = 0.0;
    double Balance_Value_2 = 0.0;
    double Balance_Value_3 = 0.0;
    double Balance_Value_4 = 0.0;
    double Balance_Value_5 = 0.0;
    double Balance_Pitch_GP = 0.0;
    double Balance_Pitch_GI = 0.0;
    double Balance_Pitch_GD = 0.0;
    double Balance_Pitch_ELIMIT = 0.0;
    double Balance_Pitch_OLIMIT = 0.0;
    double Balance_Pitch_Neg_Target = 0.0;
    double Balance_Pitch_Pos_Target = 0.0;
    double Balance_Angle_Pitch_GP = 0.0;
    double Balance_Angle_Pitch_GI = 0.0;
    double Balance_Angle_Pitch_GD = 0.0;
    double Balance_Angle_Pitch_ELIMIT = 0.0;
    double Balance_Angle_Pitch_OLIMIT = 0.0;
    double Balance_Angle_Pitch_Neg_Target = 0.0;
    double Balance_Angle_Pitch_Pos_Target = 0.0;
    double Balance_Roll_GP = 0.0;
    double Balance_Roll_GI = 0.0;
    double Balance_Roll_GD = 0.0;
    double Balance_Roll_ELIMIT = 0.0;
    double Balance_Roll_OLIMIT = 0.0;
    double Balance_Roll_Neg_Target = 0.0;
    double Balance_Roll_Pos_Target = 0.0;
    double Balance_Pitch_GP_imu = 0.0;
    double Balance_Pitch_GI_imu = 0.0;
    double Balance_Pitch_GD_imu = 0.0;
    double Balance_Pitch_Neg_Target_imu = 0.0;
    double Balance_Pitch_Pos_Target_imu = 0.0;
    double Balance_Pitch_ELIMIT_imu = 0.0;
    double Balance_Pitch_OLIMIT_imu = 0.0;
    double Balance_Roll_GP_imu = 0.0;
    double Balance_Roll_GI_imu = 0.0;
    double Balance_Roll_GD_imu = 0.0;
    double Balance_Roll_Neg_Target_imu = 0.0;
    double Balance_Roll_Pos_Target_imu = 0.0;
    double Balance_Roll_ELIMIT_imu = 0.0;
    double Balance_Roll_OLIMIT_imu = 0.0;
    bool Balance_Pitch_Flag_imu = false;
    bool Balance_Roll_Flag_imu = false;
    bool Balance_Pitch_Flag = false;
    bool Balance_Ankle_Pitch_Flag = false;
    bool Balance_Roll_Flag = false;

    /// ZMP Landing Time Control///
    double Landing_Time_R = 0.0;
    double Landing_Time_L = 0.0;
    double Landing_Error_R = 0.0;
    double Landing_Error_L = 0.0;
    bool Landing_flag_R = false;
    bool Landing_flag_L = false;
    double Swing_Control_L = 1.0;
    double Swing_Control_R = 1.0;
    double Swing_Control_add_L = 1.0;
    double Swing_Control_add_R = 1.0;
    double Swing_Moment_Gain_L = 0.3;
    double Swing_Moment_Gain_R = 0.3;
    int Swing_Control_Warning_cnt = 0;
    int Swing_Control_Safe_cnt = 0;
    int Landing_Time_Control_flag = false;

    int Ratio_Check_Flag = 0;
    int Support_Con = 99;
  };
  Balancing Balance;
  class Calc
  {
  public:
    double positive_position(double x);
    double negative_position(double x);
    double MAF(double x);
  };
  Calc Cal;

  // class Humanoid_Walk
  // {
public:
  // Balancing Balance;
  // Walk_Param Now_Param;
  // Walk_Param Past_Param;
  // Calc Cal;
  // std::shared_ptr<IK_Solve> IK;

  double Time_Right_Leg = 0.0;
  double Time_Left_Leg = 0.0;
  double Time_Right_Leg_End = 0.0;
  double Time_Left_Leg_End = 0.0;
  double Time_Right_Leg_Start = 0.0;
  double Time_Left_Leg_Start = 0.0;

  double Time_X = 0.0;
  double Time_Side = 0.0;
  double Time_Yaw_R = 0.0;
  double Time_Yaw_L = 0.0;

  double Amount_of_Change_X = 0.0;
  double Amount_of_Change_Y = 0.0;

private:
  int Ik_Flag_Past = 0;
  int Start_Cnt = 1;
  int End_Cnt = 5;

  int Repeat_Start_Cycle = 9; // Run : 9 , hanoi : 3
  int Repeat_End_Cycle = 2;   // 3

  bool Side_Change = false;
  bool X_Change = false;
  bool Yaw_L_Change = false;
  bool Yaw_R_Change = false;
  bool Ang_Change = false;

  double Temp_Param_Yaw_R = 0.0;
  double Temp_Param_Yaw_L = 0.0;
  double Temp_Param_Side = 0.0;
  double Temp_Param_X = 0.0;

  double Kinetic_X_R;
  double Kinetic_Y_R;
  double Kinetic_Z_R;
  double Kinetic_Yaw_R;

  double Past_Kinetic_X_R;
  double Past_Kinetic_Y_R;

  double Kinetic_X_L;
  double Kinetic_Y_L;
  double Kinetic_Z_L;
  double Kinetic_Yaw_L;

  double Kinetic_Shoulder_X_R;
  double Kinetic_Shoulder_X_L;
  double Kinetic_Shoulder_Y_R;
  double Kinetic_Shoulder_Y_L;

  double Accel_Rise_R = 0.0;
  double Accel_Rise_L = 0.0;
  double Accel_Entire_Time = 0.0;
  double Accel_Swing_R = 0.0;
  double Accel_Swing_L = 0.0;

  foot_trajectory Start_COM_Pattern;
  foot_trajectory Start_Rise_Pattern;
  foot_trajectory COM_Pattern;
  foot_trajectory Step_Pattern;
  foot_trajectory Side_Pattern;
  foot_trajectory Rise_Pattern;
  foot_trajectory Turn_Pattern;
  // };
public:
  class Timer
  {
  public:
    /// Timer variable
    double Timer_Time;
    double Timer_Time_Start = 0.0;
    double Timer_Time_End = 0.0;
  };
  Timer CNT;
  class Step_Acc
  {
  public:
    double step_acc_func(double basic, double acc, int num, int &cnt);

    int accel_pos_x_cnt = -1;
    int accel_neg_x_cnt = -1;
    double check_old_x = 0;
    double accel_pos_x = 0;
    double accel_neg_x = 0;
    double basic_x = 0;

    int accel_pos_y_cnt = -1;
    int accel_neg_y_cnt = -1;
    double check_old_y = 0;
    double accel_pos_y = 0;
    double accel_neg_y = 0;
    double basic_y = 0;

    int accel_pos_z_cnt = -1;
    int accel_neg_z_cnt = -1;
    double check_old_z = 0;
    double accel_pos_z = 0;
    double accel_neg_z = 0;
    double basic_z = 0;

    int accel_pos_x_num = 8;
    int accel_neg_x_num = 8;
    int accel_pos_y_num = 6;
    int accel_neg_y_num = 6;
    int accel_pos_z_num = 4;
    int accel_neg_z_num = 4;
  };
  // Step_Acc Acc;
  void Walk_Start_End(Walk_Param &Now_Param, Walk_Param &Past_Param);
  void Generate_Pattern(Walk_Param &Now_Param);
  void Result_Pattern(Walk_Param &Now_Param);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  std::mutex pub_mutex;
  // Publisher //
  rclcpp::Publisher<irc_humanoid_interfaces::msg::IkEndMsg>::SharedPtr Ikend_Pub;
  rclcpp::Publisher<irc_humanoid_interfaces::msg::IkCoordMsg>::SharedPtr Ikcoordinate_Pub;
  rclcpp::Publisher<irc_humanoid_interfaces::msg::IkPatternMsg>::SharedPtr walk_pattern_Pub;
  rclcpp::Publisher<irc_humanoid_interfaces::msg::IkComMsg>::SharedPtr COM_Pub;
  rclcpp::Publisher<irc_humanoid_interfaces::msg::IkLTCMsg>::SharedPtr Landing_Pub;
  rclcpp::Publisher<dynamixel_hardware_msgs::msg::DynamixelControlMsgs>::SharedPtr Motor_Pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kick_flag_pub_;
  
  std::shared_ptr<rclcpp::Subscription<irc_humanoid_interfaces::msg::ImuMsg>> Imu_Sub;
  std::shared_ptr<rclcpp::Subscription<irc_humanoid_interfaces::msg::Master2IkMsg>> Master2ik_Sub;
  std::shared_ptr<rclcpp::Subscription<irc_humanoid_interfaces::msg::Tune2IkMsg>> Tune2ik_Sub;
  std::shared_ptr<rclcpp::Subscription<irc_humanoid_interfaces::msg::ZmpMsg>> Zmp_Sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kick_flag_sub_;
  rclcpp::Subscription<irc_humanoid_interfaces::msg::MotionOperator>::SharedPtr
      motion_start_sub_;
  rclcpp::Subscription<irc_humanoid_interfaces::msg::MotionOperator>::SharedPtr
      motion_end_sub_;
  int32_t active_motion_num_ = -1;

  string kick_flag_msg_ = "";
  string kick_flag_msg_pending = "";
  bool kick_flag_ = false;
  bool kick_pending_ = false;
  bool kick_interval = false;
  bool prev_kick_interval_ = false;
  bool kick_time = false;
  bool ik_1 = false;
  bool ik_2 = false;
  bool ik_3 = false;
  bool kick_l_armed_ = false;
  bool kick_r_armed_ = false;
  bool prev_ik_3_ = false;
  int kick_pending_wait_cnt_ = 0;
  double kick_pending_start_time_ = -1.0;
  kick_trajectory Kick_Return;
  double kick_return_t = 0.0;
  int cycle_flag =0;
  bool acc_flag = false;
  int Entire_time_flag = -1;
  void timer_callback();
};

#endif
