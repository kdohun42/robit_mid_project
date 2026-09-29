#ifndef ZMP_POSITION_CONTROL_HPP
#define ZMP_POSITION_CONTROL_HPP

#include <iostream>
#include <fstream>
#include <cmath>
#include "pid_control_float.hpp"

using namespace std;

class IK_zmp_pos_control
{
public:
  double Target_X = 0.0;
  double Target_Pitch_Angle = 0.0;
  double Target_Roll_Angle = 0.0;

  void PID_Zmp_Pitch_Balancing(double Zmp_Input, double Pitch_GP, double Pitch_GI, double Pitch_GD, double Pitch_ELIMIT,
                               double Pitch_OLIMIT, double Pitch_Neg_Target, double Pitch_Pos_Target);
  void PID_Zmp_Roll_Balancing(double Zmp_Input, double Roll_GP, double Roll_GI, double Roll_GD, double Roll_ELIMIT,
                              double Roll_OLIMIT, double Roll_Neg_Target, double Roll_Pos_Target);
  void PID_Zmp_Pitch_Angle_Balancing(double Zmp_Input, double Imu_GP, double Imu_GI, double Imu_GD, double Imu_ELIMIT,
                                     double Imu_OLIMIT, double Imu_Neg_Target, double Imu_Pos_Target);

private:
};
#endif