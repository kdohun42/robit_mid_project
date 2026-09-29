#ifndef IMU_POSITION_CONTROL_HPP
#define IMU_POSITION_CONTROL_HPP

#include <iostream>
#include <fstream>
#include <cmath>
#include "pid_control_float.hpp"

using namespace std;

class IK_imu_pos_control
{
public:
  double Pitch_ADD_Angle = 0;
  double Roll_ADD_Angle = 0;

  void PD_Pitch_control(double input, double pitch_GP, double pitch_GI, double pitch_GD, double pitch_ELIMIT,
                        double pitch_OLIMIT, double pitch_neg_Target, double pitch_pos_Target);
  void PD_Roll_control(double input, double roll_GP, double roll_GI, double roll_GD, double roll_ELIMIT,
                       double roll_OLIMIT, double roll_neg_Target, double roll_pos_Target);

private:
};

#endif
