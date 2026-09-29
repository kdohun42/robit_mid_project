#include "imu_position_control.hpp"

PID pitch_control;
PID roll_control;

double Knee_Add_Angle = 0;
double pos_pitch_OLIMIT = 0;
double neg_pitch_OLIMIT = 0;

using namespace std;

void IK_imu_pos_control::PD_Pitch_control(double input, double pitch_GP, double pitch_GI, double pitch_GD,
                                          double pitch_ELIMIT, double pitch_OLIMIT, double pitch_neg_Target,
                                          double pitch_pos_Target)
{
  if (pitch_pos_Target > input && pitch_neg_Target < input)
  {
    pitch_OLIMIT = 1;
  }

  PID_Control_init(&pitch_control, pitch_GP, pitch_GI, pitch_GD, pitch_ELIMIT, pitch_OLIMIT);

  if (input >= pitch_pos_Target)
  {
    PID_Control_Float(&pitch_control, pitch_pos_Target, input);
    Pitch_ADD_Angle = pitch_control.nowOutput;
  }
  else if (input <= pitch_neg_Target)
  {
    PID_Control_Float(&pitch_control, pitch_neg_Target, input);
    Pitch_ADD_Angle = pitch_control.nowOutput;
    cout<<"imu_balance:"<<Pitch_ADD_Angle<<endl;
  }
  else
  {
    Pitch_ADD_Angle = 0;
  }
}

void IK_imu_pos_control::PD_Roll_control(double input, double roll_GP, double roll_GI, double roll_GD, double roll_ELIMIT,
                                     double roll_OLIMIT, double roll_neg_Target, double roll_pos_Target)
{
  if (roll_pos_Target > input && roll_neg_Target < input)
  {
    roll_OLIMIT = 1;
  }

  PID_Control_init(&roll_control, roll_GP, roll_GI, roll_GD, roll_ELIMIT, roll_OLIMIT);

  if (input >= roll_pos_Target)
  {
    PID_Control_Float(&roll_control, roll_pos_Target, input);
    Roll_ADD_Angle = roll_control.nowOutput;
  }
  else if (input <= roll_neg_Target)
  {
    PID_Control_Float(&roll_control, roll_neg_Target, input);
    Roll_ADD_Angle = roll_control.nowOutput;
  }
  else
  {
    Roll_ADD_Angle = 0;
  }
}
