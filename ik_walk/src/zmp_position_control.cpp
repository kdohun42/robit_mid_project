#include "zmp_position_control.hpp"

PID Zmp_Pitch_Balance;
PID Zmp_Pitch_Angle_Balance;
PID Zmp_Roll_Balance;

using namespace std;

void IK_zmp_pos_control::PID_Zmp_Pitch_Balancing(double Zmp_Input, double Pitch_GP, double Pitch_GI, double Pitch_GD,
                                                 double Pitch_ELIMIT, double Pitch_OLIMIT, double Pitch_Neg_Target,
                                                 double Pitch_Pos_Target)
{
  if (Pitch_Pos_Target > Zmp_Input && Pitch_Neg_Target < Zmp_Input)
  {
    Pitch_OLIMIT = 1;
  }

  PID_Control_init(&Zmp_Pitch_Balance, Pitch_GP, Pitch_GI, Pitch_GD, Pitch_ELIMIT, Pitch_OLIMIT);

  if (Zmp_Input >= Pitch_Pos_Target)
  {
    PID_Control_Float(&Zmp_Pitch_Balance, Pitch_Pos_Target, Zmp_Input);
    Target_X = -Zmp_Pitch_Balance.nowOutput;
  }
  else if (Zmp_Input <= Pitch_Neg_Target)
  {
    PID_Control_Float(&Zmp_Pitch_Balance, Pitch_Neg_Target, Zmp_Input);
    Target_X = -Zmp_Pitch_Balance.nowOutput;
  }
  else
  {
    Target_X = 0;
  }
}

void IK_zmp_pos_control::PID_Zmp_Roll_Balancing(double Zmp_Input, double Roll_GP, double Roll_GI, double Roll_GD,
                                                double Roll_ELIMIT, double Roll_OLIMIT, double Roll_Neg_Target,
                                                double Roll_Pos_Target)
{
  if (Roll_Pos_Target > Zmp_Input && Roll_Neg_Target < Zmp_Input)
  {
    Roll_OLIMIT = 1;
  }

  PID_Control_init(&Zmp_Roll_Balance, Roll_GP, Roll_GI, Roll_GD, Roll_ELIMIT, Roll_OLIMIT);

  if (Zmp_Input >= Roll_Pos_Target)
  {
    PID_Control_Float(&Zmp_Roll_Balance, Roll_Pos_Target, Zmp_Input);
    Target_Roll_Angle = Zmp_Roll_Balance.nowOutput;
  }
  else if (Zmp_Input <= Roll_Neg_Target)
  {
    PID_Control_Float(&Zmp_Roll_Balance, Roll_Neg_Target, Zmp_Input);
    Target_Roll_Angle = Zmp_Roll_Balance.nowOutput;
  }
  else
  {
    Target_Roll_Angle = 0;
  }
}

void IK_zmp_pos_control::PID_Zmp_Pitch_Angle_Balancing(double Zmp_Input, double Pitch_Angle_GP, double Pitch_Angle_GI,
                                                       double Pitch_Angle_GD, double Pitch_Angle_ELIMIT,
                                                       double Pitch_Angle_OLIMIT, double Pitch_Angle_Neg_Target,
                                                       double Pitch_Angle_Pos_Target)
{
  if (Pitch_Angle_Pos_Target > Zmp_Input && Pitch_Angle_Neg_Target < Zmp_Input)
  {
    Pitch_Angle_OLIMIT = 1;
  }

  PID_Control_init(&Zmp_Pitch_Angle_Balance, Pitch_Angle_GP, Pitch_Angle_GI, Pitch_Angle_GD, Pitch_Angle_ELIMIT,
                   Pitch_Angle_OLIMIT);

  if (Zmp_Input >= Pitch_Angle_Pos_Target)
  {
    PID_Control_Float(&Zmp_Pitch_Angle_Balance, Pitch_Angle_Pos_Target, Zmp_Input);
    Target_Pitch_Angle = Zmp_Pitch_Angle_Balance.nowOutput;
  }
  else if (Zmp_Input <= Pitch_Angle_Neg_Target)
  {
    PID_Control_Float(&Zmp_Pitch_Angle_Balance, Pitch_Angle_Neg_Target, Zmp_Input);
    Target_Pitch_Angle = Zmp_Pitch_Angle_Balance.nowOutput;
  }
  else
  {
    Target_Pitch_Angle = 0;
  }
}