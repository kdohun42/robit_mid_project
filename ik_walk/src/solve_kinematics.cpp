#include "solve_kinematics.hpp"
#include "ik_walk.hpp"

extern IK_imu_pos_control Imu_Balance;

double IK_Solve::ang2pos(double angle)
{
  return (double)((angle * 4096.0) / 360.0);
}
void IK_Solve::init_save()
{
  // 0~5번 모터만 사용 (6,7,8,9번 제외)
   double top_angle[6] = {512, 512, 512, 512, 512, 512};

  // 0~5번 모터 초기화
  for (int DXL_ID = 0; DXL_ID < 6; DXL_ID++)
  {
    g_DXL_ID_position[DXL_ID] = top_angle[DXL_ID]*4;
    g_DXL_ID_Save_position[DXL_ID] = top_angle[DXL_ID]*4;
    cout << "g_DXL_ID_Save_position[" << DXL_ID << "] = " << (g_DXL_ID_Save_position[DXL_ID] / 4) << endl;
  }

  // 10~21번 모터 초기화 (6,7,8,9번은 사용하지 않음)
  for (int DXL_ID = 10; DXL_ID < 23; DXL_ID++)
  {
    g_DXL_ID_position[DXL_ID] = 512 * 4;
    g_DXL_ID_Save_position[DXL_ID] = 512 * 4;
    cout << "g_DXL_ID_Save_position[" << DXL_ID << "] = " << (g_DXL_ID_Save_position[DXL_ID] / 4) << endl;
  }
  // 22번 모터는 사용하지 않음
  g_DXL_ID_position[22] = 512 * 4 + ang2pos(0);
  g_DXL_ID_Save_position[22] = 512 * 4 + ang2pos(0);
}

void IK_Solve::Balance_Control_Body_Upright(double Input_Data, double Robot_Z, double Time, double Rise_Condition, double Input_Data2, double Balance_Value_Msg, double Balance_Value_Msg2,
                                            double Zmp_Pitch_Limit_Left, double Zmp_Pitch_Limit_Right, double Input_Data3, double Balance_Value_Msg3, double Balance_Value_Msg4,
                                            double L_1, double Zmp_Roll_Limit_Left, double Zmp_Roll_Limit_Right, int Support_Condition)
{
  // 현재는 사용되지 않지만, 추후 제거 요망.
  // (void)Robot_Z;
  // (void)Time;
  // (void)Rise_Condition;
  // (void)Input_Data2;
  // (void)Balance_Value_Msg2;
  // (void)Zmp_Pitch_Limit_Left;
  // (void)Zmp_Pitch_Limit_Right;
  // (void)Balance_Value_Msg4;
  // (void)L_1;
  // (void)Zmp_Roll_Limit_Left;
  // (void)Zmp_Roll_Limit_Right;
  // (void)Input_Data3;
  // (void)Balance_Value_Msg3;
  // (void)Balance_Value_Msg4;
  // 현재는 사용되지 않지만, 추후 제거 요망.

  Now_Balance_Theta.Theta1 = 0;
  Now_Balance_Theta.Theta2 = 0;
  Now_Balance_Theta.Theta3 = -Input_Data * Balance_Value_Msg;
  Compensate_Angle_Data.Vertical_Angle = -Input_Data3 * Balance_Value_Msg3;

  if (Support_Condition == 1)
  {
    Now_Balance_Theta.Right_Theta1 = Now_Balance_Theta.Theta1;
    Now_Balance_Theta.Right_Theta2 = Now_Balance_Theta.Theta2;
    Now_Balance_Theta.Right_Theta3 = Now_Balance_Theta.Theta3;
    Now_Balance_Theta.Left_Theta1 = 0;
    Now_Balance_Theta.Left_Theta2 = 0;
    Now_Balance_Theta.Left_Theta3 = 0;

    //        if(Input_Data3 <= 0.0)
    //        {
    //            Compensate_Angle_Data.Vertical_Angle = Compensate_Angle_Data.Vertical_Angle;
    //        }
    //        else if(Input_Data3 > 0.0)
    //        {
    //            Compensate_Angle_Data.Vertical_Angle = 0;
    //        }
    Compensate_Angle_Data.R_Vertical_Angle = -Compensate_Angle_Data.Vertical_Angle;
    Compensate_Angle_Data.L_Vertical_Angle = 0;
    //        Compensate_Angle_Data.Compensate_Angle_Left_14 = 0;
    //        Compensate_Angle_Data.Compensate_Angle_Right_15 =
    //        (Imu_Balance.Pitch_ADD_Angle*1)/*-Imu_Info.pitch_acc*0.01*//*-(Past_Param.X.X*0.4)*/;
  }
  else if (Support_Condition == -1)
  {
    Now_Balance_Theta.Left_Theta1 = Now_Balance_Theta.Theta1;
    Now_Balance_Theta.Left_Theta2 = Now_Balance_Theta.Theta2;
    Now_Balance_Theta.Left_Theta3 = Now_Balance_Theta.Theta3;
    Now_Balance_Theta.Right_Theta1 = 0;
    Now_Balance_Theta.Right_Theta2 = 0;
    Now_Balance_Theta.Right_Theta3 = 0;

    //        if(Input_Data3 <= 0.0)
    //        {
    //            Compensate_Angle_Data.Vertical_Angle = 0;
    //        }
    //        else if(Input_Data3 > 0.0)
    //        {
    //            Compensate_Angle_Data.Vertical_Angle = -Compensate_Angle_Data.Vertical_Angle;
    //        }
    Compensate_Angle_Data.R_Vertical_Angle = 0;
    Compensate_Angle_Data.L_Vertical_Angle = Compensate_Angle_Data.Vertical_Angle;
    //        Compensate_Angle_Data.Compensate_Angle_Left_14 = (Imu_Balance.Pitch_ADD_Angle*1);
    //        Compensate_Angle_Data.Compensate_Angle_Right_15 = 0/*-Imu_Info.pitch_acc*0.01*//*-(Past_Param.X.X*0.4)*/;
  }
  else if (Support_Condition == 0)
  {
    Now_Balance_Theta.Right_Theta1 = Now_Balance_Theta.Theta1;
    Now_Balance_Theta.Right_Theta2 = Now_Balance_Theta.Theta2;
    Now_Balance_Theta.Right_Theta3 = Now_Balance_Theta.Theta3;

    Now_Balance_Theta.Left_Theta1 = Now_Balance_Theta.Theta1;
    Now_Balance_Theta.Left_Theta2 = Now_Balance_Theta.Theta2;
    Now_Balance_Theta.Left_Theta3 = Now_Balance_Theta.Theta3;

    if (Input_Data3 <= 0.0)
    {
      Compensate_Angle_Data.Vertical_Angle = Compensate_Angle_Data.Vertical_Angle;
    }
    else if (Input_Data3 > 0.0)
    {
      Compensate_Angle_Data.Vertical_Angle = -Compensate_Angle_Data.Vertical_Angle;
    }

    Compensate_Angle_Data.Vertical_Angle = 0;
    Compensate_Angle_Data.R_Vertical_Angle = Compensate_Angle_Data.Vertical_Angle * 0.41;
    Compensate_Angle_Data.L_Vertical_Angle = Compensate_Angle_Data.Vertical_Angle * 0.41;
    //        Compensate_Angle_Data.Compensate_Angle_Left_14 =
    //        (Imu_Balance.Pitch_ADD_Angle*1)/*-Imu_Info.pitch_acc*0.01*//*-(Past_Param.X.X*0.4)*/;
    //        Compensate_Angle_Data.Compensate_Angle_Right_15 =
    //        (Imu_Balance.Pitch_ADD_Angle*1)/*-Imu_Info.pitch_acc*0.01*//*-(Past_Param.X.X*0.4)*/;
  }
  else if (Support_Condition == 99)
  {
    Now_Balance_Theta.Right_Theta1 = 0;
    Now_Balance_Theta.Right_Theta2 = 0;
    Now_Balance_Theta.Right_Theta3 = 0;

    Now_Balance_Theta.Left_Theta1 = 0;
    Now_Balance_Theta.Left_Theta2 = 0;
    Now_Balance_Theta.Left_Theta3 = 0;

    Compensate_Angle_Data.Vertical_Angle = 0;
    Compensate_Angle_Data.R_Vertical_Angle = Compensate_Angle_Data.Vertical_Angle;
    Compensate_Angle_Data.L_Vertical_Angle = Compensate_Angle_Data.Vertical_Angle;
    //        Compensate_Angle_Data.Compensate_Angle_Left_14 =
    //        (Imu_Balance.Pitch_ADD_Angle*1)/*-Imu_Info.pitch_acc*0.01*//*-(Past_Param.X.X*0.4)*/;
    //        Compensate_Angle_Data.Compensate_Angle_Right_15 =
    //        (Imu_Balance.Pitch_ADD_Angle*1)/*-Imu_Info.pitch_acc*0.01*//*-(Past_Param.X.X*0.4)*/;
  }
}
void IK_Solve::solve(double pX_r, double pY_r, double pZ_r, double Yaw_R_deg, double pX_l, double pY_l, double pZ_l, double Yaw_L_deg, int body, double Shoulder_Pattern_X_R, double Shoulder_Pattern_X_L, double Shoulder_Pattern_Y_R, double Shoulder_Pattern_Y_L, double Balance_Theta1, double Balance_Theta2, double Balance_Theta3, double R_Rise_Condition, double L_Rise_Condition, double R_Rise_Max, double L_Rise_Max, double Imu_pitch, double Imu_pos, double Imu_neg, double Imu_roll, double Now_X, double Waist_Yaw_deg)
{
  //   // 현재는 사용되지 않지만, 추후 제거 요망.
  //     (void)Balance_Theta1;
  //     (void)Balance_Theta2;
  //     (void)Balance_Theta3;
  //     (void)R_Rise_Condition;
  //     (void)L_Rise_Condition;
  //     (void)R_Rise_Max;
  //     (void)L_Rise_Max;
  //     (void)Imu_pitch;
  //     (void)Imu_pos;
  //     (void)Imu_neg;
  // 현재는 사용되지 않지만, 추후 제거 요망.
  // humanoid_interfaces::msg::IkAngleSimMsg Position_Info;
  /////////////////////////////////  Solve Start  //////////////////////////////////

  const double C2p = 45, L_1 = 140, L_2 = 140; // C2p(Center to pelvis) = 중심에서 roll 축까지의 길이, L_1,2 = 링크 위 아래 길이
  double Yaw_L_rad = (double)(Yaw_L_deg * deg2rad), Yaw_R_rad = (double)(Yaw_R_deg * deg2rad);


  // 왼다리 역기구학(왼다리 모터의
  // 각도)------------------------------------------------------------------------------------------
  double L_temp[4] = {0, 0, 0, 0};
  double L_Valid =
      (double)(((pX_l * pX_l) + ((pY_l - C2p) * (pY_l - C2p)) + ((pZ_l) * (pZ_l)) - (L_1 * L_1) - (L_2 * L_2)) / (2 * L_1 * L_2)); // 제 2 cos법칙 이용

  InvLegAngle[0] = Yaw_L_rad;
  InvLegAngle[3] = (double)acos(L_Valid);
  InvLegAngle[1] = (double)(atan2(pZ_l, -pX_l * (double)sin(Yaw_L_rad) + (pY_l - C2p) * (double)cos(Yaw_L_rad)));

  L_temp[0] = (double)(-L_2 * sin(InvLegAngle[3]));
  L_temp[1] = (double)(L_1 + L_2 * cos(InvLegAngle[3]));
  L_temp[2] = (double)(pX_l * cos(Yaw_L_rad) + (pY_l - C2p) * sin(Yaw_L_rad));
  L_temp[3] = (double)(-pX_l * sin(Yaw_L_rad) * cos(InvLegAngle[1]) + (pY_l - C2p) * cos(Yaw_L_rad) * cos(InvLegAngle[1]) + pZ_l * sin(InvLegAngle[1]));

  L_Valid = (double)((L_temp[0] * L_temp[3] - L_temp[1] * L_temp[2]) / (L_temp[0] * L_temp[0] + L_temp[1] * L_temp[1]));

  InvLegAngle[2] = (double)asin(L_Valid);
  InvLegAngle[1] = (double)(InvLegAngle[1] + (PI / 2.0));

  InvLegAngle[4] = -(InvLegAngle[3] + InvLegAngle[2]);
  InvLegAngle[5] = (1.0 * InvLegAngle[1]);

  // 오른다리 역기구학(오른다리 모터의
  // 각도)--------------------------------------------------------------------------------------
  double R_temp[4] = {0, 0, 0, 0};
  double R_Valid =(double)(((pX_r * pX_r) + ((pY_r + C2p) * (pY_r + C2p)) + ((pZ_r) * (pZ_r)) - (L_1 * L_1) - (L_2 * L_2)) / (2 * L_1 * L_2)); // 제 2 cos법칙 이용

  InvLegAngle[6] = Yaw_R_rad;
  InvLegAngle[9] = (double)acos(R_Valid);
  InvLegAngle[7] = (double)(atan2(pZ_r, -pX_r * (double)sin(Yaw_R_rad) + (C2p + pY_r) * (double)cos(Yaw_R_rad)));

  R_temp[0] = (double)(-L_2 * sin(InvLegAngle[9]));
  R_temp[1] = (double)(L_1 + L_2 * cos(InvLegAngle[9]));
  R_temp[2] = (double)(pX_r * cos(Yaw_R_rad) + (pY_r + C2p) * sin(Yaw_R_rad));
  R_temp[3] = (double)(-pX_r * sin(Yaw_R_rad) * cos(InvLegAngle[7]) + (pY_r + C2p) * cos(Yaw_R_rad) * cos(InvLegAngle[7]) + pZ_r * sin(InvLegAngle[7]));

  R_Valid = (double)((R_temp[0] * R_temp[3] - R_temp[1] * R_temp[2]) / (R_temp[0] * R_temp[0] + R_temp[1] * R_temp[1]));

  InvLegAngle[8] = (double)asin(R_Valid);
  InvLegAngle[7] = (double)(InvLegAngle[7] + (PI / 2.0));

  InvLegAngle[10] = -(InvLegAngle[9] + InvLegAngle[8]);
  InvLegAngle[11] = (1.0 * InvLegAngle[7]);

  for (int i = 0; i < 23; i++)
  {
    g_DXL_ID_position[i] = g_DXL_ID_Save_position[i];
  }

  // 허리(22번, 골반 회전): 스윙(나가는) 발 방향으로 허리를 돌려 보행 틀어짐 보완.
  // Save(2048=중립) 기준으로 회전각을 더한다. Waist_Yaw_deg=0 이면 허리는 중립 유지.
  // g_DXL_ID_position[22] += ang2pos(Waist_Yaw_deg);

  Compensate_Angle_Data.Compensate_Angle_Left_10 = (Imu_Balance.Roll_ADD_Angle) +Compensate_Angle_Data.L_Vertical_Angle +
      Compensate_Angle_Data.R_Vertical_Angle /*-Imu_Info.roll_acc*0.01*/;
  if (Imu_pitch <= (Imu_pos + Imu_neg) / 2)
  {
    Compensate_Angle_Data.Compensate_Angle_Left_14 = (Imu_Balance.Pitch_ADD_Angle * 4.0) /*-Imu_Info.pitch_acc*0.01*/ /*-(Past_Param.X.X*0.4)*/;
    Compensate_Angle_Data.Compensate_Angle_Left_16 = -(Imu_Balance.Pitch_ADD_Angle * 2.0);
  }
  else
  {
    Compensate_Angle_Data.Compensate_Angle_Left_14 = (Imu_Balance.Pitch_ADD_Angle * 2.0) /*-Imu_Info.pitch_acc*0.01*/ /*-(Past_Param.X.X*0.4)*/;
    Compensate_Angle_Data.Compensate_Angle_Left_16 = -(Imu_Balance.Pitch_ADD_Angle * 0.0);
    Compensate_Angle_Data.Compensate_Angle_Left_18 = -(Imu_Balance.Pitch_ADD_Angle*1.0);
  }

  Compensate_Angle_Data.Compensate_Angle_Left_18 =
      Now_Balance_Theta.Left_Theta3 + (Imu_Balance.Pitch_ADD_Angle * 1.0) + (Imu_Balance.Pitch_ADD_Angle * 0.0);
  Compensate_Angle_Data.Compensate_Angle_Left_20 =
      -Compensate_Angle_Data.L_Vertical_Angle - (Imu_Balance.Roll_ADD_Angle * 0.5);

  //////////////////////////////////////////////////////////////
  Compensate_Angle_Data.Compensate_Angle_Right_11 = (Imu_Balance.Roll_ADD_Angle) +Compensate_Angle_Data.R_Vertical_Angle +
      Compensate_Angle_Data.L_Vertical_Angle /*-Imu_Info.roll_acc*0.01*/;
  if (Imu_pitch <= (Imu_pos + Imu_neg) / 2)
  {
    Compensate_Angle_Data.Compensate_Angle_Right_15 = (Imu_Balance.Pitch_ADD_Angle * 4.0) /*-Imu_Info.pitch_acc*0.01*/ /*-(Past_Param.X.X*0.4)*/;
    Compensate_Angle_Data.Compensate_Angle_Right_17 = -(Imu_Balance.Pitch_ADD_Angle * 2.0);
  }
  else
  {
    Compensate_Angle_Data.Compensate_Angle_Right_15 = (Imu_Balance.Pitch_ADD_Angle * 2.0) /*-Imu_Info.pitch_acc*0.01*/ /*-(Past_Param.X.X*0.4)*/;
    Compensate_Angle_Data.Compensate_Angle_Right_17 = -(Imu_Balance.Pitch_ADD_Angle * 0.0);
    // Compensate_Angle_Data.Compensate_Angle_Right_19 = -(Imu_Balance.Pitch_ADD_Angle*1.0);
  }
  Compensate_Angle_Data.Compensate_Angle_Right_19 =
      Now_Balance_Theta.Right_Theta3 + (Imu_Balance.Pitch_ADD_Angle * 1) + (Imu_Balance.Pitch_ADD_Angle * 0.0);
  Compensate_Angle_Data.Compensate_Angle_Right_21 =
      -Compensate_Angle_Data.R_Vertical_Angle - (Imu_Balance.Roll_ADD_Angle * 0.5);

  if (abs(Imu_pitch) < 20)
  {
    if (pZ_l > pZ_r)
    {
      Compensate_Angle_Data.Compensate_Angle_Left_14 -= (Imu_Balance.Pitch_ADD_Angle * 2.5);
      Compensate_Angle_Data.Compensate_Angle_Right_15 -= (Imu_Balance.Pitch_ADD_Angle * 0.0);
    }
    else
    {
      Compensate_Angle_Data.Compensate_Angle_Left_14 -= (Imu_Balance.Pitch_ADD_Angle * 0.0);
      Compensate_Angle_Data.Compensate_Angle_Right_15 -= (Imu_Balance.Pitch_ADD_Angle * 2.5);
    }
  }

  if (abs(Imu_pitch) > 25) // fall down
  {
    Compensate_Angle_Data.Compensate_Angle_Left_14 = 0;
    Compensate_Angle_Data.Compensate_Angle_Right_15 = 0;
    Compensate_Angle_Data.Compensate_Angle_Left_16 = 0;
    Compensate_Angle_Data.Compensate_Angle_Right_17 = 0;
    Compensate_Angle_Data.Compensate_Angle_Left_18 = 0;
    Compensate_Angle_Data.Compensate_Angle_Right_19 = 0;
  }

  // 무게중심 이동
  // if (Run_Flag == true)
  // {
  //   Run_Add = (Now_X * 0.5);
  // }
  // else
  //   Run_Add = 0;

  int Run_Add = 0;

  g_DXL_ID_position[0] += ang2pos(Shoulder_Pattern_X_L + 1.5 );
  g_DXL_ID_position[1] -= ang2pos(Shoulder_Pattern_X_R + 1.5 /** Imu_Balance.Pitch_ADD_Angle -Compensate_Angle_Data.Horizontal_Angle*3*/);

  g_DXL_ID_position[12] -= ang2pos(InvLegAngle[0] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_12) + Now_Motor_Angle.Motor_Angle_12 + Compensate_Angle_Data.Compensate_Angle_Left_12; // left_Yaw
  g_DXL_ID_position[14] += ang2pos(InvLegAngle[2] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_14 + Compensate_Angle_Data.Compensate_Angle_Left_14) + Now_Motor_Angle.Motor_Angle_14 + Run_Add; // left_pitch
  g_DXL_ID_position[16] += ang2pos(InvLegAngle[3] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_16 + Compensate_Angle_Data.Compensate_Angle_Left_16) + Now_Motor_Angle.Motor_Angle_16 - Run_Add; // left_pitch
  g_DXL_ID_position[18] -= ang2pos(InvLegAngle[4] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_18 +  Compensate_Angle_Data.Compensate_Angle_Left_18 /*+Compensate_Angle_Data.Horizontal_Angle*/) + Now_Motor_Angle.Motor_Angle_18; // left_pitch

  g_DXL_ID_position[10] += 2*(ang2pos(InvLegAngle[1] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_10 +  Compensate_Angle_Data.Compensate_Angle_Left_10) + Now_Motor_Angle.Motor_Angle_10); // left_roll
  g_DXL_ID_position[20] -= 2*(ang2pos((-InvLegAngle[5] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_20 /**2.5*/ /*+Compensate_Angle_Data.L_Vertical_Angle*/ + Compensate_Angle_Data.Compensate_Angle_Left_20)) + Now_Motor_Angle.Motor_Angle_20); // left_roll

  g_DXL_ID_position[13] -= ang2pos(InvLegAngle[6] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_13) + Now_Motor_Angle.Motor_Angle_13 + Compensate_Angle_Data.Compensate_Angle_Right_13; // right_Yaw
  g_DXL_ID_position[15] -= ang2pos(InvLegAngle[8] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_15 + Compensate_Angle_Data.Compensate_Angle_Right_15) + Now_Motor_Angle.Motor_Angle_15 + Run_Add; // right_pitch
  g_DXL_ID_position[17] -= ang2pos(InvLegAngle[9] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_17 + Compensate_Angle_Data.Compensate_Angle_Right_17) + Now_Motor_Angle.Motor_Angle_17 - Run_Add; // right_pitch
  g_DXL_ID_position[19] += ang2pos(InvLegAngle[10] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_19 + Compensate_Angle_Data.Compensate_Angle_Right_19 /*+Compensate_Angle_Data.Horizontal_Angle*/) + Now_Motor_Angle.Motor_Angle_19; // right_pitch

  g_DXL_ID_position[11] += 2*(ang2pos(InvLegAngle[7] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_11 + Compensate_Angle_Data.Compensate_Angle_Right_11) + Now_Motor_Angle.Motor_Angle_11); // right_roll
  g_DXL_ID_position[21] -= 2*(ang2pos((-InvLegAngle[11] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_21 /**5.5*/ /*+Compensate_Angle_Data.R_Vertical_Angle*/ + Compensate_Angle_Data.Compensate_Angle_Right_21)) + Now_Motor_Angle.Motor_Angle_21); // right_roll
  g_DXL_ID_position[2] = ang2pos(190);
  g_DXL_ID_position[6] = ang2pos(40);
  g_DXL_ID_position[7] = ang2pos(320);
  g_DXL_ID_position[8] = ang2pos(180);
  g_DXL_ID_position[9] = ang2pos(180);
  g_DXL_ID_position[23] = ang2pos(180);
  g_DXL_ID_position[24] = ang2pos(180);

  // // 발 겹침 방지
  // if (Imu_roll > 25)
  // {
  //   g_DXL_ID_position[10] += ang2pos(5);
  //   g_DXL_ID_position[20] += ang2pos(40);
  // }
  // else if (Imu_roll < -25)
  // {
  //   g_DXL_ID_position[11] -= ang2pos(5);
  //   g_DXL_ID_position[21] -= ang2pos(40);
  // }

  // // 손 프레임 변형 방지
  // if (Imu_pitch > 30)
  // {
  //   g_DXL_ID_position[0] += ang2pos(40);
  //   g_DXL_ID_position[1] -= ang2pos(40);
  // }
  // else if (Imu_pitch < -30)
  // {
  //   g_DXL_ID_position[0] -= ang2pos(30);
  //   g_DXL_ID_position[1] += ang2pos(30);
  // }

  // Position_Info.ang_l_0 = g_DXL_ID_position[0];
  // Position_Info.ang_l_1 = g_DXL_ID_position[2];
  // Position_Info.ang_l_2 = g_DXL_ID_position[10];
  // Position_Info.ang_l_3 = g_DXL_ID_position[12];
  // Position_Info.ang_l_4 = g_DXL_ID_position[14];
  // Position_Info.ang_l_5 = g_DXL_ID_position[16];
  // Position_Info.ang_l_6 = g_DXL_ID_position[18];
  // Position_Info.ang_l_7 = g_DXL_ID_position[20];
  // Position_Info.ang_l_8 = g_DXL_ID_position[4];

  // Position_Info.ang_r_0 = g_DXL_ID_position[1];
  // Position_Info.ang_r_1 = g_DXL_ID_position[3];
  // Position_Info.ang_r_2 = g_DXL_ID_position[11];
  // Position_Info.ang_r_3 = g_DXL_ID_position[13];
  // Position_Info.ang_r_4 = g_DXL_ID_position[15];
  // Position_Info.ang_r_5 = g_DXL_ID_position[17];
  // Position_Info.ang_r_6 = g_DXL_ID_position[19];
  // Position_Info.ang_r_7 = g_DXL_ID_position[21];
  // Position_Info.ang_r_8 = g_DXL_ID_position[5];

  // Motor_Position_Pub->publish(Position_Info);

  //        cout<<"Run_Flag : "<< Run_Flag <<endl;
  //        cout<<"Run_Add : "<< Run_Add <<endl;
  //  cout<<"10 : "<<g_DXL_ID_position[10]<<endl;
  //  cout<<"11 : "<<g_DXL_ID_position[11]<<endl;
  //  cout<<"12 : "<<g_DXL_ID_position[12]<<endl;
  //  cout<<"13 : "<<g_DXL_ID_position[13]<<endl;
  //  cout<<"14 : "<<g_DXL_ID_position[14]<<endl;
  //  cout<<"15 : "<<g_DXL_ID_position[15]<<endl;
  //  cout<<"16 : "<<g_DXL_ID_position[16]<<endl;
  //  cout<<"17 : "<<g_DXL_ID_position[17]<<endl;
  //  cout<<"18 : "<<g_DXL_ID_position[18]<<endl;
  //  cout<<"19 : "<<g_DXL_ID_position[19]<<endl;
  //  cout<<"20 : "<<g_DXL_ID_position[20]<<endl;
  //  cout<<"21 : "<<g_DXL_ID_position[21]<<endl;
  //  cout<<"22 : "<<g_DXL_ID_position[22]<<endl;
  // cout<<endl<<"Imu_Balance.Pitch_ADD_Angle:"<<Imu_Balance.Pitch_ADD_Angle<<endl;

  if (body == All)
    motor_packet(All, 23);
  else if (body == Leg)
    motor_packet(Leg, 23);
}
void IK_Solve::solve_kick(double pX_r, double pY_r, double pZ_r, double Yaw_R_deg,
                          double pX_l, double pY_l, double pZ_l, double Yaw_L_deg,
                          int body,
                          double Imu_pitch, double Imu_pos, double Imu_neg, double Imu_roll,
                          double Now_X, double roll_deg, bool left_kick, bool waist_turn)
{
  const double C2p = 45, L_1 = 140, L_2 = 140;
  double Yaw_L_rad = Yaw_L_deg * deg2rad, Yaw_R_rad = Yaw_R_deg * deg2rad;
  double left_roll_deg = 0.0, right_roll_deg = 0.0;
  left_roll_deg = left_kick ? 8.0: roll_deg;
  right_roll_deg = left_kick ? -roll_deg : -8.0;
  double L_temp[4] = {0, 0, 0, 0};
  double L_Valid = (pX_l*pX_l + (pY_l-C2p)*(pY_l-C2p) + pZ_l*pZ_l - L_1*L_1 - L_2*L_2) / (2*L_1*L_2);

  InvLegAngle[0] = Yaw_L_rad;
  InvLegAngle[3] = (double)acos(L_Valid);
  InvLegAngle[1] = (double)atan2(pZ_l, -pX_l*sin(Yaw_L_rad) + (pY_l-C2p)*cos(Yaw_L_rad));

  L_temp[0] = (double)(-L_2 * sin(InvLegAngle[3]));
  L_temp[1] = (double)(L_1 + L_2 * cos(InvLegAngle[3]));
  L_temp[2] = (double)(pX_l*cos(Yaw_L_rad) + (pY_l-C2p)*sin(Yaw_L_rad));
  L_temp[3] = (double)(-pX_l*sin(Yaw_L_rad)*cos(InvLegAngle[1]) + (pY_l-C2p)*cos(Yaw_L_rad)*cos(InvLegAngle[1]) + pZ_l*sin(InvLegAngle[1]));

  L_Valid = (L_temp[0]*L_temp[3] - L_temp[1]*L_temp[2]) / (L_temp[0]*L_temp[0] + L_temp[1]*L_temp[1]);

  InvLegAngle[2] = (double)asin(L_Valid);
  InvLegAngle[1] = (double)(InvLegAngle[1] + (PI / 2.0));
  InvLegAngle[4] = -(InvLegAngle[3] + InvLegAngle[2]);
  InvLegAngle[5] = (1.0 * InvLegAngle[1]) + (left_roll_deg * deg2rad);

  double R_temp[4] = {0, 0, 0, 0};
  double R_Valid = (pX_r*pX_r + (pY_r+C2p)*(pY_r+C2p) + pZ_r*pZ_r - L_1*L_1 - L_2*L_2) / (2*L_1*L_2);

  InvLegAngle[6] = Yaw_R_rad;
  InvLegAngle[9] = (double)acos(R_Valid);
  InvLegAngle[7] = (double)atan2(pZ_r, -pX_r*sin(Yaw_R_rad) + (C2p+pY_r)*cos(Yaw_R_rad));

  R_temp[0] = (double)(-L_2 * sin(InvLegAngle[9]));
  R_temp[1] = (double)(L_1 + L_2 * cos(InvLegAngle[9]));
  R_temp[2] = (double)(pX_r*cos(Yaw_R_rad) + (pY_r+C2p)*sin(Yaw_R_rad));
  R_temp[3] = (double)(-pX_r*sin(Yaw_R_rad)*cos(InvLegAngle[7]) + (pY_r+C2p)*cos(Yaw_R_rad)*cos(InvLegAngle[7]) + pZ_r*sin(InvLegAngle[7]));

  R_Valid = (R_temp[0]*R_temp[3] - R_temp[1]*R_temp[2]) / (R_temp[0]*R_temp[0] + R_temp[1]*R_temp[1]);

  InvLegAngle[8] = (double)asin(R_Valid);
  InvLegAngle[7] = (double)(InvLegAngle[7] + (PI / 2.0));
  InvLegAngle[10] = -(InvLegAngle[9] + InvLegAngle[8]);
  InvLegAngle[11] = (1.0 * InvLegAngle[7]) + (right_roll_deg * deg2rad);

  for (int i = 0; i < 23; i++)
    g_DXL_ID_position[i] = g_DXL_ID_Save_position[i];

  Compensate_Angle_Data.Compensate_Angle_Left_10 = (Imu_Balance.Roll_ADD_Angle)
      + Compensate_Angle_Data.L_Vertical_Angle + Compensate_Angle_Data.R_Vertical_Angle;
  if (Imu_pitch <= (Imu_pos + Imu_neg) / 2)
  {
    Compensate_Angle_Data.Compensate_Angle_Left_14 = (Imu_Balance.Pitch_ADD_Angle * 4.0);
    Compensate_Angle_Data.Compensate_Angle_Left_16 = -(Imu_Balance.Pitch_ADD_Angle * 2.0);
  }
  else
  {
    Compensate_Angle_Data.Compensate_Angle_Left_14 = (Imu_Balance.Pitch_ADD_Angle * 2.0);
    Compensate_Angle_Data.Compensate_Angle_Left_16 = -(Imu_Balance.Pitch_ADD_Angle * 0.0);
    Compensate_Angle_Data.Compensate_Angle_Left_18 = -(Imu_Balance.Pitch_ADD_Angle * 1.0);
  }
  Compensate_Angle_Data.Compensate_Angle_Left_18 =
      Now_Balance_Theta.Left_Theta3 + (Imu_Balance.Pitch_ADD_Angle * 1.0) + (Imu_Balance.Pitch_ADD_Angle * 0.0);
  Compensate_Angle_Data.Compensate_Angle_Left_20 =
      -Compensate_Angle_Data.L_Vertical_Angle + (Imu_Balance.Roll_ADD_Angle * 1);

  Compensate_Angle_Data.Compensate_Angle_Right_11 = (Imu_Balance.Roll_ADD_Angle)
      + Compensate_Angle_Data.R_Vertical_Angle + Compensate_Angle_Data.L_Vertical_Angle;
  if (Imu_pitch <= (Imu_pos + Imu_neg) / 2)
  {
    Compensate_Angle_Data.Compensate_Angle_Right_15 = (Imu_Balance.Pitch_ADD_Angle * 4.0);
    Compensate_Angle_Data.Compensate_Angle_Right_17 = -(Imu_Balance.Pitch_ADD_Angle * 2.0);
  }
  else
  {
    Compensate_Angle_Data.Compensate_Angle_Right_15 = (Imu_Balance.Pitch_ADD_Angle * 2.0);
    Compensate_Angle_Data.Compensate_Angle_Right_17 = -(Imu_Balance.Pitch_ADD_Angle * 0.0);
  }
  Compensate_Angle_Data.Compensate_Angle_Right_19 =
      Now_Balance_Theta.Right_Theta3 + (Imu_Balance.Pitch_ADD_Angle * 1) + (Imu_Balance.Pitch_ADD_Angle * 0.0);
  Compensate_Angle_Data.Compensate_Angle_Right_21 =
      -Compensate_Angle_Data.R_Vertical_Angle + (Imu_Balance.Roll_ADD_Angle * 1);

  if (abs(Imu_pitch) < 20)
  {
    if (pZ_l > pZ_r)
    {
      Compensate_Angle_Data.Compensate_Angle_Left_14 -= (Imu_Balance.Pitch_ADD_Angle * 2.5);
      Compensate_Angle_Data.Compensate_Angle_Right_15 -= (Imu_Balance.Pitch_ADD_Angle * 0.0);
    }
    else
    {
      Compensate_Angle_Data.Compensate_Angle_Left_14 -= (Imu_Balance.Pitch_ADD_Angle * 0.0);
      Compensate_Angle_Data.Compensate_Angle_Right_15 -= (Imu_Balance.Pitch_ADD_Angle * 2.5);
    }
  }
  if (abs(Imu_pitch) > 25)
  {
    Compensate_Angle_Data.Compensate_Angle_Left_14 = 0;
    Compensate_Angle_Data.Compensate_Angle_Right_15 = 0;
    Compensate_Angle_Data.Compensate_Angle_Left_16 = 0;
    Compensate_Angle_Data.Compensate_Angle_Right_17 = 0;
    Compensate_Angle_Data.Compensate_Angle_Left_18 = 0;
    Compensate_Angle_Data.Compensate_Angle_Right_19 = 0;
  }

  int Run_Add = 0;

  g_DXL_ID_position[0] += ang2pos(1.5);
  g_DXL_ID_position[1] -= ang2pos(1.5);

  g_DXL_ID_position[12] -= ang2pos(InvLegAngle[0] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_12) + Now_Motor_Angle.Motor_Angle_12 + Compensate_Angle_Data.Compensate_Angle_Left_12;
  g_DXL_ID_position[14] += ang2pos(InvLegAngle[2] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_14 + Compensate_Angle_Data.Compensate_Angle_Left_14) + Now_Motor_Angle.Motor_Angle_14 + Run_Add;
  g_DXL_ID_position[16] += ang2pos(InvLegAngle[3] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_16 + Compensate_Angle_Data.Compensate_Angle_Left_16) + Now_Motor_Angle.Motor_Angle_16 - Run_Add;
  g_DXL_ID_position[18] -= ang2pos(InvLegAngle[4] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_18 + Compensate_Angle_Data.Compensate_Angle_Left_18) + Now_Motor_Angle.Motor_Angle_18;
  g_DXL_ID_position[10] += 2*(ang2pos(InvLegAngle[1] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_10 + Compensate_Angle_Data.Compensate_Angle_Left_10) + Now_Motor_Angle.Motor_Angle_10);
  g_DXL_ID_position[20] -= 2*(ang2pos(-InvLegAngle[5] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_20 + Compensate_Angle_Data.Compensate_Angle_Left_20) + Now_Motor_Angle.Motor_Angle_20);

  g_DXL_ID_position[13] -= ang2pos(InvLegAngle[6] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_13) + Now_Motor_Angle.Motor_Angle_13 + Compensate_Angle_Data.Compensate_Angle_Right_13;
  g_DXL_ID_position[15] -= ang2pos(InvLegAngle[8] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_15 + Compensate_Angle_Data.Compensate_Angle_Right_15) + Now_Motor_Angle.Motor_Angle_15 + Run_Add;
  g_DXL_ID_position[17] -= ang2pos(InvLegAngle[9] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_17 + Compensate_Angle_Data.Compensate_Angle_Right_17) + Now_Motor_Angle.Motor_Angle_17 - Run_Add;
  g_DXL_ID_position[19] += ang2pos(InvLegAngle[10] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_19 + Compensate_Angle_Data.Compensate_Angle_Right_19) + Now_Motor_Angle.Motor_Angle_19;
  g_DXL_ID_position[11] += 2*(ang2pos(InvLegAngle[7] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_11 + Compensate_Angle_Data.Compensate_Angle_Right_11) + Now_Motor_Angle.Motor_Angle_11);
  g_DXL_ID_position[21] -= 2*(ang2pos(-InvLegAngle[11] * rad2deg * Now_Percentage_of_IK_Motor.Motor_Multiple_21 + Compensate_Angle_Data.Compensate_Angle_Right_21) + Now_Motor_Angle.Motor_Angle_21);
  //   if(waist_turn){
//   if (left_kick){
//     g_DXL_ID_position[22]  = 2048 +512*0.5;
//   }
//   else{
//     g_DXL_ID_position[22]  = 2048 -512*0.5;
//   }
// }else {g_DXL_ID_position[22]  = 2048;}

  if (body == All)
    motor_packet(All, 24);
  else if (body == Leg)
    motor_packet(Leg, 24);
}
void IK_Solve::motor_packet(int body, int limit)
{
  // motion_operator가 전신 모션을 재생하는 동안에는 IK의 주기 명령이
  // 같은 /dynamixel_control 토픽을 덮어쓰지 않도록 최종 publish만 막는다.
  if (!motor_publish_enabled_.load(std::memory_order_acquire))
    return;

  dynamixel_hardware_msgs::msg::DynamixelControlMsgs DXL;
  dynamixel_hardware_msgs::msg::DynamixelMsgs Motor_Info;

  DXL.motor_control.clear();

  if (Motor_Pub == nullptr)
  {
    RCLCPP_ERROR(this->get_logger(), "Motor_Pub is not initialized.");
    return;
  }
  // 기존 전체 모터 명령 생성 코드 (복원용으로 보관)
  // //body=Leg;
  //
  // if (body == Leg)
  // {
  //   // 0~5번 모터만 사용 (6,7,8,9번 제외)
  //   for(int i=0; i< 10; i++) // 6개
  //   {
  //     double radpos = ((static_cast<double>(g_DXL_ID_position[i])) / 4096) * (2 * M_PI) - M_PI;
  //
  //     Motor_Info.goal_position = float(radpos);
  //     Motor_Info.profile_velocity = vel;
  //     DXL.motor_control.push_back(Motor_Info);
  //   }
  //
  // }
  // // 10~21번 모터 (12개)
  // for(int i = body ; i < 25; i++) // 하체 12개
  // {
  //   double radpos = ((static_cast<double>(g_DXL_ID_position[i])) / 4096) * (2 * M_PI) - M_PI;
  //
  //   Motor_Info.goal_position = float(radpos);
  //   Motor_Info.profile_velocity = vel;
  //   DXL.motor_control.push_back(Motor_Info);
  // }
  // // 22번 모터 제외 (사용하지 않음)

  // 드라이버 설정 순서: 하체 10~21번 + 허리 22번, 총 13개.
  // 하체 테스트에서는 body와 limit에 관계없이 이 모터들만 제어한다.
  (void)body;
  (void)limit;
  for (int id = 10; id <= 22; ++id)
  {
    double radpos =
        (static_cast<double>(g_DXL_ID_position[id]) / 4096.0)
        * (2.0 * M_PI) - M_PI;

    Motor_Info.goal_position = static_cast<float>(radpos);
    Motor_Info.profile_velocity = vel;
    DXL.motor_control.push_back(Motor_Info);
  }

  Motor_Pub->publish(DXL);
  auto t2r = [](unsigned int t) {
    return (static_cast<double>(t) / 4096.0) * (2.0 * M_PI) - M_PI;
  };



}
