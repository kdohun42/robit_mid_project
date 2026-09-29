#ifndef PID_CONTROL_FLOAT_HPP
#define PID_CONTROL_FLOAT_HPP

typedef struct PID_struct
{
  double nowValue;
  double pastValue;

  double nowError;
  double pastError;
  double target;

  double errorSum;
  double errorSumLimit;
  double errorDiff;

  double nowOutput;
  double pastOutput;
  double outputLimit;

  double underOfPoint;

  double kP;
  double kI;
  double kD;
} PID;

void PID_Control_init(PID* temp, double TP, double TI, double TD, double ELIMIT, double OLIMIT);
void PID_Control_Float(PID* dst, double target, double input);

#endif