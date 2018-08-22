#ifndef _PID_H_
#define _PID_H_

typedef struct
{
  float Ie;
  float Kp;				// coefficient for proportional gain feedback
  float Ki;				// coefficient for integral gain feedback
  float Kd;				// coefficient for derivative gain feedback
  float COmax;
  float COmin;
  float COofs;
  float COlast;
  float PVlast;
  float acceleration;
  char  decel_enabled;
  char  clip360;
  char  decay;
} T_PID;

void  PID_Init(T_PID *pid, float params[8], char clip360, char decay);
void  PID_P_Acc_Init(T_PID *pid, float params[5], char clip360, char decel);
float PID(T_PID *pid, float SP, float PV, float dT);
void  PID_SetForEnable(T_PID *pid, float SP, float PV, float CO);
float PID_P_Acc(T_PID *pid, float SP, float PV, float dT, bool ignore_acc_max, bool double_acc);
float PIDestOutput(T_PID *pid, float Ie);

//mmri
int PID_temp(T_PID *pid, float temp, float dT, int throttle_width);

#endif
