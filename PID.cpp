#include "PID.h"
#include "IMU.h"
#include <math.h>
#include "defines.h"

#define max(a,b) ((a)>(b) ? (a) : (b))
#define min(a,b) ((a)<(b) ? (a) : (b))

/* Kc, Ti, Td, COofs, COmax, COmin */
void PID_Init(T_PID *pid, const float params[6], char clip360, char decay)
{
  pid->Ie = 0;
  pid->Kp    = params[0];
  pid->Ki    = params[1];
  pid->Kd    = params[2];
  pid->COofs = params[3];
  pid->COmax = params[4];
  pid->COmin = params[5];
  pid->COlast = pid->COofs;
  pid->PVlast = 0;
  pid->clip360 = clip360;
  pid->decay   = decay;
}

/* Kc, COmax, COmin, Acceleration */
void PID_P_Acc_Init(T_PID *pid, const float params[5], char clip360, char decel)
{
  pid->Ie = 0;
  pid->Kp    = params[0];
  pid->Kd    = params[1];
  pid->COmax = params[2];
  pid->COmin = params[3];
  pid->acceleration = params[4];
  pid->COlast = 0;
  pid->PVlast = 0;
  pid->clip360 = clip360;
  pid->decel_enabled = decel;
  pid->COofs = 0;
}

/* diff is dPV/dT
 * SP = Set Point
 * PV = ___ Variable
 * CO = Correcting Offset*/
float PID(T_PID *pid, float SP, float PV, float dT)
{
  float CO;
  float e = SP-PV;

  /* clip e to stay within +/- 180 degrees */
  if (pid->clip360)
      e = Wrap180(e);
  
  if (pid->Ki!=0)
  {
      /* PI */
      pid->Ie += e*dT;
      if (pid->decay)
        pid->Ie = 0.9998f*pid->Ie;
      
      CO = pid->COofs + pid->Kp * e + pid->Ie*pid->Ki;
    
      /* add D, if non-zero */
      if (pid->Kd && dT)
      {
          float dPV = PV - pid->PVlast;
          if (pid->clip360)
              dPV = Wrap180(dPV);
          CO -= pid->Kd*dPV/dT;
      }

      /* clip CO and Anti-Windup Protection */
      if (CO > pid->COmax)
      {
        CO = pid->COmax;
        pid->Ie = (CO - pid->COofs - pid->Kp*e)/pid->Ki;
      }
      else
      if (CO < pid->COmin)
      {
        CO = pid->COmin;
        pid->Ie = (CO - pid->COofs - pid->Kp*e)/pid->Ki;
      }
  }
  else
  {
      /* P only */
      CO = pid->COofs + pid->Kp * e;

      /* add D, if non-zero */
      if (pid->Kd && dT)
      {
          float dPV = PV - pid->PVlast;
          if (pid->clip360)
              dPV = Wrap180(dPV);
          CO -= pid->Kd*dPV/dT;
      }
    
      /* clip CO and Anti-Windup Protection */
      if (CO > pid->COmax)
        CO = pid->COmax;
      else
      if (CO < pid->COmin)
        CO = pid->COmin;
  }
  
  pid->COlast = CO;
  pid->PVlast = PV;
  return CO;
}

float PIDestOutput(T_PID *pid, float Ie)
{
    float CO = Ie*pid->Ki;
    return CO;
}

float PID_P_Acc(T_PID *pid, float SP, float PV, float dT, bool ignore_acc_max, bool double_acc)
{
  float CO;
  float k = pid->Kp;
  float accel = pid->acceleration;
  float e = SP-PV;

  if (pid->clip360)
      e = Wrap180(e);

  /* intended for angle control in RCradio/joystick modes */
  if (double_acc)
      accel *= 3;

  /* set acc/dec really high, intended for altitude control*/
  if (ignore_acc_max)
	  accel = max(10, accel);   // 1G
  
  /* limit k to stay within the max deceleration */
  if (pid->decel_enabled && e!=0)
    k = min(k, sqrtf(0.75f*accel/ABS(e)));
  
  CO = k * e + pid->COofs;

  /* add D, if non-zero */
  if (pid->Kd && dT)
  {
      float dPV = PV - pid->PVlast;
      if (pid->clip360)
          dPV = Wrap180(dPV);
      CO -= pid->Kd*dPV/dT;
  }

  /* clip CO to be within limits */
  if (CO > pid->COmax)
    CO = pid->COmax;
  else
  if (CO < pid->COmin)
    CO = pid->COmin;

  /* acceleration - make sure CO changes only within the speed limits */
  CO = ClipMinMax(CO, pid->COlast - accel * dT, pid->COlast + accel * dT);
  
  pid->COlast = CO;
  pid->PVlast = PV;
  return CO;
}

void PID_SetForEnable(T_PID *pid, float SP, float PV, float CO)
{
  float e = SP-PV;

  /* clip CO and Anti-Windup Protection */
/*  if (CO > pid->COmax)
    CO = pid->COmax;
  else
  if (CO < pid->COmin)
    CO = pid->COmin;*/
    
  if (pid->Ki)
    pid->Ie = (CO - pid->COofs - pid->Kp*e)/pid->Ki;
  pid->COlast = CO;
  pid->PVlast = PV;
}

//mmri
/*PID control function for Temperature Chamber
 * - Temperature chamber used to steadily increase temp of IMU
 *   to get characteristic curve for gyro drift vs temp
 * - want to control the change in temperature over time to be
 *   at approximatley 0.2 degC/min = 3333 degC/us
 * - PVlast = previous gyro temperature
 * Inputs:
 * 1) T_PID *pid: pointer to T_PID object
 *        - pid->Kp,Ki or Kd = P, I and D coeffs
 *        - pid->PVlast = previous gyro temperature
 *        - pid->COlast = previous error recorded by PID
 *        - pid->decay = flag to turn the PID on or off
 *        - pid->COmax,COmin = max and min of throttle_width
 *        - pid->acceleration = temperature rate of change set point
 * 2) float temp: the current temperature reported (by IMU)
 * 3) int throttle_width: the current pulse width of the PWM signal
 *                        in micro-seconds
 *        - this PWM signal is applied to an NFET gate in order to control
 *          the current through three resistor heating elements
 *        - hfc.throttle_width refers to the PwmOut object "servoTpwm"
 *        	which is the s1 servo output pin
 *        - PWM period = servoTpwm->period_us = 20000 = 20 milli-s
 *        - pulse width range is 0 to 10 milli-s
 * Outputs:
 * 1) int out = the new throttle_width to pass to the PWM signal
*/
int PID_temp(T_PID *pid, float temp, float dT, int throttle_width)
{
	float error, Pout, Iout, Dout;

	int out = 0;
	float rate_setpoint = 0;
	float max = pid->COmax;
	float min = pid->COmin;

	float prev_error = pid->COlast;
	float prev_temp  = pid->PVlast;

	/*"acceleration" property of PID used to store dynamic set point
	 * - min change in temperature is 0 degC/min = 0 degC/s
	 * - max change in temperature is 5 degC/min = 0.083333 degC/s*/
	rate_setpoint = pid->acceleration;

	//  calculate error, difference in the CHANGE of temperature
	error = ( rate_setpoint - ((temp - prev_temp)/dT) ) / rate_setpoint;

	Pout = pid->Kp * error;
	Iout = pid->Ki * (pid->Ie + (error * dT) );
	Dout = pid->Kd * ( (error - prev_error) / dT );

	out = Pout + Iout + Dout;

//	debug_print(" Pout=%f Iout=%f Dout=%f out=%d\r\n",Pout,Iout,Dout,out);

	out += throttle_width;

    if( out > max )
        out = max;
    else if( out < min )
        out = min;

    pid->COlast = error;
    pid->PVlast = temp;

    return out;

}
