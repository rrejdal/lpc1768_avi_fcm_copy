/*****************************************************************************
 * File name: IMU_calib.h
 * Purpose: Headers file for Helper Functions for calibrating accelerometer
 * 			and gyroscope of IMU.
 *
 * 	@author: Mark Malak Ramzy Ibrahim
 * 	@Date:  1-March-2018 (01/03/2018)
 * 	@version: 1.0
 ******************************************************************************/

#ifndef _IMU_CALIBRATION
#define _IMU_CALIBRATION


#include "MPU6050.h"
#include "NOKIA_5110.h"
#include "DigitalIn.h"
#include "PwmOut.h"
#include "HMC5883L.h"

#define GRAVITY_ACCEL 9.81202
#define MAX_LINES    200

void gyro_temp_calib(MPU6050* mpu,  NokiaLcd* myLcd, DigitalIn* btnCancel, Serial* pc);
int imu_calib_helper(MPU6050* mpu, NokiaLcd* myLcd, DigitalIn* btnEnter, DigitalIn* btnCancel);
void compass_calib_helper(HMC5883L* compass, NokiaLcd* myLcd, DigitalIn* btnEnter, DigitalIn* btnCancel);

#endif //_IMU_CALIBRATION
