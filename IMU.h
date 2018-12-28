//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date         Author          Notes
// 29/09/2011   SOH Madgwick    Initial release
// 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

#include "structures.h"

void MahonyAHRSupdate(float dT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float dT, float gx, float gy, float gz, float ax, float ay, float az);

void IMU_GetOrientation(float q[4]);
bool IMU_Q2PRY(float orient[3]);
void IMU_PRY2Q(float pitch, float roll, float yaw);
void IMU_Qrefine(float Q[4], float dT);
bool IMU_Q2PRY_fast(float orient[3]);

void MadgwickAHRSupdate(float dT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float dT, float gx, float gy, float gz, float ax, float ay, float az);

/*
void LP5_Init(T_LP5 *lp, unsigned char set);
void LP3_Init(T_LP3 *lp, unsigned char set);
float LP5B_1200(T_LP5 *lp, float in);
float LP3B_1200(T_LP3 *lp, float in);
*/
float LP4_1000(T_LP4 *lp, float in);
void LP4_Init(T_LP4 *lp, byte freq);

float LP_RC(float new_value, float prev_out_value, float freq, float dT);
float LP_16_Wrap180(float new_value, float prev_value);
float LP_Wrap180(float new_value, float prev_value, float n_avgs);
void  wrap180(float* value);

void Rotate(float x, float y, float a, float *x1, float *y1);
void Rotate2D(float *x, float *y, float a);
void Ground2Plane(float ground[3], float PRY[3], float plane[3]);
void Plane2Ground(float plane[3], float PRY[3], float ground[3]);
float Plane2GroundU(float plane[3], float PRY[3]);

float WrapPI(float delta);
float Wrap180(float delta);

// matrix math
void print_matrix3x3( double m[3][3] );
void inverse_matrix3x3( double m[3][3] );
void add_matrix3x3( double m1[3][3], double m2[3][3], double m[3][3]);
void subtract_matrix3x3( double m1[3][3], double m2[3][3], double m[3][3]);
void scalar_mutliply3x3(double m[3][3], double scalar);
void multiply_matrix3x3( double m1[3][3], double m2[3][3], double m[3][3]);
void integrate_matrix3x3(float data[3], float data_int[3], float dT);

#endif
