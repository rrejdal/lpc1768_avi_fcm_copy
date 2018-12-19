//=====================================================================================================
// MahonyAHRS.c
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

//---------------------------------------------------------------------------------------------------
// Header files

#include "IMU.h"
#include "mymath.h"
#include "utils.h"
#include <math.h>
#include <stdio.h>
#include "defines.h"

#define PITCH   0
#define ROLL    1
#define YAW     2

#define X       0
#define Y       1
#define Z       2

#define PIhalf  1.570796326795f
//---------------------------------------------------------------------------------------------------
// Definitions

//static AHRS.MadgwickAHRS AHRS = new AHRS.MadgwickAHRS(1f / 256f, 0.1f);   // dT, Kp, Ki
//static AHRS.MahonyAHRS AHRS = new AHRS.MahonyAHRS(1f / 256f, 5f);   // dT, Kp, Ki

//#define twoKpDef    (5.0f)          // 2 * proportional gain    // ACC has a faster impact
#define twoKpDef    (2.0f * 0.5f)   // 2 * proportional gain    // ACC has a slower impact
#define twoKiDef    (2.0f * 0.0f)   // 2 * integral gain

#define betaDef     12.5f        // 2 * proportional gain   // 0.1 acc fast impact, 0.02 acc slower impact
static float beta = betaDef;                              // 2 * proportional gain (Kp)
//static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Variable definitions

//static float twoKp = twoKpDef;                                            // 2 * proportional gain (Kp)
//static float twoKi = twoKiDef;                                            // 2 * integral gain (Ki)
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                  // quaternion of sensor frame relative to auxiliary frame, xxx, W, Y, xxx
//static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

static float qDot1last=0.0f, qDot2last=0.0f, qDot3last=0.0f, qDot4last=0.0f;

float invSqrt(float x);

/* returns W, X, Y, Z */
void IMU_GetOrientation(float q[4])
{
  q[0] = q0; // x
  q[1] = q1; // w
  q[2] = q2; // y
  q[3] = q3; // z
}

bool IMU_Q2PRY(float orient[3])
{
  double test = q1*q2 + q3*q0;
  if (test > 0.499)
  { // singularity at north pole
    orient[YAW] = -2 * atan2(q1, q0);  // heading
    orient[PITCH] = PI/2;               // attitude
    orient[ROLL] = 0;                  // bank
    return false;
  }
  if (test < -0.499)
  { // singularity at south pole
    orient[YAW] = 2 * atan2(q1, q0);
    orient[PITCH] = - PI/2;
    orient[ROLL] = 0;
    return false;
  }
  double sqx = q1*q1;
  double sqy = q2*q2;
  double sqz = q3*q3;
  orient[YAW] = -atan2f(2*q2*q0-2*q1*q3 , 1 - 2*sqy - 2*sqz);
  orient[PITCH] = asinf(2*test);
  orient[ROLL] = atan2f(2*q1*q0-2*q2*q3 , 1 - 2*sqx - 2*sqz);
  return true;
}

bool IMU_Q2PRY_fast(float orient[3])
{   // 39us
  float test = q1*q2 + q3*q0;
  if (test > 0.499f)
  { // singularity at north pole
    orient[YAW] = -2 * ATAN2fR(q1, q0);  // heading
    orient[PITCH] = PI/2;               // attitude
    orient[ROLL] = 0;                  // bank
    return false;
  }
  if (test < -0.499f)
  { // singularity at south pole
    orient[YAW] = 2 * ATAN2fR(q1, q0);
    orient[PITCH] = - PI/2;
    orient[ROLL] = 0;
    return false;
  }
  float sqx = q1*q1;
  float sqy = q2*q2;
  float sqz = q3*q3;
  orient[YAW] = -ATAN2fR(2*q2*q0-2*q1*q3 , 1 - 2*sqy - 2*sqz);
  orient[PITCH] = ASINfR(2*test);    // 4us
  orient[ROLL] = ATAN2fR(2*q1*q0-2*q2*q3 , 1 - 2*sqx - 2*sqz);
  return true;
}

void IMU_PRY2Q(float pitch, float roll, float yaw)
{
    // Assuming the angles are in radians.
    float c1 = cosf(-yaw/2);
    float s1 = sinf(-yaw/2);
    float c2 = cosf(pitch/2);
    float s2 = sinf(pitch/2);
    float c3 = cosf(roll/2);
    float s3 = sinf(roll/2);

/*    float c1 = COSfR(-yaw/2);
    float s1 = SINfR(-yaw/2);
    float c2 = COSfR(pitch/2);
    float s2 = SINfR(pitch/2);
    float c3 = COSfR(roll/2);
    float s3 = SINfR(roll/2);*/
    float c1c2 = c1*c2;
    float s1s2 = s1*s2;
    q0 =c1c2*c3 - s1s2*s3;
    q1 =c1c2*s3 + s1s2*c3;
    q2 =s1*c2*c3 + c1*s2*s3;
    q3 =c1*s2*c3 - s1*c2*s3;
}

void IMU_Qrefine(float Q[4], float dT)
{
    float gain = 0.5f;
    float dQ = Q[0] - q0;
    q0 += dQ*dT*gain;
    dQ = Q[1] - q1;
    q1 += dQ*dT*gain;
    dQ = Q[2] - q2;
    q2 += dQ*dT*gain;
    dQ = Q[3] - q3;
    q3 += dQ*dT*gain;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
    return 1/sqrtf(x);
}


#if 0
// AHRS algorithm update
// dT in seconds
// Gyroscope measurements in radians/s.
// Accelerometer measurements in any calibrated units.
// Magnetometer measurements in any calibrated units.
void MahonyAHRSupdate(float dT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        MahonyAHRSupdateIMU(dT, gx, gy, gz, ax, ay, az);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;     

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
    
        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * dT;    // integral error scaled by Ki
            integralFBy += twoKi * halfey * dT;
            integralFBz += twoKi * halfez * dT;
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    
    // Integrate rate of change of quaternion
    gx *= (0.5f * dT);     // pre-multiply common factors
    gy *= (0.5f * dT);
    gz *= (0.5f * dT);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx); 
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update
// dT in seconds
// Gyroscope measurements in radians/s.
// Accelerometer measurements in any calibrated units.
// Magnetometer measurements in any calibrated units.
void MahonyAHRSupdateIMU(float dT, float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;        

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;
    
        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dT;    // integral error scaled by Ki
            integralFBy += twoKi * halfey * dT;
            integralFBz += twoKi * halfez * dT;
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    
    // Integrate rate of change of quaternion
    gx *= (0.5f * dT);     // pre-multiply common factors
    gy *= (0.5f * dT);
    gz *= (0.5f * dT);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx); 
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}


// AHRS algorithm update
void MadgwickAHRSupdate(float dT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MadgwickAHRSupdateIMU(dT, gx, gy, gz, ax, ay, az);
        return;
    }

    /* my heading goes clock-wise while this need counter-clockwise */
    gy = -gy;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;   

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dT;
    q1 += qDot2 * dT;
    q2 += qDot3 * dT;
    q3 += qDot4 * dT;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
#endif
//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float dT, float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    /* my heading goes clock-wise while this need counter-clockwise */
    gy = -gy;
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
//    q0 += (qDot1) * dT;
//    q1 += (qDot2) * dT;
//    q2 += (qDot3) * dT;
//    q3 += (qDot4) * dT;

    q0 += (qDot1+qDot1last) * dT*0.5f;
    q1 += (qDot2+qDot2last) * dT*0.5f;
    q2 += (qDot3+qDot3last) * dT*0.5f;
    q3 += (qDot4+qDot4last) * dT*0.5f;

    qDot1last = qDot1;
    qDot2last = qDot2;
    qDot3last = qDot3;
    qDot4last = qDot4;
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

#if 0
/* lp with cut-off 16Hz, 5th order */
//#define GAIN   1.845524557e+07f
float LP5_16(T_LP5 *lp, float in)
{
    lp->xv[0] = lp->xv[1]; lp->xv[1] = lp->xv[2]; lp->xv[2] = lp->xv[3]; lp->xv[3] = lp->xv[4]; lp->xv[4] = lp->xv[5]; 
    lp->xv[5] = in * 5.418513648e-8f;
    lp->yv[0] = lp->yv[1]; lp->yv[1] = lp->yv[2]; lp->yv[2] = lp->yv[3]; lp->yv[3] = lp->yv[4]; lp->yv[4] = lp->yv[5]; 
    lp->yv[5] =   (lp->xv[0] + lp->xv[5]) + 5 * (lp->xv[1] + lp->xv[4]) + 10 * (lp->xv[2] + lp->xv[3])
                 + (  0.8888177640f * lp->yv[0]) + ( -4.5374298438f * lp->yv[1])
                 + (  9.2782000853f * lp->yv[2]) + ( -9.4993118169f * lp->yv[3])
                 + (  4.8697220775f * lp->yv[4]);
    return lp->yv[5];
}

/* lp with cut-off 13Hz, 5th order */
//#define GAIN   5.156560349e+07
float LP5_13(T_LP5 *lp, float in)
{
    lp->xv[0] = lp->xv[1]; lp->xv[1] = lp->xv[2]; lp->xv[2] = lp->xv[3]; lp->xv[3] = lp->xv[4]; lp->xv[4] = lp->xv[5]; 
    lp->xv[5] = in * 1.93927721643e-8;
    lp->yv[0] = lp->yv[1]; lp->yv[1] = lp->yv[2]; lp->yv[2] = lp->yv[3]; lp->yv[3] = lp->yv[4]; lp->yv[4] = lp->yv[5]; 
    lp->yv[5] = (lp->xv[0] + lp->xv[5]) + 5 * (lp->xv[1] + lp->xv[4]) + 10 * (lp->xv[2] + lp->xv[3])
                     + (  0.9086765286 * lp->yv[0]) + ( -4.6227208166 * lp->yv[1])
                     + (  9.4154545703 * lp->yv[2]) + ( -9.5974218007 * lp->yv[3])
                     + (  4.8960108978 * lp->yv[4]);
                 
    return lp->yv[5];
}

//#define GAIN   1.559050074e+08
float LP5_13_1250(T_LP5 *lp, float in)
{
    lp->xv[0] = lp->xv[1]; lp->xv[1] = lp->xv[2]; lp->xv[2] = lp->xv[3]; lp->xv[3] = lp->xv[4]; lp->xv[4] = lp->xv[5]; 
    lp->xv[5] = in * 6.414162166288444690455785835138e-9;
    lp->yv[0] = lp->yv[1]; lp->yv[1] = lp->yv[2]; lp->yv[2] = lp->yv[3]; lp->yv[3] = lp->yv[4]; lp->yv[4] = lp->yv[5]; 
    lp->yv[5] =   (lp->xv[0] + lp->xv[5]) + 5 * (lp->xv[1] + lp->xv[4]) + 10 * (lp->xv[2] + lp->xv[3])
            + (  0.9262471227 * lp->yv[0]) + ( -4.6972010571 * lp->yv[1])
            + (  9.5337818609 * lp->yv[2]) + ( -9.6809361436 * lp->yv[3])
            + (  4.9181080117 * lp->yv[4]);
    return lp->yv[5];
}

//1200Hz cut-off 33Hz, 5th order, Butterworth
//#define GAIN   2.715068281e+05
float LP5B_33_1250(T_LP5 *lp, float in)
{
    lp->xv[0] = lp->xv[1]; lp->xv[1] = lp->xv[2]; lp->xv[2] = lp->xv[3]; lp->xv[3] = lp->xv[4]; lp->xv[4] = lp->xv[5]; 
    lp->xv[5] = in * 3.6831486228099027318716630095683e-6;
    lp->yv[0] = lp->yv[1]; lp->yv[1] = lp->yv[2]; lp->yv[2] = lp->yv[3]; lp->yv[3] = lp->yv[4]; lp->yv[4] = lp->yv[5]; 
    lp->yv[5] =   (lp->xv[0] + lp->xv[5]) + 5 * (lp->xv[1] + lp->xv[4]) + 10 * (lp->xv[2] + lp->xv[3])
            + (  0.5712008962 * lp->yv[0]) + ( -3.1752779647 * lp->yv[1])
            + (  7.0801448816 * lp->yv[2]) + ( -7.9172343737 * lp->yv[3])
            + (  4.4410486999 * lp->yv[4]);
    return lp->yv[5];
}
#endif
#if 0
#define LP_SETS 7
static const double LP5_Coeffs[LP_SETS][6] = {
{ 0,0,0,0,0,0},
{ 1/8.840143939e+07, 0.8441170973, -4.3636080283, 9.0254524705, -9.3365274216, 4.8305655200},   // 1200Hz cut-off 10Hz, 5th order, Butterworth
{ 1/2.440306502e+07, 0.8022582284, -4.1879963546, 8.7490199167, -9.1430233323, 4.7797402305},   // 1200Hz cut-off 13Hz, 5th order, Butterworth
{ 1/1.243416665e+06, 0.6656525382, -3.5989027681, 7.7949183180, -8.4551152235, 4.5934213998},   // 1200Hz cut-off 24Hz, 5th order, Butterworth
{ 1/7.065810721e+05, 0.6325727690, -3.4521886184, 7.5502933844, -8.2733449729, 4.5426221495},   // 1200Hz cut-off 27Hz, 5th order, Butterworth
{ 1/4.271694293e+05, 0.6011158229, -3.3110475620, 7.3120812802, -8.0940554178, 4.4918309651},   // 1200Hz cut-off 30Hz, 5th order, Butterworth
{ 1/2.715068281e+05, 0.5712008962, -3.1752779647, 7.0801448816, -7.9172343737, 4.4410486999}    // 1200Hz cut-off 33Hz, 5th order, Butterworth
};

static const float LP3_Coeffs[LP_SETS][4] = {
{ 0,0,0,0},
{ 0,0,0,0},   // 1200Hz cut-off 10Hz, 3th order, Butterworth
{ 0,0,0,0},   // 1200Hz cut-off 13Hz, 3th order, Butterworth
{ 1/4.553605266e+03f, 0.7776385602f, -2.5282312191f, 2.7488358092f},   // 1200Hz cut-off 24Hz, 3th order, Butterworth
{ 1/3.245477799e+03f, 0.7535348707f, -2.4734881560f, 2.7174883169f},   // 1200Hz cut-off 27Hz, 3th order, Butterworth
{ 1/2.400694440e+03f, 0.7301653453f, -2.4196551110f, 2.6861573965f},   // 1200Hz cut-off 30Hz, 3th order, Butterworth
{ 1/1.829952363e+03f, 0.7075065802f, -2.3667229736f, 2.6548446949f}    // 1200Hz cut-off 33Hz, 3th order, Butterworth
};

void LP5_Init(T_LP5 *lp, unsigned char set)
{
    int i;
    for (i=0; i<=5; i++) lp->xv[i] = 0;
    for (i=0; i<=5; i++) lp->yv[i] = 0;
    if (set>=LP_SETS)
      set = 0;
    lp->set = set;
}

void LP3_Init(T_LP3 *lp, unsigned char set)
{
    int i;
    for (i=0; i<=3; i++) lp->xv[i] = 0;
    for (i=0; i<=3; i++) lp->yv[i] = 0;
    if (set>=LP_SETS)
      set = 0;
    lp->set = set;
}

void LP4_Init(T_LP4 *lp)
{
    lp->w1P1 = 0;
    lp->w2P1 = 0;
    lp->w1P2 = 0;
    lp->w2P2 = 0;

    lp->coeffsP1[0] = 32768;
    lp->coeffsP1[1] = 256;
    lp->coeffsP1[2] = 8373;
    lp->coeffsP1[3] = 16747;
    lp->coeffsP1[4] = 8373;
    lp->coeffsP1[5] = 27438;
    lp->coeffsP1[6] = -11548;
    lp->coeffsP2[0] = 16384;
    lp->coeffsP2[1] = 512;
    lp->coeffsP2[2] = 8192;
    lp->coeffsP2[3] = 16384;
    lp->coeffsP2[4] = 8192;
    lp->coeffsP2[5] = 30034;
    lp->coeffsP2[6] = -14192;
}
#endif


/* butterworth 4th order, 1000Hz samplig freq, b02  b1  a1  a2; pass1 and pass2 */
static const float BW4coeffs[][8] = {
{/* cutoff 6Hz  */   0.0004924668399218683, 0.0009849336798437365, 1.9313278156555946, -0.932701052631051,  0.000244140625,  0.00048828125, 1.9701624869780452, -0.9715633366695601},
{/* cutoff 8Hz  */   0.0007658247430796075, 0.001531649486159215,  1.9088649789119163, -0.9112790072360633, 0.00048828125,   0.0009765625,  1.9597916889966362, -0.9622701213105668},
{/* cutoff 10Hz */   0.0009200498139105926, 0.0018400996278211852, 1.8866095826215064, -0.8903397362840242, 0.0009765625,    0.001953125,   1.9492159580258417, -0.9530698953278909},
{/* cutoff 12Hz */   0.0018778130404873722, 0.0037556260809747445, 1.8645578256348248, -0.8698703136957515, 0.0009765625,    0.001953125,   1.938439716019559,  -0.9439627079625276},
{/* cutoff 14Hz */   0.001712258645830301,  0.003424517291660602,  1.8427059736363685, -0.8498582486553967, 0.001953125,     0.00390625,    1.9274673141069234, -0.9349485816112967},
{/* cutoff 15Hz */   0.002238816664483877,  0.004477633328967754,  1.8318538630923638, -0.8400199367024879, 0.001953125,     0.00390625,    1.921908893578823,  -0.9304764162470467},
{/* cutoff 16Hz */   0.0028756798906272277, 0.005751359781254455,  1.8210503582611857, -0.8302914688064607, 0.001953125,     0.00390625,    1.9163030327246364, -0.9260275129309736},
{/* cutoff 18Hz */   0.004535208424851733,  0.009070416849703467,  1.799587376177328,  -0.8111583041818541, 0.001953125,     0.00390625,    1.904951081805863,  -0.917199473915915},
{/* cutoff 20Hz */   0.003403194598080742,  0.006806389196161484,  1.7783134881394351, -0.7924474718329471, 0.00390625,      0.0078125,     1.8934156010225005, -0.9084644129492955},
{/* cutoff 22Hz */   0.004906702747473738,  0.009813405494947476,  1.7572252180181789, -0.7741480611243557, 0.00390625,      0.0078125,     1.8817006600777388, -0.899822255828111},
{/* cutoff 24Hz */   0.006844135425350161,  0.013688270850700322,  1.736319151809846,  -0.7562495196628964, 0.00390625,      0.0078125,     1.8698102590459458, -0.89127290676215},
{/* cutoff 26Hz */   0.009285038744371425,  0.01857007748874285,   1.7155919366299408, -0.7387416398307504, 0.00390625,      0.0078125,     1.857748328756995,  -0.8828162493471352},
{/* cutoff 28Hz */   0.0123021489050963,    0.0246042978101926,    1.6950402796943111, -0.721614545894391,  0.00390625,      0.0078125,     1.8455187312222892, -0.8744521475123016},
{/* cutoff 30Hz */   0.007985533894205376,  0.015971067788410752,  1.674660947290977,  -0.7048586816622794, 0.0078125,       0.015625,      1.8331252600998054, -0.8661804464426682},
{/* cutoff 32Hz */   0.01018498092187266,   0.02036996184374532,   1.654450763745519,  -0.6884647986656824, 0.0078125,       0.015625,      1.8205716411956112, -0.8580009734763099},
{/* cutoff 34Hz */   0.012789641406238745,  0.02557928281247749,   1.6344066103826151, -0.6724239448382732, 0.0078125,       0.015625,      1.8078615329993912, -0.849913538976941},
{/* cutoff 36Hz */   0.015840754457976405,  0.03168150891595281,   1.614525424486036,  -0.6567274536713773, 0.0078125,       0.015625,      1.794998527251614,  -0.8419179371821401},
{/* cutoff 38Hz */   0.019380452610979045,  0.03876090522195809,   1.5948041982592016, -0.6413669338229128, 0.0078125,       0.015625,      1.7819861495400882, -0.8340139470275797},
{/* cutoff 40Hz */   0.02345165099131006,   0.04690330198262012,   1.5752399777881514, -0.6263342591591413, 0.0078125,       0.015625,      1.7688278599237215, -0.8262013329476159},
{/* cutoff 42Hz */   0.014048972452727028,  0.028097944905454055,  1.5558298620086166, -0.6116215592094104, 0.015625,        0.03125,       1.7555270535814091, -0.8184798456526241},
{/* cutoff 44Hz */   0.016681757658131204,  0.03336351531626241,   1.536571001678679,  -0.5972212100150395, 0.015625,        0.03125,       1.7420870614840578, -0.8108492228834702},
{/* cutoff 46Hz */   0.01964652085381464,   0.03929304170762928,   1.5174605983583394, -0.5831258253544316, 0.015625,        0.03125,       1.7285111510878421, -0.8033091901435202},
{/* cutoff 48Hz */   0.02296581093478074,   0.04593162186956148,   1.4984959033971865, -0.569328248327397,  0.015625,        0.03125,       1.7148025270468696, -0.7958594614085953},
{/* cutoff 50Hz */   0.02666234908202244,   0.05332469816404488,   1.4796742169311927, -0.5558215432824883, 0.015625,        0.03125,       1.7009643319435255, -0.7884997398152978}};

static const byte BW4freq[] = {6, 8, 10, 12, 14, 15, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50};

void LP4_Init(T_LP4 *lp, byte freq)
{
    byte i;
    lp->w1P1 = 0;
    lp->w2P1 = 0;
    lp->w1P2 = 0;
    lp->w2P2 = 0;

    lp->coeffs = NULL;
    if (!freq)
        return;

    for (i=0; i<sizeof(BW4freq); i++)
        if (freq == BW4freq[i])
            lp->coeffs = BW4coeffs[i];
    if (lp->coeffs==NULL)
        debug_print("LP4_Init: frequency %d is not defined\n", freq);
}

#if 0
float LP4B_1000(T_LP4 *lp, float in)
{
    // Read input sample
    int x0 = in/**lp->coeffsP1[0]*/;
    int w0;
    int w1 = lp->w1P1;
    int w2 = lp->w2P1;
    long long accumulator;
    int acc;
    float output;

    // Run feedback part of filter
    acc  = w2 * lp->coeffsP1[6];    // a2;
    acc += w1 * lp->coeffsP1[5];    // a1;
    acc += x0 * lp->coeffsP1[1] ;
    w0 = (acc+8192)>>14 ;

    // Run feedforward part of filter
    acc  = w0 * lp->coeffsP1[2]; // b0;
    acc += w1 * lp->coeffsP1[3]; // b1;
    acc += w2 * lp->coeffsP1[4]; // b2;

    lp->w2P1 = w1;        // Shuffle history buffer
    lp->w1P1 = w0;

    // Write output
    x0 = acc*2;//(acc+16384)>>15 ;

    // pass 2 ====================================================
    w1 = lp->w1P2;
    w2 = lp->w2P2;

    // Run feedback part of filter
    accumulator  = (long long)w2 * lp->coeffsP2[6];    // a2;
    accumulator += (long long)w1 * lp->coeffsP2[5];    // a1;
    accumulator += (long long)x0 * lp->coeffsP2[1] ;
    w0 = (accumulator+8192)>>14 ;

    // Run feed forward part of filter
    accumulator  = (long long)w0 * lp->coeffsP2[2]; // b0;
    accumulator += (long long)w1 * lp->coeffsP2[3]; // b1;
    accumulator += (long long)w2 * lp->coeffsP2[4]; // b2;

    lp->w2P2 = w1;        // Shuffle history buffer
    lp->w1P2 = w0;

    // Write output
    x0 = (accumulator+16384)>>15 ;

    output = ((float)x0)/*/lp->coeffsP2[0]*/;

    return output;
}
#endif

float LP4_1000(T_LP4 *lp, float in)
{
    // Read input sample
    float x0 = in;
    float w0;
    float w1 = lp->w1P1;
    float w2 = lp->w2P1;
    float accumulator;

    if (!lp->coeffs)
        return in;

    // Run feedback part of filter
    accumulator  = w2 * lp->coeffs[3];    // a2;
    accumulator += w1 * lp->coeffs[2];    // a1;
    accumulator += x0;
    w0 = accumulator;

    // Run feed forward part of filter
    accumulator  = (w0+w2) * lp->coeffs[0]; // b0;
    accumulator += w1 * lp->coeffs[1]; // b1;

    lp->w2P1 = w1;        // Shuffle history buffer
    lp->w1P1 = w0;

    // Write output
    x0 = accumulator;

    // pass 2 ====================================================
    w1 = lp->w1P2;
    w2 = lp->w2P2;

    // Run feedback part of filter
    accumulator  = w2 * lp->coeffs[7];    // a2;
    accumulator += w1 * lp->coeffs[6];    // a1;
    accumulator += x0;
    w0 = accumulator;

    // Run feed forward part of filter
    accumulator  = (w0+w2) * lp->coeffs[4]; // b0;
    accumulator += w1 * lp->coeffs[5]; // b1;

    lp->w2P2 = w1;        // Shuffle history buffer
    lp->w1P2 = w0;

    // Write output
    return accumulator;
}

#if 0
float LP5B_1200(T_LP5 *lp, float in)
{
    const double *c = LP5_Coeffs[lp->set];
    
    lp->xv[0] = lp->xv[1]; lp->xv[1] = lp->xv[2]; lp->xv[2] = lp->xv[3]; lp->xv[3] = lp->xv[4]; lp->xv[4] = lp->xv[5]; 
    lp->xv[5] = in * c[0];
    lp->yv[0] = lp->yv[1]; lp->yv[1] = lp->yv[2]; lp->yv[2] = lp->yv[3]; lp->yv[3] = lp->yv[4]; lp->yv[4] = lp->yv[5]; 
    lp->yv[5] = (lp->xv[0] + lp->xv[5]) + 5 * (lp->xv[1] + lp->xv[4]) + 10 * (lp->xv[2] + lp->xv[3])
                + c[1] * lp->yv[0] + c[2] * lp->yv[1] + c[3] * lp->yv[2] + c[4] * lp->yv[3] + c[5] * lp->yv[4];
    return lp->yv[5];
}

float LP3B_1200(T_LP3 *lp, float in)
{
    const float *c = LP3_Coeffs[lp->set];
    
    lp->xv[0] = lp->xv[1]; lp->xv[1] = lp->xv[2]; lp->xv[2] = lp->xv[3]; 
    lp->xv[3] = in * c[0];
    lp->yv[0] = lp->yv[1]; lp->yv[1] = lp->yv[2]; lp->yv[2] = lp->yv[3]; 
    lp->yv[3] = (lp->xv[0] + lp->xv[3]) + 3 * (lp->xv[1] + lp->xv[2])
                + c[1] * lp->yv[0] + c[2] * lp->yv[1] + c[3] * lp->yv[2];
    return lp->yv[3];
}
#endif

/* freq is 1/T */
float LP_RC(float new_value, float prev_out_value, float freq, float dT)
{
    return (new_value - prev_out_value )*dT*freq + prev_out_value;
}

/* 12 us */
float LP_16_Wrap180(float new_value, float prev_value)
{
    float lp;

    if ((new_value - prev_value)>180)
        new_value-=360;
    else if ((new_value - prev_value)<-180)
        new_value+=360;
    lp = (new_value + prev_value*15 )/16;
    lp = Wrap180(lp);
    return lp;
}

float LP_Wrap180(float new_value, float prev_value, float n_avgs)
{
    float lp;

    if ((new_value - prev_value)>180)
        new_value-=360;
    else if ((new_value - prev_value)<-180)
        new_value+=360;
    lp = (new_value + prev_value*(n_avgs-1) )/n_avgs;
    lp = Wrap180(lp);
    return lp;
}


void wrap180(float* value)
{
    if (*value < -180 ) {
        *value += 360;
    }
    else
    if (*value > 180 ) {
        *value -= 360;
    }
    return;
}

/* rotates vector [x:y] into [x1:y1] by angle a in rads, positive is counter-clockwise */
void Rotate(float x, float y, float a, float *x1, float *y1)
{
  float sina = SINfR(a);
  float cosa = COSfR(a);
//  float sina = sinf(a);
//  float cosa = cosf(a);
  
  *x1 = x*cosa - y*sina;
  *y1 = x*sina + y*cosa;
}

/* rotates vector [x:y] by angle a in degrees, positive is counter-clockwise */
void Rotate2D(float *x, float *y, float a)
{
  if (a)
  {
    float sina = SINfD(a);
    float cosa = COSfD(a);
    float x1 = (*x)*cosa - (*y)*sina;
    float y1 = (*x)*sina + (*y)*cosa;
    *x = x1;
    *y = y1;
  }
}

/* translates vector relative to the ground into plane vector using PRY orientation in rads */
void Ground2Plane(float ground[3], float PRY[3], float plane[3])
{
    float x1, y1, z1;
    Rotate(ground[X], ground[Y],  PRY[YAW],   &x1, &y1);
    Rotate(y1,        ground[Z], -PRY[PITCH], &plane[Y], &z1);
    Rotate(x1,        z1,         PRY[ROLL],  &plane[X], &plane[Z]);
}

/* translates vector relative to the plane into the ground vector using PRY orientation in rads */
void Plane2Ground(float plane[3], float PRY[3], float ground[3])
{
    float x1, y1, z1;
    Rotate(plane[X], plane[Z], -PRY[ROLL],  &x1,        &z1);
    Rotate(plane[Y], z1,        PRY[PITCH], &y1,        &ground[Z]);
    Rotate(x1,       y1,       -PRY[YAW],   &ground[X], &ground[Y]);
}

/* translates vector relative to the plane into the ground vector using PRY orientation in rads, outputs up component only  */
float Plane2GroundU(float plane[3], float PRY[3])
{
    float z1, up;

    z1 = plane[X]*SINfR(-PRY[ROLL]) + plane[Z]*COSfR(-PRY[ROLL]);
    up = plane[Y]*SINfR(PRY[PITCH]) + z1*COSfR(PRY[PITCH]);
    return up;
}

/* keeps delta within +/-180 */
/* 3us */
float Wrap180(float delta)
{
    while(delta>=180)
        delta-=360;
    while(delta<-180)
        delta+=360;
    return delta;
//    int se = (long long)(delta*11930464.7111111f);
//    delta = se/11930464.7111111f;
//    return delta;
}

/* keeps delta within +/-PI */
float WrapPI(float delta)
{
    while(delta>=PI)
        delta-=2*PI;
    while(delta<-PI)
        delta+=2*PI;
    return delta;
//    int d = (long long)(delta*683565275.576f);
//    delta = d/683565275.576f;
//    return delta;
}

// Following is matrix math functins (add, subtract, inverse, print, scalar multiply, matrix multiply, integrate

void print_matrix3x3( double m[3][3] )
{
    int i;
    for( i = 0; i < 3; i++)
        debug_print("\t%f\t%f\t%f\r\n",m[i][0],m[i][1],m[i][2]);
}

void inverse_matrix3x3( double m[3][3] )
{

    double m_inv[3][3];

    double det = m[0][0] * ( m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
                 m[0][1] * ( m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                 m[0][2] * ( m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    double invdet = 1 / det;

    // calculate the inverse of matrix m
    m_inv[0][0] = ( m[1][1] * m[2][2] - m[2][1] * m[1][2] ) * invdet;
    m_inv[0][1] = ( m[0][2] * m[2][1] - m[0][1] * m[2][2] ) * invdet;
    m_inv[0][2] = ( m[0][1] * m[1][2] - m[0][2] * m[1][1] ) * invdet;
    m_inv[1][0] = ( m[1][2] * m[2][0] - m[1][0] * m[2][2] ) * invdet;
    m_inv[1][1] = ( m[0][0] * m[2][2] - m[0][2] * m[2][0] ) * invdet;
    m_inv[1][2] = ( m[1][0] * m[0][2] - m[0][0] * m[1][2] ) * invdet;
    m_inv[2][0] = ( m[1][0] * m[2][1] - m[2][0] * m[1][1] ) * invdet;
    m_inv[2][1] = ( m[2][0] * m[0][1] - m[0][0] * m[2][1] ) * invdet;
    m_inv[2][2] = ( m[0][0] * m[1][1] - m[1][0] * m[0][1] ) * invdet;

    memcpy(m, &m_inv, sizeof(m[0])*3);
    return;
}

void add_matrix3x3( double m1[3][3], double m2[3][3], double m[3][3])
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            m[i][j]=m1[i][j]+m2[i][j];
    return;
}

void subtract_matrix3x3( double m1[3][3], double m2[3][3], double m[3][3])
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            m[i][j]=m1[i][j]-m2[i][j];
    return;
}

void scalar_mutliply3x3(double m[3][3], double scalar)
{
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            m[i][j] *= scalar;
        }
    }
}
void multiply_matrix3x3( double m1[3][3], double m2[3][3], double m[3][3])
{
    int i, j, k;

    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            m[i][j] = 0;
            for(k = 0; k < 3; k++){
                m[i][j] +=  m1[i][k] *  m2[k][j];
            }
        }
    }
}

void integrate_matrix3x3(float data[3], float data_int[3], float dT)
{
    for(int i = 0; i < 3; i++){
        data_int[i] += data[i] * dT;
    }
}

