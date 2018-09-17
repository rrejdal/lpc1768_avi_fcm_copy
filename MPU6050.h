#ifndef _MPU6050_H_
#define _MPU6050_H_
 
#include "mbed.h"
#include "I2Ci.h"
 
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

/** Interface library for the MPU-6050 */

class MPU6050
{
    public:
        /** Create a new MPU6050 I2C interface
         * @param sda is the pin for the I2C SDA line
         * @param scl is the pin for the I2C SCL line
         */
        MPU6050(I2Ci *m_i2c, int m_gyro_scale, int m_acc_scale);
        
        bool is_ok();
        void init(char lp, int internal);
        bool SetGyroScale(char g_scale);
        bool SetAccScale(char a_scale);
        void readMotion7_start();
        bool readMotion7_finish(short int acc[3], short int gyro[3], short int *temp);
        bool readMotion7f_finish(float acc[3], float gyro[3], float *temp);

        //mmri
        bool readMotion7_blocking(float acc[3], float gyro[3], float *temp);
        uint8_t getIntDataReadyEnabled();
        void setIntDataReady(bool setbit);
        uint8_t getIntStatus();

    private:
        I2Ci *i2c;
        int           i2c_address;
        unsigned char gyro_scale;
        unsigned char acc_scale;
        volatile char m7buffer[14]; // buffer receiving values from the background I2C read
        uint8_t		  buffer;  // mmri: additional buffer used for receiving IMU I2C reads
        volatile int  status;  // completion status of the background I2C read
        char    chip_version;   // 6050 or 9250
 };
 
#endif
