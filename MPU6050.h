#ifndef _MPU6050_H_
#define _MPU6050_H_
 
#include "mbed.h"
#include "I2Ci.h"
#include "eeprom.h"
#include "structures.h"

#define NUM_TYPE_IMUS            2
#define MPU6050_ADDRESS_EXTERNAL (0x68)
#define MPU6050_ADDRESS_INTERNAL (0x69)

#define MUX_ADDRESS             (0x70 << 1)

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

#define CHIP_MPU6050         0
#define CHIP_MPU9250         1

#define WHO_AM_I_VALUE_9250     (0x71)
#define WHO_AM_I_VALUE_6050     (0x68)

// register addresses
#define MPU6050_PWR_MGMT_1       0x6B
#define MPU6050_CONFIG           0x1A
#define MPU6050_GYRO_CONFIG      0x1B
#define MPU6050_ACCEL_CONFIG     0x1C
#define MPU6050_RA_INT_PIN_CFG   0x37
#define MPU6050_ACCEL_XOUT_H     0x3B
#define MPU6050_WHO_AM_I         0x75

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT 1

#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_INT_STATUS       0x3A

#define MPU6050_INTERRUPT_DATA_RDY_BIT      0x01

enum IMU_TYPE {
    INTERNAL    = 0,
    EXTERNAL    = 1
};

// NOTE::SP: This data is taken from IMU X4-02 Calibration and is used for TESTING
// only
//#define IMU_X4_02

#ifdef IMU_X4_02
static double dflt_gyroP_temp_coeffs[] = { -0.0001494521, -0.07258565,  1.118024  };
static double dflt_gyroR_temp_coeffs[] = {  4.798885e-05,  0.0251157,  -0.8757614 };
static double dflt_gyroY_temp_coeffs[] = {  0.0003652208, -0.00113438, -2.332723  };
static double dflt_aofs[] = { 0.0302385, 0.006651, -0.025128 };

static double dflt_acc_calib_matrix[3][3] = { {0.9955115, -0.009184, -0.0071185},
                                            {0.0054765,  0.99369,  -0.005709},
                                            {-0.001209, -0.016995,  0.983033} };

static double dflt_gyr_calib_matrix[3][3] = { {0.9931725, -0.0100325, -0.0025095},
                                            {0.0106015,  1.0072235,  0.0073105},
                                            {0.003368, 0.013248,   0.9936395} };
#endif

/** Interface library for the MPU-6050 */
class MPU6050
{
    public:
        /** Create a new MPU6050 I2C interface
         * @param sda is the pin for the I2C SDA line
         * @param scl is the pin for the I2C SCL line
         */
        MPU6050(I2Ci *m_i2c, int mux_reg_setting, int m_gyro_scale, int m_acc_scale);
        
        bool is_ok();
        uint32_t init(const ConfigData *pConfig, char lp, unsigned int *serial_num);
        bool SetGyroScale(char g_scale);
        bool SetAccScale(char a_scale);
        void readMotion7_start();
        bool readMotion7_finish(short int acc[3], short int gyro[3], short int *temp);
        bool readMotion7f_finish(float acc[3], float gyro[3], float *temp);

        //mmri
        bool readMotion7_blocking(float acc[3], float gyro[3], float *temp);
        bool readMotion7_blocking();

        uint8_t getIntDataReadyEnabled();
        void setIntDataReady(bool setbit);
        uint8_t getIntStatus();

        void determine_orient();

        void ForceCalData();

        // Helper functions for Accelerometer Calibration
        void write_accel_data();
        void get_accel_calib_data(int avg_num_samples);

        // Helper funcitons for Gyro Calibration
        void write_omegaS_data();
        void average_omegaS();
        void calculate_omega(float *time_diff);
        void calculate_omega0();
        void print_omegaS();
        void get_gyro_calib_data();

        // Takes in raw data and calibrates it
        void calibrate_accel(float data_raw[3], float data_cali[3]);
        void calibrate_gyro(float data_raw[3], float data_cali[3]);

        bool accel_error(float acc_raw_avg[3]);
        // something similar for gyro error

        // EEPROM read and write functions
        void write_eeprom();
        int read_eeprom();
        void print_eeprom();

        float acc_raw_data[3];
        float gyro_raw_data[3];
        float temp_imu;

        int orient_index;
        int round_orient[3];

        EEPROM *eeprom;

        int serialNum;
        int calib_day;
        int calib_month;
        int calib_year;

        double Ca[3][3];
        double aofs[3];

		double Cg[3][3];
		double gofs[3];
        double gyroP_temp_coeffs[3];
        double gyroR_temp_coeffs[3];
        double gyroY_temp_coeffs[3];

        float dT;

        float n_samples;

        double omega_0[3][3];
        double omega_s[3][3];
        double   omega[3][3];

    private:
        I2Ci *i2c;

        int           i2c_address;
        unsigned char gyro_scale;
        unsigned char acc_scale;

        volatile char m7buffer[14]; // buffer receiving values from the background I2C read
        uint8_t		  buffer;  // mmri: additional buffer used for receiving IMU I2C reads
        volatile int  status;  // completion status of the background I2C read

        double Mp[3][3];
        double Mn[3][3];


        float rotation_angle;
        bool mux;
        char mux_chan;
};


#endif
