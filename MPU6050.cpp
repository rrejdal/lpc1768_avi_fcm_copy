#include "mbed.h"
#include "MPU6050.h"

enum {
    CHIP_NONE    = 0,
    CHIP_MPU6050 = 1,
    CHIP_MPU9250 = 2,
};

#define MPU6050_ADDRESS_EXTERNAL (0x68)
#define MPU6050_ADDRESS_INTERNAL (0x69)

#define MPU6050_ADDRESS         MPU6050_ADDRESS_INTERNAL

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

static const float gyro_scale_factor[4] = {250.0f/32768.0f, 500.0f/32768.0f, 1000.0f/32768.0f, 2000.0f/32768.0f};
static const float acc_scale_factor[4]  = {  2.0f/32768.0f,   4.0f/32768.0f,    8.0f/32768.0f,   16.0f/32768.0f};
// Public Methods //////////////////////////////////////////////////////////////

bool MPU6050::is_ok()
{
    int r = i2c->read_reg_blocking(i2c_address, MPU6050_WHO_AM_I);
    if (r < 0) {
        printf("MPU6050 not responding at address %d\r\n", i2c_address);
        return false;
    }

    if (r == WHO_AM_I_VALUE_6050) {
        chip_version = CHIP_MPU6050;
        return true;
    }
    else if (r == WHO_AM_I_VALUE_9250) {
        chip_version = CHIP_MPU9250;
        return true;
    }
    else {
        printf("Unrecognized IMU: %x\r\n", r);
        return false;
    }
}

void MPU6050::init(char lp, int internal)
{
	if (internal == 1) {
		i2c_address = (MPU6050_ADDRESS_INTERNAL << 1);
	}
	else {
		i2c_address = (MPU6050_ADDRESS_EXTERNAL << 1);
	}

	/* clock source and disable sleep */
	i2c->write_reg_blocking(i2c_address, MPU6050_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);

	/* config - sync input and LPF */
	i2c->write_reg_blocking(i2c_address, MPU6050_CONFIG, lp);

	/* gyro range */
	i2c->write_reg_blocking(i2c_address, MPU6050_GYRO_CONFIG, gyro_scale<<3);

	/* acc range */
	i2c->write_reg_blocking(i2c_address, MPU6050_ACCEL_CONFIG, acc_scale<<3);

	/* enable secondary I2C for HMC5883L */
	i2c->write_reg_blocking(i2c_address, MPU6050_RA_INT_PIN_CFG, 1<<MPU6050_INTCFG_I2C_BYPASS_EN_BIT);
}

// Constructor
MPU6050::MPU6050(I2Ci *m_i2c, int m_gyro_scale, int m_acc_scale)
{
    i2c = m_i2c;
    chip_version = CHIP_NONE;

    gyro_scale = m_gyro_scale;
    acc_scale  = m_acc_scale;

}

void MPU6050::readMotion7_start()
{
    if (chip_version != CHIP_NONE) {
        i2c->read_regs_nb(i2c_address, MPU6050_ACCEL_XOUT_H, m7buffer, sizeof(m7buffer), &status);
    }
}

bool MPU6050::readMotion7_finish(short int acc[3], short int gyro[3], short int *temp)
{
    if (chip_version == CHIP_NONE) {
        return false;
    }

    int counter = 2000; // 2ms
    while(1)
    {
        if (status)
            break;
        /* timeout */
        counter--;
        if (counter<=0)
            return false;
        wait_us(1);
    }
    acc[0]  = (((short int)m7buffer[0]) << 8) | m7buffer[1];
    acc[1]  = (((short int)m7buffer[2]) << 8) | m7buffer[3];
    acc[2]  = (((short int)m7buffer[4]) << 8) | m7buffer[5];
    *temp   = (((short int)m7buffer[6]) << 8) | m7buffer[7];
    gyro[0] = (((short int)m7buffer[8]) << 8) | m7buffer[9];
    gyro[1] = (((short int)m7buffer[10])<< 8) | m7buffer[11];
    gyro[2] = (((short int)m7buffer[12])<< 8) | m7buffer[13];
    return true;
}

bool MPU6050::readMotion7f_finish(float acc[3], float gyro[3], float *temp)
{
    short int t;
    short int a[3];
    short int g[3];

    if (!readMotion7_finish(a, g, &t)) {
        return false;
    }

    acc[0]  = a[0]*acc_scale_factor[acc_scale];
    acc[1]  = a[1]*acc_scale_factor[acc_scale];
    acc[2]  = a[2]*acc_scale_factor[acc_scale];

    if (chip_version == CHIP_MPU6050) {
        *temp   = t*0.00294117647f + 36.53f;
    }
    else {
        *temp   = t/333.87f + 21.0f;
    }

    gyro[0] = g[0]*gyro_scale_factor[gyro_scale];
    gyro[1] = g[1]*gyro_scale_factor[gyro_scale];
    gyro[2] = g[2]*gyro_scale_factor[gyro_scale];

    return true;
}




/***********************************************************
 * MMRI - Extra MPU6050 IMU functions used for calibration
 * Found at the following URL:
 * https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 *
 ************************************************************/

/** Read accelerometer, gyro and temperature data in I2C blocking mode */
bool MPU6050::readMotion7_blocking(float acc[3], float gyro[3], float *temp)
{
    short int t;
    short int a[3];
    short int g[3];

	i2c->read_regs_blocking(i2c_address, MPU6050_ACCEL_XOUT_H, m7buffer, sizeof(m7buffer), &status);

    if (!readMotion7_finish(a, g, &t)) {
        return false;
    }

    acc[0]  = a[0]*acc_scale_factor[acc_scale];
    acc[1]  = a[1]*acc_scale_factor[acc_scale];
    acc[2]  = a[2]*acc_scale_factor[acc_scale];

    if (chip_version == CHIP_MPU6050) {
        *temp   = t*0.00294117647f + 36.53f;
    }
    else {
        *temp   = t/333.87f + 21.0f;
    }

    gyro[0] = g[0]*gyro_scale_factor[gyro_scale];
    gyro[1] = g[1]*gyro_scale_factor[gyro_scale];
    gyro[2] = g[2]*gyro_scale_factor[gyro_scale];

    return true;
}

/** Get Data Ready interrupt enabled setting.
 * This event occurs each time a write operation to all of the sensor registers
 * has been completed. Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_DATA_RDY_BIT
 */
uint8_t MPU6050::getIntDataReadyEnabled() {
    buffer = i2c->read_reg_blocking(i2c_address, MPU6050_RA_INT_ENABLE);
    return buffer;//(buffer & MPU6050_INTERRUPT_DATA_RDY_BIT);
}

/** Set Data Ready interrupt enabled status.
 * @param enable (1) or disable(0)  New interrupt enabled status
 * @see getIntDataReadyEnabled()
 */
void MPU6050::setIntDataReady(bool setbit) {
    buffer = getIntDataReadyEnabled();
    if(setbit) buffer = buffer | MPU6050_INTERRUPT_DATA_RDY_BIT;
    else buffer = buffer & (0xFF^MPU6050_INTERRUPT_DATA_RDY_BIT);
    i2c->write_reg_blocking(i2c_address, MPU6050_RA_INT_ENABLE, buffer);
}

// INT_STATUS register

/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 */
uint8_t MPU6050::getIntStatus() {
    buffer = i2c->read_reg_blocking(i2c_address, MPU6050_RA_INT_STATUS);
    return (buffer & MPU6050_INTERRUPT_DATA_RDY_BIT);
}


