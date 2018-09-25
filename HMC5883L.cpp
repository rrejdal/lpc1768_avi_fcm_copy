/*
 * @file HMC5883L.cpp
 * @author Oskar Lopez de Gamboa
 *
 * @section LICENSE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * HMC5883L 3-Axis Digital Compas IC
 * The library done by Tyler Weaver with:
 *
 *  *Corrected the XZY order instead of the previous XYZ to match the datasheet.
 *  *Added Declination compensation by a define
 * 
 */

#include "HMC5883L.h"
#include "IMU.h"
#include "defines.h"
#include "mymath.h"
#include "utils.h"
#include <math.h>

//-----------
// Registers
//-----------
#define HMC5883L_I2C_ADDRESS     (0x3C)

#define CONFIG_A_REG    0
#define CONFIG_B_REG    1
#define MODE_REG        2
#define OUTPUT_REG      3
#define STATUS_REG      9
#define ID1_REG         10
#define ID2_REG         11
#define ID3_REG         12

// configuration register a
#define AVG1_SAMPLES    0x00<<5
#define AVG2_SAMPLES    0x01<<5
#define AVG4_SAMPLES    0x02<<5
#define AVG8_SAMPLES    0x03<<5

#define OUTPUT_RATE_0_75    0x00<<2
#define OUTPUT_RATE_1_5     0x01<<2
#define OUTPUT_RATE_3       0x02<<2
#define OUTPUT_RATE_7_5     0x03<<2
#define OUTPUT_RATE_15      0x04<<2
#define OUTPUT_RATE_30      0x05<<2
#define OUTPUT_RATE_75      0x06<<2

#define NORMAL_MEASUREMENT  0x00
#define POSITIVE_BIAS       0x01
#define NEGATIVE_BIAS       0x02

// mode register
#define CONTINUOUS_MODE     0x00
#define SINGLE_MODE         0x01
#define IDLE_MODE           0x02
#define HIGH_SPEED_I2C      0x80

// status register
#define STATUS_LOCK         0x02
#define STATUS_READY        0x01

// Utility
#ifndef M_PI
#define M_PI 3.1415926536f
#endif

#define PI2         (2*M_PI)
#define RAD_TO_DEG  (180.0f/M_PI)
#define DEG_TO_RAD  (M_PI/180.0f)

#define AK8963_ADDRESS   0x0C<<1
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASA       0x10  // axis sensitivity adjustment values (ASAX, ASAY, ASAZ)
#define AK8963_ASAX      0x10
#define AK8963_ASAY      0x11
#define AK8963_ASAZ      0x12

#define MFS_14BITS  0 // 0.6 mG per LSB
#define MFS_16BITS  1 // 0.15 mG per LSB

#define CNTL1_MODE_100HZ  6   // continuous 100Hz
#define CNTL1_MODE_8HZ    2   // continuous 8Hz

#define CNTL1_MODE_POWER_DOWN           0x00 // power down
#define CNTL1_MODE_FUSE_ROM             0x0F // reading ROM mode

#define STATUS2_DATA_ERROR			0x02
#define STATUS2_MAG_SENSOR_OVERFLOW 0x03

HMC5883L::HMC5883L(I2Ci *m_i2c)
{
    i2c = m_i2c;
    chip = CHIP_NONE;
    new_data_ = 0;
    pConfigData = NULL;
}

bool HMC5883L::Init(const ConfigData *pConfig)
{
    char id[3]={0,0,0};
	int numAvgsByte;
	int numAvgs;

	int   outputRateByte;
	float outputRate;

	int configA;

	pConfigData = pConfig;

	if (pConfigData->compass_type == CHIP_MMC5883MA) {
	    lsb_per_mga_ = 4.0;
	    return true;
	}
	else {
	    lsb_per_mga_ = 1.090;
	}

    i2c_address = HMC5883L_I2C_ADDRESS;
    if (getID(id, i2c_address, ID1_REG)==0 && id[0]==0x48 && id[1]==0x34 && id[2]==0x33)
    {
        // init - configure your setup here
    	numAvgsByte = AVG1_SAMPLES;
    	numAvgs     = pow(2,numAvgsByte>>5);

    	outputRateByte = OUTPUT_RATE_75;
    	switch(outputRateByte)
    	{
			case(OUTPUT_RATE_0_75):
					outputRate = 0.75f;
					break;
			case(OUTPUT_RATE_1_5):
					outputRate = 1.5f;
					break;
			case(OUTPUT_RATE_3):
					outputRate = 3.0f;
					break;
			case(OUTPUT_RATE_7_5):
					outputRate = 7.5f;
					break;
			case(OUTPUT_RATE_15):
					outputRate = 15.0f;
					break;
			case(OUTPUT_RATE_30):
					outputRate = 30.0f;
					break;
			case(OUTPUT_RATE_75):
					outputRate = 75.0f;
					break;
			default:
					outputRate = 15.0f;
					break;
    	}

        setConfigurationA(numAvgsByte | outputRateByte); // 8 sample average, 15Hz, normal mode
        configA = getConfigurationA();

        setConfigurationB(0x20); // default gain +/- 1.3Ga
        //lsb_Per_mGa = 1.090; // according to datasheet


        setMode(CONTINUOUS_MODE); // continuous sample mode

        reading_in_progress = false;
        interval = 1/outputRate;
        duration = 0;
        wait(0.006f);//wait 6ms as told in the datasheet

        chip = CHIP_HMC5883L;
        printf("Compass HMC5883L initialized\r\n");
        printf("\t - output Data Rate = %f,  Averaging %d Samples\r\n",outputRate,numAvgs);
        printf("\t - HMC5883L ConfigA REG = %d\r\n",configA);
        return true;
    }


    i2c_address = AK8963_ADDRESS;
    if (getID(id, i2c_address, 0)==0 && id[0]==0x48 && id[1]==0x9a)
    {
		uint8_t asaXYZ[3];
	
        i2c->write_reg_blocking(i2c_address, AK8963_CNTL, CNTL1_MODE_POWER_DOWN); // Power down magnetometer
        wait(0.01f);
        i2c->write_reg_blocking(i2c_address, AK8963_CNTL, CNTL1_MODE_FUSE_ROM);   // Reading ROM mode
        wait(0.01f);
        asaXYZ[0] = i2c->read_reg_blocking(i2c_address, AK8963_ASAX);   // Reading ROM adjustment values
        wait(0.01f);
        asaXYZ[1] = i2c->read_reg_blocking(i2c_address, AK8963_ASAY);   // Reading ROM adjustment values
        wait(0.01f);
        asaXYZ[2] = i2c->read_reg_blocking(i2c_address, AK8963_ASAZ);   // Reading ROM adjustment values
        wait(0.01f);
		//printf("asaXYZ(XYZ): %6.3f %6.3f %6.3f\r\n", (float)asaXYZ[0], (float)asaXYZ[1], (float)asaXYZ[2]);

		float mRes = 10.*4912./8190.; // Proper scale to return milliGauss
		magGainXYZ[0] = ((((float)asaXYZ[0] - 128) / 256) + 1) * mRes;
		magGainXYZ[1] = ((((float)asaXYZ[1] - 128) / 256) + 1) * mRes;
		magGainXYZ[2] = ((((float)asaXYZ[2] - 128) / 256) + 1) * mRes;
		//printf("magGainXYZ(XYZ): %6.3f %6.3f %6.3f\r\n", magGainXYZ[0], magGainXYZ[1], magGainXYZ[2]);

        i2c->write_reg_blocking(i2c_address, AK8963_CNTL, CNTL1_MODE_POWER_DOWN); // Power down magnetometer
        wait(0.01f);
        asaXYZ[0] = i2c->read_reg_blocking(i2c_address, AK8963_ST1);   // Clear ST1 status register
        wait(0.01f);
        asaXYZ[0] = i2c->read_reg_blocking(i2c_address, AK8963_ST2);   // Clear ST2 status register
        wait(0.01f);
        i2c->write_reg_blocking(i2c_address, AK8963_CNTL, MFS_14BITS << 4 | CNTL1_MODE_8HZ); // Set magnetometer data resolution and sample ODR
        wait(0.01f);

        reading_in_progress = false;
        interval = 1/15.0f;
        duration = 0;
        chip = CHIP_AK8963;
        printf("Compass AK8963 initialized\r\n");
        return true;
    }

    printf("Compass not responding %x %x %x\r\n", id[0], id[1], id[2]);
    return false;
}

void HMC5883L::setConfigurationA(char config)
{
    i2c->write_reg_blocking(i2c_address, CONFIG_A_REG, config);
}

void HMC5883L::setConfigurationB(char config)
{
    i2c->write_reg_blocking(i2c_address, CONFIG_B_REG, config);
}

char HMC5883L::getConfigurationA()
{
    return i2c->read_reg_blocking(i2c_address, CONFIG_A_REG);
}

char HMC5883L::getConfigurationB()
{
    return i2c->read_reg_blocking(i2c_address, CONFIG_B_REG);
}

void HMC5883L::setMode(char mode = SINGLE_MODE)
{
    i2c->write_reg_blocking(i2c_address, MODE_REG, mode);
}

char HMC5883L::getMode()
{
    return i2c->read_reg_blocking(i2c_address, MODE_REG);
}

char HMC5883L::getStatus()
{
    return i2c->read_reg_blocking(i2c_address, STATUS_REG);
}

char HMC5883L::getID(char id[3], int adr, int reg)
{
    int id1;
    id1 = i2c->read_reg_blocking(adr, reg);
    if (id1<0)
      return 255;
    id[0] = id1;
    id[1] = i2c->read_reg_blocking(adr, reg+1);
    id[2] = i2c->read_reg_blocking(adr, reg+2);
    return 0;
}

void HMC5883L::UpdateRawData(short raw_x, short raw_y, short raw_z)
{
    dataXYZ[COMP_X] = raw_x / lsb_per_mga_;
    dataXYZ[COMP_Y] = raw_y / lsb_per_mga_;
    dataXYZ[COMP_Z] = raw_z / lsb_per_mga_;
    new_data_ = 1;
}

bool HMC5883L::getRawValues(float dT)
{
    if (chip==CHIP_NONE)
        return false;

    if (!reading_in_progress)
    {
        /* waiting for the interval before the next read */
        duration += dT;
        if (duration>=interval)
        {
            /* interval reached, dispatch the I2C read */
            if (chip==CHIP_HMC5883L)
                i2c->read_regs_nb(i2c_address, 0x03, raw_data, 6, &status);
            else if (chip==CHIP_AK8963)
            {
                i2c->read_regs_nb(i2c_address, AK8963_ST1, raw_data, 8, &status);
//                i2c->read_regs_nb(i2c_address, AK8963_ST1, raw_data, 1, NULL);
//                i2c->read_regs_nb(i2c_address, AK8963_ST1+1, raw_data+1, 1, NULL);
/*                int i;
                for (i=0; i<7; i++)
                    i2c->read_regs_nb(i2c_address, AK8963_ST1+i, raw_data+i, 1, NULL);
                i2c->read_regs_nb(i2c_address, AK8963_ST1+7, raw_data+7, 1, &status);*/
            }
            reading_in_progress = true;
        }
        return false;            
    }
    else
    {
        /* wait for the data to arrive */
        if (!status)
            return false;
    }
    reading_in_progress = false;
    duration = 0;

    /* convert bytes to ints */
    if (chip==CHIP_HMC5883L)
    {
        dataXYZ[COMP_X] = int16_t(((unsigned char)raw_data[0] << 8) | (unsigned char)raw_data[1]) / lsb_per_mga_;
        dataXYZ[COMP_Z] = int16_t(((unsigned char)raw_data[2] << 8) | (unsigned char)raw_data[3]) / lsb_per_mga_;
        dataXYZ[COMP_Y] = int16_t(((unsigned char)raw_data[4] << 8) | (unsigned char)raw_data[5]) / lsb_per_mga_;
    }
    else
    if (chip==CHIP_AK8963)
    {
	    uint8_t status2 = raw_data[7];
	    if ((status2 & STATUS2_DATA_ERROR) || (status2 & STATUS2_MAG_SENSOR_OVERFLOW)) {
	        return false;
	    }
        dataXYZ[COMP_X] = int16_t((raw_data[2] << 8) | raw_data[1]) * magGainXYZ[COMP_X];
        dataXYZ[COMP_Y] = int16_t((raw_data[4] << 8) | raw_data[3]) * magGainXYZ[COMP_Y];
        dataXYZ[COMP_Z] = int16_t((raw_data[6] << 8) | raw_data[5]) * magGainXYZ[COMP_Z];
        //printf("dataXYZ(XYZ): %10d %10d %10d\r\n", dataXYZ[0], dataXYZ[1], dataXYZ[2]);
    }
    return true;
}
    
float HMC5883L::GetHeadingDeg(const unsigned char comp_orient[6], const float offsets[3],
                                  const float gains[3], const unsigned char fcm_orient[6],
                                  float declination_offset, float pitch, float roll)
{
    int i;
    float heading;
    float data[3];
    float compENU[3];
    float PRY[3] = {pitch, roll, 0};

	//printf("Compas_data(XYZ): %10d %10d %10d\r\n", dataXYZ[0], dataXYZ[1], dataXYZ[2]);
	//printf("magGain(XYZ): %6.3f %6.3f %6.3f\r\n", magGainXYZ[0], magGainXYZ[1], magGainXYZ[2]);
    
    /* apply calibration */
    /* mmri: apply gain and offset to acc
     * if comp_gains = 0, then use comp calib matrix
     * In this case we do the orientation rotation first then the calibration
     * since the compass has to be mounted on the drone before calibrating
     * and the calibration data is collected from the frame of reference of
     * the drone.*/

    // NOTE::SP: For the minute, we are ont making use of a seperate comp_calib_matrix, so
    // removing this
    //
    //if( gains[0] == 0 )
    //{
    //	for (i=0; i<3; i++) {
	//		dataXYZcalib[i] = pConfigData->comp_calib_matrix[0][i] * (dataXYZ[0] - offsets[0])
	//	                        + pConfigData->comp_calib_matrix[1][i] * (dataXYZ[1] - offsets[1])
	//	                        + pConfigData->comp_calib_matrix[2][i] * (dataXYZ[2] - offsets[2]);
    //	}
    //}
    //else {
    	for (i=0; i<3; i++) {
    		dataXYZcalib[i] = (dataXYZ[i] - offsets[i]) * gains[i];
    	}
    //}

    data[0] = dataXYZcalib[0];
    data[1] = dataXYZcalib[1];
    data[2] = dataXYZcalib[2];
    /* re-orient compass, negative value flips sign */
    for (i=0; i<3; i++)
    {
    	dataXYZcalib[i] = data[comp_orient[i]];
      if (comp_orient[i+3])
    	  dataXYZcalib[i] = -dataXYZcalib[i];
    }

    data[0] = dataXYZcalib[0];
    data[1] = dataXYZcalib[1];
    data[2] = dataXYZcalib[2];
    /* re-orient the entire flight controler, negative value flips the sign */
    for (i=0; i<3; i++)
    {
      dataXYZcalib[i] = data[fcm_orient[i]];
      if (fcm_orient[i+3])
    	  dataXYZcalib[i]  = -dataXYZcalib[i];
    }

    /* compass is in RFU coordinates at this point. Magnetic vectors point almost down in northen hemisphere
    ** the given axes has to read out positive when aligned with the magnetic vector.
    ** This needs to be re-oriented to ENU using PRY. Then heading=atan2(E,N) */
    
    /* 42us */
    Plane2Ground(dataXYZcalib, PRY, compENU);
    /* 3us */
    heading = ATAN2fD(-compENU[EAST], compENU[NORTH]);

    heading += declination_offset;
    
    if(heading < -180) // fix sign
        heading += 360;
    else        
    if(heading > 180) // fix overflow
        heading -= 360;

    new_data_ = 0;

    return heading;
}


#define MPU6050_RA_INT_PIN_CFG              0x37
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6050_PWR_MGMT_1                  0x6B
#define MPU6050_CONFIG                      0x1A
#define MPU6050_CLOCK_PLL_XGYRO             0x01
#define MPU6050_WHO_AM_I         0x75

void HMC5883L::enable_i2c_MPU6050(char mpu_address)
{
    i2c->write_reg_blocking(mpu_address, MPU6050_RA_INT_PIN_CFG, 1<<MPU6050_INTCFG_I2C_BYPASS_EN_BIT);
    i2c->write_reg_blocking(mpu_address, MPU6050_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);
    
    /* config - sync input and LPF */
    i2c->write_reg_blocking(mpu_address, MPU6050_CONFIG, 0);    
}
