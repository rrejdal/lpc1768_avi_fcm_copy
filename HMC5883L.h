/*
 * @file HMC5883L.h
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
 *
 */

#ifndef HMC5883L_H
#define HMC5883L_H

#include "mbed.h"
#include "I2Ci.h"
#include "structures.h"

#define CHIP_NONE       0
#define CHIP_HMC5883L   1
#define CHIP_AK8963     2
#define CHIP_MMC5883MA  3

#define COMP_X      0
#define COMP_Y      1
#define COMP_Z      2


/*
* Defines
*/

// Once you have your heading, you must then add your 'Declination Angle', 
// which is  the 'Error' of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/
// Mine is:  -1° 13' WEST which is -1.2167 Degrees, or (which we need) 
// 0,021234839232597676519238237683278  radians, I will use 0.02123
// If you cannot find your Declination, put 0, your compass will be slightly off.

#define  WATERLOO_DECLINATION_ANGLE -9.6333f

#define MPU6050_ADDRESS         (0x68<<1)

/** The HMC5883L 3-Axis Digital Compass IC */
class HMC5883L
{
public:
    /* Constructor.
     * Calls init function
     * @param sda - mbed pin to use for the SDA I2C line.
     * @param scl - mbed pin to use for the SCL I2C line. */
    HMC5883L(I2Ci *m_i2c);

    /**Check sensor ID, initialize the sensor. Returns false if anything fails */
    bool Init(int compass_type, ConfigData *pConfig);
    
    /* Function for setting configuration register A
    *
    * Defined constants should be ored together to create value.
    * Defualt is 0x10 - 1 Sample per output, 15Hz Data output rate, normal measurement mode
    * Refer to datasheet for instructions for setting Configuration Register A.
    * @param config the value to place in Configuration Register A */
    void setConfigurationA(char);
    
    /* Function for retrieving the contents of configuration register A */
    char getConfigurationA();
    
    /* Function for setting configuration register B
    * Configuration Register B is for setting the device gain.
    * Default value is 0x20
    * Refer to datasheet for instructions for setting Configuration Register B
    * @param config the value to place in Configuration Register B */
    void setConfigurationB(char);
    
    /* Function for retrieving the contents of configuration register B */
    char getConfigurationB();
    
    /* Funciton for setting the mode register
    * Constants: CONTINUOUS_MODE, SINGLE_MODE, IDLE_MODE
    * When you send a the Single-Measurement Mode instruction to the mode register
    * a single measurement is made, the RDY bit is set in the status register,
    * and the mode is placed in idle mode.
    * When in Continous-Measurement Mode the device continuously performs measurements
    * and places the results in teh data register.  After being placed in this mode
    * it takes two periods at the rate set in the data output rate before the first
    * sample is avaliable.
    * Refer to datasheet for more detailed instructions for setting the mode register.
    * @param mode the value for setting in the Mode Register  */
    void setMode(char);
    
    /* Function for retrieving the contents of mode register */
    char getMode();
    
    /* Function for retrieving the contents of status register
    * Bit1: LOCK, Bit0: RDY */
    char getStatus();
    
    /* Function for getting degree heading using 2-dimensional calculation.
    * dT is a time increment since last call in seconds. Once true is returned
    ** *heading contains the new value. Functions is non-blocking and returns the new value
    ** at the interval specified in init(). After getHeadingXYDeg() provides new value,
    ** getXYZ() can be used to get the raw values, for calibration.
    * @returns heading in degrees, +/-180, 0 is north, clock-wise */
    bool getRawValues(float dT);

    float GetHeadingDeg(unsigned char comp_orient[6], float offsets[3], float gains[3], unsigned char FCM_orient[6], float declination_offset, float pitch, float roll);
    
    char getID(char id[3], int adr, int reg);

    /* enables secondary I2C within MPU6050 to access HMC5883L */
    void enable_i2c_MPU6050(char mpu_address);

    void UpdateRawData(signed short raw_x, signed short raw_y, signed short raw_z);

    int16_t dataXYZ[3]; // raw sensor values
    float   dataXYZcalib[3];    // calibrated sensor values, offsets and gains applied
    float   magGainXYZ[3]; // sensitivity adjustment values
    int HaveNewData(void) { return new_data_ ;};
       
private:
    I2Ci *i2c;
    ConfigData *pConfigData;
    int i2c_address;
    float duration;
    float interval;
    volatile char  raw_data[8];
    volatile int   status;
    bool  reading_in_progress;  // true when read was issued, but data not received yet, false when accumulating time before the next read
    char  chip;
    int new_data_;
    float lsb_per_mga_;
};

#endif // HMC5883L
