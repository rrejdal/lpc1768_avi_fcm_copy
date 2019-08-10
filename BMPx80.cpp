/**
  ******************************************************************************
  * @file    BMPx80.h
  * @author  AVIDRONE FIRMWARE Team
  * @brief   Provides support for Baro
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 Avidrone Aerospace Inc.</center></h2>
  *
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "BMPx80.h"
#include "utils.h"
#include "defines.h"

#define BMP280_ID   0x58
#define BMP280_ADDRESS          0x76<<1 // I2C address of BMP280, eight bit address
#define BMP280_WHO_AM_I         0xD0    // WHO_AM_I id of BMP280, should return 0x55
#define BMP280_RESET            0xE0
#define BMP280_STATUS           0xF3
#define BMP280_CONTROL          0xF4
#define BMP280_CONFIG           0xF5

#define BMP280_PRESSURE         0xF7

#define WAIT280_ALL  0.023f // 23ms
#define BMP_TIMEOUT  0.100f // 100 ms
#define BMP280_POS  4   // x8
#define BMP280_TOS  1   // x1
#define BMP280_MODE 3   // normal
#define BMP280_SB       0   // 0.5ms
#define BMP280_FILTER   0   // off
#define BMP280_SPI      0   // no SPI

// Set initial input parameters

enum OSS {  // BMP-085 sampling rate
  OSS_0 = 0,  // 4.5 ms conversion time
  OSS_1,      // 7.5
  OSS_2,      // 13.5
  OSS_3       // 25.5
};

#define STATE_NONE          0
#define STATE_TEMP_STARTED  1
#define STATE_TEMP_READ     2
#define STATE_PRESS_STARTED 3
#define STATE_PRESS_READ    4

#define WAIT_TEMPERATURE    0.005f  // temp measurement takes 5ms
#define WAIT_PRESSURE       0.026f  // pressure measurement takes 26ms for OSS=3

static const float PA2ALT[167] = {
9054.37243f ,8945.050178f,8837.14645f, 8730.620759f,8625.434378f,8521.550248f,8418.932878f,
8317.548254f,8217.363762f,8118.348106f,8020.471241f,7923.704303f,7828.019549f,7733.390298f,
7639.790876f,7547.196567f,7455.583561f,7364.928916f,7275.210509f,7186.407003f,7098.497805f,
7011.463032f,6925.283479f,6839.940591f,6755.416427f,6671.693638f,6588.755440f,6506.585586f,
6425.168348f,6344.488491f,6264.531254f,6185.282329f,6106.727844f,6028.854343f,5951.648773f,
5875.098462f,5799.191111f,5723.914772f,5649.257842f,5575.209043f,5501.757414f,5428.892297f,
5356.603328f,5284.880422f,5213.713767f,5143.093814f,5073.011263f,5003.457060f,4934.422383f,
4865.898640f,4797.877455f,4730.350664f,4663.310307f,4596.748622f,4530.658036f,4465.031162f,
4399.860790f,4335.139885f,4270.861575f,4207.019154f,4143.606068f,4080.615919f,4018.042454f,
3955.879560f,3894.121267f,3832.761735f,3771.795255f,3711.216244f,3651.019240f,3591.198902f,
3531.750003f,3472.667426f,3413.946164f,3355.581317f,3297.568085f,3239.901769f,3182.577766f,
3125.591567f,3068.938756f,3012.615004f,2956.616070f,2900.937797f,2845.576111f,2790.527015f,
2735.786592f,2681.351001f,2627.216473f,2573.379313f,2519.835893f,2466.582656f,2413.616111f,
2360.932829f,2308.529449f,2256.402668f,2204.549243f,2152.965992f,2101.649789f,2050.597563f,
1999.806298f,1949.273032f,1898.994854f,1848.968903f,1799.192369f,1749.662491f,1700.376551f,
1651.331883f,1602.525861f,1553.955906f,1505.619481f,1457.514091f,1409.637283f,1361.986643f,
1314.559798f,1267.354410f,1220.368184f,1173.598857f,1127.044204f,1080.702035f,1034.570195f,
988.6465627f,942.9290489f,897.4155975f,852.1041838f,806.9928143f,762.0795258f,717.3623849f,
672.8394872f,628.5089569f,584.3689464f,540.4176352f,496.6532298f,453.0739631f,409.6780936f,
366.4639053f,323.4297069f,280.5738315f,237.8946359f,195.3905005f,153.0598284f,110.9010454f,
68.91259922f,27.09295935f,-14.55938353f,-56.04591789f,-97.36811173f,-138.5274130f,-179.5252498f,
-220.3630311f,-261.0421467f,-301.5639679f,-341.9298476f,-382.1411206f,-422.1991043f,-462.1050986f,
-501.8603864f,-541.4662337f,-580.9238905f,-620.2345901f,-659.3995503f,-698.4199733f,-737.2970457f,
-776.0319393f,-814.6258110f,-853.0798032f,-891.3950439f,-929.5726472f,-967.6137131f};


BMPx80::BMPx80(I2Ci *m_i2c)
{
    i2c = m_i2c;
    OSS = OSS_3;           // maximum pressure resolution
    state = STATE_NONE;
    bmp = BMP_NONE;
}

uint32_t BMPx80::Init(const ConfigData *pConfig)
{
  uint32_t result = 0;

  if (pConfig->baro_enable) {
    i2c_address = BMP280_ADDRESS;

    // Read the WHO_AM_I register of the BMP-280, this is a good test of communication
    if (i2c->read_reg_blocking(i2c_address, BMP280_WHO_AM_I) == BMP280_ID) {
      Calibration280();
      bmp = BMP_280;
    }
    else {
      result = BARO_FAIL;
    }
  }
  return result;
}

// Stores all of the BMP280's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
// These BMP-180 functions were adapted from Jim Lindblom of SparkFun Electronics
void BMPx80::Calibration280()
{
  dig_T1 = (i2c->read_reg_blocking(i2c_address, 0x88) << 0) | (i2c->read_reg_blocking(i2c_address, 0x89) << 8);
  dig_T2 = (i2c->read_reg_blocking(i2c_address, 0x8a) << 0) | (i2c->read_reg_blocking(i2c_address, 0x8b) << 8);
  dig_T3 = (i2c->read_reg_blocking(i2c_address, 0x8c) << 0) | (i2c->read_reg_blocking(i2c_address, 0x8d) << 8);

  dig_P1 = (i2c->read_reg_blocking(i2c_address, 0x8e) << 0) | (i2c->read_reg_blocking(i2c_address, 0x8f) << 8);
  dig_P2 = (i2c->read_reg_blocking(i2c_address, 0x90) << 0) | (i2c->read_reg_blocking(i2c_address, 0x91) << 8);
  dig_P3 = (i2c->read_reg_blocking(i2c_address, 0x92) << 0) | (i2c->read_reg_blocking(i2c_address, 0x93) << 8);
  dig_P4 = (i2c->read_reg_blocking(i2c_address, 0x94) << 0) | (i2c->read_reg_blocking(i2c_address, 0x95) << 8);
  dig_P5 = (i2c->read_reg_blocking(i2c_address, 0x96) << 0) | (i2c->read_reg_blocking(i2c_address, 0x97) << 8);
  dig_P6 = (i2c->read_reg_blocking(i2c_address, 0x98) << 0) | (i2c->read_reg_blocking(i2c_address, 0x99) << 8);
  dig_P7 = (i2c->read_reg_blocking(i2c_address, 0x9a) << 0) | (i2c->read_reg_blocking(i2c_address, 0x9b) << 8);
  dig_P8 = (i2c->read_reg_blocking(i2c_address, 0x9c) << 0) | (i2c->read_reg_blocking(i2c_address, 0x9d) << 8);
  dig_P9 = (i2c->read_reg_blocking(i2c_address, 0x9e) << 0) | (i2c->read_reg_blocking(i2c_address, 0x9f) << 8);
}

// Temperature returned will be in deg C
// Calculate pressure read calibration values
// b5 is also required so BMP180GetTemperature() must be called first.
// Value returned will be pressure in units of Pa.
// Altitude in meters
// Return 0 when no data, 1 when data available, -1 on timeout
int BMPx80::GetTPA(float dT, float *pTemp, float *pPress, float *pAlt)
{
  // start temperature measurement
  if (state == STATE_NONE)
  {
    i2c->write_reg_blocking(i2c_address, BMP280_CONTROL, (BMP280_TOS<<5) | (BMP280_POS<<2)    | BMP280_MODE); // start temperature measurement
    i2c->write_reg_blocking(i2c_address, BMP280_CONFIG,  (BMP280_SB<<5)  | (BMP280_FILTER<<2) | BMP280_SPI); // start temperature measurement
    state = STATE_PRESS_STARTED;
    duration = 0;
    interval = WAIT280_ALL;
    return 0;
  }

  if (state == STATE_PRESS_STARTED)
  {
    duration += dT;

    // wait for pressure measurement to complete
    if (duration < interval) {
      return 0;
    }

    // pressure measurement completed, read the data
    i2c->read_regs_nb(i2c_address, BMP280_PRESSURE, rawData, 6, &status); // read raw pressure measurement of 19 bits
    state = STATE_PRESS_READ;
    timeout = BMP_TIMEOUT;
    duration = 0;
    return 0;
  }

  if (state == STATE_PRESS_READ)
  {
    duration += dT;

    // wait for data to arrive
    if (!status) {
      if (duration < interval) {
        return -1;
      }
      return 0;
    }

    // data arrived, process it
    int temp_raw = (rawData[3] << 12) | (rawData[4] << 4) | (rawData[5] >> 4);
    int var1t = ((((temp_raw >> 3) - ((int)dig_T1 << 1))) * ((int)dig_T2)) >> 11;
    int var2t = (((((temp_raw >> 4) - ((int)dig_T1)) * ((temp_raw >> 4) - ((int)dig_T1))) >> 12) * ((int)dig_T3)) >> 14;
    int temp = var1t + var2t;
    int32_t t_fine = temp;
    temp = (temp * 5 + 128) >> 8;
    temperature = ((float)temp)*0.01f;

    uint32_t press_raw = (rawData[0] << 12) | (rawData[1] << 4) | (rawData[2] >> 4);
    int32_t var1, var2;
    uint32_t press;

    var1 = (t_fine >> 1) - 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * dig_P6;
    var2 = var2 + ((var1 * dig_P5) << 1);
    var2 = (var2 >> 2) + (dig_P4 << 16);
    var1 = (((dig_P3 * (((var1 >> 2)*(var1 >> 2)) >> 13)) >> 3) + ((dig_P2 * var1) >> 1)) >> 18;
    var1 = ((32768 + var1) * dig_P1) >> 15;

    if (var1) {

      press = (((1048576 - press_raw) - (var2 >> 12))) * 3125;

      if(press < 0x80000000)
          press = (press << 1) / var1;
      else
          press = (press / var1) * 2;

      var1 = ((int32_t)dig_P9 * ((int32_t)(((press >> 3) * (press >> 3)) >> 13))) >> 12;
      var2 = (((int32_t)(press >> 2)) * (int32_t)dig_P8) >> 13;
      press = (press + ((var1 + var2 + dig_P7) >> 4));
      pressure = ((float)press);

      int pa_idx = press/500;
      int pa_re  = press%500;
      pa_idx = ClipMinMax(pa_idx, 61, 226);
      float alt1 = PA2ALT[pa_idx-61];
      float alt2 = PA2ALT[pa_idx-61+1];
      altitude = (alt1*(500-pa_re) + alt2*pa_re+250)*0.002f;

      *pTemp  = temperature;
      *pPress = pressure;
      *pAlt   = altitude;
    }

    state = STATE_PRESS_STARTED;
    duration = 0;
    interval = WAIT280_ALL;
    return 1;
  }

  // should never get here
  state = STATE_NONE;
  return 0;
}

