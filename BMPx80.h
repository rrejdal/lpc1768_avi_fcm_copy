/**
  ******************************************************************************
  * @file    BMPx80.cpp
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
#ifndef _BMPx80_H_
#define _BMPx80_H_
 
#include "mbed.h"
#include "I2Ci.h"
#include "structures.h"

#define BMP_NONE    0
#define BMP_180     1
#define BMP_280     2

class BMPx80
{
  public:
    BMPx80(I2Ci *m_i2c);
    uint32_t Init(const ConfigData *pConfig);
    void Calibration280();
    int GetTPA(float dT, float *pTemp, float *pPress, float *pAlt);

  protected:
    I2Ci *i2c;
    int i2c_address;
    uint8_t OSS;

    // These are constants used to calculate the temperature and pressure from the BMP-280 sensor
    uint16_t    dig_T1;
    int16_t     dig_T2, dig_T3;
    uint16_t    dig_P1;
    int16_t     dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    char state;
    char bmp;
    float duration;
    float interval;
    float timeout;
    volatile char rawData[6];
    volatile int status;
    float temperature;
    float pressure;
    float altitude;
    volatile char start_cmd[2];
};

#endif
