#ifndef _BMP180_H_
#define _BMP180_H_
 
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
    bool Init(const ConfigData *pConfig);
    void Calibration180();
    void Calibration280();
    bool GetTPA(float dT, float *pTemp, float *pPress, float *pAlt);

  protected:
    I2Ci *i2c;
    int i2c_address;
    // These are constants used to calculate the temperature and pressure from the BMP-180 sensor
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md, b5;  
    uint16_t ac4, ac5, ac6;
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
    volatile char rawData[6];
    volatile int status;
    float temperature;
    float pressure;
    float altitude;
    volatile char start_cmd[2];
};

#if 0
class BMP280
{
  public:
    BMP280(I2Ci *m_i2c);
    bool Init();
    void Calibration();
    bool GetTPA(float dT, float *pTemp, float *pPress, float *pAlt);

  protected:
    I2Ci *i2c;
    int i2c_address;
    // These are constants used to calculate the temperature and pressure from the BMP-280 sensor
    uint16_t    dig_T1;
    int16_t     dig_T2, dig_T3;
    uint16_t    dig_P1;
    int16_t     dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    char state;
    float duration;
    float interval;
    volatile char rawData[6];
    volatile int status;
    float temperature;
    float pressure;
    float altitude;
    volatile char start_cmd[2];
};
#endif

#endif
