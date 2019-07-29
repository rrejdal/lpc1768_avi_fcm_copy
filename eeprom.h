#ifndef _EERPOM_H_
#define _EEPROM_H_


#define I2C_ADDR_EEPROM               0xA0

#define EEPROM_ADDR_DATA_START        0x00

#define EEPROM_ADDR_HEADER_DATA       0x00
#define EEPROM_ADDR_ACC_CALIB         EEPROM_HEADER_SIZE + EEPROM_HEADER_SIZE
#define EEPROM_ADDR_GYRO_MATRIX       EEPROM_ADDR_ACC_CALIB + EEPROM_ACC_CALIB_DATA_SIZE
#define EEPROM_ADDR_GYRO_TEMP_COEFFS  EEPROM_ADDR_GYRO_MATRIX + EEPROM_GYRO_MATRIX_SIZE
#define EEPROM_ADDR_CRC               EEPROM_ADDR_GYRO_TEMP_COEFFS + EEPROM_GYRO_TEMP_DATA_SIZE

#define EEPROM_HEADER_SIZE            6  // year, board_type, id_number, date
#define EEPROM_ACC_CALIB_DATA_SIZE   48  // 9 floats for matrix + 3 floats for offsets = 9*4+3*4 = 48
#define EEPROM_GYRO_MATRIX_SIZE      36  // 9 floats for matrix = 9*4 = 36
#define EEPROM_GYRO_TEMP_DATA_SIZE   36  // + 3*(3 floats for coeffs) = 3*3*4 = 36
#define EEPROM_CRC_DATA_SIZE          2  // 2 bytes
#define EEPROM_DATA_SIZE            128  // 128 bytes of data currently used

#define EEPROM_SIZE                 128 // 128 bytes, 0x7F is the last byte address

// The calibration matrix for each sensor is stored back to back
//    EEPROM_ADDR_---_CALIB_ROW_COLUMN
// i.e. if ROW = X and Column = X, then the value stored in
// EEPROM_ADDR_ACC_CALIB_X_X is the gain in the X-direction.
// if ROW = Y and Column = Z, then the value stored in
// EEPROM_ADDR_ACC_CALIB_Y_Z is the cross gain affect of Z on the Y axis
//                R    C      C0  C1  C1
// CALIB_MATRIX[row][col] = [ XX  XY  XZ ]  R0
//                          [ YX  YY  YZ ]  R1
//                          [ ZX  ZY  ZZ ]  R2

#include "I2Ci.h"

enum EEPROM_TYPE {
    FCM_EEPROM = 0, /*FCM EEPROM = 24AA256
                     * EEPROM: 24AA256
                     * 2 bytes are needed for the register since this
                     * eeprom is so big 256kbit = 32k bytes, 0x0000 to 0x7D00*/
    IMU_EEPROM = 1, /*FCM EEPROM = 24AA256
                     * EEPROM: 24LC01B
                     * only 1 byte needed to address EEPROM memory since the size is
                     * only 1kbit = 128 bytes, address 0x00 to 0x7F */
};

class EEPROM
{
public:
    EEPROM(I2Ci *m_i2c, EEPROM_TYPE m_type);

    int  read_data();
    int  write_data();
    int  clear();
    void print_data();
    void getSerialNum();
    int readSerialNum();
    void saveDate();
    void getDate();

    struct EEPROMdata {
//      type       ---variable name------  -----bytes----   -----notes----------------------------
      unsigned int serial_num;             //           4  YYBBBNNNN, Y=board_year, B=board_type, N= id_num
      uint16_t     calib_date;             //           2   Date of Calibration, 2 byte format: YYYY YYYM MMMD DDDD, Y=year bits,M=month bits,D=day bits
      float        acc_calib_matrix[3][3]; // 3*3*4 =  36   CALIB_MATRIX[row][col] = [ XX  XY  XZ ]
      float        acc_offsets[3];         //   3*4 =  12                            [ YX  YY  YZ ]
      float        gyro_calib_matrix[3][3];// 3*3*4 =  36                            [ ZX  ZY  ZZ ]
      float        gyroP_temp_coeffs[3];   //   3*4 =  12   coefficients for temp compensation for
      float        gyroR_temp_coeffs[3];   //   3*4 =  12   gyro pitch (P), roll (R) and yaw (Y)
      float        gyroY_temp_coeffs[3];   //   3*4 =  12   - quadratic model: offset = aT^2+bT+c
      uint8_t      crc[2];                 //   2*1 =   2   - coefficients are a, b and c
//                                        Total bytes = 128
    }__attribute__((packed));

    EEPROMdata data;

    int calib_year; // year of calibration
    int calib_month;// month of calibration
    int calib_day;  // day of calibration

    int board_year; // year the board was manufactured
    int board_type; // type of board
    int id_num;     // type of board

    EEPROM_TYPE type;  // type 0 = IMU EEPROM = 24LC01B, type 1 = FCM EEPROM = 24AA256



private:
    int getCRC(uint8_t *data, int len, uint8_t *CRC);

    I2Ci *i2c;
    uint8_t buffer[EEPROM_SIZE];  // the 24LC01B EEPROM has storage of 1Kbit = 128 bytes = 32 ints/floats
};

#endif
