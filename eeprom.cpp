#include "eeprom.h"
#include <math.h>
#include "version.h"


EEPROM::EEPROM(I2Ci *m_i2c, EEPROM_TYPE m_type)
{
    i2c = m_i2c;

    data.calib_date = 0;
    for(int i = 0; i<3; i++) {
        for(int j = 0; j<3; j++) {
                data.acc_calib_matrix[i][j] = 0;
                data.gyro_calib_matrix[i][j] = 0;
            }
        data.acc_offsets[i] = 0;
        data.gyroP_temp_coeffs[i] = 0;
        data.gyroR_temp_coeffs[i] = 0;
        data.gyroY_temp_coeffs[i] = 0;
    }

    data.serial_num = 0;

    calib_year = 0;
    calib_month = 0;
    calib_day = 0;

    board_year = 0;
    board_type = 0;
    id_num = 0;

    type = m_type;
}

int EEPROM::write_data()
{
    char *pdata;
    char paddr[2];

    memcpy(&(buffer[0]),&data,EEPROM_DATA_SIZE);
    readSerialNum();

    //Run CRC on all data and append to the end of the buffer
    getCRC(&(buffer[0]), EEPROM_DATA_SIZE - 2, &(buffer[EEPROM_DATA_SIZE-2]));

    data.crc[0] = buffer[EEPROM_DATA_SIZE-2];
    data.crc[1] = buffer[EEPROM_DATA_SIZE-1];
    printf("crc = %d-%d\r\n",data.crc[0],data.crc[1]);

    // Write the buffer to the EEPROM
    for(int i = 0; i<128; i++) {
        pdata = (char*)&(buffer[i]);

        if(type == IMU_EEPROM) {
            i2c->write_blocking(I2C_ADDR_EEPROM,(EEPROM_ADDR_DATA_START+i),pdata,1);
        }
        else if (type == FCM_EEPROM) {
            paddr[0] = 0;
            paddr[1] = EEPROM_ADDR_DATA_START+i;
            i2c->write_blocking(I2C_ADDR_EEPROM,paddr,2,pdata,1);
        }
        else {
            return 0;
        }
    }

    return 1;
}

int EEPROM::read_data()
{
    uint8_t CRC[2];
    char paddr[2];

    // Initialize buffers
    memset((void *)buffer, 0x00, sizeof(buffer));
    memset(&data, 0x00, sizeof(data));

    //Read every byte in the EEPROM
    for (int i = 0; i < EEPROM_SIZE; i++) {
        if (type == IMU_EEPROM) {
            buffer[i] = i2c->read_reg_blocking(I2C_ADDR_EEPROM,EEPROM_ADDR_DATA_START+i);
        }
        else if (type == FCM_EEPROM) {
            paddr[0] = 0;
            paddr[1] = EEPROM_ADDR_DATA_START+i;
            buffer[i] = i2c->read_reg_blocking(I2C_ADDR_EEPROM,paddr,2);
        }
        else {
            return -2;
        }
    }

    getCRC(&(buffer[0]), EEPROM_DATA_SIZE - 2, &(CRC[0]));

    if ((CRC[0] != buffer[EEPROM_DATA_SIZE - 2])
            || (CRC[1] != buffer[EEPROM_DATA_SIZE - 1])) {

        if (data.serial_num !=0) {
            //printf("===>ERROR: CRC FAIL!\r\n");
            //printf("           CRC = %d-%d   buffer = %d-%d\r\n",CRC[0],CRC[1],
            //        buffer[EEPROM_DATA_SIZE - 2],buffer[EEPROM_DATA_SIZE - 1]);
            return -1;
        }
    }

    memcpy(&data,&(buffer[0]),EEPROM_DATA_SIZE);

    if (readSerialNum() != 0) {
        return -3;
    }

    return 0;
}

int EEPROM::clear()
{
    char null_data = 0xFF;
    char paddr[2];

    // Write the null data to the EEPROM
    for(int i = 0; i<128; i++) {
        if(type == IMU_EEPROM) {
            i2c->write_blocking(I2C_ADDR_EEPROM,EEPROM_ADDR_DATA_START+i,&null_data,1);
        }
        else if (type == FCM_EEPROM) {
            paddr[0] = 0;
            paddr[1] = EEPROM_ADDR_DATA_START+i;
            i2c->write_blocking(I2C_ADDR_EEPROM,paddr,2,&null_data,1);
        }
        else {
            return 0;
        }
    }

    return 1;
}

void EEPROM::print_data()
{
    printf("data:\r\n"
           "- serial num = %d\r\n"
           "- date = %d\r\n"
           "- acc calib matrix  = [%3.6f %3.6f %3.6f]\r\n"
           "                      [%3.6f %3.6f %3.6f]\r\n"
           "                      [%3.6f %3.6f %3.6f]\r\n\r\n"
           "- acc offsets = [%3.6f %3.6f %3.6f]\r\n\r\n"
           "- gyro calib matrix = [%3.6f %3.6f %3.6f]\r\n"
           "                      [%3.6f %3.6f %3.6f]\r\n"
           "                      [%3.6f %3.6f %3.6f]\r\n\r\n"
           "- gyroX temp offset coeffs = [%3.6f %3.6f %3.6f]\r\n"
           "- gyroY temp offset coeffs = [%3.6f %3.6f %3.6f]\r\n"
           "- gyroZ temp offset coeffs = [%3.6f %3.6f %3.6f]\r\n\r\n"
           "- crc0 = %d\r\n"
           "- crc1 = %d\r\n",
           data.serial_num,
           data.calib_date,
           data.acc_calib_matrix[0][0],data.acc_calib_matrix[0][1],data.acc_calib_matrix[0][2],
           data.acc_calib_matrix[1][0],data.acc_calib_matrix[1][1],data.acc_calib_matrix[1][2],
           data.acc_calib_matrix[2][0],data.acc_calib_matrix[2][1],data.acc_calib_matrix[2][2],
           data.acc_offsets[0],data.acc_offsets[1],data.acc_offsets[2],
           data.gyro_calib_matrix[0][0],data.gyro_calib_matrix[0][1],data.gyro_calib_matrix[0][2],
           data.gyro_calib_matrix[1][0],data.gyro_calib_matrix[1][1],data.gyro_calib_matrix[1][2],
           data.gyro_calib_matrix[2][0],data.gyro_calib_matrix[2][1],data.gyro_calib_matrix[2][2],
           data.gyroP_temp_coeffs[0],data.gyroP_temp_coeffs[1],data.gyroP_temp_coeffs[2],
           data.gyroR_temp_coeffs[0],data.gyroR_temp_coeffs[1],data.gyroR_temp_coeffs[2],
           data.gyroY_temp_coeffs[0],data.gyroY_temp_coeffs[1],data.gyroY_temp_coeffs[2],
           data.crc[0],data.crc[1]
    );
    return;
}

void EEPROM::getSerialNum()
{
    data.serial_num =  (board_year*10000000)
                     + (board_type*10000)
                     +  id_num;
}

int EEPROM::readSerialNum()
{
    board_year = 2000 + data.serial_num/10000000;
    board_type = ( data.serial_num - ((board_year-2000)*10000000) )/10000;
    id_num     =   data.serial_num - ((board_year-2000)*10000000) - (board_type*10000);

    if ((id_num == 0) || (board_type != PN_IMU)) {
        return -1;
    }

    return 0;
}

void EEPROM::getDate()
{
    calib_year   = (data.calib_date & 0b1111111000000000) >> 9;
    calib_month  = (data.calib_date & 0b0000000111100000) >> 5;
    calib_day    = (data.calib_date & 0b0000000000011111);

    return;
}

void EEPROM::saveDate()
{
    data.calib_date = ((uint8_t)calib_year << 9)
                    + ((uint8_t)calib_month << 5)
                    +  (uint8_t)calib_day;
    return;
}

int EEPROM::getCRC(uint8_t *data, int len, uint8_t *CRC)
{
    int i;
    uint8_t CK_A = 0;
    uint8_t CK_B = 0;

    for (i=0; i<len; i++) {
        CK_A = CK_A + data[i];
        CK_B = CK_B + CK_A;
    }

    CRC[0] = CK_A;
    CRC[1] = CK_B;

    if (CK_A != data[len+0]) {
        return 0;
    }

    if (CK_B != data[len+1]) {
        return 0;
    }

    return 1;
}
