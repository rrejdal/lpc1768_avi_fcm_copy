#ifndef _AFSI_H_
#define _AFSI_H_

#include "mbed.h"
#include "structures.h"
#include "telemetry.h"

#define MAX_AFSI_MESSAGES  10
#define AFSI_BUFFER_SIZE   (256+8+16)  // 16 for some slack

///////////AFSI_THINGS/////////////
#define AFSI_CMD_CLASS_CTRL       0x01
#define AFSI_CMD_CLASS_STAT       0x02
#define AFSI_CMD_CLASS_ACK        0x03

#define AFSI_CTRL_ID_ARM          0x00
#define AFSI_CTRL_ID_DISARM       0x01
#define AFSI_CTRL_ID_TAKEOFF      0x02
#define AFSI_CTRL_ID_LAND         0x03
#define AFSI_CTRL_ID_SET_POS      0x04
#define AFSI_CTRL_ID_SPEED_FWD    0x05
#define AFSI_CTRL_ID_SPEED_RIGHT  0x06
#define AFSI_CTRL_ID_SET_ALT      0x07
#define AFSI_CTRL_ID_HOME         0x08
#define AFSI_CTRL_ID_HOLD         0x09
#define AFSI_CTRL_ID_HEADING      0x0A
#define AFSI_CTRL_ID_RESUME       0x0B

#define AFSI_NUM_CTRL_MSGS        12

#define AFSI_PAYLOAD_LEN_ARM          0
#define AFSI_PAYLOAD_LEN_DISARM       0
#define AFSI_PAYLOAD_LEN_TAKEOFF      0
#define AFSI_PAYLOAD_LEN_LAND         2
#define AFSI_PAYLOAD_LEN_SET_POS      8
#define AFSI_PAYLOAD_LEN_SPEED_FWD    2
#define AFSI_PAYLOAD_LEN_SPEED_RIGHT  2
#define AFSI_PAYLOAD_LEN_SET_ALT      2
#define AFSI_PAYLOAD_LEN_HOME         2
#define AFSI_PAYLOAD_LEN_HOLD         0
#define AFSI_PAYLOAD_LEN_HEADING      4
#define AFSI_PAYLOAD_LEN_RESUME       0

#define AFSI_STAT_ID_POW          0x00
#define AFSI_STAT_ID_GPS          0x01
#define AFSI_STAT_ID_SENSORS      0x02
#define AFSI_STAT_ID_FCM          0x03

#define AFSI_NUM_STAT_MSGS        4
#define AFSI_RX_STAT_PAYLOAD      0

#define AFSI_SYNC_BYTE_1          0xB5
#define AFSI_SYNC_BYTE_2          0x62

#define AFSI_STATE_INIT           0
#define AFSI_STATE_SYNC           1
#define AFSI_STATE_CLASS_ID       2
#define AFSI_STATE_LEN            3
#define AFSI_STATE_READ           4

#define AFSI_SCALE_POS            1e-7
#define AFSI_SCALE_SPEED          1e-3
#define AFSI_SCALE_ALT            1e-3
#define AFSI_SCALE_HEADING        1e-2

#define AFSI_MAX_SPEED            10/AFSI_SCALE_SPEED
#define AFSI_MIN_SPEED           -10/AFSI_SCALE_SPEED

#define AFSI_MAX_ALT              50/AFSI_SCALE_ALT
#define AFSI_MIN_ALT               5/AFSI_SCALE_ALT

#define AFSI_MAX_HEADING          180/AFSI_SCALE_HEADING
#define AFSI_MIN_HEADING         -180/AFSI_SCALE_HEADING

typedef struct{
    uint8_t sync1;
    uint8_t sync2;
    uint8_t msg_class;
    uint8_t id;
    uint16_t len;
}AFSI_HDR;

typedef struct{
    AFSI_HDR   hdr;
    uint8_t    payload[200];
    uint8_t    crc[2];
}AFSI_MSG;

typedef struct{
    AFSI_HDR*  hdr;
    uint16_t   voltage;
    uint16_t   current;
    uint16_t   total_bat_capacity;
    uint16_t   bat_capacity_used;
    uint16_t   bat_percent_used;
    uint16_t   flight_time;
}AFSI_PWR_STATUS;

typedef struct{
    AFSI_HDR*  hdr;
    uint16_t   hour;
    uint8_t    min;
    uint8_t    sec;
    uint16_t   year;
    uint8_t    month;
    uint8_t    day;
    int32_t    altitude;
    int32_t    latitude;
    int32_t    longitude;
    uint16_t   pDOP;
    uint8_t    numSV;
}AFSI_GPS_STATUS;

typedef struct{
    AFSI_HDR*  hdr;
    uint16_t   gyro_temp;
    uint16_t   baro_temp;
    uint16_t   esc_temp;
    uint32_t   baro_pressure;
    uint16_t   wind_speed;
    uint16_t   wind_course;
    uint16_t   rpm;
    uint16_t   lidar_altitude;
    uint16_t   compass_heading;
    uint16_t   altitude;

}AFSI_SENSORS_STATUS;

typedef struct{
    AFSI_HDR*  hdr;
    uint8_t    ctrl_mode_pitch;
    uint8_t    ctrl_mode_roll;
    uint8_t    ctrl_mode_yaw;
    uint8_t    ctrl_mode_collective;
    uint8_t    ctrl_mode_throttle;
    int8_t     ctrl_status;
}AFSI_FCM_STATUS;



class AFSI_Serial
{
public:
    unsigned int afsi_good_messages;
    unsigned int afsi_crc_errors;
    unsigned int afsi_start_code_searches;

    AFSI_Serial(RawSerial *m_serial, TelemSerial *m_telem);

    uint8_t processU1(uint8_t data, int scaling);
    float processU2(uint8_t*data, int scaling);
    uint8_t processI1(char*data, int len, float scaling);

    /* receiving stuff */
    void ProcessCommands(void);
    bool AddMessage(unsigned char *m_msg, int m_size, unsigned char m_msg_type, unsigned char m_priority);
    void Update();
    bool IsEmpty();
    bool IsTypeInQ(unsigned char type);
    void ProcessInputBytes(RawSerial &telemetry);
    void AddInputByte(char ch);

private:
    typedef struct {
        unsigned char *msg;
        int            size;
        unsigned char  priority;    // 0-lowest
        unsigned char  msg_type;
    } T_Tentry;

//    RawSerial *serial;
    TelemSerial *telem;

    T_Tentry curr_msg;
    T_Tentry Q[MAX_AFSI_MESSAGES];
    unsigned char messages;

    AFSI_MSG             msg_afsi;
    AFSI_HDR             afsi_hdr;

    AFSI_FCM_STATUS      msg_fcm_status;
    AFSI_SENSORS_STATUS  msg_sensors_status;
    AFSI_GPS_STATUS      msg_gps_status;
    AFSI_PWR_STATUS      msg_pwr_status;

    unsigned int afsi_rx_bytes;
    uint8_t  afsi_rx_buffer[AFSI_BUFFER_SIZE];
    uint32_t rx_payload_len;
    int      rx_msg_state;
    int      rx_msg_rdy;
    const uint32_t ctrl_msg_lengths[AFSI_NUM_CTRL_MSGS] = {
            AFSI_PAYLOAD_LEN_ARM,
            AFSI_PAYLOAD_LEN_DISARM,
            AFSI_PAYLOAD_LEN_TAKEOFF,
            AFSI_PAYLOAD_LEN_LAND,
            AFSI_PAYLOAD_LEN_SET_POS,
            AFSI_PAYLOAD_LEN_SPEED_FWD,
            AFSI_PAYLOAD_LEN_SPEED_RIGHT,
            AFSI_PAYLOAD_LEN_SET_ALT,
            AFSI_PAYLOAD_LEN_HOME,
            AFSI_PAYLOAD_LEN_SET_ALT,
            AFSI_PAYLOAD_LEN_HEADING,
            AFSI_PAYLOAD_LEN_RESUME
    };

    int  ProcessAsfiCtrlCommands(AFSI_MSG *msg);
    int  ProcessAsfiStatusCommands(AFSI_MSG *msg);
    void EnableAFSI(void);
    int  GetCRC(uint8_t *data, int len, uint8_t *CRC);
    void ResetRxMsgData(void);

    void InitStatusHdr(void);
    void GeneratePwrStatus(void);
    void GenerateGpsStatus(void);
    void GenerateSensorsStatus(void);
    void GenerateFcmStatus(void);

    unsigned int RemoveBytes(unsigned char *b, int remove, int size);

    bool CheckRangeAndSetI(int *pvalue, uint8_t *pivalue, float vmin, float vmax);
    bool CheckRangeAndSetF(float *pvalue, byte *pivalue, float vmin, float vmax);
    bool CheckRangeI(int value, int vmin, int vmax);
    bool CheckRangeAndSetB(byte *pvalue, byte *pivalue, int vmin, int vmax);

};

#endif
