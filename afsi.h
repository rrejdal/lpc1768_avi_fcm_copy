#ifndef _AFSI_H_
#define _AFSI_H_

#include "mbed.h"
#include "structures.h"
#include "telemetry.h"

#define MAX_AFSI_MSGS  10
#define AFSI_BUFFER_SIZE   (256+8+16)  // 16 for some slack

///////////AFSI_THINGS/////////////

// CLASS defines
#define AFSI_CMD_CLASS_CTRL       0x01
#define AFSI_CMD_CLASS_STAT       0x02
#define AFSI_CMD_CLASS_ACK        0x03

#define AFSI_HEADER_LEN           6
#define AFSI_SYCN_LEN             2
#define AFSI_CRC_LEN              2

// CTRL message defines
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

#define AFSI_CTRL_PAYL_LEN_ARM          0
#define AFSI_CTRL_PAYL_LEN_DISARM       0
#define AFSI_CTRL_PAYL_LEN_TAKEOFF      0
#define AFSI_CTRL_PAYL_LEN_LAND         2
#define AFSI_CTRL_PAYL_LEN_SET_POS      8
#define AFSI_CTRL_PAYL_LEN_SPEED_FWD    2
#define AFSI_CTRL_PAYL_LEN_SPEED_RIGHT  2
#define AFSI_CTRL_PAYL_LEN_SET_ALT      2
#define AFSI_CTRL_PAYL_LEN_HOME         2
#define AFSI_CTRL_PAYL_LEN_HOLD         0
#define AFSI_CTRL_PAYL_LEN_HEADING      4
#define AFSI_CTRL_PAYL_LEN_RESUME       0

// STATUS message defines
#define AFSI_STAT_ID_PWR                AFSI_STAT_PWR
#define AFSI_STAT_ID_GPS                AFSI_STAT_GPS
#define AFSI_STAT_ID_SEN                AFSI_STAT_SEN
#define AFSI_STAT_ID_FCM                AFSI_STAT_FCM

#define AFSI_STAT_PAYL_LEN_PWR          12
#define AFSI_STAT_PAYL_LEN_GPS          23
#define AFSI_STAT_PAYL_LEN_SEN          22
#define AFSI_STAT_PAYL_LEN_FCM          6

#define AFSI_NUM_STAT_MSGS              AFSI_MAX_STAT_MSGS

#define AFSI_RX_STAT_PAYL_LEN           1

// ACK message defines
#define AFSI_ACK_ID_NACK                0x00
#define AFSI_ACK_ID_ACK                 0x01

#define AFSI_ACK_PAYL_LEN               2

// SYNC bytes
#define AFSI_SYNC_BYTE_1                0xB5
#define AFSI_SYNC_BYTE_2                0x62

// STATE MACHINE
#define AFSI_STATE_INIT                 0
#define AFSI_STATE_SYNC                 1
#define AFSI_STATE_CLASS_ID             2
#define AFSI_STATE_LEN                  3
#define AFSI_STATE_READ                 4

// SCALING for different variables
#define AFSI_SCALE_POS                  1e-7
#define AFSI_SCALE_SPEED                1e-3
#define AFSI_SCALE_ALT                  1e-3
#define AFSI_SCALE_HEADING              1e-2

// Max and mins of variables
#define AFSI_MAX_SPEED            10/AFSI_SCALE_SPEED
#define AFSI_MIN_SPEED           -10/AFSI_SCALE_SPEED

#define AFSI_MAX_ALT              50/AFSI_SCALE_ALT
#define AFSI_MIN_ALT               5/AFSI_SCALE_ALT

#define AFSI_MAX_HEADING          180/AFSI_SCALE_HEADING
#define AFSI_MIN_HEADING         -180/AFSI_SCALE_HEADING

typedef struct AFSI_MSG{
    const uint8_t sync1 = AFSI_SYNC_BYTE_1;
    const uint8_t sync2 = AFSI_SYNC_BYTE_2;
    uint8_t   msg_class;
    uint8_t   id;
    uint16_t  len;
    uint8_t   payload[200];
    uint8_t   crc[2];
}AFSI_MSG;

typedef struct AFSI_ACK_MSG{
    const uint8_t sync1     = AFSI_SYNC_BYTE_1;
    const uint8_t sync2     = AFSI_SYNC_BYTE_2;
    const uint8_t msg_class = AFSI_CMD_CLASS_ACK;
    const uint8_t id        = AFSI_ACK_ID_ACK;
    const uint16_t len      = AFSI_ACK_PAYL_LEN;
    uint8_t    ackd_msg_class;
    uint8_t    ackd_msg_id;
    uint8_t    crc[2];
}AFSI_ACK_MSG;

typedef struct AFSI_NACK_MSG{
    const uint8_t  sync1     = AFSI_SYNC_BYTE_1;
    const uint8_t  sync2     = AFSI_SYNC_BYTE_2;
    const uint8_t  msg_class = AFSI_CMD_CLASS_ACK;
    const uint8_t  id        = AFSI_ACK_ID_NACK;
    const uint16_t len       = AFSI_ACK_PAYL_LEN;
    uint8_t    nackd_msg_class;
    uint8_t    nackd_msg_id;
    uint8_t    crc[2];
}AFSI_NACK_MSG;

typedef struct AFSI_STAT_MSG_PWR{
    const uint8_t  sync1     = AFSI_SYNC_BYTE_1;
    const uint8_t  sync2     = AFSI_SYNC_BYTE_2;
    const uint8_t  msg_class = AFSI_CMD_CLASS_STAT;
    const uint8_t  id        = AFSI_STAT_ID_PWR;
    const uint16_t len       = AFSI_STAT_PAYL_LEN_PWR;
    uint16_t   voltage;
    uint16_t   current;
    uint16_t   total_bat_capacity;
    uint16_t   bat_capacity_used;
    uint16_t   bat_percent_used;
    uint16_t   flight_time;
    uint8_t    crc[2];
}AFSI_STAT_MSG_PWR;

typedef struct AFSI_STAT_MSG_GPS{
    const uint8_t  sync1     = AFSI_SYNC_BYTE_1;
    const uint8_t  sync2     = AFSI_SYNC_BYTE_2;
    const uint8_t  msg_class = AFSI_CMD_CLASS_STAT;
    const uint8_t  id        = AFSI_STAT_ID_GPS;
    const uint16_t len       = AFSI_STAT_PAYL_LEN_GPS;
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
    uint8_t    crc[2];
}AFSI_STAT_MSG_GPS;

typedef struct AFSI_STAT_MSG_SEN{
    const uint8_t  sync1     = AFSI_SYNC_BYTE_1;
    const uint8_t  sync2     = AFSI_SYNC_BYTE_2;
    const uint8_t  msg_class = AFSI_CMD_CLASS_STAT;
    const uint8_t  id        = AFSI_STAT_ID_SEN;
    const uint16_t len       = AFSI_STAT_PAYL_LEN_SEN;
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
    uint8_t    crc[2];
}AFSI_STAT_MSG_SEN;

typedef struct AFSI_STAT_MSG_FCM{
    const uint8_t sync1     = AFSI_SYNC_BYTE_1;
    const uint8_t sync2     = AFSI_SYNC_BYTE_2;
    const uint8_t msg_class = AFSI_CMD_CLASS_STAT;
    const uint8_t id        = AFSI_STAT_ID_FCM;
    const uint16_t len      = AFSI_STAT_PAYL_LEN_FCM;
    uint8_t    ctrl_mode_pitch;
    uint8_t    ctrl_mode_roll;
    uint8_t    ctrl_mode_yaw;
    uint8_t    ctrl_mode_collective;
    uint8_t    ctrl_mode_throttle;
    int8_t     ctrl_status;
    uint8_t    crc[2];
}AFSI_STAT_MSG_FCM;


class AFSI_Serial
{
public:
    unsigned int afsi_good_messages;
    unsigned int afsi_crc_errors;
    unsigned int afsi_start_code_searches;

    float ctrl_out[10];
    int stat_msg_enable[AFSI_MAX_STAT_MSGS];
    int stat_msg_period[AFSI_MAX_STAT_MSGS];
    int stat_msg_cnt[AFSI_MAX_STAT_MSGS];

    AFSI_Serial(RawSerial *m_serial, TelemSerial *m_telem);

    void ProcessInputBytes(RawSerial &telemetry);
    void GenerateStatMsg(int id);
    void SendMsgs();
    bool IsTypeInQ(int type);

private:
    typedef struct {
        uint8_t *msg;
        int      size;
        int      msg_type;
    } AFSI_MSG_ENTRY;

    RawSerial   *afsi_serial;
    TelemSerial *telem;

    AFSI_MSG_ENTRY curr_msg;
    AFSI_MSG_ENTRY msg_queue[MAX_AFSI_MSGS];
    int            num_msgs_in_q;

    AFSI_MSG          msg_afsi;

    AFSI_STAT_MSG_PWR msg_stat_pwr;
    AFSI_STAT_MSG_GPS msg_stat_gps;
    AFSI_STAT_MSG_SEN msg_stat_sensors;
    AFSI_STAT_MSG_FCM msg_stat_fcm;

    AFSI_ACK_MSG      msg_ack;
    AFSI_NACK_MSG     msg_nack;

    unsigned int afsi_rx_bytes;
    uint8_t  afsi_rx_buffer[AFSI_BUFFER_SIZE];
    uint32_t rx_payload_len;
    int      rx_msg_state;
    int      rx_msg_rdy;
    const uint32_t ctrl_msg_lengths[AFSI_NUM_CTRL_MSGS] = {
            AFSI_CTRL_PAYL_LEN_ARM,
            AFSI_CTRL_PAYL_LEN_DISARM,
            AFSI_CTRL_PAYL_LEN_TAKEOFF,
            AFSI_CTRL_PAYL_LEN_LAND,
            AFSI_CTRL_PAYL_LEN_SET_POS,
            AFSI_CTRL_PAYL_LEN_SPEED_FWD,
            AFSI_CTRL_PAYL_LEN_SPEED_RIGHT,
            AFSI_CTRL_PAYL_LEN_SET_ALT,
            AFSI_CTRL_PAYL_LEN_HOME,
            AFSI_CTRL_PAYL_LEN_SET_ALT,
            AFSI_CTRL_PAYL_LEN_HEADING,
            AFSI_CTRL_PAYL_LEN_RESUME
    };

    void AddInputByte(char ch);
    int  ProcessAsfiCtrlCommands(AFSI_MSG *msg);
    int  ProcessAsfiStatusCommands(AFSI_MSG *msg);
    void EnableAFSI(void);
    void ResetRxMsgData(void);

    void GeneratePwrStatus(void);
    void GenerateGpsStatus(void);
    void GenerateSensorsStatus(void);
    void GenerateFcmStatus(void);
    void GenerateNACK(int msg_class, int msg_id);
    void GenerateACK(int msg_class, int msg_id);

    bool QueueMsg(uint8_t *msg, int size, int msg_type);

    int  GetCRC(uint8_t *data, int len, uint8_t *CRC);

    float processU2(uint8_t*data, int scaling);
    bool CheckRangeI(int value, int vmin, int vmax);

};

#endif
