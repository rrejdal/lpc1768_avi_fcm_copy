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

#define AFSI_CMD_ID_ARM           0x00
#define AFSI_CMD_ID_DISARM        0x01
#define AFSI_CMD_ID_TAKEOFF       0x02`
#define AFSI_CMD_ID_LAND          0x03
#define AFSI_CMD_ID_SET_POS       0x04
#define AFSI_CMD_ID_SPEED_FWD     0x05
#define AFSI_CMD_ID_SPEED_AFT     0x06
#define AFSI_CMD_ID_SPEED_RIGHT   0x07
#define AFSI_CMD_ID_SPEED_LEFT    0x08
#define AFSI_CMD_ID_SET_ALT       0x09
#define AFSI_CMD_ID_HOME          0x0A
#define AFSI_CMD_ID_HOLD          0x0B
#define AFSI_CMD_ID_HEADING       0x0C

#define AFSI_CMD_ID_STAT_POW      0x00
#define AFSI_CMD_ID_STAT_GPS      0x01
#define AFSI_CMD_ID_STAT_OP       0x02
#define AFSI_CMD_ID_STAT_FCM      0x03

#define AFSI_SNC_CH_1             0xB5
#define AFSI_SNC_CH_2             0x62

#define AFSI_STATE_INIT           0
#define AFSI_STATE_SYNC           1
#define AFSI_STATE_CLASS          2
#define AFSI_STATE_ID             3
#define AFSI_STATE_LEN            4
#define AFSI_STATE_MSG            5
#define AFSI_STATE_DONE           6

#define AFSI_SCALE_POS            1e-7
#define AFSI_SCALE_SPEED          1e-3
#define AFSI_SCALE_ALT            1e-3
#define AFSI_SCALE_HEADING        1e-2

#define AFSI_MAX_SPEED            10/AFSI_SCALE_SPEED
#define AFSI_MIN_SPEED            0

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
    AFSI_HDR * hdr = new AFSI_HDR;
    uint8_t payload[200];
    uint8_t crc[2];
}AFSI_MSG;

typedef struct{
    int     latitude;
    int     longitude;
}AFSI_POS_DATA;


typedef struct{
    int     speed;
}AFSI_SPEED_DATA;

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

    RawSerial *serial;
    TelemSerial *telem;

    T_Tentry curr_msg;
    T_Tentry Q[MAX_AFSI_MESSAGES];
    unsigned char messages;

    unsigned int afsi_recv_bytes;
    byte afsi_recv_buffer[AFSI_BUFFER_SIZE];

    AFSI_MSG *msg_afsi = new AFSI_MSG;
    AFSI_HDR *hdr_afsi= msg_afsi->hdr;

    int ProcessAsfiCtrlCommands(AFSI_MSG *msg);
    int ProcessAsfiStatusCommands(AFSI_MSG *msg);
    int AFSI_Serial::EnableAFSI(void);
    int GetCRC(uint8_t *data, int len, uint8_t *CRC);

    unsigned int RemoveBytes(unsigned char *b, int remove, int size);

    bool CheckRangeAndSetI(int *pvalue, uint8_t *pivalue, float vmin, float vmax)
    bool CheckRangeAndSetF(float *pvalue, byte *pivalue, float vmin, float vmax);
    bool CheckRangeI(int value, int vmin, int vmax);
    bool CheckRangeAndSetB(byte *pvalue, byte *pivalue, int vmin, int vmax);

};

#endif
