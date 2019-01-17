#ifndef _AFSI_H_
#define _AFSI_H_

#include "mbed.h"
#include "structures.h"

#define MAX_AFSI_MESSAGES  10
#define AFSI_BUFFER_SIZE   (256+8+16)  // 16 for some slack

///////////AFSI_THINGS/////////////
#define AFSI_CMD_CLASS_CTRL     0x01
#define AFSI_CMD_CLASS_STAT     0x02
#define AFSI_CMD_CLASS_ACK      0x03

#define AFSI_CMD_ID_ARM             0x00
#define AFSI_CMD_ID_DISARM          0x01
#define AFSI_CMD_ID_TAKEOFF         0x02
#define AFSI_CMD_ID_LAND            0x03
#define AFSI_CMD_ID_SET_POS         0x04
#define AFSI_CMD_ID_FWD_SPEED       0x05
#define AFSI_CMD_ID_AFT_SPEED       0x06
#define AFSI_CMD_ID_RIGHT_SPEED     0x07
#define AFSI_CMD_ID_LEFT_SPEED      0x08
#define AFSI_CMD_ID_SET_ALT         0x09
#define AFSI_CMD_ID_HOME            0x0A
#define AFSI_CMD_ID_HOLD            0x0B
#define AFSI_CMD_ID_HEADING         0x0C

#define AFSI_CMD_ID_STAT_POW    0x00
#define AFSI_CMD_ID_STAT_GPS    0x01
#define AFSI_CMD_ID_STAT_OP     0x02
#define AFSI_CMD_ID_STAT_FCM    0x03

#define AFSI_SNC_CH_1           0xB5
#define AFSI_SNC_CH_2           0x62

#define AFSI_STATE_INIT         0
#define AFSI_STATE_SYNC         1
#define AFSI_STATE_CLASS        2
#define AFSI_STATE_ID           3
#define AFSI_STATE_LEN          4
#define AFSI_STATE_MSG          5
#define AFSI_STATE_DONE         6

typedef struct{
    uint8_t sync1;
    uint8_t sync2;
    uint8_t msg_class;
    uint8_t id;
    uint16_t len;
}TELEM_AFSI_HDR;

typedef struct{
    TELEM_AFSI_HDR * hdr = new TELEM_AFSI_HDR;
    uint8_t payload[200];
    uint8_t crc[2];
}TELEM_AFSI_MSG;


class AFSI_Serial
{
public:
    unsigned int afsi_good_messages;
    unsigned int afsi_crc_errors;
    unsigned int afsi_start_code_searches;

    AFSI_Serial(RawSerial *m_serial);

    uint8_t processU1(uint8_t data, int scaling);
    unsigned short processU2(uint8_t*data, int scaling);
    uint8_t processI1(char*data, int len, float scaling);

    void Initialize(FlightControlData *p_hfc, const ConfigData *p_config) { hfc = p_hfc, pConfig = p_config; }
    /* receiving stuff */
    void ProcessCommands(void);
    bool AddMessage(unsigned char *m_msg, int m_size, unsigned char m_msg_type, unsigned char m_priority);
    void Update();
    bool IsEmpty();
    bool IsTypeInQ(unsigned char type);
    void ProcessInputBytes(RawSerial &telemetry);
    void AddInputByte(char ch);

    /* transmit stuff */
    void SendMsgToGround(int msg_id);

    void Generate_Ctrl0(int time_ms);
    void Generate_GPS1(int time_ms);
    void Generate_System2(int time_ms);
    int  Generate_Streaming(void);
    void Generate_Tcpip7(void);
    void Generate_Msg2Ground(void);
    void Generate_AircraftCfg(void);
    void SetWaypoint(float lat, float lon, float altitude, unsigned char waypoint_type, unsigned char wp_retire);

    /* control stuff */
    void SelectCtrlSource(byte source);
    void SaveValuesForAbort(void);
    void ApplyDefaults(void);

    void CalcDynYawRate(void);
    float CalcFTWPlimit(char gps_in_cruise);
    void SetPositionHold(void);
    void SetHome(void);
    void Arm(void);
    void Disarm(void);

    void CommandTakeoffArm(void);
    void CommandLanding(bool final, bool setWP);
    void CommandLandingWP(float lat, float lon, float alt_ground);
    void PlaylistSaveState(void);
    void PlaylistRestoreState(void);

    void ResetIMU(bool print);

private:
    typedef struct {
        unsigned char *msg;
        int            size;
        unsigned char  priority;    // 0-lowest
        unsigned char  msg_type;
    } T_Tentry;

    RawSerial *serial;
    FlightControlData *hfc;
    const ConfigData *pConfig;
    T_Tentry curr_msg;
    T_Tentry Q[MAX_AFSI_MESSAGES];
    unsigned char messages;

    unsigned int afsi_recv_bytes;
    byte afsi_recv_buffer[AFSI_BUFFER_SIZE];


    int ProcessAsfiCtrlCommands(TELEM_AFSI_MSG *msg);
    int ProcessAsfiStatusCommands(TELEM_AFSI_MSG *msg);

    bool ProcessParameters(T_Telem_Params4 *msg);
    bool CopyCommand(T_Telem_Commands5 *msg);

    unsigned int CalcCRC32(byte *data, unsigned int len);
    unsigned int RemoveBytes(unsigned char *b, int remove, int size);
    void InitHdr(unsigned char *msg, int msg_size);
    void InitHdr32(unsigned char type, unsigned char *msg, int msg_size);
    float WinSpeedEst(float Wangle);

    bool CheckRangeAndSetF(float *pvalue, byte *pivalue, float vmin, float vmax);
    bool CheckRangeAndSetI(int *pvalue, byte *pivalue, int vmin, int vmax);
    bool CheckRangeAndSetB(byte *pvalue, byte *pivalue, int vmin, int vmax);

    char PreFlightChecks(void);
    int FindNearestLandingSite(void);



    TELEM_AFSI_MSG *msg_afsi = new TELEM_AFSI_MSG;
    TELEM_AFSI_HDR *hdr_afsi= msg_afsi->hdr;
    T_Telem_Commands5 * msg_cmd = new T_Telem_Commands5;
    T_Telem_Params4 * msg_par = new T_Telem_Params4;
    T_TelemUpHdr *hdr = new T_TelemUpHdr;

};

#endif
