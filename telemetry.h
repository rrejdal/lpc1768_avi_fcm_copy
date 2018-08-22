#ifndef _TELEMETRY_H_
#define _TELEMETRY_H_

#include "mbed.h"
#include "structures.h"

#define MAX_TELEM_MESSAGES  10
#define TELEM_BUFFER_SIZE   (256+8+16)  // 16 for some slack

typedef struct
{
    unsigned char *msg;
    int            size;
    unsigned char  priority;    // 0-lowest
    unsigned char  msg_type;
} T_Tentry;

class TelemSerial
{
public:
    TelemSerial(RawSerial *m_serial);
    bool AddMessage(unsigned char *m_msg, int m_size, unsigned char m_msg_type, unsigned char m_priority);
    void Update();
    bool IsEmpty();
    bool IsTypeInQ(unsigned char type);
    
    /* receiving stuff */
    void    AddInputByte(char ch, T_HFC *hfc);
    uint32  telem_good_messages;
    uint32  telem_crc_errors;
    uint32  telem_start_code_searches;
private:
    RawSerial *serial;
    T_Tentry curr_msg;
    T_Tentry Q[MAX_TELEM_MESSAGES];
    unsigned char messages;
    
    /* receiving stuff */
    unsigned int telem_recv_bytes;
    byte         telem_recv_buffer[TELEM_BUFFER_SIZE];
    
};

void Telemetry_Generate_Ctrl0(T_HFC *hfc, T_Telem_Ctrl0 *msg, int time_ms);
void Telemetry_Generate_GPS1(T_HFC *hfc, T_Telem_GPS1 *msg, int time_ms);
void Telemetry_Generate_System2(T_HFC *hfc, T_Telem_System2 *msg, int time_ms, TelemSerial *telem);
int  Telemetry_Generate_Streaming(T_HFC *hfc, T_Telem_DataStream3 *msg);
void Telemetry_Generate_Tcpip7(T_HFC *hfc, T_Telem_TCPIP7 *msg);
void Telemetry_Generate_Msg2Ground(T_HFC *hfc, T_Telem_Msg2Ground *msg);
void Telemetry_Generate_AircraftCfg(T_HFC *hfc, T_AircraftConfig *msg);

void Telemetry_SetWaypoint(T_HFC *hfc, float lat, float lon, float altitude, unsigned char waypoint_type, unsigned char wp_retire);
void SelectCtrlSource(T_HFC *hfc, byte source);
void SaveValuesForAbort(T_HFC *hfc);
void ApplyDefaults(T_HFC *hfc);
//void DynamicAccInTurns(T_HFC *hfc, T_PID *pid);
void Calc_DynYawRate(T_HFC *hfc);
float CalcFTWPlimit(T_HFC *hfc, char gps_in_cruise);
void ProcessCommands(T_HFC *hfc);
void SetPositionHold(T_HFC *hfc);
void SetHome(T_HFC *hfc);
void Arm(T_HFC *hfc);
void Disarm(T_HFC *hfc);
void SendMsgToGround(T_HFC *hfc, int msg_id);
void Command_TakeoffArm(T_HFC *hfc);
void Command_Landing(T_HFC *hfc, bool final, bool setWP);
void Command_LandingWP(T_HFC *hfc, float lat, float lon, float alt_ground);
void Heading_Update(T_HFC *hfc, float heading_rate, float dT);

void Playlist_SaveState(T_HFC *hfc);
void Playlist_RestoreState(T_HFC *hfc);

void ResetIMU(T_HFC *hfc, bool print);

#endif
