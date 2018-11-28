/**
  ******************************************************************************
  * @file    afsi.h
  * @author  AVIDRONE FIRMWARE Team
  * @brief   Provides support for AVI-FCM Serial Interface
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
#ifndef _AFSI_H_
#define _AFSI_H_

#include "mbed.h"
#include "structures.h"
#include "pGPS.h"

#define MAX_MESSAGES  10
#define BUFFER_SIZE   (256+8+16)

#define AFSI_CMD_CLASS_CTRL     0x01
#define AFSI_CMD_CLASS_STAT     0x02
#define AFSI_CMD_CLASS_ACK      0x03

#define AFSI_CMD_ID_ARM         0x00
#define AFSI_CMD_ID_DISARM      0x01
#define AFSI_CMD_ID_TAKEOFF     0x02
#define AFSI_CMD_ID_LAND        0x03
#define AFSI_CMD_ID_SET_POS     0x04
#define AFSI_CMD_ID_FWD_SPEED   0x05
#define AFSI_CMD_ID_AFT_SPEED   0x06
#define AFSI_CMD_ID_RGT_SPEED   0x07
#define AFSI_CMD_ID_LFT_SPEED   0x08
#define AFSI_CMD_ID_ALT         0x09
#define AFSI_CMD_ID_HOME        0x0A
#define AFSI_CMD_ID_HOLD        0x0B
#define AFSI_CMD_ID_HDNG        0x0C

#define AFSI_CMD_ID_STAT_POW    0x00
#define AFSI_CMD_ID_STAT_GPS    0x01
#define AFSI_CMD_ID_STAT_OP     0x02
#define AFSI_CMD_ID_STAT_FCM    0x03

#define AFSI_SNC_CH_1           0XB5
#define AFSI_SNC_CH_2           0X62

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
}afsi_hdr;

typedef struct{
    afsi_hdr * hdr;
    uint8_t payload[200];
    uint8_t crc[2];
}afsi_msg;

class afsi
{

public:

    void Initialize(FlightControlData *p_hfc, const ConfigData *p_config) { hfc = p_hfc, pConfig = p_config; }

    /* receiving stuff */
    void ProcessCommands(void);
    bool AddMessage(unsigned char *m_msg, int m_size, unsigned char m_msg_type, unsigned char m_priority);
    void Update();
    bool IsEmpty();
    bool IsTypeInQ(unsigned char type);
    void ProcessInputBytes(RawSerial &telemetry);
    void AddInputByte(char ch);

    bool checkCRC(uint8_t len,uint8_t c1, uint8_t c2);
    void clearBuffer();
    static void trackMsgState ();
    int parseMsg();
    static void processMsg();
    void deleteMsg();
    afsi(RawSerial *m_serial);

    void SendMsgToGround(int msg_id);
    unsigned char processU1(uint8_t val, float scaling);
    unsigned short processU2(uint8_t val, float scaling);
    void Arm();
    void Disarm();
    void SetHome();
    void CommandTakeoffArm (float takeoffSpeed);
    void SelectCtrlSource(byte source);
    void CommandLanding(bool final, bool setWP);
    void SetWaypoint(float lat, float lon, float altitude, unsigned char waypoint_type, unsigned char wp_retire);
    float Wrap180(float delta);

    uint8_t afsi_buffer[200] = {0};
    uint8_t afsi_byte;
    uint8_t afsi_index;
    uint8_t afsi_msgCnt;
    uint32_t afsi_state;


    /* transmit stuff */
    /*
    void SendMsgToGround(int msg_id);

    void Generate_Ctrl0(int time_ms);
    void Generate_GPS1(int time_ms);
    void Generate_System2(int time_ms);
    int  Generate_Streaming(void);
    void Generate_Tcpip7(void);
    void Generate_Msg2Ground(void);
    void Generate_AircraftCfg(void);
    void SetWaypoint(float lat, float lon, float altitude, unsigned char waypoint_type, unsigned char wp_retire);
    */

private:
    afsi_msg * msg;
    RawSerial *serial;
    FlightControlData *hfc;
    const ConfigData *pConfig;



    /*
    typedef struct {
        unsigned char *msg;
        int            size;
        unsigned char  priority;    // 0-lowest
        unsigned char  msg_type;
    } T_Tentry;

    T_Tentry curr_msg;
    T_Tentry Q[MAX_TELEM_MESSAGES];
    unsigned char messages;
    
    unsigned int telem_recv_bytes;
    byte telem_recv_buffer[TELEM_BUFFER_SIZE];
    
    bool ProcessParameters(T_Telem_Params4 *msg);
    bool CopyCommand(T_Telem_Commands5 *msg);

    unsigned int CalcCRC32(byte *data, unsigned int len);
    unsigned int RemoveBytes(unsigned char *b, int remove, int size);
    void InitHdr(unsigned char *msg, int msg_size);
    void InitHdr32(unsigned char type, unsigned char *msg, int msg_size);
    */
};

#endif
