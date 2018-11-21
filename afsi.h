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

#define MAX_MESSAGES  10
#define BUFFER_SIZE   (256+8+16)

class AFSI
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
    
    AFSI(RawSerial *m_serial);

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
