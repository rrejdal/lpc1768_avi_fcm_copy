#include <stddef.h>
#include <stdio.h>
#include "utils.h"
#include "mymath.h"
#include "HMC5883L.h"
#include "pGPS.h"
#include "IMU.h"
#include "defines.h"
#include "afsi.h"
#include "telemetry.h"

extern int SavePIDUpdates(FlightControlData *fcm_data);
extern void ResetIterms(void);
extern void GenerateSpeed2AngleLUT(void);
extern void AltitudeUpdate(float alt_rate, float dT);
extern void HeadingUpdate(float heading_rate, float dT);

extern HMC5883L compass;
extern GPS gps;
extern XBus xbus;


#include "NOKIA_5110.h"
#include "hardware.h"
extern NokiaLcd myLcd;

AFSI_Serial::AFSI_Serial(RawSerial *m_serial, TelemSerial *m_telem)
{
    curr_msg.msg = NULL;
    curr_msg.size = 0;
    messages      = 0;
    serial = m_serial;
    telem = m_telem;

    afsi_good_messages       = 0;
    afsi_crc_errors          = 0;
    afsi_start_code_searches = 0;
    afsi_recv_bytes          = 0;

}

bool AFSI_Serial::AddMessage(unsigned char *m_msg, int m_size, unsigned char m_msg_type, unsigned char m_priority)
{
    if (messages >= MAX_TELEM_MESSAGES) {
        return false;
    }

    Q[messages].msg  = m_msg;
    Q[messages].size = m_size;
    Q[messages].msg_type = m_msg_type;
    Q[messages].priority = m_priority;
    messages++;

    return true;
}

void AFSI_Serial::Update()
{
    while(1) {
        unsigned int i;
        int index = 0;
        int priority;

        /* if current message is in progress, keep sending it till serial buffer is full */
        while (curr_msg.size) {
            if (!serial->writeable()) {
                return;
            }
            serial->putc(*curr_msg.msg++);
            curr_msg.size--;
        }

        /* if no more messages, exit */
        if (!messages) {
            return;
        }

        /* find the first highest priority message and move it to the current message to go out */
        priority = -1;
        for (i=0; i<messages; i++) {
            if (Q[i].priority>priority) {
                priority = Q[i].priority;
                index = i;
            }
        }
        curr_msg = Q[index];

        for (i=index+1; i<messages; i++) {
            Q[i-1] = Q[i];
        }

        messages--;
    }
}

bool AFSI_Serial::IsEmpty()
{
    if (!curr_msg.size && !messages) {
        return true;
    }
    else {
        return false;
    }
}

bool AFSI_Serial::IsTypeInQ(unsigned char type)
{
    int i;

    if (curr_msg.msg_type==type) {
        return true;
    }

    for (i=0; i<messages; i++) {
        if (Q[i].msg_type==type) {
            return true;
        }
    }

    return false;
}

unsigned int AFSI_Serial::CalcCRC32(byte *data, unsigned int len)
{
  unsigned int i;
  unsigned int result = 0xffffffff;

  for (i=0; i<len; i++) {
    result=(result << 8) ^ crc32_table[(result >> 24) ^ data[i]];
  }

  return result;
}

unsigned int AFSI_Serial::RemoveBytes(unsigned char *b, int remove, int size)
{
    int j;

    for (j=remove; j<size; j++) {
      b[j-remove] = b[j];
    }

    size -= remove;

    return size;
}

void AFSI_Serial::ProcessInputBytes(RawSerial &afsi_serial)
{
    while (afsi_serial.readable()) {
        AddInputByte(afsi_serial.getc());
    }
}

void AFSI_Serial::AddInputByte(char ch)
{

    /* if full, just clear it */
    if (afsi_recv_bytes >= AFSI_BUFFER_SIZE) {
        afsi_recv_bytes = 0;
    }

    /* add input byte into the recv buffer and increment telem_recv_bytes*/
    afsi_recv_buffer[afsi_recv_bytes++] = ch;

    /* if first byte is not the start code find it, by discarding all data in front of it */
    if (afsi_recv_buffer[0]!= AFSI_SNC_CH_1)
    {
        unsigned int i;
        bool foundSC_AFSI = false;
        for (i=0; i<afsi_recv_bytes; i++)
        {
            if (afsi_recv_buffer[i]==AFSI_SNC_CH_1)
            {
                afsi_recv_bytes = RemoveBytes(afsi_recv_buffer,i,afsi_recv_bytes);
                foundSC_AFSI = true;
                break;
            }
        }
        afsi_start_code_searches++;
        if (!foundSC_AFSI)
        {
            afsi_recv_bytes = 0;
            return;
        }
    }

    if (!afsi_recv_bytes)
        return;

    /* still did not find it, this case should never happen */
    if (afsi_recv_buffer[0] != AFSI_SNC_CH_1) {
        afsi_recv_bytes = 0;
        return;
    }

    //Check if enough bytes for header
    if (afsi_recv_bytes < 6) {
        return;
    }

    //Populate AFSI message header
    hdr_afsi->sync1     = afsi_recv_buffer[0];
    hdr_afsi->sync2     = afsi_recv_buffer[1];
    hdr_afsi->msg_class = afsi_recv_buffer[2];
    hdr_afsi->id        = afsi_recv_buffer[3];
    hdr_afsi->len       = (afsi_recv_buffer[4]<<8) + afsi_recv_buffer[5];

    //Check if enough bytes for rest of message
    //+2 at the end added for CRC
    if (afsi_recv_bytes < sizeof(AFSI_HDR)+hdr_afsi->len+2) {
        return;
    }

    //Populate rest of the message
    for (int i = 0; i<msg_afsi->hdr->len; i++) {
        msg_afsi->payload[i] = afsi_recv_buffer[i+6];
    }

    //Get, set, and check CRC failed
    if (GetCRC(&(afsi_recv_buffer[2]), 4 + hdr_afsi->len, &(msg_afsi->crc[0]))) {
        afsi_recv_buffer[0] = 0;
        afsi_crc_errors++;
        return;
    }

    if ( msg_afsi->hdr->msg_class == AFSI_CTRL ) {
        if (ProcessAsfiCtrlCommands(msg_afsi)) {
            afsi_good_messages++;

            /////////////////////////////////////
            // Send ACK message
            /////////////////////////////////////

        }
        else {

            /////////////////////////////////////
            // Send NACK message????
            /////////////////////////////////////
        }
        afsi_recv_bytes = RemoveBytes(afsi_recv_buffer, hdr_afsi->len+6, afsi_recv_bytes);
        return;
    }
    else if ( msg_afsi->hdr->msg_class == AFSI_STATUS ) {
        if (ProcessAsfiStatusCommands(msg_afsi)){
            afsi_good_messages++;

            /////////////////////////////////////
            // Send ACK message OR JUST STATUS?
            /////////////////////////////////////

        }
        else {

            /////////////////////////////////////
            // Send NACK message
            /////////////////////////////////////
        }
        afsi_recv_bytes = RemoveBytes(afsi_recv_buffer, hdr_afsi->len+6, afsi_recv_bytes);
        return;
    }
    else {
        /* unknown message, wipe out the start code so the search can be restarted */
        afsi_recv_buffer[0] = 0;
    }
}

int AFSI_Serial::GetCRC(uint8_t *data, int len, uint8_t *CRC)
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


int AFSI_Serial::ProcessAsfiCtrlCommands(AFSI_MSG *msg)
{
    float speed, alt, heading = {0};

    switch (msg->hdr->id){
        case AFSI_CMD_ID_ARM:
            telem->Arm();
            break;

        case AFSI_CMD_ID_DISARM:
            telem->Disarm();
            break;

        case AFSI_CMD_ID_TAKEOFF:
            telem->CommandTakeoffArm();
            break;

        case  AFSI_CMD_ID_LAND:
            hfc->playlist_status = PLAYLIST_STOP;
            // TODO MMRI: check if this is imediate landing
            telem->CommandLanding(false, true);
            break;

        case AFSI_CMD_ID_SET_POS:
            float lat, lon;
            AFSI_POS_DATA *pos = &msg->payload[0];

            if ( CheckRangeI(pos->latitude, -90, 90)    ||
                 CheckRangeI(pos->longitude, -180, 180)    ) {
                return 0;
            }

            lat = pos->latitude  * AFSI_SCALE_POS;
            lon = pos->longitude * AFSI_SCALE_POS;

            /* since this is asynchronous to the main loop, the control_mode change might be detected and the init skipped,
            ** thus it needs to be done here */
            if (hfc->control_mode[PITCH] < CTRL_MODE_POSITION)
            {
                PID_SetForEnable(&hfc->pid_Dist2T, 0, 0, hfc->gps_speed);
                PID_SetForEnable(&hfc->pid_Dist2P, 0, 0, 0);
                hfc->speedCtrlPrevEN[0] = 0;
                hfc->speedCtrlPrevEN[1] = 0;
            }

            telem->SetWaypoint(lat, lon, -9999, WAYPOINT_GOTO, false);
            break;

        case AFSI_CMD_ID_SPEED_FWD:
            speed = processU2(msg->payload,AFSI_SCALE_SPEED);

            if (CheckRangeI(speed, AFSI_MIN_SPEED, AFSI_MAX_SPEED) && (speed != 0) ) {
                hfc->afsi_values = 1;
                hfc->afsi_new_values[AFSI_SPEED_FWD] = speed;
            }
            else {
                return 0;
            }
            break;

        case AFSI_CMD_ID_SPEED_AFT:
            speed = processU2(msg->payload,AFSI_SCALE_SPEED);

            if (CheckRangeI(speed, AFSI_MIN_SPEED, AFSI_MAX_SPEED) && (speed != 0) ) {
                hfc->afsi_values = 1;
                hfc->afsi_new_values[AFSI_SPEED_AFT] = speed;
            }
            else {
                return 0;
            }
            break;

        case AFSI_CMD_ID_SPEED_RIGHT:
            speed = processU2(msg->payload,AFSI_SCALE_SPEED);

            if (CheckRangeI(speed, AFSI_MIN_SPEED, AFSI_MAX_SPEED) && (speed != 0) ) {
                hfc->afsi_values = 1;
                hfc->afsi_new_values[AFSI_SPEED_RIGHT] = speed;
            }
            else {
                return 0;
            }
            break;

        case AFSI_CMD_ID_SPEED_LEFT:
            speed = processU2(msg->payload,AFSI_SCALE_SPEED);

            if (CheckRangeI(speed, AFSI_MIN_SPEED, AFSI_MAX_SPEED) && (speed != 0) ) {
                hfc->afsi_values = 1;
                hfc->afsi_new_values[AFSI_SPEED_LEFT] = speed;
            }
            else {
                return 0;
            }
            break;

        case AFSI_CMD_ID_SET_ALT:
            alt = processU2(msg->payload,AFSI_SCALE_ALT);

            if (CheckRangeI(alt, AFSI_MIN_ALT, AFSI_MAX_ALT)) {
                hfc->afsi_values = 1;
                hfc->afsi_new_values[AFSI_ALTITUDE] = alt;
            }
            else {
                return 0;
            }
            break;

        case AFSI_CMD_ID_HOME:
            speed = processU2(msg->payload,AFSI_SCALE_SPEED);
            if (CheckRangeI(speed, AFSI_MIN_SPEED, AFSI_MAX_SPEED) && (speed != 0) ) {
                hfc->pid_Dist2T.COmax = speed;
                telem->ApplyDefaults();
                telem->SetWaypoint(hfc->home_pos[0], hfc->home_pos[1], -9999, WAYPOINT_GOTO, 0);
            }
            else {
                return 0;
            }
            break;

        case AFSI_CMD_ID_HOLD:
            telem->SetPositionHold();
            break;

        case AFSI_CMD_ID_HEADING:
            heading = processU2(msg->payload,AFSI_SCALE_SPEED);

            if (CheckRangeI(heading, AFSI_MIN_HEADING, AFSI_MAX_HEADING)) {
                hfc->afsi_values = 1;
                hfc->afsi_new_values[AFSI_HEADING] = heading;
            }
            else {
                return 0;
            }
            break;

        default:
            return 0;
    }

    return 1;
}

int AFSI_Serial::ProcessAsfiStatusCommands(AFSI_MSG *msg)
{
    switch (msg->hdr->id)
    {
        case AFSI_CMD_ID_STAT_POW:
            break;
        case AFSI_CMD_ID_STAT_GPS:
            break;
        case AFSI_CMD_ID_STAT_OP:
            break;
        case  AFSI_CMD_ID_STAT_FCM:
            break;
        default:
            return 0;
    }

    return 1;
}

void AFSI_Serial::InitHdr(unsigned char *msg, int msg_size)
{
    msg[0] = 0x47;
    msg[1] = msg_size-4-1;
    msg[2] = 0x39;      // initial value
    msg[2] = CalcCRC8(msg, msg_size);
}

void AFSI_Serial::InitHdr32(unsigned char type, unsigned char *msg, int msg_size)
{
    T_TelemUpHdr *hdr = (T_TelemUpHdr*)msg;

    hdr->start_code = 0x47;
    hdr->type       = type;
    hdr->len        = msg_size-8-1;
    hdr->crc8       = 0x39;
    hdr->crc8       = CalcCRC8(msg, 4);

    hdr->crc        = 0x12345678;
    hdr->crc        = CalcCRC32((unsigned char*)msg, msg_size);
}

bool AFSI_Serial::CheckRangeAndSetF(float *pvalue, byte *pivalue, float vmin, float vmax)
{
    float value = *((float*)pivalue);
//  debug_print("%f\r\n", value);
    if (!pvalue || value>vmax || value<vmin) {
        return false;
    }

    *pvalue = value;
    return true;
}

bool AFSI_Serial::CheckRangeAndSetI(int *pvalue, uint8_t *pivalue, float vmin, float vmax)
{
    float value = *((int*)pivalue);
//  debug_print("%f\r\n", value);
    if (!pvalue || value>vmax || value<vmin) {
        return false;
    }

    *pvalue = value;
    return true;
}

bool AFSI_Serial::CheckRangeI(int value, int vmin, int vmax)
{
    if ( (value > vmax) || (value < vmin) ) {
        return false;
    }

    return true;
}

bool AFSI_Serial::CheckRangeAndSetB(byte *pvalue, byte *pivalue, int vmin, int vmax)
{
    int value = *((int*)pivalue);
    if (!pvalue || value>vmax || value<vmin) {
        return false;
    }

    *pvalue = value;
    return true;
}

void AFSI_Serial::SendMsgToGround(int msg_id)
{
    hfc->msg2ground_id = msg_id;
    hfc->msg2ground_count = MSG2GROUND_RESEND_COUNT;
}

void AFSI_Serial::SetPositionHold(void)
{
    /* set heading to the current one */
    hfc->ctrl_out[ANGLE][YAW] = hfc->IMUorient[YAW]*R2D;
    /* set vcontrol to max to make sure it can hold the pos */
    ApplyDefaults();
    SetWaypoint(hfc->positionLatLon[0], hfc->positionLatLon[1],
                                (hfc->altitude - hfc->altitude_base), WAYPOINT_GOTO, 0);
    hfc->ctrl_source = CTRL_SOURCE_AUTO3D;
}

float AFSI_Serial::processU2(uint8_t*data, int scaling)
{
    float result = 0;

    for (int i = 0; i<2; i++)
    {
        result += (data[i]<<(8*(1-i)));
    }

    return result*scaling;
}
