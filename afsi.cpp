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

extern FlightControlData hfc;
extern ConfigData *pConfig;

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

    afsi_rx_bytes  = 0;
    rx_payload_len = 0;
    rx_msg_state   = AFSI_STATE_INIT;
    rx_msg_rdy     = 0;
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

void AFSI_Serial::AddInputByte(char rx_byte)
{
    switch (rx_msg_state)
    {
        case AFSI_STATE_INIT:
            if(rx_byte == AFSI_SYNC_BYTE_1) {
                afsi_rx_buffer[afsi_rx_bytes] = rx_byte;
                afsi_rx_bytes++;
                rx_msg_state = AFSI_STATE_SYNC;
            }
            break;
        case AFSI_STATE_SYNC:
            if(rx_byte == AFSI_SYNC_BYTE_2) {
                afsi_rx_buffer[afsi_rx_bytes] = rx_byte;
                afsi_rx_bytes++;
                rx_msg_state = AFSI_STATE_CLASS_ID;
            }
            else {
                afsi_rx_bytes = 0;
                rx_msg_state = AFSI_STATE_INIT;
            }
            break;
        case AFSI_STATE_CLASS_ID:
            afsi_rx_buffer[afsi_rx_bytes] = rx_byte;
            afsi_rx_bytes++;
            if( afsi_rx_bytes >= 4 ) {
                rx_msg_state = AFSI_STATE_LEN;
            }
            break;
        case AFSI_STATE_LEN:
            afsi_rx_buffer[afsi_rx_bytes] = rx_byte;
            afsi_rx_bytes++;
            if( afsi_rx_bytes >= 6 ) {
                rx_msg_state = AFSI_STATE_READ;
                rx_payload_len = ((uint16_t)afsi_rx_buffer[5] << 8 ) + (uint16_t)afsi_rx_buffer[4];
            }
            break;
        case AFSI_STATE_READ:
            afsi_rx_buffer[afsi_rx_bytes] = rx_byte;
            afsi_rx_bytes++;

            if( afsi_rx_bytes >= (sizeof(AFSI_HDR) + rx_payload_len + 2)) {
                afsi_rx_bytes = 0;
                rx_msg_rdy = 1;
                rx_msg_state = AFSI_STATE_INIT;
            }
            break;
        default:
            break;
    }

#if 0
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
#endif

    if (rx_msg_rdy == 1) {
        memcpy(&msg_afsi, afsi_rx_buffer, afsi_rx_bytes);

        //Get, set, and check CRC
        if (GetCRC(&(afsi_rx_buffer[2]), 4 + rx_payload_len, &(msg_afsi.crc[0])) == 0) {
            afsi_crc_errors++;
            ResetRxMsgData();
            return;
        }

        if ( msg_afsi.hdr.msg_class == AFSI_CTRL ) {
            if (ProcessAsfiCtrlCommands(&msg_afsi)) {
                afsi_good_messages++;

                /////////////////////////////////////
                // Send ACK message
                /////////////////////////////////////

            }
            else {
                afsi_crc_errors++;
                /////////////////////////////////////
                // Send NACK
                /////////////////////////////////////
            }
            return;
        }
        else if ( msg_afsi.hdr.msg_class == AFSI_STATUS ) {
            if (ProcessAsfiStatusCommands(&msg_afsi)){
                afsi_good_messages++;

                /////////////////////////////////////
                // Send appropriate STATUS?
                /////////////////////////////////////

            }
            else {
                afsi_crc_errors++;
                /////////////////////////////////////
                // Send NACK message
                /////////////////////////////////////
            }
            return;
        }

        ResetRxMsgData();
    }

    return;

}

void AFSI_Serial::ResetRxMsgData(void)
{
    rx_payload_len = 0;
    rx_msg_rdy = 0;
    afsi_rx_bytes = 0;
    rx_msg_state = AFSI_STATE_INIT;
    return;
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

void AFSI_Serial::EnableAFSI(void) {

    hfc.afsi_new_value[PITCH]= 0;
    hfc.afsi_new_value[ROLL] = 0;
    hfc.afsi_new_value[YAW]  = 0;
    hfc.afsi_new_value[COLL] = 0;
    hfc.afsi_new_value[THRO] = 0;
    hfc.afsi_new_value = true;

    /* altitude hold and yaw angle */
    SetCtrlMode(&hfc, pConfig, PITCH, CTRL_MODE_SPEED);
    SetCtrlMode(&hfc, pConfig, ROLL,  CTRL_MODE_SPEED);
    SetCtrlMode(&hfc, pConfig, YAW,   CTRL_MODE_ANGLE);
    SetCtrlMode(&hfc, pConfig, COLL,  CTRL_MODE_POSITION);
    /* set target altitude and heading to the current one */
    hfc.ctrl_out[ANGLE][YAW] = hfc.IMUorient[YAW]*R2D;
    hfc.afsi_new_value = 1;

    if (hfc.playlist_status==PLAYLIST_PLAYING)
    {
        telem->PlaylistSaveState();
        telem->SelectCtrlSource(CTRL_SOURCE_AFSI);
        hfc.playlist_status = PLAYLIST_PAUSED;
    }
    else
    if (hfc.playlist_status==PLAYLIST_PAUSED)
    {
        telem->SelectCtrlSource(CTRL_SOURCE_AFSI);
        hfc.playlist_status = PLAYLIST_PAUSED;
    }
    else {
        telem->SelectCtrlSource(CTRL_SOURCE_AFSI);
    }

    return;
}

int AFSI_Serial::ProcessAsfiCtrlCommands(AFSI_MSG *msg)
{
    float lat, lon, speed, alt, heading = {0};
    EnableAFSI();

    if (rx_payload_len != ctrl_msg_lengths[msg->hdr.id]) {
        return 0;
    }

    switch (msg->hdr.id){
        case AFSI_CTRL_ID_ARM:
            telem->Arm();
            break;

        case AFSI_CTRL_ID_DISARM:
            telem->Disarm();
            break;

        case AFSI_CTRL_ID_TAKEOFF:
            telem->CommandTakeoffArm();
            break;

        case  AFSI_CTRL_ID_LAND:
            hfc.playlist_status = PLAYLIST_STOP;
            // TODO MMRI: check if this is immediate landing
            telem->CommandLanding(false, true);
            break;

        case AFSI_CTRL_ID_SET_POS:
            lat = (int)((uint32_t)(msg_afsi.payload[0])       |
                        (uint32_t)(msg_afsi.payload[1] << 8)  |
                        (uint32_t)(msg_afsi.payload[2] << 16) |
                        (uint32_t)(msg_afsi.payload[3] << 24)   ) * AFSI_SCALE_POS;


            lon = (int)((uint32_t)(msg_afsi.payload[4])       |
                        (uint32_t)(msg_afsi.payload[5] << 8)  |
                        (uint32_t)(msg_afsi.payload[6] << 16) |
                        (uint32_t)(msg_afsi.payload[7] << 24)   ) * AFSI_SCALE_POS;

            if ( CheckRangeI(lat, -90, 90)   ||
                 CheckRangeI(lon, -180, 180)    ) {
                return 0;
            }

            /* since this is asynchronous to the main loop, the control_mode
             * change might be detected and the init skipped,
             * thus it needs to be done here */
            if (hfc.control_mode[PITCH] < CTRL_MODE_POSITION) {
                PID_SetForEnable(&hfc.pid_Dist2T, 0, 0, hfc.gps_speed);
                PID_SetForEnable(&hfc.pid_Dist2P, 0, 0, 0);
                hfc.speedCtrlPrevEN[0] = 0;
                hfc.speedCtrlPrevEN[1] = 0;
            }

            telem->SetWaypoint(lat, lon, -9999, WAYPOINT_GOTO, false);
            break;

        case AFSI_CTRL_ID_SPEED_FWD:
            speed = processU2(msg->payload,AFSI_SCALE_SPEED);

            if (CheckRangeI(speed, AFSI_MIN_SPEED, AFSI_MAX_SPEED)) {
                hfc.afsi_new_value = 1;
                hfc.afsi_values[AFSI_SPEED_FWD] = speed;
            }
            else {
                return 0;
            }
            break;

        case AFSI_CTRL_ID_SPEED_RIGHT:
            speed = processU2(msg->payload,AFSI_SCALE_SPEED);

            if (CheckRangeI(speed, AFSI_MIN_SPEED, AFSI_MAX_SPEED)) {
                hfc.afsi_new_value = 1;
                hfc.afsi_values[AFSI_SPEED_RIGHT] = speed;
            }
            else {
                return 0;
            }
            break;

        case AFSI_CTRL_ID_SET_ALT:
            alt = processU2(msg->payload,AFSI_SCALE_ALT);

            if (CheckRangeI(alt, AFSI_MIN_ALT, AFSI_MAX_ALT)) {
                hfc.afsi_new_value = 1;
                hfc.afsi_values[AFSI_ALTITUDE] = alt;
            }
            else {
                return 0;
            }
            break;

        case AFSI_CTRL_ID_HOME:
            speed = processU2(msg->payload,AFSI_SCALE_SPEED);
            if (CheckRangeI(speed, AFSI_MIN_SPEED, AFSI_MAX_SPEED) && (speed > 0) ) {
                hfc.pid_Dist2T.COmax = speed;
                telem->ApplyDefaults();
                telem->SetWaypoint(hfc.home_pos[0], hfc.home_pos[1], -9999, WAYPOINT_GOTO, 0);
            }
            else {
                return 0;
            }
            break;

        case AFSI_CTRL_ID_HOLD:
            telem->SetPositionHold();
            break;

        case AFSI_CTRL_ID_HEADING:
            heading = processU2(msg->payload,AFSI_SCALE_SPEED);

            if (CheckRangeI(heading, AFSI_MIN_HEADING, AFSI_MAX_HEADING)) {
                hfc.afsi_new_value = 1;
                hfc.afsi_values[AFSI_HEADING] = heading;
            }
            else {
                return 0;
            }
            break;

        case AFSI_CTRL_ID_RESUME:
            /* disabling joystick */
            if (hfc.playlist_status == PLAYLIST_PAUSED)
                telem->PlaylistRestoreState();
            else
                telem->SelectCtrlSource(CTRL_SOURCE_RCRADIO);
            break;

        default:
            return 0;
    }

    return 1;
}

void AFSI_Serial::InitStatusHdr(void) {
    afsi_hdr.sync1     = AFSI_SYNC_BYTE_1;
    afsi_hdr.sync2     = AFSI_SYNC_BYTE_2;
    afsi_hdr.msg_class = AFSI_CMD_CLASS_STAT;
}

void AFSI_Serial::GeneratePwrStatus(void) {

    InitStatusHdr();
    afsi_hdr.id        = AFSI_STAT_ID_POW;
    afsi_hdr.len       = 12;
    msg_pwr_status.hdr = &afsi_hdr;

    msg_pwr_status.bat_capacity_used  = hfc.power.capacity_used;
    msg_pwr_status.total_bat_capacity = hfc.power.capacity_total;
    msg_pwr_status.bat_percent_used   = hfc.power.battery_level;
    msg_pwr_status.current            = hfc.power.Iesc;
    msg_pwr_status.voltage            = hfc.power.Vmain;
    msg_pwr_status.flight_time        = hfc.power.flight_time_left;
}

int AFSI_Serial::ProcessAsfiStatusCommands(AFSI_MSG *msg)
{
    if (rx_payload_len != AFSI_RX_STAT_PAYLOAD) {
        return 0;
    }

    switch (msg->hdr.id)
    {
        case AFSI_STAT_ID_POW:
            break;
        case AFSI_STAT_ID_GPS:
            break;
        case AFSI_STAT_ID_OP:
            break;
        case  AFSI_STAT_ID_FCM:
            break;
        default:
            return 0;
    }

    return 1;
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

float AFSI_Serial::processU2(uint8_t*data, int scaling)
{
    float result = 0;

    for (int i = 0; i<2; i++)
    {
        result += (data[i]<<(8*(1-i)));
    }

    return result*scaling;
}
