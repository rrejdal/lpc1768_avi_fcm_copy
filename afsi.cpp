/**
  ******************************************************************************
  * @file    afsi.cpp
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
#include <stddef.h>
#include <stdio.h>
#include "afsi.h"
#include "defines.h"

afsi::afsi(RawSerial *m_serial){
    serial = NULL;
    hfc = NULL;
    pConfig = NULL;
}

void afsi::clearBuffer(){
    afsi_index = 0;
    for (int i = 0;i<200;i++){
        afsi_buffer[i] = 0;
    }
}

void afsi::deleteMsg(){
    delete msg->hdr;
    delete msg;
    msg = new afsi_msg;
    msg->hdr = new afsi_hdr;
}

bool afsi::checkCRC(uint8_t len,uint8_t c1, uint8_t c2){
    //-2 is added because otherwise the 2 CRC bytes are included
    for (int i=0; i<len-2; i++) {
        c1 += msg->payload[i];
        c2 += c1;
    }

    if (c1 != msg->crc[0]) {
        return false;
    }

    if (c2 != msg->crc[1]) {
        return false;
    }

    return true;
}

//State machine reading bytes into a buffer
static void afsi::trackMsgState(){
    switch (afsi_state){
        case AFSI_STATE_INIT:
            if (afsi_byte == AFSI_SNC_CH_1){
                afsi_buffer[afsi_index] = afsi_byte;
                afsi_index++;
                afsi_state = AFSI_STATE_SYNC;
            }
            else{
                clearBuffer();
                deleteMsg();
            }
            break;
        case AFSI_STATE_SYNC:
            if (afsi_byte == AFSI_SNC_CH_2){
                afsi_buffer[afsi_index] = afsi_byte;
                afsi_index++;
                afsi_state = AFSI_STATE_CLASS;
            }
            else{
                clearBuffer();
                deleteMsg();
            }
            break;
        case AFSI_STATE_CLASS:
            afsi_buffer[afsi_index] = afsi_byte;
            afsi_index++;
            afsi_state = AFSI_STATE_ID;

            break;
        case AFSI_STATE_ID:
            afsi_buffer[afsi_index] = afsi_byte;
            afsi_index++;
            afsi_state = AFSI_STATE_LEN;

            break;
        case AFSI_STATE_LEN:
            if (afsi_index == 4){
                msg->hdr->len = (uint16_t)afsi_byte;
            }
            else if (afsi_index == 5){
                msg->hdr->len += ((uint16_t)afsi_byte<<8);
                afsi_state = AFSI_STATE_MSG;
                afsi_msgCnt = 0;
            }

            afsi_buffer[afsi_index] = afsi_byte;
            afsi_index++;

            break;
        case AFSI_STATE_MSG:
            if (afsi_msgCnt < msg->hdr->len+2){
                afsi_state = AFSI_STATE_DONE;
            }
            afsi_buffer[afsi_index] = afsi_byte;
            afsi_index++;
            afsi_msgCnt++;

            break;
    }
}

//Processes byte buffer into message
int afsi::parseMsg (){
    int totalLength = 1+1+1+1+2+msg->hdr->len + 2;

    msg->crc[0] = afsi_buffer[totalLength-1];
    msg->crc[1] = afsi_buffer[totalLength-2];

    if (!afsi::checkCRC(totalLength,msg->crc[0],msg->crc[1])){
        return -1;
    }


    //Populate information from buffer
    msg->hdr->sync1 = afsi_buffer[0];
    msg->hdr->sync2 = afsi_buffer[1];
    msg->hdr->msg_class = afsi_buffer[2];
    msg->hdr->id = afsi_buffer[3];
    for (int i=0; i<msg->hdr->len; i++){
        msg->payload[i] = afsi_buffer[4+i];
    }

    return 0;
}

static void afsi::processMsg(){
    switch(msg->hdr->msg_class){
        case AFSI_CMD_CLASS_ACK:
            unsigned char ClassID;
            unsigned char MsgID;
            if (msg->hdr->id == 0x01 || msg->hdr->id == 0x00){
                ClassID = afsi::processU1(msg->payload[0],1);
                MsgID = afsi::processU1(msg->payload[1],1);
            }

            break;
        case AFSI_CMD_CLASS_CTRL:
            if (msg->hdr->id == AFSI_CMD_ID_ARM){
                Arm();
            }
            else if (msg->hdr->id == AFSI_CMD_ID_DISARM){
                Disarm();
            }
            else if (msg->hdr->id = AFSI_CMD_ID_TAKEOFF){
                CommandTakeoffArm(processU2(msg->payload[0],1e-3));
            }
            else if (msg->hdr->id == AFSI_CMD_ID_LAND){

            }
    }

}

unsigned char afsi::processU1(uint8_t val, float scaling){
    return (unsigned char)(val*scaling);
}

unsigned short afsi::processU2(uint8_t val, float scaling){
    return (unsigned short)(val*scaling);
}

void afsi::Arm(){
    if (hfc->throttle_armed)
        return;

    /* throttle level needs to be low */
    if (hfc->throttle_value > -0.95f*pConfig->Stick100range)
    {
        SendMsgToGround(MSG2GROUND_ARMING_THROTTLE);
        return;
    }

    /* cannot arm in manual mode unless explicitly allowed */
    if (hfc->control_mode[PITCH]==CTRL_MODE_MANUAL && !pConfig->AllowArmInManual)
    {
        SendMsgToGround(MSG2GROUND_ARMING_MODE);
        return;
    }

    hfc->throttle_armed = 1;
    gps.glitches_ = 0;   // reset GPS glitch counter
    hfc->stats.can_servo_tx_errors = 0;
    hfc->stats.can_power_tx_errors = 0;
    SetHome();

//        GyroCalibDynamic(hfc);
}

void afsi::SendMsgToGround(int msg_id)
{
    hfc->msg2ground_id = msg_id;
    hfc->msg2ground_count = MSG2GROUND_RESEND_COUNT;
}

void afsi::SetHome(void)
{
    GpsData gps_data = gps.GetGpsData();

    hfc->home_pos[0] = gps_data.latF;
    hfc->home_pos[1] = gps_data.lonF;
    hfc->home_pos[2] = hfc->altitude;
    hfc->altitude_base = hfc->home_pos[2];
}

void afsi::Disarm(){
    hfc->throttle_armed    = 0;
    hfc->inhibitRCswitches = false;
    hfc->waypoint_type   = WAYPOINT_NONE;
    hfc->playlist_status = PLAYLIST_STOPPED;
    hfc->LidarCtrlMode   = false;
    hfc->fixedThrottleMode = THROTTLE_IDLE;

    xbus.InitXbusValues();

    // TODO::SP: Error handling on Flash write error??
    if (hfc->pid_params_changed) {
        SavePIDUpdates(hfc);
    }
}

void afsi::CommandTakeoffArm(float takeoffSpeed)
{
    int status = hfc->playlist_status;

    /* check for armed */
    if (!hfc->throttle_armed)
    {
        Disarm();
        /* send message that system has to be armed */
        SendMsgToGround(MSG2GROUND_ARMED_FOR_TAKEOFF);
        return;
    }

#if 0
    /* check for xbus being active */
    if (!xbus.receiving)
    {
        Disarm();
        /* send message that xbus radio has to be on */
        SendMsgToGround(MSG2GROUND_XBUS_FOR_TAKEOFF);
        return;
    }
#endif

    /* check sensors */
    if (!PreFlightChecks())
    {
        Disarm();
        return;
    }

    hfc->inhibitRCswitches = true;

    /* reset all I terms */
    ResetIterms();

    /* set PRY controls to Angle mode, coll to manual */
    SetCtrlMode(hfc, pConfig, PITCH, CTRL_MODE_RATE);
    SetCtrlMode(hfc, pConfig, ROLL,  CTRL_MODE_RATE);
    SetCtrlMode(hfc, pConfig, YAW,   CTRL_MODE_RATE);
    SetCtrlMode(hfc, pConfig, COLL,  CTRL_MODE_MANUAL);

    /* initialize PRY angles to the current orientation */
    hfc->ctrl_out[RATE][PITCH] = 0;
    hfc->ctrl_out[RATE][ROLL]  = 0;
    hfc->ctrl_out[RATE][YAW]   = 0;

    /* 0 angle of blades (from config) */
//    hfc->ctrl_out[RAW][COLL]   = pConfig->CollZeroAngle;
//    hfc->pid_CollVspeed.COlast = pConfig->CollZeroAngle;  // needed once alt hold is enabled and this could be uninitialized for the takeoff condition

    hfc->ctrl_collective_raw = hfc->collective_raw_curr;    // set to current position
    hfc->ctrl_collective_3d  = pConfig->CollZeroAngle;   // target

    /* enable fixed control mode */
    SelectCtrlSource(CTRL_SOURCE_AUTO3D);
    // it is getting reset by select source !!!!!!!!!!!!!!!!!!!!!!!!!!
    hfc->playlist_status = status;

    /* send message that to prepare radio and spool up */
    if (hfc->full_auto) {
        SendMsgToGround(MSG2GROUND_ALLOW_SPOOLUP);
    }
    else {
        SendMsgToGround(MSG2GROUND_SPOOLUP);
    }

    hfc->message_from_ground = 0;   // reset it so we can wait for the message from ground
    hfc->waypoint_type   = WAYPOINT_TAKEOFF;
    if (hfc->full_auto)
        hfc->waypoint_stage  = FM_TAKEOFF_AUTO_SPOOL;
    else
        hfc->waypoint_stage  = FM_TAKEOFF_ARM;
    hfc->message_timeout = 60000000;    // 60 seconds
}

void afsi::SelectCtrlSource(byte source)
{
    /* save RC stick values when switching away from RCradio source, for the abort function */
    if (hfc->ctrl_source==CTRL_SOURCE_RCRADIO) {
        SaveValuesForAbort();
    }

    /* stop playlist and clear waypoint when going manual, set vspeed limit to max */
    if (source==CTRL_SOURCE_RCRADIO || source==CTRL_SOURCE_JOYSTICK)
    {
        ApplyDefaults();
        hfc->ctrl_out[POS][COLL] = hfc->altitude;
        hfc->playlist_status = PLAYLIST_STOPPED;
    }

    if (source == CTRL_SOURCE_JOYSTICK) {
        hfc->telem_ctrl_period = 100000;   // in us - 10Hz
    }
    else {
        hfc->telem_ctrl_period = 0;
    }

    hfc->telem_ctrl_period = max(hfc->telem_ctrl_period, pConfig->telem_min_ctrl_period*1000);
    hfc->ctrl_source = source;
    hfc->waypoint_type = WAYPOINT_NONE;
}

/* keeps delta within +/-180 */
/* 3us */
float afsi::Wrap180(float delta)
{
    while(delta>=180)
        delta-=360;
    while(delta<-180)
        delta+=360;
    return delta;
//    int se = (long long)(delta*11930464.7111111f);
//    delta = se/11930464.7111111f;
//    return delta;
}

void afsi::SetWaypoint(float lat, float lon, float altitude, unsigned char waypoint_type, unsigned char wp_retire)
{
//    debug_print("Telemetry_SetWaypoint %f %f\r\n", lat, lon);
    /* store stick values for an abort, only the first the waypoint mode is turned on */
    unsigned char source;
    float Sx, Sy, Tx, Ty;
//    float prev_STcourse;

    if (hfc->playlist_status==PLAYLIST_PLAYING)
    {
        /* in playlist mode, copy the current waypoint to the previous */
        hfc->waypoint_pos_prev[0] = hfc->waypoint_pos[0];
        hfc->waypoint_pos_prev[1] = hfc->waypoint_pos[1];
        hfc->waypoint_pos_prev[2] = hfc->ctrl_out[POS][COLL];//hfc->waypoint_pos[2]; use the last set altitude to better connect alt at waypoints
//        prev_STcourse = hfc->waypoint_STcourse;
    }
    else
    {
        /* in manual mode, use current position as the previous waypoint */
        hfc->waypoint_pos_prev[0] = hfc->positionLatLon[0];
        hfc->waypoint_pos_prev[1] = hfc->positionLatLon[1];
        hfc->waypoint_pos_prev[2] = hfc->altitude;
//        prev_STcourse = hfc->IMUorient[YAW]*R2D;  // use current heading
    }

    if (altitude>-9999)  // if altitude present, set source to 3D
        source = CTRL_SOURCE_AUTO3D;
    else
    if (hfc->ctrl_source==CTRL_SOURCE_AUTO3D)   // if already at 3D keep it
        source = CTRL_SOURCE_AUTO3D;
    else
        source = CTRL_SOURCE_AUTO2D;    // 2D otherwise
    SelectCtrlSource(source);

    hfc->waypoint_pos[0] = lat;
    hfc->waypoint_pos[1] = lon;

    /* turn on horizontal position mode, yaw angle and keep collective and throttle as is */
    hfc->waypoint_retire    = wp_retire;

    hfc->waypoint_type  = waypoint_type;

    SetCtrlMode(hfc, pConfig, PITCH, CTRL_MODE_POSITION);
    SetCtrlMode(hfc, pConfig, ROLL,  CTRL_MODE_POSITION);
    SetCtrlMode(hfc, pConfig, YAW,   CTRL_MODE_ANGLE);
    /* if altitude specified, set collective to position */
    if (altitude>-9999)
    {
        SetCtrlMode(hfc, pConfig, COLL,  CTRL_MODE_POSITION);
        hfc->waypoint_pos[2] = hfc->altitude_base + altitude;
    }
    hfc->altitude_WPnext = -9999;   // mark as used

    hfc->distance2WP_min = 999999;

    /* pre-calculate waypoint path data */

    /* distance to the next waypoint */
    hfc->waypoint_STdist = DistanceCourse(hfc->waypoint_pos_prev[0], hfc->waypoint_pos_prev[1], hfc->waypoint_pos[0], hfc->waypoint_pos[1], &hfc->waypoint_STcourse);
//    debug_print("S->T: dist = %4.1f, cour = %4.1f\r\n", hfc->waypoint_STdist, hfc->waypoint_STcourse);

    Sx = 0;
    Sy = 0;
    Tx = (float)(hfc->waypoint_pos[1] - hfc->waypoint_pos_prev[1]);
    Ty = (float)(hfc->waypoint_pos[0] - hfc->waypoint_pos_prev[0]);
    Tx = Tx/DPM*COSfD(((float)hfc->waypoint_pos[0]));
    Ty = Ty/DPM;
//    debug_print("S: %f %f\n", Sx, Sy);
//    debug_print("T: %f %f\n", Tx, Ty);

    hfc->path_a = Ty-Sy;
    hfc->path_b = Sx-Tx;
    if (ABS(hfc->path_a)>=3 || ABS(hfc->path_b)>=3)  // at least 3 meters apart
        hfc->path_dist_denom = 1.0f/sqrtf(hfc->path_a*hfc->path_a+hfc->path_b*hfc->path_b);
    else
        hfc->path_dist_denom = 0;
    /* reset the distance to path PID so the acc logic can start working again properly */
    hfc->pid_Dist2P.COlast = 0;

    /* coordinated turns handing in heli mode (thrust vectoring) */
    CalcDynYawRate();

    /* reduce the distance to WP by the WP retire radius to make sure heli is at the right altitude at that moment */
    float ofs;

    if (hfc->waypoint_type==WAYPOINT_FLYTHROUGH) {
      ofs = CalcFTWPlimit(false);
    }
    else {
      ofs = hfc->rw_cfg.GTWP_retire_radius;
    }

    hfc->waypoint_STofs = ofs;
    hfc->waypoint_STdist = Max(1, hfc->waypoint_STdist - ofs);
}

/* vspeed needs to be negative */
void afsi::CommandLanding(bool final, bool setWP)
{
    {
        if (!final && hfc->altitude_lidar>3)
        {
            /* check lidar and send a warning message if no ground lock is being received from lidar */
            if (hfc->altitude_lidar_raw>35)
                SendMsgToGround(MSG2GROUND_LIDAR_NOGROUND);

            /* set 2D waypoint at the current location */
            if (setWP) {
                SetWaypoint(hfc->positionLatLon[0], hfc->positionLatLon[1], -9999, WAYPOINT_GOTO, 0); // need to set altitude to switch to 3D control mode
            }

            SelectCtrlSource(CTRL_SOURCE_AUTO3D);
            SetCtrlMode(hfc, pConfig, YAW, CTRL_MODE_ANGLE);

            /* if wind strong enough, rotate tail down-wind otherwise just keep the current heading */
            if (hfc->wind_speed > hfc->rw_cfg.landing_wind_threshold) {  // 3m/s
                hfc->ctrl_out[ANGLE][YAW] = Wrap180(hfc->wind_course);
            }
            else {
                hfc->ctrl_out[ANGLE][YAW] = hfc->IMUorient[YAW]*R2D;
            }

            /* if switching from manual collective, initialize the ctrl speed with the current one */
            if (hfc->control_mode[COLL] < CTRL_MODE_SPEED) {
                hfc->ctrl_out[SPEED][COLL] = hfc->IMUspeedGroundENU[UP];
            }

            SetCtrlMode(hfc, pConfig, COLL,  CTRL_MODE_SPEED);
            hfc->ctrl_vspeed_3d = max(-2*pConfig->landing_vspeed, hfc->pid_CollAlt.COmin);
            hfc->waypoint_type  = WAYPOINT_LANDING;
            hfc->waypoint_stage = FM_LANDING_HIGH_ALT;
        }
        else
        {
            /* set PRY controls to speed/Angle mode */
            SetCtrlMode(hfc, pConfig, PITCH, CTRL_MODE_SPEED);
            SetCtrlMode(hfc, pConfig, ROLL,  CTRL_MODE_SPEED);
            SetCtrlMode(hfc, pConfig, YAW,   CTRL_MODE_ANGLE);
            SetCtrlMode(hfc, pConfig, COLL,  CTRL_MODE_SPEED);

            /* if previous pitch/roll mode was above speed, initialize Iterm such that CO does not instantly change */
            hfc->ctrl_out[SPEED][PITCH] = 0;
            hfc->ctrl_out[SPEED][ROLL]  = 0;
            hfc->ctrl_out[ANGLE][YAW]   = hfc->IMUorient[YAW]*R2D;

            /* set vspeed mode and initialize it */
//            hfc->ctrl_out[SPEED][COLL] = vspeed;
            hfc->ctrl_vspeed_3d = -pConfig->landing_vspeed;
            SelectCtrlSource(CTRL_SOURCE_AUTO3D);

            hfc->waypoint_type  = WAYPOINT_LANDING;
            hfc->waypoint_stage = FM_LANDING_LOW_ALT;
        }
    }
}
