#include "telemetry.h"
#include "utils.h"
#include "mymath.h"
#include "HMC5883L.h"
#include "xbus.h"
#include "pGPS.h"
#include "config.h"
#include "IMU.h"
#include <stddef.h>
#include <stdio.h>

extern HMC5883L compass;
extern GPS gps;
extern XBus xbus;

TelemSerial::TelemSerial(RawSerial *m_serial)
{
    curr_msg.msg = NULL;
    curr_msg.size = 0;
    messages      = 0;
    serial = m_serial;
    
    telem_good_messages       = 0;
    telem_crc_errors          = 0;
    telem_start_code_searches = 0;
    telem_recv_bytes          = 0;

}

void TelemSerial::ProcessInputBytes(RawSerial &telemetry)
{
    while (telemetry.readable()) {
        AddInputByte(telemetry.getc());
    }
}

bool TelemSerial::AddMessage(unsigned char *m_msg, int m_size, unsigned char m_msg_type, unsigned char m_priority)
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

void TelemSerial::Update()
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

bool TelemSerial::IsEmpty()
{
    if (!curr_msg.size && !messages) {
        return true;
    }
    else {
        return false;
    }
}

bool TelemSerial::IsTypeInQ(unsigned char type)
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

unsigned int TelemSerial::CalcCRC32(byte *data, unsigned int len)
{
  unsigned int i;
  unsigned int result = 0xffffffff;

  for (i=0; i<len; i++) {
    result=(result << 8) ^ crc32_table[(result >> 24) ^ data[i]];
  }

  return result;
}

unsigned int TelemSerial::RemoveBytes(unsigned char *b, int remove, int size)
{
    int j;

    for (j=remove; j<size; j++) {
      b[j-remove] = b[j];
    }

    size -= remove;

    return size;
}

void TelemSerial::AddInputByte(char ch)
{
    T_TelemUpHdr *hdr;

//  printf("Telem c=%d\r\n", ch);
    
    /* if full, just clear it */
    if (telem_recv_bytes >= TELEM_BUFFER_SIZE) {
        telem_recv_bytes = 0;
    }

    /* add input byte into the recv buffer and increment telem_recv_bytes*/
    telem_recv_buffer[telem_recv_bytes++] = ch;
    
    /* if first byte is not the start code find it, by discarding all data in front of it */
    if (telem_recv_buffer[0]!=0x47)
    {
        unsigned int i;
        bool foundSC = false;
        for (i=0; i<telem_recv_bytes; i++)
        {
            if (telem_recv_buffer[i]==0x47)
            {
                telem_recv_bytes = RemoveBytes(telem_recv_buffer, i, telem_recv_bytes);
                foundSC = true;
                break;
            }
        }
        telem_start_code_searches++;
        if (!foundSC)
        {
        	telem_recv_bytes = 0;
        	return;
        }
    }

    if (!telem_recv_bytes)
    	return;

    /* still did not find it, this case should never happen */
    if (telem_recv_buffer[0]!=0x47)
    {
//        printf("NSC: %2d ", telem_recv_bytes);
//        for (i=0; i<8; i++) printf("%02x ", telem_recv_buffer[i]);
//        printf("\r\n");
        telem_recv_bytes = 0;
        return;
    }

    if (telem_recv_bytes<4)
    	return;
    
    hdr = (T_TelemUpHdr*)telem_recv_buffer;

    /* check CRC of the header */
    {
    	byte hdr1[4];
    	byte crc8;

    	hdr1[0] = telem_recv_buffer[0];
    	hdr1[1] = telem_recv_buffer[1];
    	hdr1[2] = 0x39;
    	hdr1[3] = telem_recv_buffer[3];
    	crc8 = CalcCRC8(hdr1, 4);
    	if (crc8!=hdr->crc8)
    	{
    		telem_recv_buffer[0] = 0;
//            printf("Wrong hdr CRC\r\n");
    		return;

    	}
    }
	if (hdr->type>=TELEMETRY_LAST_MSG_TYPE)
	{
		telem_recv_buffer[0] = 0;
//            printf("Wrong msg type %d\r\n", hdr->type);
		return;
	}
	if (hdr->type==TELEMETRY_COMMANDS5 && hdr->len>18)
	{
		telem_recv_buffer[0] = 0;
//            printf("Wrong msg size %d type %d\r\n", hdr->len, hdr->type);
		return;
	}
	if (hdr->type==TELEMETRY_PROFILE_CMD && hdr->len>(sizeof(T_Telem_Profile8)-8-1))
	{
		telem_recv_buffer[0] = 0;
//            printf("Wrong msg size %d type %d\r\n", hdr->len, hdr->type);
		return;
	}

	/*do we have enough bytes for the header */
    if (telem_recv_bytes<8)
        return;
        
    /* do we have enough bytes for the message */
    if (telem_recv_bytes < (unsigned int)(hdr->len+8+1))
        return;
        
    /* check crc */
    {
        unsigned int crc_org = hdr->crc;
        unsigned int crc_calc;
        hdr->crc = 0x12345678;
        crc_calc = CalcCRC32(telem_recv_buffer, hdr->len+8+1);
        if (crc_org != crc_calc)
        {
            /* error - wipe out the start code so the search can be restarted for a new message */
            telem_recv_buffer[0] = 0;
            telem_crc_errors++;
            return;
        }
    }

//    printf("Telem Msg type=%d\r\n", hdr->type);
    
    if (hdr->type==TELEMETRY_PARAMETERS4)
    {
        T_Telem_Params4 *msg = (T_Telem_Params4*)telem_recv_buffer;

        if (ProcessParameters(msg))
        {
            telem_good_messages++;
            /* do not confirm new joystick values */
            if (!hfc->joystick_new_values)
            {
                /* this might be produced before the previous one was transmitted !!!!!!! */
                hfc->tcpip_confirm  = true;
                hfc->tcpip_org_type = hdr->type;
                hfc->tcpip_org_len  = hdr->len;
                hfc->tcpip_user1    = 0;
                hfc->tcpip_user2    = 0;
            }
        }
        telem_recv_bytes = RemoveBytes(telem_recv_buffer, hdr->len+8+1, telem_recv_bytes);
        return;
    }
    else if (hdr->type==TELEMETRY_COMMANDS5) {

        T_Telem_Commands5 *msg = (T_Telem_Commands5*)telem_recv_buffer;

        if (CopyCommand(msg)) {
            telem_good_messages++;
            /* this might be produced before the previous one was transmitted !!!!!!! */
            hfc->tcpip_confirm  = true;
            hfc->tcpip_org_type = hdr->type;
            hfc->tcpip_org_len  = hdr->len;
            hfc->tcpip_user1    = 0;
            hfc->tcpip_user2    = 0;
        }

        telem_recv_bytes = RemoveBytes(telem_recv_buffer, hdr->len+8+1, telem_recv_bytes);

        return;
    }
    else
    if (hdr->type==TELEMETRY_PLAYLIST)
    {
        if (hfc->playlist_status==PLAYLIST_STOPPED)
        {
            T_Telem_Playlist6 *msg = (T_Telem_Playlist6*)telem_recv_buffer;
            int i;
                        
            if (msg->page==0)
            {
                hfc->playlist_items = 0;
                hfc->playlist_position = 0;
            }
            
            if (hfc->playlist_items != 21*msg->page)
            {
//                printf("Playlist items %d does not correspond to the current page %d\r\n", hfc->playlist_items, msg->page);
                hfc->playlist_items = 21*msg->page;
            }
            if ((hfc->playlist_items+msg->lines) <= PLAYLIST_SIZE)
            {
                for (i=0; i<msg->lines; i++)
                    hfc->playlist[hfc->playlist_items++] = msg->items[i];
            }
            /* this might be produced before the previous one was transmitted !!!!!!! */
            hfc->tcpip_confirm  = true;
            hfc->tcpip_org_type = hdr->type;
            hfc->tcpip_org_len  = hdr->len;
            hfc->tcpip_user1    = hfc->playlist_items;
            hfc->tcpip_user2    = PLAYLIST_SIZE;
//            printf("new playlist item\r\n");
        }
        telem_good_messages++;
        telem_recv_bytes = RemoveBytes(telem_recv_buffer, hdr->len+8+1, telem_recv_bytes);
        return;
    }
    else
    if (hdr->type == TELEMETRY_PROFILE_CMD)
    {
        T_Telem_Profile8 *msg = (T_Telem_Profile8*)telem_recv_buffer;
        int i;
        
        hfc->streaming_enable = msg->stream_enable;     // enables data streaming
        
        /* error checking */
        if (hfc->streaming_enable && (msg->stream_channels<1 || msg->stream_channels>7))
            hfc->streaming_enable = false;
            
        if (hfc->streaming_enable)
        {
            for (i=0; i<7; i++)
                hfc->streaming_types[i] = msg->stream_data_types[i];     // selects individual data types for streaming
            hfc->streaming_channels = msg->stream_channels;   // number of channels included
            hfc->streaming_channel  = 0;      // current channel
            hfc->streaming_samples  = 120 / hfc->streaming_channels;
            hfc->streaming_sample   = 0;       // current sample
            // msg->stream_period;
//            printf("Streaming ch %d s %d \r\n", hfc->streaming_channels, hfc->streaming_samples);
        }

        if (msg->profile_enable)
        {
            if (msg->profile_ctrl_level==3) { // speed - set home position since this profiling is relative to home
                SetHome();
            }

            hfc->profile_ctrl_variable = msg->profile_ctrl_variable;    // selects the controlled variable for auto-profiling (P, R, Y, C, T)
            hfc->profile_ctrl_level    = msg->profile_ctrl_level;       // selects the level of control for auto-profiling (raw/rate/angle/speed)
            hfc->profile_lag           = msg->profile_lag;              // initial lag (T1) in ms
            hfc->profile_period        = msg->profile_period;           // test period (Tt) in ms
            hfc->profile_delta         = msg->profile_delta;            // control value (dC) in % of stick
            hfc->profile_mode = PROFILING_START;
//            printf("profiling started channel %d level %d lag %d period %d delta %d\r\n",
//                    hfc->profile_ctrl_variable, hfc->profile_ctrl_level, hfc->profile_lag, hfc->profile_period, hfc->profile_delta);
        }        
        
        telem_good_messages++;
        /* this might be produced before the previous one was transmitted !!!!!!! */
        hfc->tcpip_confirm  = true;
        hfc->tcpip_org_type = hdr->type;
        hfc->tcpip_org_len  = hdr->len;
        hfc->tcpip_user1    = 0;
        hfc->tcpip_user2    = 0;
        telem_recv_bytes = RemoveBytes(telem_recv_buffer, hdr->len+8+1, telem_recv_bytes);
        return;
    }
    else
    if (hdr->type==TELEMETRY_LANDINGSITES)
    {
    	T_Telem_LandingSites *msg = (T_Telem_LandingSites*)telem_recv_buffer;
		int i;

		if (msg->page==0)
		{
			hfc->landing_sites_num = 0;
		}

		if (hfc->landing_sites_num != 15*msg->page)
		{
//                printf("Playlist items %d does not correspond to the current page %d\r\n", hfc->playlist_items, msg->page);
			hfc->landing_sites_num = 15*msg->page;
		}
		if ((hfc->landing_sites_num+msg->lines) <= LANDING_SITES)
		{
			for (i=0; i<msg->lines; i++)
				hfc->landing_sites[hfc->landing_sites_num++] = msg->items[i];
		}
		/* this might be produced before the previous one was transmitted !!!!!!! */
		hfc->tcpip_confirm  = true;
		hfc->tcpip_org_type = hdr->type;
		hfc->tcpip_org_len  = hdr->len;
		hfc->tcpip_user1    = hfc->landing_sites_num;
		hfc->tcpip_user2    = LANDING_SITES;
//            printf("new playlist item\r\n");

		telem_good_messages++;
        telem_recv_bytes = RemoveBytes(telem_recv_buffer, hdr->len+8+1, telem_recv_bytes);
        return;
    }
    else
    {
        /* unknown message, wipe out the start code so the search can be restarted */
        telem_recv_buffer[0] = 0;
    }
}

void TelemSerial::InitHdr(unsigned char *msg, int msg_size)
{
    msg[0] = 0x47;
    msg[1] = msg_size-4-1;
    msg[2] = 0x39;      // initial value
    msg[2] = CalcCRC8(msg, msg_size);
}

void TelemSerial::InitHdr32(unsigned char type, unsigned char *msg, int msg_size)
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

void TelemSerial::Generate_Ctrl0(int time_ms)
{
    int i;
    T_Telem_Ctrl0 *msg = &hfc->telemCtrl0;

    /* payload */
    msg->time       = time_ms;
    msg->ctrl_modes = 0;             // p/r/y/c/t   3-bits each
    for (i=0; i<5; i++) msg->ctrl_modes |= (hfc->control_mode[i] & 0x7) << (i*3);
    for (i=0; i<3; i++) msg->gyro_lp[i]  = Float32toFloat16(hfc->gyroFilt[i]);
    for (i=0; i<3; i++) msg->acc_lp[i]   = Float32toFloat16(hfc->accFilt[i]);
//    for (i=0; i<3; i++) msg->acc_lp[i]   = Float32toFloat16(hfc->accGroundENUhp[i]);
    for (i=0; i<3; i++) msg->compass[i]  = Float32toFloat16(compass.dataXYZcalib[i]);
    msg->compass_heading = Float32toFloat16(hfc->compass_heading);
    for (i=0; i<3; i++) msg->orient[i]   = Float32toFloat16(hfc->IMUorient[i]*R2D);
    msg->speedGroundENU[0] = Float32toFloat16(hfc->IMUspeedGroundENU[0]);
    msg->speedGroundENU[1] = Float32toFloat16(hfc->IMUspeedGroundENU[1]);
    msg->speedGroundENU[2] = Float32toFloat16(hfc->IMUspeedGroundENU[2]);
    msg->altitude      = hfc->altitude;
    msg->baro_altitude = hfc->baro_altitude_raw_lp;
    msg->baro_vspeed   = Float32toFloat16(hfc->baro_vspeed);
//    msg->baro_vspeed   = Float32toFloat16(hfc->baro_vspeedDF);
//    msg->baro_vspeed   = Float32toFloat16(hfc->lidar_vspeed);
    
    for (i=0; i<5; i++) msg->ctrl_manual[i] = Float32toFloat16(hfc->ctrl_out[RAW][i]);   // PRYCT
    for (i=0; i<3; i++) msg->ctrl_rate[i]   = Float32toFloat16(hfc->ctrl_out[RATE][i]);  // PRY
    for (i=0; i<3; i++) msg->ctrl_angle[i]  = Float32toFloat16(hfc->ctrl_out[ANGLE][i]); // PRY
    for (i=0; i<2; i++) msg->ctrl_speed[i]  = Float32toFloat16(hfc->ctrl_out[SPEED][i]); // FR
    msg->ctrl_speed[2]                      = Float32toFloat16(hfc->ctrl_out[SPEED][COLL]);
    msg->ctrl_altitude = hfc->ctrl_out[POS][COLL];
    
    msg->latitude  = (int)(hfc->positionLatLon[0]*10000000);
    msg->longitude = (int)(hfc->positionLatLon[1]*10000000);
    msg->lidar_alt = Float32toFloat16(hfc->altitude_lidar);
    
    InitHdr32(TELEMETRY_CTRL, (unsigned char*)msg, sizeof(T_Telem_Ctrl0));
}

void TelemSerial::Generate_GPS1(int time_ms)
{
    int i;
    T_Telem_GPS1 *msg = &hfc->telemGPS1;

    GpsData gps_data = gps.GetGpsData();

    /* payload */
    msg->time        = time_ms;
    msg->gps_pos[0]  = gps_data.lat/10;
    msg->gps_pos[1]  = gps_data.lon/10;
    msg->gps_pos[2]  = (int)(gps_data.altitude*100);
    msg->gps_speed   = Float32toFloat16(gps_data.HspeedC);
    msg->gps_heading = Float32toFloat16(gps_data.courseC);
    for (i=0; i<3; i++) msg->gps_to_home[i]     = Float32toFloat16(hfc->gps_to_home[i]);     // distance/heading/altitude difference
    if (hfc->ctrl_source==CTRL_SOURCE_AUTO2D || hfc->ctrl_source==CTRL_SOURCE_AUTO3D)
        for (i=0; i<3; i++) msg->gps_to_waypoint[i] = Float32toFloat16(hfc->gps_to_waypoint[i]); // distance/heading/altitude difference
    else
        for (i=0; i<3; i++) msg->gps_to_waypoint[i] = Float32toFloat16(0);
    for (i=0; i<3; i++) msg->gps_speed_ENU[i] = Float32toFloat16(gps_data.speedENU[i]);
    InitHdr32(TELEMETRY_GPS, (unsigned char*)msg, sizeof(T_Telem_GPS1));
}

float TelemSerial::WinSpeedEst(float Wangle)
{
    int angle = ABS(Wangle)*16;
    int anglei = min(44, angle/16);
    int angler = angle & 0xf;
    float speed1 = pConfig->WindSpeedLUT[anglei]*pConfig->WindTableScale;
    float speed2 = pConfig->WindSpeedLUT[anglei+1]*pConfig->WindTableScale;
    float speed = (speed1*(16-angler) + speed2*angler) * 0.0625f;

    if (Wangle<0) {
        speed = -speed;
    }

    return speed;
}

void TelemSerial::Generate_System2(int time_ms)
{
    T_Telem_System2 *msg = &hfc->telemSystem2;

    GpsData gps_data = gps.GetGpsData();
	char fixC, fix2, sats2;
    float WangleE = PIDestOutput(&hfc->pid_PitchSpeed, hfc->speed_Iterm_E_lp);
    float WangleN = PIDestOutput(&hfc->pid_PitchSpeed, hfc->speed_Iterm_N_lp);
    float WspeedE = WinSpeedEst(WangleE);
    float WspeedN = WinSpeedEst(WangleN);
    float Wspeed = sqrtf(WspeedN*WspeedN + WspeedE*WspeedE);
    float Wcour = 0;
//    printf("%f %f\r\n", WangleE, WangleN);

    fixC = gps_data.fix;

    if (gps.GetLastUpdate() > 300000) {
   		fixC = 3;
    }

    gps.GetFixSatsOther(&fix2, &sats2);

    if (WspeedN || WspeedE)
        Wcour = 90-ATAN2fD(WspeedN, WspeedE);
    if (Wcour>180)
        Wcour-=360;
    else if (Wcour<-180)
        Wcour+=360;

    hfc->wind_speed = Wspeed;
    hfc->wind_course = Wcour;
    
    byte waypoint_ctrl_mode = hfc->ctrl_source==CTRL_SOURCE_AUTO2D || hfc->ctrl_source==CTRL_SOURCE_AUTO3D ? 1 : 0;
    byte joystick_ctrl_mode = hfc->ctrl_source==CTRL_SOURCE_JOYSTICK ? 1 : 0;
    
    /* payload */
    msg->time        = time_ms;
    
    msg->baro_pressure = hfc->baro_pressure;
    msg->gps_time = gps_data.time/100;
    msg->gps_date = gps_data.date;
    msg->precess_period_lp = (hfc->ticks_lp+32)>>6;      // in us
    msg->precess_period_max = hfc->ticks_max;     // in us
    msg->telem_good_messages       = telem_good_messages;
    msg->telem_crc_errors          = telem_crc_errors;
    msg->telem_start_code_searches = telem_start_code_searches;
    msg->flight_time_left	= ClipMinMax(hfc->power.flight_time_left, 0, 65535);
    msg->power.Iaux			= min(255, (int)(hfc->power.Iaux*32+0.5f));	// 0-8A		*32
    msg->power.Iesc			= min(4095, (int)(hfc->power.Iesc*64+0.5f));	// 0-64V	*64
    msg->power.Vaux			= min(1023, (int)(hfc->power.Vaux*64+0.5f));	// 0-16V	*64
    msg->power.Vesc			= min(4095, (int)(hfc->power.Vesc*64+0.5f));	// 0-64V	*64
    msg->power.Vmain		= min(4095, (int)(hfc->power.Vmain*64+0.5f));
    msg->power.Vservo		= min(1023, (int)(hfc->power.Vservo*64+0.5f));	// 0-16V	*64
    msg->power.battery_level = ClipMinMax((int)(hfc->power.battery_level*2.5f+0.5f), 0, 255);	// in %, 250=100%	*250
    msg->power.capacity_used = min(255, (int)(hfc->power.capacity_used*8/3600+0.5f));	// 0-32Ah			*8
    msg->gyro_temperature = Float32toFloat16(hfc->gyro_temp_lp);
    msg->baro_temperature = Float32toFloat16(hfc->baro_temperature);
    msg->gps_hdop = Float32toFloat16(gps_data.PDOP*0.01f);
    msg->gps_sats_curr  = Min(15, gps_data.sats);
    msg->gps_sats_other = Min(15, sats2);
    msg->gps_fix_curr   = fixC;
    msg->gps_fix_other  = fix2;
    msg->gps_current    = gps.selected_channel_;
    msg->reserved       = 0;
    msg->cpu_utilization = hfc->cpu_utilization_lp * 2.55f;
    msg->control_status = (xbus.receiving & 1) | (waypoint_ctrl_mode<<1)| (((!hfc->throttle_armed)&1)<<2) | ((joystick_ctrl_mode&1)<<3)
                            | ((hfc->playlist_status&0x3)<<4) | ((hfc->full_auto&1)<<6);
    msg->playlist_items    = hfc->playlist_items;
    msg->playlist_position = hfc->playlist_position;
    msg->gps_errors        = gps.glitches_;
    msg->wind_speed        = Float32toFloat16(Wspeed);
    msg->wind_course       = Float32toFloat16(Wcour);

    msg->RPM               = Float32toFloat16(hfc->RPM);
    msg->power_lp          = Float32toFloat16(hfc->power.power_lp);

    msg->gyro_offsets[0]   = Float32toFloat16(hfc->gyroOfs[PITCH]);
    msg->gyro_offsets[1]   = Float32toFloat16(hfc->gyroOfs[ROLL]);
    msg->gyro_offsets[2]   = Float32toFloat16(hfc->gyroOfs[YAW]);
    msg->esc_temp          = Float32toFloat16(pConfig->linklive ? hfc->esc_temp : hfc->gyro_temp_lp);

    InitHdr32(TELEMETRY_SYSTEM, (unsigned char*)msg, sizeof(T_Telem_System2));
}

void TelemSerial::Generate_AircraftCfg(void)
{
    T_AircraftConfig *msg = &hfc->aircraftConfig;

    float thr_min, thr_max;

    /* payload */
    if (pConfig->throttle_ctrl==PROP_VARIABLE_PITCH)
    {
        thr_min = pConfig->throttle_values[0];
        thr_max = 2 * pConfig->Stick100range * pConfig->throttle_values[1] + pConfig->throttle_values[0];
    }
    else
    {
        thr_min = -pConfig->Stick100range;
        thr_max =  pConfig->Stick100range;
    }

    msg->throttle_min       = Float32toFloat16(thr_min);       // throttle value at 0%
    msg->throttle_max       = Float32toFloat16(thr_max);       // throttle value at 100%

    msg->power_hover        = pConfig->power_typical;              // typical power used at hover

    msg->collective_flat    = Float32toFloat16(pConfig->CollZeroAngle);         // collective value for 0 pitch
    msg->collective_hover   = Float32toFloat16(hfc->pid_CollVspeed.COofs);  // collective at hover
    msg->collective_max     = Float32toFloat16(hfc->pid_CollVspeed.COmax);  // maximum value

    msg->rpm_hover          = pConfig->rpm_typical;                             // typical RPM at hover
    msg->return_speed       = Float32toFloat16(pConfig->landing_appr_speed);
    msg->bat_cells          = pConfig->battery_cells;
    msg->unused[0]          = 0;
    msg->unused[1]          = 0;
    msg->unused[2]          = 0;

    InitHdr32(TELEMETRY_AIRCRAFT_CFG, (unsigned char*)msg, sizeof(T_AircraftConfig));
}

void TelemSerial::Generate_Tcpip7(void)
{
    T_Telem_TCPIP7 *msg = &hfc->telemTcpip7;

    msg->type       = TELEMETRY_TCPIP;

    /* payload */
    msg->org_type   = hfc->tcpip_org_type;
    msg->org_len    = hfc->tcpip_org_len;
    msg->user1      = hfc->tcpip_user1;
    msg->user2      = hfc->tcpip_user2;
    InitHdr((unsigned char*)msg, sizeof(T_Telem_TCPIP7));
}

int TelemSerial::Generate_Streaming(void)
{
    T_Telem_DataStream3 *msg = &hfc->telemDataStream3;

    int i, size = hfc->streaming_channels * hfc->streaming_samples;

    msg->type       = TELEMETRY_DATASTREAM3;
    msg->time = hfc->time_ms;
    // bits 0-2 number of data elements, bit 3 profilling, streaming otherwise; bit 4 in profiling mode, indicates the last message
    msg->stream_info = hfc->streaming_channels & 0x7;            // bits 0-2 number of data elements, bit 5 profilling, streaming otherwise, bit 6 first message, bit 7 last message
    if (hfc->profile_mode!=PROFILING_OFF)
    {
        msg->stream_info |= 0x8;
        if (hfc->profile_mode==PROFILING_FINISH)
        {
            /* profiling is done, stop profiling and streaming */
            msg->stream_info |= 0x10;
            hfc->profile_mode = PROFILING_OFF;
            hfc->streaming_enable = false;
//            printf("profiling done\r\n");
        }
    }
    for (i=0; i<hfc->streaming_channels; i++)
        msg->data_type[i] = hfc->streaming_types[i];       // identifies the data types of individual elements
        
    for (i=0; i<size; i++)
        msg->data[i] = hfc->stream_data[i];
        
    size = size*2 + 4 + 4 + 8;
    InitHdr((unsigned char*)msg, size);
    
    return size;
}

void TelemSerial::Generate_Msg2Ground(void)
{
    T_Telem_Msg2Ground *msg = &hfc->telemMsg2ground;

    msg->type       = TELEMETRY_MSG2GROUND;

    /* payload */
    msg->msg_id      = hfc->msg2ground_id;
    msg->user_data   = 0;
    InitHdr((unsigned char*)msg, sizeof(T_Telem_Msg2Ground));
}

void TelemSerial::SaveValuesForAbort(void)
{
//	printf("Saved values for abort\r\n");
    hfc->ctrl_initial[THRO]  = xbus.valuesf[XBUS_THR_LV];
    hfc->ctrl_initial[PITCH] = xbus.valuesf[XBUS_PITCH];
    hfc->ctrl_initial[ROLL]  = xbus.valuesf[XBUS_ROLL];
    hfc->ctrl_initial[YAW]   = xbus.valuesf[XBUS_YAW];
    hfc->ctrl_initial[COLL]  = xbus.valuesf[XBUS_THRO];
}

void TelemSerial::ApplyDefaults(void)
{
    hfc->pid_CollAlt.COmax        = pConfig->VspeedMax;
    hfc->pid_CollAlt.COmin        = pConfig->VspeedMin;
    hfc->pid_CollAlt.acceleration = pConfig->VspeedAcc;
    hfc->pid_Dist2T.COmax         = pConfig->HspeedMax;
    hfc->pid_Dist2T.acceleration  = pConfig->HspeedAcc;
    CalcDynYawRate();
}

void TelemSerial::SelectCtrlSource(byte source)
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

void TelemSerial::CalcDynYawRate(void)
{
   float AImax    = pConfig->TurnAccParams[0];  // m/s2 max side acceleration during turns
   float MaxSpeed = pConfig->TurnAccParams[1];  // m/s speed at which the max acc is reached
   float MinBlend = pConfig->TurnAccParams[2];  // low speed knee after which yaw rate starts to slow down
   float MYR = pConfig->dyn_yaw_rate_max;
   float Smax = AImax*360/2/PI/MYR;
   float AI = min(AImax, AImax/Smax*hfc->gps_speed);

   AI = min(AI, (1-MinBlend)*AImax/max(1, MaxSpeed)*hfc->gps_speed+MinBlend*AImax);
   hfc->acc_dyn_turns = AI;

   if (hfc->gps_speed<1) {
     hfc->dyn_yaw_rate = MYR;
   }
   else {
     hfc->dyn_yaw_rate = AI*360/2/PI/hfc->gps_speed;
   }
}

float TelemSerial::CalcFTWPlimit(char gps_in_cruise)
{
    float current_speed = hfc->cruise_mode && gps_in_cruise ? hfc->gps_speed : hfc->pid_Dist2T.COmax;
#ifdef THRUST_VECTORING
    float speed = power1_7(current_speed)/max(0.1f, hfc->acc_dyn_turns);
    float limit = pConfig->FTWP_retire_sr_factor*speed;
#else
    float limit = 360*current_speed/hfc->dyn_yaw_rate/2/PI*pConfig->FTWP_retire_sr_factor;
    limit += 4; // it appears to overshoot by 4m regardless of speed, could be yaw acc
#endif
    return limit;
}

void TelemSerial::SetWaypoint(float lat, float lon, float altitude, unsigned char waypoint_type, unsigned char wp_retire)
{
//    printf("Telemetry_SetWaypoint %f %f\r\n", lat, lon);
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
//        prev_STcourse = hfc->IMUorient[YAW]*R2D;	// use current heading
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
    hfc->altitude_WPnext = -9999;	// mark as used
    
    hfc->distance2WP_min = 999999;

    /* pre-calculate waypoint path data */

    /* distance to the next waypoint */
    hfc->waypoint_STdist = DistanceCourse(hfc->waypoint_pos_prev[0], hfc->waypoint_pos_prev[1], hfc->waypoint_pos[0], hfc->waypoint_pos[1], &hfc->waypoint_STcourse);
//    printf("S->T: dist = %4.1f, cour = %4.1f\r\n", hfc->waypoint_STdist, hfc->waypoint_STcourse);
    
    Sx = 0;
    Sy = 0;
    Tx = (float)(hfc->waypoint_pos[1] - hfc->waypoint_pos_prev[1]);
    Ty = (float)(hfc->waypoint_pos[0] - hfc->waypoint_pos_prev[0]);
    Tx = Tx/DPM*COSfD(((float)hfc->waypoint_pos[0]));
    Ty = Ty/DPM;
//    printf("S: %f %f\n", Sx, Sy);
//    printf("T: %f %f\n", Tx, Ty);
    
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

    if (hfc->waypoint_type==WAYPOINT_FLYTHROUGH)
      ofs = CalcFTWPlimit(false);
    else
      ofs = pConfig->GTWP_retire_radius;
    hfc->waypoint_STofs = ofs;
    hfc->waypoint_STdist = Max(1, hfc->waypoint_STdist - ofs);
}

bool TelemSerial::CheckRangeAndSetF(float *pvalue, byte *pivalue, float vmin, float vmax)
{
    float value = *((float*)pivalue);
//  printf("%f\r\n", value);
    if (!pvalue || value>vmax || value<vmin) {
        return false;
    }

    *pvalue = value;
    return true;
}

bool TelemSerial::CheckRangeAndSetI(int *pvalue, byte *pivalue, int vmin, int vmax)
{
    int value = *((int*)pivalue);
    if (!pvalue || value>vmax || value<vmin) {
        return false;
    }

    *pvalue = value;
    return true;
}

bool TelemSerial::CheckRangeAndSetB(byte *pvalue, byte *pivalue, int vmin, int vmax)
{
    int value = *((int*)pivalue);
    if (!pvalue || value>vmax || value<vmin) {
        return false;
    }

    *pvalue = value;
    return true;
}

bool TelemSerial::ProcessParameters(T_Telem_Params4 *msg)
{
    unsigned int i;
    unsigned int params = (msg->hdr.len+1)/6;
    bool waypoint = false;
    int waypoint_type  = WAYPOINT_GOTO;
    int wp_retire = false;
    float lat=0, lon=0, altitude=-9999;

//    printf("ProcessParameters %d\r\n", params);

    if (params>42)
        return false;
        
    for (i=0; i<params; i++)
    {
        T_ParamStruct *p = &msg->data[i];
        byte param = p->param;
        byte sub_param = p->sub_param;
        
//        printf("i=%d param=%d sub=%d\r\n", i, param, sub_param);

        if (param==TELEM_PARAM_WAYPOINT)
        {
            if (sub_param==TELEM_PARAM_WP_LATITUDE)
            {
                if (CheckRangeAndSetF(&lat, p->data, -90, 90))
                    waypoint = true;
            }
            else
            if (sub_param==TELEM_PARAM_WP_LONGITUDE)
            {
                if (CheckRangeAndSetF(&lon, p->data, -180, 180))
                    waypoint = true;
            }
            else
            if (sub_param==TELEM_PARAM_WP_ALTITUDE)
                CheckRangeAndSetF(&altitude, p->data, -8999, 9999);
            else
            if (sub_param==TELEM_PARAM_WP_MAX_H_SPEED)
                CheckRangeAndSetF(&hfc->pid_Dist2T.COmax, p->data, 0.1f, 50);
            else
            if (sub_param==TELEM_PARAM_WP_MAX_H_ACC)
                CheckRangeAndSetF(&hfc->pid_Dist2T.acceleration, p->data, 0.1f, 100);
            else
            if (sub_param==TELEM_PARAM_WP_MAX_V_SPEED)
            {
                if (CheckRangeAndSetF(&hfc->pid_CollAlt.COmax, p->data, 0.1f, 10)) {
                	pConfig->VspeedMax = hfc->pid_CollAlt.COmax;
                }
            }
            else
            if (sub_param==TELEM_PARAM_WP_MAX_V_ACC)
            {
                if (CheckRangeAndSetF(&hfc->pid_CollAlt.acceleration, p->data, 0.1f, 100)) {
                    pConfig->VspeedAcc = hfc->pid_CollAlt.acceleration;
                }
            }
            else
            if (sub_param==TELEM_PARAM_WP_TYPE)
                CheckRangeAndSetI(&waypoint_type, p->data, 0, 1);
            else
            if (sub_param==TELEM_PARAM_WP_RETIRE)
                CheckRangeAndSetI(&wp_retire, p->data, 0, 1);
            else
            if (sub_param==TELEM_PARAM_WP_YAWSPEEDRATE)
            {
//                CheckRangeAndSetF(&pConfig->yaw_rate_speed, p->data, 10, 10000);
            }
            else
            if (sub_param==TELEM_PARAM_WP_GTWP_RADIUS)
                CheckRangeAndSetF(&pConfig->GTWP_retire_radius, p->data, 0, 20);
            else
            if (sub_param==TELEM_PARAM_WP_GTWP_SPEED)
                CheckRangeAndSetF(&pConfig->GTWP_retire_speed, p->data, 0, 20);
            else
            if (sub_param==TELEM_PARAM_WP_FTWP_SR_FACTOR)
                CheckRangeAndSetF(&pConfig->FTWP_retire_sr_factor, p->data, 0, 10);
            else
            if (sub_param==TELEM_PARAM_WP_LOW_SPEED_LMT)
                CheckRangeAndSetF(&pConfig->low_speed_limit, p->data, 1, 30);
            else
            if (sub_param==TELEM_PARAM_WP_MIN_V_SPEED)
            {
                if (CheckRangeAndSetF(&hfc->pid_CollAlt.COmin, p->data, -10, -0.5)) {
                	pConfig->VspeedMin = hfc->pid_CollAlt.COmin;
                }
            }
            else
            if (sub_param==TELEM_PARAM_WP_ALTITUDE_BASE)
                CheckRangeAndSetF(&altitude, p->data, 0, 9999);
        }
        else
        if (param==TELEM_PARAM_STICK)
        {
            if (sub_param==TELEM_PARAM_STICK_PR_RATE)
            {
                if (CheckRangeAndSetF(&pConfig->PRstickRate, p->data, 10, 500))
                    hfc->PRstick_rate  = pConfig->PRstickRate/pConfig->Stick100range;
            }
            else
            if (sub_param==TELEM_PARAM_STICK_PR_ANGLE)
            {
                if (CheckRangeAndSetF(&pConfig->PRstickAngle, p->data, -180, 180))
                    hfc->PRstick_angle  = pConfig->PRstickAngle/pConfig->Stick100range;
            }
            else
            if (sub_param==TELEM_PARAM_STICK_HSPEED)
            {
                if (CheckRangeAndSetF(&pConfig->StickHspeed, p->data, 0.1f, 50))
                    hfc->Stick_Hspeed  = pConfig->StickHspeed/pConfig->Stick100range;
            }
            else
            if (sub_param==TELEM_PARAM_STICK_VSPEED)
            {
                if (CheckRangeAndSetF(&pConfig->StickVspeed, p->data, 0.1f, 0.8f*pConfig->VspeedMax))
                    hfc->Stick_Vspeed  = pConfig->StickVspeed/pConfig->Stick100range;
            }
            else
            if (sub_param==TELEM_PARAM_STICK_Y_RATE)
            {
                if (CheckRangeAndSetF(&pConfig->YawStickRate, p->data, 10, 500))
                    hfc->YawStick_rate  = pConfig->YawStickRate/pConfig->Stick100range;
            }
            else
            if (sub_param==TELEM_PARAM_STICK_PR_ROTATE)
                CheckRangeAndSetF(&pConfig->RollPitchAngle, p->data, -360, 360);
            else
            if (sub_param==TELEM_PARAM_STICK_H_ACC)
                CheckRangeAndSetF(&pConfig->StickHaccel, p->data, 0.5f, 9999);
        }
        else
        if (param==TELEM_PARAM_JOYSTICK)
        {
            if (sub_param==TELEM_PARAM_JOY_PITCH)
            {
                if (CheckRangeAndSetF(&hfc->joy_values[PITCH], p->data, -1, 1))
                    hfc->joystick_new_values = 1;
                hfc->joy_PRmode = true;
            }
            else
            if (sub_param==TELEM_PARAM_JOY_ROLL)
            {
                if (CheckRangeAndSetF(&hfc->joy_values[ROLL], p->data, -1, 1))
                    hfc->joystick_new_values = 1;
                hfc->joy_PRmode = true;
            }
            else
            if (sub_param==TELEM_PARAM_JOY_YAW)
            {
                if (CheckRangeAndSetF(&hfc->joy_values[YAW], p->data, -1, 1))
                    hfc->joystick_new_values = 1;
            }
            else
            if (sub_param==TELEM_PARAM_JOY_COLLECTIVE)
            {
                if (CheckRangeAndSetF(&hfc->joy_values[COLL], p->data, -1, 1))
                    hfc->joystick_new_values = 1;
            }
            else
            if (sub_param==TELEM_PARAM_JOY_THROTTLE)
            {
                if (CheckRangeAndSetF(&hfc->joy_values[THRO], p->data, -1, 1))
                    hfc->joystick_new_values = 1;
                hfc->joy_PRmode = false;
            }
            else
            if (sub_param==TELEM_PARAM_JOY_DELTA_ALT)
            {
                float dAlt;

                // TODO::SP: FIX
                /*if (CheckRangeAndSetF(&dAlt, p->data, -10, 10))
                    AltitudeUpdate(dAlt, 1); */
            }
        }
        else
        if (param==TELEM_PARAM_CONTROL)
        {
            if (sub_param==TELEM_PARAM_CTRL_HEADING_REL)
            {
                float value;

                // TODO::SP: FIX
                /*if (CheckRangeAndSetF(&value, p->data, -180, 180))
                    HeadingUpdate(value, 1);*/
            }
            else
            if (sub_param==TELEM_PARAM_CTRL_HEADING_ABS)
                CheckRangeAndSetF(&hfc->ctrl_out[ANGLE][YAW], p->data, -180, 180);
            else
            if (sub_param==TELEM_PARAM_CTRL_LIDAR_ALT)
                CheckRangeAndSetI(&pConfig->ManualLidarAltitude, p->data, 0, 1);
            else
            if (sub_param==TELEM_PARAM_CTRL_WIND_COMP)
                CheckRangeAndSetI(&pConfig->wind_compensation, p->data, 0, 1);
            else
            if (sub_param==TELEM_PARAM_CTRL_PATH_NAVIG)
                CheckRangeAndSetI(&pConfig->path_navigation, p->data, 0, 1);
            else
            if (sub_param==TELEM_PARAM_CTRL_ANGLE_COLL_MIX)
                CheckRangeAndSetF(&pConfig->AngleCollMixing, p->data, 0, 2);
            else
            if (sub_param==TELEM_PARAM_CTRL_THR_OFFSET)
                CheckRangeAndSetF(&hfc->throttle_offset, p->data, -1, 1);
            else
            if (sub_param==TELEM_PARAM_CTRL_CRUISE_LIMIT)
                CheckRangeAndSetF(&pConfig->cruise_speed_limit, p->data, 0, 100);
            else
            if (sub_param==TELEM_PARAM_CTRL_YAW_ACC)
                CheckRangeAndSetF(&hfc->pid_YawAngle.acceleration, p->data, 0.1f, 10000);
            else
            if (sub_param==TELEM_PARAM_CTRL_NOSE2WP)
                CheckRangeAndSetI(&pConfig->nose_to_WP, p->data, 0, 1);
            else
            if (sub_param==TELEM_PARAM_CTRL_WINDLIMIT)
                CheckRangeAndSetF(&pConfig->landing_wind_threshold, p->data, 0, 100);
            else
            if (sub_param==TELEM_PARAM_CTRL_BAT_CAPACITY)
            {
                if (CheckRangeAndSetI(&pConfig->battery_capacity, p->data, 1, 1000000))
                {
                    hfc->power.capacity_total = pConfig->battery_capacity/1000.0f*3600; // As
                    hfc->power.energy_total   = hfc->power.capacity_total * pConfig->battery_cells * 3.7f;  // Ws
                }
            }
            else
            if (sub_param==TELEM_PARAM_CTRL_WINDTAB_SCALE)
            {
                /* TODO::SP: This will be in main
                if (CheckRangeAndSetF(&pConfig->WindTableScale, p->data, 0.1f, 10))
                    GenerateSpeed2AngleLUT();
                    */
            }
//            else
//            if (sub_param==TELEM_PARAM_CTRL_VSPEED_PID_D)
//                CheckRangeAndSetF(&pConfig->VspeedDterm, p->data, -10, 10);
        }
        else if (param==TELEM_PARAM_PID_TUNE)
        {
            /* indicate that the PID parameters were changed to dump them to a file on disarm */
            hfc->pid_params_changed = true;
            if (sub_param==TELEM_PARAM_PID_PITCH_RATE_P)
                CheckRangeAndSetF(&hfc->pid_PitchRate.Kp, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_PITCH_RATE_I)
                CheckRangeAndSetF(&hfc->pid_PitchRate.Ki, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_PITCH_RATE_D)
                CheckRangeAndSetF(&hfc->pid_PitchRate.Kd, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_ROLL_RATE_P)
                CheckRangeAndSetF(&hfc->pid_RollRate.Kp, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_ROLL_RATE_I)
                CheckRangeAndSetF(&hfc->pid_RollRate.Ki, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_ROLL_RATE_D)
                CheckRangeAndSetF(&hfc->pid_RollRate.Kd, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_YAW_RATE_P)
                CheckRangeAndSetF(&hfc->pid_YawRate.Kp, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_YAW_RATE_I)
                CheckRangeAndSetF(&hfc->pid_YawRate.Ki, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_YAW_RATE_D)
                CheckRangeAndSetF(&hfc->pid_YawRate.Kd, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_PITCH_ANGLE_P)
                CheckRangeAndSetF(&hfc->pid_PitchAngle.Kp, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_PITCH_ANGLE_I)
                CheckRangeAndSetF(&hfc->pid_PitchAngle.Ki, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_PITCH_ANGLE_D)
                CheckRangeAndSetF(&hfc->pid_PitchAngle.Kd, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_ROLL_ANGLE_P)
                CheckRangeAndSetF(&hfc->pid_RollAngle.Kp, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_ROLL_ANGLE_I)
                CheckRangeAndSetF(&hfc->pid_RollAngle.Ki, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_ROLL_ANGLE_D)
                CheckRangeAndSetF(&hfc->pid_RollAngle.Kd, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_YAW_ANGLE_P)
                CheckRangeAndSetF(&hfc->pid_YawAngle.Kp, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_YAW_ANGLE_D)
                CheckRangeAndSetF(&hfc->pid_YawAngle.Kd, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_PITCH_SPEED_P)
                CheckRangeAndSetF(&hfc->pid_PitchSpeed.Kp, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_PITCH_SPEED_I)
                CheckRangeAndSetF(&hfc->pid_PitchSpeed.Ki, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_PITCH_SPEED_D)
                CheckRangeAndSetF(&hfc->pid_PitchSpeed.Kd, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_ROLL_SPEED_P)
                CheckRangeAndSetF(&hfc->pid_RollSpeed.Kp, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_ROLL_SPEED_I)
                CheckRangeAndSetF(&hfc->pid_RollSpeed.Ki, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_ROLL_SPEED_D)
                CheckRangeAndSetF(&hfc->pid_RollSpeed.Kd, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_COL_VSPEED_P)
                CheckRangeAndSetF(&hfc->pid_CollVspeed.Kp, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_COL_VSPEED_I)
                CheckRangeAndSetF(&hfc->pid_CollVspeed.Ki, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_COL_VSPEED_D)
                CheckRangeAndSetF(&hfc->pid_CollVspeed.Kd, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_COL_ALT_P)
                CheckRangeAndSetF(&hfc->pid_CollAlt.Kp, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_COL_ALT_D)
                CheckRangeAndSetF(&hfc->pid_CollAlt.Kd, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_DIST2T_P)
                CheckRangeAndSetF(&hfc->pid_Dist2T.Kp, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_DIST2T_D)
                CheckRangeAndSetF(&hfc->pid_Dist2T.Kd, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_DIST2P_P)
                CheckRangeAndSetF(&hfc->pid_Dist2P.Kp, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_DIST2P_D)
                CheckRangeAndSetF(&hfc->pid_Dist2P.Kd, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_PITCH_CRUISE_P)
                CheckRangeAndSetF(&hfc->pid_PitchCruise.Kp, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_PITCH_CRUISE_D)
                CheckRangeAndSetF(&hfc->pid_PitchCruise.Kd, p->data, -100, 100);
            else
            if (sub_param==TELEM_PARAM_PID_IMU_P)
            {
                CheckRangeAndSetF(&hfc->pid_IMU[0].Kp, p->data, -100, 100);
                CheckRangeAndSetF(&hfc->pid_IMU[1].Kp, p->data, -100, 100);
                CheckRangeAndSetF(&hfc->pid_IMU[2].Kp, p->data, -100, 100);
            }
            else
            if (sub_param==TELEM_PARAM_PID_IMU_I)
            {
                CheckRangeAndSetF(&hfc->pid_IMU[0].Ki, p->data, -100, 100);
                CheckRangeAndSetF(&hfc->pid_IMU[1].Ki, p->data, -100, 100);
                CheckRangeAndSetF(&hfc->pid_IMU[2].Ki, p->data, -100, 100);
            }
            else
            if (sub_param==TELEM_PARAM_PID_IMU_D)
            {
                CheckRangeAndSetF(&hfc->pid_IMU[0].Kd, p->data, -100, 100);
                CheckRangeAndSetF(&hfc->pid_IMU[1].Kd, p->data, -100, 100);
                CheckRangeAndSetF(&hfc->pid_IMU[2].Kd, p->data, -100, 100);
            }
        }
    }
    
    if (waypoint)
    {
        /* since this is asynchronous to the main loop, the control_mode change might be detected and the init skipped,
        ** thus it needs to be done here */
    	if (hfc->control_mode[PITCH] < CTRL_MODE_POSITION)
    	{
            PID_SetForEnable(&hfc->pid_Dist2T, 0, 0, hfc->gps_speed);
            PID_SetForEnable(&hfc->pid_Dist2P, 0, 0, 0);
            hfc->speedCtrlPrevEN[0] = 0;
            hfc->speedCtrlPrevEN[1] = 0;
    	}
        SetWaypoint(lat, lon, altitude, waypoint_type, wp_retire);
    }
    else
    if (altitude>-9999)
    {
        hfc->ctrl_out[POS][COLL] = hfc->altitude_base + altitude;
        hfc->waypoint_pos[2]     = hfc->ctrl_out[POS][COLL];
    }
    return true;
}

bool TelemSerial::CopyCommand(T_Telem_Commands5 *msg)
{
    if (hfc->command.command != TELEM_CMD_NONE) {
#if defined(DEBUG)
        printf("Previous command not processed yet, overwriting it!!!!!!!!!! %d\r\n", hfc->command.command);
#endif
    }

    hfc->command.command = msg->command;
    hfc->command.sub_cmd = msg->sub_cmd;

    *((int*)(hfc->command.data+0)) = *((int*)(msg->data+0));
    *((int*)(hfc->command.data+4)) = *((int*)(msg->data+4));
    *((int*)(hfc->command.data+8)) = *((int*)(msg->data+8));
    *((int*)(hfc->command.data+12))= *((int*)(msg->data+12));

    return true;
}

void TelemSerial::SetHome(void)
{
    GpsData gps_data = gps.GetGpsData();

    hfc->home_pos[0] = gps_data.latF;
    hfc->home_pos[1] = gps_data.lonF;
    hfc->home_pos[2] = hfc->altitude;
    hfc->altitude_base = hfc->home_pos[2];
}

void TelemSerial::SendMsgToGround(int msg_id)
{
	hfc->msg2ground_id = msg_id;
	hfc->msg2ground_count = MSG2GROUND_RESEND_COUNT;
}

/* returns true when everything is ok, false otherwise */
char TelemSerial::PreFlightChecks(void)
{
    int gps_error;

	// Resetting IMU before pre-flight check, to deal with noisy compass type
    ResetIMU(false);

    /* check gyro to be well calibrated and still */
    if (ABS(hfc->gyro_lp_disp[0])>0.05f || ABS(hfc->gyro_lp_disp[1])>0.05f || ABS(hfc->gyro_lp_disp[2])>0.05f)
    {
        SendMsgToGround(MSG2GROUND_PFCHECK_GYRO);
        return false;
    }

    /* check that acc is within limits */
    if (ABS(hfc->accFilt[0])>0.3f || ABS(hfc->accFilt[1])>0.3f || hfc->accFilt[2]>1.1f || hfc->accFilt[2]<0.9f)
    {
        SendMsgToGround(MSG2GROUND_PFCHECK_ACC);
        return false;
    }

    /* IMU and ACC horizon estimation have to be close to each other */
    if (ABS(hfc->SmoothAcc[PITCH] - hfc->IMUorient[PITCH])>(0.5f*D2R) || ABS(hfc->SmoothAcc[ROLL] - hfc->IMUorient[ROLL])>(0.5f*D2R))
    {
        SendMsgToGround(MSG2GROUND_PFCHECK_IMUACC_HORIZ);
        return false;
    }

#define COMP_IMU_ORIENT_DELTA 2
    /* IMU and compass heading estimation have to be close to each other */
    if (ABS(hfc->compass_heading_lp - hfc->IMUorient[YAW]*R2D)>COMP_IMU_ORIENT_DELTA )
    {
        SendMsgToGround(MSG2GROUND_PFCHECK_IMUCOMP_HEAD);
        return false;
    }

    /* baro */
    if (hfc->baro_pressure<60000 || hfc->baro_pressure>110000 || hfc->baro_altitude_raw_lp<-500 || hfc->baro_altitude_raw_lp>8000)
    {
        SendMsgToGround(MSG2GROUND_PFCHECK_BARO);
        return false;
    }

    /* compass */
    if (ABS(compass.dataXYZcalib[0])>600 || ABS(compass.dataXYZcalib[1])>600 || compass.dataXYZcalib[2]<-600 || compass.dataXYZcalib[2]>-350)
    {
        SendMsgToGround(MSG2GROUND_PFCHECK_COMPASS);
        return false;
    }

    /* power module */
    if (pConfig->power_node)
    {
        if (hfc->stats.can_power_tx_errors)
        {
            SendMsgToGround(MSG2GROUND_PFCHECK_CAN_POWER);
            return false;
        }

        /* battery */
        if (hfc->power.battery_level<10 || (hfc->power.Vmain/pConfig->battery_cells)<3.6f || (hfc->power.Vesc/pConfig->battery_cells)<3.6f)
        {
            SendMsgToGround(MSG2GROUND_PFCHECK_BATTERY);
            return false;
        }
    }

    /* angle, less than 5 deg */
    if (ABS(hfc->IMUorient[0]*R2D)>5 || ABS(hfc->IMUorient[1]*R2D)>5)
    {
        SendMsgToGround(MSG2GROUND_PFCHECK_ANGLE);
        return false;
    }

    /* servo module */
    if (pConfig->can_servo && hfc->stats.can_servo_tx_errors)
    {
        SendMsgToGround(MSG2GROUND_PFCHECK_CAN_SERVO);
        return false;
    }

    /* GPS */
    gps_error = gps.PreFlightCheck();
    if (gps_error)
    {
        SendMsgToGround(gps_error-1+MSG2GROUND_PFCHECK_GPS_NOFIX);
        return false;
    }

    if (ABS(hfc->altitude-hfc->altitude_gps)>2)
    {
        SendMsgToGround(MSG2GROUND_PFCHECK_IMU_GPS_ALT);
        return false;
    }
    return true;
}

void TelemSerial::CommandTakeoffArm(void)
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
    //ResetIterms();    // TODO::SP: Declared in MAin
    
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
//    hfc->pid_CollVspeed.COlast = pConfig->CollZeroAngle;	// needed once alt hold is enabled and this could be uninitialized for the takeoff condition

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
    
	hfc->message_from_ground = 0;	// reset it so we can wait for the message from ground
	hfc->waypoint_type   = WAYPOINT_TAKEOFF;
	if (hfc->full_auto)
		hfc->waypoint_stage  = FM_TAKEOFF_AUTO_SPOOL;
	else
		hfc->waypoint_stage  = FM_TAKEOFF_ARM;
    hfc->message_timeout = 60000000;	// 60 seconds
}

/* vspeed needs to be negative */
void TelemSerial::CommandLandingWP(float lat, float lon, float alt_ground)
{
    hfc->gps_to_waypoint[0] = 99;	// need something here so the logic does not immediatelly think it is already there
    								// before it gets properly initialized
    /* set 2D waypoint at the current location */
    SetWaypoint(lat, lon, alt_ground, WAYPOINT_GOTO, 0);

    hfc->waypoint_type  = WAYPOINT_LANDING;
    hfc->waypoint_stage = FM_LANDING_WAYPOINT;
}

/* vspeed needs to be negative */
void TelemSerial::CommandLanding(bool final, bool setWP)
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
            if (hfc->wind_speed>pConfig->landing_wind_threshold)  // 3m/s
                hfc->ctrl_out[ANGLE][YAW] = Wrap180(hfc->wind_course);
            else
                hfc->ctrl_out[ANGLE][YAW]   = hfc->IMUorient[YAW]*R2D;

            /* if switching from manual collective, initialize the ctrl speed with the current one */
            if (hfc->control_mode[COLL]<CTRL_MODE_SPEED)
            	hfc->ctrl_out[SPEED][COLL] = hfc->IMUspeedGroundENU[UP];
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

void TelemSerial::Disarm(void)
{
	hfc->throttle_armed    = 0;
	hfc->inhibitRCswitches = false;
	hfc->waypoint_type   = WAYPOINT_NONE;
	hfc->playlist_status = PLAYLIST_STOPPED;
	hfc->LidarCtrlMode   = false;

	// TODO::SP: Need to do this
	//Save_PIDvalues(hfc);
}

void TelemSerial::PlaylistSaveState(void)
{
	T_State *state = &hfc->state;
	int i;
    state->pid_CollAlt_COmax = hfc->pid_CollAlt.COmax;
    state->pid_CollAlt_COmin = hfc->pid_CollAlt.COmin;
    state->pid_CollAlt_acc   = hfc->pid_CollAlt.acceleration;
    state->pid_Dist2T_COmax  = hfc->pid_Dist2T.COmax;
    state->pid_Dist2T_acc    = hfc->pid_Dist2T.acceleration;
    state->acc_dyn_turns     = hfc->acc_dyn_turns;
    state->altitude          = hfc->ctrl_out[POS][COLL];
    state->telem_ctrl_period = hfc->telem_ctrl_period;
    state->ctrl_source       = hfc->ctrl_source;
    state->waypoint_type     = hfc->waypoint_type;
    for (i=0; i<5; i++)
    	state->control_mode[i] = hfc->control_mode[i];
}

void TelemSerial::PlaylistRestoreState(void)
{
	T_State *state = &hfc->state;
	int i;
	hfc->pid_CollAlt.COmax        = state->pid_CollAlt_COmax;
	hfc->pid_CollAlt.COmin        = state->pid_CollAlt_COmin;
	hfc->pid_CollAlt.acceleration = state->pid_CollAlt_acc;
	hfc->pid_Dist2T.COmax         = state->pid_Dist2T_COmax;
	hfc->pid_Dist2T.acceleration  = state->pid_Dist2T_acc;
	hfc->acc_dyn_turns            = state->acc_dyn_turns;
	hfc->ctrl_out[POS][COLL]      = state->altitude;
	hfc->telem_ctrl_period        = state->telem_ctrl_period;
	hfc->ctrl_source              = state->ctrl_source;
	hfc->waypoint_type            = state->waypoint_type;
    for (i=0; i<5; i++)
    	hfc->control_mode[i] = state->control_mode[i];

    /* re-initialize the waypoint and everything related, since the playlist is not in PLAYING yet,
     * it will use the current position as the starting point */
    SetWaypoint(hfc->waypoint_pos[0], hfc->waypoint_pos[1],
                    (hfc->waypoint_pos[2] - hfc->altitude_base), hfc->waypoint_type, hfc->waypoint_retire);

    SaveValuesForAbort();
	hfc->playlist_status = PLAYLIST_PLAYING;
	hfc->delay_counter = 0;
}

int TelemSerial::FindNearestLandingSite(void)
{
	int i;
	int index = -1;
	float currlat = hfc->positionLatLon[0];
	float currlon = hfc->positionLatLon[1];
	float nearest_dist = 9999999;
	/* no landing sites loaded */
	if (!hfc->landing_sites_num)
		return -1;

	for (i=0; i<hfc->landing_sites_num; i++)
	{
		float dist = Distance(hfc->landing_sites[i].lat, hfc->landing_sites[i].lon, currlat, currlon);
		if (dist < nearest_dist)
		{
			nearest_dist = dist;
			index = i;
		}
	}
	return index;
}

void TelemSerial::Arm(void)
{
    if (hfc->throttle_armed)
        return;

    /* throttle level needs to be low */
    if (hfc->throttle_value > -0.95f*pConfig->Stick100range)
    {
        SendMsgToGround(MSG2GROUND_ARMING_THROTTLE);
        return;
    }

    /* cannot arm in manual mode unless explicitely allowed */
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

void TelemSerial::ResetMBED(void)
{
    // TODO::SP: FIX
    /*
    Save_PIDvalues();
    mbed_interface_reset();
    */
}

void TelemSerial::ProcessCommands(void)
{
    byte cmd = hfc->command.command;
    byte sub_cmd = hfc->command.sub_cmd;
    
    if (cmd==TELEM_CMD_NONE)
        return;

//  printf("Command %d\r\n", cmd);

    if (cmd==TELEM_CMD_ARMING)
    {
        if (sub_cmd==CMD_ARM)   // 0-disarm, 1-arm
        {
            Arm();
            hfc->waypoint_type = WAYPOINT_NONE;
        }
        else
        	Disarm();
    }
    else
    if (cmd==TELEM_CMD_SET_HOME)
    {
        SetHome();
    }
    // NOTE::SP: This is no longer used, removing....
#if 0
    else
    if (cmd==TELEM_CMD_CALIBRATE)
    {
        if (!hfc->calibrate)
        {
#if defined(DEBUG)
            printf("Calibrating.......\r\n");
#endif
            hfc->calibrate = CALIBRATE_SAMPLES;
        }
    }
#endif
    else
    if (cmd==TELEM_CMD_JOYSTICK)
    {
        if (sub_cmd==CMD_JOYSTICK_ENABLE)   // 0-disable, 1-enable
        {
            hfc->joy_values[PITCH]= 0;
            hfc->joy_values[ROLL] = 0;
            hfc->joy_values[YAW]  = 0;
            hfc->joy_values[COLL] = 0;
            hfc->joy_values[THRO] = 0;
            hfc->joy_PRmode = true;

            /* altitude hold and yaw angle */
            SetCtrlMode(hfc, pConfig, PITCH, CTRL_MODE_SPEED);
            SetCtrlMode(hfc, pConfig, ROLL,  CTRL_MODE_SPEED);
            SetCtrlMode(hfc, pConfig, YAW,   CTRL_MODE_ANGLE);
            SetCtrlMode(hfc, pConfig, COLL,  CTRL_MODE_POSITION);
            /* set target altitude and heading to the current one */
            hfc->ctrl_out[ANGLE][YAW] = hfc->IMUorient[YAW]*R2D; 
            hfc->joystick_new_values = 1;

        	if (hfc->playlist_status==PLAYLIST_PLAYING)
        	{
        		PlaylistSaveState();
                SelectCtrlSource(CTRL_SOURCE_JOYSTICK);
        		hfc->playlist_status = PLAYLIST_PAUSED;
        	}
        	else
			if (hfc->playlist_status==PLAYLIST_PAUSED)
			{
				SelectCtrlSource(CTRL_SOURCE_JOYSTICK);
				hfc->playlist_status = PLAYLIST_PAUSED;
			}
			else
                SelectCtrlSource(CTRL_SOURCE_JOYSTICK);
        }
        else
        {
        	/* disabling joystick */
        	if (hfc->playlist_status == PLAYLIST_PAUSED)
        		PlaylistRestoreState();
        	else
        		SelectCtrlSource(CTRL_SOURCE_RCRADIO);
        }
    }
    else
    if (cmd==TELEM_CMD_GOTO_HOME)
    {
    	ApplyDefaults();
        SetWaypoint(hfc->home_pos[0], hfc->home_pos[1], -9999, WAYPOINT_GOTO, 0); // do not change altitude or perhaps use a preset value for this
    }
    else
    if (cmd==TELEM_CMD_PLAYLIST_CONTROL)
    {
        if (sub_cmd==PLAYLIST_PLAY)
        {
            hfc->playlist_position = 0;
            if (hfc->playlist_items > 0)
            {
                hfc->playlist_status = PLAYLIST_PLAYING;
                hfc->pl_wp_initialized = false;
                hfc->altitude_WPnext   = -9999;  // altitude unchanged by default, can be set from playlist
                hfc->delay_counter     = 0;
                
                /* set default values */
            	ApplyDefaults();
                /* initialize current waypoint with the current position */
                hfc->waypoint_pos[0] = hfc->positionLatLon[0];
                hfc->waypoint_pos[1] = hfc->positionLatLon[1];
                hfc->waypoint_pos[2] = hfc->altitude;
                
                /* reset GOTO counters !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
            }
        }
        else
        if (sub_cmd==PLAYLIST_JUMP)
        {
        }
        else
        if (sub_cmd==PLAYLIST_PAUSE)
        {
        	if (hfc->playlist_status==PLAYLIST_PLAYING)
        	{
            	PlaylistSaveState();
                SelectCtrlSource(CTRL_SOURCE_RCRADIO);
        		hfc->playlist_status = PLAYLIST_PAUSED;
        		if (hfc->full_auto)
        		{
        			/* set speed mode since RCradio switches do not work in full_auto */
                    SetCtrlMode(hfc, pConfig, PITCH, CTRL_MODE_SPEED);
                    SetCtrlMode(hfc, pConfig, ROLL,  CTRL_MODE_SPEED);
                    SetCtrlMode(hfc, pConfig, YAW,   CTRL_MODE_ANGLE);
                    SetCtrlMode(hfc, pConfig, COLL,  CTRL_MODE_POSITION);
                    /* zero horizontal speed, unlikely a slow acceleration */
                    hfc->ctrl_out[SPEED][PITCH] = 0;
                    hfc->ctrl_out[SPEED][ROLL]  = 0;
                    hfc->ctrl_out[POS][COLL] = hfc->altitude;
        		}

        	}
        }
        else
        if (sub_cmd==PLAYLIST_RESUME)
        {
        	if (hfc->playlist_status == PLAYLIST_PAUSED)
        		PlaylistRestoreState();
        }
        else
        if (sub_cmd==PLAYLIST_STOP)
        {
            /* stop playlist and waypoint mode */
            SelectCtrlSource(CTRL_SOURCE_RCRADIO);
        }
    }
    else
    if (cmd==TELEM_CMD_TAKEOFF)
    {
        if (sub_cmd==TAKEOFF_ARM) {
            CommandTakeoffArm();
        }
    }
    else
    if (cmd==TELEM_CMD_LAND)
    {
        hfc->playlist_status = PLAYLIST_STOP;
        if (sub_cmd==LANDING_CURRENT) {
        	CommandLanding(false, true);
        }
        else if (sub_cmd==LANDING_WAYPOINT)
        {
            float lat = *((float*)&hfc->command.data[4]);	// lat as float
            float lon = *((float*)&hfc->command.data[8]);	// lon as float
        	CommandLandingWP(lat, lon, 10);
        	hfc->pid_Dist2T.COmax = pConfig->landing_appr_speed;
        }
        else if (sub_cmd==LANDING_SITE)
        {
        	int site = FindNearestLandingSite();
//        	printf("Landing at %d\r\n", site);
        	if (site>=0)
        	{
        		float alt_ground = hfc->landing_sites[site].altitude - hfc->altitude_base + hfc->landing_sites[site].above_ground ;
        		CommandLandingWP(hfc->landing_sites[site].lat, hfc->landing_sites[site].lon, alt_ground);
            	hfc->pid_Dist2T.COmax = pConfig->landing_appr_speed;
        	}
        	else {
            	CommandLanding(false, true);
        	}
        }
    }
    else
    if (cmd==TELEM_CMD_POS_HOLD)
    {
        SetPositionHold();
    }
    else if (cmd==TELEM_CMD_GPS_NEXT)
    {
        gps.SetNextChannel();
    }
    else if (cmd==TELEM_CMD_MSG)
    {
        if (sub_cmd==CMD_MSG_LANDING_ADD1MIN)
        {
            if (hfc->message_timeout<60000000)
                hfc->message_timeout += 60000000;   // add 60sec, only once to prevent multiple messages to keep incrementing, until TCPIP works
        }
        else
            hfc->message_from_ground = sub_cmd;
    }
    else if (cmd==TELEM_CMD_KILLSWITCH)
    {
    	if (sub_cmd==KILLSWITCH_AUTOROTATE)
    	{
            SelectCtrlSource(CTRL_SOURCE_AUTO3D);
            SetCtrlMode(hfc, pConfig, PITCH, CTRL_MODE_ANGLE);
            SetCtrlMode(hfc, pConfig, ROLL,  CTRL_MODE_ANGLE);
            SetCtrlMode(hfc, pConfig, YAW,   CTRL_MODE_ANGLE);
            SetCtrlMode(hfc, pConfig, COLL,  CTRL_MODE_MANUAL);
            hfc->ctrl_angle_pitch_3d 	= 0;
            hfc->ctrl_angle_roll_3d  	= 0;
            hfc->ctrl_out[RAW][COLL] 	= pConfig->CollAngleAutoRotate;
        	hfc->auto_throttle 			= true;
        	hfc->throttle_value 		= -pConfig->Stick100range;
    	}
    }
    else if (cmd==TELEM_CMD_TOGGLE_PWR)
    {
        if (sub_cmd==TOGGLE_PWR_AUX12V)   // toggle aux12V power
            hfc->power.power_aux12v     = !hfc->power.power_aux12v;
        else if (sub_cmd==TOGGLE_PWR_SERVO)   // toggle servo power
            hfc->power.power_servo  = !hfc->power.power_servo;
    }
    else if (cmd==TELEM_CMD_RESET_IMU_ALT)
    {
        ResetIMU(false);

        /* altitude */
        if (sub_cmd==IMURESET_GPS)
            hfc->altitude_ofs = hfc->altitude_gps - hfc->altitude_baro;
        else
        if (sub_cmd==IMURESET_ALTITUDE)
            hfc->altitude_ofs = (*((float*)&hfc->command.data[0])) - hfc->altitude_baro;
    }
    else if (cmd==TELEM_CMD_PREFLIGHT_CHECK)
    {
        if (PreFlightChecks())
            SendMsgToGround(MSG2GROUND_PFCHECK_ALL_GOOD);
    }
    else if (cmd==TELEM_CMD_RESET)
    {
        if (sub_cmd == RESET_SUBID) {
            // TODO::SP: NOT APPLICABLE ON AVI FW
            ResetMBED();
        }
    }
    else
    {
//        printf("Unknown command: %d\r\n", cmd);
    }
    
    hfc->command.command = TELEM_CMD_NONE;
}

void TelemSerial::SetPositionHold(void)
{
    /* set heading to the current one */
    hfc->ctrl_out[ANGLE][YAW] = hfc->IMUorient[YAW]*R2D; 
    /* set vcontrol to max to make sure it can hold the pos */
	ApplyDefaults();
    SetWaypoint(hfc->positionLatLon[0], hfc->positionLatLon[1],
                                (hfc->altitude - hfc->altitude_base), WAYPOINT_GOTO, 0);
    hfc->ctrl_source = CTRL_SOURCE_AUTO3D;
}

void TelemSerial::ResetIMU(bool print)
{
    int i;
    float accGroundENU[3];

    /* horizon and compass */
    hfc->IMUorient[PITCH] = hfc->SmoothAcc[PITCH];
    hfc->IMUorient[ROLL]  = hfc->SmoothAcc[ROLL];

    /* re-orient compass based on the new pitch/roll */
    hfc->compass_heading = compass.GetHeadingDeg(pConfig->comp_orient, pConfig->comp_ofs, pConfig->comp_gains, pConfig->fcm_orient,
                                                pConfig->comp_declination_offset, hfc->IMUorient[PITCH], hfc->IMUorient[ROLL]);
    hfc->compass_heading_lp = hfc->compass_heading;
    hfc->IMUorient[YAW] = hfc->compass_heading_lp*D2R;
    IMU_PRY2Q(hfc->IMUorient[PITCH], hfc->IMUorient[ROLL], hfc->IMUorient[YAW]);

    for (i=0; i<3; i++) hfc->gyroOfs[i] += hfc->gyro_lp_disp[i];
    for (i=0; i<3; i++) hfc->gyro_lp_disp[i] = 0;
    PID_SetForEnable(&hfc->pid_IMU[0], hfc->SmoothAcc[PITCH]*R2D, hfc->IMUorient[PITCH]*R2D, -hfc->gyroOfs[0]);
    PID_SetForEnable(&hfc->pid_IMU[1], hfc->SmoothAcc[ROLL]*R2D,  hfc->IMUorient[ROLL]*R2D,  -hfc->gyroOfs[1]);
    PID_SetForEnable(&hfc->pid_IMU[2], hfc->compass_heading,      hfc->IMUorient[YAW]*R2D,   -hfc->gyroOfs[2]);

    /*mmri: just took out the carriage return so that gyrotemp
     * compensation output data is more easily read in .csv file*/
#if defined(DEBUG)
    if (print)
        printf("PRY %+5.1f %+5.1f %+5.1f Ofs %+5.3f %+5.3f %+5.3f ===",
                hfc->IMUorient[PITCH]*R2D, hfc->IMUorient[ROLL]*R2D, hfc->IMUorient[YAW]*R2D,
                hfc->gyroOfs[0], hfc->gyroOfs[1], hfc->gyroOfs[2]);
#endif

    Plane2Ground(hfc->accHeliRFU, hfc->IMUorient, accGroundENU);

    accGroundENU[2] -= 1; // remove gravity
    for (i=0; i<3; i++)
    {
        hfc->accGroundENUhp[i] = 0;
        hfc->accGroundENU_prev[i] = accGroundENU[i];
        hfc->IMUspeedGroundENU[i] = 0;
    }
}
