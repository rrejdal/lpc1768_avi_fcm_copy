#include "pGPS.h"
#include "mymath.h"
#include "defines.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "avican.h"

GPS::GPS()
{
	selected_channel_ = 0;
	glitch_ = 0;
	glitches_ = 0;
	number_of_channels_ = 0;
	set_next_channel_ = 0;
}

int GPS::Init(int number_of_channels)
{
    if (number_of_channels > MAX_GPS_CHANNELS) {
		return 0;
	}

	number_of_channels_ = number_of_channels;

	for (int i=0; i < number_of_channels_; i++) {
		gps_channel_[i] = {0};
	}

	gps_data_ = {0};

	selected_channel_ = 0;

	return 1;
}

void GPS::AddGpsData(int channel, int msg_id, char *msg)
{
	static int new_data_mask = 0;

    if (channel >= number_of_channels_) {
        return;
    }

    switch(msg_id) {
        case AVIDRONE_MSGID_GPS_0:
        {
            GpsAviCanMsg0 *msg0 = (GpsAviCanMsg0*)msg;

            gps_channel_[channel].msg0.altitude = msg0->altitude;
            gps_channel_[channel].msg0.sats_fix = msg0->sats_fix;

            gps_channel_[channel].fix = (msg0->sats_fix & 0x3);

            gps_channel_[channel].online_ = msg0->gps_status;
            gps_channel_[channel].other_fix_ = msg0->other_sats_fix;

            //gps_channel_[channel].new_data = 1;
            new_data_mask |= (1 <<0);
            break;
        }
        case AVIDRONE_MSGID_GPS_1:
        {
            GpsAviCanMsg1 *msg1 = (GpsAviCanMsg1*)msg;
            gps_channel_[channel].msg1.lat = msg1->lat;
            gps_channel_[channel].msg1.lon = msg1->lon;
            new_data_mask |= (1 <<1);
            break;
        }
        case AVIDRONE_MSGID_GPS_2:
        {
            GpsAviCanMsg2 *msg2 = (GpsAviCanMsg2*)msg;
            gps_channel_[channel].msg2.speedE = msg2->speedE;
            gps_channel_[channel].msg2.speedN = msg2->speedN;
            gps_channel_[channel].msg2.speedU = msg2->speedU;

			gps_channel_[channel].speedENU[0] = msg2->speedE;
			gps_channel_[channel].speedENU[1] = msg2->speedN;
			gps_channel_[channel].speedENU[2] = msg2->speedU;

            gps_channel_[channel].msg2.PDOP   = msg2->PDOP;
            new_data_mask |= (1 <<2);
            break;
        }
        case AVIDRONE_MSGID_GPS_3:
        {
            GpsAviCanMsg3 *msg3 = (GpsAviCanMsg3*)msg;
            gps_channel_[channel].msg3.date = msg3->date;
            gps_channel_[channel].msg3.time = msg3->time;
            new_data_mask |= (1 <<3);
            break;
        }
        case AVIDRONE_MSGID_GPS_4:
        {
            GpsAviCanMsg4 *msg4 = (GpsAviCanMsg4*)msg;
            gps_channel_[channel].msg4.glitch_data = msg4->glitch_data;
            gps_channel_[channel].msg4.crc_err_cnt = msg4->crc_err_cnt;
            gps_channel_[channel].msg4.msg_cnt = msg4->msg_cnt;
            gps_channel_[channel].glitch =  ((gps_channel_[channel].msg4.glitch_data & 0x8000) >> 15);
            gps_channel_[channel].glitches =  gps_channel_[channel].msg4.glitch_data & 0x7FFF;
            new_data_mask |= (1 <<4);
            break;
        }
        default:
            break;
    }

    if (new_data_mask == 0x1F) {
        //printf("New Data in ch[%d]\r\n", channel);
        gps_channel_[channel].new_data = 1;
        new_data_mask = 0;
    }
}

// Returns ptr to current in use GPS data
GpsData GPS::GetGpsData(void)
{
   return (gps_data_);
}

/* called every time a new GGA message comes to calculate new speed/course
** from changing coordinates */
void GPS::CalculateSpeedCourse()
{
    float course = 0;
    int speedN = gps_channel_[selected_channel_].msg2.speedN;
    int speedE = gps_channel_[selected_channel_].msg2.speedE;

    gps_data_.HspeedC = sqrtf((float)((speedN * speedN) + (speedE * speedE))) * 0.01f;

    if ((speedN != 0) || (speedE != 0)) {
        course = 90-ATAN2fD(speedN, speedE);
    }

    if (course > 180) {
        course -=360;
    }
    else if (course < -180) {
        course += 360;
    }

    gps_data_.courseC = course;
}

void GPS::GlitchDetect(int channel)
{
	int sE = gps_channel_[channel].msg2.speedE;
	int sN = gps_channel_[channel].msg2.speedN;
	int sU = gps_channel_[channel].msg2.speedU;
	int alt = gps_channel_[channel].msg0.altitude;
	int Lat = gps_channel_[channel].msg1.lat;
	int Lon = gps_channel_[channel].msg1.lon;

	if ( (ABS(sE-gps_channel_[channel].speedENUprev[0])>500)	    // 5m/s
	        || (ABS(sN-gps_channel_[channel].speedENUprev[1])>500)  // 5m/s
	        || (ABS(sU-gps_channel_[channel].speedENUprev[2])>200)	// 2m/s
	        || (ABS(alt-gps_channel_[channel].alt_prev)>2000)		// 2m
	        || (ABS(Lat-gps_channel_[channel].lat_prev)>1800)		// 20m (1deg=111km, ~250km/h limit at 5Hz GPS)
	        || (ABS(Lon-gps_channel_[channel].lon_prev)>1800)) {    // 20m

		/* delta speed exceeded - GPS glitch */
		/* do not take the new values, use last good */
		if (!gps_channel_[channel].glitch) {
		    gps_channel_[channel].glitches++;
		}
		gps_channel_[channel].glitch = 1;
	}
	else {
	    gps_channel_[channel].speedENU[0] = sE;
	    gps_channel_[channel].speedENU[1] = sN;
	    gps_channel_[channel].speedENU[2] = sU;
	    gps_channel_[channel].glitch = 0;
	}

    gps_channel_[channel].speedENUprev[0] = sE;
    gps_channel_[channel].speedENUprev[1] = sN;
    gps_channel_[channel].speedENUprev[2] = sU;
    gps_channel_[channel].alt_prev        = alt;
    gps_channel_[channel].lat_prev		  = Lat;
    gps_channel_[channel].lon_prev		  = Lon;
}

void GPS::FindNewChannel()
{
	int PDOPmin = 999999;
	int min_channel = -1;

	/* find a good GPS with the lowest PDOP */
    for (int i=0; i < number_of_channels_; i++) {
       // T_GPSmuxChannel *pgps = &gps_channel_[i];
        if ((gps_channel_[i].msg0.sats_fix & 0x3)
                && !(gps_channel_[i].glitch) && (gps_channel_[i].last_update < 400000)) {

            if (gps_channel_[i].msg2.PDOP < PDOPmin) {
                PDOPmin = gps_channel_[i].msg2.PDOP;
                min_channel = i;
            }
        }
    }

    /* if a matching GPS is found, use it */
    if (min_channel >= 0) {
        /* changing channel, but not because of a glitch */
        if ((selected_channel_ != min_channel) && !(gps_channel_[selected_channel_].glitch)) {
            glitch_ = 1;
            ++glitches_;
        }
        selected_channel_ = min_channel;
        return;
    }

	/* it should find the GPS with the lowest PDOP */
	for (int i=0; i < number_of_channels_; i++) {
		if ((gps_channel_[i].msg0.sats_fix & 0x3)
		        && !gps_channel_[i].glitch
		        && (gps_channel_[i].last_update < 400000)) {

			/* changing channel, but not because of a glitch */
			if ((selected_channel_ != i) && !(gps_channel_[i].glitch)) {
				glitch_ = 1;
				++glitches_;
			}

			selected_channel_ = i;
			return;
		}
	}

	/* did not find anything good, but the current is running, so keep it */
	if (gps_channel_[selected_channel_].last_update < 400000) {
		return;
	}

	/* find anything running without a glitch */
	for (int i=0; i < number_of_channels_; i++) {

		if (!(gps_channel_[i].glitch) && (gps_channel_[i].last_update < 400000)) {

		    /* changing channel, but not because of a glitch */
		    if ((selected_channel_ != i) && !(gps_channel_[i].glitch)) {
				glitch_ = 1;
				++glitches_;
			}

			selected_channel_ = i;
			return;
		}
	}
}

int GPS::GpsUpdate(void)
{
    if (gps_channel_[selected_channel_].new_data) {

        gps_data_.fix = gps_channel_[selected_channel_].msg0.sats_fix & 0x3;

        if (gps_data_.fix) {

            /* Ground speed and course from N/E speeds*/
            CalculateSpeedCourse();

            /* publish latest data from the selected GPS channel (m/s) */
            gps_data_.speedENU[0] = gps_channel_[selected_channel_].speedENU[0]*0.01f;
            gps_data_.speedENU[1] = gps_channel_[selected_channel_].speedENU[1]*0.01f;
            gps_data_.speedENU[2] = gps_channel_[selected_channel_].speedENU[2]*0.01f;
        }

        gps_data_.date = gps_channel_[selected_channel_].msg3.date;
        gps_data_.time = gps_channel_[selected_channel_].msg3.time;
        gps_data_.altitude = gps_channel_[selected_channel_].msg0.altitude*0.001f;  // meters
        gps_data_.sats = gps_channel_[selected_channel_].msg0.sats_fix >> 2;
        gps_data_.PDOP = gps_channel_[selected_channel_].msg2.PDOP;     // *100
        gps_data_.lat  = gps_channel_[selected_channel_].msg1.lat;      // *10M
        gps_data_.lon  = gps_channel_[selected_channel_].msg1.lon;      // *10M
        gps_data_.latF = gps_data_.lat*0.0000001f;
        gps_data_.lonF = gps_data_.lon*0.0000001f;
        gps_data_.latD = gps_data_.lat*0.0000001;
        gps_data_.lonD = gps_data_.lon*0.0000001;

        glitches_ = gps_channel_[selected_channel_].glitches;

        gps_channel_[selected_channel_].new_data = 0;
        return 1;
    }

    return 0;

}

// Return 1 is new data available, 0 otherwise
GpsData GPS::GpsUpdate(int dTus, char* new_data_flag)
{
	int have_new_data = 0;

	glitch_ = false;

	for (int i=0; i < number_of_channels_; i++) {

		gps_channel_[i].last_update += dTus;

		if (gps_channel_[i].new_data) {

			have_new_data = 1;
			gps_channel_[i].last_update = 0;

			if (number_of_channels_ > 1) {
                GlitchDetect(i);

                if ((i==selected_channel_) && (gps_channel_[i].glitch)) {
                    ++glitches_;
                }
			}
		}
	}

	// Force next Channel comes from user button update
	if (set_next_channel_) {
	    ++selected_channel_;

		if (selected_channel_ >= number_of_channels_) {
			selected_channel_ -= number_of_channels_;
		}

		set_next_channel_ = 0;
		glitch_ = true;
	}

	/* if no new data in any channel, just return */
	if (!have_new_data) {
	    *new_data_flag = 0;
		return (gps_data_);
	}

	/* if the current channel does not have new data, return */
	if (!(gps_channel_[selected_channel_].new_data)
	        && (gps_channel_[selected_channel_].last_update < 400000))
	{
		for (int i=0; i < number_of_channels_; i++) {
			gps_channel_[i].new_data = 0;
		}
        *new_data_flag = 0;
        return (gps_data_);
	}

	if (gps_channel_[selected_channel_].glitch) {
		glitch_ = true;
	}

	/* No point in finding new channels if only have a single channel! */
	if (number_of_channels_ > 1) {
        if (!(gps_channel_[selected_channel_].fix)
                || (gps_channel_[selected_channel_].last_update > 400000)
                || (gps_channel_[selected_channel_].glitch)
                || (gps_channel_[selected_channel_].msg2.PDOP >= 300)) {

                FindNewChannel();
        }
    }

	if (gps_channel_[selected_channel_].new_data) {

	    gps_data_.fix = gps_channel_[selected_channel_].msg0.sats_fix & 0x3;

        if (gps_data_.fix) {

            /* Ground speed and course from N/E speeds*/
            CalculateSpeedCourse();

            /* publish latest data from the selected GPS channel (m/s) */
            gps_data_.speedENU[0] = gps_channel_[selected_channel_].speedENU[0]*0.01f;
            gps_data_.speedENU[1] = gps_channel_[selected_channel_].speedENU[1]*0.01f;
            gps_data_.speedENU[2] = gps_channel_[selected_channel_].speedENU[2]*0.01f;
        }

        gps_data_.date = gps_channel_[selected_channel_].msg3.date;
        gps_data_.time = gps_channel_[selected_channel_].msg3.time;
        gps_data_.altitude = gps_channel_[selected_channel_].msg0.altitude*0.001f;	// meters
        gps_data_.sats = gps_channel_[selected_channel_].msg0.sats_fix >> 2;
        gps_data_.PDOP = gps_channel_[selected_channel_].msg2.PDOP;     // *100
        gps_data_.lat  = gps_channel_[selected_channel_].msg1.lat;		// *10M
        gps_data_.lon  = gps_channel_[selected_channel_].msg1.lon;		// *10M
        gps_data_.latF = gps_data_.lat*0.0000001f;
        gps_data_.lonF = gps_data_.lon*0.0000001f;
        gps_data_.latD = gps_data_.lat*0.0000001;
        gps_data_.lonD = gps_data_.lon*0.0000001;

        glitches_ = gps_channel_[selected_channel_].glitches;

        *new_data_flag = 1;
	}

	for (int i=0; i < number_of_channels_; i++) {
		gps_channel_[i].new_data = 0;
	}

    return (gps_data_);
}

/* provides fix/sats for a channel which is not the current one */
void GPS::GetFixSatsOther(char *pfix, char *psats)
{
	for (int i = 0; i < number_of_channels_; i++) {

	    if (i != selected_channel_) {

			*pfix = gps_channel_[i].fix;
			*psats = (gps_channel_[i].msg0.sats_fix >> 2);

			// no data detection
			if (gps_channel_[i].last_update > 400000) {
				*pfix = 3;
			}

			return;
		}
	}

	*psats = 0;
	*pfix  = 3;
}

int GPS::PreFlightCheck()
{
    /* all have to be receiving data */
    for (int i=0; i < number_of_channels_; i++) {
        if (gps_channel_[i].last_update > 400000) {
            return GPS_CHECK_NOSIGNAL;
        }
    }

    /* all have to be locked */
    for (int i=0; i < number_of_channels_; i++) {
        if (!gps_channel_[i].fix) {
            return GPS_CHECK_NOFIX;
        }
    }

    /* all have to have HDOP<2 */
    for (int i=0; i < number_of_channels_; i++) {
        if (gps_channel_[i].msg2.PDOP > 200)
            return GPS_CHECK_PDOP;
    }

    /* all have to have valid altitude */
    for (int i=0; i < number_of_channels_; i++) {
        if ((gps_channel_[i].msg0.altitude < -100000) || (gps_channel_[i].msg0.altitude > 8000000)) {
            return GPS_CHECK_ALTITUDE;
        }
    }

    /* latitude and longitude have to be within 7.7m */
    for (int i=1; i < number_of_channels_; i++) {
        if ((ABS(gps_channel_[i].msg1.lat - gps_channel_[0].msg1.lat) > 700)
                || (ABS(gps_channel_[i].msg1.lon - gps_channel_[0].msg1.lon) > 700)) {
            return GPS_CHECK_POSITION;
        }
    }

    return GPS_CHECK_OK;
}
