#include "xbus.h"
#include "utils.h"
#include "mbed.h"

#define kXBusBaudrate                   250000          // bps


static sbusFrame_t sbusFrame;

DigitalOut Test(p13);		// local test pin for scoping interrupt routine

XBus::XBus(PinName rx) : serial(NC, rx)
{
    int i;
    servos       = 0;
    sync         = true;
    bytes        = 0;
    good_packets = 0;
    bad_packets  = 0;
    new_values   = false;
    time_since_last_good = 0;
    receiving    = false;
    no_prev_signal = true;
    /* throttle has to get set to min otherwise it will turn on the motor after reset with the transmitter off */
    valuesf[0] = -0.571f;
    for (i=1; i<MAX_XBUS_SERVOS; i++) valuesf[i] = 0;
    for (i=0; i<MAX_XBUS_SERVOS; i++) revert[i]  = 0;
    revert[1] = 1; revert[2] = 1; revert[3] = 1;

    //valuesf[0] = 0;         // XBUS_THRO, collective, 0 vertical speed
    valuesf[1] = 0;         // XBUS_ROLL, 0 side speed
    valuesf[2] = 0;         // XBUS_PITCH, 0 forward speed
    valuesf[3] = 0;         // XBUS_YAW, 0 heading change
    valuesf[4] = -0.571f;   // XBUS_THR_SW, altitude hold
    valuesf[5] = 0.571f;    // XBUS_THR_LV, full throttle
    valuesf[6] = 0.571f;    // XBUS_CTRLMODE_SW, full auto mode
    valuesf[7] = -0.571f;   // XBUS_MODE_SW, speed mode

}

XBus::~XBus(void)
{
//    serial.attach(this, 0, RawSerial::RxIrq);
}

void XBus::ConfigRx()
{
    if(sbus_enabled == 0)
    {
    	serial.baud(kXBusBaudrate);
    	serial.attach(this, &XBus::ProcessByte, RawSerial::RxIrq);
    }
    else
    {
        serial.baud(100000);
        serial.format (8,Serial::Even,2);
        serial.attach(this, &XBus::ProcessSbyte, RawSerial::RxIrq);
    }
}


// Receive ISR callback function for Futaba Sbus
void XBus::ProcessSbyte()
{
    int c = serial.getc();

    if(bytes==0)
    {
    		/* look for sync byte */
        if (c == SBUS_START_OF_FRAME)
        {
            sbusFrame.bytes[bytes++] = (uint8_t)c;
        }
    }
    else
    {
    	/* accumulate packet */
    	sbusFrame.bytes[bytes++] = (uint8_t)c;

    	/* All bytes of frame accumulated */
        if( bytes == SBUS_FRAME_SIZE)
        {
        	bytes = 0;
        	new_values = true;
        }
    }
}

// byte[2] model number
// bytes[3] ???  xbus mode 0-mode B, 1-mode A ???
void XBus::ProcessByte()
{
    /* 2, 1, 9us */
    int c = serial.getc();
 
    if (sync)
    {
        /* look for sync byte */
        if (c == 164)
        {
            buffer[0] = c;
            sync = false;
            bytes = 1;
        }
    }
    else
    {
        if (bytes>=MAX_XBUS_PAKT_SIZE)
        {
            bad_packets++;
            sync = true;
            return;
        }

        /* accumulate packet, then process it */
        buffer[bytes++] = c;

        if (bytes==2)
        {
            /* size too large, start over */
            if ((buffer[1]+3)>MAX_XBUS_PAKT_SIZE)
            {
                bad_packets++;
                sync = true;
            }
        }
        else
        {
            /* complete packet, check CRC and process it */
            if (bytes>=(buffer[1]+3))
            {
                if (CalcCRC8(buffer, buffer[1]+2)!=buffer[buffer[1]+2])
                {
                    bad_packets++;
                }
                else
                {
                    unsigned char b3 = buffer[3] & 0xfe;
                    if (b3!=0 && b3!=0x20)  // 0x80 or 0xa0 indicates xbus receiver is not getting any signal
                    {
                        bad_packets++;
                    }
                    else
                    {
                        int i;
                        unsigned char *s = buffer+4;
                        /* good looking packet, process it */
                        servos = (buffer[1]-4)>>2;
                        for (i=0; i<servos; i++)
                        {
                            if ((s[1]&0x80)==0)
                                valuesu[i] = (s[2]<<8) | s[3];
                            s+=4;
                        }
                        good_packets++;
                        new_values = true;
                        receiving = true;
                    }
                }
                /* start searching for new packet */
                sync = true;
                
            }
        }
    }
}

static const unsigned char s_crc_array[256] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,    0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,    0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,    0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};


unsigned char crc_table(unsigned char  data, unsigned char  crc)
{
    unsigned char  index = (data ^ crc) & 0xff;
    
    crc = s_crc_array[index];
    return crc;
}


unsigned char CalcCRC8(unsigned char *buffer, int  length)
{
    unsigned char  crc = 0;
    
    while (length-- > 0)
        crc = crc_table(*buffer++, crc);
    return crc;
}

// returns 0- no new, 1-new, 2-timeout
//char XBus::NewValues(char sBus_enable, float dT)
char XBus::NewValues(float dT, unsigned char throttle_armed, unsigned char fixed_throttle_mode)
{
    time_since_last_good += dT;
    /* if new packet arrived in the meantime, reset counters and indicate success */
    if (new_values)
    {
    	if(sbus_enabled == 0)
    	{
    		int i;
    		for (i=0; i<servos; i++)
    		{
    			valuesf[i] = (float)(((int)valuesu[i])-32768)*(1.0f/32768.0f);
    			if (revert[i])
    				valuesf[i] = -valuesf[i];
    		}
    	}
    	else
    	{
    		valuesf[0] = -(float(sbusFrame.frame.chan2) * sbusScale + sbusBias) * 0.571f;
    		valuesf[1] =  (float(sbusFrame.frame.chan0) * sbusScale + sbusBias) * 0.571f;
    		valuesf[2] =  (float(sbusFrame.frame.chan1) * sbusScale + sbusBias) * 0.571f;
    		valuesf[3] =  (float(sbusFrame.frame.chan3) * sbusScale + sbusBias) * 0.571f;
    		valuesf[4] =  (float(sbusFrame.frame.chan4) * sbusScale + sbusBias) * 0.571f;
    		valuesf[5] =  (float(sbusFrame.frame.chan5) * sbusScale + sbusBias) * 0.571f;
    		valuesf[6] =  (float(sbusFrame.frame.chan6) * sbusScale + sbusBias) * 0.571f;
    		valuesf[7] =  (float(sbusFrame.frame.chan7) * sbusScale + sbusBias) * 0.571f;

    		// Sbus can report signal loss and fail safe
    		if ((sbusFrame.frame.flags & SBUS_FLAG_SIGNAL_LOSS) || (sbusFrame.frame.flags & SBUS_FLAG_FAILSAFE_ACTIVE))
            {
                receiving = false;
                sbus_flag_errors++;
            }
            else
            {
            	receiving = true;
                good_packets++;
            }
    	}

        new_values = 0;
        time_since_last_good = 0;
        if (no_prev_signal)
        {
        	no_prev_signal = false;
        	return XBUS_NEW_VALUES_1ST;
        }
        else
        	return XBUS_NEW_VALUES;
    }
    /* check if timeout value has been reached */
    else if (time_since_last_good > XBUS_TIMEOUT_VALUE)
    {
        timeouts++;
        // NOTE::SP: This is only done when we are armed and flying.
        // TODO::SP: THIS NEEDS TO ALSO WORK FOR VARIBALE PITCH UAVs (HELIs)
        if (throttle_armed && (fixed_throttle_mode == THROTTLE_FLY))
        {
            valuesf[0] = 0;			// XBUS_THRO, collective, 0 vertical speed
            valuesf[1] = 0;			// XBUS_ROLL, 0 side speed
            valuesf[2] = 0;			// XBUS_PITCH, 0 forward speed
            valuesf[3] = 0;			// XBUS_YAW, 0 heading change
            valuesf[4] = -0.571f;	// XBUS_THR_SW, altitude hold
            valuesf[5] = 0.571f;	// XBUS_THR_LV, full throttle
            valuesf[6] = 0.571f;	// XBUS_CTRLMODE_SW, full auto mode
            valuesf[7] = -0.571f;	// XBUS_MODE_SW, speed mode
        }
        time_since_last_good = 0;
        receiving = false;
        no_prev_signal = true;
        sync = true;    // !!!!!!!!!!!111 not sure if this is a good idea, it might be preventing from re-aquiring xbus
        return XBUS_TIMEOUT;
    }
    else
        return XBUS_NO_NEW_VALUES;
}
