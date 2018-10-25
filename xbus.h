#ifndef _XBUS_H_
#define _XBUS_H_

#include "mbed.h"
#include "structures.h"

#define MAX_XBUS_SERVOS         16
#define MAX_XBUS_PAKT_SIZE  (4+MAX_XBUS_SERVOS*4+3)

#define XBUS_NO_NEW_VALUES  0
#define XBUS_NEW_VALUES     1
#define XBUS_TIMEOUT        2
#define XBUS_NEW_VALUES_1ST 3	// first good data after no data

#define XBUS_TIMEOUT_VALUE  0.100f       // 100ms

// Sbus transmitter
#define SBUS_FRAME_SIZE         25
#define SBUS_MAX_FRAME_TIME     3000
#define SBUS_START_OF_FRAME     (0x0F)
#define SBUS_END_BYTE			(0x00)

#define SBUS_FLAG_CHANNEL_17        (1 << 0)
#define SBUS_FLAG_CHANNEL_18        (1 << 1)
#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)

const float sbusScale = 0.0014880938f;
const float sbusBias = -1.523808f;

extern FlightControlData hfc;

struct sbusFrame_s {
    uint8_t syncByte;
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
    uint8_t flags;
    uint8_t endByte;
} __attribute__ ((__packed__));

typedef union {
    uint8_t bytes[SBUS_FRAME_SIZE];
    struct sbusFrame_s frame;
} sbusFrame_t;



class XBus
{
    public:
        XBus(PinName rx);
       ~XBus(void);

       void SetSbusEnabled(int sbus) { sbus_enabled = sbus; }

        public:
        int             servos;                     // number of servos
        float           valuesf[MAX_XBUS_SERVOS];   // servo values, +/- 1.0;
        unsigned short int valuesu[MAX_XBUS_SERVOS]; // raw values
        unsigned char   revert[MAX_XBUS_SERVOS];    // true will revert channel polarity
        int             good_packets;               // number of received good packets
        int             bad_packets;                // number of received bad packets
        int             timeouts;                // number of timeout conditions
        int             sbus_flag_errors;       // number of error conditions reported from sbus flags
//        char            NewValues(char sBus_enable, float dT);        // returns 0- no new, 1-new, 2-timeout
        char            NewValues(float dT, unsigned char throttle_armed, unsigned char fixed_throttle_mode);        // returns 0- no new, 1-new, 2-timeout
        void			ConfigRx();

        bool            receiving;                  // true if data being received
    
    private:
        RawSerial       serial;
        int             sbus_enabled;
        unsigned char   buffer[MAX_XBUS_PAKT_SIZE];
        char            sync;           // true if searching for a sync byte, false accumulating packet data
        unsigned char   bytes;          // bytes accumulated in buffer
        float           time_since_last_good;   // tracks time since last good packet
        char            new_values;                 // set everytime a good packet is processed
        char			no_prev_signal;
        uint32_t 		sbusFrameStartAt;
        uint32_t		sbusFrameTime;

        void ProcessByte();
        void ProcessSbyte();
};

uint8_t CalcCRC8(uint8_t * buffer, int length);

#endif
