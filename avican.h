#ifndef _AVICAN_H_
#define _AVICAN_H_

/* Using CAN2.0B standard mode, providing 11 bit identifiers.
 * Avidrone Can node/msg assignment as follows:
 *
 *  - bits[10..8] Node TYPE (0.7 node Types)
 *  - bits[07..4] Node ID   (0.15 node ids per node type)
 *  - bits[03..0] Msg ID    (0..15 message ids per node type)
 *
 *  NOTE: AT PRESENT, ANY CHANGES TO THIS FILE SHOULD ALSO BE MADE TO
 *        SERVO AND GPS NODE. (NEED TO MAKE THIS COMMON....)
 */

#define AVIDRONE_CAN_NODE_TYPE_MASK     0x700
#define AVIDRONE_CAN_NODE_TYPE_SHIFT    8

#define AVIDRONE_CAN_NODE_MASK      0xF0
#define AVIDRONE_CAN_NODE_SHIFT     4

#define AVIDRONE_CAN_MSGID_MASK     0xF
#define AVIDRONE_CAN_MSGID_SHIFT    0

#define AVIDRONE_CAN_ID(__TYPE__,__NODE__,__MSG__) ( ((__TYPE__) << AVIDRONE_CAN_NODE_TYPE_SHIFT) \
                                                   | ((__NODE__) << AVIDRONE_CAN_NODE_SHIFT) \
                                                   | ((__MSG__) & AVIDRONE_CAN_MSGID_MASK) )

#define AVIDRONE_CAN_NODETYPE(__MSG__) (((__MSG__) & AVIDRONE_CAN_NODE_TYPE_MASK) >> AVIDRONE_CAN_NODE_TYPE_SHIFT)
#define AVIDRONE_CAN_NODEID(__MSG__) (((__MSG__) & AVIDRONE_CAN_NODE_MASK) >> AVIDRONE_CAN_NODE_SHIFT)
#define AVIDRONE_CAN_MSGID(__MSG__) ((__MSG__) & AVIDRONE_CAN_MSGID_MASK)

typedef enum {
    AVIDRONE_NODETYPE_NONE  = 0,
    AVIDRONE_FCM_NODETYPE   = 1,
    AVIDRONE_SERVO_NODETYPE = 2,
    AVIDRONE_GPS_NODETYPE   = 3,
    AVIDRONE_PWR_NODETYPE   = 4,
    AVIDRONE_RSVD1_NODETYPE = 5,
    AVIDRONE_RSVD2_NODETYPE = 6,
    AVIDRONE_RSVD3_NODETYPE = 7,
    AVIDRONE_NODETYPE_MAX   = AVIDRONE_PWR_NODETYPE,
} AVIDRONE_CAN_NODE_TYPE;

#define DEFAULT_NODE_ID 1
#define MAX_NODE_ID     15

typedef enum {
    AVIDRONE_MSGID_SERVO_NONE    = 0,
    AVIDRONE_MSGID_SERVO_INFO    = 1,
    AVIDRONE_MSGID_LIDAR         = 2,
    AVIDRONE_MSGID_CASTLE_0      = 3,
    AVIDRONE_MSGID_CASTLE_1      = 4,
    AVIDRONE_MSGID_CASTLE_2      = 5,
    AVIDRONE_MSGID_CASTLE_3      = 6,
    AVIDRONE_MSGID_CASTLE_4      = 7,
    AVIDRONE_MSGID_SERVO_CFG     = 8,
    AVIDRONE_MSGID_SERVO_ACK     = 9,
    AVIDRONE_MSGID_SERVO_LO_CTRL = 10,
    AVIDRONE_MSGID_SERVO_HI_CTRL = 11,
    AVIDRONE_MSGID_SERVO_MAX     = AVIDRONE_MSGID_SERVO_HI_CTRL,
} AVIDRONE_CAN_SERVO_MSG_IDS;

#define MAX_NUM_GPS_MSG 4

typedef enum {
    AVIDRONE_MSGID_GPS_NONE      = 0,
    AVIDRONE_MSGID_GPS_INFO      = 1,
    AVIDRONE_MSGID_GPS_SYNC      = 2,
    AVIDRONE_MSGID_GPS_0         = 3,   /* Info - Id, Sats fix, altitude */
    AVIDRONE_MSGID_GPS_1         = 4,   /* Lat/Lon */
    AVIDRONE_MSGID_GPS_2         = 5,   /* Speed ENU, PDOP */
    AVIDRONE_MSGID_GPS_3         = 6,   /* date/Time */
    AVIDRONE_MSGID_GPS_4         = 7,   /* glitch_flag, glitch_cnt, crc_err_cnt, msg_cnt */
    AVIDRONE_MSGID_GPS_CFG       = 8,
    AVIDRONE_MSGID_GPS_ACK       = 9,
    AVIDRONE_MSGID_COMPASS_XYZ   = 10,
    AVIDRONE_MSGID_GPS_MAX       = AVIDRONE_MSGID_COMPASS_XYZ,
} AVIDRONE_CAN_GPS_MSG_IDS;

typedef enum {
    AVIDRONE_MSGID_PWR_NONE = 0,
    AVIDRONE_MSGID_PWR_MAX  = AVIDRONE_MSGID_PWR_NONE,
} AVIDRONE_CAN_PWR_MSG_IDS;

//#else

#define AVICAN_SERVO_LIDAR  0x585
#define AVICAN_SERVO_GPS_ID 0x581

#define AVICAN_SERVO_ID     0x580
#define AVICAN_SERVO_MASK   0x7f0   // 0x580-0x58f

#define AVICAN_SERVOPRINTF_ID       0x100
#define AVICAN_POWERPRINTF_ID       0x110

#define AVICAN_POWER_SWITCHES   0x480   // ESC=0x5b, Servo=0x9e, Aux12V=0xd6, LED=0xca
#define AVICAN_POWER_GPS        0x481   // 481-484
#define AVICAN_POWER_LIDAR      0x486
#define AVICAN_POWER_VALUES1    0x487   // ADC0-3
#define AVICAN_POWER_VALUES2    0x488   // ADC4-5, dTmS, count

typedef struct
{
    unsigned short int id : 3;
    signed short int value : 13;
} T_Servo;

typedef struct
{
    T_Servo servo[4];
} T_Servos;

typedef union {
  uint8_t     msg[4];
  unsigned int lidarCount;
} T_lidar;
//#endif
#endif

