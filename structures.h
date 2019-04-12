#ifndef _STRUCTURES_H_
#define _STRUCTURES_H_

#include "defines.h"
#include "PID.h"

#define MAX_CONFIG_SIZE  (4 * 1024) // 4KB Max config Size

#define LANDING_SITES		20
#define SPEED2ANGLE_SIZE    56
#define ANGLE2SPEED_SIZE    46
#define V2ENERGY_SIZE       36

#define COMMAND_MAX_CHARS   8

#define uint32  unsigned int
#define uint16  unsigned short int
#define sint16  short int
#define uint8   unsigned char
#define f16     unsigned short int
#define byte    unsigned char

//#define TELEMETRY_CTRL0         0
//#define TELEMETRY_GPS1          1
//#define TELEMETRY_SYSTEM2       2
#define TELEMETRY_DATASTREAM3        3
#define TELEMETRY_PARAMETERS4        4
#define TELEMETRY_COMMANDS5          5
#define TELEMETRY_PLAYLIST           6
#define TELEMETRY_TCPIP              7
#define TELEMETRY_PROFILE_CMD        8
#define TELEMETRY_MSG2GROUND	       9
#define TELEMETRY_LANDINGSITES	    10
#define TELEMETRY_AIRCRAFT_CFG      11
#define TELEMETRY_CTRL              12
#define TELEMETRY_GPS               13
#define TELEMETRY_SYSTEM            14
#define TELEMETRY_CALIBRATE         15
#define TELEMETRY_GND2AIR_HEARTBEAT 16
#define TELEMETRY_LAST_MSG_TYPE     17


#define TELEM_PARAM_WAYPOINT        1

#define TELEM_PARAM_WP_LATITUDE     0
#define TELEM_PARAM_WP_LONGITUDE    1
#define TELEM_PARAM_WP_ALTITUDE     2
#define TELEM_PARAM_WP_MAX_H_SPEED  3
#define TELEM_PARAM_WP_MAX_H_ACC    4
#define TELEM_PARAM_WP_MAX_V_SPEED  5
#define TELEM_PARAM_WP_MAX_V_ACC    6
#define TELEM_PARAM_WP_TYPE         8
#define TELEM_PARAM_WP_RETIRE       9
#define TELEM_PARAM_WP_YAWSPEEDRATE 10
#define TELEM_PARAM_WP_GTWP_RADIUS  11
#define TELEM_PARAM_WP_GTWP_SPEED   12
#define TELEM_PARAM_WP_FTWP_SR_FACTOR 	13
#define TELEM_PARAM_WP_LOW_SPEED_LMT 	14
#define TELEM_PARAM_WP_MIN_V_SPEED  	15
#define TELEM_PARAM_WP_ALTITUDE_BASE	16

#define TELEM_PARAM_STICK           0

#define TELEM_PARAM_STICK_PR_RATE   0
#define TELEM_PARAM_STICK_PR_ANGLE  1
#define TELEM_PARAM_STICK_HSPEED    2
#define TELEM_PARAM_STICK_VSPEED    3
#define TELEM_PARAM_STICK_Y_RATE    4
#define TELEM_PARAM_STICK_PR_ROTATE 5
#define TELEM_PARAM_STICK_H_ACC     6

#define TELEM_PARAM_JOYSTICK        2

#define TELEM_PARAM_JOY_PITCH       0
#define TELEM_PARAM_JOY_ROLL        1
#define TELEM_PARAM_JOY_YAW         2
#define TELEM_PARAM_JOY_COLLECTIVE  3
#define TELEM_PARAM_JOY_DELTA_ALT   4
#define TELEM_PARAM_JOY_THROTTLE    5

#define TELEM_PARAM_CONTROL         3

#define TELEM_PARAM_CTRL_DECEL          0
#define TELEM_PARAM_CTRL_HEADING_REL    1
#define TELEM_PARAM_CTRL_HEADING_ABS    2
#define TELEM_PARAM_CTRL_LIDAR_ALT      3
#define TELEM_PARAM_CTRL_WIND_COMP      4
#define TELEM_PARAM_CTRL_PATH_NAVIG     5
#define TELEM_PARAM_CTRL_ANGLE_COLL_MIX 6
#define TELEM_PARAM_CTRL_VSPEED_PID_D	7
#define TELEM_PARAM_CTRL_THR_OFFSET     8
#define TELEM_PARAM_CTRL_CRUISE_LIMIT   9
#define TELEM_PARAM_CTRL_YAW_ACC       10
#define TELEM_PARAM_CTRL_NOSE2WP       11
#define TELEM_PARAM_CTRL_WINDLIMIT     12
#define TELEM_PARAM_CTRL_BAT_CAPACITY  13
#define TELEM_PARAM_CTRL_WINDTAB_SCALE 14

#define TELEM_PARAM_PID_TUNE        4

#define TELEM_PARAM_PID_PITCH_RATE_P    0
#define TELEM_PARAM_PID_PITCH_RATE_I    1
#define TELEM_PARAM_PID_PITCH_RATE_D    2
#define TELEM_PARAM_PID_ROLL_RATE_P     3
#define TELEM_PARAM_PID_ROLL_RATE_I     4
#define TELEM_PARAM_PID_ROLL_RATE_D     5
#define TELEM_PARAM_PID_YAW_RATE_P      6
#define TELEM_PARAM_PID_YAW_RATE_I      7
#define TELEM_PARAM_PID_YAW_RATE_D      8
#define TELEM_PARAM_PID_PITCH_ANGLE_P   9
#define TELEM_PARAM_PID_PITCH_ANGLE_I   10
#define TELEM_PARAM_PID_PITCH_ANGLE_D   11
#define TELEM_PARAM_PID_ROLL_ANGLE_P    12
#define TELEM_PARAM_PID_ROLL_ANGLE_I    13
#define TELEM_PARAM_PID_ROLL_ANGLE_D    14
#define TELEM_PARAM_PID_YAW_ANGLE_P     15
#define TELEM_PARAM_PID_YAW_ANGLE_D     16
#define TELEM_PARAM_PID_PITCH_SPEED_P   17
#define TELEM_PARAM_PID_PITCH_SPEED_I   18
#define TELEM_PARAM_PID_PITCH_SPEED_D   19
#define TELEM_PARAM_PID_ROLL_SPEED_P    20
#define TELEM_PARAM_PID_ROLL_SPEED_I    21
#define TELEM_PARAM_PID_ROLL_SPEED_D    22
#define TELEM_PARAM_PID_COL_VSPEED_P    23
#define TELEM_PARAM_PID_COL_VSPEED_I    24
#define TELEM_PARAM_PID_COL_VSPEED_D    25
#define TELEM_PARAM_PID_COL_ALT_P       26
#define TELEM_PARAM_PID_COL_ALT_D       27
#define TELEM_PARAM_PID_DIST2T_P        28
#define TELEM_PARAM_PID_DIST2T_D        29
#define TELEM_PARAM_PID_DIST2P_P        30
#define TELEM_PARAM_PID_DIST2P_D        31
#define TELEM_PARAM_PID_PITCH_CRUISE_P  32
#define TELEM_PARAM_PID_PITCH_CRUISE_D  33
#define TELEM_PARAM_PID_IMU_P           34
#define TELEM_PARAM_PID_IMU_I           35
#define TELEM_PARAM_PID_IMU_D           36

///////////////////////////////////////
#define TELEM_PARAM_AIRFRAME 5
#define TELEM_PARAM_AIRFRAME_RESET_LAST 8

enum TELEM_PARAMS_AIRFRAME {
  TELEM_PARAM_AIRFRAME_ACCEL_LPF = 0,
  TELEM_PARAM_AIRFRAME_GYRO_P_LPF,
  TELEM_PARAM_AIRFRAME_GYRO_R_LPF,
  TELEM_PARAM_AIRFRAME_GYRO_Y_LPF,
  TELEM_PARAM_AIRFRAME_ACC_INTEGRAL_X_GAINS,
  TELEM_PARAM_AIRFRAME_ACC_INTEGRAL_Y_GAINS,
  TELEM_PARAM_AIRFRAME_ACC_INTEGRAL_Z_GAINS,
  TELEM_PARAM_AIRFRAME_ALTITUDE_BARO_GPS_BLEND,
  TELEM_PARAM_AIRFRAME_POS_GPS_IMU_BLEND_GLITCH,
  TELEM_PARAM_AIRFRAME_POS_GPS_IMU_BLEND_REG,
  TELEM_PARAM_AIRFRAME_BARO_LPF,
  TELEM_PARAM_AIRFRAME_BARO_VSPEED_LPF,
  TELEM_PARAM_AIRFRAME_BARO_VSPEED_WEIGHT,
  TELEM_PARAM_AIRFRAME_BARO_ALTITUDE_WEIGHT,

  TELEM_PARAM_AIRFRAME_GPS_VSPEED_WEIGHT,
  TELEM_PARAM_AIRFRAME_VSPEED_MODE,
  TELEM_PARAM_AIRFRAME_TURN_ACCELERATIONS_MAX_SIDE,
  TELEM_PARAM_AIRFRAME_TURN_ACCELERATIONS_MAX_SPEED,
  TELEM_PARAM_AIRFRAME_TURN_ACCELERATIONS_LOW_SPEED,

  TELEM_PARAM_AIRFRAME_JOYSTICK_MAX_SPEED,
};
/////////////////////////////////////////

#define TELEM_PARAM_CALIBRATE   6
#define TELEM_PARAM_CAL_MAX_MIN 0

#define TELEM_PARAM_CALIBRATE_DONE 7
#define TELEM_PARAM_CAL_DONE_OFS    0
#define TELEM_PARAM_CAL_DONE_GAINS  1
#define TELEM_PARAM_CAL_DONE_MAX    2
#define TELEM_PARAM_CAL_DONE_MIN    3

/* commands */
#define TELEM_CMD_NONE              255
#define TELEM_CMD_ARMING            0   // 0-disarm, 1-arm
#define TELEM_CMD_SET_HOME          1   // sets current position as home
#define TELEM_CMD_CALIBRATE         2
#define TELEM_CMD_JOYSTICK          3   // 0-disable, 1-enable
#define TELEM_CMD_GOTO_HOME         4
#define TELEM_CMD_PLAYLIST_CONTROL  5
#define TELEM_CMD_TAKEOFF           6   // 0-arm, 1-execute (bytes 1-4 float ver take off speed)
#define TELEM_CMD_LAND              7   // byte 0 - mode unused, (bytes 1-4 float ver landing speed)
#define TELEM_CMD_POS_HOLD          8   // inserts alt+pos waypoint at current pos
#define TELEM_CMD_GPS_NEXT			9	// forces GPS to select the next channel
#define TELEM_CMD_MSG				10	// message from ground, sub_command is the message id/value
#define TELEM_CMD_KILLSWITCH        11  // kill power and auto-rotate
#define TELEM_CMD_TOGGLE_PWR        12  // toggle selected power switch
#define TELEM_CMD_RESET_IMU_ALT     13  // setting IMU altitude to match GPS
#define TELEM_CMD_PREFLIGHT_CHECK   14  // executes pre-flight check
#define TELEM_CMD_RESET             15  // saves PIDs if modified and reboots mbed

/* playlist commands */
#define PLAYLIST_PLAY         0   // play from beggining
#define PLAYLIST_JUMP         1   // play from item x
#define PLAYLIST_PAUSE        2   // pause playlist, position/altitude hold
#define PLAYLIST_RESUME       3   // resume after pause
#define PLAYLIST_STOP         4   // stop, position/altitude hold, playlist needs to be stopped to allow upload

#define PLAYLIST_NONE         0
#define PLAYLIST_STOPPED      1
#define PLAYLIST_PLAYING      2
#define PLAYLIST_PAUSED       3

#define PL_ITEM_END           0   // end of the playlist
#define PL_ITEM_WP            1   // waypoint
#define PL_ITEM_PARAM         2   // parameter
#define PL_ITEM_GOTO          3   // (conditional) jump
#define PL_ITEM_DELAY         4   // fixed delay
#define PL_ITEM_HOLD          5   // conditional hold

#define CMD_DISARM              0
#define CMD_ARM                 1

#define CMD_JOYSTICK_DISABLE    0
#define CMD_JOYSTICK_ENABLE     1

#define TAKEOFF_ARM             0

#define LANDING_CURRENT			0
#define LANDING_WAYPOINT		1
#define LANDING_SITE			2

#define KILLSWITCH_AUTOROTATE   13
#define RESET_SUBID             0xe2

#define TOGGLE_PWR_AUX12V       1
#define TOGGLE_PWR_SERVO        2

#define IMURESET_GPS            0
#define IMURESET_ALTITUDE       1

#define CALIBRATE_STOP          0
#define CALIBRATE_IMU           1
#define CALIBRATE_COMPASS       2

#define ARMED_TIMEOUT           60 //seconds, 1 minute to arm and get to flying

/* control source, transition from RCradio to anything saves stick values */
#define CTRL_SOURCE_RCRADIO     0
#define CTRL_SOURCE_JOYSTICK    1   // 2 is skipped to keep compatibility with Release v.4.01 AGS
#define CTRL_SOURCE_AUTOPILOT   3   // everything internally driven
#define CTRL_SOURCE_AFSI        4   // use serial interface

/* take off states */
#define FM_TAKEOFF_NONE         0
#define FM_TAKEOFF_WAIT         1   // wait for user input
#define FM_TAKEOFF_AUTO_SPOOL   2
#define FM_TAKEOFF_ARM          3   // angle mode, spool up
#define FM_TAKEOFF_START        4   // collective to vspeed
#define FM_TAKEOFF_LEVEL        5   // collective to vspeed
#define FM_TAKEOFF_SPEED        6   // speed ground, 25% vspeed threshold
#define FM_TAKEOFF_ALTITUDE     7   // set target altitude once above 4m
#define FM_TAKEOFF_HOLD         8   // position+alt hold at +5m, 50% vspeed threshold or +1m altitude
#define FM_TAKEOFF_COMPLETE     9   // within 0.2m

/* landing states */
#define FM_LANDING_NONE         0
#define FM_LANDING_STOP         1
#define FM_LANDING_WAYPOINT		  2
#define FM_LANDING_HOLD         3
#define FM_LANDING_HIGH_ALT     4
#define FM_LANDING_LOW_ALT      5
#define FM_LANDING_FINAL_APP	  6
#define FM_LANDING_TOUCHDOWN    7
#define FM_LANDING_TIMEOUT      8
#define FM_LANDING_LANDED       9

#define MSG2GROUND_RESEND_COUNT			    10  // how many times the same message is send, no TCPIP yet
#define MSG2GROUND_ARMED_FOR_TAKEOFF    1   // system has to be armed before takeoff
#define MSG2GROUND_XBUS_FOR_TAKEOFF     2   // xbus has to be active before takeoff
#define MSG2GROUND_SPOOLUP              3   // prepare RC radio and spool up
#define MSG2GROUND_TAKEOFF_TIMEOUT      4   // takeoff has timed out
#define MSG2GROUND_ALLOW_SPOOLUP        5   // asking to allow a spool up
#define MSG2GROUND_ALLOW_TAKEOFF        6   // asking to allow takeoff
#define MSG2GROUND_ALLOW_LANDING        7   // asking to allow landing/extent hold time
#define MSG2GROUND_ARMING_THROTTLE      8   // throttle level needs to be low for arming
#define MSG2GROUND_ARMING_MODE          9   // cannot armed in manual mode
#define MSG2GROUND_PFCHECK_ALL_GOOD     10  // pre-flight check passed
#define MSG2GROUND_PFCHECK_ACC_IMU      11  // IMU and ACC horizon estimation do not match
#define MSG2GROUND_PFCHECK_GYRO         12  // gyro exceed limits
#define MSG2GROUND_PFCHECK_ACC          13  // accelerometer exceed limits
#define MSG2GROUND_PFCHECK_BARO         14  // barometer exceed limits
#define MSG2GROUND_PFCHECK_COMPASS      15  // compass exceed limits
#define MSG2GROUND_PFCHECK_BATTERY      16  // battery level/voltage too low
#define MSG2GROUND_PFCHECK_ANGLE        17  // level limit exceeded
#define MSG2GROUND_PFCHECK_CAN_SERVO    18  // failed to send msg to servo module
#define MSG2GROUND_PFCHECK_CAN_POWER    19  // failed to send msg to power module
#define MSG2GROUND_PFCHECK_IMU_GPS_ALT  20  // IMU and GPS altitude have to within 2m
#define MSG2GROUND_PFCHECK_GPS_NOFIX    21  // not all GPS units have a lock
#define MSG2GROUND_PFCHECK_GPS_PDOP     22  // not all GPS units have PDOP<2
#define MSG2GROUND_PFCHECK_GPS_ALTITUDE 23  // altitude has to be legal
#define MSG2GROUND_PFCHECK_GPS_POSITION 24  // all GPS position have to be within 5.5m
#define MSG2GROUND_PFCHECK_GPS_NOSIGNAL 25  // All GPS units have to be providing data
#define MSG2GROUND_PFCHECK_IMUACC_HORIZ 30  // IMU and ACC horizon estimation have to be close to each other
#define MSG2GROUND_PFCHECK_IMUCOMP_HEAD 31  // IMU and compass heading estimation have to be close to each other
#define MSG2GROUND_PFCHECK_LIDAR        32  // Lidar reports invalid value.
#define MSG2GROUND_THROTTLE_LEVER_LOW   34  // Throttle Level needs is LOW
#define MSG2GROUND_THROTTLE_LEVER_HIGH  35  // Throttle Level needs is High
#define MSG2GROUND_LIDAR_NOGROUND       40  // lidar does not see ground before landing
#define MSG2GROUND_TAKEOFF              41  // tell AGS that takeoff has begun


#define CMD_MSG_TAKEOFF_OK        1
#define CMD_MSG_TAKEOFF_ABORT     2
#define CMD_MSG_TAKEOFF_ALLOWED   3

#define CMD_MSG_LANDING_GO        4
#define CMD_MSG_LANDING_ADD1MIN   5

#define THROTTLE_IDLE			1
#define THROTTLE_DEAD			2
#define THROTTLE_RAMP			3
#define THROTTLE_FLY			4

#define PITCH_COMP_LIMIT       (180/9) //-90 to 90 degrees, in 20 groups of 9 degrees, must be a whole number
#define ROLL_COMP_LIMIT        (360/9) //-180 to 180 degrees in 40 groups of 9 degrees, must be a whole number
#define NUM_ANGLE_POINTS       110

#define NO_COMP_CALIBRATE       0
#define COMP_CALIBRATING        1
#define COMP_CALIBRATE_DONE     2

#define MAX_NUM_LIDARS            5
#define LIDAR_TIMEOUT        0.100f  //0.100 seconds = 100 milli-seconds
#define LIDAR_HISTORY_SIZE       10
#define INVALID_LIDAR_DATA     9999
#define MAX_LIDAR_PULSE       40000   /* 40000 us @ 1us/1mm = 40m max range */
#define MIN_LIDAR_PULSE           0   /* 40000 us @ 1us/1mm = 40m max range */

typedef struct /* size 28bytes */
{
    uint32 Vmain  : 12;     // 0-64V    *64
    uint32 Vaux   : 10;     // 0-16V    *64
    uint32 Vservo : 10;     // 0-16V    *64

    uint32 Vesc : 12;       // 0-64V    *64
    uint32 Iesc : 13;       // 0-256A   *32
    uint32 Iaux : 7;        // 0-8A *16

    uint16 battery_level : 7;   // in %, 120=100%   *120
    uint16 capacity_used : 9;   // 0-128Ah      *4
} T_PowerMsg;

typedef struct
{
    unsigned char       start_code;     // 0x47
    unsigned char       len;            // payload size after header-1
    unsigned char       crc8;           // CRC8 of the header (1st 4 bytes), CRC8 is init to 0x3d
    unsigned char       type;
    unsigned int        crc;            // CRC of the entire packet including the header, where CRC is set to 0x12345678
} T_TelemUpHdr;

/* size of 92bytes + hdr */
/* Telemetry - Control 0   sent with background priority as often as the link allows (94 bytes, ~60Hz) */
typedef struct
{
    T_TelemUpHdr    hdr;

    /* payload */
    uint32  time;
    float   altitude;
    float   baro_altitude;
    float   ctrl_altitude;
    uint16  ctrl_modes;             // p/r/y/c/t   3-bits each
    f16     gyro_lp[3];      // PRY
    f16     acc_lp[3];       // RFU
    f16     compass[3];      // ????????
    f16     compass_heading;
    f16     orient[3];       // PRY
    f16     speedGroundENU[3]; // ENU
    f16     baro_vspeed;
    f16     ctrl_manual[5];  // PRYCT
    f16     ctrl_rate[3];    // PRY
    f16     ctrl_angle[3];   // PRY
    f16     ctrl_speed[3];   // RFU
    int     latitude;        // latitude  from IMU * 10M
    int     longitude;       // longitude from IMU * 10M
    f16     lidar_alt;
    int     controlStatus;  // bit mask denoting the last control command made.
    int     lidar_online_mask;  //lidar_online_mask identifies which lidars are reporting, bit 0 = lidar node 0, bit # = lidar node_id #
} T_Telem_Ctrl0;

/* Telemetry - GPS 1       sent the GPS update rate, typically 5-10Hz, 42bytes */
typedef struct
{
    /* header */
    T_TelemUpHdr    hdr;

    /* payload */
    uint32  time;
    int     gps_pos[3];             // Lat*1M/lon*1M/alt*100, raw GPS values
    f16     gps_speed;
    f16     gps_heading;
    f16     gps_to_home[3];         // distance/heading/altitude difference
    f16     gps_to_waypoint[3];     // distance/heading/altitude difference
    f16     gps_speed_ENU[3];       // east/north/up speeds m/s
} T_Telem_GPS1;

/* Telemetry - System 2   sent every second with background priority (82 bytes) */
typedef struct
{
    T_TelemUpHdr    hdr;

    /* payload */
    uint32  time;
    float   baro_pressure;
    uint32  gps_time;
    uint32  gps_date;
    uint16  precess_period_lp;      // in us
    uint16  precess_period_max;     // in us
    uint16  telem_good_messages;
    uint16  telem_crc_errors;
    uint16  telem_start_code_searches;
	uint16	flight_time_left;	// in sec
    T_PowerMsg power;
    f16     gyro_temperature;
    f16     baro_temperature;
    f16     gps_hdop;
    byte    gps_sats_curr  : 4;
    byte    gps_sats_other : 4;
    byte    gps_fix_curr   : 2;
    byte    gps_fix_other  : 2;
    byte    gps_current    : 2;
    byte    motor_state;
    byte    cpu_utilization;
    byte    control_status;   // bit 0   - 1 means xbus receiving,
                              // bit 1   - 0 means xbus in control, 1 ground station in control,
                              // bit 2   - throttle disarmed (1)
                              // bit 3   - joystick control
                              // bit 4-5 - playlist status (0-stopped, 1-playing, 2-paused)
                              // bit 6   - rc_ctrl_request
                              // bit 7   - eng_super_user
    //-------------------------------- size 56
    uint16  playlist_items;
    uint16  playlist_position;
    //-------------------------------- size 60
    uint16  gps_errors;         // number of detected GPS errors
    f16     wind_speed;         // wind speed in m/s
    f16     wind_course;        // wind course in deg, cw from north
    //-------------------------------- size 64
    f16     RPM;
    f16     power_lp;
    //-------------------------------- size 68
    f16     gyro_offsets[3];    // P/R/Y, deg/s
    f16     esc_temp;           // ESC temperature in degC
    byte    num_landing_sites;
    byte    ctrl_source;
    //-------------------------------- size 77
} T_Telem_System2;

/* Telemetry - DataStream 3       sent at the full rate (1200Hz) divided by the number of elements,
** the message can be shorter than having 120 data elements */
typedef struct
{
    /* header */
    unsigned char       start_code;     // 0x47
    unsigned char       len;            // payload size after header-1
    unsigned char       crc;            // CRC of the entire packet including the header, where CRC is set to 0x39, same as for xbus
    unsigned char       type;

    /* payload */
    uint32  time;
    uint8   stream_info;            // bits 0-2 number of data elements, bit 3 profilling, streaming otherwise; bit 4 in profiling mode, indicates the last message
    uint8   data_type[7];           // identifies the data types of individual elements
    f16     data[120];
} T_Telem_DataStream3;

typedef struct
{
    byte        param;
    byte        sub_param;
    byte        data[4];
} T_ParamStruct;

/* Telemetry - Parameters to heli 4 */
typedef struct
{
    /* header */
    T_TelemUpHdr        hdr;
    
    /* payload */
    T_ParamStruct       data[42];       // up to 42 parameters per message
} T_Telem_Params4;

/* Telemetry - Commands to heli 5 */
typedef struct
{
    /* header */
    T_TelemUpHdr        hdr;

    /* payload */
    byte                command;
    byte                sub_cmd;
    byte                data[240];
} T_Telem_Commands5;

typedef struct
{
    byte        type;
    byte        data[3];
    union
    {
      int       i;
      float     f;
    } value1;
    union
    {
      int       i;
      float     f;
    } value2;
} T_PlaylistItem;

/* Telemetry - Playlist upload to heli 6 */
typedef struct
{
    /* header */
    T_TelemUpHdr        hdr;

    /* payload */
    byte                pages;  // total pages in this playlist
    byte                page;   // current page
    byte                lines;  // items in this message/page
    byte                unused;
    T_PlaylistItem      items[21];  // playlist items
} T_Telem_Playlist6;

/* Telemetry - Profiling/Streaming command 8 */
typedef struct
{
    /* header */
    T_TelemUpHdr        hdr;

    /* payload */
    /* streaming command */
    byte                stream_enable;         // enables/disables streaming
    byte                stream_channels;       // selects the number of data streams
    byte                stream_data_types[7];
    byte                stream_period;         // main loop periods in between data sampling, 0 means every iteration
    
    /* profiling */
    byte                profile_enable;            // enables profiling to be performed
    byte                profile_ctrl_variable;     // selects the controlled variable for auto-profiling (P, R, Y, C, T)
    uint16              profile_lag;               // initial lag (T1) in ms
    uint16              profile_period;            // test period (Tt) in ms
    sint16              profile_delta;             // control value (dC) in % of stick - 1000 means 100%
    byte                profile_ctrl_level;        // selects the level of control for auto-profiling (raw/rate/angle/speed)
} T_Telem_Profile8;

/* Telemetry - TCPIP 7   it is sent back to the ground station in a response
** to an upstream packet (not necessarly all types of packets)  */
typedef struct
{
    /* header */
    unsigned char       start_code;     // 0x47
    unsigned char       len;            // payload size after header-1
    unsigned char       crc;            // CRC of the entire packet including the header, where CRC is set to 0x39, same as for xbus
    unsigned char       type;

    /* payload */
    byte    org_type;
    byte    org_len;
    uint16  user1;  // custom field 1
    uint16  user2;  // custom field 2
} T_Telem_TCPIP7;

/* Telemetry - MSG2GROUND 9   it is sent back to the ground station to inform it about something */
typedef struct
{
    /* header */
    unsigned char       start_code;     // 0x47
    unsigned char       len;            // payload size after header-1
    unsigned char       crc;            // CRC of the entire packet including the header, where CRC is set to 0x39, same as for xbus
    unsigned char       type;

    /* payload */
    byte    msg_id;
    byte    user_data;
} T_Telem_Msg2Ground;

/* Telemetry - Calibrating messages */
typedef struct
{
    byte        param;
    byte        sub_param;
    float       data[3];
} T_CalibrateParams;
typedef struct
{
    /* header */
    T_TelemUpHdr        hdr;

    /* payload */
    T_CalibrateParams       data[4];       // up to 4 parameters per message
} T_Telem_Calibrate;


typedef struct
{
	float	lat;
	float	lon;
	float	altitude;		// above sea level altitude of the site
	float	above_ground;	// minimum above ground approach altitude
} T_LandingSite;

/* Telemetry - TELEMETRY_LANDINGSITES 10   list of emergency landing sites */
typedef struct
{
    /* header */
    T_TelemUpHdr        hdr;

    /* payload */
    byte                pages;  // total pages in this playlist
    byte                page;   // current page
    byte                lines;  // items in this message/page
    byte                unused;
    T_LandingSite		items[15];  // landing sites
} T_Telem_LandingSites;

/* Aircraft configuration, sent every 10seconds */
typedef struct
{
    /* header */
    T_TelemUpHdr        hdr;

    /* payload */
    f16         throttle_min;       // throttle value at 0%
    f16         throttle_max;       // throttle value at 100%
    uint16      power_hover;        // typical power used at hover
    f16         collective_flat;    // collective value for 0 pitch
    f16         collective_hover;   // collective at hover
    f16         collective_max;     // maximum value
    uint16      rpm_hover;          // typical RPM at hover
    f16         return_speed;       // m/s return to home or landing
    byte        bat_cells;          // number of lipo cells
    byte        unused[3];

    uint32      fcm_serialnum_0;       // FCM Chip ID of form [3:2:1:0]
    uint32      fcm_serialnum_1;
    uint32      fcm_serialnum_2;
    uint32      fcm_serialnum_3;
    uint32      fcm_version_num;    // Version is Major:Minor:Build, each item is 8bits

    uint32      servo0_version_num; // Version is Major:Minor:Build, each item is 8bits
    uint32      servo0_serialnum_0;
    uint32      servo0_serialnum_1;
    uint32      servo0_serialnum_2;

    uint32      servo1_version_num; // Version is Major:Minor:Build, each item is 8bits
    uint32      servo1_serialnum_0;
    uint32      servo1_serialnum_1;
    uint32      servo1_serialnum_2;

    uint32      gps_version_num;    // Version is Major:Minor:Build, each item is 8bits
    uint32      gps_serialnum_0;
    uint32      gps_serialnum_1;
    uint32      gps_serialnum_2;

    uint32      pwr_version_num;    // Version is Major:Minor:Build, each item is 8bits
    uint32      pwr_serialnum_0;
    uint32      pwr_serialnum_1;
    uint32      pwr_serialnum_2;

    uint32      imu_serial_num;    // Version is Major:Minor:Build, each item is 8bits

} T_AircraftConfig;

typedef struct
{
    byte                command;
    byte                sub_cmd;
    byte                data[16];
} T_Command;

typedef struct
{
    float w1P1, w2P1, w1P2, w2P2;
    const float *coeffs;
} T_LP4;

typedef struct
{
    float pid_CollAlt_COmax;
    float pid_CollAlt_COmin;
    float pid_CollAlt_acc;
    float pid_Dist2T_COmax;
    float pid_Dist2T_acc;
    float acc_dyn_turns;
    float altitude;
    int telem_ctrl_period;
    unsigned char ctrl_source;
    int waypoint_type;
    unsigned char control_mode[5];
} T_State;

typedef struct
{
    float   dT;
	float 	Vmain;
	float	Vaux;
	float	Vservo;
	float	Vesc;
	float	Iesc;
	float	Iaux;
	float   Vslope;
    float   Voffset;
    float   Islope;
	float   Ioffset;
	float	battery_level;	// 0-100%, energy based, not capacity
	float	capacity_used;	// AmpSeconds
	float   capacity_total; // rated As
	float   energy_total;   // rated Ws of the battery
	float   energy_curr;    // current Ws
	float   power_curr;     // current Watts
	float   power_lp;
	int     flight_time_left;   // in seconds
	char    initialized;    // skips low-pass filters for the first time
    /* power control variables */
    char	power_esc;
    char	power_servo;
    char	power_aux12v;
    char	power_armed_led;
} T_Power;

typedef struct
{
    char    can_servo_tx_failed;
    char    can_power_tx_failed;
    uint16  can_servo_tx_errors;
    uint16  can_power_tx_errors;
} T_Stats;

typedef struct {
	unsigned int checksum;
	int total_size;	// Total Number of bytes (including Header)
	int version;
	char date_time[20];
} ConfigurationDataHeader;

// NOTE::SP: DO NOT!!!! CHANGE THE ORDER OF THIS ARRAY WITHOUT UPDATING
// CONFIGURATION TOOL ALSO
typedef struct ConfigurationData {

    // User Config Data
	ConfigurationDataHeader header;
    int num_servo_nodes;
    int num_gps_nodes;
    int num_power_nodes;
    int compass_type;
    int compass_selection;
    int canbus_freq_high;
    int LidarFromServo;
    int LidarFromPowerNode;

    int  autoReset;
    int  can_servo;
    int  fcm_servo;
    int  power_node;
    int  rpm_sensor;
    int  fcm_linklive_enable;
    int  gps_vspeed;
    int  ground_sensor;
    int  imu_internal;

    int ccpm_type;
    int SbusEnable;
    int transmitter_protocol;
    int RCmodeSwitchOfs;
    int AllowArmInManual;

    int     throttle_ctrl;
    float   throttle_values[2];
    float   throttle_gain;
    float   throttle_multi_min;

    float CollZeroAngle;
    float CollAngleAutoRotate;
    float CollThrAutoLevel;
    float RollPitchAngle;

    float PRstickRate;
    float PRstickAngle;
    float YawStickRate;
    float StickVspeed;
    float StickHspeed;
    float StickHaccel;
    float AngleCollMixing;
    float Stick100range;
    float stick_deadband[4];

    float HspeedMax;
    float HspeedAcc;
    float VspeedMax;
    float VspeedMin;
    float VspeedAcc;

    int landing_timeout;
    float landing_vspeed;
    float landing_appr_speed;
    int   lidar_offset;
//    int   lidar_avgs;

    int   battery_cells;
    int   battery_capacity;
    int   power_typical;
    int   rpm_typical;
    float power_coeffs[6];

    // Engineering Config Data
    unsigned char default_ctrl_source;
    float default_dyn_yaw_rate;
    float default_acc_dyn_turns;
    float default_home_position[3];
    int orient_reset_counter;

    int sensor_mode;
    int servo_raw;

    int pwm_period;
    int telem_baudrate;
    int telem_min_ctrl_period;

    float dyn_yaw_rate_max;
    float low_speed_limit;

    float VspeedDownCurve[2];
    float LidarHVcurve[4];
    float TurnAccParams[3];
    float collective_man_speed;
    float takeoff_angle_rate;
    float landing_vspeed_acc;
    float cruise_speed_limit;
    float landing_wind_threshold;
    float joystick_max_speed;

    int  ctrl_mode_inhibit[5];
    float control_gains[5];
    int  servo_revert[6];

    int YawModeMin;
    int YawModeMax;
    unsigned char ManualLidarAltitude;

    float AilRange;
    float EleRange;
    float RudRange;
    float TorqCompMult;
    float CollRange;
    float CcpmMixer;
    int   ModelSelect;
    float RearRpmTrim;

    unsigned char  wind_compensation;
    unsigned char  path_navigation;
    unsigned char  nose_to_WP;

    float pitchrate_pid_params[6];
    float rollrate_pid_params[6];
    float pitchangle_pid_params[6];
    float rollangle_pid_params[6];
    float yawrate_pid_params[6];
    float yawangle_pid_params[5];
    float collvspeed_pid_params[6];
    float pitchspeed_pid_params[6];
    float rollspeed_pid_params[6];
    float collalt_pid_params[5];
    float dist2T_pid_params[5];
    float dist2P_pid_params[5];
    float pitchCruise_pid_params[5];
    float imu_pid_params[6];
    float imu_yaw_pid_params[6];

    float servo_speed[2];
    int  acc_lp_freq;
    int  gyro_lp_freq[3];
    float AccIntegGains[3];

    int  baro_enable;
    int  baro_lp_freq;
    int  baro_vspeed_lp_freq;

    float BaroVspeedWeight;
    float GPSVspeedWeight;
    float BaroAltitudeWeight;
    int   heading_avgs;

    float IMUaccGyroBlend;
    float AltitudeBaroGPSblend_final;
    float Pos_GPS_IMU_BlendGlitch;
    float Pos_GPS_IMU_BlendReg;

    float GTWP_retire_radius;
    float GTWP_retire_speed;
    float FTWP_retire_sr_factor;

    unsigned char  acc_orient[6];
    unsigned char  gyro_orient[6];
    unsigned char  comp_orient[6];
    unsigned char  fcm_orient[6];

    int gyro_first_order;
    int acc_first_order;
    float comp_declination_offset;

    float WindTableScale;
    float WindSpeedLUT[46];

    float gear_ratio;
    int  motor_poles;

    float V2Energy[36];

    int force_gyro_acc_defaults;
    float dcpFront;
    float dcpRear;
    float swashTiltRear;

    float elevator_gain;
    float dcp_gain;
    float throttle_offset;

    int disable_pre_flight;

    float gps_speed_heading_threshold;
    float heading_offset_threshold;
    float heading_offset;
    float yaw_heading_threshold;

    float voltage_slope_percent_mod;
    float current_slope_percent_mod;
    float current_offset;

    float max_params_hspeed;  // Max allowable horizontal speed set by params
    float max_params_vspeed;  // Max allowable vertical speed set by params

    int AfsiEnabled;

    float takeoff_height; // default takeoff height.
    bool eng_super_user_enable;

//} __attribute__((packed)) ConfigData;
} ConfigData;

// This data is initialized from the config, but may be updated at Runtime, through Telemetry.
// Keep it here to easily see which config data is writable.
typedef struct
{
    float HspeedMax;
    float HspeedAcc;
    float VspeedMax;
    float VspeedMin;
    float VspeedAcc;
    float Speed2AngleLUT[56];
    float GTWP_retire_radius;
    float GTWP_retire_speed;
    float FTWP_retire_sr_factor;
    float low_speed_limit;
    float PRstickRate;
    float PRstickAngle;
    float YawStickRate;
    float StickVspeed;
    float StickHspeed;
    float StickHaccel;
    float RollPitchAngle;
    unsigned char wind_compensation;
    unsigned char path_navigation;
    unsigned char ManualLidarAltitude;
    float AngleCollMixing;
    float cruise_speed_limit;
    unsigned char nose_to_WP;
    float landing_wind_threshold;
    int battery_capacity;
    float WindTableScale;

    float elevator_gain;
    float dcp_gain;
    float throttle_offset;
    float AccIntegGains[3];
    float AltitudeBaroGPSblend_final;
    float Pos_GPS_IMU_BlendGlitch;
    float Pos_GPS_IMU_BlendReg;
    float BaroVspeedWeight;
    float BaroAltitudeWeight;
    float GPSVspeedWeight;
    int  gps_vspeed;
    float TurnAccParams[3];
    float joystick_max_speed;
    //float gyro_ofs[3];
} ModifiableConfigData;

typedef struct
{
    int version;
    int valid;
    float comp_ofs[3];
    float comp_gains[3];
    float compassMin[3];
    float compassMax[3];
} CompassCalibrationData;

typedef struct
{
    float PRstick_rate;
    float PRstick_angle;
    float YawStick_rate;
    float Stick_Vspeed;      // vertical speed
    float Stick_Hspeed;      // horizontal speed
    float StickDeadband[4];   // deadband on sticks (P, R, Y, C), % in config, rescaled to servo range
    
    int waypoint_type;     // current waypoint type
    unsigned char throttle_armed;    // enables throttle
    unsigned char waypoint_stage;    // current stage of takeoff/landing WP

    bool eng_super_user;
    unsigned char rc_ctrl_request;			// Responding to changes in RC Radio inputs
    unsigned char ctrl_source;          // selects source of control - rc, joy, auto....
    unsigned char inhibitRCswitches;    // inhibits RC radio mode switches, needed during takeoff
    unsigned char LidarCtrlMode;        // lidar drives altitude to avoid getting below minimum above ground altitude
    unsigned char cruise_mode;         // enables cruise mode
    unsigned char fixedThrottleMode;	// fixed pitch throttle mode idle, dead band, ramp, fly
    float fixedThrottleCap;				// Capture throttle position for Ramp detection
    float fixedThrottleMult;			// multiplier for ramp 0 to 1
    
    unsigned char display_mode;
    unsigned char joystick_new_values;
    unsigned char joy_PRmode;
    float joy_values[5];    // [P, R, Y, C, T] same range as RC radio
    
    int afsi_takeoff_enable;  // only used as a flag to make sure TAKEOFF happens without interruption

    /* playlist stuff */
    unsigned int playlist_items;
    unsigned int playlist_position;
    unsigned char playlist_status;
    unsigned char pl_wp_initialized;    // indicates that the current waypoint has already been initialized from the playlist
    T_PlaylistItem playlist[PLAYLIST_SIZE];
    
    /* sensor calibration stuff */
    float calib_gyro_avg[3];
    int   calib_count;
    int   calibrate;
    
    float AltitudeBaroGPSblend;
    float Pos_GPS_IMU_Blend;
    
    int   print_counter;
    char  btnMenuPrev;
    char  btnSelectPrev;
    int   btnMenuCounter;
    int   btnSelectCounter;
    int   ticks_curr;   // current loop duration in us
    int   ticks_lp;
    int   ticks_max;
    int   time_ms;
    int   touchdown_time;
    int   resetandarm_req;		// request to reset and arm the FCU from button RVW
    int   resetandarm_time;     // time captured when reset and arm was requested RVW
    int   throttle_width;  		// for battery tester and gyro calib modes
    int   delay_counter;    	// counter for DELAY command, in us

    unsigned char control_mode[5];  // [P, R, Y, C, T] Control modes, 0-inhibit, 1-manual, 2-rate, 3-angle
    float SmoothAcc[3]; // low-passed acc
    int   orient_reset_counter; // once it reaches zero, IMUorient will be reset to SmoothAcc
    float acc[3];               // accelerometer output in heli coordinates [R, F, U] G units
    float gyro[3];              // gyro output [P, R, Y] in deg/s units
    float gyroFilt[3];
    float gyroOfs[3];          //
    float gyro_lp_disp[3];           // long LP of gyro for display
    float accFilt[3];
    float accFilt_prev[3];
    float accHeliRFU[3];            // acceleration acting on heli in heli coordinates [R,F,U] in Gs, including gravity, unused
    float GPSspeedGroundENU[3];     // heli ground speed in ground coordinates [E, N, U] in m/s
    float accGroundENUhp[3];    	// vertical acceleration relative to ground, 1G removed, high-passed
    float accGroundENU_prev[3];
    float IMUspeedGroundENU[3]; 	// Acc based vertical speed
    float speedCtrlPrevEN[2];		// previous speed control values
    float speedHeliRFU[3];          // heli relative speed [right, forward, up] in m/s
    double positionLatLon[2];       // GPS coordinates [Lat, Lon] in deg, combination of raw GPS values and IMU
    float IMUorient[3];             // heli orientation relative to ground [P, R, Y] in rad
    float IMUorient_prev[3];
    float gyro_temperature;
    float gyro_temp_lp;
    float esc_temp;
    float ctrl_out[NUM_CTRL_MODES][5]; // output signals from the controller [raw, rate, angle, speed, pos][P, R, Y, C, T];
    float mixer_in[5];			// input values to the mixer
    float servos_out[8];        // values going to servos (P(A), R(B), Y, C, T)
    float ctrl_yaw_rate;        // stores actual yaw rate for auto banking
    float dyn_yaw_rate;			// yaw rate during playlist turns in heli mode
    float ctrl_vspeed_3d;		// set vspeed in AUTOPILOT mode when collective is in vspeed mode, for auto takeoff/landing
    float collective_raw_curr;  // current raw collective value
    float ctrl_collective_3d;   // target collective in AUTOPILOT mode when mode is raw collective, for takeoff/landing
    float ctrl_collective_raw;  // current collective value in AUTOPILOT mode
    float ctrl_angle_pitch_3d;	// set angle pitch in AUTOPILOT mode when pitch is in angle mode, for auto takeoff
    float ctrl_angle_roll_3d;	// set angle roll in AUTOPILOT mode when roll is in angle mode, for auto takeoff
    float acc_dyn_turns;		// speed adjusted acceleration during turns
    float throttle_value;		// throttle value in manual/auto modes
    float throttle_offset;      // offset for fine tuning of RPM
    float collective_value;		// collective value in manual/auto modes
    float wind_speed;           // wind speed in m/s
    float wind_course;          // wind course in deg cw from north, wind blowing to
    float bankPitch;            // auto banking pitch angle
    float bankRoll;             // auto banking roll angle
    
    float altitude;             // altitude in m, mix of baro, acc and gps
    float altitude_baro;        // altitude in m, mix of baro and acc
    float altitude_gps;         // altitude in m, raw GPS value
    float altitude_ofs;         // offset in between GPS and baro based altitude

    float altitude_lidar;       // altitude above ground from LIDAR, angle compensated
    float altitude_lidar_raw;   // altitude above ground from LIDAR, raw value
    float lidar_timeouts[MAX_NUM_LIDARS];
    int   lidar_online_mask;  //lidar_online_mask identifies which lidars are reporting, bit 0 = lidar node 0, bit # = lidar node_id #

    float altitude_WPnext;      // next waypoint altitude, relative to takeoff site
    unsigned int lidar_rise;	// system time at the rising edge of lidar pwm;
    unsigned int lidar_fall;    // system time at the falling edge
    unsigned int lidar_counter;
    unsigned int rpm_time_ms;   // system time in ms at the rising edge of RPM sensor pulse
    unsigned int rpm_time_ms_last;
    unsigned int rpm_ticks;     // system time in systick at the rising edge of RPM sensor pulse
    unsigned int rpm_dur_us;    // pulse duration in us, valid only when duration in ms<100
    float RPM;                  // RPM of the main shaft

    int   waypoint_retire;     // when true, waypoint retire logic is activated
    
    float home_pos[3];                  // home position [lat/long/alt] in meters
    float  altitude_base;				// reference value for altitude control
    double waypoint_pos[3];             // waypoint position [lat/long/alt] in meters
    double waypoint_pos_prev[3];        // starting waypoint position [lat/long/alt] in meters
    double waypoint_pos_resume[3];      // waypoint position to resume to [lat/long/alt] in meters
    bool waypoint_retire_resume;
    float  waypoint_STdist;             // distance between current and prev waypoints, in m
    float  waypoint_STofs;
    float  waypoint_STcourse;           // course from prev to curr waypoint in deg, CW
    float  path_a;                      // a param of the current waypoint path
    float  path_b;                      // b param of the current waypoint path
    float  path_dist_denom;             // denominator for curr2path distance calculation
    float baro_temperature;             // temperature from BMP-180 in degrees C
    float baro_pressure;                // pressure from BMP-180 in Pa
    float baro_altitude_raw;            // baro based altitude in meters
    float baro_altitude_raw_lp;         // baro based 1second lowpassed altitude in meters
    float accUp;
    float baro_vspeed;                  // baro-based vertical speed m/s
    float baro_vspeed_lp;               // baro-based vertical speed m/s low pass filter
    float baro_vspeedDF;                // baro-based vertical speed m/s with derivative filter applied
    float lidar_vspeed;
    float baro_dT;                      // time interval for vspeed
    float distance2WP_min;              // minimum distance so far to the next waypoint

    float takeoff_height;               // height to achieve during takeoff
    float landingWPHeight;              // height to achieve when reaching a landing WP.
    float landing_timeout;              // time (seconds) to wait to completely spool down motors at end of landing


    float heading;
    float heading_offset;

    float compass_heading;              // heading based on compass in deg CW starting from north
    float compass_heading_lp;           // 1s low-passed version
    
    float gps_heading;                  // deg
    float gps_speed;                    // m/s
    float gps_to_home[3];               // distance[m], course[deg], altitude difference[m]
    float gps_to_waypoint[3];           // distance[m], course[deg], altitude difference[m]
    int   tGPS_prev;                    // time in ms of the previous GPS update (GGA)
    float motor_bat[2];               	// motor battery voltage / percentage
    int   message_timeout;				// message timeout for execute stage
    int  telem_ctrl_time;
    int  telem_ctrl_period;             // CTRL0 msg period in main loop loops
    char gps_new_data;				    // copy of the flag below synchronized with the main loop
    char message_from_ground;		    // gets set by a message from the ground
    char rpm_pulse;                     // signal from RPM interrupt
    char lidar_pulse;
    char gps_alt_initialized;           // altitude has been intialized by GPS
    unsigned char fcm_linkLive_counter;       // counter for producing throttle PWM
    unsigned char linklive_item;        // 1 reset, 2 calib, 3 vol, ...
    char pid_params_changed;            // indicates that PID params have been updated in flight
    unsigned char landing_sites_num;	// number of uploaded landing sites
    unsigned int linklive_t1;
    unsigned int linklive_t2;
    float        linklive_calib;
    float        linklive_values[13];   // castle linklive actual values
    unsigned int linklive_period_T;
    
    int comp_pitch_flags[PITCH_COMP_LIMIT]; // used to tack whether the compass hit pitches from -90 to 90 deg
    int comp_roll_flags[ROLL_COMP_LIMIT]; // used to check whether the compass hit rolls from -180 to 180 degress
    int comp_calibrate;     // 0 means not calibrating, 1 means calibrating, 2 means just finished calibration
    
    unsigned char profile_mode;             // enables and controls profiling
    unsigned char profile_ctrl_variable;    // selects the controlled variable for auto-profiling (P, R, Y, C, T)
    unsigned char profile_ctrl_level;       // selects the level of control for auto-profiling (raw/rate/angle)
    uint16        profile_lag;              // initial lag (T1) in ms
    uint16        profile_period;           // test period (Tt) in ms
    sint16        profile_delta;            // control value (dC) in % of stick - 1000 means 100%
    int           profile_start_time;       // starting time of profiling in ms
    
    T_LP4 lp_gyro4[3];   // lowpass filter for gyro
    T_LP4 lp_acc4[3];    // lowpass filter for accelerometer
    T_LP4 lp_baro4;
    T_LP4 lp_baro_vspeed4;

    
    T_PID pid_PitchRate;
    T_PID pid_PitchAngle;
    T_PID pid_RollRate;
    T_PID pid_RollAngle;
    T_PID pid_YawRate;
    T_PID pid_YawAngle;
    T_PID pid_CollVspeed;
    T_PID pid_PitchSpeed;
    T_PID pid_PitchCruise;
    T_PID pid_RollSpeed;
    T_PID pid_CollAlt;
    T_PID pid_Dist2T;               //distance to target
    T_PID pid_Dist2P;               //distance to path
    T_PID pid_IMU[3];
    float   speed_Iterm_E;          // I-term for speed PID, east
    float   speed_Iterm_N;          // I-term for speed PID, north
    float   speed_Iterm_E_lp;       // I-term for speed PID, east
    float   speed_Iterm_N_lp;       // I-term for speed PID, north
    float   ctrl_initial[5];        // initial stick values at the beginning of logging
    float   cpu_utilization_lp;     // 0-100%
    float   cpu_utilization_curr;

    float	baro_derivative_filter[11];

    char    streaming_enable;       // enables data streaming
    byte    streaming_types[7];     // selects individual data types for streaming
    byte    streaming_channels;     // number of channels included
    byte    streaming_channel;      // current channel
    byte    streaming_samples;      // samples per packet, multiple channel caount as one sample
    byte    streaming_sample;       // current sample
    f16     stream_data[120];
           
	byte 	msg2ground_id;
	byte	msg2ground_count;
    char    tcpip_confirm;
    byte    tcpip_org_type;
    byte    tcpip_org_len;
    uint16  tcpip_user1;  // custom field 1
    uint16  tcpip_user2;  // custom field 2
    
    float gyro_ofs[3];

    int box_dropper_;

    uint16_t controlStatus;

    uint32_t fcm_serialnum_0;
    uint32_t fcm_serialnum_1;
    uint32_t fcm_serialnum_2;
    uint32_t fcm_serialnum_3;

    unsigned int imu_serial_num;

    ModifiableConfigData rw_cfg;

    CompassCalibrationData compass_cal;

    T_Command command;
    T_Telem_Ctrl0   telemCtrl0;
    T_Telem_GPS1    telemGPS1;
    T_Telem_System2 telemSystem2;
    T_Telem_TCPIP7  telemTcpip7;
    T_Telem_DataStream3 telemDataStream3;
    T_Telem_Msg2Ground  telemMsg2ground;
    T_AircraftConfig    aircraftConfig;
    T_Telem_Calibrate   telemCalibrate;

    /* state for resuming playlist */
    T_State	state;

    /* power/battery stuff */
    T_Power	power;

    /* statistics */
    T_Stats stats;

    /* list of emergency landing sites */
    T_LandingSite	landing_sites[LANDING_SITES];

    int debug_flags[10]; // Temp debug flags to be used however

    bool setZeroSpeed;

} FlightControlData;


typedef struct
{
    unsigned short int id : 3;
    signed short int value : 13;
} T_Servo;

typedef struct
{
    T_Servo servo[4];
} T_Servos;

typedef struct {
  float      alt[LIDAR_HISTORY_SIZE]; // circular buffer of altitude data (in meters) used for filtering, necessary only for FCM lidar
  int        data_indx;
  int        new_data_rdy;
  float      current_alt;
} Lidar_Data;

/* from power module: AVICAN_POWER_VALUES1 all uint16, Iaux_srv, Iesc, Vesc, Vbat
 * 					  AVICAN_POWER_VALUES2 all uint16 Vservo, Vaux12V, dTms_adc, adc_count */
typedef struct
{
	uint16	iAuxSrv;
	uint16	iESC;
	uint16	vESC;
	uint16	vBAT;
} T_PowerValues1;

typedef struct
{
	uint16	vServo;
	uint16	vAux12V;
	uint16	dT100us_adc;// duration of this period in hundreds of uS
	uint16	adc_count;	// samples producing the average values in this period
} T_PowerValues2;

// TODO::SP: FIX THIS TRASH
static inline void SetCtrlMode(FlightControlData *hfc, const ConfigData *pConfig, unsigned char channel, unsigned char mode)
{
  if (!pConfig->ctrl_mode_inhibit[channel]) {
    hfc->control_mode[channel] = mode;
  }
}

typedef struct {
    float   battery_voltage;
    float   ripple_voltage;
    float   current;
    float   throttle;
    float   output_power;
    float   rpm;
    float   bec_voltage;
    float   bec_current;
    float   temperature;
    int     new_data_mask;
} CastleLinkLive;

typedef struct {
    float servo_out[8];
} ServoNodeOutputs;

#endif
