#ifndef _DEFINES_H_
#define _DEFINES_H_

#include <stddef.h>
#include "mbed.h"
#include "USBSerial.h"
extern USBSerial   serial;

// Only allow debug printing when in Debug mode AND the serial/usb is in a connected state
// For Release mode builds, used for Flight, NEVER print!
#define debug_print(fmt, ...) \
            do { if (DEBUG && serial.connected()) serial.printf(fmt, ##__VA_ARGS__); } while (0)

// This is used to print to the Serial/USB for operator commands, such as config updates.
// It is therefore Always in the build and the connection check is done elsewhere.
#define usb_print(fmt, ...) \
            do { serial.printf(fmt, ##__VA_ARGS__); } while (0)

#define CONFIG_VERSION   10
#define COMPASS_CAL_VERSION 1

#define PC_BAUDRATE          115200
#define GPS_BAUDRATE         38400

#define PLAYLIST_SIZE   512

#define LOOP_PERIOD     1000    // 1000Hz   830 // in mS

//#define THRUST_VECTORING    // enables thrust vectoring during turns, airplane mode otherwise

/* indices and coordinates */
#define PITCH   0
#define ROLL    1
#define YAW     2
#define COLL    3
#define THRO    4

#define CCPM_A  0
#define CCPM_B  1
#define CCPM_C  3

#define X_AXIS  0
#define Y_AXIS  1
#define Z_AXIS  2

#define RIGHT    0
#define FORWARD  1
#define UP       2
#define EAST     0
#define NORTH    1

/* layer of control */
#define RAW             0
#define RATE            1
#define ANGLE           2
#define SPEED           3
#define POS             4
#define NUM_CTRL_MODES  5   // number of control layers

/* low-pass selectors for sensors */
#define LP_GYRO     0
#define LP_ACC      1

/* xbus channel mappings */
#define XBUS_THRO     		0
#define XBUS_ROLL     		1
#define XBUS_PITCH    		2
#define XBUS_YAW      		3
#define XBUS_THR_SW   		4		// selects collective mode
#define XBUS_THR_LV   		5		// controls throttle
#define XBUS_CTRLMODE_SW 	6		// selects between full-auto and RC manual modes
#define XBUS_MODE_SW  		7		// selects cyclic mode
#define DCPGAIN             12      // Tandem mixer DCP gain channel
#define ELEVGAIN            13      // Tandem mixer elevator gain channel

/* modes */
#define CCPM_NONE   0
#define CCPM_120    1
#define CCPM_140    2
#define CCPM_QUAD   3
#define CCPM_HEX    4
#define CCPM_OCTO    5
#define MIXERTANDEM 6

#define PROP_FIXED_PITCH       0 // collective channel drives throttle, throttle lever gates it
#define PROP_VARIABLE_PITCH    1 // lever drives throttle, collective channel drives blade pitch

// control modes
#define CTRL_MODE_INHIBIT       0   // inhibit servo control, set to mid pos, or min for throttle
#define CTRL_MODE_MANUAL        1   // fully manual stick=servo
#define CTRL_MODE_RATE          2   // stick control rate
#define CTRL_MODE_ANGLE         3   // stick controls angle
#define CTRL_MODE_SPEED         4   // stick controls speed
#define CTRL_MODE_POSITION      6   // stick controls altitude

#define WAYPOINT_NONE           -1
#define WAYPOINT_GOTO           0   // go-to waypoint
#define WAYPOINT_FLYTHROUGH     1   // fly-through waypoint
#define WAYPOINT_TAKEOFF        2
#define WAYPOINT_LANDING        3

#define TAKEOFF_HEIGHT_MIN      2   // Takeoff minimum height
#define TAKEOFF_HEIGHT_DEFAULT  10  // Default Takeoff height
#define TAKEOFF_HEIGHT_MAX      40  // Max Takeoff Height

#define LANDING_THRESHOLD_HEIGHT 0.2 //in meters, used in landing

/* display modes defines */
#define DISPLAY_SPLASH		  0
#define DISPLAY_STATUS        1
#define DISPLAY_POWER         2
#define DISPLAY_CONTROLS      3
#define DISPLAY_XBUS          4
#define DISPLAY_GPS1          5
#define DISPLAY_GPS2          6
#define DISPLAY_GPS3          7
#define DISPLAY_BARO          8
#define DISPLAY_COMPASS       9
#define DISPLAY_CALIB         10
#define DISPLAY_WAYPOINT      11
#define DISPLAY_ENG           12
#define DISPLAY_PAGES         13

#define FLY_ALL_SENSORS 0
#define FLY_NO_COMPASS  1

#define GROUND_SENSOR_NONE      0
#define GROUND_SENSOR_LIDAR     1
#define GROUND_SENSOR_SONAR     2

/* constants */
#define D2R 0.017453292f
#define R2D 57.29578122f
#define EarthR  6372795
#define DPM (180.0f/EarthR/3.14159265359f)   // degrees per meter

#define IMU_ACC_GYRO_FREQ                 (1/20.0f)  // 15 seconds
#define ALTITUDE_BARO_GPS_BLEND_FREQ      600.0f     // 600sec baro-gps altitude blend - final
#define ALTITUDE_BARO_GPS_BLEND_FREQ_INIT 10.0f      // 10sec baro-gps altitude blend - initial
#define POS_GPS_IMU_BLEND_REG             (1)        // 1 sec gps-imu horizonatal position blend - regular mode
#define POS_GPS_IMU_BLEND_GLITCH          (20)       // 20 sec gps-imu horizonatal position blend - during GPS measurement glitch
#define ACC_HP_DECAY                      (0.99988f) // ~7 sec acc high pass filter decay 
#define ALT_BARO_GPS_DECAY                (0.9973f)  // decay from the initial to the final blending factor, assuming 10Hz GPS, it takes 5min to get to final
#define VSPEED_ACC_GPS_BLEND			  (1.0f)	 // 1 sec

#define CALIBRATE_SAMPLES   10*500      // 5 seconds

#define AUTO_PROF_TERMINATE_THRS    0.05f   // threshold on stick value change for terminating of the auto-profiling

/* functions */
#define ABS(a) ((a)<0 ? -(a) : (a))
#define CLIP(a,limit) ((a)>(limit) ? (limit) : (a)<-(limit) ? -(limit) : (a))
#define ClipMinMax(a, minv, maxv) ((a)>(maxv) ? (maxv) : (a)<(minv) ? (minv) : (a))
#define max(a,b) ((a)>(b) ? (a) : (b))
#define min(a,b) ((a)<(b) ? (a) : (b))
#define Min(a,b) ((a)<(b) ? (a) : (b))
#define Max(a,b) ((a)>(b) ? (a) : (b))
#define SERVOMINMAX(a) ((a)>1 ? 1 : (a)<-1 ? -1 : (a))
#define SQR(a) ((a)*(a))
#define sign(a) ((a)<0 ? -1 : 1)

#define CLOCK() (LPC_RIT->RICOUNTER)

// high pass filter U2 = T/(1+T)*(U-Uprev+U2prev)
/* logging parameters 6.2 designation - category.sub-category */
#define LOG_PARAM_NONE              0
#define LOG_PARAM_XBUS_INPUT0_3     1   // xbus input[0-3] (+/-1 f16)
#define LOG_PARAM_XBUS_INPUT4_7     2   // xbus input[4-7] (+/-1 f16)
#define LOG_PARAM_JOY_INPUT         3   // joystick inputs[P/R/Y/C] (based on selected mode, moslt speeds in m/s f16)
#define LOG_PARAM_GYRO_RAW          4   // gyro [P/R/Y] (in deg/s f16)
#define LOG_PARAM_GYRO_LP           5   // gyro low passed [P/R/Y] (in deg/s f16)
#define LOG_PARAM_ACC_RAW           6   // acc [R/F/U] (in Gs f16)
#define LOG_PARAM_ACC_LP            7   // acc low passed [R/F/U] (in Gs f16)
#define LOG_PARAM_COMPASS           8   // compass [heading/R/F/U] (heading in deg CW from north) (f16)
#define LOG_PARAM_ORIENT            9   // IMUorient [P/R/Y] in deg (f16)
#define LOG_PARAM_BARO              10  // baro [altitude in m / pressure in kPa / temp in deg C] (f16)
#define LOG_PARAM_PITOT             11  // pitot [pressure in kPA / speed in m/s / temp in deg C] (f16)
#define LOG_PARAM_SERVOS0_3         12  // servos [0-3] +/-1 (f16)
#define LOG_PARAM_SERVOS4_7         13  // servos [4-7] +/-1 (f16)
#define LOG_PARAM_CTRL_MODE         14  // ctrl_mode [P0-2, R3-5, Y6-8, C9-11, T12-14] (int, 3-bits each)
#define LOG_PARAM_CTRL_RAW0         15  // control output values [P, R, Y, C] +/-1 (f16)
#define LOG_PARAM_CTRL_RAW1         16  // control output values [T] +/-1 (f16)
#define LOG_PARAM_CTRL_RATE         17  // ctrl_rate [P/R/Y] in deg/s (f16)
#define LOG_PARAM_CTRL_ANGLE        18  // ctrl_angle [P/R/Y] in deg (f16)
#define LOG_PARAM_CTRL_SPEED        19  // ctrl_speed in m/s [F/R/U] (f16)
#define LOG_PARAM_ALTITUDE          20  // altitude [CTRL / current IMU / lidar] in m (f16)
#define LOG_PARAM_GROUND_SPEED      21  // ground speed in m/s [total/E/N/U] (f16)
#define LOG_PARAM_HELI_SPEED        22  // heli speed in m/s [R/F/U] (f16)
#define LOG_PARAM_CPU               23  // CPU stats [process_period in uS int / utilization in % f16]
#define LOG_PARAM_TIME              24  // CPU time in ms (int)
#define LOG_PARAM_POSITION          25  // heli position relative to home [horizontal distance in m / heading in deg / altitude difference in m] (f16)
#define LOG_PARAM_TANDEM            26


//#define LOG_PARAM_MOT_BAT_VOLT      7  // motor_bat_voltage (f16 hex)
//#define LOG_PARAM_RECV_BAT_VOLT     8  // recv_bat_voltage (f16 volt)
//#define LOG_PARAM_TEMPERATURE       13  // temperature in deg C from gyro (f16)
//#define LOG_PARAM_XBUS_IN_PROC      2  // processed xbus inputs [P, R, Y, C, T] +/-1 (f16 hex)
//#define LOG_PARAM_THR_KILL          6  // throttle_kill switch (0/1 int hex)

#define PROFILING_OFF       0   // disabled
#define PROFILING_START     1   // start profiling
#define PROFILING_ON        2   // active
#define PROFILING_FINISH    3   // to be stopped

#define LOCK_JTAG     (1)
#define UNLOCK_JTAG   (!LOCK_JTAG)

#define CONTROL_STATUS_NONE      0x00
#define CONTROL_STATUS_TAKEOFF   (1 << 0)
#define CONTROL_STATUS_LAND      (1 << 1)
#define CONTROL_STATUS_HOME      (1 << 2)
#define CONTROL_STATUS_POINTFLY  (1 << 3)
#define CONTROL_STATUS_PLAY      (1 << 4)
#define CONTROL_STATUS_PAUSE     (1 << 5)
#define CONTROL_STATUS_PREFLIGHT (1 << 6)

#define IN_THE_AIR(X) ( (( X ) > 0.5) ? 1 : 0 )
#define THROTTLE_LEVER_DOWN() ((xbus.valuesf[XBUS_THR_LV] < (-0.99f*pConfig->Stick100range)) ? 1 : 0)
#define THROTTLE_LEVER_UP()   ((xbus.valuesf[XBUS_THR_LV] > ( 0.99f*pConfig->Stick100range)) ? 1 : 0)

#endif
