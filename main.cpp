/**
  ******************************************************************************
  * @file    main.cpp
  * @author  AVIDRONE FIRMWARE Team
  * @brief   Provides support for FCM node of Avidrone FCS
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
#include "mbed.h"
#include "hardware.h"
#include "MPU6050.h"
#include "utils.h"
#include "mymath.h"
#include "NOKIA_5110.h"
#include "IMU.h"
#include "xbus.h"
#include "PID.h"
#include "HMC5883L.h"
#include "BMP180.h"
#include "pGPS.h"
#include "telemetry.h"
#include "avican.h"
#include "USBSerial.h"
#include "IAP.h"
#include "version.h"

extern int __attribute__((__section__(".ramconfig"))) ram_config;
static unsigned char *pRamConfigData = (unsigned char *)&ram_config;

void GenerateSpeed2AngleLUT(void);
void ResetIterms(void);
void AltitudeUpdate(float alt_rate, float dT);
void HeadingUpdate(float heading_rate, float dT);

extern int LoadConfiguration(const ConfigData **pConfig);
extern int LoadCompassCalibration(const CompassCalibrationData **pCompass_cal);
extern int SaveNewConfig(void);
extern int SavePIDUpdates(FlightControlData *fcm_data);
extern int SaveCompassCalibration(const CompassCalibrationData *pCompass_cal);

USBSerial   serial(0x1f00, 0x2012, 0x0001, false);
SPI         spi(MC_SP1_MOSI, MC_SP1_MISO, MC_SP1_SCK);
NokiaLcd    myLcd( &spi, MC_LCD_DC, MC_LCD_CS, MC_LCD_RST );

I2Ci        Li2c(I2C_SDA1, I2C_SCL1);
MPU6050     mpu(&Li2c, -1, MPU6050_GYRO_FS_500, MPU6050_ACCEL_FS_4);
HMC5883L    compass(&Li2c);
BMPx80      baro(&Li2c);

DigitalIn   btnSelect(MC_BUTTON2);
DigitalIn   btnMenu(MC_BUTTON1);

DigitalOut  led1(LED_1);
DigitalOut  led2(LED_2);
DigitalOut  led3(LED_3);
DigitalOut  led4(LED_4);
DigitalOut  ArmedLed(LED_3);

XBus xbus(XBUS_IN);
CAN can(CAN_RXD1, CAN_TXD1);
GPS gps;

//Serial pc(TRGT_TXD, TRGT_RXD);  // Debug Serial Port
//Serial pc(GPS_TX, GPS_RX);  // Debug Serial Port

RawSerial telemetry(TELEM_TX, TELEM_RX);
TelemSerial telem(&telemetry);

InterruptIn  lidar(LIDAR_PWM);
InterruptIn  *rpm = NULL;

PwmOut *FCM_SERVO_CH6 = NULL;   // This can be re-purposed as RPM Sensor Input
PwmOut FCM_SERVO_CH5(CHANNEL_5);
PwmOut FCM_SERVO_CH4(CHANNEL_4);
PwmOut FCM_SERVO_CH3(CHANNEL_3);
PwmOut FCM_SERVO_CH2(CHANNEL_2);
PwmOut *FCM_SERVO_CH1 = NULL;   // This can be re-purposed as LiveLink Input

DigitalInOut *FCMLinkLive = NULL;   // When re-purposed Ch1 for LiveLink
InterruptIn  *linklive = NULL;      // When re-purposed Ch1 for LiveLink
Ticker livelink_timer;

FlightControlData hfc = {0};
const ConfigData *pConfig = NULL;

// Text displayed on ShowSplash
#define AVIDRONE_SPLASH "== AVIDRONE =="
#define AVIDRONE_FCM_SPLASH "    AVI-FCM      "

#define MAX_NUM_LIDAR        2
#define MAX_NUM_CASTLE_LINKS 2
#define MAX_NUM_GPS          1
#define MAX_BOARD_TYPES      7
#define MAX_NODE_NUM         15

#define MAX_NUMBER_SERVO_NODES  2

/* Passed in the GPS CANbus CFG Message, 1byte */
#define GPS_SEL_CHIP_0          (1 << 0)
#define GPS_SEL_CHIP_1          (1 << 1)
#define GPS_SEL_CHIP_AUTO       (GPS_SEL_CHIP_0 | GPS_SEL_CHIP_1)
#define COMPASS_SEL_MASK(x)     ((x) << 2)

static T_lidar lidarPayload[MAX_NUM_LIDAR];
#define MAX_LIDAR_PULSE 40000   // 40m max range; 10us/cm PWM

typedef struct {
    uint8_t major_version;
    uint8_t minor_version;
    uint8_t build_version;
    uint32_t serial_number0;
    uint32_t serial_number1;
    uint32_t serial_number2;
} BoardInfo;

BoardInfo board_info[MAX_BOARD_TYPES][MAX_NODE_NUM];

CastleLinkLive castle_link_live[MAX_NUM_CASTLE_LINKS];
double heading[MAX_NUM_GPS];
static void GpsHeartbeat(int node_id);
static void ServoHeartbeat(int node_id);
static void PowerNodeHeartbeat(int node_id);

ServoNodeOutputs servo_node_pwm[MAX_NUMBER_SERVO_NODES+1]; // Index 0 is FCM so MAX number is +1
static int init_ok = 1;
static int init_warning = 0;

#define CAN_TIMEOUT 500
static int can_node_found = 0;
static int canbus_ack = 0;
static int write_canbus_error = 0;
static int can_power_coeff = 0;
static int canbus_livelink_avail = 0;
static int power_update_avail = 0;

#define FCM_FATAL_ERROR() { \
    while(1) { \
    	WDT_Kick(); \
    	led1 = 1; led2 = 1; led3 = 1; led4 = 1; \
    	wait(1.0f); \
    	led1 = 0; led2 = 0; led3 = 0; led4 = 0; \
    	wait(1.0f); \
    } \
}

#define FCM_NOTIFY_CFG_UPDATED() { \
    int state = 0; \
	for (int i=0; i < 10; i++) { \
    	WDT_Kick(); \
    	led1 = led2 = led3 = led4 = state; \
    	wait(0.3f); \
    	state = !state; \
    } \
}

void ProcessButtonSelection();
static void Buttons();
static void PrintOrient();
static void RPM_rise();
static void UpdateBatteryStatus(float dT);

float GetAngleFromSpeed(float speed, const float WindSpeedLUT[ANGLE2SPEED_SIZE], float scale)
{
  int i;
  int angle1;
  int angle2 = ANGLE2SPEED_SIZE-1;
  float angle;
  float speed1, speed2;

  for (i=0; i<ANGLE2SPEED_SIZE; i++) {
    if ((WindSpeedLUT[i]*scale)>speed) {
      angle2 = i;
      break;
    }
  }

  angle1 = angle2 - 1;
  speed1 = WindSpeedLUT[angle1]*scale;
  speed2 = WindSpeedLUT[angle2]*scale;

  if (speed1==speed2) {
    return angle1;
  }

  angle = (angle2-angle1) / (speed2-speed1) * (speed-speed1) + angle1;

  return angle;
}

void GenerateSpeed2AngleLUT(void)
{
    int i;
    for (i=0; i<SPEED2ANGLE_SIZE; i++) {
        float speed = i*0.5f;
        float angle = GetAngleFromSpeed(speed, pConfig->WindSpeedLUT, hfc.rw_cfg.WindTableScale);
        hfc.rw_cfg.Speed2AngleLUT[i] = angle;
    }
}

void ResetIterms(void)
{
    hfc.pid_PitchRate.Ie  = 0;
    hfc.pid_RollRate.Ie   = 0;
    hfc.pid_YawRate.Ie    = 0;
    hfc.pid_PitchAngle.Ie = 0;
    hfc.pid_RollAngle.Ie  = 0;
    hfc.pid_YawAngle.Ie   = 0;
    hfc.pid_CollVspeed.Ie = 0;
    hfc.pid_PitchSpeed.Ie = 0;
    hfc.pid_RollSpeed.Ie  = 0;
    hfc.pid_CollAlt.Ie    = 0;
    hfc.pid_Dist2T.Ie     = 0;
    hfc.pid_Dist2P.Ie     = 0;
    hfc.pid_PitchCruise.Ie= 0;
    hfc.speed_Iterm_E     = 0;
    hfc.speed_Iterm_N     = 0;
}

void AutoReset(void)
{
	float delta_accel[3];
	float delta_orient[3];

	float degreesPerSecLimit = 0.2;
	float accelLimit = 0.05;

	if( !hfc.throttle_armed )
	{
		if( pConfig->autoReset && ((hfc.print_counter % 500) == 0) )
		{
			for(int i = 0; i < 3; i++)
			{
				delta_accel[i]  = ABS( hfc.accFilt[i] - hfc.accFilt_prev[i] );
				delta_orient[i] = ABS( hfc.IMUorient[i] - hfc.IMUorient_prev[i] );
				hfc.accFilt_prev[i] = hfc.accFilt[i];
				hfc.IMUorient_prev[i] = hfc.IMUorient[i];
			}

			/*Check if accelerometer readings have changed significantly, IF NOT THEN
			* --> check IF the orientation of the IMU has changed significantly, OR
			*     IF the accelerometer estimate of orientation is far from the gyro estimate
			*     IF YES THEN...
			* --> do a reset*/
			if(  delta_accel[X_AXIS] < accelLimit
			 &&  delta_accel[Y_AXIS] < accelLimit
			 &&  delta_accel[Z_AXIS] < accelLimit )
			{
				if( delta_orient[PITCH] >  degreesPerSecLimit
				 || delta_orient[ROLL]  >  degreesPerSecLimit
				 || delta_orient[YAW]   >  degreesPerSecLimit
				 || ABS(hfc.SmoothAcc[PITCH] - hfc.IMUorient[PITCH])>(0.5f*D2R)
				 || ABS(hfc.SmoothAcc[ROLL]  - hfc.IMUorient[ROLL] )>(0.5f*D2R)  )
				{
					telem.ResetIMU(false);
				}
			}

			/*Check if GPS signal is good, if so, then reset if
			 * IMU altitude and GPS altitude differ by more than 2m */
			//GpsData gps_data = gps.GetGpsData();
			if ( (gps.gps_data_.fix > GPS_FIX_NONE) && (gps.gps_data_.PDOP < 250) ) {
				if (ABS(hfc.altitude_baro - hfc.altitude_gps) >= 2) {
					hfc.altitude_ofs = hfc.altitude_gps - hfc.altitude_baro;
				}
			}
		}
	}

	return;
}

static void OrientResetCounter()
{
    if (hfc.orient_reset_counter) {
        hfc.orient_reset_counter--;
        if (!(hfc.orient_reset_counter&0x3ff)) {
            /*mmri: just took out the carriage return so that gyrotemp
             * compensation output data is more easily read in .csv file*/
            debug_print("IMU reset   ====   %f %f === ", hfc.SmoothAcc[PITCH]*R2D, hfc.SmoothAcc[ROLL]*R2D);
            telem.ResetIMU(false);
        }
    }
}

#define AccLP_Freq  5.0f     // 1/T

/*Calculation of pitch and roll based on accelerometer data.
 * see: NXP application note AN3461 section 3 - Pitch and Roll Estimation
 *  https://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf */
void Get_Orientation(float *SmoothAcc, float *AccData, float dt)
{
//    float GyroRate[3];
    float micro = 0.02f;
    float AccAngle[3];

    float sign = AccData[Z_AXIS] < 0 ? -1 : 1;
//    float temp1, temp2;
    
    /* Accelerometer xyz into Pitch and Roll relative to the horizon
     * - roll is limited to +/- 180vdegress
     * - pitch is limited to +/- 90 degrees */
    AccAngle[ROLL]  =  ATAN2fR(-AccData[X_AXIS], sign*sqrtf(AccData[Z_AXIS] * AccData[Z_AXIS] + micro*AccData[Y_AXIS] * AccData[Y_AXIS]));
    AccAngle[PITCH] = (ATAN2fR(AccData[Y_AXIS], sqrtf(AccData[Z_AXIS] * AccData[Z_AXIS] + AccData[X_AXIS] * AccData[X_AXIS])));

    /* Smooth out Acc Pitch and Roll */
    SmoothAcc[ROLL]  = (AccAngle[ROLL]  - SmoothAcc[ROLL] )*dt*AccLP_Freq + SmoothAcc[ROLL];  // Averaging roll ACC values
    SmoothAcc[PITCH] = (AccAngle[PITCH] - SmoothAcc[PITCH])*dt*AccLP_Freq + SmoothAcc[PITCH]; // Averaging pitch ACC values
} 

static void linklive_fall(void)
{
    hfc.linklive_t2 = CLOCK();
    linklive->fall(NULL);
}

static void throttle_pulse_int(void)
{
    FCMLinkLive->write(1);
    hfc.linklive_t1 = CLOCK();
    hfc.linklive_t2 = hfc.linklive_t1;  // if t2 stays the same as t1, pulse from ESC did not happen -> sync
    livelink_timer.detach();
    FCMLinkLive->input();
    FCMLinkLive->mode(PullUp);
    linklive->fall(linklive_fall);
}

static const float LINKLIVE_SCALES[13] =
{
/*  0 */        0,
/*  1 */        0,
/*  2 */        0,
/*  3 */        20*0.001f, // V
/*  4 */        4*0.001f,  // rip V
/*  5 */        50*0.001f, // I
/*  6 */        1*0.001f,  // thro
/*  7 */        0.2502f*0.001f,    // power
/*  8 */        20416.7f*0.001f*2, // rpm
/*  9 */        4*0.001f,          // BEC V
/* 10 */        4*0.001f,          // BEC I
/* 11 */        30*0.001f,         // temp lin
/* 12 */        63.8125f*0.001f    // temp log
};


// temp = 1 / (ln(value*10200 / (255-value) / 10000.0f) / 3455.0f + 1/298.0f) - 273;
// index = value/2
static const short int LL2TEMP[128] = {
23707,23707,18923,16500,14917,13757,12848,12105,11477,10936,10460,10036,9654,9307,8988,8694,8420,
8165,7926,7700,7487,7284,7091,6908,6732,6563,6401,6246,6095,5950,5809,5673,5541,5413,5288,5166,
5047,4931,4818,4708,4599,4493,4389,4287,4186,4087,3990,3895,3800,3707,3616,3525,3436,3348,3260,
3174,3089,3004,2920,2837,2754,2672,2591,2510,2429,2349,2269,2190,2111,2032,1954,1875,1797,1719,
1641,1563,1485,1407,1328,1250,1171,1092,1013,934,854,774,693,612,530,447,364,280,195,109,22,
-66,-155,-246,-338,-432,-527,-624,-723,-825,-928,-1035,-1144,-1256,-1372,-1492,-1616,-1745,
-1880,-2020,-2168,-2323,-2489,-2665,-2854,-3059,-3284,-3534,-3817,-4146,-4542,-5047,-5766,
-7156};

signed short int pwm_values[MAX_NUMBER_SERVO_NODES][8];
//static int servo_remap[8] = { 4, 1, 0, 2, 3, 5, 6, 7 };
static int servo_remap[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };

static void WriteToServoNodeServos(int num_servo_nodes)
{
    CANMessage can_tx_message;
    static int pwm_out = 0;
    float temp;

	for (int i=0; i < num_servo_nodes; i++) {
		for (int j=0; j < 8; j++) {
			temp = servo_node_pwm[i+1].servo_out[j];
			if (i < 6) {
				if (pConfig->servo_revert[i] & (1<<i)) {
					temp = -hfc.servos_out[i];
				}
			}
			// debug_print("WriteToServos[%d][%d], temp=%f\r\n", i+1, j, temp);
			pwm_values[i][j] = (((SERVOMINMAX(temp) * 32767) * 500) /32768) + 1500;
			// debug_print("pwm_values[%d][%d] = %d\r\n", i, j, pwm_values[i][j]);
		}
	}

    //if ((hfc.print_counter%1000)==0) {
    //    debug_print("FT PWM [%d], RT PWM[%d]\r\n", pwm_values[0][0], pwm_values[1][0]);
    //}

    can_tx_message.type   = CANData;
    can_tx_message.format = CANStandard;
    can_tx_message.len    = 8;

    if (pwm_out ^= 1) {
        for (int i=0; i < num_servo_nodes; i++) {
            memcpy(can_tx_message.data, &pwm_values[i][0], 8);

            can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_SERVO_NODETYPE,
                                                    (DEFAULT_NODE_ID + i), AVIDRONE_MSGID_SERVO_LO_CTRL);

            if (!can.write(can_tx_message)) {
                ++write_canbus_error;
            }
        }
    }
    else {
        for (int i=0; i < num_servo_nodes; i++) {
            memcpy(can_tx_message.data, &pwm_values[i][4], 8);

            can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_SERVO_NODETYPE,
                                                    (DEFAULT_NODE_ID + i), AVIDRONE_MSGID_SERVO_HI_CTRL);

            if (!can.write(can_tx_message)) {
                ++write_canbus_error;
            }
        }
    }
}

// NOTE::SP: Only handles a single power node
static void WriteToPowerNodeServos()
{
    CANMessage can_tx_message;
    static int pwm_out = 0;
    float temp;

    for (int i=0; i < 8; i++) {
        temp = hfc.servos_out[servo_remap[i]];

        if (i < 6) {
            if (pConfig->servo_revert[i] & (1<<i)) {
                temp = -hfc.servos_out[i];
            }
        }

        pwm_values[0][i] = (((SERVOMINMAX(temp) * 32767) * 500) /32768) + 1500;
    }

    can_tx_message.type   = CANData;
    can_tx_message.format = CANStandard;
    can_tx_message.len    = 8;

    if (pwm_out ^= 1) {
        memcpy(can_tx_message.data, &pwm_values[0][0], 8);

        can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_PWR_NODETYPE,
                                                (DEFAULT_NODE_ID), AVIDRONE_MSGID_PWR_LO_CTRL);

        if (!can.write(can_tx_message)) {
            ++write_canbus_error;
        }
    }
    else {

        memcpy(can_tx_message.data, &pwm_values[0][4], 8);

        can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_PWR_NODETYPE,
                                                (DEFAULT_NODE_ID), AVIDRONE_MSGID_PWR_HI_CTRL);

        if (!can.write(can_tx_message)) {
            ++write_canbus_error;
        }
    }
}

// inputs values can range from -1 to 1 corresponding to the absolute maximum range
static void WriteToFcmServos(void)
{
    float pwm_values[8];
    float temp;

    for (int i = 0; i < 8; i++) {
        temp = hfc.servos_out[i];
        if (i < 6) {
            if (pConfig->servo_revert[i] & (1<<i)) {
                temp = -hfc.servos_out[i];
            }
        }
        // clip servo values to +/-150%
        pwm_values[i] = SERVOMINMAX(temp);
    }

    // CH1 may have been re-purposed for link live
    if (FCM_SERVO_CH1) {
        FCM_SERVO_CH1->pulsewidth_us((int)(1500.5f + pwm_values[4] * 500));
    }

    FCM_SERVO_CH2.pulsewidth_us((int)(1500.5f + pwm_values[1] * 500));
    FCM_SERVO_CH3.pulsewidth_us((int)(1500.5f + pwm_values[0] * 500));
    FCM_SERVO_CH4.pulsewidth_us((int)(1500.5f + pwm_values[2] * 500));
    FCM_SERVO_CH5.pulsewidth_us((int)(1500.5f + pwm_values[3] * 500));

    // CH6 may have been re-purposed for rpm speed sense
    if (FCM_SERVO_CH6) {
            FCM_SERVO_CH6->pulsewidth_us((int)(1500.5f + pwm_values[5] * 500));
    }
}

static void ProcessFcmLinkLive(void)
{
    hfc.fcm_linkLive_counter++;

    // Every 50Hz
    if (hfc.fcm_linkLive_counter >= 20) {
        int delta = (hfc.linklive_t2 - hfc.linklive_t1);
        // debug_print("%dus\n", delta);
        linklive->fall(NULL);
        FCMLinkLive->output();
        FCMLinkLive->write(0);

        float throttle = hfc.servos_out[THRO];
        if (pConfig->servo_revert[THRO]) {
            throttle = - hfc.servos_out[THRO];
        }
        throttle = SERVOMINMAX(throttle);

        livelink_timer.attach_us(throttle_pulse_int, (int)(1500.5f + throttle * 500));
        hfc.fcm_linkLive_counter = 0;

         // Sync
         if (delta == 0) {
            unsigned int T = CLOCK();
            unsigned int delta_us = (T - hfc.linklive_period_T + 48) / 96;
            const float *coeffs = pConfig->power_coeffs;

            hfc.linklive_item = 1;
            hfc.linklive_period_T = T;
            hfc.power.Iaux   = hfc.linklive_values[10];    // I BEC
            hfc.power.Iesc   = (hfc.linklive_values[5] * coeffs[1] + 3 * hfc.power.Iesc) * 0.25f;  // rvw
            hfc.power.Vmain  = (hfc.linklive_values[3] *coeffs[3] + 3 * hfc.power.Vmain) * 0.25f;
            hfc.power.Vesc   = hfc.power.Vmain;
            hfc.power.Vservo = hfc.linklive_values[9];    // V BEC
            hfc.power.Vaux   = hfc.linklive_values[9];    // V BEC

            if (!pConfig->rpm_sensor) {
                hfc.RPM = hfc.linklive_values[8] / pConfig->gear_ratio / pConfig->motor_poles;
            }

            if (hfc.linklive_values[11] > hfc.linklive_values[12]) {
                hfc.esc_temp = hfc.linklive_values[11]; // linear temperature
            }
            else {

                float value = hfc.linklive_values[12];
                if (value) {
                    int idx = ClipMinMax((int)value * 0.5f, 1, 126);
                    int re = ((int)(value * 0.5f * 256)) & 0xff;
                    int temp1 = LL2TEMP[idx];
                    int temp2 = LL2TEMP[idx+1];
                    int temp3 = (temp1 * (256-re) + temp2 * re);
                    float temp = temp3 / 25600.0f;
                    // float temp = 1 / (logf(value*10200 / (255-value) / 10000.0f) / 3455.0f + 1/298.0f) - 273;
                    hfc.esc_temp = (temp + 3 * hfc.esc_temp) * 0.25f;
                }
            }

            // debug_print("Iesc %5.1f Iaux %4.1f Vmain %5.2f Vesc %5.2f Vaux %5.2f RPM %4.0f Temp %4.1f\n",
            //                  hfc->power.Iesc, hfc->power.Iaux, hfc->power.Vmain, hfc->power.Vesc, hfc->power.Vaux, hfc->RPM, hfc->esc_temp);

            UpdateBatteryStatus(delta_us * 0.000001f);
        }
        else if (hfc.linklive_item > 0) { // keep incrementing if already initialized
            hfc.linklive_item++;
        }

        if (hfc.linklive_item == 2) {
            hfc.linklive_calib = 1000.0f / Max(delta, 500 * 96);
            // debug_print("Calib %d\n", delta);
        }

        if ((hfc.linklive_item > 2) && (hfc.linklive_item < 13)) {
            hfc.linklive_values[hfc.linklive_item] = ((delta * hfc.linklive_calib) - 500.0f) * LINKLIVE_SCALES[hfc.linklive_item];
            // debug_print("%02d %4d %4.2f\n", hfc->linklive_item, delta, hfc->linklive_values[hfc->linklive_item]);
        }
    }
}

static const char LINE_LABEL[8] = {'T', 'R', 'P', 'Y', 'C', ' ', ' ', ' '};
static const unsigned char CTRL_MODE2Idx[7] = {RAW, RAW, RATE, ANGLE, SPEED, SPEED, POS};

static void Display_CtrlMode(unsigned char line, unsigned char channel, const int ctrl_inh[5], unsigned char ctrl_modes[5], float ctrl_out[NUM_CTRL_MODES][5], float throttle)
{
    unsigned char ctrl_mode = ctrl_modes[channel];
    float ctrl_value = ctrl_out[CTRL_MODE2Idx[ctrl_mode]][channel];
    char str[20];
    char *pstr;

    if (channel==THRO)
        ctrl_value = throttle;
        
    str[0] = LINE_LABEL[line];
    pstr = str+1;
    if (ctrl_inh[channel])
    {
        pstr+= PRINTs(pstr, (char*)" INH   ");
        pstr+= PRINTf(pstr, ctrl_value, 5, 3, 1);
    } else if (ctrl_mode==CTRL_MODE_MANUAL)
    {
        pstr+= PRINTs(pstr, (char*)" MAN   ");
        pstr+= PRINTf(pstr, ctrl_value, 5, 3, 1);
    } else if (ctrl_mode==CTRL_MODE_RATE)
    {
        pstr+= PRINTs(pstr, (char*)" RATE  ");
        pstr+= PRINTf(pstr, ctrl_value, 1, 1, 1);
    } else if (ctrl_mode==CTRL_MODE_ANGLE)
    {
        pstr+= PRINTs(pstr, (char*)" ANGLE ");
        pstr+= PRINTf(pstr, ctrl_value, 1, 1, 1);
    } else if (ctrl_mode==CTRL_MODE_SPEED)
    {
        pstr+= PRINTs(pstr, (char*)" SPEED ");
        pstr+= PRINTf(pstr, ctrl_value, 1, 1, 1);
    } else if (ctrl_mode==CTRL_MODE_POSITION)
    {
        pstr+= PRINTs(pstr, (char*)" POS   ");
        pstr+= PRINTf(pstr, ctrl_value, 1, 1, 1);
    }
    myLcd.SetLine(channel, str, 0);
}

static void SetControlMode(void)
{
    if (xbus.valuesf[XBUS_CTRLMODE_SW]>0.5f)
        hfc.full_auto = true;
    else
        hfc.full_auto = false;

    /* in full auto mode, ignore all switches, keep storing stick values inc throttle, auto throttle */
    if (hfc.full_auto)
    {
    	telem.SaveValuesForAbort();
    	hfc.auto_throttle = true;
    	return;
    }

    /* this needs to be after full_auto check otherwise takeoff cannot be aborted */
    if (hfc.inhibitRCswitches)
        return;

    /* in non-RCradio mode, check for stick movement to abort */
    if (hfc.ctrl_source!=CTRL_SOURCE_RCRADIO)
    {
        char abort = 0;
		if (ABS(hfc.ctrl_initial[PITCH] - xbus.valuesf[XBUS_PITCH]) > AUTO_PROF_TERMINATE_THRS)
			abort = 1;
		if (ABS(hfc.ctrl_initial[ROLL]  - xbus.valuesf[XBUS_ROLL])  > AUTO_PROF_TERMINATE_THRS)
			abort = 1;
		if (hfc.ctrl_source==CTRL_SOURCE_AUTO3D || hfc.ctrl_source==CTRL_SOURCE_JOYSTICK)
		{
			if (ABS(hfc.ctrl_initial[COLL]  - xbus.valuesf[XBUS_THRO])  > AUTO_PROF_TERMINATE_THRS)
				abort = 1;
			if (ABS(hfc.ctrl_initial[YAW]   - xbus.valuesf[XBUS_YAW])   > AUTO_PROF_TERMINATE_THRS)
				abort = 1;
		}
        
        if (abort)
        {
        	if (hfc.playlist_status==PLAYLIST_PLAYING)
        	{
        		telem.PlaylistSaveState();
        		telem.SelectCtrlSource(CTRL_SOURCE_RCRADIO);
        		hfc.playlist_status = PLAYLIST_PAUSED;
        	}
        	else
        	{
        		if (hfc.playlist_status == PLAYLIST_PAUSED)
        		{
        			telem.SelectCtrlSource(CTRL_SOURCE_RCRADIO);
            		hfc.playlist_status = PLAYLIST_PAUSED;
        		}
        		else
        			telem.SelectCtrlSource(CTRL_SOURCE_RCRADIO);
        	}
        }
    }

    /* if auto throttle in auto mode, switch back to manual throttle on a throttle lever move */
    if (hfc.auto_throttle)
    {
    	if (ABS(hfc.ctrl_initial[THRO] - xbus.valuesf[XBUS_THR_LV]) > AUTO_PROF_TERMINATE_THRS)
    		hfc.auto_throttle = false;
    }

    /* collective mode, only in RCradio or auto2D modes */
    if (hfc.ctrl_source==CTRL_SOURCE_RCRADIO || hfc.ctrl_source==CTRL_SOURCE_AUTO2D)
    {
        if (!pConfig->ctrl_mode_inhibit[COLL])
        {
            if (xbus.valuesf[XBUS_THR_SW]>0.5f)
                hfc.control_mode[COLL] = CTRL_MODE_MANUAL;
            else if (xbus.valuesf[XBUS_THR_SW]<-0.5f)
                hfc.control_mode[COLL] = CTRL_MODE_POSITION;
            else
                hfc.control_mode[COLL] = CTRL_MODE_SPEED;
        }
    }

    /* pitch/rate/yaw mode switches checked only in RCradio mode */
    if (hfc.ctrl_source==CTRL_SOURCE_RCRADIO)
    {
    	int mode;

    	if (xbus.valuesf[XBUS_MODE_SW] < -0.5f) {
    		mode = CTRL_MODE_ANGLE;
    	}
		else if (xbus.valuesf[XBUS_MODE_SW] > 0.5f) {
    		mode = CTRL_MODE_MANUAL;
		}
		else {
    		mode = CTRL_MODE_RATE;
		}

    	mode += pConfig->RCmodeSwitchOfs;	// shift the mode up
    	mode = min(mode, CTRL_MODE_POSITION);

		SetCtrlMode(&hfc, pConfig, PITCH, mode);
		SetCtrlMode(&hfc, pConfig, ROLL,  mode);
		SetCtrlMode(&hfc, pConfig, YAW,   ClipMinMax(mode, pConfig->YawModeMin, pConfig->YawModeMax));

		/* during profiling, force the given mode */
	    if (hfc.profile_mode == PROFILING_ON)
	        SetCtrlMode(&hfc, pConfig, hfc.profile_ctrl_variable, hfc.profile_ctrl_level+1);
    }
}

/* CCPM 120deg mixer ==========================================================
** input: raw trottle, pitch and roll values
** output: servo signals A (at 6 o'clock) B (at 2 o'clock) and C (at 10 o'clock)
** value range: -/+ 0.571 corresponds to 100% stick */
static void CCPM120mix(float pitch, float roll, float collective, float *sA, float *sB, float *sC)
{
    /* collective */
    float A = collective;
    float B = collective;
    float C = collective;
//    float R;
    
    /* limit pitch roll such they together never exceed 100% */
#if 0    // not supported since the limits have to go back to PID min/max
    R = sqrtf(pitch*pitch + roll*roll);
    if (R>0.571f)
    {
        pitch = pitch/R*0.571f;
        roll  = roll/R*0.571f;
    }
#endif
    
    /* roll */
    B -= roll*0.8660254038f;//cos(30);
    C += roll*0.8660254038f;//cos(30);
    
    /* pitch */
    A += pitch;
    B -= pitch*0.5f;//sin(30);
    C -= pitch*0.5f;//sin(30);
    
    *sA = A;
    *sB = B;
    *sC = C;
}

/* CCPM 140deg mixer ==========================================================
** input: raw trottle, pitch and roll values
** output: servo signals A (at 6 o'clock) B (at 2 o'clock) and C (at 10 o'clock)
** value range: -/+ 0.571 corresponds to 100% stick */
static void CCPM140mix(float pitch, float roll, float collective, float *sA, float *sB, float *sC)
{
    /* collective */
    float A = collective;
    float B = collective;
    float C = collective;
//    float R;

    /* roll */
    B -= roll*0.8660254038f;//cos(30);
    C += roll*0.8660254038f;//cos(30);

    /* pitch */
    A += pitch;
    B -= pitch;
    C -= pitch;

    *sA = A;
    *sB = B;
    *sC = C;
}
/* Tandem Helicopter Mixer ====================================================
 ** input: collective, pitch and roll values
 ** input: XBUS channel 6 and 7 raw values
 ** output: servo signals
 ** value range: -/+ 0.571 corresponds to 100% stick
 */
static void MixerTandem(ServoNodeOutputs *servo_node_pwm)
{
    float elevGain, dcpGain;
    float ROLL_Taileron;
    float PITCH_Televator, PITCH_TelevatorR;
    float YAW_Trudder;
    float dcp, torqComp;
    float TcollectFront,TcollectRear;
    float ROLL_TaileronFront, ROLL_TaileronRear;

    // gain calculation 0 to .571 maximum.  Todo could x 1.7512f for full 0 to 1 range
    // gain is set usually by digital levers on the transmitter for in flight adjustment
    elevGain = abs(xbus.valuesf[ELEVGAIN]);         // use only positive half
    dcpGain  = abs(xbus.valuesf[DCPGAIN]);          // set transmitter correctly

    ROLL_Taileron  = hfc.mixer_in[ROLL]  * pConfig->AilRange;
    PITCH_Televator = hfc.mixer_in[PITCH] * pConfig->EleRange * elevGain;
    PITCH_TelevatorR = PITCH_Televator * pConfig->swashTiltRear;
    YAW_Trudder   = hfc.mixer_in[YAW]   * pConfig->RudRange;

    // Differential collective pitch calculation
    // Torque compensation due to dcp and elevator
    dcp      = PITCH_Televator * dcpGain;
    torqComp = dcp * pConfig->TorqCompMult;

    TcollectFront = hfc.mixer_in[COLL] * pConfig->CollRange - dcp * pConfig->dcpFront;
    TcollectRear  = hfc.mixer_in[COLL] * pConfig->CollRange + dcp * pConfig->dcpRear;

    ROLL_TaileronFront = ROLL_Taileron + YAW_Trudder + torqComp;
    ROLL_TaileronRear  = ROLL_Taileron - YAW_Trudder - torqComp;

    //         Servo map on helicopter
    //    Front(Node 1)               Rear(Node 2)
    //   B(3)                             B(3)
    //       C(4)   <---------------  C(4)
    //   A(2)                             A(2)
    // () servo channel output
    //
    // Throttle(1)
    // CcpmMixer value of 0.5 multiplier is used for 120deg CCPM
    // CcpmMixer value of 1.0 multiplier is used for 140deg CCPM
    if (pConfig->ModelSelect == 1) {
        // Selection for E6T
        // Front Servo
        servo_node_pwm[1].servo_out[1] = 0 - ROLL_TaileronFront + (pConfig->CcpmMixer * PITCH_Televator) - TcollectFront;   // aFrontServo
        servo_node_pwm[1].servo_out[2] = 0 - ROLL_TaileronFront - (pConfig->CcpmMixer * PITCH_Televator) + TcollectFront;   // bFrontServo
        servo_node_pwm[1].servo_out[3] = 0 + TcollectFront + PITCH_Televator;                                                 // cFrontServo

        // Rear Servo
        servo_node_pwm[2].servo_out[1] = 0 - TcollectRear  + (pConfig->CcpmMixer * PITCH_TelevatorR) + ROLL_TaileronRear;    // aRearServo
        servo_node_pwm[2].servo_out[2] = 0 + TcollectRear  - (pConfig->CcpmMixer * PITCH_TelevatorR) + ROLL_TaileronRear;    // bRearServo
        servo_node_pwm[2].servo_out[3] = 0 + TcollectRear  + PITCH_TelevatorR;

    }
    else {
        // Selection for 210T
        // Front Servo
        servo_node_pwm[1].servo_out[1] = 0 + ROLL_TaileronFront - (pConfig->CcpmMixer * PITCH_Televator) + TcollectFront;   // aFrontServo
        servo_node_pwm[1].servo_out[2] = 0 + ROLL_TaileronFront + (pConfig->CcpmMixer * PITCH_Televator) - TcollectFront;   // bFrontServo
        servo_node_pwm[1].servo_out[3] = 0 + TcollectFront + PITCH_Televator;                                                 // cFrontServo

        // Rear Servo
        servo_node_pwm[2].servo_out[1] = 0 + TcollectRear  - (pConfig->CcpmMixer * PITCH_Televator) - ROLL_TaileronRear;    // aRearServo
        servo_node_pwm[2].servo_out[2] = 0 - TcollectRear  + (pConfig->CcpmMixer * PITCH_Televator) - ROLL_TaileronRear;    // bRearServo
        servo_node_pwm[2].servo_out[3] = 0 - TcollectRear  - PITCH_Televator;                                                 // cRearServo
    }
}

/* QUAD Copter Mixer ==========================================================
 * Quad in X configuration.
 *
 *  1CW       0CCW
 *     \  ^  /
 *     /  |  \
 * 2CCW       3CW
 *
 * T, R, P, Y controls */
static void MixerQuad()
{
    hfc.servos_out[0] = hfc.mixer_in[THRO] * pConfig->throttle_gain;
    hfc.servos_out[1] = hfc.mixer_in[THRO] * pConfig->throttle_gain;
    hfc.servos_out[2] = hfc.mixer_in[THRO] * pConfig->throttle_gain;
    hfc.servos_out[6] = 0;
    hfc.servos_out[4] = hfc.mixer_in[THRO] * pConfig->throttle_gain;
    hfc.servos_out[7] = 0;

	if(hfc.throttle_value > 0.5 && hfc.throttle_armed)	// Geoff's LEDs
	    hfc.servos_out[3] = 1;
	else
	    hfc.servos_out[3] = -1;

    if (hfc.throttle_armed)								// Another one of Geoff's LEDs
        hfc.servos_out[5] = 1;
    else
        hfc.servos_out[5] = -1;


    hfc.servos_out[0] -= hfc.mixer_in[ROLL]*0.5f;
    hfc.servos_out[1] += hfc.mixer_in[ROLL]*0.5f;
    hfc.servos_out[2] += hfc.mixer_in[ROLL]*0.5f;
    hfc.servos_out[4] -= hfc.mixer_in[ROLL]*0.5f;

    hfc.servos_out[0] -= hfc.mixer_in[PITCH]*0.5f;
    hfc.servos_out[1] -= hfc.mixer_in[PITCH]*0.5f;
    hfc.servos_out[2] += hfc.mixer_in[PITCH]*0.5f;
    hfc.servos_out[4] += hfc.mixer_in[PITCH]*0.5f;

    hfc.servos_out[0] -= hfc.mixer_in[YAW];
    hfc.servos_out[1] += hfc.mixer_in[YAW];			//cw
    hfc.servos_out[2] -= hfc.mixer_in[YAW];
    hfc.servos_out[4] += hfc.mixer_in[YAW];			//cw

//    if ((hfc.print_counter&0x7ff)==2)
//        debug_print("%d %d %d %d\n", (int)(servos_out[0]*1000),(int)(servos_out[1]*1000),(int)(servos_out[2]*1000),(int)(servos_out[3]*1000));


    //Keep motors from turning off once armed
    if ((hfc.throttle_armed && hfc.throttle_value>-0.5f) || hfc.waypoint_stage == FM_TAKEOFF_AUTO_SPOOL)
    {
        int i;
        for (i=0; i<8; i++){
    	if(hfc.servos_out[i] < pConfig->throttle_multi_min)
    	    hfc.servos_out[i] = pConfig->throttle_multi_min;
        }
    }
}
/* Hex Copter Mixer ==========================================================
 * Hex layout as shown
 *
 *  2CCW     0CW
 * 		\	/
 *  3CW-- | --1CCW
 *  	/	\
 *	5CCW     4CW
 *
 * T, R, P, Y - controls */
static void MixerHex(void)
{
    hfc.servos_out[0] = hfc.mixer_in[THRO]* pConfig->throttle_gain;
    hfc.servos_out[1] = hfc.mixer_in[THRO]* pConfig->throttle_gain;
    hfc.servos_out[2] = hfc.mixer_in[THRO]* pConfig->throttle_gain;
    hfc.servos_out[3] = hfc.mixer_in[THRO]* pConfig->throttle_gain;
    hfc.servos_out[4] = hfc.mixer_in[THRO]* pConfig->throttle_gain;
    hfc.servos_out[5] = hfc.mixer_in[THRO]* pConfig->throttle_gain;
    hfc.servos_out[6] = hfc.mixer_in[THRO]* pConfig->throttle_gain;
    hfc.servos_out[7] = hfc.mixer_in[THRO]* pConfig->throttle_gain;

    hfc.servos_out[4] -= hfc.mixer_in[ROLL]*0.5f;
    hfc.servos_out[1] -= hfc.mixer_in[ROLL];
    hfc.servos_out[0] -= hfc.mixer_in[ROLL]*0.5f;
    hfc.servos_out[2] += hfc.mixer_in[ROLL]*0.5f;
    hfc.servos_out[3] += hfc.mixer_in[ROLL];
    hfc.servos_out[5] += hfc.mixer_in[ROLL]*0.5f;
    
    hfc.servos_out[0] -= hfc.mixer_in[PITCH]*0.866f;
    hfc.servos_out[2] -= hfc.mixer_in[PITCH]*0.866f;
    hfc.servos_out[4] += hfc.mixer_in[PITCH]*0.866f;
    hfc.servos_out[5] += hfc.mixer_in[PITCH]*0.866f;
    
    hfc.servos_out[4] += hfc.mixer_in[YAW];		//cw
    hfc.servos_out[3] += hfc.mixer_in[YAW];		//cw
    hfc.servos_out[0] += hfc.mixer_in[YAW];		//cw
    hfc.servos_out[1] -= hfc.mixer_in[YAW];
    hfc.servos_out[2] -= hfc.mixer_in[YAW];
    hfc.servos_out[5] -= hfc.mixer_in[YAW];

    //Keep motors from turning off once armed
    if ((hfc.throttle_armed && hfc.throttle_value>-0.5f) || hfc.waypoint_stage == FM_TAKEOFF_AUTO_SPOOL)
    {
        int i;
        for (i=0; i<8; i++){
    	if(hfc.servos_out[i] < pConfig->throttle_multi_min)
    	    hfc.servos_out[i] = pConfig->throttle_multi_min;
        }
    }
}

/* OCTO Copter Mixer ==========================================================
 * Octo in X configuration.  Servo[6] is location S4, Servo[7] is location S5
 * on the output of the FCM.
 * Other servos are output on the Servo Node
 *
 *    7CCW     0CW
 *      \   ^  /
 *  6CW     |     1CCW
 *     \-   |   -/
 *     /-       -\
 * 5CCW           2CW
 *      /       \
 *    4CW     3CCW
 *
 * T, R, P, Y controls */
static void MixerOcto(void)
{
    hfc.servos_out[0] = hfc.mixer_in[THRO]* pConfig->throttle_gain;
    hfc.servos_out[1] = hfc.mixer_in[THRO]* pConfig->throttle_gain;
    hfc.servos_out[2] = hfc.mixer_in[THRO]* pConfig->throttle_gain;
    hfc.servos_out[3] = hfc.mixer_in[THRO]* pConfig->throttle_gain;
    hfc.servos_out[4] = hfc.mixer_in[THRO]* pConfig->throttle_gain;
    hfc.servos_out[5] = hfc.mixer_in[THRO]* pConfig->throttle_gain;
    hfc.servos_out[6] = hfc.mixer_in[THRO]* pConfig->throttle_gain;
    hfc.servos_out[7] = hfc.mixer_in[THRO]* pConfig->throttle_gain;

    hfc.servos_out[0] += hfc.mixer_in[PITCH];
    hfc.servos_out[1] += hfc.mixer_in[PITCH] * 0.414174f;
    hfc.servos_out[2] -= hfc.mixer_in[PITCH] * 0.414174f;
    hfc.servos_out[3] -= hfc.mixer_in[PITCH];
    hfc.servos_out[4] -= hfc.mixer_in[PITCH];
    hfc.servos_out[5] -= hfc.mixer_in[PITCH] * 0.414174f;
    hfc.servos_out[6] += hfc.mixer_in[PITCH] * 0.414174f;
    hfc.servos_out[7] += hfc.mixer_in[PITCH];

    hfc.servos_out[0] -= hfc.mixer_in[ROLL] * 0.414174f;
    hfc.servos_out[1] -= hfc.mixer_in[ROLL];
    hfc.servos_out[2] -= hfc.mixer_in[ROLL];
    hfc.servos_out[3] -= hfc.mixer_in[ROLL] * 0.414174f;
    hfc.servos_out[4] += hfc.mixer_in[ROLL] * 0.414174f;
    hfc.servos_out[5] += hfc.mixer_in[ROLL];
    hfc.servos_out[6] += hfc.mixer_in[ROLL];
    hfc.servos_out[7] += hfc.mixer_in[ROLL] * 0.414174f;

    hfc.servos_out[0] += hfc.mixer_in[YAW];			//cw
    hfc.servos_out[1] -= hfc.mixer_in[YAW];
    hfc.servos_out[2] += hfc.mixer_in[YAW];			//cw
    hfc.servos_out[3] -= hfc.mixer_in[YAW];
    hfc.servos_out[4] += hfc.mixer_in[YAW];			//cw
    hfc.servos_out[5] -= hfc.mixer_in[YAW];
    hfc.servos_out[6] += hfc.mixer_in[YAW];			//cw
    hfc.servos_out[7] -= hfc.mixer_in[YAW];

    //Keep motors from turning off once armed
    if ((hfc.throttle_armed && hfc.throttle_value>-0.5f) || hfc.waypoint_stage == FM_TAKEOFF_AUTO_SPOOL)
    {
        int i;
        for (i=0; i<8; i++){
    	if(hfc.servos_out[i] < pConfig->throttle_multi_min)
    	    hfc.servos_out[i] = pConfig->throttle_multi_min;
        }
    }
}

static void ServoMixer(void)
{
    if (pConfig->ccpm_type == CCPM_NONE)
    {
        hfc.servos_out[THRO]  = hfc.mixer_in[THRO];
        hfc.servos_out[PITCH] = hfc.mixer_in[PITCH];
        hfc.servos_out[ROLL]  = hfc.mixer_in[ROLL];
        hfc.servos_out[YAW]   = hfc.mixer_in[YAW];
        hfc.servos_out[COLL]  = 0;
    }
    else if (pConfig->ccpm_type == CCPM_120)
    {
        hfc.servos_out[YAW]  = hfc.mixer_in[YAW];
        hfc.servos_out[THRO] = hfc.mixer_in[THRO];
        CCPM120mix(hfc.mixer_in[PITCH], hfc.mixer_in[ROLL], hfc.mixer_in[COLL],
                    &hfc.servos_out[CCPM_A], &hfc.servos_out[CCPM_B], &hfc.servos_out[CCPM_C]);
    }
    else if (pConfig->ccpm_type == CCPM_140)
    {
        hfc.servos_out[YAW] = hfc.mixer_in[YAW];
        hfc.servos_out[THRO] = hfc.mixer_in[THRO];
        CCPM140mix(hfc.mixer_in[PITCH], hfc.mixer_in[ROLL], hfc.mixer_in[COLL],
                        &hfc.servos_out[CCPM_A], &hfc.servos_out[CCPM_B], &hfc.servos_out[CCPM_C]);
    }
    else if (pConfig->ccpm_type == CCPM_HEX)
    {
        MixerHex();
    }
    else if (pConfig->ccpm_type==CCPM_QUAD)
    {
        MixerQuad();
    }
    else if (pConfig->ccpm_type==CCPM_OCTO)
    {
        MixerOcto();
    }
    else if (pConfig->ccpm_type == MIXERTANDEM)
    {
        servo_node_pwm[1].servo_out[0] = hfc.mixer_in[THRO];                        // Link Live Throttle on Front Servo output 0
        servo_node_pwm[2].servo_out[0] = hfc.mixer_in[THRO] + pConfig->RearRpmTrim; // Link Live Throttle on Rear Servo output 0
        MixerTandem(&servo_node_pwm[0]);
    }
}

static inline void ProcessStickInputs(FlightControlData *hfc, float dT)
{
    int channel;
    
    /* process P, R, Y, C */
    for (channel=0; channel<4; channel++)
    {
        unsigned char mode = hfc->control_mode[channel];
        if (mode>0)
        {
            float db = hfc->StickDeadband[channel];
            /* no deadband for manual collective */
            if (channel==3 && mode==CTRL_MODE_MANUAL)
            	db = 0;
            if (db)
            {
                float v = hfc->ctrl_out[RAW][channel];
                if (v<0)
                {
                    if (v<=-db)
                        v +=db;
                    else
                        v = 0;
                }
                else
                {
                    if (v>=db)
                        v -=db;
                    else
                        v = 0;
                }
                hfc->ctrl_out[RAW][channel] = v;
            }
        }
    }
}

void HeadingUpdate(float heading_rate, float dT)
{
    hfc.ctrl_out[ANGLE][YAW] += heading_rate*dT;

    if (hfc.ctrl_out[ANGLE][YAW]>180) {
      hfc.ctrl_out[ANGLE][YAW]-=360;
    }
    else if (hfc.ctrl_out[ANGLE][YAW]<-180) {
      hfc.ctrl_out[ANGLE][YAW]+=360;
    }
}

void AltitudeUpdate(float alt_rate, float dT)
{
    hfc.ctrl_out[POS][COLL] += alt_rate*dT;

    if (hfc.ctrl_out[POS][COLL] > 7000) {
        hfc.ctrl_out[POS][COLL] = 7000;
    }
    else if (hfc.ctrl_out[POS][COLL] < 0) {
        hfc.ctrl_out[POS][COLL] = 0;
    }
}

static const char CTRL_MODES[7] = {'-', 'M', 'R', 'A', 'S', 'G', 'P'};

static char GetModeChar(FlightControlData *hfc, byte channel)
{
    if (pConfig->ctrl_mode_inhibit[channel])
        return CTRL_MODES[0];
    else
        return CTRL_MODES[hfc->control_mode[channel]];
}

static void Display_Process(FlightControlData *hfc, char xbus_new_values, float dT)
{
    char str[40];

    if(hfc->display_mode == DISPLAY_SPLASH)
    {
        if ((hfc->print_counter&0xff)==2)
        {
            myLcd.ShowSplash(AVIDRONE_SPLASH, AVIDRONE_FCM_SPLASH, FCM_VERSION);

            if (init_ok) {
                if (init_warning == 0) {
                    PRINTs(str, (char*)"");
                    myLcd.SetLine(3, str, 0);

                    PRINTs(str, (char*)"");
                    myLcd.SetLine(4, str, 0);
                }

                if(!hfc->throttle_armed) {
                    PRINTs(str, (char*)"NEXT       ARM");
                    myLcd.SetLine(5, str, 0);
                }
                else {
                    PRINTs(str, (char*)"NEXT    DISARM");
                    myLcd.SetLine(5, str, 0);
                }
            }
        }
    }
    else if (hfc->display_mode == DISPLAY_STATUS)
    {
        //GpsData gps_data = gps.GetGpsData();

        if ((hfc->print_counter&0xff)==2)
        {
            // Loop    us
            sPRINTdd(str, (char*)"Loop %duS %d%%", hfc->ticks_max/*(hfc->ticks_lp+32)>>6*/, (int)hfc->cpu_utilization_lp);
            myLcd.SetLine(0, str, 0);
            // GPS     hdop
            if (gps.gps_data_.fix>0)
                sPRINTfd(str, (char*)"GPS  %4.2f / %d", gps.gps_data_.PDOP*0.01f, gps.gps_data_.sats);
            else
                PRINTs(str, (char*)"GPS  ----");
            myLcd.SetLine(1, str, 0);
            // Bat     %
            sPRINTdf(str, (char*)"Bat  %d%% %4.2fV", (int)hfc->power.battery_level, hfc->power.Vmain/pConfig->battery_cells);
            myLcd.SetLine(2, str, 0);
            // Xbus
            if(pConfig->SbusEnable == 0)
            {
            if (!xbus.receiving)
                PRINTs(str, (char*)"Xbus ----");
            else if (hfc->full_auto)
                PRINTs(str, (char*)"Xbus FullAuto");
            else
                PRINTs(str, (char*)"Xbus Good");
            }
            if(pConfig->SbusEnable == 1)
            {
			if (!xbus.receiving)
				PRINTs(str, (char*)"Sbus ----");
			else if (hfc->full_auto)
				PRINTs(str, (char*)"Sbus FullAuto");
			else
				PRINTs(str, (char*)"Sbus Good");
            }
            myLcd.SetLine(3, str, 0);
            // Mode    PRYCT (m/r/a/s/p)
            PRINTs(str, (char*)"Mode -/-/-/-/-");
            str[5] = GetModeChar(hfc, PITCH);
            str[7] = GetModeChar(hfc, ROLL);
            str[9] = GetModeChar(hfc, YAW);
            str[11]= GetModeChar(hfc, COLL);
            str[13]= GetModeChar(hfc, THRO);
            myLcd.SetLine(4, str, 0);
        }
    }
    else if (hfc->display_mode == DISPLAY_POWER)
    {
        if ((hfc->print_counter&0xff)==2)
        {
            sPRINTfd(str, (char*)"BAT %4.2fV %dC", hfc->power.Vmain, pConfig->battery_cells);
            myLcd.SetLine(0, str, 0);
            sPRINTff(str, (char*)"ESC %3.1fV %3.1fA", hfc->power.Vesc, hfc->power.Iesc);
            myLcd.SetLine(1, str, 0);
            sPRINTff(str, (char*)"S/A %3.1fV %3.1fV", hfc->power.Vservo, hfc->power.Vaux);
            myLcd.SetLine(2, str, 0);
            sPRINTff(str, (char*)"CAP %3.1f %3.1fAh", hfc->power.capacity_used/3600.0f, hfc->power.capacity_total/3600.0f);
            myLcd.SetLine(3, str, 0);
            sPRINTddd(str, (char*)"FT  %02d:%02dS %d%%", hfc->power.flight_time_left/60, hfc->power.flight_time_left%60, (int)hfc->power.battery_level);
            myLcd.SetLine(4, str, 0);
        }
    }
    else if (hfc->display_mode == DISPLAY_CONTROLS)
    {
        if (xbus_new_values)
        {
            float thr1 = (hfc->ctrl_out[RAW][THRO] + pConfig->Stick100range)*0.5f;
            Display_CtrlMode(0, THRO,  pConfig->ctrl_mode_inhibit, hfc->control_mode, hfc->ctrl_out, thr1);   // throttle
            Display_CtrlMode(1, ROLL,  pConfig->ctrl_mode_inhibit, hfc->control_mode, hfc->ctrl_out, thr1);   // roll
            Display_CtrlMode(2, PITCH, pConfig->ctrl_mode_inhibit, hfc->control_mode, hfc->ctrl_out, thr1);   // pitch
            Display_CtrlMode(3, YAW,   pConfig->ctrl_mode_inhibit, hfc->control_mode, hfc->ctrl_out, thr1);   // yaw
            Display_CtrlMode(4, COLL,  pConfig->ctrl_mode_inhibit, hfc->control_mode, hfc->ctrl_out, thr1);   // collective
        }
    }
    else if (hfc->display_mode == DISPLAY_XBUS)
    {
        if (xbus_new_values)
        {
        	sPRINTdd(str, (char*)"1:%4d  2:%4d", (int)(xbus.valuesf[0]*1000+0.5),(int)(xbus.valuesf[1]*1000+0.5));
            myLcd.SetLine(0, str, 0);
            sPRINTdd(str, (char*)"3:%4d  4:%4d", (int)(xbus.valuesf[2]*1000+0.5),(int)(xbus.valuesf[3]*1000+0.5));
            myLcd.SetLine(1, str, 0);
            sPRINTdd(str, (char*)"5:%4d  6:%4d", (int)(xbus.valuesf[4]*1000+0.5),(int)(xbus.valuesf[5]*1000+0.5));
            myLcd.SetLine(2, str, 0);
            sPRINTdd(str, (char*)"7:%4d  8:%4d", (int)(xbus.valuesf[6]*1000+0.5),(int)(xbus.valuesf[7]*1000+0.5));
            myLcd.SetLine(3, str, 0);
          	sPRINTdd(str, (char*)"XBUS %d/%d", xbus.good_packets, xbus.bad_packets);
           	myLcd.SetLineX(0, 4, str, 0);
        }
    }
    else if (hfc->display_mode == DISPLAY_GPS1)
    {
        //GpsData gps_data = gps.GetGpsData();

//        if (hfc->gps_new_data)
        if ((hfc->print_counter&0x7f)==2)
        {
        	sPRINTf(str, (char*)"LAT %+9.6f", gps.gps_data_.latF);
			myLcd.SetLine(0, str, 0);
			sPRINTf(str, (char*)"LON %+9.6f", gps.gps_data_.lonF);
			myLcd.SetLine(1, str, 0);
			sPRINTf(str, (char*)"ALT %+4.2fm", gps.gps_data_.altitude);
			myLcd.SetLine(2, str, 0);
			sPRINTdf(str, (char*)"S/D %d/%4.2f", gps.gps_data_.sats, gps.gps_data_.PDOP*0.01f);
			myLcd.SetLine(3, str, 0);
			sPRINTddd(str, (char*)"F/E/C %d/%d/%d", gps.gps_data_.fix, gps.glitches_, gps.selected_channel_);
			myLcd.SetLine(4, str, 0);
        }
    }
    else if (hfc->display_mode == DISPLAY_GPS2)
    {
//        if (hfc->gps_new_data)
        if ((hfc->print_counter&0x7f)==2)
        {
          float distance = hfc->gps_to_home[0];
          float course   = hfc->gps_to_home[1];
          float dAlt     = hfc->gps_to_home[2];
          
          sPRINTf(str, (char*)"SPEED %3.1fm/s", hfc->gps_speed);
          myLcd.SetLine(0, str, 0);
          sPRINTf(str, (char*)"HEAD %+3.1fdeg", hfc->gps_heading);
          myLcd.SetLine(1, str, 0);
          sPRINTf(str, (char*)"DIST  %3.1fm", distance);
          myLcd.SetLine(2, str, 0);
          sPRINTf(str, (char*)"dALT %+4.2fm", dAlt);
          myLcd.SetLine(3, str, 0);
          sPRINTf(str, (char*)"COUR %+3.1fdeg", course);
          myLcd.SetLine(4, str, 0);
        }
    }
    else if (hfc->display_mode == DISPLAY_GPS3)
    {
        //GpsData gps_data = gps.GetGpsData();

        if ((hfc->print_counter&0xff)==2)
        {
			char fix = gps.gps_data_.fix;
			unsigned long date = gps.gps_data_.date;
			unsigned long time = gps.gps_data_.time/100;
            
            if (fix==GPS_FIX_OK)
                PRINTs(str, (char*)"FIX  OK");
            else
            if (fix==GPS_FIX_DIFF)
                PRINTs(str, (char*)"FIX  DIFF");
            else
                PRINTs(str, (char*)"FIX  NONE");
            myLcd.SetLine(0, str, 0);
            sPRINTd(str, (char*)"DATE %06d", (int)date);
            myLcd.SetLine(1, str, 0);
            sPRINTd(str, (char*)"TIME %06d", (int)time);
            myLcd.SetLine(2, str, 0);
            sPRINTd(str, (char*)"ERRs %d", (int)gps.glitches_);
            myLcd.SetLine(3, str, 0);
            sPRINTd(str, (char*)"CURR %d", (int)gps.selected_channel_);
            myLcd.SetLine(4, str, 0);
        }
    }
    else if (hfc->display_mode == DISPLAY_BARO)
    {
        if ((hfc->print_counter&0xff)==2)
        {
            float dAlt = 0;
            sPRINTd(str, (char*)"BaroP %dPa", (int)hfc->baro_pressure);
            myLcd.SetLine(0, str, 0);
            sPRINTf(str, (char*)"Temp   %+3.1fdeg", hfc->baro_temperature);
            myLcd.SetLine(1, str, 0);
            sPRINTf(str, (char*)"AltB   %+3.1fm", hfc->baro_altitude_raw);
            myLcd.SetLine(2, str, 0);
            sPRINTf(str, (char*)"AltIMU %+3.1fm", hfc->altitude);
//            sPRINTf(str, "VSPD %+3.1fd", hfc->baro_vspeed);
            myLcd.SetLine(3, str, 0);
            sPRINTf(str, (char*)"dALT   %+4.2fm", dAlt);
            myLcd.SetLine(4, str, 0);
        }
    }
    else if (hfc->display_mode == DISPLAY_COMPASS)
    {
        if ((hfc->print_counter&0xff)==2)
        {
            int compR = (int)compass.dataXYZcalib[0];
            int compF = (int)compass.dataXYZcalib[1];
            int compU = (int)compass.dataXYZcalib[2];
            myLcd.SetLine(0, (char*)"COMPASS", 0);
            if(hfc->comp_calibrate == COMP_CALIBRATING)
            {
                int i_pitch = -1;
                int i_roll = -1;
                for(int j = 0; j < ROLL_COMP_LIMIT; j++) {
                    if( (hfc->comp_pitch_flags[j] < NUM_ANGLE_POINTS)
                            && (j < PITCH_COMP_LIMIT)){
                        i_pitch = j;
                        break;
                    }
                    else if(hfc->comp_roll_flags[j] < NUM_ANGLE_POINTS) {
                        i_roll = j;
                        break;
                    }
                }

                if(i_pitch != -1) {
                    sPRINTd(str, (char*)"COMPASS P:%d",
                            (i_pitch - PITCH_COMP_LIMIT/2)*180/PITCH_COMP_LIMIT);
                    myLcd.SetLine(0, str, 0);
                    debug_print("check i_pitch = %d\r\n",
                            (i_pitch - PITCH_COMP_LIMIT/2)*180/PITCH_COMP_LIMIT);
                }
                else if(i_roll != -1 ) {
                    sPRINTd(str, (char*)"COMPASS R:%d",
                            (i_roll - ROLL_COMP_LIMIT/2)*360/ROLL_COMP_LIMIT);
                    myLcd.SetLine(0, str, 0);
                    debug_print("check i_roll = %d\r\n",
                            (i_roll - ROLL_COMP_LIMIT/2)*360/ROLL_COMP_LIMIT);
                }

                myLcd.SetLine(1, (char*)"CALIBRATING!  ", 0);
                compR = (int)compass.dataXYZ[0];
                compF = (int)compass.dataXYZ[1];
                compU = (int)compass.dataXYZ[2];
            }
            else if(hfc->comp_calibrate == COMP_CALIBRATE_DONE)
            {
                myLcd.SetLine(1, (char*)"DONE COMP CAL!", 0);
            }
            else
            {
                sPRINTf(str, (char*)"HEAD %+3.1fdeg  ", hfc->compass_heading_lp);
                myLcd.SetLine(1, str, 0);
            }
            sPRINTddd(str, (char*)"R%+4d %+4d%+4d", compR, hfc->compass_cal.compassMin[0], hfc->compass_cal.compassMax[0]);
            myLcd.SetLine(2, str, 0);
            sPRINTddd(str, (char*)"F%+4d %+4d%+4d", compF, hfc->compass_cal.compassMin[1], hfc->compass_cal.compassMax[1]);
            myLcd.SetLine(3, str, 0);
            sPRINTddd(str, (char*)"U%+4d %+4d%+4d", compU, hfc->compass_cal.compassMin[2], hfc->compass_cal.compassMax[2]);
            myLcd.SetLine(4, str, 0);
        }
    }
    else if (hfc->display_mode == DISPLAY_CALIB)
    {
        if ((hfc->print_counter&0xff)==2)
        {
            char *pstr = str;
            sPRINTf(str, (char*)"GYRO %+3.1fdeg", hfc->gyro_temp_lp);
            myLcd.SetLine(0, str, 0);
            pstr+= PRINTd(pstr, (int)(hfc->gyro_lp_disp[0]*1000), 3, 0, 1); *pstr++ = ' ';
            pstr+= PRINTd(pstr, (int)(hfc->gyro_lp_disp[1]*1000), 3, 0, 1); *pstr++ = ' ';
            pstr+= PRINTd(pstr, (int)(hfc->gyro_lp_disp[2]*1000), 3, 0, 1); *pstr++ = ' ';
            *pstr++ = ' ';
            *pstr++ = 0;
            str[14] = 0;
            myLcd.SetLine(1, str, 0);

            pstr = str;

            pstr+= PRINTd(pstr, (int)(hfc->gyroOfs[0]*1000), 3, 0, 1); *pstr++ = ' ';
            pstr+= PRINTd(pstr, (int)(hfc->gyroOfs[1]*1000), 3, 0, 1); *pstr++ = ' ';
            pstr+= PRINTd(pstr, (int)(hfc->gyroOfs[2]*1000), 3, 0, 1); *pstr++ = ' ';
            *pstr++ = ' ';
            *pstr++ = 0;
            str[14] = 0;
            myLcd.SetLine(2, str, 0);

            sPRINTf(str, (char*)"LIDAR %4.2fm", hfc->altitude_lidar);
            myLcd.SetLine(3, str, 0);
            sPRINTdd(str, (char*)"TIME  %02d:%02ds", (hfc->time_ms/1000)/60, (hfc->time_ms/1000)%60);
            myLcd.SetLine(4, str, 0);
        }
    }
    else if (hfc->display_mode == DISPLAY_WAYPOINT)
    {
        if ((hfc->print_counter&0x7f)==2)
        {
            float head = hfc->gps_to_waypoint[1] - hfc->IMUorient[2]*R2D;
            short int se = head*182.044444f+0.5f;
            head = se*0.0054931640625f;
            
            myLcd.SetLine(0, (char*)"PLAYLIST", 0);
            if (hfc->playlist_status==PLAYLIST_PLAYING)
                myLcd.SetLine(1, (char*)"PLAYING", 0);
            else if (hfc->playlist_status==PLAYLIST_PAUSED)
                myLcd.SetLine(1, (char*)"PAUSED", 0);
            else
                myLcd.SetLine(1, (char*)"STOPPED", 0);
            sPRINTdd(str, (char*)"%d / %d", hfc->playlist_position, hfc->playlist_items);
            myLcd.SetLine(2, str, 0);
            sPRINTf(str, (char*)"Dist: %5.1fm", hfc->gps_to_waypoint[0]);
            myLcd.SetLine(3, str, 0);
            sPRINTd(str, (char*)"Cour: %ddeg", (int)head);
            myLcd.SetLine(4, str, 0);
        }
    }
    else
    {
        if ((hfc->print_counter&0xff)==2)
        {
            myLcd.SetLine(0, (char*)"UNKNOWN", 0);
            myLcd.SetLine(1, NULL, 0);
            myLcd.SetLine(2, NULL, 0);
            myLcd.SetLine(3, NULL, 0);
            myLcd.SetLine(4, NULL, 0);
        }
    }    
}

/*static void CheckRangeAndSetD(double *pvalue, double value, double vmin, double vmax)
{
//  debug_print("%f\r\n", value);
    if (!pvalue || value>vmax || value<vmin)
        return;
    *pvalue = value;
}*/

static bool CheckRangeAndSetF(float *pvalue, float value, float vmin, float vmax)
{
//  debug_print("%f\r\n", value);
    if (!pvalue || value>vmax || value<vmin)
        return false;
    *pvalue = value;
    return true;
}

static void CheckRangeAndSetI(int *pvalue, int value, int vmin, int vmax)
{
    if (!pvalue || value>vmax || value<vmin)
        return;
    *pvalue = value;
}

static void CheckRangeAndSetB(byte *pvalue, int value, int vmin, int vmax)
{
    if (!pvalue || value>vmax || value<vmin)
        return;
    *pvalue = value;
}

static void Playlist_ProcessTop(FlightControlData *hfc)
{
    T_PlaylistItem *item;
    
    /* process only active playlist */
    if (hfc->playlist_status!=PLAYLIST_PLAYING)
        return;
        
    /* check for end of the playlist */
    if (hfc->playlist_position>=hfc->playlist_items)
    {
        /* stop playlist and waypoint mode - put into position hold */
        hfc->playlist_status = PLAYLIST_STOPPED;
        /* if in flight, put a waypoint at the current position, else do nothing */
        if (hfc->throttle_armed)
        	telem.SetPositionHold();
        return;
    }

    /* process current playlist item */
    /* only WP and PARAM are processed here, the rest is processed by Playlist_ProcessBottom() and it increments the playlist pointer */
    item = &hfc->playlist[hfc->playlist_position];
    
    if (item->type==PL_ITEM_WP)
    {
        if (item->data[0]==WAYPOINT_GOTO || item->data[0]==WAYPOINT_FLYTHROUGH)
        {
            if (!hfc->pl_wp_initialized)
            {
                telem.SetWaypoint(item->value1.i/10000000.0f, item->value2.i/10000000.0f, hfc->altitude_WPnext, item->data[0], item->data[1]);
                hfc->pl_wp_initialized = true;
            }
        }
        else
        if (item->data[0]==WAYPOINT_TAKEOFF)
        {
            /* initialize it only for the first time */
            if (hfc->waypoint_type != WAYPOINT_TAKEOFF)
            {
                // TODO::??: check if in the air, if so, do NOT TAKE OFF!
                telem.CommandTakeoffArm();
//            	if( hfc->fixedThrottleMode == THROTTLE_DEAD ) // check to make sure motors are off
//            	{
//            		Command_TakeoffArm(hfc);
//            	}
//            	else // check if you are flying and high enough in the air (1m)
//            	if(hfc->fixedThrottleMode == THROTTLE_FLY && hfc->altitude_lidar >= 1 )
//            	{
//            		hfc->waypoint_stage = FM_TAKEOFF_COMPLETE;
//            	}
//            	else // do not take off if motors are on and you are not high enough
//            	{
//            		hfc->waypoint_stage = FM_TAKEOFF_NONE;
//            	}
            }
        }
        else
        if (item->data[0]==WAYPOINT_LANDING)
        {
            /* initialize it only for the first time */
            if (hfc->waypoint_type != WAYPOINT_LANDING) {
                telem.CommandLandingWP(item->value1.i/10000000.0f, item->value2.i/10000000.0f, 10);
//                debug_print("%f %f\r\n", item->value1.i/10000000.0f, item->value2.i/10000000.0f);
            }
        }
    }
    else
    if (item->type==PL_ITEM_PARAM)
    {
        int group = item->data[0];
        int sub_param = item->data[1];
        if (group==TELEM_PARAM_STICK)
        {
        }
        else
        if (group==TELEM_PARAM_WAYPOINT)
        {
            if (sub_param==TELEM_PARAM_WP_ALTITUDE)	// relative altitude
                CheckRangeAndSetF(&hfc->altitude_WPnext, item->value1.f, -8999, 9999);
            else
//            if (sub_param==TELEM_PARAM_WP_LATITUDE)
//                CheckRangeAndSetD(&hfc->waypoint_pos[0], item->value1.i/10000000.0, -90, 90);
//            else
//            if (sub_param==TELEM_PARAM_WP_LONGITUDE)
//                CheckRangeAndSetD(&hfc->waypoint_pos[1], item->value1.i/10000000.0, -180, 180);
//            else
            if (sub_param==TELEM_PARAM_WP_MAX_H_SPEED)
            {
                CheckRangeAndSetF(&hfc->pid_Dist2T.COmax, item->value1.f, 0.1f, 50);
//            	DynamicAccInTurns(hfc, &hfc->pid_Dist2T);
            }
            else
            if (sub_param==TELEM_PARAM_WP_MAX_H_ACC)
            {
                CheckRangeAndSetF(&hfc->pid_Dist2T.acceleration, item->value1.f, 0.1f, 100);
//            	DynamicAccInTurns(hfc, &hfc->pid_Dist2T);
            }
            else
            if (sub_param==TELEM_PARAM_WP_MAX_V_SPEED)
            {
                if (CheckRangeAndSetF(&hfc->pid_CollAlt.COmax, item->value1.f, 0.1f, 10))
                	hfc->rw_cfg.VspeedMax = hfc->pid_CollAlt.COmax;
            }
            else
            if (sub_param==TELEM_PARAM_WP_MAX_V_ACC)
            {
                if (CheckRangeAndSetF(&hfc->pid_CollAlt.acceleration, item->value1.f, 0.1f, 100))
                	hfc->rw_cfg.VspeedAcc = hfc->pid_CollAlt.acceleration;
            }
            else
//            if (sub_param==TELEM_PARAM_WP_TYPE)
//                CheckRangeAndSetI(&hfc->waypoint_type, item->value1.i, 0, 1);
//            else
            if (sub_param==TELEM_PARAM_WP_RETIRE)
                CheckRangeAndSetI(&hfc->waypoint_retire, item->value1.i, 0, 1);
            else
            if (sub_param==TELEM_PARAM_WP_YAWSPEEDRATE)
            {
//                CheckRangeAndSetF(&pConfig->yaw_rate_speed, item->value1.f, 10, 10000);
            }
            else
            if (sub_param==TELEM_PARAM_WP_GTWP_RADIUS)
                CheckRangeAndSetF(&hfc->rw_cfg.GTWP_retire_radius, item->value1.f, 0, 20);
            else
            if (sub_param==TELEM_PARAM_WP_GTWP_SPEED)
                CheckRangeAndSetF(&hfc->rw_cfg.GTWP_retire_speed, item->value1.f, 0, 20);
            else
            if (sub_param==TELEM_PARAM_WP_FTWP_SR_FACTOR)
                CheckRangeAndSetF(&hfc->rw_cfg.FTWP_retire_sr_factor, item->value1.f, 0, 10);
            else
            if (sub_param==TELEM_PARAM_WP_LOW_SPEED_LMT)
                CheckRangeAndSetF(&hfc->rw_cfg.low_speed_limit, item->value1.f, 1, 30);
            else
            if (sub_param==TELEM_PARAM_WP_MIN_V_SPEED)
            {
                if (CheckRangeAndSetF(&hfc->pid_CollAlt.COmin, item->value1.f, -10, -0.5))
                	hfc->rw_cfg.VspeedMin = hfc->pid_CollAlt.COmin;
            }
            else
            if (sub_param==TELEM_PARAM_WP_ALTITUDE_BASE)
                CheckRangeAndSetF(&hfc->altitude_base, item->value1.f, 0, 9999);
        }
        else
        if (group==TELEM_PARAM_JOYSTICK)
        {
        }
        else
        if (group==TELEM_PARAM_CONTROL)
        {
            if (sub_param==TELEM_PARAM_CTRL_HEADING_REL)
            {
                float value;
                if (CheckRangeAndSetF(&value, item->value1.f, -180, 180))
                    // TODO::MRI: What is this being used for
                    HeadingUpdate(value, 1);
            }
            else
            if (sub_param==TELEM_PARAM_CTRL_HEADING_ABS)
                CheckRangeAndSetF(&hfc->ctrl_out[ANGLE][YAW], item->value1.f, -180, 180);
            else
            if (sub_param==TELEM_PARAM_CTRL_WIND_COMP)
                CheckRangeAndSetB(&hfc->rw_cfg.wind_compensation, item->value1.i, 0, 1);
            else
            if (sub_param==TELEM_PARAM_CTRL_PATH_NAVIG)
                CheckRangeAndSetB(&hfc->rw_cfg.path_navigation, item->value1.i, 0, 1);
            else
            if (sub_param==TELEM_PARAM_CTRL_ANGLE_COLL_MIX)
                CheckRangeAndSetF(&hfc->rw_cfg.AngleCollMixing, item->value1.f, 0, 2);
            else
            if (sub_param==TELEM_PARAM_CTRL_CRUISE_LIMIT)
                CheckRangeAndSetF(&hfc->rw_cfg.cruise_speed_limit, item->value1.f, 0, 100);
            else
            if (sub_param==TELEM_PARAM_CTRL_NOSE2WP)
                CheckRangeAndSetB(&hfc->rw_cfg.nose_to_WP, item->value1.i, 0, 1);
        }
    }
    else if (item->type == PL_ITEM_DELAY)
    {
        if (hfc->delay_counter<=0)
            hfc->delay_counter = item->value1.i*1000;
    }
}

static void Playlist_ProcessBottom(FlightControlData *hfc, bool retire_waypoint)
{
    T_PlaylistItem *item;
    
    /* single waypoint mode handling - waypoint retire logic */
    if ((hfc->ctrl_source==CTRL_SOURCE_AUTO2D || hfc->ctrl_source==CTRL_SOURCE_AUTO3D) && hfc->playlist_status==PLAYLIST_STOPPED)
    {
        if (retire_waypoint)
            telem.SelectCtrlSource(CTRL_SOURCE_RCRADIO);
        return;
    }
    
    /* process only running playlist mode */
    if (hfc->playlist_status!=PLAYLIST_PLAYING)
        return;
        
    item = &hfc->playlist[hfc->playlist_position];

    if (item->type == PL_ITEM_WP)
    {
        if (item->data[0]==WAYPOINT_TAKEOFF)
        {
            if (hfc->waypoint_stage == FM_TAKEOFF_COMPLETE)
            {
                hfc->waypoint_type  = WAYPOINT_NONE;
                hfc->playlist_position++;
            }
        }
        else
        if (item->data[0]==WAYPOINT_LANDING)
        {
            if (hfc->waypoint_stage == FM_LANDING_LANDED)
            {
                hfc->waypoint_type  = WAYPOINT_NONE;
                hfc->playlist_position++;
            }
        }
        else
        {
            /* if WP is blocking, wait for the retire flag */
            if (!item->data[1] || retire_waypoint)  // WP completion flag
            {
                hfc->playlist_position++;
                hfc->pl_wp_initialized = false;
            }
        }
    }    
    else if (item->type == PL_ITEM_PARAM)
    {
        hfc->playlist_position++;
    }
    else if (item->type == PL_ITEM_GOTO)
    {
        /* handle other types of GOTOs */
        /* sanity check is done below */
        hfc->playlist_position = item->value1.i;
    }
    else if (item->type == PL_ITEM_DELAY)
    {
        hfc->delay_counter -= hfc->ticks_curr;
        if (hfc->delay_counter<=0)
            hfc->playlist_position++;
    }
    else if (item->type == PL_ITEM_HOLD)
    {
        hfc->playlist_position++;
    }
    else    // PL_ITEM_END and every thing else will stop playlist
        hfc->playlist_position = hfc->playlist_items;
        
    
    /* check for end of the playlist or errors */
    if (hfc->playlist_position>=hfc->playlist_items)
    {
        /* stop playlist and waypoint mode */
        hfc->playlist_status = PLAYLIST_STOPPED;
        /* if in flight, put a waypoint at the current position, else do nothing */
        if (hfc->throttle_armed) {
        	telem.SetPositionHold();
        }
    }
}

/* this function runs after the previous control modes are saved, thus PIDs will get aqutomatically re-initialized on mode change */
static void ProcessFlightMode(FlightControlData *hfc)
{
    //GpsData gps_data = gps.GetGpsData();

    if (hfc->message_timeout>0)
        hfc->message_timeout -= hfc->ticks_curr;

    if (hfc->waypoint_type == WAYPOINT_TAKEOFF)
    {

        if (hfc->waypoint_stage == FM_TAKEOFF_AUTO_SPOOL && (hfc->message_from_ground>0 || hfc->message_timeout<=0))
    	{
        	/* cancel and disarmed */
        	if (hfc->message_from_ground!=CMD_MSG_TAKEOFF_OK || hfc->message_timeout<=0)
        	{
                hfc->inhibitRCswitches = false;
    		    /* send message that takeoff has timed out */
        		if (hfc->message_timeout<=0)
        			telem.SendMsgToGround(MSG2GROUND_TAKEOFF_TIMEOUT);

                telem.SelectCtrlSource(CTRL_SOURCE_RCRADIO);
            	telem.Disarm();
                hfc->waypoint_type  = WAYPOINT_NONE;
            	return;
        	}

        	/* switch to auto throttle and go to full throttle */
        	hfc->auto_throttle = true;
        	hfc->throttle_value = pConfig->Stick100range;

   	    	telem.SendMsgToGround(MSG2GROUND_ALLOW_TAKEOFF);

    		hfc->message_from_ground = 0;	// reset it so we can wait for the message from ground
   			hfc->waypoint_stage  = FM_TAKEOFF_ARM;
    	    hfc->message_timeout = 60000000;	// 60 seconds
    	}
    	else if (hfc->waypoint_stage == FM_TAKEOFF_ARM && (hfc->message_from_ground==CMD_MSG_TAKEOFF_ALLOWED || hfc->message_from_ground==CMD_MSG_TAKEOFF_ABORT || hfc->message_timeout<=0))
        {
            hfc->inhibitRCswitches = false;
        	/* cancel and disarmed */
        	if (hfc->message_from_ground!=CMD_MSG_TAKEOFF_ALLOWED || hfc->message_timeout<=0)
        	{
    		    /* send message that takeoff has timed out */
        		if (hfc->message_timeout <= 0) {
        			telem.SendMsgToGround(MSG2GROUND_TAKEOFF_TIMEOUT);
        		}

//                telem.SelectCtrlSource(CTRL_SOURCE_RCRADIO);
            	telem.Disarm();
            	/* on takeoff abort, keep in 3D ctrl source with manual coll at the last value,
            	 * use final landing timeout to prevent RC radio from instantly changing collective */
                SetCtrlMode(hfc, pConfig, COLL,  CTRL_MODE_MANUAL);
                hfc->ctrl_out[RAW][COLL] = pConfig->CollZeroAngle;
                hfc->waypoint_type  = WAYPOINT_LANDING;
                hfc->waypoint_stage = FM_LANDING_TIMEOUT;
                hfc->touchdown_time = hfc->time_ms;
            	return;
        	}

            telem.SelectCtrlSource(CTRL_SOURCE_AUTO3D);
            telem.SaveValuesForAbort();
            hfc->waypoint_type = WAYPOINT_TAKEOFF;

            /* set PRY controls to Angle mode, coll to manual */
            SetCtrlMode(hfc, pConfig, PITCH, CTRL_MODE_ANGLE);
            SetCtrlMode(hfc, pConfig, ROLL,  CTRL_MODE_ANGLE);
            SetCtrlMode(hfc, pConfig, YAW,   CTRL_MODE_ANGLE);
    //        SetCtrlMode(hfc, pConfig, COLL,  CTRL_MODE_SPEED);

            /* initialize PRY angles to the current orientation */
            hfc->ctrl_out[ANGLE][PITCH] = hfc->IMUorient[PITCH]*R2D;
            hfc->ctrl_out[ANGLE][ROLL]  = hfc->IMUorient[ROLL]*R2D;
            hfc->ctrl_out[ANGLE][YAW]   = hfc->IMUorient[YAW]*R2D;
            hfc->ctrl_angle_pitch_3d = hfc->ctrl_out[ANGLE][PITCH];
            hfc->ctrl_angle_roll_3d  = hfc->ctrl_out[ANGLE][ROLL];
            /* set home position */
            telem.SetHome();

            /* set vspeed to the takeoff speed */
            hfc->ctrl_collective_3d  = hfc->pid_CollVspeed.COmax;   // set target collective to max value and wait for the heli to clear ground
            hfc->fixedThrottleMode = THROTTLE_FLY;					// rvw
            /* default PID values */
        	telem.ApplyDefaults();
        	/* slow max vspeed to make collective to move slowely */
//        	hfc->pid_CollAlt.COmax = pConfig->throttle_ctrl==PROP_VARIABLE_PITCH ? 0.1f : 0.5f;
            hfc->waypoint_stage  = FM_TAKEOFF_START;
        }
        else
        if (hfc->waypoint_stage == FM_TAKEOFF_START)
        {
            float thr = ClipMinMax(pConfig->CollThrAutoLevel, 0, 1);
        	float limit = (1-thr)*pConfig->CollZeroAngle + thr*hfc->pid_CollVspeed.COofs;
            /* set pitch/roll angle to trim values once collective exceeds the set % of hover value */
            if ((hfc->ctrl_collective_raw>limit) || (hfc->IMUspeedGroundENU[2]>0.2f))
            {
                hfc->ctrl_angle_pitch_3d = hfc->pid_PitchSpeed.COofs;
                hfc->ctrl_angle_roll_3d  = hfc->pid_RollSpeed.COofs;

                hfc->waypoint_stage = FM_TAKEOFF_LEVEL;
            }
        }
        else
        if (hfc->waypoint_stage == FM_TAKEOFF_LEVEL)
        {
            /* starting to take off, watch for the ENU speeds exceeding the threshold and switch to alt hold mode */
            if ((ABS(hfc->IMUspeedGroundENU[0])>0.4f) || (ABS(hfc->IMUspeedGroundENU[1])>0.4f) || (hfc->IMUspeedGroundENU[2]>0.5f))
            {
            	/* enable alt hold mode */
                hfc->ctrl_out[RAW][COLL]  = hfc->ctrl_collective_raw;
                hfc->ctrl_out[SPEED][COLL] = hfc->IMUspeedGroundENU[2];    // initialize to current vert speed

                SetCtrlMode(hfc, pConfig, COLL,  CTRL_MODE_POSITION);
                hfc->ctrl_out[POS][COLL] = hfc->home_pos[2] + 10;
                hfc->waypoint_pos[2] = hfc->ctrl_out[POS][COLL];    // needs to be initialized for further waypoint flying if altitude is not specified


                /* enable horizontal speed control with zero speed */
                SetCtrlMode(hfc, pConfig, PITCH, CTRL_MODE_SPEED);
                SetCtrlMode(hfc, pConfig, ROLL,  CTRL_MODE_SPEED);
                hfc->ctrl_out[SPEED][PITCH] = 0;
                hfc->ctrl_out[SPEED][ROLL]  = 0;
                
                hfc->waypoint_stage = FM_TAKEOFF_SPEED;
            }
        }
        else
        if (hfc->waypoint_stage == FM_TAKEOFF_SPEED)
        {
            /* once in horizontal speed mode, watch the altitude and switch to position and altitude hold */
            if (hfc->altitude > (hfc->home_pos[2]+0.3f))
            {
                hfc->waypoint_pos[0] = hfc->home_pos[0];
                hfc->waypoint_pos[1] = hfc->home_pos[1];
                hfc->waypoint_retire    = 0;
                SetCtrlMode(hfc, pConfig, PITCH, CTRL_MODE_POSITION);
                SetCtrlMode(hfc, pConfig, ROLL,  CTRL_MODE_POSITION);
//                SetCtrlMode(hfc, pConfig, COLL,  CTRL_MODE_POSITION);
//                hfc->ctrl_out[POS][COLL] = hfc->home_pos[2] + 5;
                hfc->waypoint_stage = FM_TAKEOFF_HOLD;
            }
        }
        else
        if (hfc->waypoint_stage == FM_TAKEOFF_HOLD)
        {
        	if (hfc->altitude>(hfc->home_pos[2]+5))
                hfc->waypoint_stage = FM_TAKEOFF_COMPLETE;
        }
    }
    else
    if (hfc->waypoint_type == WAYPOINT_LANDING)
    {
        if (hfc->waypoint_stage == FM_LANDING_WAYPOINT)
        {
        	if (gps.gps_data_.HspeedC <= hfc->rw_cfg.GTWP_retire_speed && hfc->gps_to_waypoint[0] <= hfc->rw_cfg.GTWP_retire_radius)
            {
                /* send out message and setup timeout */
                hfc->waypoint_stage = FM_LANDING_HOLD;
                hfc->message_from_ground = 0;   // reset it so we can wait for the message from ground
                hfc->message_timeout = 30000000;    // 30 seconds
                telem.SendMsgToGround(MSG2GROUND_ALLOW_LANDING);
            }
        }
        else if (hfc->waypoint_stage == FM_LANDING_HOLD && (hfc->message_from_ground==CMD_MSG_LANDING_GO || hfc->message_timeout<=0))
        {
            /* check for incoming message or timeout */
            telem.CommandLanding(false, false);
        }
        else if (hfc->waypoint_stage == FM_LANDING_HIGH_ALT)
        {
            if (hfc->altitude_lidar <= 3)
                telem.CommandLanding(true, false);
        }
        else
        if (hfc->waypoint_stage == FM_LANDING_LOW_ALT)
        {
            /* heli coming down, watch collective manual value, once below 80% of hover, switch to rate mode */
//            float CO_thr = 0.7f*hfc->pid_CollVspeed.COofs + 0.3f*hfc->CollZeroAngle;
//            if (hfc->pid_CollVspeed.COlast < CO_thr)
            /* heli coming down, switch to rate mode once below 0.1m */
            if (hfc->altitude_lidar < 0.2f)
            {
                /* set PRY controls to rate */
                SetCtrlMode(hfc, pConfig, PITCH, CTRL_MODE_RATE);
                SetCtrlMode(hfc, pConfig, ROLL,  CTRL_MODE_RATE);
                SetCtrlMode(hfc, pConfig, YAW,   CTRL_MODE_RATE);
                
                hfc->ctrl_out[RATE][PITCH] = 0;
                hfc->ctrl_out[RATE][ROLL]  = 0;
                hfc->ctrl_out[RATE][YAW]   = 0;
                /* double the vertical speed just before touchdown to reduce the jump
                 * and to reduce the time blades take to flatten */
//                if (pConfig->throttle_ctrl==PROP_VARIABLE_PITCH)
                hfc->ctrl_vspeed_3d = -pConfig->landing_vspeed*0.6f;
                hfc->waypoint_stage = FM_LANDING_TOUCHDOWN;
            }
        }
        else
        if (hfc->waypoint_stage == FM_LANDING_TOUCHDOWN)
        {
            /* heli is on the ground in rate mode, wait till coll is at zero angle and then shut down */
            /* for fixed prop, kill throttle once close to the ground */
            if (/*(pConfig->throttle_ctrl==PROP_FIXED_PITCH && hfc->altitude_lidar < 0.15f)
            || (pConfig->throttle_ctrl==PROP_VARIABLE_PITCH &&*/ hfc->pid_CollVspeed.COlast <= pConfig->CollZeroAngle)
            {
                SetCtrlMode(hfc, pConfig, COLL,  CTRL_MODE_MANUAL);
                hfc->ctrl_out[RAW][COLL] = hfc->pid_CollVspeed.COlast;
                hfc->ctrl_collective_raw = hfc->pid_CollVspeed.COlast;
                hfc->ctrl_collective_3d = hfc->pid_CollVspeed.COlast;
                /* cutoff motor */
                hfc->throttle_armed = 0;
                hfc->waypoint_stage = FM_LANDING_TIMEOUT;
                hfc->touchdown_time = hfc->time_ms;

                // TODO::SP: Error Handling on Flash write error??
                if (hfc->pid_params_changed) {
                	SavePIDUpdates(hfc);
                }
            }
        }
        else
        if (hfc->waypoint_stage == FM_LANDING_TIMEOUT)
        {
            if ((hfc->time_ms-hfc->touchdown_time)>pConfig->landing_timeout)
            {
                telem.SelectCtrlSource(CTRL_SOURCE_RCRADIO);
                hfc->waypoint_type  = WAYPOINT_LANDING;   // set wp back to landing so playlist can detect completion
                hfc->waypoint_stage = FM_LANDING_LANDED;
                hfc->playlist_status = PLAYLIST_STOPPED;
            }
        }
    }
}

static void SetSpeedAcc(float *value, float speed, float acc, float dT)
{
    float v = *value;
    float delta = acc*dT;
    v = ClipMinMax(speed, v-delta, v+delta);
    *value = v;
}

static float CalcMinAboveGroundAlt(float speed)
{
    float alt = ClipMinMax((speed - pConfig->LidarHVcurve[0]) * pConfig->LidarHVcurve[1] + pConfig->LidarHVcurve[2],
                               pConfig->LidarHVcurve[2], pConfig->LidarHVcurve[3]);

    /* clip not to exceed the sensor operating range otherwise it would keep pushing the heli up */
    if (pConfig->ground_sensor == GROUND_SENSOR_SONAR) {
        alt = min(alt, 7);
    }

    return alt;
}

// TODO::SP: This needs to ensure when in servoRaw that the throttle
// CANNOT be engaged When ARMED!
static void ServoUpdateRAW(float dT)
{
    char xbus_new_values = xbus.NewValues(dT);

    if (!hfc.full_auto && !hfc.throttle_armed) {
        hfc.auto_throttle = false;
    }

    if (!hfc.auto_throttle) {
        hfc.throttle_value   = xbus.valuesf[XBUS_THR_LV];
    }
    else if (!hfc.throttle_armed) {
        hfc.throttle_value = -pConfig->Stick100range;
    }

    hfc.ctrl_out[RAW][THRO]  = hfc.collective_value;
    hfc.ctrl_out[RAW][PITCH] = xbus.valuesf[XBUS_PITCH];
    hfc.ctrl_out[RAW][ROLL]  = xbus.valuesf[XBUS_ROLL];
    hfc.ctrl_out[RAW][YAW]   = xbus.valuesf[XBUS_YAW];
    hfc.ctrl_out[RAW][COLL]  = hfc.collective_value;

    if(!hfc.throttle_armed) {
        hfc.fixedThrottleMode = THROTTLE_IDLE;
    }

    if (!hfc.full_auto && !hfc.auto_throttle) {
        hfc.collective_value = xbus.valuesf[XBUS_THRO];
    }
    else {
        hfc.collective_value = 0;
    }

    if (xbus_new_values) {
        if (xbus_new_values == XBUS_NEW_VALUES_1ST) {
            telem.SaveValuesForAbort();
        }
        SetControlMode();
    }

    ProcessStickInputs(&hfc, dT);

    /* throttle control */
    //TODO::SP: Check where throttle_values come from i.e from modifiable data?
    if (pConfig->throttle_ctrl == PROP_VARIABLE_PITCH) {
          hfc.ctrl_out[RAW][THRO] = (hfc.throttle_value+pConfig->Stick100range)*pConfig->throttle_values[1]+pConfig->throttle_values[0];     // set by channel 6
    }


    Display_Process(&hfc, xbus_new_values, dT);

    hfc.collective_raw_curr = hfc.ctrl_out[RAW][COLL];
    hfc.ctrl_out[RAW][THRO] += hfc.throttle_offset;

    hfc.mixer_in[PITCH] = hfc.ctrl_out[RAW][PITCH];
    hfc.mixer_in[ROLL]  = hfc.ctrl_out[RAW][ROLL];
    hfc.mixer_in[YAW]   = hfc.ctrl_out[RAW][YAW];
    hfc.mixer_in[COLL]  = hfc.ctrl_out[RAW][COLL];

    if (hfc.throttle_armed) {
        hfc.mixer_in[THRO]  = hfc.ctrl_out[RAW][THRO];
    }
    else {
        hfc.mixer_in[THRO]  = 1000; // ensure throttle is off
    }

    ServoMixer();

    if (pConfig->ccpm_type == MIXERTANDEM) {
        WriteToServoNodeServos(pConfig->num_servo_nodes);
    }
    else {
        if (pConfig->power_node) {
            WriteToPowerNodeServos();
        }
        else if (pConfig->fcm_servo) {
            WriteToFcmServos();
        }
    }
}

/* TODO::??: Fix AngleCompensation calculation and if its use is valid */
static void ServoUpdate(float dT)
{
    char  control_mode_prev[4] = {0,0,0,0};
    char xbus_new_values = xbus.NewValues(dT);
    bool retire_waypoint = false;
    float AngleCompensation = COSfD(min(45, ABS(hfc.IMUorient[PITCH]*R2D)) * COSfD(min(45, ABS(hfc.IMUorient[ROLL]*R2D))));
//    float throttle_prev = hfc.ctrl_out[RAW][THRO];

    hfc.altitude_lidar = hfc.altitude_lidar_raw * AngleCompensation;
//    debug_print("%+3d %4d %+4d %d\r\n", (int)(hfc.ctrl_out[RAW][COLL]*1000), (int)(hfc.altitude_lidar*1000), (int)(hfc.lidar_vspeed*1000), lidar_last_time/1000);
//    debug_print("%+3d %4d %+4d %d\r\n", (int)(hfc.ctrl_out[SPEED][COLL]*1000), (int)(hfc.altitude_lidar*1000), (int)(hfc.lidar_vspeed*1000), lidar_last_time/1000);

    if (!hfc.full_auto && !hfc.throttle_armed)
    	hfc.auto_throttle = false;
    
    if (!hfc.auto_throttle)
    	hfc.throttle_value   = xbus.valuesf[XBUS_THR_LV];
    else if (!hfc.throttle_armed)
    	hfc.throttle_value = -pConfig->Stick100range;


//    if (!(hfc.print_counter&0x3f))
//    	debug_print("FA %d AT %d thr=%+5.3f col=%+5.3f\r\n", hfc.full_auto, hfc.auto_throttle, hfc.throttle_value, hfc.collective_value);
    hfc.ctrl_out[RAW][THRO]  = hfc.collective_value;

//    if (!hfc.full_auto)
    {
		if (hfc.ctrl_source!=CTRL_SOURCE_JOYSTICK)
		{
			hfc.ctrl_out[RAW][PITCH] = xbus.valuesf[XBUS_PITCH];
			hfc.ctrl_out[RAW][ROLL]  = xbus.valuesf[XBUS_ROLL];
			hfc.ctrl_out[RAW][YAW]   = xbus.valuesf[XBUS_YAW];
			hfc.ctrl_out[RAW][COLL]  = hfc.collective_value;
		}
		else
		{
			hfc.ctrl_out[RAW][PITCH] = hfc.joy_values[PITCH];
			hfc.ctrl_out[RAW][ROLL]  = hfc.joy_values[ROLL];
			hfc.ctrl_out[RAW][YAW]   = hfc.joy_values[YAW];
			hfc.ctrl_out[RAW][COLL]  = hfc.joy_values[COLL];
			if (hfc.joystick_new_values)
			{
				xbus_new_values = XBUS_NEW_VALUES;
				hfc.joystick_new_values = 0;
			}
			else
				xbus_new_values = XBUS_NO_NEW_VALUES;
		}
    }
    // RVW throttle stick to collective logic section
    if(!hfc.throttle_armed)
    	hfc.fixedThrottleMode = THROTTLE_IDLE;		//  set to follow lever

    if (!hfc.full_auto && !hfc.auto_throttle)
    {
        if (pConfig->throttle_ctrl==PROP_FIXED_PITCH && pConfig->SbusEnable == 1)
        {
        	// throttle follows xbus stick position
        	if(hfc.fixedThrottleMode == THROTTLE_FLY)
        	{
            	hfc.collective_value = xbus.valuesf[XBUS_THRO];
            		// wait for time out and disarm
            	if( hfc.collective_value < -0.5 && hfc.altitude_lidar < 0.2) // && ( (hfc.IMUspeedGroundENU[2]> -0.001f) || (hfc.IMUspeedGroundENU[2]< 0.001f) ))     // three second timeout 3000 counts
            	{
            		if(hfc.fixedThrottleMult == 3000)
            		{
            		hfc.fixedThrottleMode = THROTTLE_IDLE;
            		telem.Disarm();
            		}
            		hfc.fixedThrottleMult += 1;
            	}
            	else
            		hfc.fixedThrottleMult = 0;
        	}
        	// ramp to stick position in 3 seconds
        	if(hfc.fixedThrottleMode == THROTTLE_RAMP)
        	{
            	if(hfc.fixedThrottleMult >= 1)
            	{
            		hfc.fixedThrottleMode = THROTTLE_FLY;
            	}
            	hfc.fixedThrottleMult += 0.0003f;
            	hfc.collective_value = hfc.fixedThrottleCap - ((1 - hfc.fixedThrottleMult) * 0.571f * (-1.0f * pConfig->throttle_multi_min ));  // rvw
            	if(hfc.fixedThrottleMult < 0.8)
            	{
                ResetIterms();
                hfc.ctrl_out[RAW][PITCH] = 0;
                hfc.ctrl_out[RAW][ROLL]  = 0;
                hfc.ctrl_out[RAW][YAW]   = 0;
            	}
        	}
        	// wait for stick movement to go to next state time out if nothing
        	if(hfc.fixedThrottleMode == THROTTLE_DEAD)
        	{
				if ((xbus.valuesf[XBUS_THRO] - hfc.fixedThrottleCap)  > AUTO_PROF_TERMINATE_THRS)
				{
					hfc.fixedThrottleMult = 0;
					hfc.collective_value = -0.571;
					hfc.fixedThrottleMode = THROTTLE_RAMP;
				}
                ResetIterms();
        	}
        	// check if lever is raised to top to start machine
        	if(hfc.fixedThrottleMode == THROTTLE_IDLE && hfc.throttle_value > 0.5)
        	{
        		hfc.fixedThrottleCap =  xbus.valuesf[XBUS_THRO];  	// capture midstick value
				hfc.collective_value = -0.571;
        		hfc.fixedThrottleMode = THROTTLE_DEAD;				// next state
        	}
        	else if(hfc.fixedThrottleMode == THROTTLE_IDLE)
        	{
        		hfc.collective_value = -0.571;
        	}
        }
    	else
    	{
        	hfc.collective_value = xbus.valuesf[XBUS_THRO];  // RVW not fixed pitch so no self center throttle stick
    	}
    }
    else
    {
    	hfc.collective_value = 0;
    }
    control_mode_prev[PITCH] = hfc.control_mode[PITCH];
    control_mode_prev[ROLL]  = hfc.control_mode[ROLL];
    control_mode_prev[YAW]   = hfc.control_mode[YAW];
    control_mode_prev[COLL]  = hfc.control_mode[COLL];
    
    /* process command queue here */
    telem.ProcessCommands();

    Playlist_ProcessTop(&hfc);
    
    /* check for a new control mode */
    if (xbus_new_values)
    {
    	if (xbus_new_values==XBUS_NEW_VALUES_1ST) {
    		telem.SaveValuesForAbort();
    	}

        SetControlMode();
//        debug_print("Full auto %d Auto Throttle %d\r\n", hfc.full_auto, hfc.auto_throttle);
    }

    /* 11us low pass stick input to soften the response */
    ProcessStickInputs(&hfc, dT);

    /* throttle control */
    if (pConfig->throttle_ctrl==PROP_VARIABLE_PITCH)
      hfc.ctrl_out[RAW][THRO] = (hfc.throttle_value+pConfig->Stick100range)*pConfig->throttle_values[1]+pConfig->throttle_values[0];     // set by channel 6

    /* performs logging and automatic profiling control */
    Profiling_Process(&hfc, pConfig);

    if (hfc.ctrl_source==CTRL_SOURCE_RCRADIO || hfc.ctrl_source==CTRL_SOURCE_JOYSTICK)
    {
        if (hfc.ctrl_source==CTRL_SOURCE_JOYSTICK)
        {
          if (hfc.joy_PRmode)
          {
              SetSpeedAcc(&hfc.ctrl_out[SPEED][PITCH], -hfc.ctrl_out[RAW][PITCH]*hfc.Stick_Hspeed, hfc.rw_cfg.StickHaccel, dT);
              SetSpeedAcc(&hfc.ctrl_out[SPEED][ROLL],   hfc.ctrl_out[RAW][ROLL]*hfc.Stick_Hspeed,  hfc.rw_cfg.StickHaccel, dT);
          }
          else
          {
            hfc.ctrl_out[SPEED][PITCH] += hfc.joy_values[THRO]*hfc.rw_cfg.StickHaccel*dT;

            if (hfc.ctrl_out[SPEED][PITCH] > pConfig->joystick_max_speed)
              hfc.ctrl_out[SPEED][PITCH] = pConfig->joystick_max_speed;

            if (hfc.joy_values[THRO]<0 && hfc.ctrl_out[SPEED][PITCH]<0)
              hfc.ctrl_out[SPEED][PITCH] = 0;

            SetSpeedAcc(&hfc.ctrl_out[SPEED][ROLL],   0,  hfc.rw_cfg.StickHaccel, dT);
          }
        }
        else
        {
          SetSpeedAcc(&hfc.ctrl_out[SPEED][PITCH], -hfc.ctrl_out[RAW][PITCH]*hfc.Stick_Hspeed, hfc.rw_cfg.StickHaccel, dT);
          SetSpeedAcc(&hfc.ctrl_out[SPEED][ROLL],   hfc.ctrl_out[RAW][ROLL]*hfc.Stick_Hspeed,  hfc.rw_cfg.StickHaccel, dT);
        }

        hfc.ctrl_out[RATE][PITCH]  = hfc.ctrl_out[RAW][PITCH]*hfc.PRstick_rate  + hfc.pid_PitchAngle.COofs;
        hfc.ctrl_out[ANGLE][PITCH] = hfc.ctrl_out[RAW][PITCH]*hfc.PRstick_angle + hfc.pid_PitchSpeed.COofs;
        hfc.ctrl_out[RAW][PITCH]  += hfc.pid_PitchRate.COofs;
        hfc.ctrl_out[RATE][ROLL]   = hfc.ctrl_out[RAW][ROLL]*hfc.PRstick_rate   + hfc.pid_RollAngle.COofs;
        hfc.ctrl_out[ANGLE][ROLL]  = hfc.ctrl_out[RAW][ROLL]*hfc.PRstick_angle  + hfc.pid_RollSpeed.COofs;
        hfc.ctrl_out[RAW][ROLL]   += hfc.pid_RollRate.COofs;

//        hfc.ctrl_out[SPEED][PITCH] =-hfc.ctrl_out[RAW][PITCH]*hfc.Stick_Hspeed;
//        hfc.ctrl_out[SPEED][ROLL]  = hfc.ctrl_out[RAW][ROLL]*hfc.Stick_Hspeed;
    }
    
    /* set heading to the IMU's heading for throttle stick below -0.55, kind of like Landed mode detection */
    if (hfc.throttle_value<-0.55f)
        hfc.ctrl_out[ANGLE][YAW] = hfc.IMUorient[YAW]*R2D; 

    if (hfc.ctrl_source!=CTRL_SOURCE_AUTO3D)
    {
        float yaw_rate_ctrl = hfc.ctrl_out[RAW][YAW]*hfc.YawStick_rate;
        hfc.ctrl_out[SPEED][COLL]  = hfc.ctrl_out[RAW][COLL]*hfc.Stick_Vspeed;

        if (hfc.rw_cfg.ManualLidarAltitude) {
            hfc.ctrl_out[POS][COLL] = 2 + 2*hfc.ctrl_out[RAW][COLL];
        }

        yaw_rate_ctrl = ClipMinMax(yaw_rate_ctrl, hfc.pid_YawAngle.COmin, hfc.pid_YawAngle.COmax);

        // TODO::MRI: What are these used for?
        HeadingUpdate(yaw_rate_ctrl, dT);
        AltitudeUpdate(hfc.ctrl_out[RAW][COLL]*hfc.Stick_Vspeed, dT);
        
        hfc.ctrl_out[RATE][YAW]  = yaw_rate_ctrl;
        hfc.ctrl_out[RAW][YAW]  += hfc.pid_YawRate.COofs;


        /* do not add offsets to RAW above, apply gain to RAW, added offset to RAW,
        ** clip RAW using rate PID limits */
        hfc.ctrl_out[RAW][COLL]  = hfc.ctrl_out[RAW][COLL] * pConfig->control_gains[COLL];
        hfc.ctrl_out[RAW][COLL] += hfc.pid_CollVspeed.COofs;
    }

    /* processes staged waypoints - takeoff, landing, ... */
    ProcessFlightMode(&hfc);

    /* push control to display */
    /* 2, 0, 45us */
    Display_Process(&hfc, xbus_new_values, dT);
    
    /* horizontal position */
    if (hfc.control_mode[PITCH]==CTRL_MODE_POSITION || hfc.control_mode[ROLL]==CTRL_MODE_POSITION)
    {
        float distance_to_ref;
        float D2T_clipped;
        float course_to_ref;
        float speed;
        float PathSpeedR;

        distance_to_ref = DistanceCourse(hfc.positionLatLon[0], hfc.positionLatLon[1], hfc.waypoint_pos[0], hfc.waypoint_pos[1], &course_to_ref);
//        debug_print("dist %4.1f \r\n", distance_to_ref);
        /* error handling: do nothing if distance over 100k */
        if (distance_to_ref>100000)
          distance_to_ref = 0;

        /* keep track of the minimum distance to the next waypoint */
        if ( distance_to_ref < hfc.distance2WP_min)
            hfc.distance2WP_min = distance_to_ref;
            
        hfc.gps_to_waypoint[0] = distance_to_ref;
        hfc.gps_to_waypoint[1] = course_to_ref;

        D2T_clipped = hfc.waypoint_type==WAYPOINT_FLYTHROUGH ? 1000 : distance_to_ref;

        /* reset on the first time position mode is turned on */
        if (control_mode_prev[PITCH]<CTRL_MODE_POSITION)
        {
          /* not sure if this needed any more since RC radio cannot set WP any more */
          if (hfc.ctrl_source==CTRL_SOURCE_RCRADIO) {
              telem.SetWaypoint(hfc.positionLatLon[0], hfc.positionLatLon[1], -9999, WAYPOINT_GOTO, 0);
          }

          PID_SetForEnable(&hfc.pid_Dist2T, 0, 0, hfc.gps_speed);
          PID_SetForEnable(&hfc.pid_Dist2P, 0, 0, 0);
          hfc.speedCtrlPrevEN[0] = 0;
          hfc.speedCtrlPrevEN[1] = 0;
        }

        speed = PID_P_Acc(&hfc.pid_Dist2T, D2T_clipped, 0, dT, false, false); // speed

        /* do path navigation only once far enough from the target
        ** since otherwise trust vectoring will take care of the final approach */
        if (hfc.rw_cfg.path_navigation && distance_to_ref>2)
        {
            /* add a side vector to the main speed vector to the target waypoint.
            ** The side vector is proportional to the current distance from the path
            ** and it is pulling the aircraft to stay on the path */
            float CTc = course_to_ref;
            float D2P, S2P;
            float STc = hfc.waypoint_STcourse;
            float Cx = (float)(hfc.positionLatLon[1] - hfc.waypoint_pos_prev[1]);
            float Cy = (float)(hfc.positionLatLon[0] - hfc.waypoint_pos_prev[0]);
            float deltaCourse = Wrap180(STc - CTc);
            float S2Prot;
            Cx = Cx/DPM*COSfD(((float)hfc.positionLatLon[0]));
            Cy = Cy/DPM;

            D2P = ABS(hfc.path_a*Cx+hfc.path_b*Cy)*hfc.path_dist_denom;
            S2P = PID_P_Acc(&hfc.pid_Dist2P, D2P, 0, dT, false, false); // speed to path
            
            /* always rotate the speed vector towards the path */
            if (deltaCourse>=0)
                S2Prot = -90;
            else
                S2Prot = 90;
                
//            Rotate(0, S2P, hfc.IMUorient[YAW] - (STc+S2Prot)*D2R, &PathSpeedR, &PathSpeedP);
            PathSpeedR = -S2P*SINfR(hfc.IMUorient[YAW] - (STc+S2Prot)*D2R);
      }
      else      
      {
          hfc.pid_Dist2P.COlast = 0;
          PathSpeedR = 0;
      }

#ifdef THRUST_VECTORING
      {
    	  /* split speed into E/N components */
    	  float speedE = speed * SINfD(course_to_ref);
    	  float speedN = speed * COSfD(course_to_ref);

    	  /* apply acceleration limit to speed changes */
    	  float dE = speedE - hfc.speedCtrlPrevEN[0];
    	  float dN = speedN - hfc.speedCtrlPrevEN[1];
    	  float dS = sqrtf(dE*dE + dN*dN);

    	  if (dS)
    	  {
			  float dSlimit = Min(dS, hfc.acc_dyn_turns*dT);
			  dE = dE * dSlimit/dS;
			  dN = dN * dSlimit/dS;
			  hfc.speedCtrlPrevEN[0] += dE;
			  hfc.speedCtrlPrevEN[1] += dN;
			  speedE = hfc.speedCtrlPrevEN[0];
			  speedN = hfc.speedCtrlPrevEN[1];
    	  }

    	  /* rotate speed E/N to Right/Forward */
          Rotate(speedE, speedN, hfc.IMUorient[YAW], &hfc.ctrl_out[SPEED][ROLL], &hfc.ctrl_out[SPEED][PITCH]);
	  }
#endif

      /* for high speeds, make the nose to point towards the target,
      * or to follow the ground speed vector. For low speeds, do not change it */
      if (distance_to_ref>5)
          hfc.ctrl_out[ANGLE][YAW] = hfc.rw_cfg.nose_to_WP ? course_to_ref : hfc.waypoint_STcourse;

#ifndef THRUST_VECTORING
      if (/*speed>pConfig->low_speed_limit &&*/ distance_to_ref>5 || hfc.waypoint_type==WAYPOINT_FLYTHROUGH)
      {
          hfc.ctrl_out[SPEED][PITCH] = speed;
          hfc.ctrl_out[SPEED][ROLL]  = 0;
      }
      else
      {
          /* split the speed vector pointing to the target to pitch/roll speed components
          ** considering the current orientation of the heli */
          float angle = course_to_ref - hfc.IMUorient[YAW]*R2D;
          hfc.ctrl_out[SPEED][PITCH] = speed * COSfD(angle);
          hfc.ctrl_out[SPEED][ROLL]  = speed * SINfD(angle);
      }
#endif

//      hfc.ctrl_out[SPEED][PITCH] += PathSpeedP;
      hfc.ctrl_out[SPEED][ROLL]  += PathSpeedR; // side component only
      
      /* altitude control - interpolation between waypoints */
      if (hfc.ctrl_source==CTRL_SOURCE_AUTO3D && hfc.waypoint_type != WAYPOINT_TAKEOFF)
      {
        if (hfc.waypoint_STdist>2)
        {
            float a = ClipMinMax((distance_to_ref-hfc.waypoint_STofs) / hfc.waypoint_STdist, 0, 1);
            float altitude = hfc.waypoint_pos_prev[2] * a + hfc.waypoint_pos[2] * (1-a);
            hfc.ctrl_out[POS][COLL] = altitude;
//            if (!(hfc.print_counter&0x3f))
//                debug_print("D2T %4.1f SDdist %4.1f old %5.1f new %5.1f a %4.2f curr %5.1f\r\n", distance_to_ref, hfc.waypoint_STdist, hfc.waypoint_pos_prev[2], hfc.waypoint_pos[2], a, altitude);
        }
        else
            hfc.ctrl_out[POS][COLL] = hfc.waypoint_pos[2];
      }
      
      /* check for waypoint retire conditions */
      if (hfc.waypoint_retire)
      {
          if (hfc.waypoint_type==WAYPOINT_FLYTHROUGH)
          {
              float limit = telem.CalcFTWPlimit(true);
              /* once it gets close enough considering the current speed */
              if (hfc.gps_to_waypoint[0] < max(1, limit))
                retire_waypoint = true;
          }
          else
          {
              /* once it gets close enough at low enough speed, also wait for altitude to match the target !!!!!!!!! */
              //GpsData gps_data = gps.GetGpsData();
              if (gps.gps_data_.HspeedC <= hfc.rw_cfg.GTWP_retire_speed && distance_to_ref <= hfc.rw_cfg.GTWP_retire_radius)
                retire_waypoint = true;
          }
      }
    }

    /* dynamic yaw rate - limits yaw rate to prevent airframe overloading during turns */
    telem.CalcDynYawRate();
    hfc.pid_YawAngle.COmax =  hfc.dyn_yaw_rate;
    hfc.pid_YawAngle.COmin = -hfc.dyn_yaw_rate;

    /* rotate ground speed vector to plane speed vector */
    Rotate(hfc.IMUspeedGroundENU[0], hfc.IMUspeedGroundENU[1],  hfc.IMUorient[YAW], &hfc.speedHeliRFU[0], &hfc.speedHeliRFU[1]);
    hfc.speedHeliRFU[2] = hfc.IMUspeedGroundENU[2];
    
    hfc.bankPitch = 0;
    hfc.bankRoll = 0;   // clear here, it might get set by auto-banking code to compensate Acc for IMU atitude estimation

    /* speed heli - SpeedGroundEN - rotate to SpeedHeliRF, PID(CtrlSpeedRF, SpeedHeliRF)->Angle(R)(-P) */
    if (hfc.control_mode[PITCH]>=CTRL_MODE_SPEED || hfc.control_mode[ROLL]>=CTRL_MODE_SPEED)
    {
//      if (!(hfc.print_counter&0x1f))
//        debug_print("%4.1f %4.1f ", hfc.speed_Iterm_E, hfc.speed_Iterm_N);
      /* rotate E/N speed PID I-terms into current R/F */
      if (hfc.rw_cfg.wind_compensation)
      {
          Rotate(hfc.speed_Iterm_E, hfc.speed_Iterm_N, hfc.IMUorient[YAW], &hfc.pid_RollSpeed.Ie, &hfc.pid_PitchSpeed.Ie);
//          if (!(hfc.print_counter&0x1f))
//             debug_print("1 E %f N %f R %f P %f\n", hfc.speed_Iterm_E, hfc.speed_Iterm_N, hfc.pid_RollSpeed.Ie, hfc.pid_PitchSpeed.Ie);
      }

//      if (!(hfc.print_counter&0x1f))
//        debug_print("%4.1f %4.1f   ", hfc.pid_RollSpeed.Ie, hfc.pid_PitchSpeed.Ie);
      
      /* if previous mode was below SPEED, reset PIDs to be bumpless */
      if ((control_mode_prev[PITCH]<CTRL_MODE_SPEED && !pConfig->ctrl_mode_inhibit[PITCH]) || (control_mode_prev[ROLL]<CTRL_MODE_SPEED && !pConfig->ctrl_mode_inhibit[ROLL]))
      {
          hfc.ctrl_out[SPEED][PITCH] = hfc.speedHeliRFU[1];
          hfc.ctrl_out[SPEED][ROLL]  = hfc.speedHeliRFU[0];
          PID_SetForEnable(&hfc.pid_PitchSpeed,   hfc.ctrl_out[SPEED][PITCH], hfc.speedHeliRFU[1], -hfc.ctrl_out[ANGLE][PITCH]);
          PID_SetForEnable(&hfc.pid_PitchCruise,  hfc.ctrl_out[SPEED][PITCH], hfc.speedHeliRFU[1], -hfc.ctrl_out[ANGLE][PITCH]);
          PID_SetForEnable(&hfc.pid_RollSpeed,    hfc.ctrl_out[SPEED][ROLL],  hfc.speedHeliRFU[0],  hfc.ctrl_out[ANGLE][ROLL]);
      }

      if (!hfc.cruise_mode)
      {
          if (ABS(hfc.ctrl_out[SPEED][PITCH]) >= hfc.rw_cfg.cruise_speed_limit)
          {
              hfc.cruise_mode = true;
              /* smoothly engage cruise mode by keeping the current angle */
              hfc.pid_PitchCruise.COlast = hfc.pid_PitchSpeed.COlast;
          }
      }
      else
      {
          if (ABS(hfc.ctrl_out[SPEED][PITCH]) < 0.8f*hfc.rw_cfg.cruise_speed_limit)
          {
              hfc.cruise_mode = false;
              /* smoothly engage normal speed mode */
              PID_SetForEnable(&hfc.pid_PitchSpeed,   hfc.ctrl_out[SPEED][PITCH], hfc.speedHeliRFU[1], hfc.pid_PitchCruise.COlast);
          }
      }
      if (hfc.cruise_mode)
      {
          /* set trip to an angle, which corresponds to the target speed */
          float angle = hfc.rw_cfg.Speed2AngleLUT[min((int)(ABS(hfc.ctrl_out[SPEED][PITCH])*2+0.5f), SPEED2ANGLE_SIZE-1)];
          if (hfc.ctrl_out[SPEED][PITCH]<0)
              angle = -angle;
          hfc.pid_PitchCruise.COofs = angle;
          hfc.ctrl_out[ANGLE][PITCH] = -PID_P_Acc(&hfc.pid_PitchCruise, hfc.ctrl_out[SPEED][PITCH], hfc.speedHeliRFU[1], dT, false, false); // speed forward
//          if (!(hfc.print_counter&0x3f))
//              debug_print("S %f A %f out %f\n", hfc.ctrl_out[SPEED][PITCH], angle, hfc.ctrl_out[ANGLE][PITCH]);
      }
      else
          hfc.ctrl_out[ANGLE][PITCH] = -PID(&hfc.pid_PitchSpeed, hfc.ctrl_out[SPEED][PITCH], hfc.speedHeliRFU[1], dT); // speed forward

      hfc.ctrl_out[ANGLE][ROLL]  =  PID(&hfc.pid_RollSpeed,  hfc.ctrl_out[SPEED][ROLL],  hfc.speedHeliRFU[0], dT); // speed right
//      if (!(hfc.print_counter&0x1f))
//          debug_print("cS %5.3f mS %5.3f a %5.2f i %f\n", hfc.ctrl_out[SPEED][ROLL], hfc.speedHeliRFU[0], hfc.ctrl_out[ANGLE][ROLL], hfc.pid_RollSpeed.Ie);

      /* rotate back R/F I-terms to E/N */
      if (hfc.rw_cfg.wind_compensation)
      {
          Rotate(hfc.pid_RollSpeed.Ie, hfc.pid_PitchSpeed.Ie, -hfc.IMUorient[YAW], &hfc.speed_Iterm_E, &hfc.speed_Iterm_N);
//          if (!(hfc.print_counter&0x1f))
//             debug_print("2 E %f N %f R %f P %f\n", hfc.speed_Iterm_E, hfc.speed_Iterm_N, hfc.pid_RollSpeed.Ie, hfc.pid_PitchSpeed.Ie);
          hfc.speed_Iterm_E_lp = (hfc.speed_Iterm_E + hfc.speed_Iterm_E_lp*4095)/4096;
          hfc.speed_Iterm_N_lp = (hfc.speed_Iterm_N + hfc.speed_Iterm_N_lp*4095)/4096;
//          if (!(hfc.print_counter&0x3f))
//              debug_print("%f %f %f %f\n", hfc.speed_Iterm_E_lp, hfc.speed_Iterm_E, hfc.speed_Iterm_N_lp, hfc.speed_Iterm_N);
      }
      
      /* pitch-roll mixing to prevent side slip */
      // a=2*PI*YR*speed/360     side acceleration during a turn at speed v and yaw rate YR
      // angle = -atan(a/9.81)
#ifdef THRUST_VECTORING
      if (hfc.control_mode[PITCH]!=CTRL_MODE_POSITION && hfc.control_mode[ROLL]!=CTRL_MODE_POSITION)
#endif
      {
          /* use yaw rate PID input instead of yaw_rate_ctrl */
          float speedP = hfc.ctrl_out[SPEED][PITCH];
          float speedR = hfc.ctrl_out[SPEED][ROLL];
          float ctrl_speed = sqrtf(speedP*speedP + speedR*speedR);
          if (ctrl_speed>0)
          {
        	  float a = 2*PI*hfc.ctrl_yaw_rate*ctrl_speed/360;
        	  float ai = CLIP(a, 9.81f);  // 1G side limit
        	  float bank = ATAN2fD(ai, 9.81f);       // float roll = R2D*atanf(a/9.81f);
          
        	  /* rescale the speed vector to have "bank" magnitude and rotate CW by 90deg */
        	  speedP = speedP * bank / ctrl_speed;
        	  speedR = speedR * bank / ctrl_speed;
        	  hfc.bankRoll =  speedP;
        	  hfc.bankPitch = -speedR;
        	  hfc.ctrl_out[ANGLE][PITCH] -= hfc.bankPitch;
        	  hfc.ctrl_out[ANGLE][ROLL]  += hfc.bankRoll;
              hfc.ctrl_out[ANGLE][PITCH] = ClipMinMax(hfc.ctrl_out[ANGLE][PITCH], hfc.pid_PitchSpeed.COmin, hfc.pid_PitchSpeed.COmax);
              hfc.ctrl_out[ANGLE][ROLL]  = ClipMinMax(hfc.ctrl_out[ANGLE][ROLL],  hfc.pid_RollSpeed.COmin,  hfc.pid_RollSpeed.COmax);
          }
      }
    }

    /* 16us */
    if (hfc.control_mode[PITCH]>=CTRL_MODE_ANGLE)
    {
      if (hfc.control_mode[PITCH]==CTRL_MODE_ANGLE && hfc.ctrl_source==CTRL_SOURCE_AUTO3D)
      	  SetSpeedAcc(&hfc.ctrl_out[ANGLE][PITCH], hfc.ctrl_angle_pitch_3d, pConfig->takeoff_angle_rate, dT);
      /* if previous mode was below ANGLE, reset PIDs to be bumpless */
      if (control_mode_prev[PITCH]<CTRL_MODE_ANGLE)
                     PID_SetForEnable(&hfc.pid_PitchAngle, hfc.ctrl_out[ANGLE][PITCH], hfc.IMUorient[PITCH]*R2D, hfc.ctrl_out[RATE][PITCH]);
      hfc.ctrl_out[RATE][PITCH] = PID(&hfc.pid_PitchAngle, hfc.ctrl_out[ANGLE][PITCH], hfc.IMUorient[PITCH]*R2D, dT);
    }

    if (hfc.control_mode[ROLL]>=CTRL_MODE_ANGLE)
    {
      if (hfc.control_mode[ROLL]==CTRL_MODE_ANGLE && hfc.ctrl_source==CTRL_SOURCE_AUTO3D)
        	  SetSpeedAcc(&hfc.ctrl_out[ANGLE][ROLL], hfc.ctrl_angle_roll_3d, pConfig->takeoff_angle_rate, dT);
      /* if previous mode was below ANGLE, reset PIDs to be bumpless */
      if (control_mode_prev[ROLL]<CTRL_MODE_ANGLE)
                     PID_SetForEnable(&hfc.pid_RollAngle,  hfc.ctrl_out[ANGLE][ROLL],  hfc.IMUorient[ROLL]*R2D,  hfc.ctrl_out[RATE][ROLL]);
      hfc.ctrl_out[RATE][ROLL]  = PID(&hfc.pid_RollAngle,  hfc.ctrl_out[ANGLE][ROLL],  hfc.IMUorient[ROLL]*R2D,  dT);
    }

//    debug_print("%d %5.1f %5.1f\r\n", pr_control_mode, roll_angle, roll_rate);

    if (hfc.control_mode[PITCH]>=CTRL_MODE_RATE)
    {    
      /* if previous mode was below RATE, reset PIDs to be bumpless */
      if (control_mode_prev[PITCH]<CTRL_MODE_RATE)
                    PID_SetForEnable(&hfc.pid_PitchRate, hfc.ctrl_out[RATE][PITCH], hfc.gyroFilt[PITCH], hfc.ctrl_out[RAW][PITCH]);
      hfc.ctrl_out[RAW][PITCH] = PID(&hfc.pid_PitchRate, hfc.ctrl_out[RATE][PITCH], hfc.gyroFilt[PITCH], dT);
    }
    else
      hfc.ctrl_out[RAW][PITCH] = ClipMinMax(hfc.ctrl_out[RAW][PITCH], hfc.pid_PitchRate.COmin, hfc.pid_PitchRate.COmax);
      
    if (hfc.control_mode[ROLL]>=CTRL_MODE_RATE)
    {
      /* if previous mode was below RATE, reset PIDs to be bumpless */
      if (control_mode_prev[ROLL]<CTRL_MODE_RATE)
                   PID_SetForEnable(&hfc.pid_RollRate,  hfc.ctrl_out[RATE][ROLL],   hfc.gyroFilt[ROLL],  hfc.ctrl_out[RAW][ROLL]);
      hfc.ctrl_out[RAW][ROLL] = PID(&hfc.pid_RollRate,  hfc.ctrl_out[RATE][ROLL],   hfc.gyroFilt[ROLL],  dT);
    }
    else
      hfc.ctrl_out[RAW][ROLL] = ClipMinMax(hfc.ctrl_out[RAW][ROLL], hfc.pid_RollRate.COmin, hfc.pid_RollRate.COmax);

    /* 26, 16, 50us */
    if (hfc.control_mode[YAW]>=CTRL_MODE_ANGLE)
    {
      bool double_angle_acc = hfc.ctrl_source==CTRL_SOURCE_JOYSTICK || hfc.ctrl_source==CTRL_SOURCE_RCRADIO ? true : false;
      if (control_mode_prev[YAW]<CTRL_MODE_ANGLE)
      {
        hfc.ctrl_out[ANGLE][YAW] = hfc.IMUorient[YAW]*R2D; 
        PID_SetForEnable(&hfc.pid_YawAngle, hfc.ctrl_out[ANGLE][YAW], hfc.ctrl_out[ANGLE][YAW], hfc.ctrl_out[RATE][YAW]);
      }  
      hfc.ctrl_out[RATE][YAW] = PID_P_Acc(&hfc.pid_YawAngle, hfc.ctrl_out[ANGLE][YAW], hfc.IMUorient[YAW]*R2D, dT, false, double_angle_acc);
    }  

    if (hfc.control_mode[YAW]>=CTRL_MODE_RATE)
    {
      if (control_mode_prev[YAW]<CTRL_MODE_RATE)
                  PID_SetForEnable(&hfc.pid_YawRate, hfc.ctrl_out[RATE][YAW], hfc.gyroFilt[YAW], hfc.ctrl_out[RAW][YAW]);
      hfc.ctrl_out[RAW][YAW] = PID(&hfc.pid_YawRate, hfc.ctrl_out[RATE][YAW], hfc.gyroFilt[YAW], dT);
    }
    else
      hfc.ctrl_out[RAW][YAW] = ClipMinMax(hfc.ctrl_out[RAW][YAW], hfc.pid_YawRate.COmin, hfc.pid_YawRate.COmax);

    hfc.ctrl_yaw_rate = hfc.ctrl_out[RATE][YAW];  // store yaw rate for auto banking
    

    /* collective */
    /* 34, 33, 53us */
    if (hfc.control_mode[COLL]>=CTRL_MODE_POSITION)
    {
        float CurrAltitude, CtrlAltitude, LidarMinAlt, vspeedmin;
        float e_alt;
        bool double_acc = (hfc.ctrl_source==CTRL_SOURCE_RCRADIO || hfc.ctrl_source==CTRL_SOURCE_JOYSTICK) ? true : false;

        /* set minimum above ground altitude as a function of speed */
        LidarMinAlt = CalcMinAboveGroundAlt(hfc.gps_speed);
        
        /* switch between regular (IMU) and lidar based altitude control mode */
        if (!hfc.LidarCtrlMode)
        {
            /* if lidar alt dips below the min limit, switch to lidat ctrl mode.
             * Never do this for takeoff since it needs to get above LidarMinAlt first */
            if (hfc.altitude_lidar < LidarMinAlt && hfc.waypoint_type!=WAYPOINT_TAKEOFF)
                hfc.LidarCtrlMode = true;
        }
        else
        {
            /* if IMU altitude dips below the set altitude, switch back to regular altitude ctrl mode */
            /* or once lidar altitude increases sufficiently above the min lidar altitude */
            if ((hfc.altitude < hfc.ctrl_out[POS][COLL]) || (hfc.altitude_lidar > 1.2f*LidarMinAlt))
                hfc.LidarCtrlMode = false;            
        }

        /* never use lidar ctrl mode in manual lidar ctrl mode */
        if (hfc.rw_cfg.ManualLidarAltitude) {
            hfc.LidarCtrlMode = false;
        }

        /* select regular or lidar based altitude values */
        CurrAltitude = hfc.rw_cfg.ManualLidarAltitude || hfc.LidarCtrlMode ? hfc.altitude_lidar : hfc.altitude;
        CtrlAltitude = hfc.LidarCtrlMode ? LidarMinAlt : hfc.ctrl_out[POS][COLL];

        /* increase vertical down speed limit with an increased horizontal speed */
        vspeedmin = max(pConfig->VspeedDownCurve[1], hfc.rw_cfg.VspeedMin+pConfig->VspeedDownCurve[0]*hfc.gps_speed);
		hfc.pid_CollAlt.COmin = vspeedmin;

//        if (!(hfc.print_counter&0x3f))
//            debug_print("Mode %s currA %4.1f  ctrlA %4.1f alt %4.1f ctrlalt %4.1f\r\n", hfc.LidarCtrlMode ? "Lidar" : "baro ", CurrAltitude, CtrlAltitude, hfc.altitude, hfc.ctrl_out[POS][COLL]);

        if (control_mode_prev[COLL]<CTRL_MODE_POSITION)
        {
            hfc.LidarCtrlMode = false;            
            if (hfc.waypoint_type != WAYPOINT_TAKEOFF)
                hfc.ctrl_out[POS][COLL] = hfc.altitude;
                             PID_SetForEnable(&hfc.pid_CollAlt, CtrlAltitude, CurrAltitude, hfc.ctrl_out[SPEED][COLL]);
        }
        e_alt = CtrlAltitude - CurrAltitude;
        hfc.ctrl_out[SPEED][COLL] = PID_P_Acc(&hfc.pid_CollAlt, CtrlAltitude, CurrAltitude, dT, hfc.LidarCtrlMode && (e_alt>=0), double_acc);  // in lidar mode, ignore acc up
    }
    else
        hfc.ctrl_out[POS][COLL] = hfc.altitude;
        
    if (hfc.control_mode[COLL]>=CTRL_MODE_SPEED)
    {
      if (hfc.control_mode[COLL]==CTRL_MODE_SPEED && hfc.ctrl_source==CTRL_SOURCE_AUTO3D)
    	  SetSpeedAcc(&hfc.ctrl_out[SPEED][COLL], hfc.ctrl_vspeed_3d, pConfig->landing_vspeed_acc, dT);
      if (control_mode_prev[COLL]<CTRL_MODE_SPEED)
      {
//		debug_print("vspeed = %f   GPS = %f  manual = %f\r\n", hfc.ctrl_out[SPEED][COLL], hfc.IMUspeedGroundENU[2], hfc.ctrl_out[RAW][COLL]);
    	  PID_SetForEnable(&hfc.pid_CollVspeed, hfc.ctrl_out[SPEED][COLL], hfc.IMUspeedGroundENU[2], hfc.ctrl_out[RAW][COLL]);
      }

      hfc.ctrl_out[RAW][COLL] = PID(&hfc.pid_CollVspeed, hfc.ctrl_out[SPEED][COLL], hfc.IMUspeedGroundENU[2], dT);
//          debug_print("%4.2f %4.2f %4.2f %5.3f - ", hfc.ctrl_out[RAW][COLL], hfc.ctrl_out[SPEED][COLL], hfc.IMUspeedGroundENU[UP], dT);
    }
    else
    {
      /* RC stick always sets RAW values. In full auto, manual coll needs to be explicitly set here */
      /* this is only for auto takeoff-arm */
      float ctrl = hfc.ctrl_out[RAW][COLL];
      if (hfc.ctrl_source==CTRL_SOURCE_AUTO3D)
      {
//        ctrl = pConfig->CollZeroAngle;
          SetSpeedAcc(&hfc.ctrl_collective_raw, hfc.ctrl_collective_3d, pConfig->collective_man_speed, dT);
          ctrl = hfc.ctrl_collective_raw;
      }
      hfc.ctrl_out[RAW][COLL] = ClipMinMax(ctrl, hfc.pid_CollVspeed.COmin, hfc.pid_CollVspeed.COmax);
      hfc.ctrl_out[POS][COLL] = hfc.altitude;
    }
    hfc.collective_raw_curr = hfc.ctrl_out[RAW][COLL];

    /* for fixed pitch prop, collective drives the throttle, throttle lever gates it */
    if (pConfig->throttle_ctrl==PROP_FIXED_PITCH)
    {
        if (hfc.rw_cfg.AngleCollMixing) {
            hfc.ctrl_out[RAW][COLL] += hfc.rw_cfg.AngleCollMixing*(1/AngleCompensation-1);
        }

        hfc.ctrl_out[RAW][THRO] = hfc.ctrl_out[RAW][COLL];
        /* if lever is low, set throttle to minimum and everything else to 0 to prevent any prop from accidental spinning because of PIDs */
        if (hfc.throttle_value<-0.50f || !hfc.throttle_armed || (hfc.control_mode[COLL]<CTRL_MODE_SPEED && hfc.collective_value<-0.50f)
          || (hfc.waypoint_type == WAYPOINT_TAKEOFF && (hfc.waypoint_stage == FM_TAKEOFF_ARM || hfc.waypoint_stage == FM_TAKEOFF_AUTO_SPOOL)))
        {
            float throttle = pConfig->throttle_values[0];
            if (hfc.throttle_armed && hfc.throttle_value>-0.5f && hfc.waypoint_type == WAYPOINT_TAKEOFF && (hfc.waypoint_stage == FM_TAKEOFF_ARM || hfc.waypoint_stage == FM_TAKEOFF_AUTO_SPOOL))
          	throttle = -0.5;

            ResetIterms();
            hfc.ctrl_out[RAW][THRO] = throttle;
            hfc.ctrl_out[RAW][PITCH] = 0;
            hfc.ctrl_out[RAW][ROLL]  = 0;
            hfc.ctrl_out[RAW][YAW]   = 0;

        }
    }

    /* add offset for fine tuning of RPM */
    hfc.ctrl_out[RAW][THRO] += hfc.throttle_offset;

    /* 1us inhibit individual channels */
    if (pConfig->ctrl_mode_inhibit[THRO] || !hfc.throttle_armed)
        hfc.ctrl_out[RAW][THRO] = pConfig->throttle_values[0];
    if (pConfig->ctrl_mode_inhibit[PITCH])
        hfc.ctrl_out[RAW][PITCH] = hfc.pid_PitchRate.COofs;
    if (pConfig->ctrl_mode_inhibit[ROLL])
        hfc.ctrl_out[RAW][ROLL] = hfc.pid_RollRate.COofs;
    if (pConfig->ctrl_mode_inhibit[YAW])
        hfc.ctrl_out[RAW][YAW] = hfc.pid_YawRate.COofs;
    if (pConfig->ctrl_mode_inhibit[COLL])
        hfc.ctrl_out[RAW][COLL] = hfc.pid_CollVspeed.COofs;

    SetSpeedAcc(&hfc.mixer_in[PITCH], hfc.ctrl_out[RAW][PITCH]* pConfig->control_gains[PITCH], pConfig->servo_speed[PITCH], dT);
    SetSpeedAcc(&hfc.mixer_in[ROLL],  hfc.ctrl_out[RAW][ROLL] * pConfig->control_gains[ROLL],  pConfig->servo_speed[ROLL], dT);
    hfc.mixer_in[YAW]   = hfc.ctrl_out[RAW][YAW]  * pConfig->control_gains[YAW];
    hfc.mixer_in[COLL]	= hfc.ctrl_out[RAW][COLL];
    hfc.mixer_in[THRO]	= hfc.ctrl_out[RAW][THRO];

//    Rotate2D(&hfc.mixer_in[ROLL], &hfc.mixer_in[PITCH], pConfig->RollPitchAngle); // this would interfeer with SetSpeedAcc() just above

    ServoMixer();

    if (pConfig->ccpm_type == MIXERTANDEM) {
        WriteToServoNodeServos(pConfig->num_servo_nodes);
    }
    else {
        if (pConfig->power_node) {
            WriteToPowerNodeServos();
        }
        else {
            WriteToFcmServos();
        }
    }

    if (FCMLinkLive) {
        ProcessFcmLinkLive();
    }

    Playlist_ProcessBottom(&hfc, retire_waypoint);
}

// re-orients sensors within FCM, applies gains and offsets and the re-orients FCM
static void SensorsRescale(float accRaw[3], float gyroRaw[3])
{
    int i;
    float acc1[3];
    float gyro1[3];
    float tmp[3] = {0}; // temporary variable, used for calibration

    // recalculate gyro drifts based on temperature data
    float t = hfc.gyro_temp_lp;
    hfc.gyro_ofs[0] = t*( t*mpu.gyroP_temp_coeffs[0] + mpu.gyroP_temp_coeffs[1] ) +  mpu.gyroP_temp_coeffs[2];
    hfc.gyro_ofs[1] = t*( t*mpu.gyroR_temp_coeffs[0] + mpu.gyroR_temp_coeffs[1] ) +  mpu.gyroR_temp_coeffs[2];
    hfc.gyro_ofs[2] = t*( t*mpu.gyroY_temp_coeffs[0] + mpu.gyroY_temp_coeffs[1] ) +  mpu.gyroY_temp_coeffs[2];

    // remove gyro drift
    for (i=0; i<3; i++) {
        gyroRaw[i] -= hfc.gyro_ofs[i];
    }

    // apply gain to gyro, if first order, take the diagonal only
    if (pConfig->gyro_first_order) {
        for (i=0; i<3; i++) {
            gyro1[i] = mpu.Cg[i][i] * gyroRaw[i];
        }
    }
    else {
        for (i=0; i<3; i++) {
            gyro1[i] = mpu.Cg[i][0]*gyroRaw[0]
                           + mpu.Cg[i][1]*gyroRaw[1]
                           + mpu.Cg[i][2]*gyroRaw[2];
        }
    }

    tmp[0] = gyro1[0];
    tmp[1] = gyro1[1];
    tmp[2] = gyro1[2];

    // re-orient gyro, negative value flips the sign
    for (i=0; i<3; i++)  {
        gyro1[i] = tmp[pConfig->gyro_orient[i]];
        if (pConfig->gyro_orient[i+3]) {
            gyro1[i] = -gyro1[i];
        }
    }

    // apply gain and offset to acc, if first order, take the diagonal only
    if (pConfig->acc_first_order) {
        for (i=0; i<3; i++) {
            acc1[i] = mpu.Ca[i][i] * (accRaw[i]- mpu.aofs[i]);
        }
    }
    else {
        for (i=0; i<3; i++) {
            acc1[i] = mpu.Ca[i][0] * (accRaw[0] - mpu.aofs[0])
                            + mpu.Ca[i][1] * (accRaw[1] - mpu.aofs[1])
                            + mpu.Ca[i][2] * (accRaw[2] - mpu.aofs[2]);
        }
    }

    tmp[0] = acc1[0];
    tmp[1] = acc1[1];
    tmp[2] = acc1[2];
    // re-orient accelerometer, negative value flips sign
    for (i=0; i<3; i++) {
        acc1[i] = tmp[pConfig->acc_orient[i]];
        if (pConfig->acc_orient[i+3]) {
            acc1[i] = -acc1[i];
        }
    }

    // re-orient the entire flight controller, negative value flips the sign
    for (i=0; i<3; i++) {
        int index = pConfig->fcm_orient[i];
        hfc.acc[i]  = acc1[index];
        hfc.gyro[i] = gyro1[index];

        if (pConfig->fcm_orient[i+3]) {
            hfc.acc[i]  = -hfc.acc[i];
            hfc.gyro[i] = -hfc.gyro[i];
        }
    }

    // secondary gyro offset for fine drift removal
    for (i=0; i<3; i++) {
        hfc.gyro[i] -= hfc.gyroOfs[i];
    }

    // low passed gyro averaged value for dynamic gyro calibration
    for (i=0; i<3; i++)
    {
        hfc.gyro_lp_disp[i] = (hfc.gyroFilt[i] + hfc.gyro_lp_disp[i]*4095)/4096;
    }
}

static float UnloadedBatteryLevel(float voltage, const float V2Energy[V2ENERGY_SIZE])
{
  int index1 = ClipMinMax((int)((voltage-3.5f)*50), 0, V2ENERGY_SIZE-1);
  int index2 = Min(index1+1, V2ENERGY_SIZE-1);
  float voltage1 = index1/50.0f+3.5f;
  float voltage2 = index2/50.0f+3.5f;
  float energy1, energy2, energy;

  energy1 = V2Energy[index1];
  energy2 = V2Energy[index2];

  if (energy1==energy2)
    return energy1;

  energy = (energy2-energy1) / (voltage2-voltage1) * (voltage-voltage1) + energy1;
  return energy;
}

static void UpdateBatteryStatus(float dT)
{
    T_Power *p = &hfc.power;
    float I = p->Iesc + p->Iaux;
    float power = I * p->Vmain;
    float dE = power * dT;

    /* do not process if duration is not right */
    if (dT>1 || dT<=0) {
        return;
    }

    /* reset accumulated current when main voltage drops - battery swap */
    if (p->Vmain < 1) {
        p->capacity_used = 0;
    }

    /* integrate current to obtain used up capacity */
    p->capacity_used += I * dT;

    p->power_lp = LP_RC(power, p->power_lp, 0.5f, dT);

    /* when current is below 0.2C, battery is considered unloaded */
    if (I < (0.0002f * hfc.rw_cfg.battery_capacity)) {

        float level = UnloadedBatteryLevel(p->Vmain / Max(1, pConfig->battery_cells), pConfig->V2Energy);
        float Ecurr = p->energy_total * level;

        if (!p->initialized) {
            p->energy_curr = Ecurr;
        }

        /* low pass small changes in voltage, big changes go unfiltered to speed up the initial estimate */
        if (ABS(Ecurr-p->energy_curr) > (0.3f*p->energy_total)) {
            p->energy_curr = Ecurr;
        }
        else {
            p->energy_curr = LP_RC(Ecurr, p->energy_curr, 0.05f, dT);
        }

        power = pConfig->power_typical;  // use typical power consumed to est flight time
    }
    else {
        p->energy_curr -= dE;

        if (p->energy_curr < 0) {
            p->energy_curr = 0;
        }
    }

    /* low-pass power used */
    if (!p->initialized) {
        p->power_curr = pConfig->power_typical;
    }

    p->power_curr = LP_RC(power, p->power_curr, 0.05f, dT);

    /* energy based battery level */
    if (p->energy_total) {
        p->battery_level = p->energy_curr / p->energy_total * 100;
    }

    if (p->power_curr) {
        p->flight_time_left = p->energy_curr / p->power_curr;
    }

    p->initialized = true;
}

static void UpdateBoardInfo(int node_id, unsigned char *pdata)
{
    int board_type = pdata[0];

    if ((board_type < MAX_BOARD_TYPES) && (node_id < MAX_NODE_NUM)) {
        board_info[board_type][node_id].major_version = pdata[1];
        board_info[board_type][node_id].minor_version = pdata[2];
        board_info[board_type][node_id].build_version = pdata[3];

        board_info[board_type][node_id].serial_number2 = pdata[4] << 24;
        board_info[board_type][node_id].serial_number2 |= pdata[5] << 16;
        board_info[board_type][node_id].serial_number2 |= (pdata[6] << 8);
        board_info[board_type][node_id].serial_number2 |= (pdata[7]);
    }
}

static void UpdateBoardPartNum(int node_id, int board_type, unsigned char *pdata)
{
    if ((board_type < MAX_BOARD_TYPES) && (node_id < MAX_NODE_NUM)) {
        board_info[board_type][node_id].serial_number1 = pdata[0] << 24;
        board_info[board_type][node_id].serial_number1 |= pdata[1] << 16;
        board_info[board_type][node_id].serial_number1 |= (pdata[2] << 8);
        board_info[board_type][node_id].serial_number1 |= (pdata[3]);

        board_info[board_type][node_id].serial_number0 = pdata[4] << 24;
        board_info[board_type][node_id].serial_number0 |= pdata[5] << 16;
        board_info[board_type][node_id].serial_number0 |= (pdata[6] << 8);
        board_info[board_type][node_id].serial_number0 |= (pdata[7]);
    }
}

static void UpdateLidar(int node_id, unsigned char *pdata)
{
    float alt;

    // TODO::SP: Always Taking Node1 Lidar - need to look at both
    if (node_id > 0) {
        return;
    }

    lidarPayload[node_id].lidarCount = *(uint32_t *)pdata;
    unsigned int pulse = min(MAX_LIDAR_PULSE, max(0, (lidarPayload[node_id].lidarCount - pConfig->lidar_offset)));
    alt = pulse * 0.001f;
    hfc.altitude_lidar_raw = (alt + 3*hfc.altitude_lidar_raw)*0.25f;
}

static void UpdateCastleLiveLink(int node_id, int message_id, unsigned char *pdata)
{
    float *data= (float *)pdata;
    static int new_data_mask = 0;

    // TODO::SP: Only taking Castle Link from Servo Node id 1 at present
    if (node_id > 0) {
        return;
    }

    switch (message_id) {
        case AVIDRONE_MSGID_CASTLE_0:
            castle_link_live[node_id].battery_voltage = *data++;
            castle_link_live[node_id].ripple_voltage = *data;
            new_data_mask |= (1 << 0);
            break;
        case AVIDRONE_MSGID_CASTLE_1:
            castle_link_live[node_id].current = *data++;
            castle_link_live[node_id].output_power = *data;
            new_data_mask |= (1 << 1);
            break;
        case AVIDRONE_MSGID_CASTLE_2:
            castle_link_live[node_id].throttle = *data++;
            castle_link_live[node_id].rpm = *data;
            new_data_mask |= (1 << 2);
            break;
        case AVIDRONE_MSGID_CASTLE_3:
            castle_link_live[node_id].bec_voltage = *data++;
            castle_link_live[node_id].bec_current = *data;
            new_data_mask |= (1 << 3);
            break;
        case AVIDRONE_MSGID_CASTLE_4:
            castle_link_live[node_id].temperature = *(float *)data;
            new_data_mask |= (1 << 4);
            break;
        default:
            break;
    }

    if (new_data_mask == 0x1F) {
        new_data_mask = 0;

        hfc.power.Iaux   = castle_link_live[node_id].bec_current;
        // TODO::??: Note, removed the use of PowerCoeffs here. Check why they are needed.
        hfc.power.Iesc   = (castle_link_live[node_id].current + 3* hfc.power.Iesc ) * 0.25f;
        hfc.power.Iesc = ClipMinMax(hfc.power.Iesc, 0, hfc.power.Iesc);
        hfc.power.Vmain  = (castle_link_live[node_id].battery_voltage + 3* hfc.power.Vmain) * 0.25f;

        hfc.power.Vesc   = hfc.power.Vmain;
        hfc.power.Vservo = castle_link_live[node_id].bec_voltage;
        hfc.power.Vservo = ClipMinMax(hfc.power.Vservo, 0, hfc.power.Vservo);

        hfc.power.Vaux   = castle_link_live[node_id].bec_voltage;
        hfc.power.Vaux   = ClipMinMax(hfc.power.Vaux, 0, hfc.power.Vaux);
        if (!pConfig->rpm_sensor) {
            hfc.RPM = (castle_link_live[node_id].rpm / pConfig->gear_ratio / pConfig->motor_poles);
        }

        hfc.esc_temp = castle_link_live[node_id].temperature;
        canbus_livelink_avail = 1;
    }
}

static void UpdateCompassData(int node_id, unsigned char *pdata)
{
    signed short raw_x;
    signed short raw_y;
    signed short raw_z;

#ifdef COMPASS_HEADING
    heading[node_id] = *(double *)pdata;
#else

    raw_x = pdata[0] | (pdata[1] << 8);
    raw_y = pdata[2] | (pdata[3] << 8);
    raw_z = pdata[4] | (pdata[5] << 8);

    compass.UpdateRawData(raw_x, raw_y, raw_z);

#endif
}

static void UpdatePowerNodeVI(int node_id, unsigned char *pdata)
{
    hfc.power.Vmain = *(float *)pdata;
    pdata += 4;
    hfc.power.Iesc =  *(float *)pdata;
    power_update_avail = 1;
}

static void UpdatePowerNodeCoeff(int node_id, unsigned char *pdata)
{
    hfc.power.Vcoeff = *(float *)pdata;
    pdata += 4;
    hfc.power.Icoeff =  *(float *)pdata;
}

//
static void can_handler(void)
{
    static int rx_count = 0;
    CANMessage can_rx_message;

    if ((++rx_count % 10) == 0) {
        led4 = !led4;
    }

    while(can.read(can_rx_message)) {

        int node_type = AVIDRONE_CAN_NODETYPE(can_rx_message.id);
        int node_id = (AVIDRONE_CAN_NODEID(can_rx_message.id) -1);
        int message_id = AVIDRONE_CAN_MSGID(can_rx_message.id);
        unsigned char *pdata = &can_rx_message.data[0];

        if (node_type == AVIDRONE_SERVO_NODETYPE) {
            if (message_id == AVIDRONE_MSGID_SERVO_ACK) {
                canbus_ack = 1;
            }
            else if (message_id == AVIDRONE_MSGID_LIDAR) {
                UpdateLidar(node_id, pdata);
            }
            else if ((message_id >= AVIDRONE_MSGID_CASTLE_0) && (message_id <= AVIDRONE_MSGID_CASTLE_4)) {
                UpdateCastleLiveLink(node_id, message_id, pdata);
            }
            else if (message_id == AVIDRONE_MSGID_SERVO_INFO) {
                UpdateBoardInfo(node_id, pdata);
                can_node_found = 1;
            }
            else if (message_id == AVIDRONE_MSGID_SERVO_PN) {
                UpdateBoardPartNum(node_id, PN_SN, pdata);
            }
            else {
                // Unknown
            }
        }
        else if (node_type == AVIDRONE_GPS_NODETYPE) {
            if ((message_id >= AVIDRONE_MSGID_GPS_0) && (message_id <= AVIDRONE_MSGID_GPS_4)) {
                gps.AddGpsData(node_id, message_id, (char *)pdata);
            }
            else if (message_id == AVIDRONE_MSGID_COMPASS_XYZ) {
                UpdateCompassData(node_id, pdata);
            }
            else if (message_id == AVIDRONE_MSGID_GPS_ACK) {
                canbus_ack = 1;
            }
            else if (message_id == AVIDRONE_MSGID_GPS_INFO) {
                UpdateBoardInfo(node_id, pdata);
            }
            else if (message_id == AVIDRONE_MSGID_GPS_PN) {
                UpdateBoardPartNum(node_id, PN_GPS, pdata);
                can_node_found = 1;
            }
            else {
                // Unknown
            }
        }
        else if (node_type == AVIDRONE_PWR_NODETYPE) {
            if (message_id == AVIDRONE_MSGID_PWR_ACK) {
                canbus_ack = 1;
            }
            else if (message_id == AVIDRONE_PWR_MSGID_LIDAR) {
                UpdateLidar(node_id, pdata);
            }
            else if ((message_id >= AVIDRONE_PWR_MSGID_CASTLE_0) && (message_id <= AVIDRONE_PWR_MSGID_CASTLE_4)) {
                UpdateCastleLiveLink(node_id, message_id, pdata);
            }
            else if (message_id == AVIDRONE_MSGID_PWR_V_I) {
                UpdatePowerNodeVI(node_id, pdata);
            }
            else if (message_id == AVIDRONE_MSGID_PWR_INFO) {
                UpdateBoardInfo(node_id, pdata);
            }
            else if (message_id == AVIDRONE_MSGID_PWR_PN) {
                UpdateBoardPartNum(node_id, PN_PWR, pdata);
                can_node_found = 1;
            }
            else if (message_id == AVIDRONE_MSGID_PWR_COEFF) {
                can_power_coeff = 1;
                UpdatePowerNodeCoeff(node_id, pdata);
            }
            else {
                // Unknown
            }
        }
        else {
            // Unknown
        }
    }
}

// TODO::SP: Either extend or remove this..
static void ProcessStats(void)
{
    if ((hfc.print_counter&0x7ff)==6)
    {
        if (hfc.stats.can_power_tx_failed)
        {
            hfc.stats.can_power_tx_errors++;
            hfc.stats.can_power_tx_failed = false;
        }

        if (write_canbus_error > 0) {
            debug_print("CAN write failed messages[%d]\r\n", write_canbus_error);
            hfc.stats.can_servo_tx_errors = write_canbus_error;
        }
    }
}

static void Lidar_Process(FlightControlData *hfc)
{
    if (!hfc->lidar_pulse)
        return;

    /* 1us/mm */
    unsigned int time = hfc->lidar_fall;
    unsigned int d = time - hfc->lidar_rise;
//    debug_print("fall %d %d\r\n", time, d);
    if (pConfig->ground_sensor==GROUND_SENSOR_LIDAR)
    {
        if (d>100000)
            hfc->lidar_counter=0;
        else
            hfc->lidar_counter++;

        if (d<=100000 && hfc->lidar_counter>=10)
        {
            float alt;
            d = min(40000, max(0, d-pConfig->lidar_offset));
            alt = d*0.001f;
            hfc->altitude_lidar_raw = (alt + 3*hfc->altitude_lidar_raw)*0.25f;
    //        debug_print("%5d %6.3f\r\n", d, hfc.altitude_lidar_raw);
        }
        else
            hfc->altitude_lidar_raw = 40;
    }
    else if (pConfig->ground_sensor==GROUND_SENSOR_SONAR)
    {
        hfc->altitude_lidar_raw = d * 0.0001724137931f - pConfig->lidar_offset*0.001f; // 5.8us/mm
//        debug_print("%5d %6.3f\r\n", d, hfc->altitude_lidar_raw);
    }
    hfc->lidar_pulse = false;
}

static void RPM_Process(void)
{
    /* RPM math */
    if (pConfig->rpm_sensor) {
        if (hfc.rpm_pulse) {

            int dms = hfc.rpm_time_ms - hfc.rpm_time_ms_last;

            if (dms<=0 || dms>=1000) {
                hfc.RPM = 0;
            }
            else {
                float rpm = 0;
                if (dms<100) {
                    if (hfc.rpm_dur_us>0) {
                        rpm = 30000000.0f/hfc.rpm_dur_us;   // 2 pulses per rotation
                    }
                }
                else {
                    rpm = 30000.0f/dms;
                }

                hfc.RPM = LP_RC(rpm, hfc.RPM, 1, dms*0.001f);
            }

            hfc.rpm_time_ms_last = hfc.rpm_time_ms;
            hfc.rpm_pulse = false;
        }
        else {
            int dur_ms = hfc.time_ms - hfc.rpm_time_ms_last;
            if (dur_ms>=1000)
            hfc.RPM = 0;
        }
    }
}

/*Function used for making new and on-going compass calibration
 * adjustments to the compass gains and offsets
 * A new Compass Calibration is initiated by going to the COMPASS
 * menu screen on the FCM LCD and pushing the CANCEL button
 *
 *
 * Inputs(not explicitly stated):
 *
 * 1. hfc.compassMin[3]: minimum values measured from compass in x, y and z.
 *                       Reset to -9999 when new calibration initiated.
 *                       Constantly being updated during flight so that
 *                       offsets and gains are always up-to-date.
 *                       If new value is -200 less then current min then
 *                       assume it is an anomaly and IGNORE
 *
 * 2. hfc.compassMax[3]: maximum values measured from compass in x, y and z
 *                       Reset to +9999 when new calibration initiated.
 *                       Constantly being updated during flight so that
 *                       offsets and gains are always up-to-date.
 *                       If new value is +200 more then current max then
 *                       assume it is an anomaly and IGNORE.
 *
 * 3. hfc.IMUorient[PITCH,ROLL]: used to check what orientation the compass is in
 *
 * 4. hfc.comp_pitch[PITCH_COMP_LIMIT]: array of flags to check if a compass
 *                 measurement was made at pitch angles from -PITCH_COMP_LIMIT/2
 *                 to +PITCH_COMP_LIMIT/2.
 *                 Reset to ZEROS when new calibration is Initiated.
 *
 * 5. hfc.comp_roll[ROLL_COMP_LIMIT]: array of flags to check if a compass
 *                 measurement was made at roll angles from -ROLL_COMP_LIMIT/2
 *                 to +ROLL_COMP_LIMIT/2.
 *                 Reset to ZEROS when new calibration is Initiated.
 *
 * Outputs:
 * 1. pConfig->comp_ofs[3] = (hfc.compassMin[i]+hfc.compassMax[i]+1)/2;
 * 2. pConfig->comp_gains[3] = 500.0f/((hfc.compassMax[i]-hfc.compassMin[i])/2.0f);*
 *
 * NOTES:
 * Earth's magnetic field intensity ranges between is 250 and 650 Gauss
 * While it is expected that each axis will have some magnetic offset
 * and gain variation, we expect that the minimum magnetic field
 * measured would be 250*0.90 and maximum would be 650*1.10, therefore:
 * 2*(250*0.90) < compassMax[i] - compassMin[i] < 2*(650*1.10)
 *          450 < compassMax[i] - compassMin[i] < 1430              */
static void CompassCalibration(void)
{
    int i;
    int i_pitch = 0;        //index used for comp_pitch[] flags array
    int i_roll  = 0;        //index used for comp_roll[] flags array
    int mag_range[3] = {0};
    int min_range = 450;
    int max_range = 1430;

    if( hfc.comp_calibrate == NO_COMP_CALIBRATE )
    {
        return;
    }

    /*UPdate Max and Min compass readings on x,y,z axes.
     * - Always use new minimum value if in calibration mode, the assumption here
     *   is that the user knows only to calibrate in an area void of EM interference
     * - If not calibrating, only update max and min if limits are not exceeded*/
    for (i=0; i<3; i++)
    {
        hfc.compass_cal.compassMin[i] = min(hfc.compass_cal.compassMin[i], compass.dataXYZ[i]);
        hfc.compass_cal.compassMax[i] = max(hfc.compass_cal.compassMax[i], compass.dataXYZ[i]);
        mag_range[i] = hfc.compass_cal.compassMax[i] - hfc.compass_cal.compassMin[i];
    }

    for(i = 0; i<3; i++)
    {
        if( mag_range[i] > max_range )
        {
            hfc.compass_cal.compassMax[i] = 0;
            hfc.compass_cal.compassMin[i] = 0;
        }
    }

    /*Set the pitch angle flag index and roll angle flag index to the
     * current pitch and roll angle of the IMU.
     * Re-adjust the indices so that they fit the range of the
     * comp_pitch_flags[] and comp_roll_flags[] arrays since we have
     * to take into account negative angles or angles outside of the
     * PITCH and ROLL angle limits (PITCH_COMP_LIMIT,ROLL_COMP_LIMIT)*/
    i_pitch = ceil(hfc.IMUorient[PITCH]*R2D)*PITCH_COMP_LIMIT/180 + PITCH_COMP_LIMIT/2;
    i_roll  = ceil(hfc.IMUorient[ROLL]*R2D)*ROLL_COMP_LIMIT/360  + ROLL_COMP_LIMIT/2;

    if(i_pitch > PITCH_COMP_LIMIT-1)
    {
        i_pitch = PITCH_COMP_LIMIT - 1;
    }
    else if(i_pitch < 0)
    {
        i_pitch = 0 ;
    }

    if(i_roll > ROLL_COMP_LIMIT-1)
    {
        i_roll = ROLL_COMP_LIMIT - 1;
    }
    else if(i_roll < 0)
    {
        i_roll = 0;
    }


    /*Assign and value of 1 to indicate that we have measured
     * the magnetic field for that given orientation*/
    hfc.comp_pitch_flags[i_pitch] += 1;//hfc.IMUorient[PITCH]*R2D;
    hfc.comp_roll_flags[i_roll]   += 1;//hfc.IMUorient[ROLL]*R2D;


    /*At this point i_pitch and i_roll are used as flags
     * to see if comp_pitch and comp_roll, respectively, have
     * been filled.*/
    for(i = 0; i < PITCH_COMP_LIMIT; i++)
    {
        if(hfc.comp_pitch_flags[i] < NUM_ANGLE_POINTS)
        {
            i_pitch = 0;
            break;
        }
        if(i == (PITCH_COMP_LIMIT-1))
        {
            i_pitch = 1;
        }
    }

    for(i = 0; i < ROLL_COMP_LIMIT; i++)
    {
        if( hfc.comp_roll_flags[i] < NUM_ANGLE_POINTS)
        {
            i_roll = 0;
            break;
        }
        if(i == (ROLL_COMP_LIMIT-1))
        {
            i_roll = 1;
        }
    }

    if ((i_pitch == 1) && (mag_range[0] >= min_range)
    		&& (i_roll == 1) && (mag_range[1] >= min_range)
            && (mag_range[2] >= min_range)) {

        hfc.comp_calibrate = COMP_CALIBRATE_DONE;
    }


    if(hfc.comp_calibrate == COMP_CALIBRATE_DONE)
    {
        for (i=0; i<3; i++)
        {
            hfc.compass_cal.comp_ofs[i] = (hfc.compass_cal.compassMin[i]+hfc.compass_cal.compassMax[i]+1)/2;

            //537.37mGa is the expected magnetic field intensity in Kitchener & Waterloo region
            hfc.compass_cal.comp_gains[i] = 537.37f/((hfc.compass_cal.compassMax[i]-hfc.compass_cal.compassMin[i])/2.0f);
        }

        hfc.compass_cal.valid = 1;
        hfc.compass_cal.version = COMPASS_CAL_VERSION;

        // TODO::SP: Error handling...?
        SaveCompassCalibration(&hfc.compass_cal);
        hfc.comp_calibrate = NO_COMP_CALIBRATE;
    }

}

void do_control()
{
    float accRaw[3];
    float gyroRaw[3];
    int i;
    int ticks;
    int time_ms;
    float dT;
    float baro_altitude_raw_prev;
    int utilization = 0;
    //GpsData gps_data;

    Buttons();

    if (!mpu.readMotion7f_finish(accRaw, gyroRaw, &hfc.gyro_temperature)) {
        //debug_print("MPU timeout\n");
    }

    ticks = Ticks_us_minT(LOOP_PERIOD, &utilization);   // defines loop time in uS

    mpu.readMotion7_start();

    time_ms = GetTime_ms();
    dT = ticks*0.000001f;
    hfc.time_ms    = time_ms;
    hfc.ticks_curr = ticks;
    hfc.ticks_max  = max(hfc.ticks_max, ticks);
    hfc.ticks_lp   = (ticks*64 + 63*hfc.ticks_lp+32)>>6;
    hfc.cpu_utilization_curr = utilization*0.1f;
    hfc.cpu_utilization_lp = (hfc.cpu_utilization_curr + hfc.cpu_utilization_lp*255) * 0.00390625f;

    if (hfc.gyro_temp_lp == 0) {
      hfc.gyro_temp_lp = hfc.gyro_temperature;
    }
    else {
      hfc.gyro_temp_lp = (hfc.gyro_temperature + 511*hfc.gyro_temp_lp)*0.001953125f;
    }

    if (!(hfc.print_counter&0x3f)) {
         // debug_print("util %4.1f%\r\n", hfc.cpu_utilization_lp);
    }

    /* copy and clear the new data flag set by GPS to a new variable to avoid a race */
    //gps_data = gps.GpsUpdate(ticks, &hfc.gps_new_data);
    hfc.gps_new_data = gps.GpsUpdate();

    //if (hfc.gps_new_data) {
        //if ((hfc.print_counter %500) == 0) {
            //debug_print("GPS[%d]\r\n", gps_msg);
    		//debug_print("GPS: lat[%d]:lon[%d], latf[%f]:lonf[%f], latD[%f]:lonD[%f]\r\n",
            //         gps.gps_data_.lat, gps.gps_data_.lon, gps.gps_data_.latF, gps.gps_data_.lonF,
            //         gps.gps_data_.latD, gps.gps_data_.lonD);
        //}
    //}

    if (pConfig->sensor_mode == FLY_ALL_SENSORS) {

        int new_values = (pConfig->num_gps_nodes == 0) ? compass.getRawValues(dT): compass.HaveNewData();

        if (new_values) {
            hfc.compass_heading = compass.GetHeadingDeg(pConfig->comp_orient, hfc.compass_cal.comp_ofs, hfc.compass_cal.comp_gains,
                                                            pConfig->fcm_orient, pConfig->comp_declination_offset,
                                                            hfc.IMUorient[PITCH], hfc.IMUorient[ROLL]);

            if (hfc.compass_heading_lp == 0) {
                hfc.compass_heading_lp = hfc.compass_heading;
            }
            else {
                // hfc.compass_heading_lp = LP_16_Wrap180(hfc.compass_heading, hfc.compass_heading_lp);
                hfc.compass_heading_lp = LP_Wrap180(hfc.compass_heading, hfc.compass_heading_lp, pConfig->heading_avgs);
            }

            // debug_print("Comp: %+5.1f %+5.1f\r\n", hfc.compass_heading, hfc.compass_heading_lp);
            CompassCalibration();
        }
    }

    if (pConfig->baro_enable == 1) {
    	hfc.baro_dT += dT;
    	baro_altitude_raw_prev = hfc.baro_altitude_raw_lp;

    	if (baro.GetTPA(dT, &hfc.baro_temperature, &hfc.baro_pressure, &hfc.baro_altitude_raw)) {   // runs at approximately 32Hz
    		//    hfc.baro_vspeedDF = DerivativeFilter11(hfc.baro_altitude_raw, hfc.baro_derivative_filter)/hfc.baro_dT;

    		if (hfc.baro_altitude_raw_lp < -999) {
    			hfc.altitude_baro = hfc.baro_altitude_raw_lp = baro_altitude_raw_prev = hfc.baro_altitude_raw;
    		}

    		hfc.baro_altitude_raw_lp = LP4_1000(&hfc.lp_baro4, hfc.baro_altitude_raw);
    		hfc.baro_vspeed          = (hfc.baro_altitude_raw_lp - baro_altitude_raw_prev)/hfc.baro_dT;
    		hfc.baro_vspeed_lp = LP4_1000(&hfc.lp_baro_vspeed4,hfc.baro_vspeed);

    		//    hfc.baro_altitude_raw_lp = (hfc.baro_altitude_raw + 7*hfc.baro_altitude_raw_lp)*0.125f;   // about 0.25 second lowpass
    		//    hfc.baro_vspeed          = (hfc.baro_altitude_raw_lp - baro_altitude_raw_prev)/hfc.baro_dT;
    		//    debug_print("T %5.1f P %5.0f Alt %6.2f AltLP %6.2f vs %+3.1f dT %5.3f\r\n", hfc.baro_temperature, hfc.baro_pressure, hfc.baro_altitude_raw, hfc.baro_altitude_raw_lp, hfc.baro_vspeed, hfc.baro_dT);
    		//    debug_print("vs %+5.3f  aB %+5.3f  aBrawLP4 %+5.3f  aBrawLP %+5.3f\n", hfc.IMUspeedGroundENU[2], hfc.altitude, hfc.baro_altitude_rawLP4, hfc.baro_altitude_raw_lp);

    		hfc.baro_dT = 0;
    	}
    }

    /* remap ACC axes into my XYZ (RFU) */
    SensorsRescale(accRaw, gyroRaw);

    /* low-pass sensors */
    /* gyro +/-500 deg/s, acc +/-4G */
    for (i=0; i<3; i++)  {
        hfc.gyroFilt[i] = LP4_1000(&hfc.lp_gyro4[i], hfc.gyro[i]);
    }

    for (i=0; i<3; i++) {
        hfc.accFilt[i]  = LP4_1000(&hfc.lp_acc4[i],  hfc.acc[i]);
    }

    for (i=0; i<3; i++) {
        hfc.accHeliRFU[i] = hfc.accFilt[i];
    }

    Get_Orientation(hfc.SmoothAcc, hfc.accFilt, dT);
    //debug_print("%+5.2f %+5.2f %+5.2f   %+5.2f %+5.2f %+5.2f\n", gyroRaw[PITCH], gyroRaw[ROLL], gyroRaw[YAW], hfc.gyroFilt[PITCH], hfc.gyroFilt[ROLL], hfc.gyroFilt[YAW]);

    MadgwickAHRSupdateIMU(dT, (hfc.gyro[ROLL])*D2R, (hfc.gyro[YAW])*D2R, (hfc.gyro[PITCH])*D2R, 0,0,0);

    if (IMU_Q2PRY_fast(hfc.IMUorient)) {
        /* orient is in rad, gyro in deg */
        hfc.gyroOfs[PITCH] = -PID(&hfc.pid_IMU[PITCH], hfc.SmoothAcc[PITCH]*R2D-hfc.bankPitch,hfc.IMUorient[PITCH]*R2D, dT);
        hfc.gyroOfs[ROLL]  = -PID(&hfc.pid_IMU[ROLL],  hfc.SmoothAcc[ROLL]*R2D+hfc.bankRoll,  hfc.IMUorient[ROLL]*R2D,  dT);
        hfc.gyroOfs[YAW]   = -PID(&hfc.pid_IMU[YAW],   hfc.compass_heading_lp,                hfc.IMUorient[YAW]*R2D,   dT);
        /*
        if (!(hfc.print_counter & 0x3ff)) {
            debug_print("Gyro %+6.3f %+6.3f %+6.3f  IMU %+6.3f %+6.3f %+6.3f  Err %+6.3f %+6.3f %+6.3f\r\n",
            hfc.gyroOfs[PITCH],   hfc.gyroOfs[ROLL], hfc.gyroOfs[YAW],
            hfc.IMUorient[PITCH], hfc.IMUorient[ROLL], hfc.IMUorient[YAW],
            hfc.SmoothAcc[PITCH]*R2D - hfc.IMUorient[PITCH]*R2D, hfc.SmoothAcc[ROLL]*R2D - hfc.IMUorient[ROLL]*R2D, hfc.compass_heading_lp - hfc.IMUorient[YAW]*R2D);
            debug_print("Ofs %+6.3f %+6.3f %+6.3f  Gyro %+6.3f %+6.3f %+6.3f Err %+6.3f %+6.3f %+6.3f\r\n", hfc.gyroOfs[0], hfc.gyroOfs[1],
                              hfc.gyroOfs[2], hfc.gyro_lp_disp[0], hfc.gyro_lp_disp[1], hfc.gyro_lp_disp[2], hfc.SmoothAcc[PITCH]*R2D - hfc.IMUorient[PITCH]*R2D,
                              hfc.SmoothAcc[ROLL]*R2D - hfc.IMUorient[ROLL]*R2D, hfc.compass_heading_lp - hfc.IMUorient[YAW]*R2D);
            debug_print("IMU %+6.3f %+6.3f %+6.3f SmAcc %+6.3f %+6.3f %+6.3f Com %+6.3f %+6.3f\r\n", hfc.IMUorient[PITCH]*R2D, hfc.IMUorient[ROLL]*R2D,
                          hfc.IMUorient[YAW]*R2D, hfc.SmoothAcc[PITCH]*R2D, hfc.SmoothAcc[ROLL]*R2D, hfc.SmoothAcc[YAW]*R2D, hfc.compass_heading_lp, hfc.compass_heading);
            debug_print("%+8.5f %+5.1f %+5.1f %f %f\r\n", hfc.gyroOfs[0], hfc.SmoothAcc[PITCH]*R2D, hfc.IMUorient[PITCH]*R2D, hfc.pid_IMU[PITCH].Kp, hfc.pid_IMU[PITCH].Ki);
        }
        */
        OrientResetCounter();
    }

    /* Accelerometer based vertical speed, GPS vspeed blended in */
    float accGroundENU[3];

    Plane2Ground(hfc.accHeliRFU, hfc.IMUorient, accGroundENU);

    accGroundENU[2] -= 1; // remove gravity

    // high pass filter U2 = T/(1+T)*(U-Uprev+U2prev)
    for (i=0; i<3; i++) {
        hfc.accGroundENUhp[i] = 0.99993896484375f*(accGroundENU[i]-hfc.accGroundENU_prev[i]+hfc.accGroundENUhp[i]);   // T=16384 ~ 14sec 0.99993896484375 T=4096 ~3.4s
        hfc.accGroundENU_prev[i] = accGroundENU[i];
        hfc.IMUspeedGroundENU[i] += pConfig->AccIntegGains[i] * hfc.accGroundENUhp[i]*9.81f*dT;

        // always mix in GPS for X and Y speed
        if (i<2) {
            hfc.IMUspeedGroundENU[i] += 1.0f*dT*(hfc.GPSspeedGroundENU[i] - hfc.IMUspeedGroundENU[i]);    // blend in GPS speed, it drifts if 0.25
        }
        // blend in GPS vertical speed with IMU vertical speed
        else if (pConfig->gps_vspeed == 1 ) {
            hfc.IMUspeedGroundENU[2] += pConfig->GPSVspeedWeight*dT*(hfc.GPSspeedGroundENU[2] - hfc.IMUspeedGroundENU[2]);    // blend in GPS speed
        }
        // blend in Baro vertical speed with IMU vertical speed
        else if ((pConfig->gps_vspeed) == 2 && (pConfig->baro_enable)) {
            hfc.IMUspeedGroundENU[2] += pConfig->BaroVspeedWeight*dT*(hfc.baro_vspeed_lp - hfc.IMUspeedGroundENU[2]);    // blend in baro vspeed
        }
        // blend in GPS and Baro vertical speed with IMU vertical speed
        else if ((pConfig->gps_vspeed == 3) && (pConfig->baro_enable)) {
            hfc.IMUspeedGroundENU[2] += pConfig->GPSVspeedWeight *dT*(hfc.GPSspeedGroundENU[2] - hfc.IMUspeedGroundENU[2])
                            + pConfig->BaroVspeedWeight*dT*(hfc.baro_vspeed_lp       - hfc.IMUspeedGroundENU[2]);    // blend in GPS and baro vspeed
        }
        // use only GPS for vertical speed
        else if (pConfig->gps_vspeed == 4 ) {
            hfc.IMUspeedGroundENU[2] = hfc.GPSspeedGroundENU[2];
        }
        // use only Baro for vertical speed
        else if ((pConfig->gps_vspeed == 5) && (pConfig->baro_enable)) {
            hfc.IMUspeedGroundENU[2] = hfc.baro_vspeed_lp;
        }
        else {
            // blend in GPS vertical speed with IMU vertical speed (hfc.config.gps_vspeed == 1 )
            hfc.IMUspeedGroundENU[2] += pConfig->GPSVspeedWeight*dT*(hfc.GPSspeedGroundENU[2] - hfc.IMUspeedGroundENU[2]);    // blend in GPS speed
        }
    }

    /*
    if (!(hfc.print_counter & 0x3f)) {
        debug_print("Z %+5.3f UR %+5.3f Uhp %+5.3f speed %+5.3f\r\n", hfc.accHeliRFU[2], accGroundU, hfc.accGroundUhp, hfc.speedGroundU);
    }
    if (!(hfc.print_counter & 0x3f)) {
        debug_print("E %+5.2f N %+5.2f U %+5.2f\n", hfc.IMUspeedGroundENU[0], hfc.IMUspeedGroundENU[1], hfc.IMUspeedGroundENU[2]);
    }
    if (!(hfc.print_counter & 0x3f)) {
        debug_print("E %+5.3f N %+5.3f U %+5.3f  E %+5.3f N %+5.3f U %+5.3f\n", hfc.accGroundENUhp[0], hfc.accGroundENUhp[1], hfc.accGroundENUhp[2], hfc.IMUspeedGroundENU[0], hfc.IMUspeedGroundENU[1], hfc.IMUspeedGroundENU[2]);
    }
    */

    /* help baro-altitude using vertical speed */
    if (pConfig->baro_enable == 1) {
    	hfc.altitude_baro += hfc.IMUspeedGroundENU[2] * dT;
    	hfc.altitude_baro += pConfig->BaroAltitudeWeight*dT*(hfc.baro_altitude_raw_lp - hfc.altitude_baro);    // blend in baro vspeed
    	//hfc.altitude_baro += 0.25f*dT*(hfc.baro_altitude_raw_lp - hfc.altitude_baro);    // blend in baro vspeed
    }

    /*
    hfc.accUp += hfc.accGroundENUhp[2]*9.81f;
    if (!(hfc.print_counter & 0x7)) {
        hfc.accUp/=8;
        debug_print("T %d acc %+5.3f vs %+5.3f altIMU %5.3f altB %5.3f\n", hfc.time_ms, hfc.accUp, hfc.IMUspeedGroundENU[2], hfc.altitude_baro, hfc.baro_altitude_raw);
        hfc.accUp = 0;
    }*/

    /* horizontal position is an integral of horizontal speed */
    /* convert horizontal speeds in m/s to Earth-rotational latitude/longitude speeds */
    /* blend in GPS position */
    /* integrate E/N speed into delta Lat/Lon */

    float dLat = hfc.GPSspeedGroundENU[1] * dT * DPM;   // lat
    float dLon = hfc.GPSspeedGroundENU[0] * dT * DPM / COSfD((float)hfc.positionLatLon[0]);   // lon

    /* adjust position */
    hfc.positionLatLon[0] += (double)dLat;
    hfc.positionLatLon[1] += (double)dLon;
    
    /* if GPS detects a glitch, set the blending factor to the long value, otherwise decay towards the regular blanding value */
    if (gps.glitch_) {
        hfc.Pos_GPS_IMU_Blend = pConfig->Pos_GPS_IMU_BlendGlitch;
    }
    else if (hfc.Pos_GPS_IMU_Blend>pConfig->Pos_GPS_IMU_BlendReg) {
        hfc.Pos_GPS_IMU_Blend -= dT;
    }

    /* if GPS coordinates are more than 222m away from the current pos, just reset it, otherwise blend the current position with GPS */
    if (gps.gps_data_.fix) {
        double gps_latitude  = gps.gps_data_.latD;
        double gps_longitude = gps.gps_data_.lonD;
        if ((ABS(gps_latitude-hfc.positionLatLon[0]) > 0.002) || (ABS(gps_longitude-hfc.positionLatLon[1]) > 0.002)) {
            hfc.positionLatLon[0] = gps_latitude;
            hfc.positionLatLon[1] = gps_longitude;
        }
        else {
            /* HspeedGPSaccBlend could be adaptive with speed - more gps at high speed, more acc at low speeds */
            if (hfc.Pos_GPS_IMU_Blend > 0) {
                hfc.positionLatLon[0] += (1/hfc.Pos_GPS_IMU_Blend) * dT * (gps_latitude-(hfc.positionLatLon[0]));
                hfc.positionLatLon[1] += (1/hfc.Pos_GPS_IMU_Blend) * dT * (gps_longitude-(hfc.positionLatLon[1]));
            }
        }
    }
  
    /* offset baro based altitude to match GPS */
    hfc.altitude = hfc.altitude_baro + hfc.altitude_ofs;
    hfc.gps_to_home[2] = hfc.altitude - hfc.home_pos[2];

    if (hfc.gps_new_data) {
    	double latitude  = gps.gps_data_.latD;
    	double longitude = gps.gps_data_.lonD;

        hfc.altitude_gps = gps.gps_data_.altitude;
        //hfc.altitude = hfc.altitude_gps;

        hfc.gps_to_home[0] = DistanceCourse(latitude, longitude, hfc.home_pos[0], hfc.home_pos[1], &hfc.gps_to_home[1]);
        //debug_print("D %4.1f C %+5.1f  \r\n", hfc.gps_to_ref[0], hfc.gps_to_ref[1]);

        /* if we have fix and a new position data, run the gradient descent algo
        ** to bring baro-altitude in sync with gps altitude */
        if (gps.gps_data_.fix > GPS_FIX_NONE && gps.gps_data_.PDOP < 250)
        {
            float dTGPS = 0.001f * ((int)(time_ms - hfc.tGPS_prev));
            hfc.tGPS_prev = time_ms;

            /* initialize altitude only the first time */
            if (!hfc.gps_alt_initialized) {
                hfc.altitude_ofs = hfc.altitude_gps - hfc.altitude_baro;
                hfc.gps_alt_initialized = true;
                gps.glitches_ = 0;
            }
            else if (dTGPS < 1 && dTGPS>0) {
                hfc.altitude_ofs +=  dTGPS * (hfc.altitude_gps - hfc.altitude) / hfc.AltitudeBaroGPSblend;
                /* decay the initial blending factor into the final value */
                hfc.AltitudeBaroGPSblend = min(hfc.AltitudeBaroGPSblend+dTGPS, pConfig->AltitudeBaroGPSblend_final);
                //debug_print("%8d\t%5.3f\t%f\t%f\t%f\t%f\t%f\r\n", time_ms, dTGPS, hfc.AltitudeBaroGPSblend, hfc.altitude_ofs, hfc.altitude_baro, hfc.altitude_gps, hfc.altitude);
            }
        }
      
        /* auto-set home for the first time after GPS is locked, it needs to be locked for at least 15sec */
        if (hfc.home_pos[2] == 99999 && gps.gps_data_.fix>GPS_FIX_NONE && gps.gps_data_.PDOP<200 && hfc.AltitudeBaroGPSblend>25) {
            telem.SetHome();
        }

        hfc.gps_heading  = gps.gps_data_.courseC;
        hfc.gps_speed    = gps.gps_data_.HspeedC;
        //debug_print("GPS s/c %5.1f C %+5.1f  COOR s/c %5.1f C %+5.1f\r\n", gps_speed, gps_heading, hfc.gps_speed, hfc.gps_heading);
      
        /* split GPS speed into east and north components */
        hfc.GPSspeedGroundENU[0] = gps.gps_data_.speedENU[0];
        hfc.GPSspeedGroundENU[1] = gps.gps_data_.speedENU[1];
        hfc.GPSspeedGroundENU[2] = gps.gps_data_.speedENU[2];
        //debug_print("GPS time %d %f %f %f\n", hfc.time_ms, gps.gps_data_.speedENU[0], gps.gps_data_.speedENU[1], gps.gps_data_.speedENU[2]);
    }

    PrintOrient();

    if (pConfig->servo_raw) {
        ServoUpdateRAW(dT);
    }
    else {
        ServoUpdate(dT);
    }

    if (hfc.msg2ground_count && !telem.IsTypeInQ(TELEMETRY_MSG2GROUND)) {
        telem.Generate_Msg2Ground();
        telem.AddMessage((unsigned char*)&hfc.telemMsg2ground, sizeof(T_Telem_Msg2Ground), TELEMETRY_MSG2GROUND, 6);
        hfc.msg2ground_count--;
    }

    /* generate a new telemetry system message every 1s or so, only if is not still in the output Q */
    if ((hfc.print_counter&0x3ff)==7 && !telem.IsTypeInQ(TELEMETRY_SYSTEM)) {
        telem.Generate_System2(time_ms);
        telem.AddMessage((unsigned char*)&hfc.telemSystem2, sizeof(T_Telem_System2), TELEMETRY_SYSTEM, 5);
        //debug_print("%d %d\n", hfc.ticks_max, GetTime_ms());
        //perf_printf();
        hfc.ticks_max = 0;
    }
  
    /* TCPIP packet confirmation */
    if (hfc.tcpip_confirm && !telem.IsTypeInQ(TELEMETRY_TCPIP)) {
        telem.Generate_Tcpip7();
        telem.AddMessage((unsigned char*)&hfc.telemTcpip7, sizeof(T_Telem_TCPIP7), TELEMETRY_TCPIP, 4);
        hfc.tcpip_confirm = false;
    }

    /* capture streaming data and generate a new packet once data cache is full */
    if (Streaming_Process(&hfc)) {
        /* Push the new data to the output only if the previous packet already
         ** has been sent, since we have only one data cache. Otherwise the current
         ** data cache is dropped, this is not expected to be happening under normal conditions */

        if (!telem.IsTypeInQ(TELEMETRY_DATASTREAM3)) {
            int size = telem.Generate_Streaming();
            telem.AddMessage((unsigned char*)&hfc.telemDataStream3, size, TELEMETRY_DATASTREAM3, 3);
        }
    }
  
    /* if new GPS RMS message arrived and it is not in the serial Q already, generate it and push it to the temetry output */
    if (hfc.gps_new_data) {
        if (!telem.IsTypeInQ(TELEMETRY_GPS)) {
            telem.Generate_GPS1(time_ms);
            telem.AddMessage((unsigned char*)&hfc.telemGPS1, sizeof(T_Telem_GPS1), TELEMETRY_GPS, 2);
        }
    }

    /* generate aircraft config message every 8s or so, only if is not still in the output Q */
    if ((hfc.print_counter&0x1fff)==9 && !telem.IsTypeInQ(TELEMETRY_AIRCRAFT_CFG)) {
        telem.AddMessage((unsigned char*)&hfc.aircraftConfig, sizeof(T_AircraftConfig), TELEMETRY_AIRCRAFT_CFG, 1);
    }

    /* if telemetry output Q is empty, generate the Ctrl telemetry message and push it out */
    if (hfc.telem_ctrl_time >= hfc.telem_ctrl_period && telem.IsEmpty()) {
        telem.Generate_Ctrl0(time_ms);
        telem.AddMessage((unsigned char*)&hfc.telemCtrl0, sizeof(T_Telem_Ctrl0), TELEMETRY_CTRL, 0);
        hfc.telem_ctrl_time = 0;
    }

    hfc.telem_ctrl_time += ticks;	// in uS

    telem.Update();

    if (hfc.display_mode != DISPLAY_SPLASH) {
        myLcd.Update();
    }

    ProcessStats();

    // TODO::SP: Assumption here is that lidar not from servo and not from power,
    // then must be onboard FCM. Should be better handled!
    if ((pConfig->LidarFromServo == 0) && (pConfig->LidarFromPowerNode == 0)) {
        Lidar_Process(&hfc);
    }

   RPM_Process();

   telem.ProcessInputBytes(telemetry);

   AutoReset();

   if (pConfig->num_gps_nodes > 0) {
       GpsHeartbeat(pConfig->num_gps_nodes);
   }

   if (pConfig->num_power_nodes > 0) {
       PowerNodeHeartbeat(pConfig->num_power_nodes);
   }

   if ((hfc.print_counter % 100) == 0) {
       // Every 100ms update battery status, if new data available
       if (canbus_livelink_avail || power_update_avail){
           UpdateBatteryStatus(dT);
           canbus_livelink_avail = 0;
           power_update_avail = 0;
       }
   }

    hfc.gps_new_data = false;
    hfc.print_counter++;
}

static void Lidar_fall(void)
{
    hfc.lidar_fall = GetTime_us();
    hfc.lidar_pulse = true;
}

static void Lidar_rise(void)
{
    hfc.lidar_rise = GetTime_us();
}

static void RPM_rise(void)
{
    hfc.rpm_dur_us = Ticks2us(hfc.rpm_ticks);
    hfc.rpm_ticks = Ticks1();
    hfc.rpm_time_ms = hfc.time_ms;
    hfc.rpm_pulse = true;
}

static void Servos_Init(void)
{
    // TODO::SP: Prob don't need to support RPM sensor moving forward....
    if (pConfig->rpm_sensor) {
        if ((pConfig->ccpm_type==CCPM_HEX || pConfig->ccpm_type==CCPM_QUAD
                        || pConfig->ccpm_type==CCPM_OCTO ) && pConfig->fcm_servo) {
            //debug_print("Cannot use RPM sensor on multicopter when servo's driven from FCM\n");
        }
        else {
            rpm = new InterruptIn(p21);
            if (rpm) {
                rpm->mode(PullUp);
                rpm->rise(&RPM_rise);
            }
        }
    }
    else {
        FCM_SERVO_CH6 = new PwmOut(p21);
    }

    if (pConfig->fcm_linklive_enable) {
        FCMLinkLive = new DigitalInOut(p26);
        FCMLinkLive->output();
        FCMLinkLive->write(1);
        hfc.fcm_linkLive_counter = 0;
        linklive  = new InterruptIn(p26);
    }
    else {
        FCM_SERVO_CH1 = new PwmOut(p26);
    }

    if (pConfig->fcm_servo) {
        if (FCM_SERVO_CH6) {
          FCM_SERVO_CH6->period_us(pConfig->pwm_period);
        }

        FCM_SERVO_CH5.period_us(pConfig->pwm_period);
        FCM_SERVO_CH4.period_us(pConfig->pwm_period);
        FCM_SERVO_CH3.period_us(pConfig->pwm_period);
        FCM_SERVO_CH2.period_us(pConfig->pwm_period);

        if (FCM_SERVO_CH1) {
            FCM_SERVO_CH1->period_us(pConfig->pwm_period);
        }

        if (FCM_SERVO_CH6) {
          FCM_SERVO_CH6->pulsewidth_us(1500);
        }

        FCM_SERVO_CH5.pulsewidth_us(1500);
        FCM_SERVO_CH4.pulsewidth_us(1500);
        FCM_SERVO_CH3.pulsewidth_us(1500);
        FCM_SERVO_CH2.pulsewidth_us(1500);

        if (FCM_SERVO_CH1) {
            FCM_SERVO_CH1->pulsewidth_us(1000);
        }
    }
}

volatile static int rx_in=0;
volatile static bool have_config = false;
/**
  * @brief  ConfigRx callback handler.
  * @param  none
  * @retval none
  */
static void ConfigRx(void)
{

    while (serial.readable() && (rx_in < MAX_CONFIG_SIZE)) {
        pRamConfigData[rx_in++] = serial._getc();
    }

    have_config = true;
}

/**
  * @brief  Process user commands from USB Serial Port.
  * @param  received command byte
  * @retval none
  */
static void ProcessUserCmnds(char c)
{
    char request[20] = {0};
    ConfigData *pRamConfigData = (ConfigData *)&ram_config;

    // 'L' == Load Request
    if (c == 'L') {

        led1 = led2 = led3 = led4 = 1;

        usb_print("OK");

        // Wait for Load Type - config
        serial.scanf("%19s", request);
        if (strcmp(request, "config_upload") == 0) {
            // Clear current RamConfig in preparation for new data.
            memset((uint8_t *)&ram_config, 0xFF, MAX_CONFIG_SIZE);

            serial.attach(&ConfigRx);	// This handles incoming configuration file

            have_config = false;

            // Clear chars - Dummy Read on Port
            while (serial.readable()) {
                volatile int rx = serial._getc();
            }

            usb_print("ACK");	// Informs Host to start transfer

            while (!have_config) {}

            unsigned char *pData = (unsigned char *)&ram_config;
            pData += sizeof(ConfigurationDataHeader);
            // CRC data and check.
            if (pRamConfigData->header.checksum
                    == crc32b(pData, (sizeof(ConfigData) - sizeof(ConfigurationDataHeader)))) {

                if (SaveNewConfig() == 0) {
                    usb_print("ACK");	// Informs Host, all done

                    FCM_NOTIFY_CFG_UPDATED();

                    // NOTE::SP: NO RETURN FROM HERE. THIS GENERATES A SOFTWARE RESET OF THE LPC1768
                    //           WHICH CAUSES THE NEW CONFIGURATION TO BE LOADED.
                    NVIC_SystemReset();
                }
                else {
                    usb_print("NACK");	// Informs Host, Error
                }
            }
            else {
                usb_print("NACK");	// Informs Host, Error
            }
        }
        else if (strcmp(request, "config_dlload") == 0) {
            // Download fcm configuration to requester
            const int16_t block_size = 64;
            uint8_t *pData = (uint8_t*)pConfig;
            int16_t block_num = (sizeof(ConfigData) / block_size) +1;

            do {
                if (block_num == 1) {
                    serial.writeBlock(pData, (sizeof(ConfigData) % block_size));
                }
                else {
                    serial.writeBlock(pData, block_size);
                }

                pData += block_size;
            } while (--block_num >= 1);
        }
        else {
            // Unknown command
            usb_print("NACK");
        }
    }
    else if (c == 'C') {
        // Calibrate requested
        usb_print("OK");

        int timeout = CAN_TIMEOUT;
        CANMessage can_tx_message;

        // Wait for Load Type - config
        serial.scanf("%19s", request);
        if (strcmp(request, "read") == 0) {
            can_tx_message.len = 0;
            can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_PWR_NODETYPE,
                                                    DEFAULT_NODE_ID, AVIDRONE_MSGID_PWR_COEFF);

            while(!can_power_coeff && --timeout) {
                if (!can.write(can_tx_message)) {
                    ++write_canbus_error;
                }
                wait_ms(20);
            }

            if (!can_power_coeff) {
                usb_print("ERROR");
            }
            else {
                usb_print("Vcoeff[%f], Icoeff[%f]", hfc.power.Vcoeff, hfc.power.Icoeff);
            }
            can_power_coeff = 0;
        }
        else if (strcmp(request, "start") == 0) {
            // Issue a request to start power node calibration
            can_power_coeff = 0;
            timeout = 2;
            can_tx_message.len = 2;
            can_tx_message.data[0] = PWM_NO_CHANNEL;
            can_tx_message.data[1] = CALIBRATE_PWR_NODE;

            can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_PWR_NODETYPE,
                                                DEFAULT_NODE_ID, AVIDRONE_MSGID_PWR_CFG);

            if (!can.write(can_tx_message)) {
                ++write_canbus_error;
            }

            timeout = 60; // seconds
            while(--timeout) {
                WDT_Kick();
                wait(1.0f);
            }

            // Now poll to see if it completed successfully
            can_tx_message.len = 0;
            can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_PWR_NODETYPE,
                                                    DEFAULT_NODE_ID, AVIDRONE_MSGID_PWR_COEFF);

            timeout = CAN_TIMEOUT;
            while(!can_power_coeff && --timeout) {
                if (!can.write(can_tx_message)) {
                    ++write_canbus_error;
                }
                wait_ms(20);
            }

            if (!can_power_coeff) {
                usb_print("ERROR");
            }
            else {
                usb_print("Vcoeff[%f], Icoeff[%f]", hfc.power.Vcoeff, hfc.power.Icoeff);
            }
            can_power_coeff = 0;
        }
    }
    else if (c == 'M') {
        // System Manifest
        IAP iap;
        int *fcm_serial_num;
        fcm_serial_num = iap.read_serial();

        usb_print("Type[FCM], Node[%d], Version[%02x:%02x:%02x],  SERIAL[%08x:%08x:%08x:%08x]\r\n",
                            DEFAULT_NODE_ID, MAJOR_VERSION, MINOR_VERSION, BUILD_VERSION,
                            fcm_serial_num[0], fcm_serial_num[1], fcm_serial_num[2], fcm_serial_num[3]);

        usb_print("\r\nCANBus Board Info..\r\n");
        for (int i = 0; i < pConfig->num_servo_nodes; i++) {
            usb_print("Type[ SN], Node[%d], Version[%02x:%02x:%02x], SERIAL[%08x:%08x:%08x]\r\n", i+1,
                        board_info[PN_SN][i].major_version, board_info[PN_SN][i].minor_version, board_info[PN_SN][i].build_version,
                        board_info[PN_SN][i].serial_number2, board_info[PN_SN][i].serial_number1, board_info[PN_SN][i].serial_number0);
        }

        for (int i = 0; i < pConfig->num_gps_nodes; i++) {
            usb_print("Type[GPS], Node[%d], Version[%02x:%02x:%02x], SERIAL[%08x:%08x:%08x]\r\n", i+1,
                        board_info[PN_GPS][i].major_version, board_info[PN_GPS][i].minor_version, board_info[PN_GPS][i].build_version,
                        board_info[PN_GPS][i].serial_number2, board_info[PN_GPS][i].serial_number1, board_info[PN_GPS][i].serial_number0);
        }

        for (int i = 0; i < pConfig->num_power_nodes; i++) {
            usb_print("Type[PWR], Node[%d], Version[%02x:%02x:%02x], SERIAL[%08x:%08x:%08x]\r\n", i+1,
                        board_info[PN_PWR][i].major_version, board_info[PN_PWR][i].minor_version, board_info[PN_PWR][i].build_version,
                        board_info[PN_PWR][i].serial_number2, board_info[PN_PWR][i].serial_number1, board_info[PN_PWR][i].serial_number0);
            usb_print("---Vcoeff[%f], Icoeff[%f]\r\n", hfc.power.Vcoeff, hfc.power.Icoeff);
        }
    }
}

void ProcessButtonSelection()
{
//    if (hfc.display_mode==DISPLAY_SPLASH || hfc.display_mode==DISPLAY_STATUS || hfc.display_mode==DISPLAY_POWER)
    // Only allow arm and disarm function on SPLASH screen
    if (hfc.display_mode==DISPLAY_SPLASH)
    {
        if (hfc.throttle_armed)
        {
            if (hfc.btnSelectCounter>500)
                telem.Disarm();
        }
        else {
            telem.Arm();			// old code was only Arm
        }

        hfc.resetandarm_req = 1;					// Request reset and arm event
        hfc.resetandarm_time = GetTime_ms();		// capture current time to compare against
        /*
         *
        instead of Arm right after button has been pushed held and released as in the above else
        the new sequence should be:
        1 - delay 500mS to allow for any movement to stop
        2 - reset IMU
        3 - Arm(&hfc)

        Should we just wait here or set a flag

        hfc.resetandarm_req
        and set counter to current time in ms
        hfc.resetandarm_time;

        in the main loop we can check flag and clear once time > 500ms of delay.
        main loop will then reset imu, arm and then clear the flag.

        New concept code is shown below.
        */
        /*
{
		if(hfc.resetandarm_req && (hfc.time_ms - hfc.resetandarm_time) > 500)
		{
		    ResetIMU(&hfc, true);
		                Arm(&hfc);
            hfc.resetandarm_req = 0;    //RVW
		}
} */


        ArmedLed = !hfc.throttle_armed;
        hfc.waypoint_type = WAYPOINT_NONE;
    }
    else
    if (hfc.display_mode==DISPLAY_GPS1)
        gps.SetNextChannel();
    else
    if (hfc.display_mode==DISPLAY_GPS2)
        telem.SetHome();
    else
    if (hfc.display_mode==DISPLAY_CALIB)
    {
        telem.ResetIMU(false);
    }
    else
    if (hfc.display_mode==DISPLAY_COMPASS)
    {
        int i = 0;

        if( hfc.comp_calibrate == NO_COMP_CALIBRATE ) {
			hfc.compass_cal.compassMin[0] = hfc.compass_cal.compassMin[1] = hfc.compass_cal.compassMin[2] = 9999;
			hfc.compass_cal.compassMax[0] = hfc.compass_cal.compassMax[1] = hfc.compass_cal.compassMax[2] = -9999;

			for(i = 0; i < PITCH_COMP_LIMIT; i++)
			{
				hfc.comp_pitch_flags[i] = 0;
			}

			for(i = 0; i < ROLL_COMP_LIMIT; i++)
			{
				hfc.comp_roll_flags[i] = 0;
			}

			hfc.comp_calibrate = COMP_CALIBRATING;
			//debug_print("Starting Compass Calibration\r\n");
        }
        else {
        	hfc.comp_calibrate = COMP_CALIBRATE_DONE;
        	//debug_print("Compass Calibration Finished\r\n");
        }
    }
}

void button_Menu(void)
{
    hfc.display_mode++;
    if (hfc.display_mode>=DISPLAY_PAGES)
        hfc.display_mode = 0;
}

static void Buttons()
{
    bool btn = btnMenu;
    if (btn && !hfc.btnMenuPrev)
      button_Menu();
    hfc.btnMenuPrev = btn;
    if (!btn)
        hfc.btnMenuCounter++;
    else
        hfc.btnMenuCounter=0;
    
    btn = btnSelect;
    if (btn && !hfc.btnSelectPrev)
      ProcessButtonSelection();
    hfc.btnSelectPrev = btn;
    if (!btn)
        hfc.btnSelectCounter++;
    else
        hfc.btnSelectCounter=0;
}

static void PrintOrient()
{
    char str[30];
    char *pstr = str;
  
    if (!(hfc.display_mode == DISPLAY_SPLASH) && (hfc.print_counter&0x1f) == 8 ) {
        //debug_print("P %+5.1f R %+5.1f Y %+5.1f    ", SmoothAcc[PITCH]*R2D, SmoothAcc[ROLL]*R2D, SmoothAcc[YAW]*R2D);
        //debug_print("P %+5.1f R %+5.1f Y %+5.1f\r\n", hfc.IMUorient[PITCH]*R2D, hfc.IMUorient[ROLL]*R2D, hfc.IMUorient[YAW]*R2D);
        //debug_print("P %+5.1f R %+5.1f Y %+5.1f\r\n", gBfiltered[PITCH], gBfiltered[ROLL], gBfiltered[YAW]);
        pstr+= PRINTf(pstr, hfc.IMUorient[PITCH]*R2D, 1, 1, 0);
        *pstr++ = ' ';

        pstr+= PRINTf(pstr, hfc.IMUorient[ROLL]*R2D, 1, 1, 0);
        *pstr++ = ' ';

        pstr+= PRINTf(pstr, hfc.IMUorient[YAW]*R2D, 1, 1, 0);
        *pstr++ = ' ';
        *pstr++ = ' ';
        *pstr++ = 0;

        str[14] = 0;
        myLcd.SetLine(5, str, 0);
    }

    if ((hfc.print_counter & 0x1f) == 8) {
        led1 = hfc.throttle_armed;
        led2 = hfc.throttle_armed;
        led3 = hfc.throttle_armed;
        //led4 = hfc.throttle_armed;
        ArmedLed = !hfc.throttle_armed;
    }
}

int InitCanServoNodes(int num_servo_nodes)
{
    CANMessage can_tx_message;
    int timeout = CAN_TIMEOUT;

    // ping the board to determine -
    //  - a) Is the board connected and
    //  - b) Read board information (Firmware Version, Hardware Serial Num)
    for (int i = 0; i < num_servo_nodes; i++) {
        can_node_found = 0;
        can_tx_message.len = 0;
        can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_SERVO_NODETYPE,
                                                (DEFAULT_NODE_ID + i), AVIDRONE_MSGID_SERVO_INFO);
        while(!can_node_found && --timeout) {
            if (!can.write(can_tx_message)) {
                ++write_canbus_error;
            }
            wait_ms(20);
        }

        if (!can_node_found) {
            return 0;
        }
    }
    return 1;
}

int InitCanGpsNodes(int num_gps_nodes)
{
    CANMessage can_tx_message;
    int timeout = CAN_TIMEOUT;

    // ping the board to determine -
    //  - a) Is the board connected and
    //  - b) Read board information (Firmware Version, Hardware Serial Num)
    for (int i = 0; i < num_gps_nodes; i++) {
        can_node_found = 0;
        can_tx_message.len = 0;
        can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_GPS_NODETYPE,
                                                (DEFAULT_NODE_ID + i), AVIDRONE_MSGID_GPS_INFO);
        while(!can_node_found && --timeout) {
            if (!can.write(can_tx_message)) {
                ++write_canbus_error;
            }
            wait_ms(20);
        }

        if (!can_node_found) {
            return 0;
        }
    }
    return 1;
}

int InitCanPowerNodes(int num_power_nodes)
{
    CANMessage can_tx_message;
    int timeout = CAN_TIMEOUT;

    // ping the board to determine -
    //  - a) Is the board connected and
    //  - b) Read board information (Firmware Version, Hardware Serial Num)
    for (int i = 0; i < num_power_nodes; i++) {
        can_node_found = 0;
        can_tx_message.len = 0;
        can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_PWR_NODETYPE,
                                                (DEFAULT_NODE_ID + i), AVIDRONE_MSGID_PWR_INFO);
        while(!can_node_found && --timeout) {
            if (!can.write(can_tx_message)) {
                ++write_canbus_error;
            }
            wait_ms(20);
        }

        if (!can_node_found) {
            return 0;
        }
    }
    return 1;
}

int ConfigureCanServoNodes(int num_servo_nodes)
{
    CANMessage can_tx_message;
    int timeout = CAN_TIMEOUT;

    // Configure Servo board. Board will 'ACK' message to indicate success.
    for (int i = 0; i < num_servo_nodes; i++) {
        canbus_ack = 0;
        can_tx_message.len = 2;
        // Activates Channels for Castle Link/Throttle, A, B, C
        // TODO::SP: This will ultimately come from configuration file
        can_tx_message.data[0] = PWM_CHANNEL_2 | PWM_CHANNEL_3 | PWM_CHANNEL_4;
        can_tx_message.data[1] = LIDAR_ACTIVE; /* Link Live is automatically enabled on Tandem Servos */

        can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_SERVO_NODETYPE,
                                               (DEFAULT_NODE_ID + i), AVIDRONE_MSGID_SERVO_CFG);

        while(!canbus_ack && --timeout) {
            if (!can.write(can_tx_message)) {
                ++write_canbus_error;
            }
            wait_ms(20);
        }

        if (!canbus_ack) {
            return 0;
        }
    }
    return 1;
}

int ConfigureCanGpsNodes(int num_gps_nodes)
{
    CANMessage can_tx_message;
    int timeout = CAN_TIMEOUT;

    // Configure Servo board. Board will 'ACK' message to indicate success.
    //  - If no ACK after Timeout, then Fail
    for (int i = 0; i < num_gps_nodes; i++) {
        canbus_ack = 0;

        can_tx_message.len = 1;
        can_tx_message.data[0] = GPS_SEL_CHIP_AUTO | COMPASS_SEL_MASK(pConfig->compass_selection);
        can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_GPS_NODETYPE,
                                                (DEFAULT_NODE_ID + i), AVIDRONE_MSGID_GPS_CFG);
        while(!canbus_ack && --timeout) {
            if (!can.write(can_tx_message)) {
                ++write_canbus_error;
            }
            wait_ms(20);
        }

        if (!canbus_ack) {
            return 0;
        }
    }
    return 1;
}

int ConfigureCanPowerNodes(int num_power_nodes)
{
    CANMessage can_tx_message;
    int timeout = CAN_TIMEOUT;

    // Configure Servo board. Board will 'ACK' message to indicate success.
    //  - If no ACK after Timeout, then Fail
    for (int i = 0; i < num_power_nodes; i++) {
        canbus_ack = 0;

        can_tx_message.len = 2;
        can_tx_message.data[0] = PWM_CHANNEL_1_8;
        can_tx_message.data[1] = (pConfig->LidarFromPowerNode) ? LIDAR_ACTIVE : 0;

        can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_PWR_NODETYPE,
                                                (DEFAULT_NODE_ID + i), AVIDRONE_MSGID_PWR_CFG);
        while(!canbus_ack && --timeout) {
            if (!can.write(can_tx_message)) {
                ++write_canbus_error;
            }
            wait_ms(20);
        }

        if (!canbus_ack) {
            return 0;
        }
    }
    return 1;
}

static void GpsHeartbeat(int num_gps_nodes)
{
    CANMessage can_tx_message;

    can_tx_message.type   = CANData;
    can_tx_message.format = CANStandard;
    can_tx_message.len    = 0;

    for (int i = 0; i < num_gps_nodes; i++) {
        can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_GPS_NODETYPE,
                                                (DEFAULT_NODE_ID + i), AVIDRONE_MSGID_GPS_SYNC);
        if (!can.write(can_tx_message)) {
            ++write_canbus_error;
        }
    }
}

static void PowerNodeHeartbeat(int num_power_nodes)
{
    CANMessage can_tx_message;

    can_tx_message.type   = CANData;
    can_tx_message.format = CANStandard;
    can_tx_message.len    = 0;

    for (int i = 0; i < num_power_nodes; i++) {
        can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_PWR_NODETYPE,
                                                (DEFAULT_NODE_ID + i), AVIDRONE_MSGID_PWR_SYNC);
        if (!can.write(can_tx_message)) {
            ++write_canbus_error;
        }
    }
}

// NOTE::SP: Remove - Just for Debug....
static void ServoHeartbeat(int num_servo_nodes)
{
    CANMessage can_tx_message;

    can_tx_message.type   = CANData;
    can_tx_message.format = CANStandard;
    can_tx_message.len    = 2;

    can_tx_message.data[0] = PWM_CHANNEL_1 | PWM_CHANNEL_2 | PWM_CHANNEL_3 | PWM_CHANNEL_4;
    can_tx_message.data[1] = (LIDAR_ACTIVE | LIVE_LINK_ACTIVE);

    for (int i = 0; i < num_servo_nodes; i++) {
        can_tx_message.id = AVIDRONE_CAN_ID(AVIDRONE_SERVO_NODETYPE,
                                                    (DEFAULT_NODE_ID + i), AVIDRONE_MSGID_SERVO_CFG);
    }

    if (!can.write(can_tx_message)) {
        ++write_canbus_error;
    }
}

//
void InitializeRuntimeData(void)
{
	// Clear out the Runtime RAm copy of the config Data
	memset(pRamConfigData, 0x00, sizeof(ConfigData));

    hfc.PRstick_rate  = pConfig->PRstickRate / pConfig->Stick100range;
    hfc.PRstick_angle = pConfig->PRstickAngle /pConfig->Stick100range;
    hfc.YawStick_rate = pConfig->YawStickRate / pConfig->Stick100range;
    hfc.Stick_Vspeed  = pConfig->StickVspeed / pConfig->Stick100range;
    hfc.Stick_Hspeed  = pConfig->StickHspeed / pConfig->Stick100range;

    // convert dead band values in % to servo range
    for (int i = 0; i < 4; i++) {
        hfc.StickDeadband[i] = pConfig->stick_deadband[i] * 0.01f * pConfig->Stick100range;
    }

    PID_Init(&hfc.pid_PitchRate,  pConfig->pitchrate_pid_params,  0, 1);
    PID_Init(&hfc.pid_RollRate,   pConfig->rollrate_pid_params,   0, 1);
    PID_Init(&hfc.pid_YawRate,    pConfig->yawrate_pid_params,    0, 1);
    PID_Init(&hfc.pid_PitchAngle, pConfig->pitchangle_pid_params, 1, 0);
    PID_Init(&hfc.pid_RollAngle,  pConfig->rollangle_pid_params,  1, 0);
    PID_Init(&hfc.pid_CollVspeed, pConfig->collvspeed_pid_params, 0, 0);
    PID_Init(&hfc.pid_PitchSpeed, pConfig->pitchspeed_pid_params, 0, 0);
    PID_Init(&hfc.pid_RollSpeed,  pConfig->rollspeed_pid_params,  0, 0);

    PID_Init(&hfc.pid_IMU[0],     pConfig->imu_pid_params, 1, 0);
    PID_Init(&hfc.pid_IMU[1],     pConfig->imu_pid_params, 1, 0);
    PID_Init(&hfc.pid_IMU[2],     pConfig->imu_pid_params, 1, 0);

    PID_P_Acc_Init(&hfc.pid_YawAngle,    pConfig->yawangle_pid_params,    1, true); // enable deceleration
    PID_P_Acc_Init(&hfc.pid_CollAlt,     pConfig->collalt_pid_params,     0, true); // same acc and dec
    PID_P_Acc_Init(&hfc.pid_Dist2T,      pConfig->dist2T_pid_params,      0, true);
    PID_P_Acc_Init(&hfc.pid_Dist2P,      pConfig->dist2P_pid_params,      0, false);
    PID_P_Acc_Init(&hfc.pid_PitchCruise, pConfig->pitchCruise_pid_params, 0, false);

    hfc.speed_Iterm_E     = 0;
    hfc.speed_Iterm_N     = 0;
    hfc.speed_Iterm_E_lp  = 0;
    hfc.speed_Iterm_N_lp  = 0;

    // save default values for playlist mode, duplicated and used within hfc.
    //   - These used to be in cfg, but this is now READ only
    hfc.rw_cfg.VspeedMax = hfc.pid_CollAlt.COmax;
    hfc.rw_cfg.VspeedMin = hfc.pid_CollAlt.COmin;
    hfc.rw_cfg.VspeedAcc = hfc.pid_CollAlt.acceleration;
    hfc.rw_cfg.HspeedMax = hfc.pid_Dist2T.COmax;
    hfc.rw_cfg.HspeedAcc = hfc.pid_Dist2T.acceleration;

    // initialize sensor's low pass filters
    for (int i=0; i < 3; i++) {
        hfc.calib_gyro_avg[i]  = 0;
    }

    LP4_Init(&hfc.lp_gyro4[PITCH], pConfig->gyro_lp_freq[PITCH]);
    LP4_Init(&hfc.lp_gyro4[ROLL], pConfig->gyro_lp_freq[ROLL]);
    LP4_Init(&hfc.lp_gyro4[YAW], pConfig->gyro_lp_freq[YAW]);

    for (int i=0; i < 3; i++) {
        LP4_Init(&hfc.lp_acc4[i], pConfig->acc_lp_freq);
    }

    LP4_Init(&hfc.lp_baro4, pConfig->baro_lp_freq);
    LP4_Init(&hfc.lp_baro_vspeed4, pConfig->baro_vspeed_lp_freq);

    hfc.Pos_GPS_IMU_Blend = pConfig->Pos_GPS_IMU_BlendReg;
    hfc.telem_ctrl_period = Max(hfc.telem_ctrl_period, (pConfig->telem_min_ctrl_period * 1000));

    hfc.throttle_value   = -pConfig->Stick100range;
    hfc.collective_value = -pConfig->Stick100range;

    hfc.power.capacity_total = (pConfig->battery_capacity / 1000.0f * 3600); // As
    hfc.power.energy_total   = (hfc.power.capacity_total * pConfig->battery_cells * 3.7f);  // Ws

    hfc.dyn_yaw_rate = pConfig->default_dyn_yaw_rate;
    hfc.ctrl_source = pConfig->default_ctrl_source;
    hfc.acc_dyn_turns = pConfig->default_acc_dyn_turns;

    for (int i = 0; i < 3; i++) {
        hfc.home_pos[i] = pConfig->default_home_position[i];
    }

    hfc.orient_reset_counter = pConfig->orient_reset_counter;

    // NOTE:SP: This is data which is updated at runtime to a duplicated
    // Read/Write area.
    hfc.rw_cfg.GTWP_retire_radius = pConfig->GTWP_retire_radius;
    hfc.rw_cfg.GTWP_retire_speed = pConfig->GTWP_retire_speed;
    hfc.rw_cfg.FTWP_retire_sr_factor = pConfig->FTWP_retire_sr_factor;
    hfc.rw_cfg.low_speed_limit = pConfig->low_speed_limit;
    hfc.rw_cfg.PRstickRate = pConfig->PRstickRate;
    hfc.rw_cfg.PRstickAngle = pConfig->PRstickAngle;
    hfc.rw_cfg.YawStickRate = pConfig->YawStickRate;
    hfc.rw_cfg.StickVspeed = pConfig->StickVspeed;
    hfc.rw_cfg.StickHspeed = pConfig->StickHspeed;
    hfc.rw_cfg.StickHaccel = pConfig->StickHaccel;
    hfc.rw_cfg.RollPitchAngle = pConfig->RollPitchAngle;
    hfc.rw_cfg.wind_compensation = pConfig->wind_compensation;
    hfc.rw_cfg.path_navigation = pConfig->path_navigation;
    hfc.rw_cfg.ManualLidarAltitude = pConfig->ManualLidarAltitude;
    hfc.rw_cfg.AngleCollMixing = pConfig->AngleCollMixing;
    hfc.rw_cfg.cruise_speed_limit = pConfig->cruise_speed_limit;
    hfc.rw_cfg.nose_to_WP = pConfig->nose_to_WP;
    hfc.rw_cfg.landing_wind_threshold = pConfig->landing_wind_threshold;
    hfc.rw_cfg.battery_capacity = pConfig->battery_capacity;
    hfc.rw_cfg.WindTableScale = pConfig->WindTableScale;

    hfc.command.command = TELEM_CMD_NONE;
    hfc.full_auto = true;
    hfc.auto_throttle = true;
    hfc.playlist_status = PLAYLIST_STOPPED;
    hfc.display_mode = DISPLAY_SPLASH;
    hfc.control_mode[PITCH] = CTRL_MODE_ANGLE;
    hfc.control_mode[ROLL] = CTRL_MODE_ANGLE;
    hfc.control_mode[YAW] = CTRL_MODE_ANGLE;
    hfc.control_mode[COLL] = CTRL_MODE_MANUAL;
    hfc.control_mode[THRO] = CTRL_MODE_MANUAL;
    hfc.waypoint_type = WAYPOINT_NONE;
    hfc.btnMenuPrev = true;
    hfc.btnSelectPrev = true;
    hfc.AltitudeBaroGPSblend = ALTITUDE_BARO_GPS_BLEND_FREQ_INIT;
    hfc.baro_altitude_raw_lp = -9999;
    hfc.esc_temp = 20;
    hfc.altitude_lidar_raw = 40;
    hfc.distance2WP_min = 999999;
    hfc.rpm_ticks          = Ticks1();
    hfc.comp_calibrate = NO_COMP_CALIBRATE;

    GenerateSpeed2AngleLUT();

    // If there is a valid compass calibration, load it.
    // otherwise use defaults.
    const CompassCalibrationData *pCompass_cal = NULL;

    if (LoadCompassCalibration(&pCompass_cal) == 0) {
        memcpy(&hfc.compass_cal, pCompass_cal, sizeof(CompassCalibrationData));
    }
    else {
        // no valid calibration, use defaults
        for (int i = 0; i < 3; i++) {
            hfc.compass_cal.comp_ofs[i] = 0;
        }

        for (int i = 0; i < 3; i++) {
            hfc.compass_cal.comp_gains[i] = 1;
        }

        for (int i = 0; i < 3; i++) {
            hfc.compass_cal.compassMin[i] = 9999;
        }

        for (int i = 0; i < 3; i++) {
            hfc.compass_cal.compassMax[i] = -9999;
        }
    }
}

/**
  * @brief  InitCanbusNodes.
  * @retval 0 if success, < 0 on failure
  */
static int InitCanbusNodes(void)
{
    int canbus_status = 0;
    int init = 0;

    if (pConfig->num_servo_nodes) {
        if ((init = InitCanServoNodes(pConfig->num_servo_nodes)) != 0) {
            init = ConfigureCanServoNodes(pConfig->num_servo_nodes);
        }

        if (!init) {
            myLcd.ShowError("Failed to initialize CANBUS SERVO NODE(S)\n", "SERVO NODE", "INITIALIZATION", "FAILED");
            canbus_status = -1;
        }
    }

    if (pConfig->num_gps_nodes) {

        gps.Init(pConfig->num_gps_nodes);

        if ((init = InitCanGpsNodes(pConfig->num_gps_nodes)) != 0) {
            init = ConfigureCanGpsNodes(pConfig->num_gps_nodes);
        }

        if (!init) {
            myLcd.ShowError("Failed to initialize CANBUS GPS NODE(S)\n", "GPS NODE", "INITIALIZATION", "FAILED");
            canbus_status = -1;
        }
    }

    if (pConfig->num_power_nodes) {
        if ((init = InitCanPowerNodes(pConfig->num_power_nodes)) != 0) {
            init = ConfigureCanPowerNodes(pConfig->num_power_nodes);
        }

        if (!init) {
            myLcd.ShowError("Failed to initialize CANBUS POWER NODE(S)\n", "POWER NODE", "INITIALIZATION", "FAILED");
            canbus_status = -1;
        }
    }

    return canbus_status;
}

/**
  * @brief  main.
  * @retval none
  */
int main()
{

    led1 = 1;

    spi.frequency(4000000);
    spi.format(8, 0);   // 0-sd ok, disp ok, 1-no sd, disp ok

    btnMenu.mode(PullUp);
    btnSelect.mode(PullUp);

    lidar.mode(PullNone);

    myLcd.InitLcd();

    xbus.revert[1] = 1;

    SysTick_Run();

    myLcd.ShowSplash(AVIDRONE_SPLASH, AVIDRONE_FCM_SPLASH, FCM_VERSION);

    int cfg_result;
    if ((cfg_result = LoadConfiguration(&pConfig)) < 0) {
        init_ok = 0;

        if (cfg_result == -1) {
            myLcd.ShowError("Failed to Load Configuration\n", "CONFIG", "CHECKSUM", "ERROR");
        }
        else if (cfg_result == -2) {
            char str_temp[20];
            sprintf(str_temp, "REQ VER: %d", CONFIG_VERSION);
            myLcd.ShowError("Failed to Load Configuration\n", "CONFIG", "VERSION ERROR", str_temp);
        }
        else if (cfg_result == -3) {
            myLcd.ShowError("Failed to Load Configuration\n", "CONFIG", "ERROR", "SIZE");
        }
        else {
            myLcd.ShowError("Failed to Load Configuration\n", "CONFIG", "LOAD", "FAILED");
        }
    }

    if (init_ok) {

        InitializeRuntimeData();

        // Configure CAN frequency to either 1Mhz or 500Khz, based on configuration
        int frequency = (pConfig->canbus_freq_high == 1) ? 1000000 : 500000;
        can.frequency(frequency);
        can.attach(can_handler);

        if (InitCanbusNodes() < 0) {
            init_ok = 0;
        }

        if (init_ok) {
            int mpu_init = 0;
            if (pConfig->imu_internal) {
                mpu_init = mpu.init(INTERNAL, MPU6050_DLPF_BW_188, pConfig->force_gyro_acc_defaults);
            }
            else {
                mpu_init = mpu.init(EXTERNAL, MPU6050_DLPF_BW_188, pConfig->force_gyro_acc_defaults);
            }

            init_ok = 1;
            if (mpu_init <= 0) {
                if (mpu_init == -1) {
                    // On failing eeprom, calibration values will be set to default. This is not
                    // therefore a hard error and we continue.
                    myLcd.ShowError("Failed to initialize IMU\n", "IMU", "EEPROM ERROR", "");
                    init_warning = 1;
                }
                else if (mpu_init == -2) {
                    myLcd.ShowError("Failed to initialize IMU\n", "IMU", "INITIALIZATION", "FAILED");
                    init_ok = 0;
                }
            }
        }

        if (init_ok) {
            if (pConfig->sensor_mode == FLY_ALL_SENSORS) {
                if ((pConfig->compass_type == CHIP_HMC5883L) || (pConfig->compass_type == CHIP_AK8963)) {
                    compass.enable_i2c_MPU6050(MPU6050_ADDRESS);
                }

                if (!compass.Init(pConfig)) {
                    myLcd.ShowError("Failed to initialize compass HMC5883L/AK8963\n", "COMPASS", "INITIALIZATION", "FAILED");
                    init_ok = 0;
                }
            }
        }

        if (init_ok) {
            if (pConfig->baro_enable == 1) {
                if (!baro.Init()) {
                    myLcd.ShowError("Failed to initialize BAROMETER\n", "BAROMETER", "INITIALIZATION", "FAILED");
                    init_ok = 0;
                }
            }
        }
    }

    if (init_ok) {

        xbus.SetSbusEnabled(pConfig->SbusEnable);
        xbus.ConfigRx();

        telem.Initialize(&hfc, pConfig);
        telemetry.baud(pConfig->telem_baudrate);

        Servos_Init();

        if (pConfig->LidarFromServo == 0) {
            hfc.lidar_rise = GetTime_us();
            lidar.fall(&Lidar_fall);
            lidar.rise(&Lidar_rise);
        }

        telem.Generate_AircraftCfg();

        mpu.readMotion7_start();

        while(1) {

            WDT_Kick();

            // Main FCM control loop
            do_control();

            // Process Serial commands if USB is available
            if (serial.connected() && serial.readable()) {
                ProcessUserCmnds(serial.getc());
            }
        }
    }
    else {

        // Turn on ALL leds Solid and look for user commands from
        // USB Serial. This Provides a means to update configuration and 'recover'
        // board.
        while(1) {
            WDT_Kick();
            led1 = 1; led2 = 1; led3 = 1; led4 = 1;

            // Process Serial commands if USB is available
            if (serial.connected() && serial.readable()) {
                ProcessUserCmnds(serial.getc());
            }
        }
    }
}

/************************ (C) COPYRIGHT Avidrone Aerospace Inc. *****END OF FILE****/
