#include <BMPx80.h>
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
#include "IMU.h"
#include "xbus.h"
#include "PID.h"
#include "pGPS.h"
#include "telemetry.h"
#include "avican.h"
#include "USBSerial.h"
#include "IAP.h"
#include "afsi.h"
#include "main.h"
#include "mediator.h"
#include "config.h"
#include "mixer.h"
#include "compass.h"

// Linker defined sections. DO NOT ALTER
extern int __attribute__((__section__(".ramscratch"))) ram_scratch;
static unsigned char *pRamConfigData = (unsigned char *)&ram_scratch;
int __attribute__((__section__(".hfcruntime"))) _hfc_runtime;

// Create required devices classes
USBSerial serial(0x0D28, 0x0204, 0x0001, false);
I2Ci Li2c(I2C_SDA1, I2C_SCL1);
MPU6050 mpu(&Li2c, -1, MPU6050_GYRO_FS_500, MPU6050_ACCEL_FS_4);
BMPx80 baro(&Li2c);
XBus xbus(XBUS_IN);
CAN *Canbus = NULL;
GPS gps;
Compass *compass = NULL;

// Telemetry Serial connection
RawSerial telemetry(TELEM_TX, TELEM_RX);
TelemSerial telem(&telemetry);

// AFSI serial connection
RawSerial afsi_serial(AFSI_TX,AFSI_RX, 38400);
AFSI_Serial afsi(&afsi_serial,&telem);

// FCM PWM outputs
PwmOut FCM_SERVO_CH6(CHANNEL_6);
PwmOut FCM_SERVO_CH5(CHANNEL_5);
PwmOut FCM_SERVO_CH4(CHANNEL_4);
PwmOut FCM_SERVO_CH3(CHANNEL_3);
PwmOut FCM_SERVO_CH2(CHANNEL_2);
PwmOut FCM_SERVO_CH1(CHANNEL_1);

// ---- Local Prototypes ---- //
static inline void UpdatePitchRateScalingFactor(float xbus_value);
static inline void ApplyPidScaling(T_PID *pid_layer, const float params[6], float pid_scaling);
static inline void ResetPidScaling(T_PID *pid_layer, const float params[6]);
static void initRpmThresholdCheck(void);
static int rpm_threshold_check(float dT);
static void UpdateBatteryStatus(float dT);
static float GetAngleFromSpeed(float speed, const float WindSpeedLUT[ANGLE2SPEED_SIZE], float scale);
static void AutoReset(void);
static void OrientResetCounter();
static void Get_Orientation(float *SmoothAcc, float *AccData, float dt);
static void CanbusSync(void);
static void CanbusFailsafe(int node_type, int node_id);
static uint32_t EnableCanbusPDPs(void);
static uint32_t ConfigureCanbusNode(int node_type, int node_id, int seq_id, CANMessage *can_tx_message, int timeout = 100);
static uint32_t InitCanbusNode(int node_type, int node_id, int timeout = 100);
static void WriteToServos(int node_id, int num_servo_nodes);
static void SetAgsControls(void);
static void SetRCRadioControl(void);
static void ServoMixer(void);
static inline void ServoOutput(void);
static inline void KeepMotorsOnWhenArmed(void);
static inline void SetThrottle(void);
static inline void SetDcpElevGains(void);
static inline void TandemFanControl(void);
static inline void ProcessStickInputs(FlightControlData *phfc, float dT);
static bool CheckRangeAndSetF(float *pvalue, float value, float vmin, float vmax);
static void CheckRangeAndSetI(int *pvalue, int value, int vmin, int vmax);
static void CheckRangeAndSetB(byte *pvalue, int value, int vmin, int vmax);
static void Playlist_ProcessTop();
static void Playlist_ProcessBottom(FlightControlData *hfc, bool retire_waypoint);
static void AbortFlight(void);
static void ProcessTakeoff(float dT);
static void ProcessLanding(float dT);
static void ProcessTouchAndGo(float dT);
static void ProcessFlightMode(FlightControlData *hfc, float dT);
static void SetSpeedAcc(float *value, float speed, float acc, float dT);
static float CalcMinAboveGroundAlt(float speed);
static void ServoUpdateRAW(float dT);
static inline void UpdatePitchRateScalingFactor(float xbus_value);
static inline void ApplyPidScaling(T_PID *pid_layer, const float params[6], float pid_scaling);
static inline void ResetPidScaling(T_PID *pid_layer, const float params[6]);
static void ServoUpdate(float dT);
static void SensorsRescale(float accIn[3], float gyroIn[3], float accOut[3], float gyroOut[3]);
static float UnloadedBatteryLevel(float voltage, const float V2Energy[V2ENERGY_SIZE]);
static void UpdateBatteryStatus(float dT);
static void UpdateHwIdLow(int node_id, unsigned char *pdata);
static void UpdateHwIdHigh(int node_id, int board_type, unsigned char *pdata);
static void UpdateHardwareStatus(int node_id, int node_type, unsigned char *pdata);
static void UpdateServoNodeMonV(int node_id, unsigned char *pdata);
static void UpdateLidar(int node_id, int pulse_us);
static void UpdateCastleLiveLink(int node_id, int seq_id, unsigned char *pdata);
static void UpdatePowerNodeVI(int node_id, unsigned char *pdata);
static void CanbusISRHandler(void);
static int CanbusNodeTypeStatus(int node_type);
static void CanbusNodeMonitor(float dT);
static void SystemMonitor(float dT);
static void FailSafeMode(void);
static void CompassCalibration(void);
static void ArmedTimeout(float dT);
static void FlightOdometer(void);
static void DoFlightControl();
static void InitFcmIO(void);
static void ConfigRx(void);
static void ProcessUserCmnds(char c);
static void InitializeRuntimeData(void);
static uint32_t InitializeSystemData();
static uint32_t InitCanbus(void);
static void RunDefaultSystem(void);

// ---- Local Data ---- //
AviMixer *mixer = NULL;
AviTandemMixer  *tandem_mixer = NULL;
CastleLinkLive castle_link_live[MAX_NUM_CASTLE_LINKS];
Mediator* lidar_median[MAX_NUM_LIDARS];

static int can_node_found = 0;
static int canbus_ack = 0;
static int canbus_livelink_avail = 0;
static int power_update_avail = 0;
volatile static int rx_in = 0;
volatile static bool have_config = false;

FlightControlData *phfc = (FlightControlData *)&_hfc_runtime;  // This area is Reserved by the linker!;
const ConfigData *pConfig = NULL;

// ---- Constants ---- //
#define CAN_NODE_TIMEOUT 1.0f // 1.0 second
#define ABORT_TIMEOUT_SEC  5.0f // After 5 seconds, if we have no valid connection/instructions abort the flight.
#define IMU_CHECK_TIMEOUT 1.0f  // 1.0 seconds
#define IMU_ERROR_THRESHOLD 800 // threshold of IMU errors within IMU_CHECK_TIMEOUT

#define BARO_CHECK_TIMEOUT 1.0f  // 1.0 seconds
#define BARO_ERROR_THRESHOLD 8   // threshold of IMU errors within BARO_CHECK_TIMEOUT

#define WATCHDOG_TIMEOUT 0.5f // if we don't kick preston for 0.5 seconds he will bite us!
#define WD_RESET_LIMIT 3  // After being bitten 3 time - attempt failsafe and give up

#define CAN_SERVO_FRONT 0
#define CAN_SERVO_REAR  1
#define FCM_SERVO       2

// Default configuration is to all GPS to auto select device in use.
#define DEFAULT_GPS_CFG GPS_SEL_CHIP_AUTO

// Macros for controlling 'special' reserved FCM outputs
#define FCM_SET_ARM_LED(X) ( FCM_SERVO_CH6.pulsewidth_us( ((X) == 1) ? 2000 : 1000) )
#define FCM_DROP_BOX_CTRL(X) ( FCM_SERVO_CH5.pulsewidth_us( ((X) == 1) ? pConfig->aux_pwm_max : pConfig->aux_pwm_min) )

// ---- Public Interfaces ---- //

// @brief
// @param
// @retval
int getNodeVersionNum(int type, int nodeId)
{
  int vers_num;

  vers_num = (phfc->board_info[type][nodeId].major_version) << 16;
  vers_num |= (phfc->board_info[type][nodeId].minor_version) << 8;
  vers_num |= (phfc->board_info[type][nodeId].build_version);

  return vers_num;
}

// @brief
// @param
// @retval
void getNodeSerialNum(int type, int nodeId, uint32_t *pSerailNum)
{
  pSerailNum[0] = (phfc->board_info[type][nodeId].serial_number0);
  pSerailNum[1] = (phfc->board_info[type][nodeId].serial_number1);
  pSerailNum[2] = (phfc->board_info[type][nodeId].serial_number2);
}

// @brief
// @param
// @retval
void GenerateSpeed2AngleLUT(void)
{
  for (int i=0; i<SPEED2ANGLE_SIZE; i++) {
    float speed = i*0.5f;
    float angle = GetAngleFromSpeed(speed, pConfig->WindSpeedLUT, phfc->rw_cfg.WindTableScale);
    phfc->rw_cfg.Speed2AngleLUT[i] = angle;
  }
}

// @brief
// @param
// @retval
void ResetIterms(void)
{
  phfc->pid_PitchRate.Ie  = 0;
  phfc->pid_RollRate.Ie   = 0;
  phfc->pid_YawRate.Ie    = 0;
  phfc->pid_PitchAngle.Ie = 0;
  phfc->pid_RollAngle.Ie  = 0;
  phfc->pid_YawAngle.Ie   = 0;
  phfc->pid_CollVspeed.Ie = 0;
  phfc->pid_PitchSpeed.Ie = 0;
  phfc->pid_RollSpeed.Ie  = 0;
  phfc->pid_CollAlt.Ie    = 0;
  phfc->pid_Dist2T.Ie     = 0;
  phfc->pid_Dist2P.Ie     = 0;
  phfc->pid_PitchCruise.Ie= 0;
  phfc->speed_Iterm_E     = 0;
  phfc->speed_Iterm_N     = 0;
}

// @brief
// @param
// @retval
int TakeoffControlModes(void) {
  if (  (phfc->control_mode[COLL] <= CTRL_MODE_MANUAL)
     || (phfc->control_mode[PITCH] <= CTRL_MODE_ANGLE)
     || (phfc->control_mode[ROLL] <= CTRL_MODE_ANGLE)
     || (phfc->control_mode[YAW] <= CTRL_MODE_ANGLE) ) {
    return 1;
  }
  else {
    return 0;
  }
}

// @brief
// @param
// @retval
int GetMotorsState(void) {
  if (phfc->throttle_value == -pConfig->Stick100range) {
    return 0; // Motors off
  }
  else if (phfc->throttle_value == pConfig->Stick100range) {
    return 1; // Motors On
  }

  return 2; // Unknown state
}

// @brief
// @param
// @retval
void HeadingUpdate(float heading_rate, float dT)
{
    phfc->ctrl_out[ANGLE][YAW] += heading_rate*dT;

    if (phfc->ctrl_out[ANGLE][YAW]>180) {
      phfc->ctrl_out[ANGLE][YAW]-=360;
    }
    else if (phfc->ctrl_out[ANGLE][YAW]<-180) {
      phfc->ctrl_out[ANGLE][YAW]+=360;
    }
}

// @brief
// @param
// @retval
void AltitudeUpdate(float alt_rate, float dT)
{
  phfc->ctrl_out[POS][COLL] += alt_rate*dT;

  if (phfc->ctrl_out[POS][COLL] > 7000) {
    phfc->ctrl_out[POS][COLL] = 7000;
  }
  else if (phfc->ctrl_out[POS][COLL] < 0) {
    phfc->ctrl_out[POS][COLL] = 0;
  }
}

// @brief
// @param
// @retval
bool LidarOnline(void)
{
  for (int i = 0; i < phfc->num_lidars; i++) {
    if (((phfc->lidar_online_mask >> i) & 1) == 0 ) {
      return false;
    }
  }
  return true;
}

// @brief
// @param
// @retval
void CompassCalDone(void)
{
  // Update new offset and gains to the compass class
  for (int i=0; i<3; i++) {
    compass->UpdateOffsets(i, phfc->compass_cal.compassMin[i], phfc->compass_cal.compassMax[i]);
    compass->UpdateGains(i, phfc->compass_cal.compassMin[i], phfc->compass_cal.compassMax[i]);

    phfc->compass_cal.comp_ofs[i] = compass->GetOffsets(i);
    phfc->compass_cal.comp_gains[i] = compass->GetGains(i);
  }

  phfc->compass_cal.valid = 1;
  phfc->compass_cal.version = COMPASS_CAL_VERSION;

  int size = telem.CalibrateCompassDone();
  telem.AddMessage((unsigned char*)&phfc->telemCalibrate, size, TELEMETRY_CALIBRATE, 6);

  // TODO::SP: Error handling...?
  SaveCompassCalibration(&phfc->compass_cal);
}

// --- Private Functions --- //

// @brief
// @param
// @retval
static float GetAngleFromSpeed(float speed, const float WindSpeedLUT[ANGLE2SPEED_SIZE], float scale)
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

// @brief
// @param
// @retval
static void AutoReset(void)
{
    float delta_accel[3];
    float delta_orient[3];

    float degreesPerSecLimit = 0.2;
    float accelLimit = 0.05;

    if( !phfc->throttle_armed )
    {
        if( pConfig->autoReset && ((phfc->print_counter % 500) == 0) )
        {
            for(int i = 0; i < 3; i++)
            {
                delta_accel[i]  = ABS( phfc->acc[i] - phfc->acc_prev[i] );
                delta_orient[i] = ABS( phfc->IMUorient[i] - phfc->IMUorient_prev[i] );
                phfc->acc_prev[i] = phfc->acc[i];
                phfc->IMUorient_prev[i] = phfc->IMUorient[i];
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
                 || ABS(phfc->SmoothAcc[PITCH] - phfc->IMUorient[PITCH])>(0.5f*D2R)
                 || ABS(phfc->SmoothAcc[ROLL]  - phfc->IMUorient[ROLL] )>(0.5f*D2R)  )
                {
                    telem.ResetIMU(false);
                }
            }

            /*Check if GPS signal is good, if so, then reset if
             * IMU altitude and GPS altitude differ by more than 2m */
            //GpsData gps_data = gps.GetGpsData();
            if ( (gps.gps_data_.fix > GPS_FIX_NONE) && (gps.gps_data_.PDOP < 250) ) {
                if (ABS(phfc->altitude_baro - phfc->altitude_gps) >= 2) {
                    phfc->altitude_ofs = phfc->altitude_gps - phfc->altitude_baro;
                }
            }
        }
    }
}

// @brief
// @param
// @retval
static void OrientResetCounter()
{
  if (phfc->orient_reset_counter) {
    phfc->orient_reset_counter--;

    if (!(phfc->orient_reset_counter&0x3ff)) {
      debug_print("IMU reset   ====   %f %f === ", phfc->SmoothAcc[PITCH]*R2D, phfc->SmoothAcc[ROLL]*R2D);
      telem.ResetIMU(false);
    }
  }
}

// @brief - Calculation of pitch and roll based on accelerometer data.
//           see: NXP application note AN3461 section 3 - Pitch and Roll Estimation
//           https://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf
// @param
// @retval
static void Get_Orientation(float *SmoothAcc, float *AccData, float dt)
{
    float micro = 0.02f;
    float AccAngle[3];
    const float AccLP_Freq = 5.0f; // 1/T

    float sign = AccData[Z_AXIS] < 0 ? -1 : 1;
    
    /* Accelerometer xyz into Pitch and Roll relative to the horizon
     * - roll is limited to +/- 180vdegress
     * - pitch is limited to +/- 90 degrees */
    AccAngle[ROLL]  =  ATAN2fR(-AccData[X_AXIS], sign*sqrtf(AccData[Z_AXIS] * AccData[Z_AXIS] + micro*AccData[Y_AXIS] * AccData[Y_AXIS]));
    AccAngle[PITCH] = (ATAN2fR(AccData[Y_AXIS], sqrtf(AccData[Z_AXIS] * AccData[Z_AXIS] + AccData[X_AXIS] * AccData[X_AXIS])));

    /* Smooth out Acc Pitch and Roll */
    SmoothAcc[ROLL]  = (AccAngle[ROLL]  - SmoothAcc[ROLL] )*dt*AccLP_Freq + SmoothAcc[ROLL];  // Averaging roll ACC values
    SmoothAcc[PITCH] = (AccAngle[PITCH] - SmoothAcc[PITCH])*dt*AccLP_Freq + SmoothAcc[PITCH]; // Averaging pitch ACC values
} 

// @brief
// @param
// @retval
static void WriteToServos(int node_type, int num_nodes)
{
  CANMessage can_tx_message;
  static int pwm_out = 0;
  float temp;
  float fcm_pwm_values[1][8];
  signed short int pwm_values[MAX_CAN_SERVO_NODES][8];
  can_tx_message.len = 8;

  for (int i=0; i < num_nodes; i++) {
    for (int j=0; j < MAX_SERVO_OUTPUTS; j++) {
      temp = phfc->servos_out[i][j];

      if (phfc->servo_reverse_mask & (1<<j)) {
        temp = -phfc->servos_out[i][j];
      }

      if (node_type == AVI_FCM_NODETYPE) {
        fcm_pwm_values[i][j] = SERVOMINMAX(temp); // clip servo values to +/-150%
      }
      else {
        pwm_values[i][j] = (((SERVOMINMAX(temp) * 32767) * 500) /32768) + 1500;
      }
    }
  }

  if (node_type == AVI_FCM_NODETYPE) {
    FCM_SERVO_CH1.pulsewidth_us((int)(1500.5f + fcm_pwm_values[0][0] * 500));
    FCM_SERVO_CH2.pulsewidth_us((int)(1500.5f + fcm_pwm_values[0][1] * 500));
    FCM_SERVO_CH3.pulsewidth_us((int)(1500.5f + fcm_pwm_values[0][2] * 500));
    FCM_SERVO_CH4.pulsewidth_us((int)(1500.5f + fcm_pwm_values[0][3] * 500));
    FCM_SERVO_CH5.pulsewidth_us((int)(1500.5f + fcm_pwm_values[0][4] * 500));
    FCM_SERVO_CH6.pulsewidth_us((int)(1500.5f + fcm_pwm_values[0][5] * 500));
  }
  else {
    int node_fail_mask = (node_type == AVI_SERVO_NODETYPE) ? SERVO_NODE_FAIL : PWR_NODE_FAIL;

    if (pwm_out ^= 1) {
      for (int i=0; i < num_nodes; i++) {
        memcpy(can_tx_message.data, &pwm_values[i][0], 8);

        can_tx_message.id = AVI_CAN_ID(node_type, (DEFAULT_NODE_ID + i), AVI_SERVOS_0_3, AVI_MSGID_CDP);
        if ((phfc->system_status_mask & (node_fail_mask << i)) == 0) {
          Canbus->write(can_tx_message);
        }
      }
    }
    else {
      for (int i=0; i < num_nodes; i++) {
        memcpy(can_tx_message.data, &pwm_values[i][4], 8);

        can_tx_message.id = AVI_CAN_ID(node_type, (DEFAULT_NODE_ID + i), AVI_SERVOS_4_7, AVI_MSGID_CDP);
        if ((phfc->system_status_mask & (node_fail_mask << i)) == 0) {
          Canbus->write(can_tx_message);
        }
      }
    }
  }
}

// @brief
// @param
// @retval
static void SetAgsControls(void)
{
  if ((phfc->waypoint_type == WAYPOINT_TAKEOFF) || phfc->touch_and_go_takeoff) {

    if (phfc->waypoint_stage == FM_TAKEOFF_COMPLETE) {
      // Land, home, point and fly active once we have reached Takeoff.
      phfc->controlStatus = CONTROL_STATUS_LAND | CONTROL_STATUS_HOME | CONTROL_STATUS_POINTFLY;
    }
    else {
      phfc->controlStatus = CONTROL_STATUS_LAND;
    }
  }
  else if ((phfc->waypoint_type == WAYPOINT_LANDING) || phfc->touch_and_go_landing) {
    if (phfc->waypoint_stage >= FM_LANDING_WAYPOINT) {

      phfc->controlStatus = CONTROL_STATUS_HOME | CONTROL_STATUS_LAND;

      if (phfc->playlist_status == PLAYLIST_PLAYING) {
        phfc->controlStatus |= CONTROL_STATUS_PAUSE;
      }
      else if (phfc->playlist_status == PLAYLIST_PAUSED) {
        phfc->controlStatus |= CONTROL_STATUS_PLAY | CONTROL_STATUS_POINTFLY | CONTROL_STATUS_ABORT;
      }
      else {
        phfc->controlStatus |= CONTROL_STATUS_ABORT;
      }
    }
  }
  else if ((phfc->waypoint_type == WAYPOINT_GOTO) && (phfc->playlist_status != PLAYLIST_PLAYING)) {
    phfc->controlStatus = CONTROL_STATUS_LAND | CONTROL_STATUS_HOME | CONTROL_STATUS_POINTFLY | CONTROL_STATUS_ABORT;

    if (phfc->playlist_status == PLAYLIST_PAUSED) {
      phfc->controlStatus |= CONTROL_STATUS_PLAY;
    }
  }
  else if (phfc->playlist_status <= PLAYLIST_STOPPED) {
    if (IN_THE_AIR(phfc->altitude_lidar)) {
      phfc->controlStatus = CONTROL_STATUS_LAND | CONTROL_STATUS_HOME | CONTROL_STATUS_POINTFLY;
      if ((phfc->playlist_items > 0) && (phfc->playlist_position < phfc->playlist_items)) {
        phfc->controlStatus |= CONTROL_STATUS_PLAY;
      }
    }
    else {
      if (phfc->throttle_armed) {
        phfc->controlStatus =  CONTROL_STATUS_PREFLIGHT | CONTROL_STATUS_TAKEOFF;
      }
      else {
        phfc->controlStatus =  CONTROL_STATUS_PREFLIGHT;
      }
    }
  }
  else if (phfc->playlist_status == PLAYLIST_PAUSED) {
      phfc->controlStatus = CONTROL_STATUS_PLAY | CONTROL_STATUS_LAND | CONTROL_STATUS_HOME | CONTROL_STATUS_POINTFLY;
  }
  else if (phfc->playlist_status == PLAYLIST_PLAYING) {
      phfc->controlStatus = CONTROL_STATUS_PAUSE | CONTROL_STATUS_LAND | CONTROL_STATUS_HOME;
  }
  else if (phfc->playlist_status != PLAYLIST_PLAYING) {
    phfc->controlStatus = CONTROL_STATUS_LAND | CONTROL_STATUS_HOME | CONTROL_STATUS_POINTFLY;
  }
}

// @brief
// @param
// @retval
static void SetRCRadioControl(void)
{
    if (xbus.valuesf[XBUS_CTRLMODE_SW] > 0.5f) {
        phfc->rc_ctrl_request = false;
    }
    else {
        phfc->rc_ctrl_request = true;
    }

    // if Dynamic PID scaling is configured AND the configured value is 0
    // then we ALWAYS take the value from the the RC Lever. This allows
    // 'GOD' to make adjustments regardless of the control mode.
    if (pConfig->enable_dynamic_speed_pid && (pConfig->dynamic_pid_speed_gain == 0)) {
      UpdatePitchRateScalingFactor(xbus.valuesf[ELEVGAIN]);
    }

    // if not rc_ctrl_request, ignore all RC Radio inputs, keep storing stick values
    if (!phfc->rc_ctrl_request) {

      // Only switch to autopilot of control source is currently RCRADIO, otherwise do nothing
      if (phfc->ctrl_source == CTRL_SOURCE_RCRADIO) {

          telem.SelectCtrlSource(CTRL_SOURCE_AUTOPILOT);

          if (IN_THE_AIR(phfc->altitude_lidar)) {
            telem.SetZeroSpeed();
          }
          // if you're on the ground and armed, and we just switched from
          // RC Control to AUTOPILOT, just disarm.
          else if (!IN_THE_AIR(phfc->altitude_lidar) && phfc->throttle_armed) {
              telem.Disarm();
          }
      }

      telem.SaveValuesForAbort();

      return;
    }

    if (pConfig->eng_super_user_enable) {
      if (xbus.valuesf[XBUS_ENG_SUPER_USER] > 0.5) {
        phfc->eng_super_user = true;
      }
      else {
        phfc->eng_super_user = false;
      }
    }
    else {
      phfc->eng_super_user = false;
    }

    /* always ignore RC radio control, cannot switch from AUTOPILOT to RCRADIO */
    if (phfc->inhibitRCswitches) {
        return;
    }

    /* in non-RCradio mode, check for stick movement to abort */
    if (phfc->ctrl_source!=CTRL_SOURCE_RCRADIO) {

        char abort = 0;

        // if we're flying, or taking off or landing
        if (   IN_THE_AIR(phfc->altitude_lidar)
            || (phfc->waypoint_type == WAYPOINT_TAKEOFF)
            || (phfc->waypoint_type == WAYPOINT_LANDING)
            || (phfc->waypoint_type == WAYPOINT_TOUCH_AND_GO)   ) {

            // When in AUTOPILOT, if the throttle lever is not UP, and we are
            // in the air, then STAY in AUTOPILOT and send message to ground station.
            if (!THROTTLE_LEVER_UP() && !phfc->eng_super_user) {
               telem.SendMsgToGround(MSG2GROUND_THROTTLE_LEVER_LOW);
            }
            // Otherwise, pass control to RC RADIO if sticks move
            else {
                if (ABS(phfc->ctrl_initial[PITCH] - xbus.valuesf[XBUS_PITCH]) > AUTO_PROF_TERMINATE_THRS) {
                    abort = 1;
                }
                if (ABS(phfc->ctrl_initial[ROLL] - xbus.valuesf[XBUS_ROLL]) > AUTO_PROF_TERMINATE_THRS) {
                    abort = 1;
                }
                if (ABS(phfc->ctrl_initial[COLL] - xbus.valuesf[XBUS_THRO]) > AUTO_PROF_TERMINATE_THRS) {
                    abort = 1;
                }
                if (ABS(phfc->ctrl_initial[YAW] - xbus.valuesf[XBUS_YAW]) > AUTO_PROF_TERMINATE_THRS) {
                    abort = 1;
                }
                // If we are not in the air, then a change in the throttle
                // lever passes over control to the RC RADIO
                if (  (ABS(phfc->ctrl_initial[THRO]   - xbus.valuesf[XBUS_THR_LV]) > AUTO_PROF_TERMINATE_THRS)
                   && !IN_THE_AIR(phfc->altitude_lidar) ){
                    abort = 1;
                }
            }
        }
        // check if we're on the ground
        else if (!IN_THE_AIR(phfc->altitude_lidar)) {
            // If the throttle lever is not DOWN, then send message to ground station
            // otherwise, hand over control to RC controller immediately.
            if (!THROTTLE_LEVER_DOWN() && !phfc->eng_super_user) {

               // if we are doing a take off, then don't send this dialog to AGS
               // since the user was just asked to throttle up with the RC lever.
               if ((phfc->waypoint_type != WAYPOINT_TAKEOFF) && (phfc->waypoint_type != WAYPOINT_TOUCH_AND_GO)) {
                 telem.SendMsgToGround(MSG2GROUND_THROTTLE_LEVER_HIGH);
               }
            }
            else {
              abort = 1;
            }
        }

        
        if (abort)  {
            phfc->rc_ctrl_request = true;
            telem.SelectCtrlSource(CTRL_SOURCE_RCRADIO);
            phfc->waypoint_type = WAYPOINT_NONE;
        }
        else if (   ((phfc->waypoint_type == WAYPOINT_TAKEOFF) && (phfc->waypoint_stage >= FM_TAKEOFF_ARM))
                 || (phfc->waypoint_type != WAYPOINT_TAKEOFF) ){
          // In the case of take off with the RC radio in the loop,
          // phfc->rc_ctrl_request = true even though phfc->ctrl_source = AUTOPILOT.
          // This is a necessary state during take off to ensure that expert human pilot
          // can take control in case of emergency.
          // Once take off is complete, phfc->rc_ctrl_request must be set to false
          phfc->rc_ctrl_request = false;
        }
    }

    /* set RC radio control modes */
    if (phfc->ctrl_source==CTRL_SOURCE_RCRADIO)
    {
      if(    THROTTLE_LEVER_DOWN()
          && phfc->throttle_armed
          && (phfc->fixedThrottleMode != THROTTLE_IDLE)       ) {
        telem.Disarm();
      }

      if (THROTTLE_LEVER_UP() && !phfc->throttle_armed && phfc->eng_super_user) {
          telem.Arm();
          phfc->fixedThrottleMode = THROTTLE_FLY;
          phfc->throttle_value = pConfig->Stick100range;
      }

      if (   !phfc->eng_super_user
          && phfc->throttle_armed
          && (phfc->fixedThrottleMode < THROTTLE_FLY)
          && !IN_THE_AIR(phfc->altitude_lidar)
          && !TakeoffControlModes()) {
        telem.Disarm();
        telem.SendMsgToGround(MSG2GROUND_ARMING_MODE);
      }

      if (!pConfig->ctrl_mode_inhibit[COLL])
      {
          if (xbus.valuesf[XBUS_THR_SW]>0.5f)
              phfc->control_mode[COLL] = CTRL_MODE_MANUAL;
          else if (xbus.valuesf[XBUS_THR_SW]<-0.5f)
              phfc->control_mode[COLL] = CTRL_MODE_POSITION;
          else
              phfc->control_mode[COLL] = CTRL_MODE_SPEED;
      }

      /* pitch/rate/yaw mode switches checked only in RCradio mode */
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

      mode += pConfig->RCmodeSwitchOfs;   // shift the mode up
      mode = min(mode, CTRL_MODE_POSITION);

      SetCtrlMode(phfc, pConfig, PITCH, mode);
      SetCtrlMode(phfc, pConfig, ROLL,  mode);
      SetCtrlMode(phfc, pConfig, YAW,   ClipMinMax(mode, pConfig->YawModeMin, pConfig->YawModeMax));

      /* during profiling, force the given mode */
      if (phfc->profile_mode == PROFILING_ON) {
        SetCtrlMode(phfc, pConfig, phfc->profile_ctrl_variable, phfc->profile_ctrl_level+1);
      }
    }
}

// @brief
// @param
// @retval
static void ServoMixer(void)
{
  if (pConfig->ccpm_type == CCPM_TANDEM) {
    SetThrottle();
    SetDcpElevGains();
    tandem_mixer->ServoMixer(phfc->mixer_in, &phfc->servos_out[0][0], &phfc->servos_out[1][0]);
    TandemFanControl();
  }
  else {
    SetThrottle();
    mixer->ServoMixer(phfc->mixer_in, &phfc->servos_out[0][0]);
    
    // TODO::SP This is a special and needs sorting out
    if(pConfig->ccpm_type == CCPM_QUAD) {
      phfc->servos_out[0][4] = ((phfc->throttle_value > 0.5) && phfc->throttle_armed) ? 1 : -1;
      phfc->servos_out[0][5] = phfc->throttle_armed ? 1 : -1;
    }
    KeepMotorsOnWhenArmed();
  }
}

// @brief
// @param
// @retval
static inline void ServoOutput(void)
{

  // If failsafe mode set - no need to keep updating servos
  if (phfc->failsafe == true) {
    return;
  }

  // Output Servo mixer data, depending upon hardware in use.
  if (pConfig->can_servo) {
    WriteToServos(AVI_SERVO_NODETYPE, pConfig->num_servo_nodes);
  }
  else if(pConfig->power_node) {
    WriteToServos(AVI_PWR_NODETYPE, pConfig->num_power_nodes);
  }
  else if(pConfig->fcm_servo) {
    WriteToServos(AVI_FCM_NODETYPE, 1);
  }
}

// @brief
// @param
// @retval
static inline void KeepMotorsOnWhenArmed(void)
{
  // Keep motors from turning off once armed
  if ((phfc->throttle_armed && (phfc->throttle_value > -0.5f))
          || (phfc->waypoint_stage == FM_TAKEOFF_AUTO_SPOOL)) {

    for (int i = 0; i < phfc->num_motors; i++) {
      if(phfc->servos_out[0][i] < pConfig->throttle_multi_min) {
        phfc->servos_out[0][i] = pConfig->throttle_multi_min;
      }
    }
  }
}

// @brief
// @param
// @retval
static inline void SetThrottle(void)
{
  if (pConfig->ccpm_type == CCPM_TANDEM) {
    // Front
    if (phfc->throttle_value > 0.5f) {
      phfc->servos_out[0][0] = phfc->mixer_in[THRO] * pConfig->throttle_gain;
    }
    else {
      phfc->servos_out[0][0] = phfc->mixer_in[THRO];
    }

    // Rear
    if(pConfig->ModelSelect == TANDEM_210TL) {
      // Link Live Throttle on Rear Servo output 0, set to front value + trim value
      phfc->servos_out[1][0] = phfc->servos_out[0][0] + pConfig->RearRpmTrim;
    }
    else {
      phfc->servos_out[1][0] = phfc->servos_out[0][0];
    }
  }
  else {
    for (int i=0; i < phfc->num_motors; i++) {
        phfc->servos_out[0][i] = phfc->mixer_in[THRO] * pConfig->throttle_gain;
    }
  }
}

// @brief
// @param
// @retval
static inline void SetDcpElevGains(void)
{
  if ((pConfig->dcp_gain == 0) && (pConfig->elevator_gain == 0)) {
      // Take values from xbus in this case
      phfc->rw_cfg.elevator_gain = abs(xbus.valuesf[ELEVGAIN]);         // use only positive half
      phfc->rw_cfg.dcp_gain  = abs(xbus.valuesf[DCPGAIN]);
  }
  else {
      // Take values for dcp and elevator gain from the config
      phfc->rw_cfg.elevator_gain = pConfig->elevator_gain;         // use only positive half
      phfc->rw_cfg.dcp_gain  = pConfig->dcp_gain;
  }

  tandem_mixer->UpdateDcpElevGains(phfc->rw_cfg.dcp_gain, phfc->rw_cfg.elevator_gain);
}

// @brief
// @param
// @retval
static inline void TandemFanControl(void)
{
  // Drive PWM Ch5 on front back servos for Fan Control
  phfc->servos_out[0][7] = phfc->throttle_armed ? 1 : -1;
  phfc->servos_out[1][7] = phfc->throttle_armed ? 1 : -1;
}

// @brief
// @param
// @retval
static inline void ProcessStickInputs(FlightControlData *phfc, float dT)
{
    int channel;
    
    /* process P, R, Y, C */
    for (channel=0; channel<4; channel++)
    {
        unsigned char mode = phfc->control_mode[channel];
        if (mode>0)
        {
            float db = phfc->StickDeadband[channel];
            /* no deadband for manual collective */
            if (channel==3 && mode==CTRL_MODE_MANUAL)
                db = 0;
            if (db)
            {
                float v = phfc->ctrl_out[RAW][channel];
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
                phfc->ctrl_out[RAW][channel] = v;
            }
        }
    }
}

// @brief
// @param
// @retval
static bool CheckRangeAndSetF(float *pvalue, float value, float vmin, float vmax)
{
//  debug_print("%f\r\n", value);
    if (!pvalue || value>vmax || value<vmin)
        return false;
    *pvalue = value;
    return true;
}

// @brief
// @param
// @retval
static void CheckRangeAndSetI(int *pvalue, int value, int vmin, int vmax)
{
    if (!pvalue || value>vmax || value<vmin)
        return;
    *pvalue = value;
}

// @brief
// @param
// @retval
static void CheckRangeAndSetB(byte *pvalue, int value, int vmin, int vmax)
{
    if (!pvalue || value>vmax || value<vmin)
        return;
    *pvalue = value;
}

// @brief
// @param
// @retval
static void Playlist_ProcessTop()
{
    T_PlaylistItem *item;
    
    /* process only active playlist */
    if (phfc->playlist_status!=PLAYLIST_PLAYING)
        return;
        
    /* check for end of the playlist */
    if (phfc->playlist_position>=phfc->playlist_items)
    {
        /* stop playlist and waypoint mode - put into position hold */
        phfc->playlist_status = PLAYLIST_STOPPED;
        /* if in flight, put a waypoint at the current position, else do nothing */
        if (phfc->throttle_armed)
            telem.SetZeroSpeed();
        return;
    }

    /* process current playlist item */
    /* only WP and PARAM are processed here, the rest is processed by Playlist_ProcessBottom() and it increments the playlist pointer */
    item = &phfc->playlist[phfc->playlist_position];
    
    if (item->type==PL_ITEM_WP)
    {
        if (item->data[0]==WAYPOINT_GOTO || item->data[0]==WAYPOINT_FLYTHROUGH)
        {
            if (!phfc->pl_wp_initialized)
            {
                telem.SetWaypoint(item->value1.i/10000000.0f, item->value2.i/10000000.0f, phfc->altitude_WPnext, item->data[0], item->data[1]);
                phfc->pl_wp_initialized = true;
            }
        }
        else
        if (item->data[0]==WAYPOINT_TAKEOFF)
        {
            /* initialize it only for the first time */
            if (phfc->waypoint_type != WAYPOINT_TAKEOFF)
            {
              if (IN_THE_AIR(phfc->altitude_lidar)) {
                phfc->playlist_status = PLAYLIST_STOPPED;
                telem.SetZeroSpeed();
              }
              else {
                // For Mission takeoff, Set the desired takeoff height
                phfc->takeoff_height = item->data[1];
                phfc->auto_takeoff = item->data[2];
                telem.CommandTakeoffArm();
              }
            }
        }
        else
        if (item->data[0]==WAYPOINT_LANDING)
        {
          if (phfc->waypoint_type != WAYPOINT_LANDING)
          {
            /* initialize it only for the first time */
            phfc->landingWPHeight = item->data[1];
            phfc->auto_landing = item->data[2];
            telem.CommandLandingWP(item->value1.i/10000000.0f, item->value2.i/10000000.0f, phfc->landingWPHeight);
          }
        }
        else
        if (item->data[0]==WAYPOINT_TOUCH_AND_GO)
        {
          if (phfc->waypoint_type != WAYPOINT_TOUCH_AND_GO)
          {
            float lat = item->value1.i/10000000.0f;
            float lon = item->value2.i/10000000.0f;
            float alt_ground = phfc->altitude_WPnext;

            telem.CommandLandingWP(lat, lon, alt_ground);

            phfc->waypoint_type = WAYPOINT_TOUCH_AND_GO;
            phfc->waypoint_stage = FM_LANDING_WAYPOINT;

            phfc->touch_and_go_wait = item->data[1] + item->data[2]; //set the wait time
            phfc->touch_and_go_do_the_thing_cnt = item->data[2];//set "do the thing" count down

            if (phfc->touch_and_go_do_the_thing_cnt == 0) {
              phfc->touch_and_go_do_the_thing = false;
            }
            else {
              phfc->touch_and_go_do_the_thing = true;
            }

            phfc->touch_and_go_landing = true;
            phfc->touch_and_go_takeoff = false;
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
            if (sub_param==TELEM_PARAM_WP_ALTITUDE) // relative altitude
                CheckRangeAndSetF(&phfc->altitude_WPnext, item->value1.f, -8999, 9999);
            else
//            if (sub_param==TELEM_PARAM_WP_LATITUDE)
//                CheckRangeAndSetD(&phfc->waypoint_pos[0], item->value1.i/10000000.0, -90, 90);
//            else
//            if (sub_param==TELEM_PARAM_WP_LONGITUDE)
//                CheckRangeAndSetD(&phfc->waypoint_pos[1], item->value1.i/10000000.0, -180, 180);
//            else
            if (sub_param==TELEM_PARAM_WP_MAX_H_SPEED)
            {
                CheckRangeAndSetF(&phfc->pid_Dist2T.COmax, item->value1.f, 0.1f, pConfig->max_params_hspeed);
//            	DynamicAccInTurns(hfc, &phfc->pid_Dist2T);

            }
            else
            if (sub_param==TELEM_PARAM_WP_MAX_H_ACC)
            {
                CheckRangeAndSetF(&phfc->pid_Dist2T.acceleration, item->value1.f, 0.1f, pConfig->HspeedAcc);
//              DynamicAccInTurns(hfc, &phfc->pid_Dist2T);
            }
            else
            if (sub_param==TELEM_PARAM_WP_MAX_V_SPEED)
            {
                if (CheckRangeAndSetF(&phfc->pid_CollAlt.COmax, item->value1.f, 0.1f, pConfig->max_params_vspeed))
                	phfc->rw_cfg.VspeedMax = phfc->pid_CollAlt.COmax;
            }
            else
            if (sub_param==TELEM_PARAM_WP_MAX_V_ACC)
            {
                if (CheckRangeAndSetF(&phfc->pid_CollAlt.acceleration, item->value1.f, 0.1f, pConfig->VspeedAcc))
                    phfc->rw_cfg.VspeedAcc = phfc->pid_CollAlt.acceleration;
            }
            else
//            if (sub_param==TELEM_PARAM_WP_TYPE)
//                CheckRangeAndSetI(&phfc->waypoint_type, item->value1.i, 0, 1);
//            else
            if (sub_param==TELEM_PARAM_WP_RETIRE)
                CheckRangeAndSetI(&phfc->waypoint_retire, item->value1.i, 0, 1);
            else
            if (sub_param==TELEM_PARAM_WP_YAWSPEEDRATE)
            {
//                CheckRangeAndSetF(&pConfig->yaw_rate_speed, item->value1.f, 10, 10000);
            }
            else
            if (sub_param==TELEM_PARAM_WP_GTWP_RADIUS)
                CheckRangeAndSetF(&phfc->rw_cfg.GTWP_retire_radius, item->value1.f, 0, 20);
            else
            if (sub_param==TELEM_PARAM_WP_GTWP_SPEED)
                CheckRangeAndSetF(&phfc->rw_cfg.GTWP_retire_speed, item->value1.f, 0, 20);
            else
            if (sub_param==TELEM_PARAM_WP_FTWP_SR_FACTOR)
                CheckRangeAndSetF(&phfc->rw_cfg.FTWP_retire_sr_factor, item->value1.f, 0, 10);
            else
            if (sub_param==TELEM_PARAM_WP_LOW_SPEED_LMT)
              // Note: Low_speed_limit not used
                CheckRangeAndSetF(&phfc->rw_cfg.low_speed_limit, item->value1.f, 1, pConfig->low_speed_limit);
            else
            if (sub_param==TELEM_PARAM_WP_MIN_V_SPEED)
            {
                if (CheckRangeAndSetF(&phfc->pid_CollAlt.COmin, item->value1.f, -5, pConfig->VspeedMin))
                    phfc->rw_cfg.VspeedMin = phfc->pid_CollAlt.COmin;
            }
            else
            if (sub_param==TELEM_PARAM_WP_ALTITUDE_BASE)
                CheckRangeAndSetF(&phfc->altitude_base, item->value1.f, 0, 9999);
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
                    // NOTE::MRI: What is this being used for
                    HeadingUpdate(value, 1);
            }
            else
            if (sub_param==TELEM_PARAM_CTRL_HEADING_ABS)
                CheckRangeAndSetF(&phfc->ctrl_out[ANGLE][YAW], item->value1.f, -180, 180);
            else
            if (sub_param==TELEM_PARAM_CTRL_WIND_COMP)
                CheckRangeAndSetB(&phfc->rw_cfg.wind_compensation, item->value1.i, 0, 1);
            else
            if (sub_param==TELEM_PARAM_CTRL_PATH_NAVIG)
                CheckRangeAndSetB(&phfc->rw_cfg.path_navigation, item->value1.i, 0, 1);
            else
            if (sub_param==TELEM_PARAM_CTRL_ANGLE_COLL_MIX)
                CheckRangeAndSetF(&phfc->rw_cfg.AngleCollMixing, item->value1.f, 0, 2);
            else
            if (sub_param==TELEM_PARAM_CTRL_CRUISE_LIMIT)
                CheckRangeAndSetF(&phfc->rw_cfg.cruise_speed_limit, item->value1.f, 0, 100);
            else
            if (sub_param==TELEM_PARAM_CTRL_NOSE2WP)
                CheckRangeAndSetB(&phfc->rw_cfg.nose_to_WP, item->value1.i, 0, 1);
            else
            if (sub_param==TELEM_PARAM_CTRL_BAT_CAPACITY)
                CheckRangeAndSetI(&phfc->box_dropper_, item->value1.i, 0, 1);
        }
    }
    else if (item->type == PL_ITEM_DELAY)
    {
      int current_time_seconds = gps.GetDayTimeInSecs();

      bool start_time_set = item->data[0];
      int delay_seconds = item->value1.i;
      int start_time_seconds = item->value2.i;

      if (phfc->delay_time < 0) {
        if (start_time_set) {
          phfc->delay_time = start_time_seconds + delay_seconds;
        }
        else {
          phfc->delay_time = current_time_seconds + delay_seconds;
        }
      }
    }
}

// @brief
// @param
// @retval
static void Playlist_ProcessBottom(FlightControlData *hfc, bool retire_waypoint)
{
    T_PlaylistItem *item;
    static int goto_counter = -1;
    
    /* single waypoint mode handling - waypoint retire logic */
    if ((phfc->ctrl_source==CTRL_SOURCE_AUTOPILOT) && (phfc->playlist_status!=PLAYLIST_PLAYING))
    {
        if (retire_waypoint) {
            telem.SelectCtrlSource(CTRL_SOURCE_AUTOPILOT);
            phfc->waypoint_type = WAYPOINT_NONE;
        }
        return;
    }
    
    /* process only running playlist mode */
    if (phfc->playlist_status!=PLAYLIST_PLAYING)
        return;
        
    item = &phfc->playlist[phfc->playlist_position];

    if (item->type == PL_ITEM_WP)
    {
        if (item->data[0]==WAYPOINT_TAKEOFF)
        {
            if (phfc->waypoint_stage == FM_TAKEOFF_COMPLETE)
            {
                phfc->waypoint_type  = WAYPOINT_NONE;
                phfc->playlist_position++;
            }
        }
        else if (item->data[0]==WAYPOINT_TOUCH_AND_GO) {
            if (phfc->waypoint_stage == FM_TAKEOFF_COMPLETE)
            {
                phfc->waypoint_type  = WAYPOINT_NONE;
                phfc->playlist_position++;
            }
        }
        else
        {
            /* if WP is blocking, wait for the retire flag */
            if (!item->data[1] || retire_waypoint)  // WP completion flag
            {
                phfc->playlist_position++;
                phfc->pl_wp_initialized = false;
            }
        }
    }    
    else if (item->type == PL_ITEM_PARAM)
    {
        phfc->playlist_position++;
    }
    else if (item->type == PL_ITEM_GOTO)
    {
      if (item->data[0] == PL_GOTO_COUNTER)
      {
        if (goto_counter == -1)
        {
          // first time in loop, ensure the requested jump to
          // position is before our current position in the loop
          // otherwise, just skip this step.
          if (item->value1.i < phfc->playlist_position) {
            hfc->playlist_position = item->value1.i;
            goto_counter = item->value2.i-1;
          }
          else {
            hfc->playlist_position++;
          }
        }
        else if (goto_counter == 0)
        {
          // end loop
          hfc->playlist_position++;
          goto_counter = -1;
        }
        else {
          hfc->playlist_position = item->value1.i;
          --goto_counter;
        }
      }
    }
    else if (item->type == PL_ITEM_DELAY)
    {
      if( gps.GetDayTimeInSecs() >= hfc->delay_time) {
          hfc->playlist_position++;
          hfc->delay_time = -1;
      }
    }
    else if (item->type == PL_ITEM_HOLD)
    {
        phfc->playlist_position++;
    }
    else    // PL_ITEM_END and every thing else will stop playlist
        phfc->playlist_position = phfc->playlist_items;
        
    
    /* check for end of the playlist or errors */
    if ((phfc->playlist_status > PLAYLIST_STOPPED) && (phfc->playlist_position >= phfc->playlist_items))
    {
        /* stop playlist and waypoint mode */
        phfc->playlist_status = PLAYLIST_STOPPED;
        phfc->playlist_position = phfc->playlist_items;

        /* if in flight, put a waypoint at the current position, else do nothing */
        if (phfc->throttle_armed) {
            telem.SetZeroSpeed();
        }
    }
}

// @brief
// @param
// @retval
static void AbortFlight(void)
{
  // Ensure we are in AutoPilot
  SetRCRadioControl();  // This will set Autopilot on loss of RcLink

  if ((phfc->waypoint_type != WAYPOINT_LANDING) && !phfc->touch_and_go_landing) {
    if (IN_THE_AIR(phfc->altitude_lidar)) {
      float lat;
      float lon;
      float altitude;

      int site = telem.FindNearestLandingSite();
      if (site >= 0)
      {
        // Goto nearest landing site and land
        lat = phfc->landing_sites[site].lat;
        lon = phfc->landing_sites[site].lon;
        altitude = phfc->landing_sites[site].altitude - phfc->altitude_base + phfc->landing_sites[site].above_ground;
      }
      else
      {
        // go home, and then land
        lat = phfc->home_pos[0];
        lon = phfc->home_pos[1];
        altitude = -9999;
      }

      telem.CommandLandingWP(lat, lon, altitude);
      phfc->pid_Dist2T.COmax = pConfig->landing_appr_speed;
    }
    else {
      telem.Disarm();
    }
  }
}

// @brief
// @param
// @retval
static void initRpmThresholdCheck(void)
{
  phfc->rpm_state = RPM_SPOOLING;
  phfc->spool_timeout = RPM_SPOOL_TIMEOUT;
}

// Return -1 on error
// 0 on success
// 1 on waiting
// @brief
// @param
// @retval
static int rpm_threshold_check(float dT)
{
  switch (phfc->rpm_state)
  {
  case RPM_SPOOLING:
    // wait for rpm to be with x% of configured value.
    // if rpm not reached within that time, exit
    phfc->spool_timeout -= dT;
    if (phfc->spool_timeout > 0) {
      if (N1WithinPercentOfN2(phfc->RPM, RPM_THRESHOLD_ERROR, pConfig->rpm_typical)) {
        phfc->rpm_state = RPM_SPOOLED;
        phfc->spool_hold_timeout = RPM_HOLD_TIMEOUT;
      }
    }
    else {
      // did not spool to required rpm is designated time, fail the sequence.
      phfc->rpm_state = RPM_FAIL;
      return -1;
    }
    break;
  case RPM_SPOOLED:
    // Now we are spooled, we expect to stay with x% desired rpm for a set time
    // before we are ready to continue with sequece.
    phfc->spool_hold_timeout -= dT;
    if (N1WithinPercentOfN2(phfc->RPM, RPM_THRESHOLD_ERROR*2, pConfig->rpm_typical)) {
      if (phfc->spool_hold_timeout < 0) {
        phfc->rpm_state = RPM_DONE;
        return 0;
      }
    }
    else {
      // drop out of rpm % error, fail the sequence
      phfc->rpm_state = RPM_SPOOLING;
      //return -1;
    }
    break;

  default:
    break;
  }

  return 1;
}

static void ProcessTakeoff(float dT)
{
    if (    (phfc->waypoint_stage == FM_TAKEOFF_WAIT)
         && (    (phfc->message_from_ground>0)
              || (phfc->message_timeout<=0)         ) )
    {

      /* cancel and disarmed */
      if (phfc->message_from_ground!=CMD_MSG_TAKEOFF_OK || phfc->message_timeout<=0)
      {
          phfc->inhibitRCswitches = false;
          /* send message that takeoff has timed out */
          if (phfc->message_timeout<=0)
              telem.SendMsgToGround(MSG2GROUND_TAKEOFF_TIMEOUT);

          telem.Disarm();
          return;
      }

      phfc->waypoint_stage  = FM_TAKEOFF_AUTO_SPOOL;

    }
    else if (   (phfc->waypoint_stage == FM_TAKEOFF_AUTO_SPOOL)
             && (   (phfc->message_from_ground>0)
                 || (phfc->message_timeout<=0)                ) )
    {
        /* cancel and disarmed */
        if (phfc->message_from_ground!=CMD_MSG_TAKEOFF_OK || phfc->message_timeout<=0)
        {
            phfc->inhibitRCswitches = false;
            /* send message that takeoff has timed out */
            if (phfc->message_timeout<=0)
                telem.SendMsgToGround(MSG2GROUND_TAKEOFF_TIMEOUT);

            telem.Disarm();
            return;
        }

        /* set the throttle value to spool */
        phfc->throttle_value = pConfig->Stick100range;

        initRpmThresholdCheck();
        phfc->waypoint_stage = FM_TAKEOFF_RPM_CHECK;

    }
    else if (phfc->waypoint_stage == FM_TAKEOFF_RPM_CHECK) {

      int rpm_check = -1;
      static time_t spool_time = gps.GetEpochTimeInSecs(DEFAULT_FIXED_PROP_SPOOL_TIME);

      if (pConfig->throttle_ctrl==PROP_VARIABLE_PITCH) {
        rpm_check = rpm_threshold_check(dT);

        if (rpm_check < 0) {
          // Cancel and disarm
          phfc->inhibitRCswitches = false;
          /* send message that takeoff has timed out */
          phfc->message_timeout = 0;
          telem.SendMsgToGround(MSG2GROUND_TAKEOFF_TIMEOUT);
          telem.Disarm();
          return;
        }
      }
      else if(spool_time <= gps.GetEpochTimeInSecs(0)) { //PROP_FIXED_PITCH spooled up
        rpm_check = 0;
      }

      if (rpm_check == 0) {
        // rpm reached - continue with takeoff

        if (phfc->auto_takeoff || phfc->touch_and_go_takeoff) {
          phfc->message_from_ground = CMD_MSG_TAKEOFF_ALLOWED;
        }
        else {
          telem.SendMsgToGround(MSG2GROUND_ALLOW_TAKEOFF);
          phfc->message_from_ground = 0;   // reset it so we can wait for the message from ground
        }

        phfc->message_timeout = DEFAULT_TAKEOFF_TIMEOUT;    // 60 seconds
        phfc->waypoint_stage  = FM_TAKEOFF_ARM;
      }
    }
    else if (   (phfc->waypoint_stage == FM_TAKEOFF_ARM)
             && (   (phfc->message_from_ground==CMD_MSG_TAKEOFF_ALLOWED)
                  || (phfc->message_from_ground==CMD_MSG_TAKEOFF_ABORT)
                  || (phfc->message_timeout<=0)                          ) )
    {
        phfc->inhibitRCswitches = false;
        /* cancel and disarmed */
        if (   (phfc->message_from_ground!=CMD_MSG_TAKEOFF_ALLOWED)
            || (GetMotorsState() == 0)
            || (phfc->message_timeout<=0))
        {
            /* send message that takeoff has timed out */
            if (phfc->message_timeout <= 0) {
                telem.SendMsgToGround(MSG2GROUND_TAKEOFF_TIMEOUT);
            }
            telem.Disarm();
            return;
        }

        if (!phfc->touch_and_go_takeoff) {
          // AVI says "Commence Takeoff Sequence"
          telem.SendMsgToGround(MSG2GROUND_TAKEOFF);
        }

        telem.SelectCtrlSource(CTRL_SOURCE_AUTOPILOT);
        telem.SaveValuesForAbort();

        /* set PRY controls to Angle mode, coll to manual */
        SetCtrlMode(phfc, pConfig, PITCH, CTRL_MODE_ANGLE);
        SetCtrlMode(phfc, pConfig, ROLL,  CTRL_MODE_ANGLE);
        SetCtrlMode(phfc, pConfig, YAW,   CTRL_MODE_ANGLE);

        /* initialize PRY angles to the current orientation */
        phfc->ctrl_out[ANGLE][PITCH] = phfc->IMUorient[PITCH]*R2D;
        phfc->ctrl_out[ANGLE][ROLL]  = phfc->IMUorient[ROLL]*R2D;
        phfc->ctrl_out[ANGLE][YAW]   = phfc->IMUorient[YAW]*R2D;
        phfc->ctrl_angle_pitch_3d = phfc->ctrl_out[ANGLE][PITCH];
        phfc->ctrl_angle_roll_3d  = phfc->ctrl_out[ANGLE][ROLL];

        if (!phfc->touch_and_go_takeoff) {
          /* set home position */
          telem.SetHome();
        }

        telem.SetTakeoffPosition();

        /* set vspeed to the takeoff speed
         * set target collective to max value and wait for the heli to clear ground*/
        phfc->ctrl_collective_3d  = phfc->takeoff_vertical_speed; //phfc->pid_CollVspeed.COmax;
        phfc->fixedThrottleMode = THROTTLE_FLY;                  // rvw
        /* default PID values */
        telem.ApplyDefaults();
        /* slow max vspeed to make collective to move slowely */
//          phfc->pid_CollAlt.COmax = pConfig->throttle_ctrl==PROP_VARIABLE_PITCH ? 0.1f : 0.5f;
        phfc->waypoint_stage  = FM_TAKEOFF_START;
    }
    else
    if (phfc->waypoint_stage == FM_TAKEOFF_START)
    {
        float thr = ClipMinMax(pConfig->CollThrAutoLevel, 0, 1);
        float limit = (1-thr)*pConfig->CollZeroAngle + thr*phfc->pid_CollVspeed.COofs;
        /* set pitch/roll angle to trim values once collective exceeds the set % of hover value */
        if ((phfc->ctrl_collective_raw>limit) || (phfc->IMUspeedGroundENU[2]>0.2f))
        {
            phfc->ctrl_angle_pitch_3d = 0;// phfc->pid_PitchSpeed.COofs;
            phfc->ctrl_angle_roll_3d  = phfc->pid_RollSpeed.COofs + pConfig->tail_rotor_roll_trim;

            phfc->waypoint_stage = FM_TAKEOFF_LEVEL;
        }
    }
    else
    if (phfc->waypoint_stage == FM_TAKEOFF_LEVEL)
    {
        /* starting to take off, watch for the ENU speeds exceeding the threshold and switch to alt hold mode */
        if ((ABS(phfc->IMUspeedGroundENU[0])>0.4f) || (ABS(phfc->IMUspeedGroundENU[1])>0.4f) || (phfc->IMUspeedGroundENU[2]>0.5f))
        {
            /* enable alt hold mode */
            phfc->ctrl_out[RAW][COLL]  = phfc->ctrl_collective_raw;
            phfc->ctrl_out[SPEED][COLL] = phfc->takeoff_vertical_speed;    // initialize to current vert speed
            phfc->pid_CollAlt.COmax = phfc->takeoff_vertical_speed;

            SetCtrlMode(phfc, pConfig, COLL,  CTRL_MODE_POSITION);
            phfc->ctrl_out[POS][COLL] = phfc->takeoff_pos[2] + ClipMinMax(phfc->takeoff_height, TAKEOFF_HEIGHT_MIN, TAKEOFF_HEIGHT_MAX);
            phfc->waypoint_pos[2] = phfc->ctrl_out[POS][COLL];    // needs to be initialized for further waypoint flying if altitude is not specified


            /* enable horizontal speed control with zero speed */
            SetCtrlMode(phfc, pConfig, PITCH, CTRL_MODE_SPEED);
            SetCtrlMode(phfc, pConfig, ROLL,  CTRL_MODE_SPEED);
            phfc->ctrl_out[SPEED][PITCH] = 0;
            phfc->ctrl_out[SPEED][ROLL]  = 0;

            phfc->waypoint_stage = FM_TAKEOFF_SPEED;
        }
    }
    else
    if (phfc->waypoint_stage == FM_TAKEOFF_SPEED)
    {
        /* once in horizontal speed mode, watch the altitude and switch to position and altitude hold at 0.3meters */
        if (phfc->altitude > (phfc->takeoff_pos[2]+0.3f))
        {
          phfc->waypoint_pos[0] = phfc->takeoff_pos[0];
          phfc->waypoint_pos[1] = phfc->takeoff_pos[1];

          phfc->waypoint_retire = 0;
          SetCtrlMode(phfc, pConfig, PITCH, CTRL_MODE_POSITION);
          SetCtrlMode(phfc, pConfig, ROLL,  CTRL_MODE_POSITION);
          phfc->waypoint_stage = FM_TAKEOFF_HOLD;
        }
    }
    else
    if (phfc->waypoint_stage == FM_TAKEOFF_HOLD)
    {
      int retire_takeoff_height = phfc->waypoint_pos[2];

      // if we are playing a playlist and the takeoff is not the last item in the playlist, then
      // retire the takeoff TAKEOFF_HEIGHT_RETIRE_OFFSET meters earlier.
      if ( (phfc->playlist_status > PLAYLIST_STOPPED) && (phfc->playlist_position < (phfc->playlist_items - 1)) ){
        retire_takeoff_height = max(phfc->takeoff_pos[2]+TAKEOFF_HEIGHT_MIN, phfc->waypoint_pos[2]-TAKEOFF_HEIGHT_RETIRE_OFFSET);
      }

      // Once altitude reaches the requested waypoint altitude within TAKEOFF_HEIGHT_RETIRE_OFFSET, Takeoff is complete
      if (phfc->altitude >= retire_takeoff_height) {
        // make sure the next waypoint uses the intended takeoff height
        phfc->altitude_WPnext = phfc->waypoint_pos[2] - phfc->altitude_base;
        phfc->waypoint_stage = FM_TAKEOFF_COMPLETE;
        phfc->auto_takeoff = false;
      }
    }
}

static void ProcessLanding(float dT)
{
  if (phfc->waypoint_stage == FM_LANDING_STOP)
  {
      if ((phfc->setZeroSpeed == false) && (gps.gps_data_.HspeedC <= phfc->rw_cfg.GTWP_retire_speed))
      {
          /* send out message and setup timeout */
          phfc->waypoint_stage = FM_LANDING_HOLD;
          phfc->message_from_ground = 0;   // reset it so we can wait for the message from ground
          phfc->message_timeout = DEFAULT_LANDING_TIMEOUT;

          if(phfc->auto_landing) {
            phfc->message_timeout = DEFAULT_TOUCH_AND_GO_LANDING_TIMEOUT;    // 3 seconds
            phfc->message_from_ground = CMD_MSG_LANDING_GO ;
          }
          else {
            telem.SendMsgToGround(MSG2GROUND_ALLOW_LANDING);
          }
      }
  }
  else
  if (phfc->waypoint_stage == FM_LANDING_WAYPOINT)
  {
      if (gps.gps_data_.HspeedC <= phfc->rw_cfg.GTWP_retire_speed && phfc->gps_to_waypoint[0] <= phfc->rw_cfg.GTWP_retire_radius)
      {
          /* send out message and setup timeout */
          phfc->waypoint_stage = FM_LANDING_HOLD;
          phfc->message_from_ground = 0;   // reset it so we can wait for the message from ground
          phfc->message_timeout = DEFAULT_LANDING_TIMEOUT;

          if(phfc->auto_landing || phfc->touch_and_go_landing) {
            phfc->message_timeout = DEFAULT_TOUCH_AND_GO_LANDING_TIMEOUT;    // 3 seconds
            phfc->message_from_ground = CMD_MSG_LANDING_GO ;
          }
          else {
            telem.SendMsgToGround(MSG2GROUND_ALLOW_LANDING);
          }
      }
  }
  else if (phfc->waypoint_stage == FM_LANDING_HOLD && (phfc->message_from_ground==CMD_MSG_LANDING_GO || phfc->message_timeout<=0))
  {
    if (phfc->touch_and_go_landing) {
      phfc->takeoff_height = phfc->altitude - phfc->altitude_base;
    }
    /* check for incoming message or timeout */
    telem.CommandLanding(false, false);
  }
  else if (phfc->waypoint_stage == FM_LANDING_HIGH_ALT)
  {
      if (phfc->altitude_lidar <= 3)
          telem.CommandLanding(true, false);
  }
  else
  if (phfc->waypoint_stage == FM_LANDING_LOW_ALT)
  {
      /* heli coming down, watch collective manual value, once below 80% of hover, switch to rate mode */
//            float CO_thr = 0.7f*phfc->pid_CollVspeed.COofs + 0.3f*phfc->CollZeroAngle;
//            if (phfc->pid_CollVspeed.COlast < CO_thr)
      float landing_threshold;
      if (pConfig->throttle_ctrl==PROP_FIXED_PITCH) {
        landing_threshold = LANDING_THRESHOLD_HEIGHT_FIXED_PROP;
      }
      else {
        landing_threshold = LANDING_THRESHOLD_HEIGHT_VARIABLE_PROP;
      }

      /* touching down, switch to rate mode once below threshold */
      if (phfc->altitude_lidar <= landing_threshold)
      {
          /* set PRY controls to rate */
          SetCtrlMode(phfc, pConfig, PITCH, CTRL_MODE_RATE);
          SetCtrlMode(phfc, pConfig, ROLL,  CTRL_MODE_RATE);
          SetCtrlMode(phfc, pConfig, YAW,   CTRL_MODE_RATE);

          phfc->ctrl_out[RATE][PITCH] = 0;
          phfc->ctrl_out[RATE][ROLL]  = 0;
          phfc->ctrl_out[RATE][YAW]   = 0;
          /* double the vertical speed just before touchdown to reduce the jump
           * and to reduce the time blades take to flatten */
//                if (pConfig->throttle_ctrl==PROP_VARIABLE_PITCH)
          phfc->ctrl_vspeed_3d = -pConfig->landing_vspeed*0.6f;
          phfc->waypoint_stage = FM_LANDING_TOUCHDOWN;

          if (pConfig->throttle_ctrl==PROP_FIXED_PITCH) {
            phfc->landing_timeout = LANDING_THRESHOLD_HEIGHT_FIXED_PROP / phfc->ctrl_vspeed_3d; // in seconds
          }
      }
  }
  else
  if (phfc->waypoint_stage == FM_LANDING_TOUCHDOWN)
  {
    if (pConfig->throttle_ctrl==PROP_FIXED_PITCH) {
      phfc->landing_timeout -= dT;

      // wait for timeout to expire
      if (phfc->landing_timeout <= 0) {
        //Now decay the collective speed as fast as possible based on config
        phfc->ctrl_vspeed_3d -= 2*pConfig->landing_vspeed_acc*dT;

        if (phfc->ctrl_vspeed_3d <= phfc->pid_CollAlt.COmin) {
          if (phfc->touch_and_go_landing) {
            /* stay spooled */
            phfc->waypoint_stage = FM_TOUCH_AND_GO_WAIT;

            phfc->touch_and_go_wait += gps.GetDayTimeInSecs();
            phfc->touch_and_go_do_the_thing_cnt += gps.GetDayTimeInSecs();
          }
          else {
            if (phfc->playlist_status == PLAYLIST_PLAYING) {
              phfc->playlist_position++;
              phfc->playlist_status = PLAYLIST_STOPPED;
            }
            telem.Disarm();
          }
        }
      }
    }
    else {
      /* on the ground in rate mode, wait till coll is at zero angle and then shut down */
      if (phfc->pid_CollVspeed.COlast <= pConfig->CollZeroAngle)
      {
        SetCtrlMode(phfc, pConfig, COLL,  CTRL_MODE_MANUAL);
        phfc->ctrl_out[RAW][COLL] = phfc->pid_CollVspeed.COlast;
        phfc->ctrl_collective_raw = phfc->pid_CollVspeed.COlast;
        phfc->ctrl_collective_3d = phfc->pid_CollVspeed.COlast;

        if (phfc->touch_and_go_landing) {
          /* stay spooled */
          phfc->waypoint_stage = FM_TOUCH_AND_GO_WAIT;
        }
        else {
          if (phfc->playlist_status == PLAYLIST_PLAYING) {
            phfc->playlist_position++;
            phfc->playlist_status = PLAYLIST_STOPPED;
          }
          telem.Disarm();
        }
      }
    }
  }
}

// @brief
// @param
// @retval
static void ProcessTouchAndGo(float dT)
{
  if(phfc->touch_and_go_landing) {
    ProcessLanding(dT);
  }

  if(phfc->waypoint_stage == FM_TOUCH_AND_GO_WAIT) {

    phfc->touch_and_go_landing = false;

    phfc->inhibitRCswitches = true;

    /* reset all I terms */
    ResetIterms();

    /* set PRY controls to rate mode, coll to manual while on ground*/
    SetCtrlMode(phfc, pConfig, PITCH, CTRL_MODE_RATE);
    SetCtrlMode(phfc, pConfig, ROLL,  CTRL_MODE_RATE);
    SetCtrlMode(phfc, pConfig, YAW,   CTRL_MODE_RATE);
    SetCtrlMode(phfc, pConfig, COLL,  CTRL_MODE_MANUAL);

    /* initialize PRY angles to the current orientation */
    phfc->ctrl_out[RATE][PITCH] = 0;
    phfc->ctrl_out[RATE][ROLL]  = 0;
    phfc->ctrl_out[RATE][YAW]   = 0;

    phfc->ctrl_collective_raw = phfc->collective_raw_curr;    // set to current position

    //For Heli pConfig->CollZeroAngle = collective zero angle
    //For fixed prop UAV pConfig->CollZeroAngle = idle/spool up motor speed
    phfc->ctrl_collective_3d  = pConfig->CollZeroAngle;

    telem.SelectCtrlSource(CTRL_SOURCE_AUTOPILOT);

    phfc->fixedThrottleMode = THROTTLE_IDLE;  // for fixed prop UAVs
    phfc->throttle_value = pConfig->Stick100range;

    phfc->rc_ctrl_request = false;

    phfc->touch_and_go_wait -= dT;
    phfc->touch_and_go_do_the_thing_cnt -= dT;

    // phfc->touch_and_go_do_the_thing flag is set by message_from_ground
    if ( (gps.GetDayTimeInSecs() >= phfc->touch_and_go_do_the_thing_cnt) && phfc->touch_and_go_do_the_thing) {
        phfc->touch_and_go_do_the_thing_cnt = 0;

        //do the thing
        phfc->box_dropper_  = (phfc->box_dropper_ == 0) ? 1 : 0;
        FCM_DROP_BOX_CTRL(phfc->box_dropper_);
        phfc->touch_and_go_do_the_thing = false;
      }

    if(gps.GetDayTimeInSecs() >= phfc->touch_and_go_wait) {
      phfc->touch_and_go_wait = 0;

      phfc->message_timeout = DEFAULT_TAKEOFF_TIMEOUT;
      phfc->message_from_ground = CMD_MSG_TAKEOFF_OK;

      phfc->waypoint_stage = FM_TAKEOFF_AUTO_SPOOL;
      phfc->touch_and_go_takeoff = true;
    }
  }

  if(phfc->touch_and_go_takeoff) {

    ProcessTakeoff(dT);
    if(phfc->waypoint_stage == FM_TAKEOFF_COMPLETE) {
      phfc->touch_and_go_takeoff = false;
    }
  }
}

// this function runs after the previous control modes are saved,
// thus PIDs will get automatically re-initialized on mode change
//
// @brief
// @param
// @retval
static void ProcessFlightMode(FlightControlData *hfc, float dT)
{
  static float abortTimer = ABORT_TIMEOUT_SEC;
  static float led_timer = 1.0; // toggle time for Armed LED
  static int led_state = 1;

    // If all these conditions are true for Abort_timeout time,
    // then we decide we no longer have any way to receive instructions and we
    // make the decision to return nearest safe landing point
    if (phfc->throttle_armed
          && !telem.IsOnline()
          && !xbus.RcLinkOnline()
          && (phfc->playlist_status != PLAYLIST_PLAYING)
          && (phfc->ctrl_source != CTRL_SOURCE_AFSI)) {

      if (abortTimer <= 0) {

        // Toggle Arm LED to show we are aborting the flight
        led_timer -= dT;
        if (led_timer <= 0) {

          FCM_SET_ARM_LED(led_state);

          led_state ^= 1;
          led_timer = 1.0;
        }

        AbortFlight();
      }
      else {
        abortTimer -= dT;
      }
    }
    else {
      abortTimer = ABORT_TIMEOUT_SEC;

      // Process FCM reserved LEDS.
      // PWM_5 reserved for FCM box dropper
      // PWM_6 reserved for ARM led
      FCM_SET_ARM_LED(phfc->throttle_armed);
      FCM_DROP_BOX_CTRL(phfc->box_dropper_);
    }

    if (phfc->message_timeout>0) {
        phfc->message_timeout -= phfc->ticks_curr;
    }

    if (hfc->waypoint_type == WAYPOINT_TAKEOFF) {
      ProcessTakeoff(dT);
    }
    else if (hfc->waypoint_type == WAYPOINT_LANDING) {
      ProcessLanding(dT);
    }
    else if (phfc->waypoint_type == WAYPOINT_TOUCH_AND_GO) {
      ProcessTouchAndGo(dT);
    }
}

// @brief
// @param
// @retval
static void SetSpeedAcc(float *value, float speed, float acc, float dT)
{
    float v = *value;
    float delta = acc*dT;
    v = ClipMinMax(speed, v-delta, v+delta);
    *value = v;
}

// @brief
// @param
// @retval
static float CalcMinAboveGroundAlt(float speed)
{
    float alt = ClipMinMax((speed - pConfig->LidarHVcurve[0]) * pConfig->LidarHVcurve[1] + pConfig->LidarHVcurve[2],
                               pConfig->LidarHVcurve[2], pConfig->LidarHVcurve[3]);

    // TODO::SP: Useful Lidar Max reading is 40m, but we use threshold of 35m
    // ensure lidar altitude control is within that range. - always lidar
    alt = min(alt, 35);
    return alt;
}

// @brief
// @param
// @retval
static void ServoUpdateRAW(float dT)
{
    static float led_timer = 2.0;  // toggle time for Armed LED
    static int led_state = 1;

    // Blink the ARM LED to inform user that we are in SERO-RAW MODE.
    led_timer -= dT;
    if (led_timer <= 0) {

      FCM_SET_ARM_LED(led_state);

      led_state ^= 1;
      led_timer = 1.0;
    }

    char xbus_new_values = xbus.NewValues(dT, phfc->throttle_armed, phfc->fixedThrottleMode);

#if 0
    if ((phfc->print_counter %500) == 0) {
        debug_print("Xbus");
        for (int i=0; i < 16; i++) {
            debug_print("[%d]=%f ", i, xbus.valuesf[i]);
            if (i == 7) {
                debug_print("\r\n");
            }
        }
        debug_print("\r\n");

        // full_auto, auto_throttle, ctrl_source, fixedThrottleMode, throttle_value, collective_value
        debug_print("FA[%d] AT[%d] CS[%d] FTM[%d] TV[%f] CV[%f]\r\n",
                        phfc->full_auto, phfc->auto_throttle, phfc->ctrl_source,
                        phfc->fixedThrottleMode, phfc->throttle_value, phfc->collective_value);

        debug_print("RAW Servo out : ");
        for (int i=0; i < 2; i++) {
          for (int j=0; j < 8; i++) {
            debug_print("[%d][%f] ", phfc->servos_out[i][j]);
          }
        }

        debug_print("\r\n\r\n");
    }
#endif

    float AngleCompensation = COSfD(min(45, ABS(phfc->IMUorient[PITCH]*R2D))) * COSfD(min(45, ABS(phfc->IMUorient[ROLL]*R2D)))
                                      * sqrt(1 / (COSfD(min(45, ABS(phfc->IMUorient[PITCH]*R2D))) + COSfD(min(45, ABS(phfc->IMUorient[ROLL]*R2D)))
                                      - COSfD(min(45, ABS(phfc->IMUorient[PITCH]*R2D))) * COSfD(min(45, ABS(phfc->IMUorient[ROLL]*R2D)))));

//    float throttle_prev = phfc->ctrl_out[RAW][THRO];

    // Always default to using lidar node 0.
    // This is either a single or the front lidar for tandems.
    if (phfc->lidar_online_mask & 0x1) {
      phfc->altitude_lidar = (phfc->altitude_lidar_raw[SINGLE_FRONT_LIDAR_INDEX] - pConfig->lidar_offset/1000.0f) * AngleCompensation;
    }
    else if (phfc->lidar_online_mask & 0x2) {
      // Rear Lidar (for Tandems)
      phfc->altitude_lidar = (phfc->altitude_lidar_raw[REAR_TANDEM_LIDAR_INDEX] - pConfig->lidar_offset/1000.0f) * AngleCompensation;
    }
    else {
      // since we have lost all available lidar data, replace with imu fusion altitude
      // This at least will give a fighting chance ;-)
      // TODO::SP: Validate this.....
      phfc->altitude_lidar = phfc->gps_to_home[2]; //phfc->altitude;
    }

    phfc->fixedThrottleMode = THROTTLE_IDLE;
    phfc->throttle_value = -pConfig->Stick100range; // Throttle off

    if( phfc->rc_ctrl_request) {

      if (pConfig->throttle_ctrl==PROP_VARIABLE_PITCH) {
        phfc->collective_value = xbus.valuesf[XBUS_THRO];
      }
      else {
        phfc->collective_value = -pConfig->Stick100range;
      }

      phfc->ctrl_out[RAW][THRO]  = phfc->throttle_value;
      phfc->ctrl_out[RAW][PITCH] = xbus.valuesf[XBUS_PITCH];
      phfc->ctrl_out[RAW][ROLL]  = xbus.valuesf[XBUS_ROLL];
      phfc->ctrl_out[RAW][YAW]   = xbus.valuesf[XBUS_YAW];
      phfc->ctrl_out[RAW][COLL]  = phfc->collective_value;
    }
    else if (!phfc->rc_ctrl_request) {
      telem.Disarm();
      phfc->collective_value = -pConfig->Stick100range;
    }

    if (phfc->throttle_armed || (GetMotorsState()==1) ) {
      telem.Disarm();
    }

    if (xbus_new_values) {
        if (xbus_new_values == XBUS_NEW_VALUES_1ST) {
            telem.SaveValuesForAbort();
        }
        SetRCRadioControl();
    }

    telem.ProcessCommands();

    ProcessStickInputs(phfc, dT);

    phfc->collective_raw_curr = phfc->ctrl_out[RAW][COLL];
    phfc->ctrl_out[RAW][THRO] += phfc->rw_cfg.throttle_offset;

    phfc->mixer_in[PITCH] = phfc->ctrl_out[RAW][PITCH];
    phfc->mixer_in[ROLL]  = phfc->ctrl_out[RAW][ROLL];
    phfc->mixer_in[YAW]   = phfc->ctrl_out[RAW][YAW];
    phfc->mixer_in[COLL]  = phfc->ctrl_out[RAW][COLL];

    if (pConfig->throttle_ctrl==PROP_FIXED_PITCH) {
        phfc->ctrl_out[RAW][THRO] = phfc->ctrl_out[RAW][COLL];

        // if lever is low, set throttle to minimum and everything else to 0
        // to prevent any prop from accidental spinning because of PIDs
        if (phfc->throttle_value < -0.50f || !phfc->throttle_armed || (phfc->control_mode[COLL] < CTRL_MODE_SPEED && phfc->collective_value < -0.50f)
                || (phfc->waypoint_type == WAYPOINT_TAKEOFF && (phfc->waypoint_stage == FM_TAKEOFF_ARM || phfc->waypoint_stage == FM_TAKEOFF_AUTO_SPOOL))) {

            if (phfc->throttle_armed && phfc->throttle_value > -0.5f && phfc->waypoint_type == WAYPOINT_TAKEOFF
                    && (phfc->waypoint_stage == FM_TAKEOFF_ARM || phfc->waypoint_stage == FM_TAKEOFF_AUTO_SPOOL)) {

                ResetIterms();
                phfc->ctrl_out[RAW][THRO] = -pConfig->Stick100range;
                phfc->ctrl_out[RAW][PITCH] = 0;
                phfc->ctrl_out[RAW][ROLL]  = 0;
                phfc->ctrl_out[RAW][YAW]   = 0;
            }
        }
    }

    if (pConfig->ctrl_mode_inhibit[THRO] || !phfc->throttle_armed) {
        // Sets to Minimum Throttle
        phfc->ctrl_out[RAW][THRO] = pConfig->throttle_values[0];
    }

    if (pConfig->ctrl_mode_inhibit[PITCH]) {
        phfc->ctrl_out[RAW][PITCH] = phfc->pid_PitchRate.COofs;
    }

    if (pConfig->ctrl_mode_inhibit[ROLL]) {
        phfc->ctrl_out[RAW][ROLL] = phfc->pid_RollRate.COofs;
    }

    if (pConfig->ctrl_mode_inhibit[YAW]) {
        phfc->ctrl_out[RAW][YAW] = phfc->pid_YawRate.COofs;
    }

    if (pConfig->ctrl_mode_inhibit[COLL]) {
        phfc->ctrl_out[RAW][COLL] = phfc->pid_CollVspeed.COofs;
    }

    phfc->mixer_in[THRO]  = phfc->ctrl_out[RAW][THRO];

    ServoMixer();

    ServoOutput();
}

// Mid point on Xbus Channel 13 (right side slider) is 0.1875
// Max is 0.444, Min is 0
// 15 notches from Mid to Top, & 14 from Mid to bottom
// We scale the Configured Max, Min % of adjustment across these ranges
// @brief
// @param
// @retval
static inline void UpdatePitchRateScalingFactor(float xbus_value)
{
  float pid_scale;

  if (xbus_value > 0.188) {
    pid_scale = ((xbus_value - 0.188) / 0.017);
    phfc->pid_PitchRateScalingFactor = phfc->positive_pid_scaling * pid_scale;
  }
  else if (xbus_value < 0.187) {
    pid_scale = 14 - (xbus_value / 0.0134);
    phfc->pid_PitchRateScalingFactor = phfc->negative_pid_scaling * pid_scale;
  }
  else {
    // At mid point, set scaling to 0 (i.e no effect)
    phfc->pid_PitchRateScalingFactor = 0;
  }
}

// Allow for dynamically updating PID values based on a set scaling factor
// In-lined for speed - (Compiler should be in-lining this anyhow)
// @brief
// @param
// @retval
static inline void ApplyPidScaling(T_PID *pid_layer, const float params[6], float pid_scaling)
{
  float gain = pid_scaling / 100;
  CheckRangeAndSetF(&pid_layer->Kp, (params[0] + params[0] * gain), -100, 100);
  CheckRangeAndSetF(&pid_layer->Ki, (params[1] + params[1] * gain), -100, 100);
  CheckRangeAndSetF(&pid_layer->Kd, (params[2] + params[2] * gain), -100, 100);
}

// Reset specified PID values back to those held in the configuration
// In-lined for speed - (Compiler should be in-lining this anyhow)
// @brief
// @param
// @retval
static inline void ResetPidScaling(T_PID *pid_layer, const float params[6])
{
  pid_layer->Kp = params[0];
  pid_layer->Ki = params[1];
  pid_layer->Kd = params[2];
}

// dT is the time since this function was last called (in seconds)
// @brief
// @param
// @retval
static void ServoUpdate(float dT)
{
    char control_mode_prev[4] = {0,0,0,0};
    char xbus_new_values = xbus.NewValues(dT, phfc->throttle_armed, phfc->fixedThrottleMode);

#if 0
    if ((phfc->print_counter %1000) == 0) {
        debug_print("Xbus");
        for (int i=0; i < 16; i++) {
            debug_print("[%d]=%f ", i, xbus.valuesf[i]);
            if (i == 7) {
                debug_print("\r\n");
            }
        }
        debug_print("\r\n");

        // full_auto, auto_throttle, ctrl_source, fixedThrottleMode, throttle_value, collective_value
        debug_print("FA[%d] AT[%d] CS[%d] FTM[%d] TV[%f] CV[%f] XBRX[%d]\r\n",
                        phfc->full_auto, phfc->auto_throttle, phfc->ctrl_source,
                        phfc->fixedThrottleMode, phfc->throttle_value, phfc->collective_value, xbus.RcLinkOnline());

        debug_print("Mixer in [P,R,Y,C,T]: ");
        for (int i=0; i < 5; i++) {
            debug_print("[%f] ", phfc->mixer_in[i]);
        }

        debug_print("\r\nDCP gain: [%f]", phfc->rw_cfg.dcp_gain);
        debug_print("\r\nELEV gain: [%f]", phfc->rw_cfg.elevator_gain);

        debug_print("\r\n\r\n");
    }
#endif

    bool retire_waypoint = false;
    float AngleCompensation = COSfD(min(45, ABS(phfc->IMUorient[PITCH]*R2D))) * COSfD(min(45, ABS(phfc->IMUorient[ROLL]*R2D)))
                                      * sqrt(1 / (COSfD(min(45, ABS(phfc->IMUorient[PITCH]*R2D))) + COSfD(min(45, ABS(phfc->IMUorient[ROLL]*R2D)))
                                      - COSfD(min(45, ABS(phfc->IMUorient[PITCH]*R2D))) * COSfD(min(45, ABS(phfc->IMUorient[ROLL]*R2D)))));

//    float throttle_prev = phfc->ctrl_out[RAW][THRO];

    // Always default to using lidar node 0.
    // This is either a single or the front lidar for tandems.
    if (phfc->lidar_online_mask & 0x1) {
      phfc->altitude_lidar = (phfc->altitude_lidar_raw[SINGLE_FRONT_LIDAR_INDEX] - pConfig->lidar_offset/1000.0f) * AngleCompensation;
    }
    else if (phfc->lidar_online_mask & 0x2) {
      // Rear Lidar (for Tandems)
      phfc->altitude_lidar = (phfc->altitude_lidar_raw[REAR_TANDEM_LIDAR_INDEX] - pConfig->lidar_offset/1000.0f) * AngleCompensation;
    }
    else {
      // since we have lost all available lidar data, replace with imu fusion altitude
      // This at least will give a fighting chance ;-)
      // TODO::SP: Validate this.....
      phfc->altitude_lidar = phfc->gps_to_home[2]; //phfc->altitude;
    }

//    debug_print("%+3d %4d %+4d %d\r\n", (int)(phfc->ctrl_out[RAW][COLL]*1000), (int)(phfc->altitude_lidar*1000), (int)(phfc->lidar_vspeed*1000), lidar_last_time/1000);
//    debug_print("%+3d %4d %+4d %d\r\n", (int)(phfc->ctrl_out[SPEED][COLL]*1000), (int)(phfc->altitude_lidar*1000), (int)(phfc->lidar_vspeed*1000), lidar_last_time/1000);

    phfc->ctrl_out[RAW][THRO]  = phfc->collective_value;

    if (phfc->ctrl_source==CTRL_SOURCE_RCRADIO) {
        phfc->ctrl_out[RAW][PITCH] = xbus.valuesf[XBUS_PITCH];
        phfc->ctrl_out[RAW][ROLL]  = xbus.valuesf[XBUS_ROLL];
        phfc->ctrl_out[RAW][YAW]   = xbus.valuesf[XBUS_YAW];
        phfc->ctrl_out[RAW][COLL]  = phfc->collective_value;
    }
    else if (phfc->ctrl_source==CTRL_SOURCE_JOYSTICK) {
        phfc->ctrl_out[RAW][PITCH] = phfc->joy_values[PITCH];
        phfc->ctrl_out[RAW][ROLL]  = phfc->joy_values[ROLL];
        phfc->ctrl_out[RAW][YAW]   = phfc->joy_values[YAW];
        phfc->ctrl_out[RAW][COLL]  = phfc->joy_values[COLL];

        if (phfc->joystick_new_values) {
            xbus_new_values = XBUS_NEW_VALUES;
            phfc->joystick_new_values = 0;
        }
        else {
            xbus_new_values = XBUS_NO_NEW_VALUES;
        }
    }

    // RVW throttle stick to collective logic section
    if(!phfc->throttle_armed) {
        phfc->fixedThrottleMode = THROTTLE_IDLE;      //  set to follow lever
        phfc->throttle_value = -pConfig->Stick100range; // Throttle off

        if (pConfig->throttle_ctrl==PROP_VARIABLE_PITCH) {
          phfc->collective_value = xbus.valuesf[XBUS_THRO];
        }
    }
    else if (phfc->rc_ctrl_request && phfc->throttle_armed)
    {
        // This is necessary for the scenario in which takeoff is
        // Initiated on the AGS when the control source is RC Radio
        // In that case, the RC Radio can be used to spool up, even
        // though the control source is AUTOPILOT
        if (THROTTLE_LEVER_DOWN()) {
          phfc->throttle_value = -pConfig->Stick100range;
        }
        else if (THROTTLE_LEVER_UP()) {
          phfc->throttle_value = pConfig->Stick100range;
        }

        if (pConfig->throttle_ctrl==PROP_FIXED_PITCH && pConfig->SbusEnable == 1)
        {
            // throttle follows xbus stick position
            if(phfc->fixedThrottleMode == THROTTLE_FLY)
            {
                phfc->collective_value = xbus.valuesf[XBUS_THRO];
                    // wait for time out and disarm
                if( phfc->collective_value < -0.5 && phfc->altitude_lidar < 0.2) // && ( (phfc->IMUspeedGroundENU[2]> -0.001f) || (phfc->IMUspeedGroundENU[2]< 0.001f) ))     // three second timeout 3000 counts
                {
                    if(phfc->fixedThrottleMult == 3000)
                    {
                    phfc->fixedThrottleMode = THROTTLE_IDLE;
                    telem.Disarm();
                    }
                    phfc->fixedThrottleMult += 1;
                }
                else
                    phfc->fixedThrottleMult = 0;
            }
            // ramp to stick position in 3 seconds
            if(phfc->fixedThrottleMode == THROTTLE_RAMP)
            {
                if(phfc->fixedThrottleMult >= 1)
                {
                    phfc->fixedThrottleMode = THROTTLE_FLY;
                }
                phfc->fixedThrottleMult += 0.0003f;
                phfc->collective_value = phfc->fixedThrottleCap - ((1 - phfc->fixedThrottleMult) * 0.571f * (-1.0f * pConfig->throttle_multi_min ));  // rvw
                if(phfc->fixedThrottleMult < 0.8)
                {
                ResetIterms();
                phfc->ctrl_out[RAW][PITCH] = 0;
                phfc->ctrl_out[RAW][ROLL]  = 0;
                phfc->ctrl_out[RAW][YAW]   = 0;
                }
            }
            // wait for stick movement to go to next state time out if nothing
            if(phfc->fixedThrottleMode == THROTTLE_DEAD)
            {
                if ((xbus.valuesf[XBUS_THRO] - phfc->fixedThrottleCap)  > AUTO_PROF_TERMINATE_THRS)
                {
                    phfc->fixedThrottleMult = 0;
                    phfc->collective_value = -0.571;
                    phfc->fixedThrottleMode = THROTTLE_RAMP;
                }
                ResetIterms();
            }
            // check if lever is raised to top to start machine
            if(phfc->fixedThrottleMode == THROTTLE_IDLE && phfc->throttle_value > 0.5f)
            {
                phfc->fixedThrottleCap =  xbus.valuesf[XBUS_THRO];    // capture midstick value
                phfc->collective_value = -0.571;
                phfc->fixedThrottleMode = THROTTLE_DEAD;              // next state
            }
            else if(phfc->fixedThrottleMode == THROTTLE_IDLE)
            {
                phfc->collective_value = -0.571;
            }
        }
        else
        {
            phfc->collective_value = xbus.valuesf[XBUS_THRO];  // RVW not fixed pitch so no self center throttle stick

        }

        //for variable prop UAV
        if ( (pConfig->throttle_ctrl==PROP_VARIABLE_PITCH) && THROTTLE_LEVER_UP()) {
          phfc->fixedThrottleMode = THROTTLE_FLY;
        }
    }
    else
    {
        phfc->collective_value = 0;
    }

    control_mode_prev[PITCH] = phfc->control_mode[PITCH];
    control_mode_prev[ROLL]  = phfc->control_mode[ROLL];
    control_mode_prev[YAW]   = phfc->control_mode[YAW];
    control_mode_prev[COLL]  = phfc->control_mode[COLL];
    
    telem.ProcessCommands();

    Playlist_ProcessTop();
    
    if (xbus_new_values) {
        if (xbus_new_values == XBUS_NEW_VALUES_1ST) {
            telem.SaveValuesForAbort();
        }

        SetRCRadioControl();
    }

    ProcessStickInputs(phfc, dT);

    if (pConfig->throttle_ctrl == PROP_VARIABLE_PITCH) {
        phfc->ctrl_out[RAW][THRO] = (phfc->throttle_value+pConfig->Stick100range)
                                        * pConfig->throttle_values[1] + pConfig->throttle_values[0];
    }

    // set heading to the IMU's heading for throttle stick below -0.55, kind of like Landed mode detection
    if (phfc->throttle_value < -0.55f) {
        phfc->ctrl_out[ANGLE][YAW] = phfc->IMUorient[YAW]*R2D;
    }

    if (phfc->ctrl_source == CTRL_SOURCE_RCRADIO || phfc->ctrl_source == CTRL_SOURCE_JOYSTICK)
    {
        if (phfc->ctrl_source == CTRL_SOURCE_JOYSTICK)
        {
          if (phfc->joy_PRmode)
          {
              SetSpeedAcc(&phfc->ctrl_out[SPEED][PITCH], -phfc->ctrl_out[RAW][PITCH]*phfc->Stick_Hspeed, phfc->rw_cfg.StickHaccel, dT);
              SetSpeedAcc(&phfc->ctrl_out[SPEED][ROLL],   phfc->ctrl_out[RAW][ROLL]*phfc->Stick_Hspeed,  phfc->rw_cfg.StickHaccel, dT);
          }
          else
          {
            phfc->ctrl_out[SPEED][PITCH] += phfc->joy_values[THRO]*phfc->rw_cfg.StickHaccel*dT;

            if (phfc->ctrl_out[SPEED][PITCH] > phfc->rw_cfg.joystick_max_speed)
              phfc->ctrl_out[SPEED][PITCH] = phfc->rw_cfg.joystick_max_speed;

            if (phfc->joy_values[THRO]<0 && phfc->ctrl_out[SPEED][PITCH]<0)
              phfc->ctrl_out[SPEED][PITCH] = 0;

            SetSpeedAcc(&phfc->ctrl_out[SPEED][ROLL],   0,  phfc->rw_cfg.StickHaccel, dT);
          }
        }
        else
        {
          SetSpeedAcc(&phfc->ctrl_out[SPEED][PITCH], -phfc->ctrl_out[RAW][PITCH]*phfc->Stick_Hspeed, phfc->rw_cfg.StickHaccel, dT);
          SetSpeedAcc(&phfc->ctrl_out[SPEED][ROLL],   phfc->ctrl_out[RAW][ROLL]*phfc->Stick_Hspeed,  phfc->rw_cfg.StickHaccel, dT);
        }

        phfc->ctrl_out[RATE][PITCH]  = phfc->ctrl_out[RAW][PITCH]*phfc->PRstick_rate  + phfc->pid_PitchAngle.COofs;
        phfc->ctrl_out[ANGLE][PITCH] = phfc->ctrl_out[RAW][PITCH]*phfc->PRstick_angle + phfc->pid_PitchSpeed.COofs;
        phfc->ctrl_out[RAW][PITCH]  += phfc->pid_PitchRate.COofs;
        phfc->ctrl_out[RATE][ROLL]   = phfc->ctrl_out[RAW][ROLL]*phfc->PRstick_rate   + phfc->pid_RollAngle.COofs;
        phfc->ctrl_out[ANGLE][ROLL]  = phfc->ctrl_out[RAW][ROLL]*phfc->PRstick_angle  + phfc->pid_RollSpeed.COofs;
        phfc->ctrl_out[RAW][ROLL]   += phfc->pid_RollRate.COofs;

        float yaw_rate_ctrl = phfc->ctrl_out[RAW][YAW]*phfc->YawStick_rate;
        phfc->ctrl_out[SPEED][COLL]  = phfc->ctrl_out[RAW][COLL]*phfc->Stick_Vspeed;

        if (phfc->rw_cfg.ManualLidarAltitude && !phfc->eng_super_user) {
            phfc->ctrl_out[POS][COLL] = 2 + 2*phfc->ctrl_out[RAW][COLL];
        }

        yaw_rate_ctrl = ClipMinMax(yaw_rate_ctrl, phfc->pid_YawAngle.COmin, phfc->pid_YawAngle.COmax);

        // NOTE::MRI: What are these used for?
        HeadingUpdate(yaw_rate_ctrl, dT);
        AltitudeUpdate(phfc->ctrl_out[RAW][COLL]*phfc->Stick_Vspeed, dT);
        
        phfc->ctrl_out[RATE][YAW]  = yaw_rate_ctrl;
        phfc->ctrl_out[RAW][YAW]  += phfc->pid_YawRate.COofs;

        // do not add offsets to RAW above, apply gain to RAW, added offset to RAW,
        // clip RAW using rate PID limits
        phfc->ctrl_out[RAW][COLL]  = phfc->ctrl_out[RAW][COLL] * pConfig->control_gains[COLL];
        phfc->ctrl_out[RAW][COLL] += phfc->pid_CollVspeed.COofs;
    }
    else if (phfc->ctrl_source == CTRL_SOURCE_AFSI) {
        phfc->ctrl_out[SPEED][PITCH] = afsi.GetSpeedForward();
        phfc->ctrl_out[SPEED][ROLL]  = afsi.GetSpeedRight();
        phfc->ctrl_out[ANGLE][YAW]   = afsi.GetHeading();
        phfc->ctrl_out[POS][COLL]    = afsi.GetAltitude();
    }

    // processes staged waypoints - takeoff, landing, ... etc
    ProcessFlightMode(phfc,dT);
    
    // horizontal position
    if (phfc->control_mode[PITCH] == CTRL_MODE_POSITION || phfc->control_mode[ROLL] == CTRL_MODE_POSITION) {
        float distance_to_ref;
        float D2T_clipped;
        float course_to_ref;
        float speed;
        float PathSpeedR;

        distance_to_ref = DistanceCourse(phfc->positionLatLon[0], phfc->positionLatLon[1],
                                            phfc->waypoint_pos[0], phfc->waypoint_pos[1], &course_to_ref);
        // debug_print("dist %4.1f \r\n", distance_to_ref);

        /* error handling: do nothing if distance over 100k */
        if (distance_to_ref > 100000) {
            distance_to_ref = 0;
        }

        /* keep track of the minimum distance to the next waypoint */
        if ( distance_to_ref < phfc->distance2WP_min) {
            phfc->distance2WP_min = distance_to_ref;
        }
            
        phfc->gps_to_waypoint[0] = distance_to_ref;
        phfc->gps_to_waypoint[1] = course_to_ref;

        D2T_clipped = phfc->waypoint_type==WAYPOINT_FLYTHROUGH ? 1000 : distance_to_ref;

        // reset on the first time position mode is turned on
        if (control_mode_prev[PITCH] < CTRL_MODE_POSITION) {
          // not sure if this needed any more since RC radio cannot set WP any more
          if (phfc->ctrl_source==CTRL_SOURCE_RCRADIO) {
              telem.SetWaypoint(phfc->positionLatLon[0], phfc->positionLatLon[1], -9999, WAYPOINT_GOTO, 0);
          }

          PID_SetForEnable(&phfc->pid_Dist2T, 0, 0, phfc->gps_speed);
          PID_SetForEnable(&phfc->pid_Dist2P, 0, 0, 0);
          phfc->speedCtrlPrevEN[0] = 0;
          phfc->speedCtrlPrevEN[1] = 0;
        }

        speed = PID_P_Acc(&phfc->pid_Dist2T, D2T_clipped, 0, dT, false, false);

        // do path navigation only once far enough from the target
        // since otherwise trust vectoring will take care of the final approach
        if (phfc->rw_cfg.path_navigation && distance_to_ref > 2) {
            // add a side vector to the main speed vector to the target waypoint.
            // The side vector is proportional to the current distance from the path
            // and it is pulling the aircraft to stay on the path
            float CTc = course_to_ref;
            float D2P, S2P;
            float STc = phfc->waypoint_STcourse;
            float Cx = (float)(phfc->positionLatLon[1] - phfc->waypoint_pos_prev[1]);
            float Cy = (float)(phfc->positionLatLon[0] - phfc->waypoint_pos_prev[0]);
            float deltaCourse = Wrap180(STc - CTc);
            float S2Prot;
            Cx = Cx/DPM*COSfD(((float)phfc->positionLatLon[0]));
            Cy = Cy/DPM;

            D2P = ABS(phfc->path_a*Cx+phfc->path_b*Cy)*phfc->path_dist_denom;
            S2P = PID_P_Acc(&phfc->pid_Dist2P, D2P, 0, dT, false, false); // speed to path
            
            // always rotate the speed vector towards the path
            S2Prot = (deltaCourse >= 0) ? -90 : 90;
                
            //Rotate(0, S2P, phfc->IMUorient[YAW] - (STc+S2Prot)*D2R, &PathSpeedR, &PathSpeedP);
            PathSpeedR = -S2P*SINfR(phfc->IMUorient[YAW] - (STc+S2Prot)*D2R);
      }
      else {
          phfc->pid_Dist2P.COlast = 0;
          PathSpeedR = 0;
      }

#ifdef THRUST_VECTORING
      {
          /* split speed into E/N components */
          float speedE = speed * SINfD(course_to_ref);
          float speedN = speed * COSfD(course_to_ref);

          /* apply acceleration limit to speed changes */
          float dE = speedE - phfc->speedCtrlPrevEN[0];
          float dN = speedN - phfc->speedCtrlPrevEN[1];
          float dS = sqrtf(dE*dE + dN*dN);

          if (dS)
          {
              float dSlimit = Min(dS, phfc->acc_dyn_turns*dT);
              dE = dE * dSlimit/dS;
              dN = dN * dSlimit/dS;
              phfc->speedCtrlPrevEN[0] += dE;
              phfc->speedCtrlPrevEN[1] += dN;
              speedE = phfc->speedCtrlPrevEN[0];
              speedN = phfc->speedCtrlPrevEN[1];
          }

          /* rotate speed E/N to Right/Forward */
          Rotate(speedE, speedN, phfc->IMUorient[YAW], &phfc->ctrl_out[SPEED][ROLL], &phfc->ctrl_out[SPEED][PITCH]);
      }
#endif

      /* for high speeds, make the nose to point towards the target,
      * or to follow the ground speed vector. For low speeds, do not change it */
      if (distance_to_ref > 5) {
          phfc->ctrl_out[ANGLE][YAW] = phfc->rw_cfg.nose_to_WP ? course_to_ref : phfc->waypoint_STcourse;
      }

#ifndef THRUST_VECTORING
      if (/*speed>pConfig->low_speed_limit &&*/ distance_to_ref>5 || phfc->waypoint_type==WAYPOINT_FLYTHROUGH)
      {
          phfc->ctrl_out[SPEED][PITCH] = speed;
          phfc->ctrl_out[SPEED][ROLL]  = 0;
      }
      else
      {
          /* split the speed vector pointing to the target to pitch/roll speed components
          ** considering the current orientation of the heli */
          float angle = course_to_ref - phfc->IMUorient[YAW]*R2D;
          phfc->ctrl_out[SPEED][PITCH] = speed * COSfD(angle);
          phfc->ctrl_out[SPEED][ROLL]  = speed * SINfD(angle);
      }
#endif

//      phfc->ctrl_out[SPEED][PITCH] += PathSpeedP;
      phfc->ctrl_out[SPEED][ROLL]  += PathSpeedR; // side component only
      
      /* altitude control - interpolation between waypoints */
      if (    ( (phfc->ctrl_source == CTRL_SOURCE_AUTOPILOT) || (phfc->ctrl_source == CTRL_SOURCE_AFSI) )
           && (phfc->waypoint_type != WAYPOINT_TAKEOFF) && !phfc->touch_and_go_takeoff)
      {
        if (phfc->waypoint_STdist>2)
        {
            float a = ClipMinMax((distance_to_ref-phfc->waypoint_STofs) / phfc->waypoint_STdist, 0, 1);
            float altitude = phfc->waypoint_pos_prev[2] * a + phfc->waypoint_pos[2] * (1-a);
            phfc->ctrl_out[POS][COLL] = altitude;
//            if (!(phfc->print_counter&0x3f))
//                debug_print("D2T %4.1f SDdist %4.1f old %5.1f new %5.1f a %4.2f curr %5.1f\r\n", distance_to_ref, phfc->waypoint_STdist, phfc->waypoint_pos_prev[2], phfc->waypoint_pos[2], a, altitude);
        }
        else
            phfc->ctrl_out[POS][COLL] = phfc->waypoint_pos[2];
      }
      
      /* check for waypoint retire conditions */
      if (phfc->waypoint_retire)
      {
          if (phfc->waypoint_type==WAYPOINT_FLYTHROUGH)
          {
              float limit = telem.CalcFTWPlimit(true);
              /* once it gets close enough considering the current speed */
              if (phfc->gps_to_waypoint[0] < max(1, limit))
                retire_waypoint = true;
          }
          else
          {
              /* once it gets close enough at low enough speed, also wait for altitude to match the target !!!!!!!!! */
              //GpsData gps_data = gps.GetGpsData();
              if (gps.gps_data_.HspeedC <= phfc->rw_cfg.GTWP_retire_speed && distance_to_ref <= phfc->rw_cfg.GTWP_retire_radius)
                retire_waypoint = true;
          }
      }
    }

    /* dynamic yaw rate - limits yaw rate to prevent airframe overloading during turns */
    telem.CalcDynYawRate();
    phfc->pid_YawAngle.COmax =  phfc->dyn_yaw_rate;
    phfc->pid_YawAngle.COmin = -phfc->dyn_yaw_rate;

    /* rotate ground speed vector to plane speed vector */
    Rotate(phfc->IMUspeedGroundENU[0], phfc->IMUspeedGroundENU[1],  phfc->IMUorient[YAW], &phfc->speedHeliRFU[0], &phfc->speedHeliRFU[1]);
    phfc->speedHeliRFU[2] = phfc->IMUspeedGroundENU[2];
    
    phfc->bankPitch = 0;
    phfc->bankRoll = 0;   // clear here, it might get set by auto-banking code to compensate Acc for IMU atitude estimation

    /* speed heli - SpeedGroundEN - rotate to SpeedHeliRF, PID(CtrlSpeedRF, SpeedHeliRF)->Angle(R)(-P) */
    if (phfc->control_mode[PITCH]>=CTRL_MODE_SPEED || phfc->control_mode[ROLL]>=CTRL_MODE_SPEED)
    {
      if (phfc->setZeroSpeed) {
        if (phfc->pid_Dist2T.acceleration <= 0 ) {
          phfc->pid_Dist2T.acceleration = 0.6; // m/s^2
        }

        if (telem.Accelerate(-phfc->pid_Dist2T.acceleration,dT) == -1) {
          phfc->setZeroSpeed = false;
        }
      }
//      if (!(phfc->print_counter&0x1f))
//        debug_print("%4.1f %4.1f ", phfc->speed_Iterm_E, phfc->speed_Iterm_N);
      /* rotate E/N speed PID I-terms into current R/F */
      if (phfc->rw_cfg.wind_compensation)
      {
          Rotate(phfc->speed_Iterm_E, phfc->speed_Iterm_N, phfc->IMUorient[YAW], &phfc->pid_RollSpeed.Ie, &phfc->pid_PitchSpeed.Ie);
//          if (!(phfc->print_counter&0x1f))
//             debug_print("1 E %f N %f R %f P %f\n", phfc->speed_Iterm_E, phfc->speed_Iterm_N, phfc->pid_RollSpeed.Ie, phfc->pid_PitchSpeed.Ie);
      }

//      if (!(phfc->print_counter&0x1f))
//        debug_print("%4.1f %4.1f   ", phfc->pid_RollSpeed.Ie, phfc->pid_PitchSpeed.Ie);
      
      /* if previous mode was below SPEED, reset PIDs to be bumpless */
      if ((control_mode_prev[PITCH]<CTRL_MODE_SPEED && !pConfig->ctrl_mode_inhibit[PITCH]) || (control_mode_prev[ROLL]<CTRL_MODE_SPEED && !pConfig->ctrl_mode_inhibit[ROLL]))
      {
          phfc->ctrl_out[SPEED][PITCH] = phfc->speedHeliRFU[1];
          phfc->ctrl_out[SPEED][ROLL]  = phfc->speedHeliRFU[0];
          PID_SetForEnable(&phfc->pid_PitchSpeed,   phfc->ctrl_out[SPEED][PITCH], phfc->speedHeliRFU[1], -phfc->ctrl_out[ANGLE][PITCH]);
          PID_SetForEnable(&phfc->pid_PitchCruise,  phfc->ctrl_out[SPEED][PITCH], phfc->speedHeliRFU[1], -phfc->ctrl_out[ANGLE][PITCH]);
          PID_SetForEnable(&phfc->pid_RollSpeed,    phfc->ctrl_out[SPEED][ROLL],  phfc->speedHeliRFU[0],  phfc->ctrl_out[ANGLE][ROLL]);
      }

      if (!phfc->cruise_mode)
      {
          if (ABS(phfc->ctrl_out[SPEED][PITCH]) >= phfc->rw_cfg.cruise_speed_limit)
          {
              phfc->cruise_mode = true;
              /* smoothly engage cruise mode by keeping the current angle */
              phfc->pid_PitchCruise.COlast = phfc->pid_PitchSpeed.COlast;
          }
      }
      else
      {
          if (ABS(phfc->ctrl_out[SPEED][PITCH]) < 0.8f*phfc->rw_cfg.cruise_speed_limit)
          {
              phfc->cruise_mode = false;
              /* smoothly engage normal speed mode */
              PID_SetForEnable(&phfc->pid_PitchSpeed,   phfc->ctrl_out[SPEED][PITCH], phfc->speedHeliRFU[1], phfc->pid_PitchCruise.COlast);
          }
      }
      if (phfc->cruise_mode)
      {
          /* set trip to an angle, which corresponds to the target speed */
//          float angle;
          float cruise_turn_pitch_trim;
          float cruise_alt_pitch_trim;
          static float entry_gnd_speed = phfc->speedHeliRFU[FORWARD];
          static float entry_speed_request = phfc->ctrl_out[SPEED][PITCH];
          static float headwind_speed = phfc->speedHeliRFU[FORWARD] - phfc->ctrl_out[SPEED][PITCH];


          float angle = phfc->rw_cfg.Speed2AngleLUT[min((int)(ABS(phfc->ctrl_out[SPEED][PITCH])*2+0.5f), SPEED2ANGLE_SIZE-1)];
          if (phfc->ctrl_out[SPEED][PITCH]<0) {
              angle = -angle;
          }

          if ( ABS(phfc->ctrl_yaw_rate) >= TURN_YAW_RATE_THRESHOLD ) {
            //Add pitch to slow the UAV when it is turning sharply in cruise mode.
            //Added pitch is proportional to the requested YAW rate (turn rate) and inversely
            //proportional to dyn_yaw_rate which is actually a limit on the YAW RATE based on speed
            cruise_turn_pitch_trim = phfc->rw_cfg.max_cruise_pitch_trim*ABS(phfc->ctrl_yaw_rate/phfc->dyn_yaw_rate);

            // "angle" pulled from Speed2AngleLUT is positive for positive speed, even though in reality
            // a NEGATIVE PITCH ANGLE would yield a positive speed. For this reason, cruise_turn_pitch_trim
            // is subtracted from "angle" to a minimum of 0 degrees so that UAV does not go nose up
            angle = Max(0.0f,angle - cruise_turn_pitch_trim);
          }

          if (ABS(phfc->ctrl_out[POS][COLL]-phfc->altitude) > ALT_CTRL_THRESHOLD) {
            float current_air_speed = phfc->speedHeliRFU[FORWARD] + headwind_speed;
            float entry_air_speed = entry_gnd_speed + headwind_speed;

            // The larger the request to ascend, the more we trim off the cruise angle
            // The larger the request to descend, the more we add to the cruise angle, since
            // ctrl_out[SPEED][COLL] holds its sign.
            cruise_alt_pitch_trim = phfc->rw_cfg.max_cruise_pitch_trim
                                   *(phfc->ctrl_out[SPEED][COLL]/hfc.pid_CollAlt.COmax)
                                   *current_air_speed/entry_air_speed;

            // Limit the cruise angle to a nose down pitch angle of "angle+phfc->min_added_cruise_anlge"
            // and a nose up pitch angle of "phfc->max_cruise_angle"
            angle = min( angle+phfc->min_added_cruise_anlge, max(-phfc->max_cruise_angle, angle - cruise_alt_pitch_trim));
          }
          else {
            entry_gnd_speed = phfc->speedHeliRFU[FORWARD];
            entry_speed_request = phfc->ctrl_out[SPEED][PITCH];

            // Calculate the wind speed on the front of the UAV immediately before
            // doing an altitude change
            headwind_speed = entry_speed_request - entry_gnd_speed;
          }

          phfc->pid_PitchCruise.COofs = angle;

          phfc->ctrl_out[ANGLE][PITCH] = -PID_P_Acc(&phfc->pid_PitchCruise, phfc->ctrl_out[SPEED][PITCH], phfc->speedHeliRFU[1], dT, false, false); // speed forward

          hfc.Debug[0] = hfc.ctrl_out[SPEED][COLL];
          hfc.Debug[1] = hfc.ctrl_out[SPEED][COLL]/hfc.pid_CollAlt.COmax;
          hfc.Debug[2] = cruise_alt_pitch_trim;
          hfc.Debug[3] = angle;
          hfc.Debug[4] = hfc.ctrl_out[ANGLE][PITCH];;

//          if (!(phfc->print_counter&0x3f))
//              debug_print("S %f A %f out %f\n", phfc->ctrl_out[SPEED][PITCH], angle, phfc->ctrl_out[ANGLE][PITCH]);
      }
      else
          phfc->ctrl_out[ANGLE][PITCH] = -PID(&phfc->pid_PitchSpeed, phfc->ctrl_out[SPEED][PITCH], phfc->speedHeliRFU[1], dT); // speed forward

      phfc->ctrl_out[ANGLE][ROLL]  =  PID(&phfc->pid_RollSpeed,  phfc->ctrl_out[SPEED][ROLL],  phfc->speedHeliRFU[0], dT); // speed right
//      if (!(phfc->print_counter&0x1f))
//          debug_print("cS %5.3f mS %5.3f a %5.2f i %f\n", phfc->ctrl_out[SPEED][ROLL], phfc->speedHeliRFU[0], phfc->ctrl_out[ANGLE][ROLL], phfc->pid_RollSpeed.Ie);

      /* rotate back R/F I-terms to E/N */
      if (phfc->rw_cfg.wind_compensation)
      {
          Rotate(phfc->pid_RollSpeed.Ie, phfc->pid_PitchSpeed.Ie, -phfc->IMUorient[YAW], &phfc->speed_Iterm_E, &phfc->speed_Iterm_N);
//          if (!(phfc->print_counter&0x1f))
//             debug_print("2 E %f N %f R %f P %f\n", phfc->speed_Iterm_E, phfc->speed_Iterm_N, phfc->pid_RollSpeed.Ie, phfc->pid_PitchSpeed.Ie);
          phfc->speed_Iterm_E_lp = (phfc->speed_Iterm_E + phfc->speed_Iterm_E_lp*4095)/4096;
          phfc->speed_Iterm_N_lp = (phfc->speed_Iterm_N + phfc->speed_Iterm_N_lp*4095)/4096;
//          if (!(phfc->print_counter&0x3f))
//              debug_print("%f %f %f %f\n", phfc->speed_Iterm_E_lp, phfc->speed_Iterm_E, phfc->speed_Iterm_N_lp, phfc->speed_Iterm_N);
      }
      
      /* pitch-roll mixing to prevent side slip */
      // a=2*PI*YR*speed/360     side acceleration during a turn at speed v and yaw rate YR
      // angle = -atan(a/9.81)
#ifdef THRUST_VECTORING
      if (phfc->control_mode[PITCH]!=CTRL_MODE_POSITION && phfc->control_mode[ROLL]!=CTRL_MODE_POSITION)
#endif
      {
          /* use yaw rate PID input instead of yaw_rate_ctrl */
          float speedP = phfc->ctrl_out[SPEED][PITCH];
          float speedR = phfc->ctrl_out[SPEED][ROLL];
          float ctrl_speed = sqrtf(speedP*speedP + speedR*speedR);
          if (ctrl_speed>0)
          {
              float a = 2*PI*phfc->ctrl_yaw_rate*ctrl_speed/360;
              float ai = CLIP(a, 9.81f);  // 1G side limit
              float bank = ATAN2fD(ai, 9.81f);       // float roll = R2D*atanf(a/9.81f);
          
              /* rescale the speed vector to have "bank" magnitude and rotate CW by 90deg */
              speedP = speedP * bank / ctrl_speed;
              speedR = speedR * bank / ctrl_speed;
              phfc->bankRoll =  speedP;
              phfc->bankPitch = -speedR;

//              phfc->Debug[0] = phfc->ctrl_yaw_rate;
//              phfc->Debug[1] = phfc->bankPitch;
//              phfc->Debug[2] = phfc->bankRoll;
//              phfc->Debug[3] = phfc->dyn_yaw_rate;

              phfc->ctrl_out[ANGLE][PITCH] -= phfc->bankPitch;
              phfc->ctrl_out[ANGLE][ROLL]  += phfc->bankRoll;
              phfc->ctrl_out[ANGLE][PITCH] = ClipMinMax(phfc->ctrl_out[ANGLE][PITCH], phfc->pid_PitchSpeed.COmin, phfc->pid_PitchSpeed.COmax);
              phfc->ctrl_out[ANGLE][ROLL]  = ClipMinMax(phfc->ctrl_out[ANGLE][ROLL],  phfc->pid_RollSpeed.COmin,  phfc->pid_RollSpeed.COmax);
          }
      }
    }

    if (phfc->control_mode[PITCH] >= CTRL_MODE_ANGLE) {
        if (phfc->control_mode[PITCH] == CTRL_MODE_ANGLE && phfc->ctrl_source == CTRL_SOURCE_AUTOPILOT) {
            SetSpeedAcc(&phfc->ctrl_out[ANGLE][PITCH], phfc->ctrl_angle_pitch_3d, pConfig->takeoff_angle_rate, dT);
        }

        // if previous mode was below ANGLE, reset PIDs to be bumpless
        if (control_mode_prev[PITCH] < CTRL_MODE_ANGLE) {
            PID_SetForEnable(&phfc->pid_PitchAngle, phfc->ctrl_out[ANGLE][PITCH], phfc->IMUorient[PITCH]*R2D, phfc->ctrl_out[RATE][PITCH]);
        }

        phfc->ctrl_out[RATE][PITCH] = PID(&phfc->pid_PitchAngle, phfc->ctrl_out[ANGLE][PITCH], phfc->IMUorient[PITCH]*R2D, dT);
    }

    if (phfc->control_mode[ROLL] >= CTRL_MODE_ANGLE) {
        if (phfc->control_mode[ROLL] == CTRL_MODE_ANGLE && phfc->ctrl_source == CTRL_SOURCE_AUTOPILOT) {
            SetSpeedAcc(&phfc->ctrl_out[ANGLE][ROLL], phfc->ctrl_angle_roll_3d, pConfig->takeoff_angle_rate, dT);
        }

        // if previous mode was below ANGLE, reset PIDs to be bumpless
        if (control_mode_prev[ROLL]<CTRL_MODE_ANGLE) {
            PID_SetForEnable(&phfc->pid_RollAngle, phfc->ctrl_out[ANGLE][ROLL], phfc->IMUorient[ROLL]*R2D, phfc->ctrl_out[RATE][ROLL]);
        }

        phfc->ctrl_out[RATE][ROLL] = PID(&phfc->pid_RollAngle, phfc->ctrl_out[ANGLE][ROLL], phfc->IMUorient[ROLL]*R2D,  dT);
    }

//    debug_print("%d %5.1f %5.1f\r\n", pr_control_mode, roll_angle, roll_rate);
    if (phfc->control_mode[PITCH] >= CTRL_MODE_RATE) {
        // if previous mode was below RATE, reset PIDs to be bumpless
        if (control_mode_prev[PITCH]<CTRL_MODE_RATE) {
            PID_SetForEnable(&phfc->pid_PitchRate, phfc->ctrl_out[RATE][PITCH], phfc->gyro[PITCH], phfc->ctrl_out[RAW][PITCH]);
        }

        if (pConfig->enable_dynamic_speed_pid && phfc->cruise_mode) {

          if (ABS(phfc->ctrl_out[SPEED][PITCH]) >= pConfig->dynamic_pid_speed_threshold) {
            // Adjust pid_pitchRate by dynamic scaling factor
            // When traveling at higher speeds, it may be necessary to make on-the-fly pid changes.
            ApplyPidScaling(&phfc->pid_PitchRate, pConfig->pitchrate_pid_params, phfc->pid_PitchRateScalingFactor);
          }
          else {
            if (ABS(phfc->ctrl_out[SPEED][PITCH]) < 0.8f*pConfig->dynamic_pid_speed_threshold) {
              // revert back to configured Pitch Rate PID values
              // On dropping out of higher speeds, revert pid rate values back to configured values.
              ResetPidScaling(&phfc->pid_PitchRate, pConfig->pitchrate_pid_params);
            }
          }
        }

        phfc->ctrl_out[RAW][PITCH] = PID(&phfc->pid_PitchRate, phfc->ctrl_out[RATE][PITCH], phfc->gyro[PITCH], dT);
    }
    else {
        phfc->ctrl_out[RAW][PITCH] = ClipMinMax(phfc->ctrl_out[RAW][PITCH], phfc->pid_PitchRate.COmin, phfc->pid_PitchRate.COmax);
    }
      
    if (phfc->control_mode[ROLL]>=CTRL_MODE_RATE) {
        //if previous mode was below RATE, reset PIDs to be bumpless
        if (control_mode_prev[ROLL]<CTRL_MODE_RATE) {
            PID_SetForEnable(&phfc->pid_RollRate,  phfc->ctrl_out[RATE][ROLL],   phfc->gyro[ROLL],  phfc->ctrl_out[RAW][ROLL]);
        }

        phfc->ctrl_out[RAW][ROLL] = PID(&phfc->pid_RollRate,  phfc->ctrl_out[RATE][ROLL],   phfc->gyro[ROLL],  dT);
    }
    else {
        phfc->ctrl_out[RAW][ROLL] = ClipMinMax(phfc->ctrl_out[RAW][ROLL], phfc->pid_RollRate.COmin, phfc->pid_RollRate.COmax);
    }

    if (phfc->control_mode[YAW]>=CTRL_MODE_ANGLE)
    {
      bool double_angle_acc = phfc->ctrl_source==CTRL_SOURCE_JOYSTICK || phfc->ctrl_source==CTRL_SOURCE_RCRADIO ? true : false;
      if (control_mode_prev[YAW]<CTRL_MODE_ANGLE)
      {
        phfc->ctrl_out[ANGLE][YAW] = phfc->IMUorient[YAW]*R2D; 
        PID_SetForEnable(&phfc->pid_YawAngle, phfc->ctrl_out[ANGLE][YAW], phfc->ctrl_out[ANGLE][YAW], phfc->ctrl_out[RATE][YAW]);
      }  
      phfc->ctrl_out[RATE][YAW] = PID_P_Acc(&phfc->pid_YawAngle, phfc->ctrl_out[ANGLE][YAW], phfc->IMUorient[YAW]*R2D, dT, false, double_angle_acc);
    }

    if (phfc->control_mode[YAW] >= CTRL_MODE_RATE) {
        if (control_mode_prev[YAW] < CTRL_MODE_RATE) {
            PID_SetForEnable(&phfc->pid_YawRate, phfc->ctrl_out[RATE][YAW], phfc->gyro[YAW], phfc->ctrl_out[RAW][YAW]);
        }
        phfc->ctrl_out[RAW][YAW] = PID(&phfc->pid_YawRate, phfc->ctrl_out[RATE][YAW], phfc->gyro[YAW], dT);
    }
    else {
        phfc->ctrl_out[RAW][YAW] = ClipMinMax(phfc->ctrl_out[RAW][YAW], phfc->pid_YawRate.COmin, phfc->pid_YawRate.COmax);
    }

    phfc->ctrl_yaw_rate = phfc->ctrl_out[RATE][YAW];  // store yaw rate for auto banking
    
    /* collective */
    if (phfc->control_mode[COLL]>=CTRL_MODE_POSITION)
    {
        float CurrAltitude, CtrlAltitude, vspeedmin;
        float LidarMinAlt = 0;
        float e_alt;
        bool double_acc = (phfc->ctrl_source==CTRL_SOURCE_RCRADIO || phfc->ctrl_source==CTRL_SOURCE_JOYSTICK) ? true : false;

        // TODO::SP: phfc->enable_lidar_ctrl_mode, will ultimately come from a configuration
        // setting enabling the use of this feature. For now, feature is permanently disabled.
        if (phfc->enable_lidar_ctrl_mode && !phfc->eng_super_user) {
          /* set minimum above ground altitude as a function of speed */
          LidarMinAlt = CalcMinAboveGroundAlt(phfc->gps_speed);
        
          /* switch between regular (IMU) and lidar based altitude control mode */
          if (!phfc->LidarCtrlMode)
          {
            /* if lidar alt dips below the min limit, switch to lidat ctrl mode.
             * Never do this for takeoff since it needs to get above LidarMinAlt first */
            if ((phfc->altitude_lidar < LidarMinAlt) && (phfc->waypoint_type != WAYPOINT_TAKEOFF) && !phfc->touch_and_go_takeoff) {
                phfc->LidarCtrlMode = true;
            }
          }
          else
          {
            /* if IMU altitude dips below the set altitude, switch back to regular altitude ctrl mode */
            /* or once lidar altitude increases sufficiently above the min lidar altitude */
            if ((phfc->altitude < phfc->ctrl_out[POS][COLL]) || (phfc->altitude_lidar > 1.2f*LidarMinAlt)) {
                phfc->LidarCtrlMode = false;            
            }
          }

          /* never use lidar ctrl mode in manual lidar ctrl mode */
          if (phfc->rw_cfg.ManualLidarAltitude) {
            phfc->LidarCtrlMode = false;
          }
        }

        /* select regular or lidar based altitude values */
        CurrAltitude = (phfc->rw_cfg.ManualLidarAltitude || phfc->LidarCtrlMode) ? phfc->altitude_lidar : phfc->altitude;
        CtrlAltitude = phfc->LidarCtrlMode ? LidarMinAlt : phfc->ctrl_out[POS][COLL];

        /* increase vertical down speed limit with an increased horizontal speed */
        vspeedmin = max(pConfig->VspeedDownCurve[1], phfc->rw_cfg.VspeedMin+pConfig->VspeedDownCurve[0]*phfc->gps_speed);
        phfc->pid_CollAlt.COmin = vspeedmin;

//        if (!(phfc->print_counter&0x3f))
//            debug_print("Mode %s currA %4.1f  ctrlA %4.1f alt %4.1f ctrlalt %4.1f\r\n", phfc->LidarCtrlMode ? "Lidar" : "baro ", CurrAltitude, CtrlAltitude, phfc->altitude, phfc->ctrl_out[POS][COLL]);

        if (control_mode_prev[COLL]<CTRL_MODE_POSITION)
        {
            phfc->LidarCtrlMode = false;
            if ((phfc->waypoint_type != WAYPOINT_TAKEOFF) && !phfc->touch_and_go_takeoff) {
                phfc->ctrl_out[POS][COLL] = phfc->altitude;
                PID_SetForEnable(&phfc->pid_CollAlt, CtrlAltitude, CurrAltitude, phfc->ctrl_out[SPEED][COLL]);
            }
        }
        e_alt = CtrlAltitude - CurrAltitude;
        phfc->ctrl_out[SPEED][COLL] = PID_P_Acc(&phfc->pid_CollAlt, CtrlAltitude, CurrAltitude, dT, phfc->LidarCtrlMode && (e_alt>=0), double_acc);  // in lidar mode, ignore acc up

        if ( ABS(CtrlAltitude - CurrAltitude) <= 1) {
          // change the vertical speed of UAV from takeoff mode to normal mode
          phfc->pid_CollAlt.COmax = phfc->rw_cfg.VspeedMax;
        }
    }
    else {
        phfc->ctrl_out[POS][COLL] = phfc->altitude;
    }
        
    if (phfc->control_mode[COLL] >= CTRL_MODE_SPEED) {
        if (phfc->control_mode[COLL] == CTRL_MODE_SPEED && phfc->ctrl_source == CTRL_SOURCE_AUTOPILOT) {
            SetSpeedAcc(&phfc->ctrl_out[SPEED][COLL], phfc->ctrl_vspeed_3d, pConfig->landing_vspeed_acc, dT);
        }

        if (control_mode_prev[COLL]<CTRL_MODE_SPEED) {
            //debug_print("vspeed = %f   GPS = %f  manual = %f\r\n", phfc->ctrl_out[SPEED][COLL], phfc->IMUspeedGroundENU[2], phfc->ctrl_out[RAW][COLL]);
            PID_SetForEnable(&phfc->pid_CollVspeed, phfc->ctrl_out[SPEED][COLL], phfc->IMUspeedGroundENU[2], phfc->ctrl_out[RAW][COLL]);
        }

        phfc->ctrl_out[RAW][COLL] = PID(&phfc->pid_CollVspeed, phfc->ctrl_out[SPEED][COLL], phfc->IMUspeedGroundENU[2], dT);
        //debug_print("%4.2f %4.2f %4.2f %5.3f - ", phfc->ctrl_out[RAW][COLL], phfc->ctrl_out[SPEED][COLL], phfc->IMUspeedGroundENU[UP], dT);
    }
    else {
      /* RC stick always sets RAW values. In AUTOPILOT, manual coll needs to be explicitly set here */
      /* this is only for auto takeoff-arm */
      float ctrl = phfc->ctrl_out[RAW][COLL];
      if (phfc->ctrl_source == CTRL_SOURCE_AUTOPILOT) {
          //ctrl = pConfig->CollZeroAngle;
          SetSpeedAcc(&phfc->ctrl_collective_raw, phfc->ctrl_collective_3d, pConfig->collective_man_speed, dT);
          ctrl = phfc->ctrl_collective_raw;
      }

      phfc->ctrl_out[RAW][COLL] = ClipMinMax(ctrl, phfc->pid_CollVspeed.COmin, phfc->pid_CollVspeed.COmax);
      phfc->ctrl_out[POS][COLL] = phfc->altitude;
    }

    phfc->collective_raw_curr = phfc->ctrl_out[RAW][COLL];

    // for fixed pitch prop, collective drives the throttle, throttle lever gates it
    if (pConfig->throttle_ctrl == PROP_FIXED_PITCH) {

        if (phfc->rw_cfg.AngleCollMixing) {
            phfc->ctrl_out[RAW][COLL] += phfc->rw_cfg.AngleCollMixing * (1/AngleCompensation-1);
        }

        phfc->ctrl_out[RAW][THRO] = phfc->ctrl_out[RAW][COLL];

        // if lever is low, set throttle to minimum and everything else to 0
        // to prevent any prop from accidental spinning because of PIDs
        if (phfc->throttle_value < -0.50f || !phfc->throttle_armed || (phfc->control_mode[COLL] < CTRL_MODE_SPEED && phfc->collective_value < -0.50f)
                || (((phfc->waypoint_type == WAYPOINT_TAKEOFF) || phfc->touch_and_go_takeoff)&& (phfc->waypoint_stage == FM_TAKEOFF_ARM || phfc->waypoint_stage == FM_TAKEOFF_AUTO_SPOOL))) {

            float throttle = pConfig->throttle_values[0];
            if (phfc->throttle_armed && phfc->throttle_value > -0.5f && ((phfc->waypoint_type == WAYPOINT_TAKEOFF) || phfc->touch_and_go_takeoff)
                    && (phfc->waypoint_stage == FM_TAKEOFF_ARM || phfc->waypoint_stage == FM_TAKEOFF_AUTO_SPOOL)) {
                throttle = -0.5;
            }

            ResetIterms();
            phfc->ctrl_out[RAW][THRO] = throttle;
            phfc->ctrl_out[RAW][PITCH] = 0;
            phfc->ctrl_out[RAW][ROLL]  = 0;
            phfc->ctrl_out[RAW][YAW]   = 0;
        }
    }

    // add offset for fine tuning of RPM
    phfc->ctrl_out[RAW][THRO] += phfc->rw_cfg.throttle_offset;

    // inhibit individual channels
    if (pConfig->ctrl_mode_inhibit[THRO] || !phfc->throttle_armed) {
        // Sets to Minimum Throttle
        phfc->ctrl_out[RAW][THRO] = pConfig->throttle_values[0];
    }

    if (pConfig->ctrl_mode_inhibit[PITCH]) {
        phfc->ctrl_out[RAW][PITCH] = phfc->pid_PitchRate.COofs;
    }

    if (pConfig->ctrl_mode_inhibit[ROLL]) {
        phfc->ctrl_out[RAW][ROLL] = phfc->pid_RollRate.COofs;
    }

    if (pConfig->ctrl_mode_inhibit[YAW]) {
        phfc->ctrl_out[RAW][YAW] = phfc->pid_YawRate.COofs;
    }

    if (pConfig->ctrl_mode_inhibit[COLL]) {
        phfc->ctrl_out[RAW][COLL] = phfc->pid_CollVspeed.COofs;
    }

    SetSpeedAcc(&phfc->mixer_in[PITCH], phfc->ctrl_out[RAW][PITCH]* pConfig->control_gains[PITCH], pConfig->servo_speed[PITCH], dT);
    SetSpeedAcc(&phfc->mixer_in[ROLL],  phfc->ctrl_out[RAW][ROLL] * pConfig->control_gains[ROLL],  pConfig->servo_speed[ROLL], dT);

    phfc->mixer_in[YAW]   = (phfc->ctrl_out[RAW][YAW] * pConfig->control_gains[YAW]);
    phfc->mixer_in[COLL]  = phfc->ctrl_out[RAW][COLL];
    phfc->mixer_in[THRO]  = phfc->ctrl_out[RAW][THRO];

    //Rotate2D(&phfc->mixer_in[ROLL], &phfc->mixer_in[PITCH], pConfig->RollPitchAngle); // this would interfere with SetSpeedAcc() just above

    ServoMixer();

    ServoOutput();

    Playlist_ProcessBottom(phfc, retire_waypoint);
}

// re-orients sensors within FCM, applies gains and offsets and the re-orients FCM
// @brief
// @param
// @retval
static void SensorsRescale(float accIn[3], float gyroIn[3], float accOut[3], float gyroOut[3])
{
    int i;
    float acc1[3];
    float gyro1[3];
    float tmp[3] = {0}; // temporary variable, used for calibration

    // recalculate gyro drifts based on temperature data
    float t = phfc->gyro_temp_lp;
    phfc->gyro_ofs[0] = t*( t*mpu.gyroP_temp_coeffs[0] + mpu.gyroP_temp_coeffs[1] ) +  mpu.gyroP_temp_coeffs[2];
    phfc->gyro_ofs[1] = t*( t*mpu.gyroR_temp_coeffs[0] + mpu.gyroR_temp_coeffs[1] ) +  mpu.gyroR_temp_coeffs[2];
    phfc->gyro_ofs[2] = t*( t*mpu.gyroY_temp_coeffs[0] + mpu.gyroY_temp_coeffs[1] ) +  mpu.gyroY_temp_coeffs[2];

    // remove gyro drift
    for (i=0; i<3; i++) {
      gyroIn[i] -= phfc->gyro_ofs[i];
    }

    // apply gain to gyro, if first order, take the diagonal only
    if (pConfig->gyro_first_order) {
        for (i=0; i<3; i++) {
            gyro1[i] = mpu.Cg[i][i] * gyroIn[i];
        }
    }
    else {
        for (i=0; i<3; i++) {
            gyro1[i] = mpu.Cg[i][0]*gyroIn[0]
                           + mpu.Cg[i][1]*gyroIn[1]
                           + mpu.Cg[i][2]*gyroIn[2];
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
            acc1[i] = mpu.Ca[i][i] * (accIn[i]- mpu.aofs[i]);
        }
    }
    else {
        for (i=0; i<3; i++) {
            acc1[i] = mpu.Ca[i][0] * (accIn[0] - mpu.aofs[0])
                            + mpu.Ca[i][1] * (accIn[1] - mpu.aofs[1])
                            + mpu.Ca[i][2] * (accIn[2] - mpu.aofs[2]);
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
        accOut[i]  = acc1[index];
        gyroOut[i] = gyro1[index];

        if (pConfig->fcm_orient[i+3]) {
          accOut[i]  = -accOut[i];
          gyroOut[i] = -gyroOut[i];
        }
    }
}

// @brief
// @param
// @retval
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

// @brief
// @param
// @retval
static void UpdateBatteryStatus(float dT)
{
    T_Power *p = &phfc->power;
    p->Itotal = p->Iesc + p->Iaux  + pConfig->current_offset;
    float power = p->Itotal * p->Vmain;
    float dE = power * dT;

    float energy_level_votlage_estimate = 0;
    float energy_voltage_estimate = 0;

    static int estimate_using_voltage = 1;
    static float estimate_using_voltage_timeout = 0;

    /* do not process if duration is not right */
    if (dT>1 || dT<=0) {
        return;
    }

    /* reset accumulated current when main voltage drops - battery swap */
    if (p->Vmain < 1) {
        p->capacity_used = 0;
    }

    /* integrate current to obtain used up capacity */
    p->capacity_used += p->Itotal * dT;

    p->power_lp = LP_RC(power, p->power_lp, 0.5f, dT);

    // Estimate of the energy level of a single battery cell using the voltage reading
    energy_level_votlage_estimate = UnloadedBatteryLevel(p->Vmain / Max(1, pConfig->battery_cells), pConfig->V2Energy);
    // Estimate of the energy level of the whole battery
    energy_voltage_estimate = p->energy_total * energy_level_votlage_estimate;

    if (!p->initialized) {
        p->energy_curr = energy_voltage_estimate;
    }

    /* Use the Voltage estimate for the battery level until the voltage stabilizes under
     * high load (high current, or flying) conditions */
    if (estimate_using_voltage) {
        /* low pass small changes in voltage, big changes go unfiltered to speed up the initial estimate */
        if (ABS(energy_voltage_estimate-p->energy_curr) > (0.3f*p->energy_total)) {
            p->energy_curr = energy_voltage_estimate;
        }
        else {
            p->energy_curr = LP_RC(energy_voltage_estimate, p->energy_curr, 0.05f, dT);
        }

        power = pConfig->power_typical;  // use typical power consumed to est flight time

        if( (p->Itotal > (5*pConfig->current_offset)) && IN_THE_AIR(phfc->altitude_lidar) ) {
            estimate_using_voltage_timeout += dT;

            // From captured data, it seems that it takes approximately 10 seconds
            // for the battery voltage to stablize after it's been loaded by the UAV motors.
            if (estimate_using_voltage_timeout >= 10.0f) {
                estimate_using_voltage = 0;
            }
        }

    }
    else { // Once energy_curr is initialized by voltage, use current draw to keep track for as long as FCM is on

        p->energy_curr -= dE;

        if (p->energy_curr < 0) {
            p->energy_curr = 0;
        }

//        if( p->energy_curr > energy_voltage_estimate   ) {
//            p->energy_curr -= (p->energy_curr - energy_voltage_estimate)/p->energy_curr * dE;
//        }
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

// @brief
// @param
// @retval
static void UpdateHwIdLow(int node_id, unsigned char *pdata)
{
    int board_type = pdata[0];

    if ((board_type < MAX_BOARD_TYPES) && (node_id < MAX_NODE_NUM)) {
      phfc->board_info[board_type][node_id].major_version = pdata[1];
      phfc->board_info[board_type][node_id].minor_version = pdata[2];
      phfc->board_info[board_type][node_id].build_version = pdata[3];

      phfc->board_info[board_type][node_id].serial_number2 = pdata[4] << 24;
      phfc->board_info[board_type][node_id].serial_number2 |= pdata[5] << 16;
      phfc->board_info[board_type][node_id].serial_number2 |= (pdata[6] << 8);
      phfc->board_info[board_type][node_id].serial_number2 |= (pdata[7]);
    }
}

// @brief
// @param
// @retval
static void UpdateHwIdHigh(int node_id, int board_type, unsigned char *pdata)
{
    if ((board_type < MAX_BOARD_TYPES) && (node_id < MAX_NODE_NUM)) {
      phfc->board_info[board_type][node_id].serial_number1 = pdata[0] << 24;
      phfc->board_info[board_type][node_id].serial_number1 |= pdata[1] << 16;
      phfc->board_info[board_type][node_id].serial_number1 |= (pdata[2] << 8);
      phfc->board_info[board_type][node_id].serial_number1 |= (pdata[3]);

      phfc->board_info[board_type][node_id].serial_number0 = pdata[4] << 24;
      phfc->board_info[board_type][node_id].serial_number0 |= pdata[5] << 16;
      phfc->board_info[board_type][node_id].serial_number0 |= (pdata[6] << 8);
      phfc->board_info[board_type][node_id].serial_number0 |= (pdata[7]);
    }
}

// @brief
// @param
// @retval
static void UpdateHardwareStatus(int node_id, int node_type, unsigned char *pdata)
{
  uint8_t hardware_status;
  if (node_type == AVI_GPS_NODETYPE) {

    hardware_status = pdata[0];

    if ((hardware_status & 0x1) == 0)
      phfc->system_status_mask |= N1_GPS0_FAIL;

    if ((hardware_status & 0x2) == 0)
      phfc->system_status_mask |= N1_GPS1_FAIL;

    if ((hardware_status & 0x10) == 0)
      phfc->system_status_mask |= N1_COMPASS0_FAIL;

    if ((hardware_status & 0x20) == 0)
      phfc->system_status_mask |= N1_COMPASS1_FAIL;
  }
}

// @brief Update servo node monitoring voltage
// @param
// @retval
static void UpdateServoNodeMonV(int node_id, unsigned char *pdata)
{
  phfc->servo_mon_voltage[node_id] = *(float *)pdata;
}

// @brief
// @param
// @retval
static void UpdateLidar(int node_id, int pulse_us)
{
  if (pulse_us == 0xFFFF) {
    // Lidar is offline, mark it so, and don't update value
    phfc->lidar_online_mask &= ~(1 << node_id);
    return;
  }

  phfc->lidar_online_mask |= (1 << node_id);

  // This handles the newer type sensors which max to 40cm when reading very close objects
  // as opposed to going to zero. When this occurs - fix reading to 10cm
  if (pulse_us >= 35000 && phfc->altitude_lidar_raw[node_id] < 1.0) {
    pulse_us = 100; // 10cm
  }

  MediatorInsert(lidar_median[node_id], pulse_us);
  uint16_t pulse = MediatorMedian(lidar_median[node_id]);

  phfc->altitude_lidar_raw[node_id] = ( (pulse*.001f) + 7.0f*phfc->altitude_lidar_raw[node_id] ) * 0.125f;
}

// @brief
// @param
// @retval
static void UpdateCastleLiveLink(int node_id, int seq_id, unsigned char *pdata)
{

  float *data= (float *)pdata;
  int new_data = 1;

  switch (seq_id) {
    case AVI_CL0:
      castle_link_live[node_id].battery_voltage = *data++;
      castle_link_live[node_id].ripple_voltage = *data;
      castle_link_live[node_id].new_data_mask |= (1 << 0);
      break;
    case AVI_CL1:
      castle_link_live[node_id].current = *data++;
      castle_link_live[node_id].output_power = *data;
      castle_link_live[node_id].new_data_mask |= (1 << 1);
      break;
    case AVI_CL2:
      castle_link_live[node_id].throttle = *data++;
      castle_link_live[node_id].rpm = *data;
      castle_link_live[node_id].new_data_mask |= (1 << 2);
      break;
    case AVI_CL3:
      castle_link_live[node_id].bec_voltage = *data++;
      castle_link_live[node_id].bec_current = *data;
      castle_link_live[node_id].new_data_mask |= (1 << 3);
      break;
    case AVI_CL4:
      castle_link_live[node_id].temperature = *(float *)data;
      castle_link_live[node_id].new_data_mask |= (1 << 4);
      break;
    default:
        break;
  }

  for (int i = 0; i < MAX_NUM_CASTLE_LINKS; i++) {
    if (castle_link_live[i].new_data_mask < 0x1F) {
      new_data = 0;
      break;
    }
  }

  if (new_data == 1) {
    new_data = 0;

    float Iaux = 0;
    float Iesc = 0;
    float Vmain = 0;
    float Vbec = 0;
    float RPM = 0;
    float esc_temp = 0;

    for (int i = 0; i < MAX_NUM_CASTLE_LINKS; i++) {
      castle_link_live[i].new_data_mask = 0;

      Iaux     += castle_link_live[i].bec_current;
      Iesc     += castle_link_live[i].current;
      Vmain    += castle_link_live[i].battery_voltage;
      Vbec     += castle_link_live[i].bec_voltage;

      if (pConfig->ccpm_type == CCPM_TANDEM) {
        RPM      += castle_link_live[i].rpm;
        esc_temp += castle_link_live[i].temperature;
      }
      else {
        // Case for ccpm120 TANDEM test bench heli:
        // This heli only has one motor even though it has two servo nodes and
        // two speed controls. Only the speed control on Servo Node #1 is connected
        // to the motor and so we are only concerned with the rpm and temperature
        // reported by castle link for servo node #1.
        if (i == 0) {
          RPM      = castle_link_live[i].rpm;
          esc_temp = castle_link_live[i].temperature;
        }
      }
    }

    Vmain    /= MAX_NUM_CASTLE_LINKS;
    Vbec     /= MAX_NUM_CASTLE_LINKS;

    if (pConfig->ccpm_type == CCPM_TANDEM) {
      RPM      /= MAX_NUM_CASTLE_LINKS;
      esc_temp /= MAX_NUM_CASTLE_LINKS;
    }

    phfc->power.Iaux   = Iaux;
    phfc->power.Iesc   = (Iesc + 3* phfc->power.Iesc ) * 0.25f;
    phfc->power.Iesc   = ClipMinMax(phfc->power.Iesc, 0, phfc->power.Iesc);

    phfc->power.Vmain  = (Vmain + 3* phfc->power.Vmain) * 0.25f;
    phfc->power.Vesc   = phfc->power.Vmain;

    phfc->power.Vservo = Vbec;
    phfc->power.Vservo = ClipMinMax(phfc->power.Vservo, 0, phfc->power.Vservo);

    phfc->power.Vaux   = Vbec;
    phfc->power.Vaux   = ClipMinMax(phfc->power.Vaux, 0, phfc->power.Vaux);

    phfc->esc_temp    = esc_temp;

    if (!pConfig->rpm_sensor) {
      phfc->RPM = (RPM / pConfig->gear_ratio / pConfig->motor_poles);
    }

    canbus_livelink_avail = 1;
  }
}

/* NOTE: The voltage and current reported by the power node is measured by
 * two different ADCs.
 * The ratio of the ADC1 voltage output and the actual voltage is 0.0197 V/mV.
 * The ratio of the ADC2 voltage output and the actual current is 0.04 A/mV.
 *
 * Therefore our model used is:
 *              actual = (    slope    )*( ADCvalue )
 * such that,
 *          Vactual[V] = (0.0197 [V/mv])*(Vadc1 [mv])        and
 *          Iactual[A] = (0.0400 [A/mv])*(Vadc2 [mv])
 *
 * In this function, several config variables are used to adjust the
 * current and voltage readings from the power node:
 * voltage_slope_percent_mod - percentage of the voltage slope to add or
 *                             subtract in order to adjust the voltage reading
 *                             from the power node.
 * current_slope_percent_mod - percentage of the current slope to add or
 *                             subtract in order to adjust the current reading
 *                             from the power node.
 *
 * Therefore:
 *     Vmain = (0.0197 + 0.0197*voltage_slope_percent_mod/100)*Vadc1   and,
 *     Iesc  = (0.0400 + 0.0400*current_slope_percent_mod/100)*Vadc2 + current_offset
 * */
static void UpdatePowerNodeVI(int node_id, unsigned char *pdata)
{
    float v, v_slope_mod, i, i_slope_mod;

    v = *(float *)pdata;
    pdata += 4;
    i = *(float *)pdata;

    v_slope_mod = (pConfig->voltage_slope_percent_mod / 100.0f) + 1.0f;
    phfc->power.Vmain = v*v_slope_mod;
    phfc->power.Vesc  = phfc->power.Vmain;

    i_slope_mod = (pConfig->current_slope_percent_mod / 100.0f) + 1.0f;
    phfc->power.Iesc =  i*i_slope_mod;

    power_update_avail = 1;
}

// @brief
// @param
// @retval
static void CanbusISRHandler(void)
{
  CANMessage can_rx_message;
  bool unknown_msg = false;

  while(Canbus->read(can_rx_message) && !unknown_msg) {

    int node_type = AVI_CAN_NODETYPE(can_rx_message.id);
    int node_id = (AVI_CAN_NODEID(can_rx_message.id) -1);
    int seq_id = AVI_CAN_SEQID(can_rx_message.id);
    int message_id = AVI_CAN_MSGID(can_rx_message.id);
    unsigned char *pdata = &can_rx_message.data[0];

    switch(message_id) {
    case AVI_MSGID_PDP:
      {
        if (seq_id == AVI_LIDAR) {
          UpdateLidar(node_id, *(uint32_t *)pdata);
        }
        else if (seq_id == AVI_VI) {
          if (node_type == AVI_PWR_NODETYPE) {
            UpdatePowerNodeVI(node_id, pdata);
          }
          else if (node_type == AVI_SERVO_NODETYPE) {
            UpdateServoNodeMonV(node_id, pdata);
          }
        }
        else if ((seq_id >= AVI_CL0) && (seq_id <= AVI_CL4)) {
          UpdateCastleLiveLink(node_id, seq_id, pdata);
        }
        else {
          unknown_msg = true;
        }
      }
      break;

    case AVI_MSGID_GPS:
      {
        if (seq_id <= AVI_GPS4) {
          gps.AddGpsData(node_id, seq_id, (char *)pdata);
        }
        else if (seq_id == AVI_COMPASS0) {
          compass->UpdateMagData(0, pdata);
        }
        else if (seq_id == AVI_COMPASS1) {
          compass->UpdateMagData(1, pdata);
        }
        else {
          unknown_msg = true;
        }
      }
      break;

    case AVI_MSGID_CTRL:
      {
        if (seq_id == AVI_ACK) {
          canbus_ack = 1;
        }
        else {
          unknown_msg = true;
        }
      }
      break;

    case AVI_MSGID_SDP:
      {
        if(seq_id == AVI_HWID_LOW) {
          UpdateHwIdLow(node_id, pdata);
        }
        else if(seq_id == AVI_HWID_HIGH) {
          UpdateHwIdHigh(node_id, node_type, pdata);
          can_node_found = 1;
        }
        else if (seq_id == AVI_HW_STATUS) {
          UpdateHardwareStatus(node_id, node_type, pdata);
        }
        else {
          unknown_msg = true;
        }
      }
      break;

    default:
      unknown_msg = true;
      break;
    }

    if(!unknown_msg) {
      phfc->stats.can_rx_msg_count[node_type][node_id]++;
    }
  }
}

// @brief
// @param
// @retval
static int CanbusNodeTypeStatus(int node_type)
{
  int state_mask = 0;

  switch (node_type) {
  case AVI_SERVO_NODETYPE:
    for (int node_id = 0; node_id < pConfig->num_servo_nodes; node_id++) {
      if (phfc->system_status_mask & (SERVO_NODE_FAIL << node_id)) {
        state_mask |= (1 << node_id);
      }
    }
    break;
  case AVI_PWR_NODETYPE:
    for (int node_id = 0; node_id < pConfig->num_power_nodes; node_id++) {
      if (phfc->system_status_mask & (PWR_NODE_FAIL << node_id)) {
        state_mask |= (1 << node_id);
      }
    }
    break;
  case AVI_GPS_NODETYPE:
    for (int node_id = 0; node_id < pConfig->num_gps_nodes; node_id++) {
      if (phfc->system_status_mask & (NAV_NODE_FAIL << node_id)) {
        state_mask |= (1 << node_id);
      }
    }
    break;
  default:
    break;
  }
  return state_mask;
}

// @brief
// @param
// @retval
static void SystemMonitor(float dT)
{
  static float imu_check_timeout = IMU_CHECK_TIMEOUT;
  static float baro_check_timeout = BARO_CHECK_TIMEOUT;

  // Check Canbus node operations
  CanbusNodeMonitor(dT);

  // Check IMU operations
  imu_check_timeout -= dT;
  if (imu_check_timeout <= 0) {
    if (phfc->imu_error_count > IMU_ERROR_THRESHOLD) {
      phfc->system_status_mask |= IMU_FAIL;
      // WARNING - THIS IS LAST RESORT AND WILL SEND FAILSAFE TO CANBUS NODES
      //  - ULTIMATELY KILLING POWER AND AUTOROTATING
      FailSafeMode();
    }
    imu_check_timeout = IMU_CHECK_TIMEOUT;
  }

  baro_check_timeout -= dT;
  if (baro_check_timeout <= 0) {
    if (phfc->baro_error_count > BARO_ERROR_THRESHOLD) {
      phfc->system_status_mask |= BARO_FAIL;
    }
    baro_check_timeout = BARO_CHECK_TIMEOUT;
  }

  // update lidar status into the system mask
  if (phfc->lidar_online_mask & 0x1) {
    phfc->system_status_mask &= ~LIDAR_FRONT_FAIL;
  }
  else {
    phfc->system_status_mask |= LIDAR_FRONT_FAIL;
  }

  if (phfc->num_lidars == 2) {
    if (phfc->lidar_online_mask & 0x2) {
      phfc->system_status_mask &= ~LIDAR_REAR_FAIL;
    }
    else {
      phfc->system_status_mask |= LIDAR_REAR_FAIL;
    }
  }
}

// @brief
// @param
// @retval
static void CanbusNodeMonitor(float dT)
{
  static unsigned int can_rx_msg_count[MAX_BOARD_TYPES][MAX_NODE_NUM] = {0};
  static float can_timeout = CAN_NODE_TIMEOUT;
  static int gps_reconfig_active = 0;
  static int gps_reconfigure_timeout = 0;

  can_timeout -= dT;

  // For each configured node, ensure we are receiving canbus transactions
  if (can_timeout <= 0) {

    // Check GPS
    for (int node_id = 0; node_id < pConfig->num_gps_nodes; node_id++) {
      if ((phfc->stats.can_rx_msg_count[AVI_GPS_NODETYPE][node_id] - can_rx_msg_count[AVI_GPS_NODETYPE][node_id]) == 0) {
        // This device has gone offline
        phfc->system_status_mask |= (NAV_NODE_FAIL << node_id);
        //SetFcmLedState(0xf);
      }
      else {
        phfc->system_status_mask &= ~(NAV_NODE_FAIL << node_id);
      }
      can_rx_msg_count[AVI_GPS_NODETYPE][node_id] = phfc->stats.can_rx_msg_count[AVI_GPS_NODETYPE][node_id];
    }

    // Check servo nodes
    for (int node_id = 0; node_id < pConfig->num_servo_nodes; node_id++) {
      if ((phfc->stats.can_rx_msg_count[AVI_SERVO_NODETYPE][node_id] - can_rx_msg_count[AVI_SERVO_NODETYPE][node_id]) == 0) {
        // This device has gone offline
        phfc->system_status_mask |= (SERVO_NODE_FAIL << node_id);
        //SetFcmLedState(0xf);
      }
      else {
        phfc->system_status_mask &= ~(SERVO_NODE_FAIL << node_id);
      }
      can_rx_msg_count[AVI_SERVO_NODETYPE][node_id] = phfc->stats.can_rx_msg_count[AVI_SERVO_NODETYPE][node_id];
    }

    // check power nodes
    for (int node_id = 0; node_id < pConfig->num_power_nodes; node_id++) {
      if ((phfc->stats.can_rx_msg_count[AVI_PWR_NODETYPE][node_id] - can_rx_msg_count[AVI_PWR_NODETYPE][node_id]) == 0) {
        // This device has gone offline
        phfc->system_status_mask |= (PWR_NODE_FAIL << node_id);
      }
      else {
        phfc->system_status_mask &= ~(PWR_NODE_FAIL << node_id);
      }
      can_rx_msg_count[AVI_PWR_NODETYPE][node_id] = phfc->stats.can_rx_msg_count[AVI_PWR_NODETYPE][node_id];
    }
    can_timeout = CAN_NODE_TIMEOUT;
  }

  // If we have multiple servo/power nodes AND only one of those nodes is offline, we need to
  // tell the other node to go Failsafe
  int node_offline_mask;

  if ((node_offline_mask = CanbusNodeTypeStatus(AVI_SERVO_NODETYPE)) > 0) {
    for (int node_id = 0; node_id < pConfig->num_servo_nodes; node_id++) {
     if((node_offline_mask & (1 << node_id)) == 0) {
       // This node must be told to go failsafe.
       CanbusFailsafe(AVI_SERVO_NODETYPE, node_id);
     }
    }
  }

  if ((node_offline_mask = CanbusNodeTypeStatus(AVI_PWR_NODETYPE)) > 0) {
    for (int node_id = 0; node_id < pConfig->num_servo_nodes; node_id++) {
     if((node_offline_mask & (1 << node_id)) == 0) {
       // This node must be told to go failsafe.
       CanbusFailsafe(AVI_PWR_NODETYPE, node_id);
     }
    }
  }

  // IF GPS is offline, we can make an attempt to being it back up.
  CANMessage can_tx_message;
  for (int node_id = BASE_NODE_ID; node_id <= pConfig->num_gps_nodes; node_id++) {
    // Allow Node to auto select GPS Chip and specify compass combination based on configuration.
    if (!gps_reconfig_active) {
      can_tx_message.data[0] = DEFAULT_GPS_CFG;
      can_tx_message.len = 1;
      ConfigureCanbusNode(AVI_GPS_NODETYPE, node_id, AVI_CFG, &can_tx_message, 0);
      gps_reconfigure_timeout = 100;
      gps_reconfig_active = 1;
    }

    if(--gps_reconfigure_timeout == 0) {
      can_tx_message.len = 0;
      can_tx_message.id = AVI_CAN_ID(AVI_GPS_NODETYPE, node_id, AVI_PDP_ON, AVI_MSGID_CTRL);
      Canbus->write(can_tx_message);
      gps_reconfig_active = 0;
    }
  }
}

// @brief
// @param
// @retval
static void FailSafeMode(void)
{
  for (int node_id = 0; node_id < pConfig->num_servo_nodes; node_id++) {
    CanbusFailsafe(AVI_SERVO_NODETYPE, node_id);
  }

  for (int node_id = 0; node_id < pConfig->num_power_nodes; node_id++) {
    CanbusFailsafe(AVI_PWR_NODETYPE, node_id);
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
 * 1. phfc->compass_cal.compassMin[3]: minimum values measured from compass in x, y and z.
 *                       Reset to -9999 when new calibration initiated.
 *                       Constantly being updated during flight so that
 *                       offsets and gains are always up-to-date.
 *                       If new value is -200 less then current min then
 *                       assume it is an anomaly and IGNORE
 *
 * 2. phfc->compass_cal.compassMax[3]: maximum values measured from compass in x, y and z
 *                       Reset to +9999 when new calibration initiated.
 *                       Constantly being updated during flight so that
 *                       offsets and gains are always up-to-date.
 *                       If new value is +200 more then current max then
 *                       assume it is an anomaly and IGNORE.
 *
 * 3. phfc->IMUorient[PITCH,ROLL]: used to check what orientation the compass is in
 *
 * 4. phfc->comp_pitch[PITCH_COMP_LIMIT]: array of flags to check if a compass
 *                 measurement was made at pitch angles from -PITCH_COMP_LIMIT/2
 *                 to +PITCH_COMP_LIMIT/2.
 *                 Reset to ZEROS when new calibration is Initiated.
 *
 * 5. phfc->comp_roll[ROLL_COMP_LIMIT]: array of flags to check if a compass
 *                 measurement was made at roll angles from -ROLL_COMP_LIMIT/2
 *                 to +ROLL_COMP_LIMIT/2.
 *                 Reset to ZEROS when new calibration is Initiated.
 *
 * Outputs:
 * 1. pConfig->comp_ofs[3] = (phfc->compassMin[i]+phfc->compassMax[i]+1)/2;
 * 2. pConfig->comp_gains[3] = 500.0f/((phfc->compassMax[i]-phfc->compassMin[i])/2.0f);*
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
    int mag_range[3] = {0};
    int max_range = 1430;

    if( phfc->comp_calibrate == NO_COMP_CALIBRATE ) {
        return;
    }
    else if ( phfc->comp_calibrate == COMP_CALIBRATE_DONE ) {
        phfc->comp_calibrate = NO_COMP_CALIBRATE;
        return;
    }

    /*UPdate Max and Min compass readings on x,y,z axes.
     * - Always use new minimum value if in calibration mode, the assumption here
     *   is that the user knows only to calibrate in an area void of EM interference
     * - If not calibrating, only update max and min if limits are not exceeded*/
    int max_min_changed = 0;
    for (i=0; i<3; i++) {
      float fDataXYZ = compass->GetMagData(i);

      phfc->compass_cal.compassMin[i] = min(phfc->compass_cal.compassMin[i], fDataXYZ);
      phfc->compass_cal.compassMax[i] = max(phfc->compass_cal.compassMax[i], fDataXYZ);

      mag_range[i] = phfc->compass_cal.compassMax[i] - phfc->compass_cal.compassMin[i];

      if(phfc->compass_cal.compassMax[i] == fDataXYZ
            || phfc->compass_cal.compassMin[i] == fDataXYZ) {
          max_min_changed = 1;
      }
    }

    for(i = 0; i<3; i++) {
      if (mag_range[i] > max_range) {
        phfc->compass_cal.compassMax[i] = 0;
        phfc->compass_cal.compassMin[i] = 0;
      }
    }

    // The max and min values have changed, report to the ground station
    if (0 != max_min_changed) {
      int size = telem.CalibrateCompass();
      telem.AddMessage((unsigned char*)&phfc->telemCalibrate, size, TELEMETRY_CALIBRATE, 6);
    }

#if 0
    //int i_pitch = 0;        //index used for comp_pitch[] flags array
    //int i_roll  = 0;        //index used for comp_roll[] flags array
    //int min_range = 450;

    // This procedure is no longer being used

    /*Set the pitch angle flag index and roll angle flag index to the
     * current pitch and roll angle of the IMU.
     * Re-adjust the indices so that they fit the range of the
     * comp_pitch_flags[] and comp_roll_flags[] arrays since we have
     * to take into account negative angles or angles outside of the
     * PITCH and ROLL angle limits (PITCH_COMP_LIMIT,ROLL_COMP_LIMIT)*/
    i_pitch = ceil(phfc->IMUorient[PITCH]*R2D)*PITCH_COMP_LIMIT/180 + PITCH_COMP_LIMIT/2;
    i_roll  = ceil(phfc->IMUorient[ROLL]*R2D)*ROLL_COMP_LIMIT/360  + ROLL_COMP_LIMIT/2;

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
    phfc->comp_pitch_flags[i_pitch] += 1;//phfc->IMUorient[PITCH]*R2D;
    phfc->comp_roll_flags[i_roll]   += 1;//phfc->IMUorient[ROLL]*R2D;


    /*At this point i_pitch and i_roll are used as flags
     * to see if comp_pitch and comp_roll, respectively, have
     * been filled.*/
    i_pitch = 1;
    i_roll = 1;
    for(i = 0; i < ROLL_COMP_LIMIT; i++) {
        if( (i < PITCH_COMP_LIMIT)  &&
            (phfc->comp_pitch_flags[i] < NUM_ANGLE_POINTS) ){
            i_pitch = 0;
            break;
        }
        else
        if( (i < ROLL_COMP_LIMIT)  &&
            (phfc->comp_roll_flags[i] < NUM_ANGLE_POINTS) ){
            i_roll = 0;
            break;
        }
    }

    if ( (i_pitch == 1) &&
         (i_roll  == 1) &&
         (mag_range[0] >= min_range) &&
    	 (mag_range[1] >= min_range) &&
         (mag_range[2] >= min_range)
         ) {

        phfc->comp_calibrate = COMP_CALIBRATE_DONE;
    }
    else //if we do all orientations but range is wrong then reset orientations
    if ( (i_pitch == 1) &&
         (i_roll  == 1) &&
         (mag_range[0] <= min_range) &&
         (mag_range[1] <= min_range) &&
         (mag_range[2] <= min_range) ){

        for(i = 0; i < PITCH_COMP_LIMIT; i++) {
            phfc->comp_pitch_flags[i] = 0;
        }
        for(i = 0; i < ROLL_COMP_LIMIT; i++) {
            phfc->comp_roll_flags[i] = 0;
        }
    }
#endif

    if (phfc->comp_calibrate == COMP_CALIBRATE_DONE) {
        CompassCalDone();
    }

}

/* Function used to DISARM the UAV if it has been armed for longer than
 * but motors have not come on.
 * Ignore timeout if Preflight checsk are disabled in config
 * dT is the elapsed time since this function was run, in seconds
 */
// @brief
// @param
// @retval
static void ArmedTimeout(float dT)
{

  if (pConfig->disable_pre_flight) {
      return;
  }

  static float armed_timer = ARMED_TIMEOUT;

  if (!phfc->throttle_armed) {
    armed_timer = ARMED_TIMEOUT;
  }
  else if (   phfc->throttle_armed
           && ( (GetMotorsState()==0) || (phfc->fixedThrottleMode <= THROTTLE_DEAD) )) {
    armed_timer -= dT;
  }
  else {
    armed_timer = ARMED_TIMEOUT;
  }

  if (armed_timer <= 0.0f) {
    telem.SendMsgToGround(MSG2GROUND_TAKEOFF_TIMEOUT);
    telem.Disarm();
  }
}

// @brief Called every ms and increments the flight time counter,
//        - Counter is only started and incremented when we are
//        - armed and motors are ON or Throttle is in RAMP or FLY mode
// @param
// @retval
static void FlightOdometer(void)
{
  if (phfc->throttle_armed && (GetMotorsState() || (phfc->fixedThrottleMode > THROTTLE_DEAD))) {
    phfc->OdometerReading++;
  }
}

// @brief
// @param
// @retval
static void DoFlightControl()
{
    int i;
    int ticks;
    int time_ms;
    float dT;  // runtime of do_control() loop, in seconds
    float baro_altitude_raw_prev;
    int utilization = 0;

    KICK_WATCHDOG();

    if (!(phfc->system_status_mask & IMU_FAIL)) {
      if (!mpu.readMotion7f_finish(phfc->accRaw, phfc->gyroRaw, &phfc->gyro_temperature)) {
        phfc->imu_error_count++;
      }
      else {
        phfc->imu_error_count = 0;
      }
    }

    //LedTesterOff();
    ticks = Ticks_us_minT(1000, &utilization);   // defines loop time in uS, 1000Hz (1ms)

    //LedTesterToggle();
    //LedTesterOn();
    if (!(phfc->system_status_mask & IMU_FAIL)) {
      mpu.readMotion7_start();
    }

    time_ms = GetTime_ms();
    dT = ticks*0.000001f;
    phfc->time_ms    = time_ms;
    phfc->ticks_curr = ticks;
    phfc->ticks_max  = max(phfc->ticks_max, ticks);
    phfc->ticks_lp   = (ticks*64 + 63*phfc->ticks_lp+32)>>6;
    phfc->cpu_utilization_curr = utilization*0.1f;
    phfc->cpu_utilization_lp = (phfc->cpu_utilization_curr + phfc->cpu_utilization_lp*255) * 0.00390625f;

    if (phfc->gyro_temp_lp == 0) {
      phfc->gyro_temp_lp = phfc->gyro_temperature;
    }
    else {
      phfc->gyro_temp_lp = (phfc->gyro_temperature + 511*phfc->gyro_temp_lp)*0.001953125f;
    }

    //ArmedTimeout(dT);

    FlightOdometer();

    /* copy and clear the new data flag set by GPS to a new variable to avoid a race */
    phfc->gps_new_data = gps.GpsUpdate();

    // Check for new Compass Data
    if (compass->HaveNewData()) {
      phfc->compass_heading = compass->GetHeadingDeg(phfc->IMUorient[PITCH], phfc->IMUorient[ROLL]);

      if (phfc->compass_heading_lp == 0) {
          phfc->compass_heading_lp = phfc->compass_heading;
      }
      else {
          // phfc->compass_heading_lp = LP_16_Wrap180(phfc->compass_heading, phfc->compass_heading_lp);
          phfc->compass_heading_lp = LP_Wrap180(phfc->compass_heading, phfc->compass_heading_lp, pConfig->heading_avgs);
      }

      // debug_print("Comp: %+5.1f %+5.1f\r\n", phfc->compass_heading, phfc->compass_heading_lp);
      CompassCalibration();

      phfc->compass_heading_raw[pConfig->compass_selection] = compass->GetRawHeadingXY(pConfig->compass_selection);
      if (pConfig->compass_selection == 0) {
        if (compass->HaveNewData(1)) {
          phfc->compass_heading_raw[1] = compass->GetRawHeadingXY(1);
        }
      }
      else {
        if (compass->HaveNewData(0)) {
          phfc->compass_heading_raw[0] = compass->GetRawHeadingXY(0);
        }
      }
    }

    if (pConfig->baro_enable == 1) {
      phfc->baro_dT += dT;
      baro_altitude_raw_prev = phfc->baro_altitude_raw_lp;

      // Baro updates every 25ms
      int result = 0;
      if ((result = baro.GetTPA(dT, &phfc->baro_temperature, &phfc->baro_pressure, &phfc->baro_altitude_raw)) == 1) {
        //    phfc->baro_vspeedDF = DerivativeFilter11(phfc->baro_altitude_raw, phfc->baro_derivative_filter)/phfc->baro_dT;
        if (phfc->baro_altitude_raw_lp < -999) {
            phfc->altitude_baro = phfc->baro_altitude_raw_lp = baro_altitude_raw_prev = phfc->baro_altitude_raw;
        }

        phfc->baro_altitude_raw_lp = LP4_1000(&phfc->lp_baro4, phfc->baro_altitude_raw);
        phfc->baro_vspeed          = (phfc->baro_altitude_raw_lp - baro_altitude_raw_prev)/phfc->baro_dT;
        phfc->baro_vspeed_lp = LP4_1000(&phfc->lp_baro_vspeed4,phfc->baro_vspeed);

        //    phfc->baro_altitude_raw_lp = (phfc->baro_altitude_raw + 7*phfc->baro_altitude_raw_lp)*0.125f;   // about 0.25 second lowpass
        //    phfc->baro_vspeed          = (phfc->baro_altitude_raw_lp - baro_altitude_raw_prev)/phfc->baro_dT;
        //    debug_print("T %5.1f P %5.0f Alt %6.2f AltLP %6.2f vs %+3.1f dT %5.3f\r\n", phfc->baro_temperature, phfc->baro_pressure, phfc->baro_altitude_raw, phfc->baro_altitude_raw_lp, phfc->baro_vspeed, phfc->baro_dT);
        //    debug_print("vs %+5.3f  aB %+5.3f  aBrawLP4 %+5.3f  aBrawLP %+5.3f\n", phfc->IMUspeedGroundENU[2], phfc->altitude, phfc->baro_altitude_rawLP4, phfc->baro_altitude_raw_lp);

        phfc->baro_dT = 0;
        phfc->baro_error_count = 0;
      }
      else {
        if (result == -1) {
          // Keep track of timeout reads
          phfc->baro_error_count++;
        }
      }
    }


    /* low-pass sensors */
    /* gyro +/-500 deg/s, acc +/-4G */
    for (i=0; i<3; i++)  {
        phfc->gyroFilt[i] = LP4_1000(&phfc->lp_gyro4[i], phfc->gyroRaw[i]);
    }

    for (i=0; i<3; i++) {
        phfc->accFilt[i]  = LP4_1000(&phfc->lp_acc4[i],  phfc->accRaw[i]);
    }

    /* remap ACC axes into my XYZ (RFU) */
    SensorsRescale(phfc->accFilt, phfc->gyroFilt, phfc->acc, phfc->gyro);

    for (i=0; i<3; i++) {
        phfc->accHeliRFU[i] = phfc->acc[i];
    }

    // secondary gyro offset for fine drift removal
    for (i=0; i<3; i++) {
        phfc->gyro[i] -= phfc->gyroOfs[i];
    }

    // low passed gyro averaged value for dynamic gyro calibration
    for (i=0; i<3; i++) {
        phfc->gyro_lp_disp[i] = (phfc->gyro[i] + phfc->gyro_lp_disp[i]*4095)/4096;
    }

    Get_Orientation(phfc->SmoothAcc, phfc->acc, dT);
    //debug_print("%+5.2f %+5.2f %+5.2f   %+5.2f %+5.2f %+5.2f\n", gyroRaw[PITCH], gyroRaw[ROLL], gyroRaw[YAW], phfc->gyroFilt[PITCH], phfc->gyroFilt[ROLL], phfc->gyroFilt[YAW]);

    MadgwickAHRSupdateIMU(dT, (phfc->gyro[ROLL])*D2R, (phfc->gyro[YAW])*D2R, (phfc->gyro[PITCH])*D2R, 0,0,0);

    if (IMU_Q2PRY_fast(phfc->IMUorient)) {
        /* orient is in rad, gyro in deg */
        phfc->gyroOfs[PITCH] = -PID(&phfc->pid_IMU[PITCH], phfc->SmoothAcc[PITCH]*R2D-phfc->bankPitch,phfc->IMUorient[PITCH]*R2D, dT);
        phfc->gyroOfs[ROLL]  = -PID(&phfc->pid_IMU[ROLL],  phfc->SmoothAcc[ROLL]*R2D+phfc->bankRoll,  phfc->IMUorient[ROLL]*R2D,  dT);
        phfc->gyroOfs[YAW] = -PID(&phfc->pid_IMU[YAW], phfc->compass_heading_lp, phfc->IMUorient[YAW]*R2D, dT);

        /*
        if (!(phfc->print_counter & 0x3ff)) {
            debug_print("Gyro %+6.3f %+6.3f %+6.3f  IMU %+6.3f %+6.3f %+6.3f  Err %+6.3f %+6.3f %+6.3f\r\n",
            phfc->gyroOfs[PITCH],   phfc->gyroOfs[ROLL], phfc->gyroOfs[YAW],
            phfc->IMUorient[PITCH], phfc->IMUorient[ROLL], phfc->IMUorient[YAW],
            phfc->SmoothAcc[PITCH]*R2D - phfc->IMUorient[PITCH]*R2D, phfc->SmoothAcc[ROLL]*R2D - phfc->IMUorient[ROLL]*R2D, phfc->compass_heading_lp - phfc->IMUorient[YAW]*R2D);
            debug_print("Ofs %+6.3f %+6.3f %+6.3f  Gyro %+6.3f %+6.3f %+6.3f Err %+6.3f %+6.3f %+6.3f\r\n", phfc->gyroOfs[0], phfc->gyroOfs[1],
                              phfc->gyroOfs[2], phfc->gyro_lp_disp[0], phfc->gyro_lp_disp[1], phfc->gyro_lp_disp[2], phfc->SmoothAcc[PITCH]*R2D - phfc->IMUorient[PITCH]*R2D,
                              phfc->SmoothAcc[ROLL]*R2D - phfc->IMUorient[ROLL]*R2D, phfc->compass_heading_lp - phfc->IMUorient[YAW]*R2D);
            debug_print("IMU %+6.3f %+6.3f %+6.3f SmAcc %+6.3f %+6.3f %+6.3f Com %+6.3f %+6.3f\r\n", phfc->IMUorient[PITCH]*R2D, phfc->IMUorient[ROLL]*R2D,
                          phfc->IMUorient[YAW]*R2D, phfc->SmoothAcc[PITCH]*R2D, phfc->SmoothAcc[ROLL]*R2D, phfc->SmoothAcc[YAW]*R2D, phfc->compass_heading_lp, phfc->compass_heading);
            debug_print("%+8.5f %+5.1f %+5.1f %f %f\r\n", phfc->gyroOfs[0], phfc->SmoothAcc[PITCH]*R2D, phfc->IMUorient[PITCH]*R2D, phfc->pid_IMU[PITCH].Kp, phfc->pid_IMU[PITCH].Ki);
        }
        */
        OrientResetCounter();
    }

    /* Accelerometer based vertical speed, GPS vspeed blended in */
    float accGroundENU[3];

    Plane2Ground(phfc->accHeliRFU, phfc->IMUorient, accGroundENU);

    accGroundENU[2] -= 1; // remove gravity

    // high pass filter U2 = T/(1+T)*(U-Uprev+U2prev)
    for (i=0; i<3; i++) {
        phfc->accGroundENUhp[i] = 0.99993896484375f*(accGroundENU[i]-phfc->accGroundENU_prev[i]+phfc->accGroundENUhp[i]);   // T=16384 ~ 14sec 0.99993896484375 T=4096 ~3.4s
        phfc->accGroundENU_prev[i] = accGroundENU[i];
        phfc->IMUspeedGroundENU[i] += phfc->rw_cfg.AccIntegGains[i] * phfc->accGroundENUhp[i]*9.81f*dT;

        // always mix in GPS for X and Y speed
        if (i<2) {
            phfc->IMUspeedGroundENU[i] += 1.0f*dT*(phfc->GPSspeedGroundENU[i] - phfc->IMUspeedGroundENU[i]);    // blend in GPS speed, it drifts if 0.25
        }
        // blend in GPS vertical speed with IMU vertical speed
        else if (phfc->rw_cfg.gps_vspeed == 1 ) {
            phfc->IMUspeedGroundENU[2] += phfc->rw_cfg.GPSVspeedWeight*dT*(phfc->GPSspeedGroundENU[2] - phfc->IMUspeedGroundENU[2]);    // blend in GPS speed
        }
        // blend in Baro vertical speed with IMU vertical speed
        else if ((phfc->rw_cfg.gps_vspeed) == 2 && (pConfig->baro_enable)) {
            phfc->IMUspeedGroundENU[2] += phfc->rw_cfg.BaroVspeedWeight*dT*(phfc->baro_vspeed_lp - phfc->IMUspeedGroundENU[2]);    // blend in baro vspeed
        }
        // blend in GPS and Baro vertical speed with IMU vertical speed
        else if ((phfc->rw_cfg.gps_vspeed == 3) && (pConfig->baro_enable)) {
            phfc->IMUspeedGroundENU[2] += phfc->rw_cfg.GPSVspeedWeight *dT*(phfc->GPSspeedGroundENU[2] - phfc->IMUspeedGroundENU[2])
                            + phfc->rw_cfg.BaroVspeedWeight*dT*(phfc->baro_vspeed_lp       - phfc->IMUspeedGroundENU[2]);    // blend in GPS and baro vspeed
        }
        // use only GPS for vertical speed
        else if (phfc->rw_cfg.gps_vspeed == 4 ) {
            phfc->IMUspeedGroundENU[2] = phfc->GPSspeedGroundENU[2];
        }
        // use only Baro for vertical speed
        else if ((phfc->rw_cfg.gps_vspeed == 5) && (pConfig->baro_enable)) {
            phfc->IMUspeedGroundENU[2] = phfc->baro_vspeed_lp;
        }
        else {
            // blend in GPS vertical speed with IMU vertical speed (phfc->config.gps_vspeed == 1 )
            phfc->IMUspeedGroundENU[2] += phfc->rw_cfg.GPSVspeedWeight*dT*(phfc->GPSspeedGroundENU[2] - phfc->IMUspeedGroundENU[2]);    // blend in GPS speed
        }
    }

    /*
    if (!(phfc->print_counter & 0x3f)) {
        debug_print("Z %+5.3f UR %+5.3f Uhp %+5.3f speed %+5.3f\r\n", phfc->accHeliRFU[2], accGroundU, phfc->accGroundUhp, phfc->speedGroundU);
    }
    if (!(phfc->print_counter & 0x3f)) {
        debug_print("E %+5.2f N %+5.2f U %+5.2f\n", phfc->IMUspeedGroundENU[0], phfc->IMUspeedGroundENU[1], phfc->IMUspeedGroundENU[2]);
    }
    if (!(phfc->print_counter & 0x3f)) {
        debug_print("E %+5.3f N %+5.3f U %+5.3f  E %+5.3f N %+5.3f U %+5.3f\n", phfc->accGroundENUhp[0], phfc->accGroundENUhp[1], phfc->accGroundENUhp[2], phfc->IMUspeedGroundENU[0], phfc->IMUspeedGroundENU[1], phfc->IMUspeedGroundENU[2]);
    }
    */

    /* help baro-altitude using vertical speed */
    if (pConfig->baro_enable == 1) {
        phfc->altitude_baro += phfc->IMUspeedGroundENU[2] * dT;
        phfc->altitude_baro += phfc->rw_cfg.BaroAltitudeWeight*dT*(phfc->baro_altitude_raw_lp - phfc->altitude_baro);    // blend in baro vspeed
        //phfc->altitude_baro += 0.25f*dT*(phfc->baro_altitude_raw_lp - phfc->altitude_baro);    // blend in baro vspeed
    }

    /*
    phfc->accUp += phfc->accGroundENUhp[2]*9.81f;
    if (!(phfc->print_counter & 0x7)) {
        phfc->accUp/=8;
        debug_print("T %d acc %+5.3f vs %+5.3f altIMU %5.3f altB %5.3f\n", phfc->time_ms, phfc->accUp, phfc->IMUspeedGroundENU[2], phfc->altitude_baro, phfc->baro_altitude_raw);
        phfc->accUp = 0;
    }*/

    /* horizontal position is an integral of horizontal speed */
    /* convert horizontal speeds in m/s to Earth-rotational latitude/longitude speeds */
    /* blend in GPS position */
    /* integrate E/N speed into delta Lat/Lon */

    float dLat = phfc->GPSspeedGroundENU[1] * dT * DPM;   // lat
    float dLon = phfc->GPSspeedGroundENU[0] * dT * DPM / COSfD((float)phfc->positionLatLon[0]);   // lon

    /* adjust position */
    phfc->positionLatLon[0] += (double)dLat;
    phfc->positionLatLon[1] += (double)dLon;
    
    /* if GPS detects a glitch, set the blending factor to the long value, otherwise decay towards the regular blanding value */
    if (gps.glitch_) {
        phfc->Pos_GPS_IMU_Blend = phfc->rw_cfg.Pos_GPS_IMU_BlendGlitch;
    }
    else if (phfc->Pos_GPS_IMU_Blend>phfc->rw_cfg.Pos_GPS_IMU_BlendReg) {
        phfc->Pos_GPS_IMU_Blend -= dT;
    }

    /* if GPS coordinates are more than 222m away from the current pos, just reset it, otherwise blend the current position with GPS */
    if (gps.gps_data_.fix) {
        double gps_latitude  = gps.gps_data_.latD;
        double gps_longitude = gps.gps_data_.lonD;
        if ((ABS(gps_latitude-phfc->positionLatLon[0]) > 0.002) || (ABS(gps_longitude-phfc->positionLatLon[1]) > 0.002)) {
            phfc->positionLatLon[0] = gps_latitude;
            phfc->positionLatLon[1] = gps_longitude;
        }
        else {
            /* HspeedGPSaccBlend could be adaptive with speed - more gps at high speed, more acc at low speeds */
            if (phfc->Pos_GPS_IMU_Blend > 0) {
                phfc->positionLatLon[0] += (1/phfc->Pos_GPS_IMU_Blend) * dT * (gps_latitude-(phfc->positionLatLon[0]));
                phfc->positionLatLon[1] += (1/phfc->Pos_GPS_IMU_Blend) * dT * (gps_longitude-(phfc->positionLatLon[1]));
            }
        }
    }
  
    /* offset baro based altitude to match GPS */
    phfc->altitude = phfc->altitude_baro + phfc->altitude_ofs;
    phfc->gps_to_home[2] = phfc->altitude - phfc->home_pos[2];

    if (phfc->gps_new_data) {
        double latitude  = gps.gps_data_.latD;
        double longitude = gps.gps_data_.lonD;

        phfc->altitude_gps = gps.gps_data_.altitude;
        //phfc->altitude = phfc->altitude_gps;

        phfc->gps_to_home[0] = DistanceCourse(latitude, longitude, phfc->home_pos[0], phfc->home_pos[1], &phfc->gps_to_home[1]);
        //debug_print("D %4.1f C %+5.1f  \r\n", phfc->gps_to_ref[0], phfc->gps_to_ref[1]);

        /* if we have fix and a new position data, run the gradient descent algo
        ** to bring baro-altitude in sync with gps altitude */
        if (gps.gps_data_.fix > GPS_FIX_NONE && gps.gps_data_.PDOP < 250)
        {
            float dTGPS = 0.001f * ((int)(time_ms - phfc->tGPS_prev));
            phfc->tGPS_prev = time_ms;

            /* initialize altitude only the first time */
            if (!phfc->gps_alt_initialized) {
                phfc->altitude_ofs = phfc->altitude_gps - phfc->altitude_baro;
                phfc->gps_alt_initialized = true;
                gps.glitches_ = 0;
            }
            else if (dTGPS < 1 && dTGPS>0) {
                phfc->altitude_ofs +=  dTGPS * (phfc->altitude_gps - phfc->altitude) / phfc->AltitudeBaroGPSblend;
                /* decay the initial blending factor into the final value */
                phfc->AltitudeBaroGPSblend = min(phfc->AltitudeBaroGPSblend+dTGPS, phfc->rw_cfg.AltitudeBaroGPSblend_final);
                //debug_print("%8d\t%5.3f\t%f\t%f\t%f\t%f\t%f\r\n", time_ms, dTGPS, phfc->AltitudeBaroGPSblend, phfc->altitude_ofs, phfc->altitude_baro, phfc->altitude_gps, phfc->altitude);
            }
        }
      
        /* auto-set home for the first time after GPS is locked, it needs to be locked for at least 15sec */
        if (phfc->home_pos[2] == 99999 && gps.gps_data_.fix>GPS_FIX_NONE && gps.gps_data_.PDOP<200 && phfc->AltitudeBaroGPSblend>25) {
            telem.SetHome();
        }

        phfc->gps_heading  = gps.gps_data_.courseC;
        phfc->gps_speed    = gps.gps_data_.HspeedC;
        //debug_print("GPS s/c %5.1f C %+5.1f  COOR s/c %5.1f C %+5.1f\r\n", gps_speed, gps_heading, phfc->gps_speed, phfc->gps_heading);
      
        /* split GPS speed into east and north components */
        phfc->GPSspeedGroundENU[0] = gps.gps_data_.speedENU[0];
        phfc->GPSspeedGroundENU[1] = gps.gps_data_.speedENU[1];
        phfc->GPSspeedGroundENU[2] = gps.gps_data_.speedENU[2];
        //debug_print("GPS time %d %f %f %f\n", phfc->time_ms, gps.gps_data_.speedENU[0], gps.gps_data_.speedENU[1], gps.gps_data_.speedENU[2]);
    }

    if (!pConfig->servo_raw) {
      ServoUpdate(dT);
    }
    else {
      ServoUpdateRAW(dT);
    }

    SetAgsControls();

    if (phfc->msg2ground_count && !telem.IsTypeInQ(TELEMETRY_MSG2GROUND)) {
        telem.Generate_Msg2Ground();
        telem.AddMessage((unsigned char*)&phfc->telemMsg2ground, sizeof(T_Telem_Msg2Ground), TELEMETRY_MSG2GROUND, 6);
        phfc->msg2ground_count--;
    }

    /* generate a new telemetry system message every 1s or so, only if is not still in the output Q */
    if ((phfc->print_counter&0x3ff)==7 && !telem.IsTypeInQ(TELEMETRY_SYSTEM)) {
        telem.Generate_System2(time_ms);
        telem.AddMessage((unsigned char*)&phfc->telemSystem2, sizeof(T_Telem_System2), TELEMETRY_SYSTEM, 5);
        //debug_print("%d %d\n", phfc->ticks_max, GetTime_ms());
        //perf_printf();
        phfc->ticks_max = 0;
    }
  
    /* TCPIP packet confirmation */
    if (phfc->tcpip_confirm && !telem.IsTypeInQ(TELEMETRY_TCPIP)) {
        telem.Generate_Tcpip7();
        telem.AddMessage((unsigned char*)&phfc->telemTcpip7, sizeof(T_Telem_TCPIP7), TELEMETRY_TCPIP, 4);
        phfc->tcpip_confirm = false;
    }

    /* capture streaming data and generate a new packet once data cache is full */
    if (Streaming_Process(phfc)) {
        /* Push the new data to the output only if the previous packet already
         ** has been sent, since we have only one data cache. Otherwise the current
         ** data cache is dropped, this is not expected to be happening under normal conditions */

        if (!telem.IsTypeInQ(TELEMETRY_DATASTREAM3)) {
            int size = telem.Generate_Streaming();
            telem.AddMessage((unsigned char*)&phfc->telemDataStream3, size, TELEMETRY_DATASTREAM3, 3);
        }
    }
  
    /* if new GPS RMS message arrived and it is not in the serial Q already, generate it and push it to the temetry output */
    if (phfc->gps_new_data) {
        if (!telem.IsTypeInQ(TELEMETRY_GPS)) {
            telem.Generate_GPS1(time_ms);
            telem.AddMessage((unsigned char*)&phfc->telemGPS1, sizeof(T_Telem_GPS1), TELEMETRY_GPS, 2);
        }
    }

    /* generate aircraft config message every 8s or so, only if is not still in the output Q */
    if ((phfc->print_counter&0x1fff)==9 && !telem.IsTypeInQ(TELEMETRY_AIRCRAFT_CFG)) {
        telem.Generate_AircraftCfg();
        telem.AddMessage((unsigned char*)&phfc->aircraftConfig, sizeof(T_AircraftConfig), TELEMETRY_AIRCRAFT_CFG, 1);
    }

    /* if telemetry output Q is empty, generate the Ctrl telemetry message and push it out */
    if (phfc->telem_ctrl_time >= phfc->telem_ctrl_period && telem.IsEmpty()) {
        telem.Generate_Ctrl0(time_ms);
        telem.AddMessage((unsigned char*)&phfc->telemCtrl0, sizeof(T_Telem_Ctrl0), TELEMETRY_CTRL, 0);
        phfc->telem_ctrl_time = 0;
    }

    phfc->telem_ctrl_time += ticks;   // in uS

    telem.Update();

    if (pConfig->AfsiEnabled) {
        afsi.ProcessStatusMessages();
    }

    telem.ProcessInputBytes(telemetry);

    if (pConfig->AfsiEnabled) {
        afsi.ProcessInputBytes(afsi_serial);
    }

    AutoReset();

    CanbusSync();

    phfc->power.dT += dT;

    // Update battery status, if new data available
    if (canbus_livelink_avail || power_update_avail){
        UpdateBatteryStatus(phfc->power.dT);
        phfc->power.dT = 0;
        canbus_livelink_avail = 0;
        power_update_avail = 0;
    }

#if 0
    if (phfc->debug_flags[0] == 1) {
        serial.printf("New playlist command rxed, items[%d]\r\n", phfc->debug_flags[1]);
        phfc->debug_flags[0] = 0;
    }
#endif

    SystemMonitor(dT);

    phfc->gps_new_data = false;
    phfc->print_counter++;
}

// @brief
// @param
// @retval
static void InitFcmIO(void)
{
  // NOTE::SP: HACK OF THE DAY 18-10-2018
  // REGARDLESS OF WHAT SERVO WE ARE USING, ALWAYS ENABLE CHANNEL 6
  // FOR USE WITH ARMED LED
  if (FCM_SERVO_CH6) {
    FCM_SERVO_CH6.period_us(pConfig->pwm_period);
    FCM_SERVO_CH6.pulsewidth_us(1500);
  }

  // NOTE::SP: HACK OF THE DAY 12-09-2018
  // REGARDLESS OF WHAT SERVO WE ARE USING, ALWAYS ENABLE CHANNEL 5
  // FOR USE WITH SPECIAL BOX DROPPER FOR INDRO Demo
  if (FCM_SERVO_CH5) {
    FCM_SERVO_CH5.period_us(pConfig->pwm_period);
    FCM_SERVO_CH5.pulsewidth_us(1500);
  }

  if (pConfig->fcm_servo) {
    FCM_SERVO_CH5.period_us(pConfig->pwm_period);
    FCM_SERVO_CH4.period_us(pConfig->pwm_period);
    FCM_SERVO_CH3.period_us(pConfig->pwm_period);
    FCM_SERVO_CH2.period_us(pConfig->pwm_period);
    FCM_SERVO_CH1.period_us(pConfig->pwm_period);

    FCM_SERVO_CH5.pulsewidth_us(1500);
    FCM_SERVO_CH4.pulsewidth_us(1500);
    FCM_SERVO_CH3.pulsewidth_us(1500);
    FCM_SERVO_CH2.pulsewidth_us(1500);
    FCM_SERVO_CH1.pulsewidth_us(1500);
  }
}

// @brief  ConfigRx callback handler.
// @param  none
// @retval none
static void ConfigRx(void)
{
  while (serial.readable() && (rx_in < MAX_CONFIG_SIZE)) {
    pRamConfigData[rx_in++] = serial._getc();
  }

  have_config = true;
}

// @brief  Process user commands from USB Serial Port.
// @param  received command byte
// @retval none
static void ProcessUserCmnds(char c)
{
    char request[20] = {0};
    ConfigData *pRamConfigData = (ConfigData *)&ram_scratch;

    // 'L' == Load Request
    if (c == 'L') {

        usb_print("OK");

        // Wait for Load Type - config
        serial.scanf("%19s", request);
        if (strcmp(request, "config_upload") == 0) {
            // Clear current RamConfig in preparation for new data.
            memset((uint8_t *)&ram_scratch, 0xFF, MAX_CONFIG_SIZE);

            serial.attach(&ConfigRx);   // This handles incoming configuration file

            have_config = false;

            // Clear chars - Dummy Read on Port
            while (serial.readable()) {
                volatile int rx = serial._getc();
            }

            usb_print("ACK");   // Informs Host to start transfer

            while (!have_config) {}

            unsigned char *pData = (unsigned char *)&ram_scratch;
            pData += sizeof(ConfigurationDataHeader);
            // CRC data and check.
            if (pRamConfigData->header.checksum
                    == crc32b(pData, (sizeof(ConfigData) - sizeof(ConfigurationDataHeader)))) {

                if (SaveNewConfig() == 0) {
                    usb_print("ACK");   // Informs Host, all done

                    // NOTE::SP: NO RETURN FROM HERE. THIS GENERATES A SOFTWARE RESET OF THE LPC1768
                    //           WHICH CAUSES THE NEW CONFIGURATION TO BE LOADED.
                    NVIC_SystemReset();
                }
                else {
                    usb_print("NACK");  // Informs Host, Error
                }
            }
            else {
                usb_print("NACK");  // Informs Host, Error
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
        serial.scanf("%19s", request);
        if (strcmp(request, "compass") == 0) {

            usb_print("\r\n     MAX       MIN       GAIN    OFFSET \r\n");
            usb_print("X    %+3.2f   %+3.2f   %+1.2f   %+3.2f \r\n", phfc->compass_cal.compassMax[0],phfc->compass_cal.compassMin[0],
                                                                     compass->GetGains(0),compass->GetOffsets(0));

            usb_print("Y    %+3.2f   %+3.2f   %+1.2f   %+3.2f \r\n", phfc->compass_cal.compassMax[1],phfc->compass_cal.compassMin[1],
                                                                     compass->GetGains(1),compass->GetOffsets(1));

            usb_print("Z    %+3.2f   %+3.2f   %+1.2f   %+3.2f \r\n", phfc->compass_cal.compassMax[2],phfc->compass_cal.compassMin[2],
                                                                     compass->GetGains(2),compass->GetOffsets(2));
        }
        else {
            usb_print("NACK\r\n");
        }
    }
    else if (c == 'M') {
        // System Manifest
        //IAP iap;
        //int *fcm_serial_num;
        //fcm_serial_num = iap.read_serial();

        usb_print("Type[FCM], Node[%d], Version[%02x:%02x:%02x],  SERIAL[%08x:%08x:%08x:%08x]\r\n",
                            DEFAULT_NODE_ID, MAJOR_VERSION, MINOR_VERSION, BUILD_VERSION,
                            phfc->fcm_serialnum_0, phfc->fcm_serialnum_1, phfc->fcm_serialnum_2, phfc->fcm_serialnum_3);

        usb_print("\r\nCANBus Board Info..\r\n");
        for (int i = 0; i < pConfig->num_servo_nodes; i++) {
            usb_print("Type[ SN], Node[%d], Version[%02x:%02x:%02x], SERIAL[%08x:%08x:%08x]\r\n", i+1,
                        phfc->board_info[AVI_SERVO_NODETYPE][i].major_version, phfc->board_info[AVI_SERVO_NODETYPE][i].minor_version, phfc->board_info[AVI_SERVO_NODETYPE][i].build_version,
                        phfc->board_info[AVI_SERVO_NODETYPE][i].serial_number2, phfc->board_info[AVI_SERVO_NODETYPE][i].serial_number1, phfc->board_info[AVI_SERVO_NODETYPE][i].serial_number0);
        }

        for (int i = 0; i < pConfig->num_gps_nodes; i++) {
            usb_print("Type[GPS], Node[%d], Version[%02x:%02x:%02x], SERIAL[%08x:%08x:%08x]\r\n", i+1,
                        phfc->board_info[AVI_GPS_NODETYPE][i].major_version, phfc->board_info[AVI_GPS_NODETYPE][i].minor_version, phfc->board_info[AVI_GPS_NODETYPE][i].build_version,
                        phfc->board_info[AVI_GPS_NODETYPE][i].serial_number2, phfc->board_info[AVI_GPS_NODETYPE][i].serial_number1, phfc->board_info[AVI_GPS_NODETYPE][i].serial_number0);
        }

        for (int i = 0; i < pConfig->num_power_nodes; i++) {
            usb_print("Type[PWR], Node[%d], Version[%02x:%02x:%02x], SERIAL[%08x:%08x:%08x]\r\n", i+1,
                        phfc->board_info[AVI_PWR_NODETYPE][i].major_version, phfc->board_info[AVI_PWR_NODETYPE][i].minor_version, phfc->board_info[AVI_PWR_NODETYPE][i].build_version,
                        phfc->board_info[AVI_PWR_NODETYPE][i].serial_number2, phfc->board_info[AVI_PWR_NODETYPE][i].serial_number1, phfc->board_info[AVI_PWR_NODETYPE][i].serial_number0);
            usb_print("---V[slope=%f,offset=%f], I[slope=%f,offset=%f]\r\n",phfc->power.Vslope,phfc->power.Voffset,phfc->power.Islope,phfc->power.Ioffset);
        }

        usb_print("TYPE[IMU], ID[%d], YEAR[%d], VARIANT[%d]\r\n", mpu.eeprom->id_num, mpu.eeprom->board_year, mpu.eeprom->board_type);

        mpu.eeprom->print_data();

        usb_print("Recorded Flight Time(s) [%d]\r\n", phfc->OdometerReading/1000);

    }
    else if (c == 'D') {
        serial.scanf("%19s", request);
        if (strcmp(request, "unlock") == 0) {
            if (SetJtag(UNLOCK_JTAG) == 0) {
                usb_print("ACK");
            }
            else {
                usb_print("NACK");
            }
            usb_print("MUST NOW POWER OFF/ON");
            NVIC_SystemReset();
        }
        else if (strcmp(request, "lock") == 0) {
            if (SetJtag(LOCK_JTAG) == 0) {
                usb_print("ACK");
            }
            else {
                usb_print("NACK");
            }
            usb_print("MUST NOW POWER OFF/ON");
            NVIC_SystemReset();
        }
        else if (strcmp(request, "eraseall") == 0) {
            if (EraseFlash() == 0) {
                usb_print("ACK");
            }
            else {
                usb_print("NACK");
            }
            usb_print("MUST NOW POWER OFF/ON");
        }
        else if (strcmp(request, "odoreset") == 0) {
          phfc->OdometerReading = 0;
          if (UpdateOdometerReading(phfc->OdometerReading) == 0) {
            usb_print("ACK");
          }
          else {
            usb_print("NACK");
          }
        }
        else if (strcmp(request, "forcereset") == 0) {
          NVIC_SystemReset(); // force software reset
        }
        else if (strcmp(request, "forcewd") == 0) {
          // loop until watchdog expires
          while(1);
        }
        else if (strcmp(request, "resetreason") == 0) {
          usb_print("0x%08x\r\n", GetResetReason());
        }
        else {
            usb_print("INVALID");
        }

    }
}

//
// @brief  InitCanbus.
// @param
// @retval 0 if success, < 0 on failure
static uint32_t InitCanbus(void)
{
  int error = 0;
  volatile uint32_t ret_mask = 0;
  CANMessage node_cfg_data;

  // Create New Canbus Client
  Canbus = new CAN(CAN_RXD1, CAN_TXD1, (pConfig->canbus_freq_high == 1) ? 1000000 : 500000);
  Canbus->reset();
  Canbus->attach(CanbusISRHandler);

  if (phfc->system_reset_reason & RESET_REASON_WD) {
    return ret_mask;
  }

  for (int i=0; i < 5; i++) {
    KICK_WATCHDOG();
    wait_ms(100);
  }

  // Initialize and configure any servo nodes...
  for (int node_id = BASE_NODE_ID; node_id <= pConfig->num_servo_nodes; node_id++) {
    if ((error = InitCanbusNode(AVI_SERVO_NODETYPE, node_id)) == 0) {
      // CH1 = CASTLE LINK (Auto Enabled on Servo), CH2 = A, CH3 = B, CH4 = C, CH8 = PWM FAN CTRL
      node_cfg_data.data[0] = PWM_CHANNEL_2 | PWM_CHANNEL_3 | PWM_CHANNEL_4 | PWM_CHANNEL_5 | PWM_CHANNEL_8;
      node_cfg_data.data[1] = LIDAR_ACTIVE;
      node_cfg_data.len = 2;
      wait_ms(10);
      error = ConfigureCanbusNode(AVI_SERVO_NODETYPE, node_id, AVI_CFG, &node_cfg_data);

      short int* failsafe;
      int seq_id = AVI_FAILSAFE_0_3;
      for (int i=0; i < 2; i++) {
        failsafe = (short int *)&node_cfg_data.data[0];
        for (int j=0; j < 4; j++) {
          *failsafe++ = pConfig->servo_failsafe_pwm[node_id-1][j+(i*4)];
        }

        node_cfg_data.len = 8;
        wait_ms(10);
        error = ConfigureCanbusNode(AVI_SERVO_NODETYPE, node_id, seq_id, &node_cfg_data);
        seq_id = AVI_FAILSAFE_4_7;
      }
    }

    if (error) {
      ret_mask |= (SERVO_NODE_FAIL << (node_id-1));
    }
  }

  // Initialize and configure any Power nodes...
  for (int node_id = BASE_NODE_ID; node_id <= pConfig->num_power_nodes; node_id++) {
    if ((error = InitCanbusNode(AVI_PWR_NODETYPE, node_id)) == 0) {
      wait_ms(10);
      // Enable all PWM channels and Lidar.
      node_cfg_data.data[0] = PWM_CHANNEL_1_8;
      node_cfg_data.data[1] = LIDAR_ACTIVE;
      node_cfg_data.len = 2;
      error = ConfigureCanbusNode(AVI_PWR_NODETYPE, node_id, AVI_CFG, &node_cfg_data);

      short int* failsafe;
      int seq_id = AVI_FAILSAFE_0_3;
      for (int i=0; i < 2; i++) {
        failsafe = (short int *)&node_cfg_data.data[0];
        for (int j=0; j < 4; j++) {
          *failsafe++ = pConfig->servo_failsafe_pwm[node_id-1][j+(i*4)];
        }

        node_cfg_data.len = 8;
        wait_ms(10);
        error = ConfigureCanbusNode(AVI_PWR_NODETYPE, node_id, seq_id, &node_cfg_data);
        seq_id = AVI_FAILSAFE_4_7;
      }
    }

    if (error) {
      ret_mask |= (PWR_NODE_FAIL << (node_id-1));
    }
  }

  // Initialize and configure any GPS nodes...
  for (int node_id = BASE_NODE_ID; node_id <= pConfig->num_gps_nodes; node_id++) {
    if ((error = InitCanbusNode(AVI_GPS_NODETYPE, node_id)) == 0) {

      // Allow Node to auto select GPS Chip.
      node_cfg_data.data[0] = DEFAULT_GPS_CFG;
      node_cfg_data.len = 1;
      wait_ms(10);
      error = ConfigureCanbusNode(AVI_GPS_NODETYPE, node_id, AVI_CFG, &node_cfg_data);
    }

    if (error) {
      ret_mask |= (NAV_NODE_FAIL << (node_id-1));
    }
  }

  // Enable Canbus Reporting - Broadcast to ALL nodes on system
  KICK_WATCHDOG();
  wait_ms(100);
  ret_mask |= EnableCanbusPDPs();

  return ret_mask;
}

// @brief
// @param
// @retval
static uint32_t InitCanbusNode(int node_type, int node_id, int timeout)
{
  CANMessage can_tx_message;
  uint32_t ret = 0;
  int wait_timeout = timeout;

  // ping the board to determine -
  //  - a) Is the board connected and
  //  - b) Read board information (Firmware Version, Hardware Serial Num)
  can_node_found = 0;

  can_tx_message.len = 0;
  can_tx_message.id = AVI_CAN_ID(node_type, node_id, AVI_HWID_LOW, AVI_MSGID_SDP);
  Canbus->write(can_tx_message);

  wait_ms(20);
  can_tx_message.id = AVI_CAN_ID(node_type, node_id, AVI_HWID_HIGH, AVI_MSGID_SDP);
  Canbus->write(can_tx_message);

  if (wait_timeout) {
    KICK_WATCHDOG();
    while(--wait_timeout) {
      wait_ms(1);
      if(can_node_found) {
        break;
      }
    }

    if (!can_node_found) {
      ret = 1;
    }
  }
  return ret;
}

// @brief
// @param
// @retval
static uint32_t ConfigureCanbusNode(int node_type, int node_id, int seq_id, CANMessage *can_tx_message, int timeout)
{
  uint32_t ret = 0;
  int wait_timeout = timeout;

  // Configure Node. Board will 'ACK' message to indicate success.
  canbus_ack = 0;

  can_tx_message->id = AVI_CAN_ID(node_type, node_id, seq_id, AVI_MSGID_SDP);
  Canbus->write(*can_tx_message);

  if (wait_timeout) {
    KICK_WATCHDOG();

    while(--wait_timeout) {
      wait_ms(1);
      if(canbus_ack) {
        break;
      }
    }

    if (!canbus_ack) {
      ret = 1;
    }
  }
  return ret;
}

// @brief
// @param
// @retval
static uint32_t EnableCanbusPDPs(void)
{
  CANMessage can_tx_message;
  uint32_t ret_mask = 0;

  can_tx_message.len = 0;

  for (int node_id = BASE_NODE_ID; node_id <= pConfig->num_servo_nodes; node_id++) {
    can_tx_message.id = AVI_CAN_ID(AVI_SERVO_NODETYPE, node_id, AVI_PDP_ON, AVI_MSGID_CTRL);
    if (Canbus->write(can_tx_message) == 0) {
      ret_mask |= (SERVO_NODE_FAIL << (node_id-1));
    }
  }

  for (int node_id = BASE_NODE_ID; node_id <= pConfig->num_power_nodes; node_id++) {
    can_tx_message.id = AVI_CAN_ID(AVI_PWR_NODETYPE, node_id, AVI_PDP_ON, AVI_MSGID_CTRL);
    if (Canbus->write(can_tx_message) == 0) {
      ret_mask |= (PWR_NODE_FAIL << (node_id-1));
    }
  }

  for (int node_id = BASE_NODE_ID; node_id <= pConfig->num_gps_nodes; node_id++) {
    can_tx_message.id = AVI_CAN_ID(AVI_GPS_NODETYPE, node_id, AVI_PDP_ON, AVI_MSGID_CTRL);
    if (Canbus->write(can_tx_message) == 0) {
      ret_mask |= (NAV_NODE_FAIL << (node_id-1));
    }
  }

  return ret_mask;
}

//
// - Send a Canbus Sync(Heartbeat) request to GPS
//   Servo and Pwr Nodes dont require this since we send
//   servo data to them at a 1ms interval (which acts as the heartbeat)
//
static void CanbusSync(void)
{
  CANMessage can_tx_message;

  can_tx_message.len = 0;

  // Send Sync heartbeat to GPS nodes
  for (int node_id = BASE_NODE_ID; node_id <= pConfig->num_gps_nodes; node_id++) {
    can_tx_message.id = AVI_CAN_ID(AVI_GPS_NODETYPE, node_id, AVI_SYNC, AVI_MSGID_CTRL);
    Canbus->write(can_tx_message);
  }
}

// @brief - Send a Canbus Sync(Heartbeat) request to GPS
//          Servo and Pwr Nodes dont require this since we send
//          servo data to them at a 1ms interval (which acts as the heartbeat)
// @param
// @retval
static void CanbusFailsafe(int node_type, int node_id)
{
  CANMessage can_tx_message;

  can_tx_message.len = 0;

  can_tx_message.id = AVI_CAN_ID(node_type, BASE_NODE_ID + node_id, AVI_FAILSAFE, AVI_MSGID_CTRL);
  phfc->failsafe = true;

  Canbus->write(can_tx_message);
}

// @brief
// @param
// @retval
static void InitializeRuntimeData(void)
{
  uint32_t last_reset = GetResetReason();

  // If we are warm resetting, DO NOT re-init data. We are trying to
  // keep running under a warm reset.
  if (last_reset & RESET_REASON_WD) {
    phfc->system_reset_reason = last_reset;
    phfc->system_status_mask = 0;
    phfc->soft_reset_counter++;

    // initialize incoming command to None, to ensure we don't process an old
    // command when soft resetting.
    phfc->command.command = TELEM_CMD_NONE;
    return;
  }

  // POWER ON RESET - Ensure Runtime data is Initialized.
  KICK_WATCHDOG();
  memset(phfc, 0x00, sizeof(FlightControlData));

  phfc->system_reset_reason = last_reset;
  phfc->soft_reset_counter = 0;

	// Setup FCM's serial number.
  int *fcm_serial_num;
  IAP iap;
  fcm_serial_num = iap.read_serial();
  phfc->fcm_serialnum_0 = fcm_serial_num[0];
  phfc->fcm_serialnum_1 = fcm_serial_num[1];
  phfc->fcm_serialnum_2 = fcm_serial_num[2];
  phfc->fcm_serialnum_3 = fcm_serial_num[3];

  phfc->PRstick_rate  = pConfig->PRstickRate / pConfig->Stick100range;
  phfc->PRstick_angle = pConfig->PRstickAngle /pConfig->Stick100range;
  phfc->YawStick_rate = pConfig->YawStickRate / pConfig->Stick100range;
  phfc->Stick_Vspeed  = pConfig->StickVspeed / pConfig->Stick100range;
  phfc->Stick_Hspeed  = pConfig->StickHspeed / pConfig->Stick100range;

  phfc->num_lidars = (pConfig->ccpm_type == CCPM_TANDEM) ? 2 : 1; // TODO::SP - this should come from configuration

  // convert dead band values in % to servo range
  for (int i = 0; i < 4; i++) {
    phfc->StickDeadband[i] = pConfig->stick_deadband[i] * 0.01f * pConfig->Stick100range;
  }

  PID_Init(&phfc->pid_PitchRate,  pConfig->pitchrate_pid_params,  0, 1);
  PID_Init(&phfc->pid_RollRate,   pConfig->rollrate_pid_params,   0, 1);
  PID_Init(&phfc->pid_YawRate,    pConfig->yawrate_pid_params,    0, 1);
  PID_Init(&phfc->pid_PitchAngle, pConfig->pitchangle_pid_params, 1, 0);
  PID_Init(&phfc->pid_RollAngle,  pConfig->rollangle_pid_params,  1, 0);
  PID_Init(&phfc->pid_CollVspeed, pConfig->collvspeed_pid_params, 0, 0);
  PID_Init(&phfc->pid_PitchSpeed, pConfig->pitchspeed_pid_params, 0, 0);
  PID_Init(&phfc->pid_RollSpeed,  pConfig->rollspeed_pid_params,  0, 0);

  PID_Init(&phfc->pid_IMU[0],     pConfig->imu_pid_params, 1, 0);
  PID_Init(&phfc->pid_IMU[1],     pConfig->imu_pid_params, 1, 0);
  PID_Init(&phfc->pid_IMU[2],     pConfig->imu_yaw_pid_params, 1, 0);

  PID_P_Acc_Init(&phfc->pid_YawAngle,    pConfig->yawangle_pid_params,    1, true); // enable deceleration
  PID_P_Acc_Init(&phfc->pid_CollAlt,     pConfig->collalt_pid_params,     0, true); // same acc and dec
  PID_P_Acc_Init(&phfc->pid_Dist2T,      pConfig->dist2T_pid_params,      0, true);
  PID_P_Acc_Init(&phfc->pid_Dist2P,      pConfig->dist2P_pid_params,      0, false);
  PID_P_Acc_Init(&phfc->pid_PitchCruise, pConfig->pitchCruise_pid_params, 0, false);

  phfc->speed_Iterm_E     = 0;
  phfc->speed_Iterm_N     = 0;
  phfc->speed_Iterm_E_lp  = 0;
  phfc->speed_Iterm_N_lp  = 0;

  // save default values for playlist mode, duplicated and used within phfc->
  //   - These used to be in cfg, but this is now READ only
  phfc->rw_cfg.VspeedMax = phfc->pid_CollAlt.COmax;
  phfc->rw_cfg.VspeedMin = phfc->pid_CollAlt.COmin;
  phfc->rw_cfg.VspeedAcc = phfc->pid_CollAlt.acceleration;
  phfc->rw_cfg.HspeedMax = phfc->pid_Dist2T.COmax;
  phfc->rw_cfg.HspeedAcc = phfc->pid_Dist2T.acceleration;

  // initialize sensor's low pass filters
  for (int i=0; i < 3; i++) {
    phfc->calib_gyro_avg[i]  = 0;
  }

  LP4_Init(&phfc->lp_gyro4[PITCH], pConfig->gyro_lp_freq[PITCH]);
  LP4_Init(&phfc->lp_gyro4[ROLL], pConfig->gyro_lp_freq[ROLL]);
  LP4_Init(&phfc->lp_gyro4[YAW], pConfig->gyro_lp_freq[YAW]);

  for (int i=0; i < 3; i++) {
      LP4_Init(&phfc->lp_acc4[i], pConfig->acc_lp_freq);
  }

  LP4_Init(&phfc->lp_baro4, pConfig->baro_lp_freq);
  LP4_Init(&phfc->lp_baro_vspeed4, pConfig->baro_vspeed_lp_freq);

  phfc->Pos_GPS_IMU_Blend = pConfig->Pos_GPS_IMU_BlendReg;
  phfc->telem_ctrl_period = Max(phfc->telem_ctrl_period, (pConfig->telem_min_ctrl_period * 1000));

  phfc->throttle_value   = -pConfig->Stick100range;
  phfc->collective_value = -pConfig->Stick100range;

  phfc->ctrl_collective_raw = pConfig->CollZeroAngle;    // set to current position
  phfc->ctrl_collective_3d  = pConfig->CollZeroAngle;   // target

  //Give a 10 percent (up to 1000mAh) buffer on the battery capacity
  phfc->power.capacity_total = ((pConfig->battery_capacity-min(pConfig->battery_capacity*0.1,1000)) / 1000.0f * 3600); // As
  phfc->power.energy_total   = (phfc->power.capacity_total * pConfig->battery_cells * 3.7f);  // Ws

  phfc->dyn_yaw_rate = pConfig->default_dyn_yaw_rate;
  phfc->ctrl_source = pConfig->default_ctrl_source;
  phfc->acc_dyn_turns = pConfig->default_acc_dyn_turns;

  for (int i = 0; i < 3; i++) {
    phfc->home_pos[i] = pConfig->default_home_position[i];
  }

  phfc->orient_reset_counter = pConfig->orient_reset_counter;

  phfc->takeoff_height = pConfig->takeoff_height;
  phfc->takeoff_vertical_speed = pConfig->takeoff_vertical_speed;

  phfc->controlStatus = CONTROL_STATUS_PREFLIGHT;

  // NOTE:SP: This is data which is updated at runtime to a duplicated
  // Read/Write area.
  phfc->rw_cfg.GTWP_retire_radius = pConfig->GTWP_retire_radius;
  phfc->rw_cfg.GTWP_retire_speed = pConfig->GTWP_retire_speed;
  phfc->rw_cfg.FTWP_retire_sr_factor = pConfig->FTWP_retire_sr_factor;
  phfc->rw_cfg.low_speed_limit = pConfig->low_speed_limit;
  phfc->rw_cfg.PRstickRate = pConfig->PRstickRate;
  phfc->rw_cfg.PRstickAngle = pConfig->PRstickAngle;
  phfc->rw_cfg.YawStickRate = pConfig->YawStickRate;
  phfc->rw_cfg.StickVspeed = pConfig->StickVspeed;
  phfc->rw_cfg.StickHspeed = pConfig->StickHspeed;
  phfc->rw_cfg.StickHaccel = pConfig->StickHaccel;
  phfc->rw_cfg.RollPitchAngle = pConfig->RollPitchAngle;
  phfc->rw_cfg.wind_compensation = pConfig->wind_compensation;
  phfc->rw_cfg.path_navigation = pConfig->path_navigation;
  phfc->rw_cfg.ManualLidarAltitude = pConfig->ManualLidarAltitude;
  phfc->rw_cfg.AngleCollMixing = pConfig->AngleCollMixing;
  phfc->rw_cfg.cruise_speed_limit = pConfig->cruise_speed_limit;
  phfc->rw_cfg.nose_to_WP = pConfig->nose_to_WP;
  phfc->rw_cfg.landing_wind_threshold = pConfig->landing_wind_threshold;
  phfc->rw_cfg.battery_capacity = pConfig->battery_capacity;
  phfc->rw_cfg.WindTableScale = pConfig->WindTableScale;
  phfc->rw_cfg.elevator_gain = pConfig->elevator_gain;
  phfc->rw_cfg.dcp_gain = pConfig->dcp_gain;
  phfc->rw_cfg.throttle_offset = pConfig->throttle_offset;

  for (int i=0; i < 3; i++) {
    phfc->rw_cfg.AccIntegGains[i] = pConfig->AccIntegGains[i];
  }

  phfc->rw_cfg.AltitudeBaroGPSblend_final = pConfig->AltitudeBaroGPSblend_final;
  phfc->rw_cfg.Pos_GPS_IMU_BlendGlitch = pConfig->Pos_GPS_IMU_BlendGlitch;
  phfc->rw_cfg.Pos_GPS_IMU_BlendReg = pConfig->Pos_GPS_IMU_BlendReg;
  phfc->rw_cfg.BaroVspeedWeight = pConfig->BaroVspeedWeight;
  phfc->rw_cfg.BaroAltitudeWeight = pConfig->BaroAltitudeWeight;
  phfc->rw_cfg.GPSVspeedWeight = pConfig->GPSVspeedWeight;
  phfc->rw_cfg.gps_vspeed = pConfig->gps_vspeed;

  for (int i=0; i < 3; i++) {
    phfc->rw_cfg.TurnAccParams[i] = pConfig->TurnAccParams[i];
  }

  phfc->rw_cfg.joystick_max_speed = pConfig->joystick_max_speed;

  phfc->command.command = TELEM_CMD_NONE;
  phfc->rc_ctrl_request = false;
  phfc->playlist_status = PLAYLIST_NONE;
  phfc->control_mode[PITCH] = CTRL_MODE_ANGLE;
  phfc->control_mode[ROLL] = CTRL_MODE_ANGLE;
  phfc->control_mode[YAW] = CTRL_MODE_ANGLE;
  phfc->control_mode[COLL] = CTRL_MODE_MANUAL;
  phfc->control_mode[THRO] = CTRL_MODE_MANUAL;
  phfc->waypoint_type = WAYPOINT_NONE;
  phfc->btnMenuPrev = true;
  phfc->btnSelectPrev = true;
  phfc->AltitudeBaroGPSblend = ALTITUDE_BARO_GPS_BLEND_FREQ_INIT;
  phfc->baro_altitude_raw_lp = -9999;
  phfc->esc_temp = 20;

  for (int i=0; i < MAX_NUM_LIDARS; i++) {
    phfc->altitude_lidar_raw[i] = 0;
  }

  phfc->distance2WP_min = 999999;

  phfc->comp_calibrate = NO_COMP_CALIBRATE;

    phfc->box_dropper_ = pConfig->aux_pwm_default;

  // If there is a valid compass calibration, load it.
  // otherwise use defaults.
  const CompassCalibrationData *pCompass_cal = NULL;

  if (LoadCompassCalibration(&pCompass_cal) == 0) {
    KICK_WATCHDOG();
    memcpy(&phfc->compass_cal, pCompass_cal, sizeof(CompassCalibrationData));
  }
  else {
    // no valid calibration, use defaults
    phfc->system_status_mask |= COMPASS_CAL_WARN;

    for (int i = 0; i < 3; i++) {
      phfc->compass_cal.comp_ofs[i] = 0;
    }

  for (int i = 0; i < 3; i++) {
      phfc->compass_cal.compassMin[i] = 9999;
    }

    for (int i = 0; i < 3; i++) {
      phfc->compass_cal.compassMax[i] = -9999;
    }
  }

  phfc->delay_time = -1;
  phfc->box_dropper_ = 0;

  phfc->enable_lidar_ctrl_mode = false; // TODO::SP - Initialize from pConfig when item available

  phfc->servo_reverse_mask = 0;
  for (int i=0; i < MAX_SERVO_OUTPUTS; i++) {
    if ( i < 6 ) {
      phfc->servo_reverse_mask |= pConfig->servo_revert[i] << i;
    }
    else {
      phfc->servo_reverse_mask |= pConfig->servo_revert_ch7_ch8[i-6] << i;
    }
  }
  phfc->eng_super_user = false;

  InitializeOdometer(phfc);

  phfc->positive_pid_scaling = pConfig->dynamic_pid_rc_max_gain / 15.0f;
  phfc->negative_pid_scaling = pConfig->dynamic_pid_rc_min_gain / 14.0f;
  phfc->pid_PitchRateScalingFactor = pConfig->dynamic_pid_speed_gain;

  phfc->enable_lidar_ctrl_mode = false; // TODO::SP - Initialize from pConfig when item available

  phfc->num_motors = 0;

  phfc->imu_error_count = 0;
  phfc->baro_error_count = 0;
  phfc->failsafe = false;

  phfc->rw_cfg.max_cruise_pitch_trim = pConfig->max_cruise_pitch_trim;

  GenerateSpeed2AngleLUT();

  LPC_RIT->RICOUNTER = 0;

  phfc->max_cruise_angle = 2.0f;
  phfc->min_added_cruise_anlge = 1.0f;
}

// @brief
// @param
// @retval
static uint32_t InitializeSystemData()
{
  uint32_t ret_value = 0;

  // Clear out the Runtime RAM copy of the config Data
  KICK_WATCHDOG();
  memset(pRamConfigData, 0x00, MAX_CONFIG_SIZE);

  // If we fail to load configuration, don't attempt to continue to setup
  // runtime data, since we will likely fw crash due missing data.
  // Instead, return and let higer layers handle it.
  if (LoadConfiguration(&pConfig) < 0) {
    return CONFIG_FAIL;
  }

  // Initialize Runtime Data
  InitializeRuntimeData();

  // Initialize Lidar Filter
  for (int i=0; i < phfc->num_lidars; i++) {
    lidar_median[i] = MediatorNew(35);
  }

  // Initialize Gps class data
  gps.Init(pConfig->num_gps_nodes);

  // Instantiate compass
  compass = new Compass(pConfig, phfc->compass_cal.comp_ofs, phfc->compass_cal.comp_gains);

  // Instantiate mixer to be used, based upon configuration
  switch(pConfig->ccpm_type) {
    case CCPM_120:
      mixer = new CCPM120Mixer();
    break;
    case CCPM_140:
      mixer = new CCPM140Mixer();
    break;
    case CCPM_QUAD:
      mixer = new QuadMixer();
      phfc->num_motors = 4;
    break;
    case CCPM_HEX:
      mixer = new HexMixer();
      phfc->num_motors = 6;
    break;
    case CCPM_OCTO:
      mixer = new OctoMixer();
      phfc->num_motors = 8;
    break;
    case CCPM_TANDEM:
      tandem_mixer = new AviTandemMixer(pConfig);
      break;
    default:
    break;
  }
  return ret_value;
}

// @brief No return from here, runs minimal system with telem
//        to provide AGS info that boot failed due to config error
//        Serial USB connected to enable config updates to be processed
//        THIS IS A LAST RESORT PROCESS AND SHOULD NEVER RUN!
// @param
// @retval
static void RunDefaultSystem(void)
{
  int utilization = 0;
  int count = 0;

  SysTick_Run();

  telem.Initialize(phfc, NULL);
  telemetry.baud(38400);

  while (1) {

    KICK_WATCHDOG();

    Ticks_us_minT(1000, &utilization);

    // Generate default aircraft cfg message to AGS every 1 seconds
    if (((++count % 1000) == 0) && !telem.IsTypeInQ(TELEMETRY_AIRCRAFT_CFG)) {
      telem.Generate_DefaultCfg();
      telem.AddMessage((unsigned char*)&phfc->aircraftConfig, sizeof(T_AircraftConfig), TELEMETRY_AIRCRAFT_CFG, 0);
    }

    telem.Update();

    // Process Serial commands if USB is available
    if (serial.connected() && serial.readable()) {
      ProcessUserCmnds(serial.getc());
    }

  }
}

// @brief
// @param
// @retval
int main()
{

#if defined (CRP_LOCK)
  SetJtag(LOCK_JTAG);
#endif

  SetFcmLedState(0);

  SysTick_Run();

#ifndef DEBUG
  InitializeWatchdog(WATCHDOG_TIMEOUT);
#endif

  // Early detect if we have exceeded build limits
  if (sizeof(FlightControlData) > MAX_HFC_SIZE) {
    phfc->system_status_mask |= HFC_RAM_WARN;
  }

  if (sizeof(ConfigData) > MAX_CONFIG_SIZE) {
    phfc->system_status_mask |= CFG_RAM_WARN;
  }

  // Initialize system configuration, Runtime Data and Peripherals
  phfc->system_status_mask |= InitializeSystemData();

  if (phfc->system_status_mask & CONFIG_FAIL) {
    // If config did not load - go direct to Jail and do not
    // Pass Go and Do not collect $200
    // NO RETURN FROM THIS CALL
    RunDefaultSystem();
  }

  // If we've experienced a snowball of watchdog resets
  // all we can do is assume something very bad is happening and attempt to
  // put any control systems to failsafe mode.
  if (phfc->soft_reset_counter >= WD_RESET_LIMIT) {
    FailSafeMode();
  }

  phfc->system_status_mask |= InitCanbus();

  phfc->system_status_mask |= mpu.init(pConfig, MPU6050_DLPF_BW_188, &phfc->imu_serial_num);

  phfc->system_status_mask |= baro.Init(pConfig);

  // need to recall this to ensure RICOUNTER is re-initialized before commencing control loop
  SysTick_Run();

  SetFcmLedState(phfc->system_reset_reason);

  LedTesterOff(); // This is the old lcd led

  // Initialize FCM local IO
  InitFcmIO();

  // Initialize RC and Telemetry channels
  xbus.ConfigRx(pConfig);
  telem.Initialize(phfc, pConfig);
  telemetry.baud(pConfig->telem_baudrate);
  telem.Generate_AircraftCfg();

  // Start IMU read
  mpu.readMotion7_start();

  // Loop Forever, processing control
  while(1) {

    DoFlightControl();

    // Process Serial commands if USB is available
    if (serial.connected() && serial.readable()) {
      ProcessUserCmnds(serial.getc());
    }
  }
}

/************************ (C) COPYRIGHT Avidrone Aerospace Inc. *****END OF FILE****/
