#include "config.h"
#include "utils.h"
#include "HMC5883L.h"
#include "defines.h"
#include "IMU.h"
#include "telemetry.h"
#include "NOKIA_5110.h"

#define MIN_CONFIG_VERSION      8

extern HMC5883L compass;

static void InitPIDparams(float pid_params[6], float Kc, float Ki, float Kd, float Ofs, float Max, float Min)
{
    pid_params[0] = Kc;
    pid_params[1] = Ki;
    pid_params[2] = Kd;
    pid_params[3] = Ofs;
    pid_params[4] = Max;
    pid_params[5] = Min;
}

static void InitPIDparams5(float pid_params[5], float Kc, float Kd, float Max, float Min, float Acc)
{
    pid_params[0] = Kc;
    pid_params[1] = Kd;
    pid_params[2] = Max;
    pid_params[3] = Min;
    pid_params[4] = Acc;
}

void Config_SetDefaults(T_HFC *hfc)
{
    T_Config *cfg = &hfc->config;
    int i,j;
    
    cfg->config_version      =        0;

    cfg->num_servo_nodes     =        0;
    cfg->num_gps_nodes       =        0;
    cfg->servo_raw           =        0;
    cfg->canbus_freq_high    =        1;
    cfg->sensor_mode         =        1;
    cfg->compass_type        =        0;

    cfg->PRstickRate         =        40;      // 40deg/s at full stick
    cfg->PRstickAngle        =        30;      // 30 deg at full stick
    cfg->YawStickRate        =        100;     // 100 deg/s at full stick
    cfg->StickVspeed         =        2;       // 2m/s at full stick
    cfg->StickHspeed         =        5;      // 10m/s at full stick
    cfg->StickHaccel         =        2;
    cfg->AngleCollMixing     =        0;
    cfg->Stick100range       =        0.571f;  // xbus value for a full stick
    cfg->VspeedMax           =        10;       // maximum vertical speed, absolute limit, up
    cfg->VspeedMin           =       -5;        // minimum vertical speed, absolute limit, down
    cfg->CollZeroAngle       =        0.1f;     // collective servo value for 0 angle
    cfg->CollAngleAutoRotate =		  -0.1f;	// collective value for auto-rotate
    cfg->CollThrAutoLevel    =        0.75f;
    cfg->low_speed_limit     =        1;        // tails follows ground speed vector above 3m/s
    cfg->RCmodeSwitchOfs	 = 		  1;		// rate/angle/speed by default
    cfg->YawModeMin          =        2;        // 1-man, 2-rate, 3-angle
    cfg->YawModeMax          =        2;        // 1-man, 2-rate, 3-angle
    cfg->dyn_yaw_rate_max	 =        50;
    cfg->AllowArmInManual    =        0;
    cfg->ManualLidarAltitude =        0;
    cfg->LidarFromServo		 =		  0;
    cfg->wind_compensation   =        1;
    cfg->path_navigation     =        1;
    cfg->nose_to_WP          =        1;
    cfg->autoReset			 =		  1;   // automatically reset the IMU when stationary
    cfg->can_servo           =        0;
    cfg->fcm_servo           =        1;
    cfg->power_node			 = 		  0;
    cfg->rpm_sensor          =        0;
    cfg->linklive            =        1;
    cfg->gps_vspeed          =        0;
    cfg->gps_units           =        2;
    cfg->ground_sensor       =        GROUND_SENSOR_NONE;
    cfg->collective_man_speed=        0.1;  // 0.1 servo value per second
    cfg->takeoff_angle_rate	 =        2;
    cfg->landing_vspeed		 =		  0.4f;
    cfg->landing_vspeed_acc  = 		  0.4f;
    cfg->landing_appr_speed	 =		  5;
    cfg->VspeedDownCurve[0]  =		  -0.33f;
    cfg->VspeedDownCurve[1]  =		  -5;
    cfg->LidarHVcurve[0]     = 2;       // min speed
    cfg->LidarHVcurve[1]     = 0.666f;  // slope
    cfg->LidarHVcurve[2]     = 2;       // min alt
    cfg->LidarHVcurve[3]     = 20;      // max alt
    cfg->TurnAccParams[0]    = 5;           // max acc
    cfg->TurnAccParams[1]    = 10;        // max speed
    cfg->TurnAccParams[2]    = 0.2;       // minBland
    cfg->cruise_speed_limit  = 6;       // m/s
    cfg->landing_wind_threshold = 5;    // m/s
    cfg->joystick_max_speed     = 10;   // m/s
    
    for (i=0; i<5; i++) cfg->ctrl_mode_inhibit[i] = 0;
    cfg->GTWP_retire_radius     = 1.5f; // 1.5m - radius for retiring GTWP
    cfg->GTWP_retire_speed      = 0.75f;// 0.75m/s - speed for retiring GTWP
    cfg->FTWP_retire_sr_factor  = 0.5f; // 5m at 10m/s speed-adjusted radius factor for retiring FTWP  radius = speed*FTWP_retire_sr_factor

    /*mmri:
     * - default acc_gains and gyro_gains to 0
     * - default acc_calib_matrix and gyro_calib_matrix to identity matrix
     *   identiy matrix = { {1,0,0}, {0,1,0}, {0,0,1} }
     * - defaults are chosen when not available in config file
     * This is necessary for the code to identify which of the variables,
     * gains or calib_matrix, are defined in the config file*/
    for(i=0; i<3; i++)
    {
    	cfg->acc_gains[i]  = 0;
    	cfg->gyro_gains[i] = 0;
    	cfg->comp_gains[i] = 0;
    	for(j = 0; j<3; j++)
    	{
    		if( i == j )
    		{
    			cfg->acc_calib_matrix[i][j] = 1;
    			cfg->gyro_calib_matrix[i][j] = 1;
    			cfg->comp_calib_matrix[i][j] = 1;
    		}
    		else
    		{
    			cfg->acc_calib_matrix[i][j] = 0;
    			cfg->gyro_calib_matrix[i][j] = 0;
    			cfg->comp_calib_matrix[i][j] = 0;
    		}
    	}
    }

    cfg->servo_speed[0] = 100;
    cfg->servo_speed[1] = 100;

    cfg->pwm_period = 10000; // 3333
    cfg->telem_baudrate = 38400;

    /* Kc, Ti, Td, Ofs, Max, Min, Decay */
    InitPIDparams(cfg->pitchrate_pid_params, 0.001122979f, 0.03f,  0, 0,      0.55f, -0.55f);
    InitPIDparams(cfg->rollrate_pid_params,  0.001122979f, 0.03f,  0, 0.032f, 0.55f, -0.55f);
    InitPIDparams(cfg->pitchangle_pid_params,5.166666667f, 0.7f,   0, 0,      40,    -40);
    InitPIDparams(cfg->rollangle_pid_params, 5.166666667f, 0.7f,   0, 0,      40,    -40);
    InitPIDparams(cfg->yawrate_pid_params,   0.00786953f,  0.959f, 0, 0.112f, 0.55f, -0.55f);
    InitPIDparams(cfg->collvspeed_pid_params,0.031745192f, 0.279f, 0, 0.159f, 0.55f, -0.55f);
    InitPIDparams(cfg->pitchspeed_pid_params,-4.75857538f, 5.467f, 0, 0     , 30,    -30);
    InitPIDparams(cfg->rollspeed_pid_params, 5.45733828f,  4.767f, 0, 0     , 30,    -30);
    InitPIDparams5(cfg->yawangle_pid_params, 2.0f,         0, 120, -120, 180);
    InitPIDparams5(cfg->collalt_pid_params,  0.829410087f, 0,   2,   -2,   1);
    InitPIDparams5(cfg->dist2T_pid_params,   0.271397992f, 0,   5,    0,   1);
    InitPIDparams5(cfg->dist2P_pid_params,   0.271397992f, 0,   2,    0,   1);
    InitPIDparams5(cfg->pitchCruise_pid_params,  0,        0,  40,  -40,   5);
    
    cfg->gyro_ofs[0] = 0;
    cfg->gyro_ofs[1] = 0;
    cfg->gyro_ofs[2] = 0;
    cfg->lidar_offset      = 426;

    cfg->IMUaccGyroBlend      = IMU_ACC_GYRO_FREQ;
    cfg->AltitudeBaroGPSblend_final = ALTITUDE_BARO_GPS_BLEND_FREQ;
    cfg->Pos_GPS_IMU_BlendReg       = POS_GPS_IMU_BLEND_REG;
    cfg->Pos_GPS_IMU_BlendGlitch    = POS_GPS_IMU_BLEND_GLITCH;

    cfg->RollPitchAngle  = 0;       // rotates the impact of P/R controls to P/R servos, clockwise in deg

    // inverts individual servos
    cfg->servo_revert[0] = 1;   //P
    cfg->servo_revert[1] = 0;  // R
    cfg->servo_revert[2] = 1;  // Y
    cfg->servo_revert[3] = 0;  // C
    cfg->servo_revert[4] = 0;  // T
    cfg->servo_revert[5] = 0;  // N/A
    // gains on individual control channels
    cfg->control_gains[0] = -1; // P
    cfg->control_gains[1] =  1; // R
    cfg->control_gains[2] =  1; // Y
    cfg->control_gains[3] =  0.5f; // C
    cfg->throttle_ctrl = 0;   // throttle control mode 0-linear, 1-constant
    // values for throttle switch mode
    cfg->throttle_values[0] = -0.685f;  // value when stick is low
    cfg->throttle_values[1] = 0.5f;     // throttle gain
    cfg->throttle_gain = 0.999f ;
    cfg->throttle_multi_min = -0.6f;    // (-0.6 * 500) + 1500 = 1200us  (DJI turns on at 1106)

    cfg->ccpm_type = CCPM_NONE;
    cfg->SbusEnable = 0;
    cfg->acc_lp_freq  = 15;
    cfg->gyro_lp_freq[0] = 20;
    cfg->gyro_lp_freq[1] = 20;
    cfg->gyro_lp_freq[2] = 20;
    cfg->AccIntegGains[0] = 1.1f;
    cfg->AccIntegGains[0] = 1.1f;
    cfg->AccIntegGains[0] = 1.05f;

    cfg->baro_lp_freq = 20;
    cfg->BaroVspeedWeight = 0.4f;
    cfg->GPSVspeedWeight = 0.4f;
    cfg->BaroAltitudeWeight = 0.25f;
    cfg->baro_vspeed_lp_freq = 20;
    cfg->heading_avgs = 8;


    cfg->landing_timeout = 5000;    // 5s
    cfg->telem_min_ctrl_period = 0;	// max telem period in ms

    cfg->battery_cells      = 4;
    cfg->battery_capacity   = 5000; // 5Ah
    cfg->power_typical      = 352;  // W
    cfg->rpm_typical        = 0;

    cfg->gear_ratio     = 9.07;       // motor to main shaft ratio
    cfg->motor_poles    = 6;          // motor poles

    for (i=0; i<4; i++) cfg->stick_deadband[i] = 0;
    
    for (i=0; i<3; i++) cfg->gyro_drift_coeffs[0][i] = 0;
    for (i=0; i<3; i++) cfg->gyro_drift_coeffs[0][i] = 0;
    for (i=0; i<3; i++) cfg->gyro_drift_coeffs[0][i] = 0;
    cfg->gyro_fixed_offsets = 0;
    /* default angle to speed table for wind measurements */
    cfg->WindTableScale = 1;
    cfg->WindSpeedLUT[0] = 0;
    cfg->WindSpeedLUT[1] = 0.196f;
    cfg->WindSpeedLUT[2] = 0.424f;
    cfg->WindSpeedLUT[3] = 0.684f;
    cfg->WindSpeedLUT[4] = 0.976f;
    cfg->WindSpeedLUT[5] = 1.3f;
    cfg->WindSpeedLUT[6] = 1.656f;
    cfg->WindSpeedLUT[7] = 2.044f;
    cfg->WindSpeedLUT[8] = 2.464f;
    cfg->WindSpeedLUT[9] = 2.916f;
    cfg->WindSpeedLUT[10] = 3.4f;
    cfg->WindSpeedLUT[11] = 3.916f;
    cfg->WindSpeedLUT[12] = 4.464f;
    cfg->WindSpeedLUT[13] = 5.044f;
    cfg->WindSpeedLUT[14] = 5.656f;
    cfg->WindSpeedLUT[15] = 6.3f;
    cfg->WindSpeedLUT[16] = 6.976f;
    cfg->WindSpeedLUT[17] = 7.684f;
    cfg->WindSpeedLUT[18] = 8.424f;
    cfg->WindSpeedLUT[19] = 9.196f;
    cfg->WindSpeedLUT[20] = 10;
    cfg->WindSpeedLUT[21] = 10.836f;
    cfg->WindSpeedLUT[22] = 11.704f;
    cfg->WindSpeedLUT[23] = 12.604f;
    cfg->WindSpeedLUT[24] = 13.536f;
    cfg->WindSpeedLUT[25] = 14.5f;
    cfg->WindSpeedLUT[26] = 15.496f;
    cfg->WindSpeedLUT[27] = 16.524f;
    cfg->WindSpeedLUT[28] = 17.584f;
    cfg->WindSpeedLUT[29] = 18.676f;
    cfg->WindSpeedLUT[30] = 19.8f;
    cfg->WindSpeedLUT[31] = 20.956f;
    cfg->WindSpeedLUT[32] = 22.144f;
    cfg->WindSpeedLUT[33] = 23.364f;
    cfg->WindSpeedLUT[34] = 24.616f;
    cfg->WindSpeedLUT[35] = 25.9f;
    cfg->WindSpeedLUT[36] = 27.216f;
    cfg->WindSpeedLUT[37] = 28.564f;
    cfg->WindSpeedLUT[38] = 29.944f;
    cfg->WindSpeedLUT[39] = 31.356f;
    cfg->WindSpeedLUT[40] = 32.8f;
    cfg->WindSpeedLUT[41] = 34.276f;
    cfg->WindSpeedLUT[42] = 35.784f;
    cfg->WindSpeedLUT[43] = 37.324f;
    cfg->WindSpeedLUT[44] = 38.896f;
    cfg->WindSpeedLUT[45] = 40.5f;

    for (i=0; i<V2ENERGY_SIZE; i++)
        cfg->V2Energy[i] = ((float)i)/(V2ENERGY_SIZE-1);

    /*************************************************************************************/

    hfc->AltitudeBaroGPSblend = ALTITUDE_BARO_GPS_BLEND_FREQ_INIT;

    hfc->calibrate         = 0;
    hfc->inhibitRCswitches = false;

    hfc->playlist_items = 0;
    hfc->playlist_status = PLAYLIST_STOPPED;
    hfc->pl_wp_initialized = false;

    hfc->display_mode = DISPLAY_SPLASH;
    
    /* Control modes, 0-inhibit, 1-manual, 2-rate, 3-angle */
    hfc->control_mode[0] = CTRL_MODE_ANGLE;  // P
    hfc->control_mode[1] = CTRL_MODE_ANGLE;  // R
    hfc->control_mode[2] = CTRL_MODE_ANGLE;  // Y
    hfc->control_mode[3] = CTRL_MODE_MANUAL; // C
    hfc->control_mode[4] = CTRL_MODE_MANUAL; // T
    
    hfc->full_auto      = true;	// full auto by default
    hfc->auto_throttle	= true;	// auto throttle by default
    hfc->throttle_armed = 0;    // disables throttle after reset
    hfc->throttle_offset = 0;
    hfc->linklive_item   = 0;

    hfc->waypoint_type   = WAYPOINT_NONE;

    hfc->print_counter  = 0;
    hfc->btnMenuPrev    = true;
    hfc->btnSelectPrev  = true;
    hfc->btnMenuCounter = 0;
    hfc->btnSelectCounter = 0;
    hfc->ticks_lp       = 0;
    hfc->ticks_max      = 0;
    hfc->throttle_width = 0;
    
    hfc->landing_sites_num	= 0;

    hfc->ctrl_yaw_rate          = 0;
    hfc->dyn_yaw_rate           = 50;
    hfc->orient_reset_counter   = 5000;      // once it reaches zero, IMUorient will be reset to SmoothAcc
    hfc->gyro_temperature       = 0;
    hfc->esc_temp               = 20;
    hfc->gyro_temp_lp           = 0;
    hfc->gyroOfs[0]           = 0;
    hfc->gyroOfs[1]           = 0;
    hfc->gyroOfs[2]           = 0;
    hfc->gyro_lp_disp[0]        = 0;
    hfc->gyro_lp_disp[1]        = 0;
    hfc->gyro_lp_disp[2]        = 0;
    for (i=0; i<3; i++)
    {
        hfc->GPSspeedGroundENU[i]   = 0;        // heli ground speed in ground coordinates [E, N, U] in m/s
        hfc->accGroundENUhp[i]		= 0;
        hfc->accGroundENU_prev[i]	= 0;
        hfc->IMUspeedGroundENU[i]	= 0;
    }
    hfc->speedCtrlPrevEN[0]     = 0;
    hfc->speedCtrlPrevEN[1]     = 0;
    hfc->speedHeliRFU[0]        = 0;
    hfc->speedHeliRFU[1]        = 0;
    hfc->speedHeliRFU[2]        = 0;
    hfc->ctrl_vspeed_3d			= 0;
    hfc->ctrl_angle_pitch_3d    = 0;
    hfc->ctrl_angle_roll_3d     = 0;
    hfc->acc_dyn_turns			= 2;
    hfc->cruise_mode            = 0;
//    hfc->positionEMU[0]         = 0;        // position relative to starting position [E, N, U] in m
//    hfc->positionEMU[1]         = 0;        // position relative to starting position [E, N, U] in m
//    hfc->positionEMU[2]         = 0;        // position relative to starting position [E, N, U] in m
    
    hfc->ctrl_out[ANGLE][YAW]   = 0;     // in deg
    hfc->altitude         = 0;     // in m
    hfc->altitude_baro    = 0;
    hfc->altitude_gps     = 0;
    hfc->altitude_ofs     = 0;
    hfc->altitude_lidar   = 0;
    hfc->altitude_lidar_raw = 40;
    hfc->LidarCtrlMode      = false;
    hfc->distance2WP_min    = 999999;
    hfc->RPM                = 0;
    hfc->rpm_pulse          = false;
    hfc->rpm_time_ms_last   = 0;
    hfc->rpm_ticks          = Ticks1();
    hfc->pid_params_changed = false;

    hfc->IMUorient[PITCH]   = 0;
    hfc->IMUorient[ROLL]    = 0;
    hfc->SmoothAcc[PITCH]   = 0;
    hfc->SmoothAcc[ROLL]    = 0;
    hfc->home_pos[0]   = 43.4710273f;
    hfc->home_pos[1]   = -80.5796185f;
    hfc->home_pos[2]   = 99999;
    hfc->altitude_base = 0;
    hfc->waypoint_pos[0] = 0;
    hfc->waypoint_pos[1] = 0;
    hfc->waypoint_pos[2] = 0;
    hfc->waypoint_pos_prev[0] = 0;
    hfc->waypoint_pos_prev[1] = 0;
    hfc->waypoint_pos_prev[2] = 0;
    
    hfc->baro_altitude_raw_lp = -9999;
    hfc->baro_dT = 0;
    hfc->baro_vspeed = 0;
    hfc->baro_vspeed_lp = 0;
    hfc->baro_vspeedDF = 0;
    hfc->lidar_vspeed = 0;
    hfc->tGPS_prev = 0;
    hfc->gps_new_data = false;
    hfc->message_from_ground = 0;
    hfc->lidar_rise = 0;
    hfc->lidar_fall = 0;
    hfc->lidar_pulse = 0;
    hfc->lidar_counter = 0;
    hfc->compass_heading = 0;
    hfc->compass_heading_lp = 0;
    hfc->gps_alt_initialized    = false;

    for (i=0; i<2; i++) hfc->positionLatLon[i] = 0;
    for (i=0; i<11; i++) hfc->baro_derivative_filter[i] = 0;

    hfc->ctrl_source = CTRL_SOURCE_RCRADIO;    // RCradio is in control
    
    hfc->streaming_enable = false;
    hfc->profile_mode     = PROFILING_OFF;
    
    hfc->cpu_utilization_lp = 0;
    
    hfc->telem_ctrl_time = 0;
    hfc->telem_ctrl_period = 0;//100000;   // in uS
    
	hfc->msg2ground_id    = 0;
	hfc->msg2ground_count = 0;
    hfc->tcpip_confirm = false;
    hfc->command.command = TELEM_CMD_NONE;
    
    hfc->compassMin[0] = hfc->compassMin[1] = hfc->compassMin[2] = 9999;
    hfc->compassMax[0] = hfc->compassMax[1] = hfc->compassMax[2] = -9999;

    for(i = 0; i < PITCH_COMP_LIMIT; i++)
    {
        hfc->comp_pitch_flags[i] = 0;
    }

    for(i = 0; i < ROLL_COMP_LIMIT; i++)
    {
        hfc->comp_roll_flags[i] = 0;
    }

    hfc->comp_calibrate = NO_COMP_CALIBRATE;


    hfc->wind_speed = 0;
    hfc->wind_course = 0;

    /* power */
    hfc->power.power_esc 		= 0;
    hfc->power.power_servo 		= 0;
    hfc->power.power_aux12v 	= 0;
    hfc->power.power_armed_led 	= 0;

    hfc->power.Vmain	= 0;
    hfc->power.Vaux		= 0;
    hfc->power.Vservo	= 0;
    hfc->power.Vesc		= 0;
    hfc->power.Iesc		= 0;
    hfc->power.Iaux		= 0;
    hfc->power.battery_level	= 0;	// 0-100%
    hfc->power.capacity_used	= 0;	// AmpSeconds
    hfc->power.capacity_total   = 0;
    hfc->power.flight_time_left = 0;  //
    hfc->power.energy_total = 0;   // rated Ws of the battery
    hfc->power.energy_curr  = 0;    // current Ws
    hfc->power.power_curr   = 0;    // current W
    hfc->power.initialized  = false;
    hfc->power.power_lp = 0;

    hfc->mixer_in[PITCH] = 0;
    hfc->mixer_in[ROLL]  = 0;

    memset(&hfc->stats, 0, sizeof(T_Stats));
}

void Config_Read_Compass(T_HFC *hfc)
{

    Config_Open((char*)"/local/compass.txt");

    LoadConfig_Float("ofs",   hfc->config.comp_ofs,   3);
    LoadConfig_Float("gains", hfc->config.comp_gains, 3);
    LoadConfig_Int  ("min",   hfc->compassMin, 3);
    LoadConfig_Int  ("max",   hfc->compassMax, 3);

}


void Config_Read(T_Config *cfg)
{
    Config_Open((char*)"/local/config.txt");

    LoadConfig_Byte("ConfigVersion",  &cfg->config_version, 1);

    LoadConfig_Int("PWM_PERIOD",      &cfg->pwm_period, 1);
    LoadConfig_Int("TelemBaudrate",   &cfg->telem_baudrate, 1);

    LoadConfig_Byte("CCPMtype",       &cfg->ccpm_type, 1);
    LoadConfig_Byte("SbusEnable", 	  &cfg->SbusEnable, 1);

    LoadConfig_Float("PRstickRate",   &cfg->PRstickRate, 1);
    LoadConfig_Float("PRstickAngle",  &cfg->PRstickAngle, 1);
    LoadConfig_Float("YawStickRate",  &cfg->YawStickRate, 1);
    LoadConfig_Float("StickVspeed",   &cfg->StickVspeed, 1);
    LoadConfig_Float("StickHspeed",   &cfg->StickHspeed, 1);
    LoadConfig_Float("StickHaccel",   &cfg->StickHaccel, 1);
    LoadConfig_Float("AngleCollMix",  &cfg->AngleCollMixing, 1);
    LoadConfig_Float("Stick100range", &cfg->Stick100range, 1);
    LoadConfig_Float("CollZeroAngle", &cfg->CollZeroAngle, 1);
    LoadConfig_Float("CollAutoRotate",&cfg->CollAngleAutoRotate, 1);
    LoadConfig_Float("CollAutoLevel", &cfg->CollThrAutoLevel, 1);

    LoadConfig_Float("LowSpeedLimit", &cfg->low_speed_limit, 1);
    LoadConfig_Int  ("TelemPeriod",   &cfg->telem_min_ctrl_period, 1);
    LoadConfig_Float("DynYawRateMax", &cfg->dyn_yaw_rate_max, 1);
    LoadConfig_Byte ("RCmodeSwOfs",   &cfg->RCmodeSwitchOfs, 1);
    LoadConfig_Byte ("YawModeMin",    &cfg->YawModeMin, 1);
    LoadConfig_Byte ("YawModeMax",    &cfg->YawModeMax, 1);
    LoadConfig_Float("TakeoffAngleR", &cfg->takeoff_angle_rate, 1);
    LoadConfig_Float("CollManSpeed",  &cfg->collective_man_speed, 1);

    LoadConfig_Float("LandingVspeedA",&cfg->landing_vspeed_acc, 1);
    LoadConfig_Float("LandingSpeed",  &cfg->landing_vspeed, 1);
    LoadConfig_Float("LandingAprSpd", &cfg->landing_appr_speed, 1);
    LoadConfig_Float("CruiseSpdLimit",&cfg->cruise_speed_limit, 1);
    LoadConfig_Float("LandWindLimit", &cfg->landing_wind_threshold, 1);
    LoadConfig_Float("JoyMaxSpeed",   &cfg->joystick_max_speed, 1);


    LoadConfig_Byte ("AllowArmInMan", &cfg->AllowArmInManual, 1);
    LoadConfig_Byte ("LidarAltitude", &cfg->ManualLidarAltitude, 1);
    LoadConfig_Byte ("LidarServo",    &cfg->LidarFromServo, 1);
    LoadConfig_Byte ("WindComp",      &cfg->wind_compensation, 1);
    LoadConfig_Byte ("PathNavigation",&cfg->path_navigation, 1);
    LoadConfig_Byte ("NoseToWP",      &cfg->nose_to_WP, 1);
    LoadConfig_Byte("AutoReset",      &cfg->autoReset, 1);

    LoadConfig_Byte ("CANservo",      &cfg->can_servo, 1);
    LoadConfig_Byte ("FCMservo",      &cfg->fcm_servo, 1);
    LoadConfig_Byte ("PowerNode",     &cfg->power_node, 1);
    LoadConfig_Byte ("RPMsensor",     &cfg->rpm_sensor, 1);
    LoadConfig_Byte ("LinkLive",      &cfg->linklive, 1);

    LoadConfig_Float("AilRange",     &cfg->AilRange, 1);
    LoadConfig_Float("EleRange",     &cfg->EleRange, 1);
    LoadConfig_Float("RudRange",     &cfg->RudRange, 1);
    LoadConfig_Float("TorqCompMult", &cfg->TorqCompMult, 1);
    LoadConfig_Float("CollRange",    &cfg->CollRange, 1);
    LoadConfig_Byte("ModelSelect",   &cfg->ModelSelect, 1);
    LoadConfig_Float("CcpmMixer",    &cfg->CcpmMixer, 1);

    LoadConfig_Byte ("GPSvertSpeed",  &cfg->gps_vspeed, 1);
    LoadConfig_Byte ("GPSunits",      &cfg->gps_units, 1);
    LoadConfig_Byte ("GroundSensor",  &cfg->ground_sensor, 1);
    LoadConfig_Float("VspdDownCurve", cfg->VspeedDownCurve,  2);
    LoadConfig_Float("LidarHVcurve",  cfg->LidarHVcurve,     4);

    LoadConfig_Float("DynYawRateMax", &cfg->dyn_yaw_rate_max, 1);
    LoadConfig_Float("TurnAccParams",  cfg->TurnAccParams,    3);

    LoadConfig_Byte("ThrottleCtrl",     &cfg->throttle_ctrl, 1);
    LoadConfig_Float("ThrottleValues",   cfg->throttle_values, 2);
    LoadConfig_Float("ThrottleGain",    &cfg->throttle_gain, 1);
    LoadConfig_Float("ThrottleMultiMin",&cfg->throttle_multi_min, 1);

    
    LoadConfig_Float("ControlGains",   cfg->control_gains, 4);

    LoadConfig_Float("ServoSpeed",     cfg->servo_speed, 2);

    LoadConfig_Byte("CtrlModeInhibit", cfg->ctrl_mode_inhibit, 5);
    LoadConfig_Byte("RCrevert",        cfg->xbus_revert, 8);
    LoadConfig_Byte("ServoRevert",     cfg->servo_revert, 6);
    LoadConfig_Float("StickDeadband",  cfg->stick_deadband, 4);

    LoadConfig_Float("PITCHRATE_PID",  cfg->pitchrate_pid_params,  6);
    LoadConfig_Float("ROLLRATE_PID",   cfg->rollrate_pid_params,   6);
    LoadConfig_Float("PITCHANGLE_PID", cfg->pitchangle_pid_params, 6);
    LoadConfig_Float("ROLLANGLE_PID",  cfg->rollangle_pid_params,  6);
    LoadConfig_Float("YAWRATE_PID",    cfg->yawrate_pid_params,    6);
    LoadConfig_Float("YAWANGLE_PID",   cfg->yawangle_pid_params,   5);
    LoadConfig_Float("VSPEED_PID",     cfg->collvspeed_pid_params, 6);
    LoadConfig_Float("PITCHSPEED_PID", cfg->pitchspeed_pid_params, 6);
    LoadConfig_Float("ROLLSPEED_PID",  cfg->rollspeed_pid_params,  6);
    LoadConfig_Float("ALTITUDE_PID",   cfg->collalt_pid_params,    5);
    LoadConfig_Float("DISTANCE_PID",   cfg->dist2T_pid_params,     5);
    LoadConfig_Float("DIST2PATH_PID",  cfg->dist2P_pid_params,     5);
    LoadConfig_Float("PITCHCRUISE_PID",cfg->pitchCruise_pid_params,5);
    LoadConfig_Float("IMU_PID",        cfg->imu_pid_params,        6);

    LoadConfig_Float("RollPitchAngle", &cfg->RollPitchAngle, 1);
    LoadConfig_Byte("AccLPfreq",       &cfg->acc_lp_freq,   1);
    LoadConfig_Byte("GyroLPfreq",       cfg->gyro_lp_freq,  3);
    LoadConfig_Float("AccIntegGains",   cfg->AccIntegGains, 3);

    LoadConfig_Float("GTWPretireRad",   &cfg->GTWP_retire_radius, 1);
    LoadConfig_Float("GTWPretireSpeed", &cfg->GTWP_retire_speed,  1);
    LoadConfig_Float("FTWPretireSRF",   &cfg->FTWP_retire_sr_factor, 1);

    LoadConfig_Int(  "LandingTimeout",  &cfg->landing_timeout, 1);

    LoadConfig_Int(  "LIDARoffset",     &cfg->lidar_offset, 1);

    //mmri
    LoadConfig_Byte ("AccOrient",   cfg->acc_orient,  6);
    LoadConfig_Float("AccOffsets",  cfg->acc_ofs,     3);

    //LoadConfig_Float("AccGains",    cfg->acc_gains,   3);
    if( !LoadConfig_Float("AccGains",    cfg->acc_gains,   3) )
    {
    	LoadConfig_Float("AccXGains",   cfg->acc_calib_matrix[X_AXIS],  3);
    	LoadConfig_Float("AccYGains",   cfg->acc_calib_matrix[Y_AXIS],  3);
    	LoadConfig_Float("AccZGains",   cfg->acc_calib_matrix[Z_AXIS],  3);
    }

    LoadConfig_Byte ("GyroOrient",  cfg->gyro_orient, 6);

    //LoadConfig_Float("GyroGains",   cfg->gyro_gains,  3);
    if( !LoadConfig_Float("GyroGains",   cfg->gyro_gains,  3) )
    {
    	LoadConfig_Float("GyroXGains",   cfg->gyro_calib_matrix[PITCH],  3);
    	LoadConfig_Float("GyroYGains",   cfg->gyro_calib_matrix[ROLL],  3);
    	LoadConfig_Float("GyroZGains",   cfg->gyro_calib_matrix[YAW],  3);
    }


    LoadConfig_Byte ("CompOrient",  cfg->comp_orient, 6);
    LoadConfig_Float("CompOffsets", cfg->comp_ofs,    3);
//    LoadConfig_Float("CompGains",   cfg->comp_gains,  3);
    if( !LoadConfig_Float("CompGains",   cfg->comp_gains,  3) )
    {
    	LoadConfig_Float("CompXGains",   cfg->comp_calib_matrix[COMP_X],  3);
    	LoadConfig_Float("CompYGains",   cfg->comp_calib_matrix[COMP_Y],  3);
    	LoadConfig_Float("CompZGains",   cfg->comp_calib_matrix[COMP_Z],  3);
    }
    LoadConfig_Float("CompDecOfs", &cfg->comp_declination_offset, 1);

    LoadConfig_Byte ("FCMorient",   cfg->fcm_orient,  6);

    LoadConfig_Float("IMUaccGyroBlend", &cfg->IMUaccGyroBlend,      1);
    LoadConfig_Float("AltBaroGPSblend", &cfg->AltitudeBaroGPSblend_final, 1);
    LoadConfig_Float("PosGPSIMUBldReg", &cfg->Pos_GPS_IMU_BlendReg,     1);
    LoadConfig_Float("PosGPSIMUBldGlt", &cfg->Pos_GPS_IMU_BlendGlitch,  1);

    LoadConfig_Byte ("GyroFixedOfs", &cfg->gyro_fixed_offsets,  1);
    LoadConfig_Float("GyroTempOfsP",  cfg->gyro_drift_coeffs[PITCH],  3);
    LoadConfig_Float("GyroTempOfsR",  cfg->gyro_drift_coeffs[ROLL],   3);
    LoadConfig_Float("GyroTempOfsY",  cfg->gyro_drift_coeffs[YAW],    3);

        printf("PITCH_ofs = %+7.7fT^2 + %+7.7fT + %+7.7f\r\n",
                cfg->gyro_drift_coeffs[PITCH][0],
                cfg->gyro_drift_coeffs[PITCH][1],
                cfg->gyro_drift_coeffs[PITCH][2]);
        printf("ROLL_ofs  = %+7.7fT^2 + %+7.7fT + %+7.7f\r\n",
                cfg->gyro_drift_coeffs[ROLL][0],
                cfg->gyro_drift_coeffs[ROLL][1],
                cfg->gyro_drift_coeffs[ROLL][2]);
        printf("YAW_ofs   = %+7.7fT^2 + %+7.7fT + %+7.7f\r\n",
                cfg->gyro_drift_coeffs[YAW][0],
                cfg->gyro_drift_coeffs[YAW][1],
                cfg->gyro_drift_coeffs[YAW][2]);

    LoadConfig_Byte("BaroLPfreq",          &cfg->baro_lp_freq,   1);
    LoadConfig_Byte("BaroVspeedLPfreq",    &cfg->baro_vspeed_lp_freq,   1);
    LoadConfig_Float("BaroVspeedWeight",   &cfg->BaroVspeedWeight, 1);
    LoadConfig_Float("GPSVspeedWeight",    &cfg->GPSVspeedWeight, 1);
    LoadConfig_Float("BaroAltitudeWeight", &cfg->BaroAltitudeWeight, 1);
    LoadConfig_Int("HeadingAvgs",          &cfg->heading_avgs,    1);
    if(cfg->heading_avgs < 1)
    {
        cfg->heading_avgs = 1;
    }

    LoadConfig_Float("WindTableScale", &cfg->WindTableScale,   1);
    LoadConfig_Float("AngleSpeed0",  cfg->WindSpeedLUT+0,     8);
    LoadConfig_Float("AngleSpeed1",  cfg->WindSpeedLUT+8,     8);
    LoadConfig_Float("AngleSpeed2",  cfg->WindSpeedLUT+16,    8);
    LoadConfig_Float("AngleSpeed3",  cfg->WindSpeedLUT+24,    8);
    LoadConfig_Float("AngleSpeed4",  cfg->WindSpeedLUT+32,    8);
    LoadConfig_Float("AngleSpeed5",  cfg->WindSpeedLUT+40,    6);

    LoadConfig_Byte( "MotorPoles",  &cfg->motor_poles, 1);
    LoadConfig_Float("GearRatio",   &cfg->gear_ratio,  1);

    LoadConfig_Int("BatteryCells",  &cfg->battery_cells, 1);
    LoadConfig_Int("BatCapacity",   &cfg->battery_capacity, 1);
    LoadConfig_Int("PowerTypical",  &cfg->power_typical, 1);
    LoadConfig_Int("RPMtypical",    &cfg->rpm_typical, 1);
    LoadConfig_Float("PowerCoeffs",  cfg->power_coeffs, 6);

    LoadConfig_Float("V2Eng0",  cfg->V2Energy+0,     8);
    LoadConfig_Float("V2Eng1",  cfg->V2Energy+8,     8);
    LoadConfig_Float("V2Eng2",  cfg->V2Energy+16,    8);
    LoadConfig_Float("V2Eng3",  cfg->V2Energy+24,    8);
    LoadConfig_Float("V2Eng4",  cfg->V2Energy+32,    4);

    LoadConfig_Int("CanbusStartDelay", &cfg->canbus_start_delay, 1);
    LoadConfig_Int("NumServoNodes", &cfg->num_servo_nodes, 1);
    LoadConfig_Int("NumGpsNodes", &cfg->num_gps_nodes, 1);
    LoadConfig_Int("ServoRaw", &cfg->servo_raw, 1);
    LoadConfig_Int("CanbusFreqHigh", &cfg->canbus_freq_high, 1);
    LoadConfig_Int("SensorMode", &cfg->sensor_mode, 1);
    LoadConfig_Int("CompassType", &cfg->compass_type, 1);


    Config_Close();
    
    if (Config_Open((char*)"/local/PID_CFG.txt"))
    {
        LoadConfig_Float("PITCHRATE_PID",  cfg->pitchrate_pid_params,  3);
        LoadConfig_Float("ROLLRATE_PID",   cfg->rollrate_pid_params,   3);
        LoadConfig_Float("PITCHANGLE_PID", cfg->pitchangle_pid_params, 3);
        LoadConfig_Float("ROLLANGLE_PID",  cfg->rollangle_pid_params,  3);
        LoadConfig_Float("YAWRATE_PID",    cfg->yawrate_pid_params,    3);
        LoadConfig_Float("YAWANGLE_PID",   cfg->yawangle_pid_params,   2);
        LoadConfig_Float("VSPEED_PID",     cfg->collvspeed_pid_params, 3);
        LoadConfig_Float("PITCHSPEED_PID", cfg->pitchspeed_pid_params, 3);
        LoadConfig_Float("ROLLSPEED_PID",  cfg->rollspeed_pid_params,  3);
        LoadConfig_Float("ALTITUDE_PID",   cfg->collalt_pid_params,    2);
        LoadConfig_Float("DISTANCE_PID",   cfg->dist2T_pid_params,     2);
        LoadConfig_Float("DIST2PATH_PID",  cfg->dist2P_pid_params,     2);
        LoadConfig_Float("PITCHCRUISE_PID",cfg->pitchCruise_pid_params,2);
        LoadConfig_Float("IMU_PID",        cfg->imu_pid_params,        3);
        Config_Close();
    }

    /* load calibration data if it exists */
    if (cfg->gyro_fixed_offsets)
        LoadGyroCalibData(cfg->gyro_ofs);
}

void Save_PIDvalues(T_HFC *hfc)
{
    FILE *fp;

    if (!hfc->pid_params_changed)
        return;

    fp = fopen("/local/PID_CFG.txt", "w");
    if (!fp)
        return;

    fprintf(fp, "PITCHRATE_PID\t%f\t%f\t%f\n", hfc->pid_PitchRate.Kp, hfc->pid_PitchRate.Ki, hfc->pid_PitchRate.Kd);
    fprintf(fp, "ROLLRATE_PID\t%f\t%f\t%f\n",  hfc->pid_RollRate.Kp,  hfc->pid_RollRate.Ki,  hfc->pid_RollRate.Kd);
    fprintf(fp, "YAWRATE_PID\t%f\t%f\t%f\n",   hfc->pid_YawRate.Kp,   hfc->pid_YawRate.Ki,   hfc->pid_YawRate.Kd);

    fprintf(fp, "PITCHANGLE_PID\t%f\t%f\t%f\n",  hfc->pid_PitchAngle.Kp, hfc->pid_PitchAngle.Ki, hfc->pid_PitchAngle.Kd);
    fprintf(fp, "ROLLANGLE_PID\t%f\t%f\t%f\n",   hfc->pid_RollAngle.Kp,  hfc->pid_RollAngle.Ki,  hfc->pid_RollAngle.Kd);

    fprintf(fp, "PITCHSPEED_PID\t%f\t%f\t%f\n",  hfc->pid_PitchSpeed.Kp, hfc->pid_PitchSpeed.Ki, hfc->pid_PitchSpeed.Kd);
    fprintf(fp, "ROLLSPEED_PID\t%f\t%f\t%f\n",   hfc->pid_RollSpeed.Kp,  hfc->pid_RollSpeed.Ki,  hfc->pid_RollSpeed.Kd);
    fprintf(fp, "VSPEED_PID\t%f\t%f\t%f\n",      hfc->pid_CollVspeed.Kp, hfc->pid_CollVspeed.Ki, hfc->pid_CollVspeed.Kd);

    fprintf(fp, "IMU_PID\t%f\t%f\t%f\n", hfc->pid_IMU[0].Kp, hfc->pid_IMU[0].Ki, hfc->pid_IMU[0].Kd);

    fprintf(fp, "YAWANGLE_PID\t%f\t%f\n",    hfc->pid_YawAngle.Kp, hfc->pid_YawAngle.Kd);
    fprintf(fp, "ALTITUDE_PID\t%f\t%f\n",    hfc->pid_CollAlt.Kp,  hfc->pid_CollAlt.Kd);
    fprintf(fp, "DISTANCE_PID\t%f\t%f\n",    hfc->pid_Dist2T.Kp,   hfc->pid_Dist2T.Kd);
    fprintf(fp, "DIST2PATH_PID\t%f\t%f\n",   hfc->pid_Dist2P.Kp,   hfc->pid_Dist2P.Kd);
    fprintf(fp, "PITCHCRUISE_PID\t%f\t%f\n", hfc->pid_PitchCruise.Kp, hfc->pid_PitchCruise.Kd);

    fclose(fp);
}

#if 0
void Config_Save(T_Config *cfg)
{
    FILE *fp = fopen("/local/config2.txt", "w");
    if (!fp)
        return;

    SaveConfig_Int(fp, "PWM_PERIOD", 	  &cfg->pwm_period, 1);

    SaveConfig_Byte(fp, "CCPMtype", 	  &cfg->ccpm_type, 1);
    SaveConfig_Byte(fp, "SbusEnable", 	  &cfg->SbusEnable, 1);

    SaveConfig_Float(fp, "PRstickRate",   &cfg->PRstickRate, 1);
    SaveConfig_Float(fp, "PRstickAngle",  &cfg->PRstickAngle, 1);
    SaveConfig_Float(fp, "YawStickRate",  &cfg->YawStickRate, 1);
    SaveConfig_Float(fp, "StickVspeed",   &cfg->StickVspeed, 1);
    SaveConfig_Float(fp, "StickHspeed",   &cfg->StickHspeed, 1);
    SaveConfig_Float(fp, "Stick100range", &cfg->Stick100range, 1);
    SaveConfig_Float(fp, "CollZeroAngle", &cfg->CollZeroAngle, 1);
    SaveConfig_Float(fp, "LowSpeedLimit", &cfg->low_speed_limit, 1);
    SaveConfig_Byte (fp, "YawMinModeRate",&cfg->YawMinModeRate, 1);
    
    SaveConfig_Byte(fp, "ThrottleCtrl",   &cfg->throttle_ctrl, 1);
    SaveConfig_Float(fp, "ThrottleValues", cfg->throttle_values, 2);
    SaveConfig_Float(fp, "ThrottleGain" , &cfg-> throttle_gain, 1);
    SaveConfig_Float(fp, "ThrottleMultiMin" , &cfg-> throttle_multi_min, 1);
    
    SaveConfig_Float(fp, "ControlGains",   cfg->control_gains, 4);

    SaveConfig_Byte(fp, "CtrlModeInhibit", cfg->ctrl_mode_inhibit, 5);
    SaveConfig_Byte(fp, "RCrevert",        cfg->xbus_revert, 8);
    SaveConfig_Byte(fp, "ServoRevert",     cfg->servo_revert, 6);
    SaveConfig_Float(fp, "StickDeadband",  cfg->stick_deadband, 4);

    SaveConfig_Float(fp, "PITCHRATE_PID",  cfg->pitchrate_pid_params,  6);
    SaveConfig_Float(fp, "ROLLRATE_PID",   cfg->rollrate_pid_params,   6);
    SaveConfig_Float(fp, "YAWRATE_PID",    cfg->yawrate_pid_params,    6);
    SaveConfig_Float(fp, "PITCHANGLE_PID", cfg->pitchangle_pid_params, 6);
    SaveConfig_Float(fp, "ROLLANGLE_PID",  cfg->rollangle_pid_params,  6);
    SaveConfig_Float(fp, "YAWANGLE_PID",   cfg->yawangle_pid_params,   5);
    SaveConfig_Float(fp, "PITCHSPEED_PID", cfg->pitchspeed_pid_params, 6);
    SaveConfig_Float(fp, "ROLLSPEED_PID",  cfg->rollspeed_pid_params,  6);
    SaveConfig_Float(fp, "VSPEED_PID",     cfg->collvspeed_pid_params, 6);
    SaveConfig_Float(fp, "ALTITUDE_PID",   cfg->collalt_pid_params,    5);
    SaveConfig_Float(fp, "DISTANCE_PID",   cfg->dist2T_pid_params,     5);
    SaveConfig_Float(fp, "DIST2PATH_PID",  cfg->dist2P_pid_params,     5);

    SaveConfig_Float(fp, "YawRateSpeed",   &cfg->yaw_rate_speed, 1);
    SaveConfig_Float(fp, "RollPitchAngle", &cfg->RollPitchAngle, 1);

    SaveConfig_Float(fp, "GTWPretireRad",   &cfg->GTWP_retire_radius, 1);
    SaveConfig_Float(fp, "GTWPretireSpeed", &cfg->GTWP_retire_speed,  1);
    SaveConfig_Float(fp, "FTWPretireSRF",   &cfg->FTWP_retire_sr_factor, 1);

    SaveConfig_Int(fp,   "LIDARoffset",     &cfg->lidar_offset, 1);

    SaveConfig_Int(fp,   "CompassOffsets", cfg->comp_ofs, 3);
    SaveConfig_Float(fp, "CompassGains",   cfg->comp_gains, 3);
    SaveConfig_Float(fp, "CompassDecOfs", &cfg->comp_declination_offset, 1);

    SaveConfig_Float(fp, "IMUaccGyroBlend", &cfg->IMUaccGyroBlend,      1);
    SaveConfig_Float(fp, "AltBaroGPSblend", &cfg->AltitudeBaroGPSblend_final, 1);

    fclose(fp);
}
#endif

static float GetAngleFromSpeed(float speed, float WindSpeedLUT[ANGLE2SPEED_SIZE], float scale)
{
  int i;
  int angle1;
  int angle2 = ANGLE2SPEED_SIZE-1;
  float angle;
  float speed1, speed2;
  for (i=0; i<ANGLE2SPEED_SIZE; i++)
    if ((WindSpeedLUT[i]*scale)>speed)
    {
      angle2 = i;
      break;
    }
  angle1 = angle2 - 1;
  speed1 = WindSpeedLUT[angle1]*scale;
  speed2 = WindSpeedLUT[angle2]*scale;

  if (speed1==speed2)
    return angle1;

  angle = (angle2-angle1) / (speed2-speed1) * (speed-speed1) + angle1;
  return angle;
}

void GenerateSpeed2AngleLUT(T_HFC *hfc)
{
    int i;
    for (i=0; i<SPEED2ANGLE_SIZE; i++)
    {
        float speed = i*0.5f;
        float angle = GetAngleFromSpeed(speed, hfc->config.WindSpeedLUT, hfc->config.WindTableScale);
        hfc->config.Speed2AngleLUT[i] = angle;
//        printf("speed %4.1f angle %4.1f\n", speed, angle);
    }
}

extern NokiaLcd   myLcd;

void Config_ApplyAndInit(T_HFC *hfc)
{
    T_Config *cfg = &hfc->config;
    int i;
    
    if (cfg->config_version < MIN_CONFIG_VERSION)
    {
        printf("Config version is %d, needs to be %d\n", cfg->config_version, MIN_CONFIG_VERSION);
        myLcd.ShowError("", "CONFIG VERSION", "IS OLD", "");
    }

    hfc->PRstick_rate  = cfg->PRstickRate /cfg->Stick100range;
    hfc->PRstick_angle = cfg->PRstickAngle/cfg->Stick100range;
    hfc->YawStick_rate = cfg->YawStickRate/cfg->Stick100range;
    hfc->Stick_Vspeed  = cfg->StickVspeed /cfg->Stick100range;
    hfc->Stick_Hspeed  = cfg->StickHspeed /cfg->Stick100range;
    
    /* convert deadband values in % to servo range */
    for (i=0; i<4; i++)
      hfc->StickDeadband[i] = cfg->stick_deadband[i]*0.01f*cfg->Stick100range;

    PID_Init(&hfc->pid_PitchRate, cfg->pitchrate_pid_params, 0, 1);
    PID_Init(&hfc->pid_RollRate,  cfg->rollrate_pid_params,  0, 1);
    PID_Init(&hfc->pid_YawRate,   cfg->yawrate_pid_params,   0, 1);
    PID_Init(&hfc->pid_PitchAngle,cfg->pitchangle_pid_params,1, 0);
    PID_Init(&hfc->pid_RollAngle, cfg->rollangle_pid_params, 1, 0);
    PID_Init(&hfc->pid_CollVspeed,cfg->collvspeed_pid_params,0, 0);
    PID_Init(&hfc->pid_PitchSpeed,cfg->pitchspeed_pid_params,0, 0);
    PID_Init(&hfc->pid_RollSpeed, cfg->rollspeed_pid_params, 0, 0);
    PID_Init(&hfc->pid_IMU[0],    cfg->imu_pid_params, 1, 0);
    PID_Init(&hfc->pid_IMU[1],    cfg->imu_pid_params, 1, 0);
    PID_Init(&hfc->pid_IMU[2],    cfg->imu_pid_params, 1, 0);
    PID_P_Acc_Init(&hfc->pid_YawAngle,cfg->yawangle_pid_params, 1, true);	// enable deceleration
    PID_P_Acc_Init(&hfc->pid_CollAlt, cfg->collalt_pid_params,  0, true);		// same acc and dec
    PID_P_Acc_Init(&hfc->pid_Dist2T,  cfg->dist2T_pid_params,   0, true);
    PID_P_Acc_Init(&hfc->pid_Dist2P,  cfg->dist2P_pid_params,   0, false);
    PID_P_Acc_Init(&hfc->pid_PitchCruise, cfg->pitchCruise_pid_params,   0, false);
    hfc->speed_Iterm_E     = 0;
    hfc->speed_Iterm_N     = 0;
    hfc->speed_Iterm_E_lp  = 0;
    hfc->speed_Iterm_N_lp  = 0;

    /* save angle PID max yaw rate for manual control mode */
    cfg->max_yaw_rate = hfc->pid_YawAngle.COmax;
    
    /* save default values for playlist mode */
    cfg->VspeedMax = hfc->pid_CollAlt.COmax;
    cfg->VspeedMin = hfc->pid_CollAlt.COmin;
    cfg->VspeedAcc = hfc->pid_CollAlt.acceleration;
    cfg->HspeedMax = hfc->pid_Dist2T.COmax;
    cfg->HspeedAcc = hfc->pid_Dist2T.acceleration;
    
    /* init sensor's low pass filters */
    for (i=0; i<3; i++) hfc->calib_gyro_avg[i]  = 0;
    LP4_Init(&hfc->lp_gyro4[0], hfc->config.gyro_lp_freq[0]);   // pitch
    LP4_Init(&hfc->lp_gyro4[1], hfc->config.gyro_lp_freq[1]);   // roll
    LP4_Init(&hfc->lp_gyro4[2], hfc->config.gyro_lp_freq[2]);   // yaw
    for (i=0; i<3; i++) LP4_Init(&hfc->lp_acc4[i], hfc->config.acc_lp_freq);
    
    LP4_Init(&hfc->lp_baro4, hfc->config.baro_lp_freq);
    LP4_Init(&hfc->lp_baro_vspeed4, hfc->config.baro_vspeed_lp_freq);


    hfc->Pos_GPS_IMU_Blend = cfg->Pos_GPS_IMU_BlendReg;
    hfc->telem_ctrl_period = Max(hfc->telem_ctrl_period, hfc->config.telem_min_ctrl_period*1000);

    hfc->throttle_value   = -cfg->Stick100range;
    hfc->collective_value = -cfg->Stick100range;

    hfc->power.capacity_total = cfg->battery_capacity/1000.0f*3600; // As
    hfc->power.energy_total   = hfc->power.capacity_total * cfg->battery_cells * 3.7f;  // Ws

    GenerateSpeed2AngleLUT(hfc);

    Telemetry_Generate_AircraftCfg(hfc, &hfc->aircraftConfig);
}

void Reset_Iterms(T_HFC *hfc)
{
    hfc->pid_PitchRate.Ie  = 0;
    hfc->pid_RollRate.Ie   = 0;
    hfc->pid_YawRate.Ie    = 0;
    hfc->pid_PitchAngle.Ie = 0;
    hfc->pid_RollAngle.Ie  = 0;
    hfc->pid_YawAngle.Ie   = 0;
    hfc->pid_CollVspeed.Ie = 0;
    hfc->pid_PitchSpeed.Ie = 0;
    hfc->pid_RollSpeed.Ie  = 0;
    hfc->pid_CollAlt.Ie    = 0;
    hfc->pid_Dist2T.Ie     = 0;
    hfc->pid_Dist2P.Ie     = 0;
    hfc->pid_PitchCruise.Ie= 0;
    hfc->speed_Iterm_E     = 0;
    hfc->speed_Iterm_N     = 0;
}
