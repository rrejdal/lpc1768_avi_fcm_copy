#include "config.h"
#include "utils.h"
#include "HMC5883L.h"
#include "defines.h"
#include "IMU.h"
#include "telemetry.h"
#include "NOKIA_5110.h"
#include "IAP.h"

#define MIN_CONFIG_VERSION      9

#define FLASH_CONFIG_ADDR   USER_FLASH_AREA_START
#define FLASH_CONFIG_SIZE   FLASH_SECTOR_SIZE_16_TO_29

// Load configuration structure from Flash
int LoadConfiguration(ConfigData *pConfig)
{
    // Map hfc config structure onto the flash and validate its version
    pConfig = (ConfigData*)FLASH_CONFIG_ADDR;

    if (pConfig->config_version != MIN_CONFIG_VERSION) {

        return -1;
    }

    return 0;
}




extern HMC5883L compass;
extern NokiaLcd   myLcd;


#if 0
//TODO::SP: handle Compass Data
void Config_Read_Compass(T_HFC *hfc)
{

    Config_Open((char*)"/local/compass.txt");

    LoadConfig_Float("ofs",   hfc->config.comp_ofs,   3);
    LoadConfig_Float("gains", hfc->config.comp_gains, 3);
    LoadConfig_Int  ("min",   hfc->compassMin, 3);
    LoadConfig_Int  ("max",   hfc->compassMax, 3);

}
#endif

#if 0
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
#endif

#if 0
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
#endif

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


#if 0
// TODO::SP: Add back in
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

    // TODO::SP: find telem
    //telem.Generate_AircraftCfg();
}
#endif

