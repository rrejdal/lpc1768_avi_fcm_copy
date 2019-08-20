/**
  ******************************************************************************
  * @file    config.cpp
  * @author  AVIDRONE FIRMWARE Team
  * @brief   Provides access to configuration data held Flash
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
#include "structures.h"
#include "IAP.h"
#include "utils.h"
#include "defines.h"
#include "config.h"


// Config Info
#define FLASH_CONFIG_ADDR       FLASH_SECTOR_29
#define FLASH_CONFIG_SECTOR     29
#define FLASH_CONFIG_SECTORS    (1 - 1)
#define FLASH_CONFIG_SIZE       FLASH_SECTOR_SIZE_16_TO_29
#define MAX_CONFIG_SIZE  (2 * 1024) // 4KB Max config Size

// Compass Calibration Info
#define FLASH_COMPASS_CAL_ADDR      FLASH_SECTOR_28
#define FLASH_COMPASS_CAL_SECTOR    28
#define FLASH_COMPASS_CAL_SECTORS   (1 - 1)
#define FLASH_COMAPSS_CAL_SIZE      FLASH_SECTOR_SIZE_16_TO_29
#define MAX_COMPASS_CAL             (256)

// Odometer - Wasteful in resources, but cleaner to keep in own area of flash
#define FLASH_ODOMETER_ADDR      FLASH_SECTOR_27
#define FLASH_ODOMETER_SECTOR    27
#define FLASH_ODOMETER_SECTORS   (1 - 1)
#define FLASH_ODOMETER_SIZE      (4 * 1024) // Only using 4KB of the 32K FLASH_SECTOR_SIZE_16_TO_29

IAP iap;    // Provides In Application Programming of the Flash.

int __attribute__((__section__(".ramscratch"))) ram_scratch; // this is defined by the linker. DO NOT CHANGE!
extern int __attribute__((__section__(".hfcruntime"))) _hfc_runtime;

static int erase_sectors(int start, int num_sectors);
static int write_sectors(int start, int num_sectors, char *src, int offset, int size);

/**
  * @brief  Return a pointer to Config Data held in Flash .
  * @param  **pConfigData: pointer to Config Data in Flash
  * @retval -1 if Config Data mismatch version, 0 on Success
  */
int LoadConfiguration(const ConfigData **pConfig)
{
  // Map fcm ConfigData structure onto the flash and validate its version
  ConfigData *pConfigData = (ConfigData *)FLASH_CONFIG_ADDR;

  if ((pConfigData->header.total_size <= 0) || (pConfigData->header.total_size >= MAX_CONFIG_SIZE)) {
      return -3;
  }

  // Validate Config Checksum
  unsigned char *pData = (unsigned char *)FLASH_CONFIG_ADDR;
  pData += sizeof(ConfigurationDataHeader);
  if (pConfigData->header.checksum != crc32b(pData, (pConfigData->header.total_size - sizeof(ConfigurationDataHeader)))) {
    return -1;
  }

  // If our version is > then the file version then we cannot process it.
  if (CONFIG_VERSION > pConfigData->header.version) {
      return -2;
  }

  *pConfig = pConfigData;

  return 0;
}

/**
  * @brief  Return a pointer to compass calibration Data held in Flash.
  * @param  **pCompass_cal: pointer to calibration Data in Flash
  * @retval -1 if Config Data mismatch version, 0 on Success
  */
int LoadCompassCalibration(const CompassCalibrationData **pCompass_cal)
{
  CompassCalibrationData *pCompass = (CompassCalibrationData *)FLASH_COMPASS_CAL_ADDR;

  if ((COMPASS_CAL_VERSION > pCompass->version) || !pCompass->valid) {
      return -1;
  }

  *pCompass_cal = pCompass;
  return 0;
}

/**
  * @brief  Update P, I, D values in config stucture
  * @param  *pPid_params: pointer to current Pid Params in use
  * @param  Kp: new P value
  * @param  Ki: new I value
  * @param  Kd: new D value
  * @retval None
  */
static void UpdatePIDconfigValues(float *pPid_params, float Kp, float Ki, float Kd)
{
  *pPid_params++ = Kp;
  *pPid_params++ = Ki;
  *pPid_params++ = Kd;
}

/**
  * @brief  Update P, D values in config stucture
  * @param  *pPid_params: pointer to current Pid Params in use
  * @param  Kp: new P value
  * @param  Kd: new D value
  * @retval None
  */
static void UpdatePDconfigValues(float *pPid_params, float Kp, float Kd)
{
  *pPid_params++ = Kp;
  *pPid_params++ = Kd;
}

/**
  * @brief  Update New PID values to existing RAM copy of config Data.
  * @param  *pConfigData: pointer to RAM copy of Config Data
  * @param  *fcm_data: pointer to runtime flight data
  * @retval None
  */
static void UpdatePIDconfig(ConfigData *pConfigData, FlightControlData *fcm_data)
{
  UpdatePIDconfigValues(pConfigData->pitchrate_pid_params,
                              fcm_data->pid_PitchRate.Kp, fcm_data->pid_PitchRate.Ki, fcm_data->pid_PitchRate.Kd);

  UpdatePIDconfigValues(pConfigData->rollrate_pid_params,
                            fcm_data->pid_RollRate.Kp, fcm_data->pid_RollRate.Ki, fcm_data->pid_RollRate.Kd);

  UpdatePIDconfigValues(pConfigData->yawrate_pid_params,
                            fcm_data->pid_YawRate.Kp, fcm_data->pid_YawRate.Ki, fcm_data->pid_YawRate.Kd);

  UpdatePIDconfigValues(pConfigData->pitchangle_pid_params,
                            fcm_data->pid_PitchAngle.Kp, fcm_data->pid_PitchAngle.Ki, fcm_data->pid_PitchAngle.Kd);

  UpdatePIDconfigValues(pConfigData->rollangle_pid_params,
                            fcm_data->pid_RollAngle.Kp, fcm_data->pid_RollAngle.Ki, fcm_data->pid_RollAngle.Kd);

  UpdatePIDconfigValues(pConfigData->pitchspeed_pid_params,
                            fcm_data->pid_PitchSpeed.Kp, fcm_data->pid_PitchSpeed.Ki, fcm_data->pid_PitchSpeed.Kd);

  UpdatePIDconfigValues(pConfigData->rollspeed_pid_params,
                            fcm_data->pid_RollSpeed.Kp, fcm_data->pid_RollSpeed.Ki, fcm_data->pid_RollSpeed.Kd);

  UpdatePIDconfigValues(pConfigData->collvspeed_pid_params,
                            fcm_data->pid_CollVspeed.Kp, fcm_data->pid_CollVspeed.Ki, fcm_data->pid_CollVspeed.Kd);

  UpdatePIDconfigValues(pConfigData->imu_pid_params,
                            fcm_data->pid_IMU[0].Kp, fcm_data->pid_IMU[0].Ki, fcm_data->pid_IMU[0].Kd);

  UpdatePIDconfigValues(pConfigData->imu_yaw_pid_params,
                            fcm_data->pid_IMU[2].Kp, fcm_data->pid_IMU[2].Ki, fcm_data->pid_IMU[2].Kd);

  UpdatePDconfigValues(pConfigData->yawangle_pid_params, fcm_data->pid_YawAngle.Kp, fcm_data->pid_YawAngle.Kd);

  UpdatePDconfigValues(pConfigData->collalt_pid_params, fcm_data->pid_CollAlt.Kp, fcm_data->pid_CollAlt.Kd);

  UpdatePDconfigValues(pConfigData->dist2T_pid_params, fcm_data->pid_Dist2T.Kp, fcm_data->pid_Dist2T.Kd);

  UpdatePDconfigValues(pConfigData->dist2P_pid_params, fcm_data->pid_Dist2P.Kp, fcm_data->pid_Dist2P.Kd);

  UpdatePDconfigValues(pConfigData->pitchCruise_pid_params, fcm_data->pid_PitchCruise.Kp, fcm_data->pid_PitchCruise.Kd);
}

// @brief
// @param  none
// @retval none
static int erase_sectors(int start, int num_sectors)
{
  int iap_result;
  int end = --num_sectors + start;

  KICK_WATCHDOG();

  if ((iap_result = iap.prepare(start, end)) != CMD_SUCCESS) {
    return iap_result;
  }

  iap_result = iap.erase(start, end);

  return iap_result;
}

// @brief
// @param  none
// @retval none
static int write_sectors(int start, int num_sectors, char *src, int offset, int size)
{
  int iap_result;
  int end = --num_sectors + start;

  KICK_WATCHDOG();

  if ((iap_result = iap.prepare(start, end)) != CMD_SUCCESS) {
   return iap_result;
  }

  iap_result = iap.write(src, sector_start_adress[start] + offset, size);
  return iap_result;

}

/**
  * @brief  Update New PID values to config file in flash.
  * @param  *fcm_data: pointer to runtime flight data
  * @retval -1 on error, 0 on success
  */
int UpdateFlashConfig(FlightControlData *fcm_data)
{
  int iap_result;

  // If no params have been updated, do nothing
  if (!fcm_data->pid_params_changed) {
    return 0;
  }

  // Re-Write config with PID update values from Telemetry.
  ConfigData *pFlashConfigData = (ConfigData *)FLASH_CONFIG_ADDR;

  if (CONFIG_VERSION > pFlashConfigData->header.version) {
    return -2;
  }

  if (sizeof(ConfigData) > MAX_CONFIG_SIZE) {
    return -1;
  }

  // Copy Flash config into reserved RAM space (setup from Linker)
  ConfigData *pConfigData = (ConfigData *)&ram_scratch;
  if (pConfigData == NULL) {
      return -1;
  }

  KICK_WATCHDOG();
  // copy config data from Flash to RAM
  memcpy(pConfigData, pFlashConfigData, MAX_CONFIG_SIZE);

  // Overwrite PID config values
  UpdatePIDconfig(pConfigData, fcm_data);

  // Recalculate checksum on file
  unsigned char *pData = (unsigned char *)pConfigData;
  pData += sizeof(ConfigurationDataHeader);
  pConfigData->header.checksum = crc32b(pData, (pConfigData->header.total_size - sizeof(ConfigurationDataHeader)));

  __disable_irq();

  if ((iap_result = erase_sectors(FLASH_CONFIG_SECTOR, 1)) != CMD_SUCCESS) {
    return iap_result;
  }

  // 1st block
  if ((iap_result = write_sectors(FLASH_CONFIG_SECTOR, 1, (char *)&ram_scratch, 0, 1024)) != CMD_SUCCESS) {
    return iap_result;
  }

  // 2nd block
  iap_result = write_sectors(FLASH_CONFIG_SECTOR, 1, (char *)&ram_scratch+0x400, 0x400, 1024);

  __enable_irq();

  return 0;
}

/**
  * @brief  Update New Config Data to Flash.
  *         - This will ERASE existing configuration
  * @param  *none
  * @retval -1 on error, 0 on success
  */
int SaveNewConfig(void)
{
  int iap_result;

  if (sizeof(ConfigData) > MAX_CONFIG_SIZE) {
    // Config File has exceeded Max Size
    return -1;
  }

  // Grab pointer to reserved RAM space (setup from Linker)
  // This should contain new configuration data.
  ConfigData *pConfigData = (ConfigData *)&ram_scratch;
  if (pConfigData == NULL) {
    return -1;
  }

  // Validate its version
  // If our version is > then the file version then we cannot process it.
  if (CONFIG_VERSION != pConfigData->header.version) {
    return -2;
  }

  __disable_irq();

  if ((iap_result = erase_sectors(FLASH_CONFIG_SECTOR, 1)) != CMD_SUCCESS) {
    return iap_result;
  }

  // 1st block
  if ((iap_result = write_sectors(FLASH_CONFIG_SECTOR, 1, (char *)&ram_scratch, 0, 1024)) != CMD_SUCCESS) {
    return iap_result;
  }

  // 2nd block
  iap_result = write_sectors(FLASH_CONFIG_SECTOR, 1, (char *)&ram_scratch+0x400, 0x400, 1024);

  __enable_irq();

  return iap_result;
}

/**
  * @brief  Update New compass calibration values to Flash.
  * @param  *pCompass_cal: pointer to compass calibration data in RAM
  * @retval -1 on error, 0 on success
  */
int SaveCompassCalibration(const CompassCalibrationData *pCompass_cal)
{
  int iap_result;

  if (sizeof(CompassCalibrationData) > MAX_COMPASS_CAL) {
    return -1;
  }

  __disable_irq();

  if ((iap_result = erase_sectors(FLASH_COMPASS_CAL_SECTOR, 1)) != CMD_SUCCESS) {
    return iap_result;
  }

  iap_result = write_sectors(FLASH_COMPASS_CAL_SECTOR, 1, (char *)pCompass_cal, 0, 256);

  __enable_irq();

  return iap_result;
}

// @brief
// @param  none
// @retval none
int EraseFlash(void)
{
  int iap_result;

  __disable_irq();

  iap_result = erase_sectors(0, 30);

  __enable_irq();

  return iap_result;
}


#define CRP_1       0x12345678
#define CRP_2       0x87654321
#define CRP_3       0x43218765
#define CRP_UNLOCK  0xFFFFFFFF
#define CRP_OFFSET  0x000002FC

// @brief
// @param  none
// @retval none
int SetJtag(int state)
{
  int iap_result;

  // Copy sector 0 to RAM, update 0x2FC with Flag
  // Erase Sector 0 and then re-write it
  // sector 0 is 4kB in size, make use of the reserved hfc area (which is total 12K)
  // The hfc area will be blown away and hence MUST do a reset after using this routine.
  // Grab pointer to reserved RAM space (setup from Linker)
  // This should contain new configuration data.
  uint32_t *pRamScratch = (uint32_t *)&_hfc_runtime;
  if (pRamScratch == NULL) {
      return -1;
  }

  // Determine if lock state is already set to desired, return if nothing to do.
  uint32_t *pCRP_key = (uint32_t *)CRP_OFFSET;
  if (*pCRP_key == CRP_2) {
      return 0;
  }

  KICK_WATCHDOG();
  memcpy(pRamScratch, sector_start_adress[0], FLASH_SECTOR_SIZE_0_TO_15);

  // TODO::SP - check value of state and write appropriate value
  uint32_t *tmp = pRamScratch;
  tmp += (CRP_OFFSET/4);

  if (state == LOCK_JTAG) {
    *tmp = (uint32_t)CRP_2;
  }
  else {
    *tmp = (uint32_t)CRP_UNLOCK;
  }

  __disable_irq();

  if ((iap_result = erase_sectors(0, 1)) != CMD_SUCCESS) {
    return iap_result;
  }

  iap_result = write_sectors(0, 1, (char *)pRamScratch, 0, 4096);

  __enable_irq();

  return iap_result;
}

// @brief
// @param  none
// @retval none
void InitializeOdometer(FlightControlData *hfc)
{
  uint32_t *pOdometerReading = (uint32_t *)&ram_scratch;

  memcpy(pOdometerReading, sector_start_adress[FLASH_ODOMETER_SECTOR], 256);

  if (*pOdometerReading == 0xffffffff) {
    // If flash value is default (erased) then initialize flight time odometer to 0
    hfc->OdometerReading = 0;
  }
  else {
    hfc->OdometerReading = *pOdometerReading * 1000;
  }
}

// Update value of Odometer, which indicates total flight time of unit.
// Odometer value is stored in Flash as the number of seconds.
// Max flight time recording is 1,193,046 hours
int UpdateOdometerReading(uint32_t Odometer)
{
  int iap_result;

  uint32_t *pOdometerReading = (uint32_t *)&ram_scratch;
  if (pOdometerReading == NULL) {
      return -1;
  }

  if (*pOdometerReading == (Odometer / 1000)) {
    // If no change to flight time odometer - do nothing.
    return 0;
  }

  // Update the odometer reading (in seconds) back to flash.
  *pOdometerReading = Odometer / 1000;

  __disable_irq();

  if ((iap_result = erase_sectors(FLASH_ODOMETER_SECTOR, 1)) != CMD_SUCCESS) {\
    return iap_result;
  }

  iap_result = write_sectors(FLASH_ODOMETER_SECTOR, 1, (char *)pOdometerReading, 0, 256);

  __enable_irq();

  return iap_result;
}

/************************ (C) COPYRIGHT Avidrone Aerospace Inc. *****END OF FILE****/


