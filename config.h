/**
  ******************************************************************************
  * @file    config.h
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

#ifndef CONFIG_H_
#define CONFIG_H_

// ---- Public Interfaces ---- //
int LoadConfiguration(const ConfigData **pConfig);
int LoadCompassCalibration(const CompassCalibrationData **pCompass_cal);
int SaveNewConfig(void);
int SaveCompassCalibration(const CompassCalibrationData *pCompass_cal);
void InitializeOdometer(FlightControlData *hfc);
int UpdateOdometerReading(uint32 OdometerCounter);
int EraseFlash(void);
int SetJtag(int state);

// ---- Public Data ---- //
#define CONFIG_VERSION   14
#define COMPASS_CAL_VERSION 1

#endif /* CONFIG_H_ */
