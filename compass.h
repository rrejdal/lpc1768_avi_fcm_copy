/**
  ******************************************************************************
  * @file    compass.h
  * @author  AVIDRONE FIRMWARE Team
  * @brief   Provides support for compass related routines
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

#ifndef COMPASS_H_
#define COMPASS_H_

#include "structures.h"
#include "defines.h"

class Compass
{
public :
  Compass(const ConfigData *pConfig, float *calibration_offsets, float *calibration_gains)
  {
    compass_id_ = pConfig->compass_selection;

    for (int i =0; i < 6; i++) {
      comp_orient_[i] = pConfig->comp_orient[i];
      fcm_orient_[i] = pConfig->fcm_orient[i];
    }

    for (int i=0; i < 3; i++) {
      offsets_[compass_id_][i] = calibration_offsets[i];
      gains_[compass_id_][i] = calibration_gains[i];
    }

    declination_ = -9.63; // Waterloo declination angle
    field_intensity_ = 537.37;

  };

  ~Compass() {};

  // Return compass heading in degrees, CW starting from North
  float GetHeadingDeg(float pitch, float roll);

  // Get Raw heading based on no calibrated Mag data - NOT to be used for flying!
  int GetRawHeadingXY(int compass_id);

  void UpdateMagData(int compass_id, unsigned char *pdata);

  int16_t GetMagData(int axis) { return compass_data_[compass_id_][axis]; };
  int16_t GetMagData(int compass_id, int axis) { return compass_data_[compass_id][axis]; };

  float GetCalibratedMagData(int axis) { return calibrated_data_[compass_id_][axis]; };
  float GetCalibratedMagData(int compass_id, int axis) { return calibrated_data_[compass_id][axis]; };

  void SetDeclination(float declination) { declination_ = declination; };
  void SetFieldIntensity(float field_intensity) { field_intensity_ = field_intensity; };

  int HaveNewData(void) { return new_data_[compass_id_]; };
  int HaveNewData(int compass_id) { return new_data_[compass_id]; };

private:
  int new_data_[MAX_NUM_COMPASS] = {0};

  int compass_id_;
  float declination_;
  float field_intensity_;
  float offsets_[MAX_NUM_COMPASS][3];
  float gains_[MAX_NUM_COMPASS][3];
  unsigned char comp_orient_[6];
  unsigned char fcm_orient_[6];

  int16_t compass_data_[MAX_NUM_COMPASS][3] = {0};    // Raw Compass Mag data
  float calibrated_data_[MAX_NUM_COMPASS][3] = {0};   // Compass data post calibration applied.
};

#endif /* COMPASS_H_ */



