/**
  ******************************************************************************
  * @file    compass.cpp
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
#include "defines.h"
#include "IMU.h"
#include "mymath.h"
#include "compass.h"

#define PI2 (2*PI)

void Compass::UpdateMagData(int compass_id, unsigned char *pdata)
{

  compass_data_[compass_id][0] = pdata[0] | (pdata[1] << 8);
  compass_data_[compass_id][1] = pdata[2] | (pdata[3] << 8);
  compass_data_[compass_id][2] = pdata[4] | (pdata[5] << 8);

  new_data_[compass_id] = 1;
}

// @brief Return compass heading in degrees, CW starting from North
// @param
// @retval heading
float Compass::GetHeadingDeg(float pitch, float roll)
{
  int i;
  float heading;
  float data[3];
  float compENU[3];
  float PRY[3] = {pitch, roll, 0};

  for (i=0; i<3; i++) {
    calibrated_data_[compass_id_][i] = (compass_data_[compass_id_][i] - offsets_[compass_id_][i]) * gains_[compass_id_][i];
  }

  data[0] = calibrated_data_[compass_id_][0];
  data[1] = calibrated_data_[compass_id_][1];
  data[2] = calibrated_data_[compass_id_][2];

  // re-orient compass, negative value flips sign
  for (i=0; i<3; i++)
  {
    calibrated_data_[compass_id_][i] = data[comp_orient_[i]];
    if (comp_orient_[i+3]) {
      calibrated_data_[compass_id_][i] = -calibrated_data_[compass_id_][i];
    }
  }

  data[0] = calibrated_data_[compass_id_][0];
  data[1] = calibrated_data_[compass_id_][1];
  data[2] = calibrated_data_[compass_id_][2];

  // re-orient the entire flight controller, negative value flips the sign
  for (i=0; i<3; i++)
  {
    calibrated_data_[compass_id_][i] = data[fcm_orient_[i]];

    if (fcm_orient_[i+3]) {
      calibrated_data_[compass_id_][i]  = -calibrated_data_[compass_id_][i];
    }
  }

  // compass is in RFU coordinates at this point. Magnetic vectors point almost down in northen hemisphere
  // the given axes has to read out positive when aligned with the magnetic vector.
  // This needs to be re-oriented to ENU using PRY. Then heading=atan2(E,N)
  Plane2Ground(&calibrated_data_[compass_id_][0], PRY, compENU);

  heading = ATAN2fD(-compENU[EAST], compENU[NORTH]);

  heading += declination_;

  if(heading < -180) // fix sign
      heading += 360;
  else
  if(heading > 180) // fix overflow
      heading -= 360;

  new_data_[compass_id_] = 0;

  return heading;
}

// @brief Return raw compass heading in degrees, CW starting from North
// @param
// @retval heading
int Compass::GetRawHeadingXY(int compass_id)
{
  double heading = atan2((double)compass_data_[compass_id][1], (double)compass_data_[compass_id][0]);

  heading += declination_ * D2R; // Declination is in deg, however at this point working in rad

  if (heading < 0.0) {// fix sign
    heading += PI2;
  }

  if (heading > PI2) { // fix overflow
    heading -= PI2;
  }

  int headingDegrees = heading * 180 / M_PI;

  return headingDegrees;
}


/************************ (C) COPYRIGHT Avidrone Aerospace Inc. *****END OF FILE****/




