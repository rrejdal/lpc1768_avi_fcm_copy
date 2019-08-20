/**
  ******************************************************************************
  * @file    mixer.h
  * @author  AVIDRONE FIRMWARE Team
  * @brief   Provides various mixing, based upon airframe type
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

#ifndef MIXER_H_
#define MIXER_H_

#include "structures.h"

// @brief
// @param
// @retval
class AviMixer
{
public:
  AviMixer() {};
  virtual ~AviMixer() {};

  virtual void ServoMixer(float* mixer_in, float* servos_out) = 0;
};

/* Quad Mixer
 *
 *  3CW       0CCW
 *     \  ^  /
 *     /  |  \
 * 2CCW       1CW
 */
class QuadMixer : public AviMixer
{
public :
  QuadMixer() {};
  ~QuadMixer() {};

  void ServoMixer(float* mixer_in, float* servos_out);
};

/*
 * Hex layout as shown
 *
 *  5CCW     0CW
 *      \   /
 *  4CW-- | --1CCW
 *      /   \
 *  3CCW     2CW
 */
class HexMixer : public AviMixer
{
public :
  HexMixer() {};
  ~HexMixer() {};

  void ServoMixer(float* mixer_in, float* servos_out);
};

/* Octo Mixer
 *
 *    7CCW     0CW
 *      \   ^  /
 *  6CW     |     1CCW
 *     \-   |   -/
 *     /-       -\
 * 5CCW           2CW
 *      /       \
 *    4CW     3CCW
 */
class OctoMixer : public AviMixer
{
public :
  OctoMixer() {};
  ~OctoMixer() {};

  void ServoMixer(float* mixer_in, float* servos_out);
};

// CCPM 120deg mixer
// input: raw throttle, pitch and roll values
// output: servo signals A (at 6 o'clock) B (at 2 o'clock) and C (at 10 o'clock)
// value range: -/+ 0.571 corresponds to 100% stick */
class CCPM120Mixer : public AviMixer
{
public :
  CCPM120Mixer() {};
  ~CCPM120Mixer() {};

  void ServoMixer(float* mixer_in, float* servos_out);
};

// CCPM 140deg mixer
// input: raw throttle, pitch and roll values
// output: servo signals A (at 6 o'clock) B (at 2 o'clock) and C (at 10 o'clock)
// value range: -/+ 0.571 corresponds to 100% stick */
class CCPM140Mixer : public AviMixer
{
public :
  CCPM140Mixer() {};
  ~CCPM140Mixer() {};

  void ServoMixer(float* mixer_in, float* servos_out);
};

// Servo map on Tandem
//    Front(Node 1)               Rear(Node 2)
//   B(3)                             B(3)
//       C(4)   <---------------  C(4)
//   A(2)                             A(2)
// () servo channel output
//
// Throttle(1)
// CcpmMixer value of 0.5 multiplier is used for 120deg CCPM
// CcpmMixer value of 1.0 multiplier is used for 140deg CCPM
class AviTandemMixer
{
public :
  AviTandemMixer(const ConfigData *pConfig);
  ~AviTandemMixer() {};

  void UpdateDcpElevGains(float dcp_gain, float elev_gain) { dcp_gain_ = dcp_gain; elevator_gain_ = elev_gain; };
  void ServoMixer(float *mixer_in, float *servos_front, float *servos_rear);

private:
  float ccpm_mixer_;    // 120 or 140 mixer
  int model_;           // tandem, copycat or E6

  float throttle_gain_;
  float rear_rpm_trim_;
  float ail_range_;
  float ele_range_;
  float swash_tilt_rear_;
  float rud_range_;
  float torq_comp_multiplier_;
  float collective_range_;
  float dcp_front_;
  float dcp_rear_;
  float dcp_gain_;
  float elevator_gain_;
};

#endif /* MIXER_H_ */
