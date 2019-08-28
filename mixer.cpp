/**
  ******************************************************************************
  * @file    mixer.cpp
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
#include "defines.h"
#include "mixer.h"
#include "mymath.h"

// @brief
// @param
// @retval
void QuadMixer::ServoMixer(float* mixer_in, float* servos_out)
{
  servos_out[0] -= mixer_in[ROLL];
  servos_out[1] -= mixer_in[ROLL];
  servos_out[2] += mixer_in[ROLL];
  servos_out[3] += mixer_in[ROLL];

  servos_out[0] += mixer_in[PITCH];
  servos_out[1] -= mixer_in[PITCH];
  servos_out[2] -= mixer_in[PITCH];
  servos_out[3] += mixer_in[PITCH];

  servos_out[0] += mixer_in[YAW];
  servos_out[1] -= mixer_in[YAW];
  servos_out[2] += mixer_in[YAW];
  servos_out[3] -= mixer_in[YAW];

  // 4 & 5 are reserved for Amred LED..

  // 6 & 7 are Unused outputs
  servos_out[6] = 0;
  servos_out[7] = 0;
}

// @brief
// @param
// @retval
void HexMixer::ServoMixer(float* mixer_in, float* servos_out)
{
  servos_out[0] -= mixer_in[ROLL]*0.5f;
  servos_out[1] -= mixer_in[ROLL];
  servos_out[2] -= mixer_in[ROLL]*0.5f;
  servos_out[3] += mixer_in[ROLL]*0.5f;
  servos_out[4] += mixer_in[ROLL];
  servos_out[5] += mixer_in[ROLL]*0.5f;

  servos_out[0] += mixer_in[PITCH]*0.866f;
  servos_out[2] -= mixer_in[PITCH]*0.866f;
  servos_out[3] -= mixer_in[PITCH]*0.866f;
  servos_out[5] += mixer_in[PITCH]*0.866f;

  servos_out[0] += mixer_in[YAW];     //cw
  servos_out[1] -= mixer_in[YAW];
  servos_out[2] += mixer_in[YAW];     //cw
  servos_out[3] -= mixer_in[YAW];
  servos_out[4] += mixer_in[YAW];     //cw
  servos_out[5] -= mixer_in[YAW];

  // Channels 6 & 7 not used.
  servos_out[6] = 0;
  servos_out[7] = 0;
}

// @brief
// @param
// @retval
void OctoMixer::ServoMixer(float* mixer_in, float* servos_out)
{
  servos_out[0] += mixer_in[PITCH];
  servos_out[1] += mixer_in[PITCH] * 0.414174f;
  servos_out[2] -= mixer_in[PITCH] * 0.414174f;
  servos_out[3] -= mixer_in[PITCH];
  servos_out[4] -= mixer_in[PITCH];
  servos_out[5] -= mixer_in[PITCH] * 0.414174f;
  servos_out[6] += mixer_in[PITCH] * 0.414174f;
  servos_out[7] += mixer_in[PITCH];

  servos_out[0] -= mixer_in[ROLL] * 0.414174f;
  servos_out[1] -= mixer_in[ROLL];
  servos_out[2] -= mixer_in[ROLL];
  servos_out[3] -= mixer_in[ROLL] * 0.414174f;
  servos_out[4] += mixer_in[ROLL] * 0.414174f;
  servos_out[5] += mixer_in[ROLL];
  servos_out[6] += mixer_in[ROLL];
  servos_out[7] += mixer_in[ROLL] * 0.414174f;

  servos_out[0] += mixer_in[YAW];         //cw
  servos_out[1] -= mixer_in[YAW];
  servos_out[2] += mixer_in[YAW];         //cw
  servos_out[3] -= mixer_in[YAW];
  servos_out[4] += mixer_in[YAW];         //cw
  servos_out[5] -= mixer_in[YAW];
  servos_out[6] += mixer_in[YAW];         //cw
  servos_out[7] -= mixer_in[YAW];
}

// @brief
// @param
// @retval
void CCPM120Mixer::ServoMixer(float* mixer_in, float* servos_out)
{
  /* collective */
  float A = mixer_in[COLL];
  float B = mixer_in[COLL];
  float C = mixer_in[COLL];

  /* roll */
  B -= mixer_in[ROLL] * 0.8660254038f; //cos(30);
  C += mixer_in[ROLL] * 0.8660254038f; //cos(30);

  /* pitch */
  A += mixer_in[PITCH];
  B -= mixer_in[PITCH] * 0.5f; //sin(30);
  C -= mixer_in[PITCH] * 0.5f; //sin(30);

  servos_out[CCPM_A] = A;
  servos_out[CCPM_B] = B;
  servos_out[CCPM_C] = C;

  servos_out[YAW]  = mixer_in[YAW];
  servos_out[THRO] = mixer_in[THRO];
}

// @brief
// @param
// @retval
void CCPM140Mixer::ServoMixer(float* mixer_in, float* servos_out)
{
  /* collective */
  float A = mixer_in[COLL];
  float B = mixer_in[COLL];
  float C = mixer_in[COLL];

  /* roll */
  B -= mixer_in[ROLL] * 0.8660254038f; //cos(30);
  C += mixer_in[ROLL] * 0.8660254038f; //cos(30);

  /* pitch */
  A += mixer_in[PITCH];
  B -= mixer_in[PITCH];
  C -= mixer_in[PITCH];

  servos_out[CCPM_A] = A;
  servos_out[CCPM_B] = B;
  servos_out[CCPM_C] = C;

  servos_out[YAW] = mixer_in[YAW];
  servos_out[THRO] = mixer_in[THRO];
}

// @brief
// @param
// @retval
AviTandemMixer::AviTandemMixer(const ConfigData *pConfig)
{
  throttle_gain_ =  pConfig->throttle_gain;
  ccpm_mixer_ =     pConfig->CcpmMixer;
  model_ =          pConfig->ModelSelect;
  rear_rpm_trim_ =  pConfig->RearRpmTrim;

  ail_range_ =      pConfig->AilRange;
  ele_range_ =      pConfig->EleRange;
  rud_range_ =      pConfig->RudRange;
  dcp_front_ =      pConfig->dcpFront;
  dcp_rear_ =       pConfig->dcpRear;

  swash_tilt_rear_  = pConfig->swashTiltRear;
  collective_range_ = pConfig->CollRange;

  torq_comp_multiplier_ = pConfig->TorqCompMult;

  dcp_gain_ =       pConfig->dcp_gain;
  elevator_gain_ =  pConfig->elevator_gain;

}

// @brief
// @param
// @retval
void AviTandemMixer::ServoMixer(float *mixer_in, float *servos_front, float *servos_rear)
{
  float ROLL_Taileron;
  float PITCH_Televator, PITCH_TelevatorR;
  float YAW_Trudder;
  float dcp, torqComp;
  float TcollectFront,TcollectRear;
  float ROLL_TaileronFront, ROLL_TaileronRear;

  if (model_ == TANDEM_210TL) {
    // OPERATING AS 210TL

    // gain calculation 0 to .571 maximum, could x 1.7512f for full 0 to 1 range
    // gain is set usually by digital levers on the transmitter for in flight adjustment
    ROLL_Taileron  = mixer_in[ROLL]  * ail_range_;
    PITCH_Televator = mixer_in[PITCH] * ele_range_ * elevator_gain_;
    PITCH_TelevatorR = PITCH_Televator * swash_tilt_rear_;
    YAW_Trudder   = mixer_in[YAW] * rud_range_;

    dcp      = PITCH_Televator * dcp_gain_;   // Differential collective pitch calculation
    torqComp = dcp * torq_comp_multiplier_;   // Torque compensation due to dcp and elevator

    TcollectFront = mixer_in[COLL] * collective_range_ + dcp * dcp_front_;
    TcollectRear  = mixer_in[COLL] * collective_range_ - dcp * dcp_rear_;

    ROLL_TaileronFront = ROLL_Taileron + YAW_Trudder + torqComp;
    ROLL_TaileronRear  = ROLL_Taileron - YAW_Trudder - torqComp;

    // Front Servo
    servos_front[1] = 0 + ROLL_TaileronFront + (ccpm_mixer_ * PITCH_Televator) + TcollectFront;   // aFrontServo
    servos_front[2] = 0 + ROLL_TaileronFront - (ccpm_mixer_ * PITCH_Televator) - TcollectFront;   // bFrontServo
    servos_front[3] = 0 + TcollectFront - PITCH_Televator;                                               // cFrontServo

    // Rear Servo
    servos_rear[1] = 0 - TcollectRear  - (ccpm_mixer_ * PITCH_TelevatorR) - ROLL_TaileronRear;    // aRearServo
    servos_rear[2] = 0 + TcollectRear  + (ccpm_mixer_ * PITCH_TelevatorR) - ROLL_TaileronRear;    // bRearServo
    servos_rear[3] = 0 - TcollectRear  + PITCH_TelevatorR;

  }
  else if (model_ == TANDEM_DP14) {
    // OPERATING AS DP14

    float __ROLL_TaileronFront, __ROLL_TaileronRear;
    float __PITCH_Televator, __PITCH_TelevatorR;

    const float phase_shift_F = 18.0f*PI/180.0f;  //  18 degrees for front rotor
    const float phase_shift_R = -48.0f*PI/180.0f; // -48 degrees for rear rotor

    // gain calculation 0 to .571 maximum, could x 1.7512f for full 0 to 1 range
    // gain is set usually by digital levers on the transmitter for in flight adjustment
    ROLL_Taileron  = mixer_in[ROLL]  * ail_range_;
    PITCH_Televator = mixer_in[PITCH] * ele_range_ * elevator_gain_;
    PITCH_TelevatorR = PITCH_Televator * swash_tilt_rear_;
    YAW_Trudder   = mixer_in[YAW] * rud_range_;

    dcp      = PITCH_Televator * dcp_gain_;   // Differential collective pitch calculation
    torqComp = dcp * torq_comp_multiplier_;   // Torque compensation due to dcp and elevator

    TcollectFront = mixer_in[COLL] * collective_range_ + dcp * dcp_front_;
    TcollectRear  = mixer_in[COLL] * collective_range_ - dcp * dcp_rear_;

    ROLL_TaileronFront = ROLL_Taileron + YAW_Trudder + torqComp;
    ROLL_TaileronRear  = ROLL_Taileron - YAW_Trudder - torqComp;

    //dp14, phase shifting by theta (positive is CCW an negative is CW)
    //this is equivalent to a rotational transform of pitch and roll by theta
    __PITCH_Televator = PITCH_Televator*cos(phase_shift_F) - ROLL_TaileronFront*sin(phase_shift_F);
    __ROLL_TaileronFront = PITCH_Televator*sin(phase_shift_F) + ROLL_TaileronFront*cos(phase_shift_F);

    __PITCH_TelevatorR = PITCH_TelevatorR*cos(phase_shift_R) - ROLL_TaileronRear*sin(phase_shift_R);
    __ROLL_TaileronRear = PITCH_TelevatorR*sin(phase_shift_R) + ROLL_TaileronRear*cos(phase_shift_R);

    // Front Servo
    servos_front[1] = 0 - __ROLL_TaileronFront + (ccpm_mixer_ * __PITCH_Televator) + TcollectFront;   // aFrontServo
    servos_front[2] = 0 - __ROLL_TaileronFront - (ccpm_mixer_ * __PITCH_Televator) - TcollectFront;   // bFrontServo
    servos_front[3] = 0 + TcollectFront - __PITCH_Televator;                                               // cFrontServo

    // Rear Servo
    servos_rear[1] = 0 + TcollectRear  - (ccpm_mixer_ * __PITCH_TelevatorR) + __ROLL_TaileronRear;    // aRearServo
    servos_rear[2] = 0 - TcollectRear  + (ccpm_mixer_ * __PITCH_TelevatorR) + __ROLL_TaileronRear;    // bRearServo
    servos_rear[3] = 0 + TcollectRear  + __PITCH_TelevatorR;

  }
  else if (model_ == TANDEM_COPYCAT) {
    // CCPM120 Heli, setup as 210 configuration for testing

    // Collective
    float A = mixer_in[COLL] * collective_range_;
    float B = -mixer_in[COLL] * collective_range_;
    float C = mixer_in[COLL] * collective_range_;

    // Roll
    B += mixer_in[ROLL] * 0.8660254038f; //cos(30);
    C += mixer_in[ROLL] * 0.8660254038f; //cos(30);

    // Pitch
    A -= mixer_in[PITCH];
    B -= mixer_in[PITCH] * 0.5f; //sin(30);
    C += mixer_in[PITCH] * 0.5f; //sin(30);

    servos_front[1] = A;
    servos_front[2] = B;
    servos_front[3] = C;
    servos_front[4] = -mixer_in[YAW];

    // Duplicate Rear 'false' servo to match front
    servos_rear[1] = servos_front[1];
    servos_rear[2] = servos_front[2];
    servos_rear[3] = servos_front[3];
    servos_rear[4] = servos_front[4];
  }
  else {
    // not supported.
  }
}

/************************ (C) COPYRIGHT Avidrone Aerospace Inc. *****END OF FILE****/
