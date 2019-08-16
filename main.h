/**
  ******************************************************************************
  * @file    main.h
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

#ifndef MAIN_H_
#define MAIN_H_

// ---- Public Interfaces ---- //
void ResetIterms(void);
void GenerateSpeed2AngleLUT(void);
void AltitudeUpdate(float alt_rate, float dT);
void HeadingUpdate(float heading_rate, float dT);
void CompassCalDone(void);
int  TakeoffControlModes(void);
int  GetMotorsState(void);
bool LidarOnline(void);
void NVIC_WatchdogHandler(void);
int getNodeVersionNum(int type, int nodeId);
void getNodeSerialNum(int type, int nodeId, uint32_t *pSerailNum);
void CompassCalDone(void);

// ---- Public Data ---- //

// ---- Public Macros ---- //
#define IN_THE_AIR(X) ( ( (( X ) > 0.2) && (GetMotorsState() == 1) ) ? 1 : 0 )

#define TURN_YAW_RATE_THRESHOLD  3.0f // threshold degrees per second requested that indicates that the UAV is turning

#endif /* MAIN_H_ */
