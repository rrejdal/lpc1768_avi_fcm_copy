/**
  ******************************************************************************
  * @file    hardware.h
  * @author  AVIDRONE FIRMWARE Team
  * @brief   Provides hardware specific board definitions
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
#ifndef __HARDWARE_H_
#define __HARDWARE_H_

#include "PinNames.h"

#define LED_1       P1_18
#define LED_2       P1_19
#define LED_3       P1_21
#define LED_4       P1_23

#define CHANNEL_1   P2_0
#define CHANNEL_2   P2_1
#define CHANNEL_3   P2_2
#define CHANNEL_4   P2_3
#define CHANNEL_5   P2_4
#define CHANNEL_6   P2_5

#define USB_SCON    P2_9
#define ISP_EN      P2_10

#define I2C_SDA1    P0_0
#define I2C_SCL1    P0_1

#define TRGT_TXD    P0_2
#define TRGT_RXD    P0_3

#define CAN_RXD1    P0_4
#define CAN_TXD1    P0_5

#define MC_SD_CS    P0_15

#define MC_SP1_SCK  P0_7
#define MC_SP1_MISO P0_8
#define MC_SP1_MOSI P0_9

#define GPS_TX  P0_10
#define GPS_RX  P0_11

#define MC_BUTTON1  P1_30
#define MC_BUTTON2  P0_24
#define MC_LED      P1_31

#define XBUS_IN     P0_16

#define MC_LCD_CS   P0_6
#define MC_LCD_RST  P0_17
#define MC_LCD_DC   P0_18

#define LIDAR_PWM   P0_23

#define TELEM_TX    P0_25
#define TELEM_RX    P0_26

#define AFSI_TX     P0_10
#define AFSI_RX     P0_11

#define USB_DMINUS  P0_30
#define USB_DPLUS   P0_29

//-----------------------------------------------------//


#endif

/************************ (C) COPYRIGHT Avidrone Aerospace Inc. *****END OF FILE****/
