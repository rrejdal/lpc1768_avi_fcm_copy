/**
  ******************************************************************************
  * @file    avidrone_can.h
  * @author  AVIDRONE FIRMWARE Team
  * @brief   header file for Avidrone Can constants
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AVIDRONE_CAN_H
#define __AVIDRONE_CAN_H

/* Using CAN2.0B standard mode, providing 11 bit identifiers.
 * Avidrone Can node/msg assignment as follows:
 *
 *  - bits[10..8] Node TYPE (0.7 node Types)
 *  - bits[07..4] Node ID   (0.15 node ids per node type)
 *  - bits[03..0] Msg ID    (0..15 message ids per node type)
 */

#define AVIDRONE_CAN_NODE_TYPE_MASK     0x700
#define AVIDRONE_CAN_NODE_TYPE_SHIFT    8

#define AVIDRONE_CAN_NODE_MASK      0xF0
#define AVIDRONE_CAN_NODE_SHIFT     4

#define AVIDRONE_CAN_MSGID_MASK     0xF
#define AVIDRONE_CAN_MSGID_SHIFT    0

#define AVIDRONE_CAN_ID(__TYPE__,__NODE__,__MSG__) ( ((__TYPE__) << AVIDRONE_CAN_NODE_TYPE_SHIFT) \
                                                   | ((__NODE__) << AVIDRONE_CAN_NODE_SHIFT) \
                                                   | ((__MSG__) & AVIDRONE_CAN_MSGID_MASK) )

#define AVIDRONE_CAN_NODETYPE(__MSG__) (((__MSG__) & AVIDRONE_CAN_NODE_TYPE_MASK) >> AVIDRONE_CAN_NODE_TYPE_SHIFT)
#define AVIDRONE_CAN_NODEID(__MSG__) (((__MSG__) & AVIDRONE_CAN_NODE_MASK) >> AVIDRONE_CAN_NODE_SHIFT)
#define AVIDRONE_CAN_MSGID(__MSG__) ((__MSG__) & AVIDRONE_CAN_MSGID_MASK)

typedef enum {
    AVIDRONE_NODETYPE_NONE  = 0,
    AVIDRONE_FCM_NODETYPE   = 1,
    AVIDRONE_SERVO_NODETYPE = 2,
    AVIDRONE_GPS_NODETYPE   = 3,
    AVIDRONE_PWR_NODETYPE   = 4,
    AVIDRONE_RSVD1_NODETYPE = 5,
    AVIDRONE_RSVD2_NODETYPE = 6,
    AVIDRONE_RSVD3_NODETYPE = 7,
    AVIDRONE_NODETYPE_MAX   = AVIDRONE_PWR_NODETYPE,
} AVIDRONE_CAN_NODE_TYPE;

#define DEFAULT_NODE_ID 1
#define MAX_NODE_ID     15

typedef enum {
    AVIDRONE_MSGID_SERVO_NONE    = 0,
    AVIDRONE_MSGID_SERVO_INFO    = 1,
    AVIDRONE_MSGID_LIDAR         = 2,
    AVIDRONE_MSGID_CASTLE_0      = 3,
    AVIDRONE_MSGID_CASTLE_1      = 4,
    AVIDRONE_MSGID_CASTLE_2      = 5,
    AVIDRONE_MSGID_CASTLE_3      = 6,
    AVIDRONE_MSGID_CASTLE_4      = 7,
    AVIDRONE_MSGID_SERVO_CFG     = 8,
    AVIDRONE_MSGID_SERVO_ACK     = 9,
    AVIDRONE_MSGID_SERVO_LO_CTRL = 10,
    AVIDRONE_MSGID_SERVO_HI_CTRL = 11,
    AVIDRONE_MSGID_SERVO_MAX     = AVIDRONE_MSGID_SERVO_HI_CTRL,
} AVIDRONE_CAN_SERVO_MSG_IDS;

#define MAX_NUM_GPS_MSG 4

typedef enum {
    AVIDRONE_MSGID_GPS_NONE      = 0,
    AVIDRONE_MSGID_GPS_INFO      = 1,
    AVIDRONE_MSGID_GPS_SYNC      = 2,
    AVIDRONE_MSGID_GPS_0         = 3,   /* Info - Id, Sats fix, altitude */
    AVIDRONE_MSGID_GPS_1         = 4,   /* Lat/Lon */
    AVIDRONE_MSGID_GPS_2         = 5,   /* Speed ENU, PDOP */
    AVIDRONE_MSGID_GPS_3         = 6,   /* date/Time */
    AVIDRONE_MSGID_GPS_4         = 7,   /* glitch_flag, glitch_cnt, crc_err_cnt, msg_cnt */
    AVIDRONE_MSGID_GPS_CFG       = 8,
    AVIDRONE_MSGID_GPS_ACK       = 9,
    AVIDRONE_MSGID_COMPASS_XYZ   = 10,
    AVIDRONE_MSGID_GPS_MAX       = AVIDRONE_MSGID_COMPASS_XYZ,
} AVIDRONE_CAN_GPS_MSG_IDS;

typedef enum {
    AVIDRONE_MSGID_PWR_NONE     = 0,
    AVIDRONE_MSGID_PWR_INFO     = 1,
    AVIDRONE_PWR_MSGID_LIDAR    = 2,
    AVIDRONE_PWR_MSGID_CASTLE_0 = 3,
    AVIDRONE_PWR_MSGID_CASTLE_1 = 4,
    AVIDRONE_PWR_MSGID_CASTLE_2 = 5,
    AVIDRONE_PWR_MSGID_CASTLE_3 = 6,
    AVIDRONE_PWR_MSGID_CASTLE_4 = 7,
    AVIDRONE_MSGID_PWR_V_I      = 8,
    AVIDRONE_MSGID_PWR_CFG      = 9,
    AVIDRONE_MSGID_PWR_ACK      = 10,
    AVIDRONE_MSGID_PWR_LO_CTRL  = 11,
    AVIDRONE_MSGID_PWR_HI_CTRL  = 12,
    AVIDRONE_MSGID_PWR_SYNC     = 13,
    AVIDRONE_MSGID_PWR_COEFF    = 14,
    AVIDRONE_MSGID_PWR_MAX      = AVIDRONE_MSGID_PWR_SYNC,
} AVIDRONE_CAN_PWR_MSG_IDS;

// Canbus Servo/Power Node configurations
#define PWM_NO_CHANNEL  (0)
#define PWM_CHANNEL_1   (1 << 0)
#define PWM_CHANNEL_2   (1 << 1)
#define PWM_CHANNEL_3   (1 << 2)
#define PWM_CHANNEL_4   (1 << 3)
#define PWM_CHANNEL_5   (1 << 4)
#define PWM_CHANNEL_6   (1 << 5)
#define PWM_CHANNEL_7   (1 << 6)
#define PWM_CHANNEL_8   (1 << 7)
#define PWM_CHANNEL_1_8 (PWM_CHANNEL_1 | PWM_CHANNEL_2 | PWM_CHANNEL_3 | PWM_CHANNEL_4 \
                         | PWM_CHANNEL_5 | PWM_CHANNEL_6 | PWM_CHANNEL_7 | PWM_CHANNEL_8)

#define CALIBRATE_PWR_NODE       (1 << 5)
#define LIDAR_ACTIVE             (1 << 6)
#define LIVE_LINK_ACTIVE         (1 << 7)


#endif /* __AVIDRONE_CAN_H */

/************************ (C) COPYRIGHT Avidrone Aerospace Inc. *****END OF FILE****/

