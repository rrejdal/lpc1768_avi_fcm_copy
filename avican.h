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
 *  - bits[10..8] Node TYPE (3 bits, 0-7 Node Types)
 *  - bits[7..6]  Node ID   (2 bits, 0-3 Node Ids, per Node Type)
 *  - bits[5..3]  Msg Seq   (3 bits, 0-7 Message Seq Num (per Msg Id))
 *  - bits[2..0]  Msg ID    (3 bits, 0-7 message ids per node type)
 */

#define AVI_CAN_NODE_TYPE_MASK  0x700
#define AVI_CAN_NODE_TYPE_SHIFT 8

#define AVI_CAN_NODE_MASK  0xC0
#define AVI_CAN_NODE_SHIFT 6

#define AVI_CAN_SEQ_MASK   0x38
#define AVI_CAN_SEQ_SHIFT  3

#define AVI_CAN_MSGID_MASK  0x7
#define AVI_CAN_MSGID_SHIFT 0

#define AVI_CAN_ID(__TYPE__, __NODE__, __SEQ__, __MSG__) \
  ( ((__TYPE__) << AVI_CAN_NODE_TYPE_SHIFT) \
		  | ((__NODE__) << AVI_CAN_NODE_SHIFT) \
		  | ((__SEQ__) << AVI_CAN_SEQ_SHIFT) \
      | ((__MSG__) & AVI_CAN_MSGID_MASK) )

#define AVI_CAN_NODETYPE(__MSG__) (((__MSG__) & AVI_CAN_NODE_TYPE_MASK) >> AVI_CAN_NODE_TYPE_SHIFT)
#define AVI_CAN_NODEID(__MSG__) (((__MSG__) & AVI_CAN_NODE_MASK) >> AVI_CAN_NODE_SHIFT)
#define AVI_CAN_SEQID(__MSG__) (((__MSG__) & AVI_CAN_SEQ_MASK) >> AVI_CAN_SEQ_SHIFT)
#define AVI_CAN_MSGID(__MSG__) ((__MSG__) & AVI_CAN_MSGID_MASK)

typedef enum {
    AVI_NODETYPE_NONE  = 0,
    AVI_FCM_NODETYPE   = 1,
    AVI_SERVO_NODETYPE = 2,
    AVI_GPS_NODETYPE   = 3,
    AVI_PWR_NODETYPE   = 4,
    AVI_RSVD1_NODETYPE = 5,
    AVI_RSVD2_NODETYPE = 6,
    AVI_RSVD3_NODETYPE = 7,
} AVI_CAN_NODE_TYPE;

#define DEFAULT_NODE_ID 1
#define MAX_NODE_ID     15

typedef enum {
  AVI_MSGID_CTRL    = 0,
  AVI_MSGID_SDP     = 1,  // Setup Data Packet
  AVI_MSGID_CDP     = 2,  // Control Data Packet (From FCM)
  AVI_MSGID_PDP     = 3,  // Process Data Packet (To FCM)
  AVI_MSGID_GPS     = 4,
} AVI_CANBUS_COMMON_MSG_IDS;

typedef enum {
  AVI_SYNC = 0,
  AVI_ACK  = 7,
} AVI_CANBUS_SEQID_CTRL;

typedef enum {
  AVI_CFG          = 0,
  AVI_BOARDINFO_0  = 1,
  AVI_BOARDINFO_1  = 2,
  AVI_FAILSAFE_0_3 = 3,
  AVI_FAILSAFE_4_7 = 4,
} AVI_CANBUS_SEQID_SDP;

typedef enum {
  AVI_SERVOS_0_3  = 0,
  AVI_SERVOS_4_7  = 1,
} AVI_CANBUS_SEQID_CDP;

typedef enum {
  AVI_LIDAR  = 0,
  AVI_CL0  = 1,
  AVI_CL1  = 2,
  AVI_CL2  = 3,
  AVI_CL3  = 4,
  AVI_CL4  = 5,
  AVI_VI   = 6,
} AVI_CANBUS_SEQID_PDP;

typedef enum {
  AVI_GPS0  = 0,
  AVI_GPS1  = 1,
  AVI_GPS2  = 2,
  AVI_GPS3  = 3,
  AVI_GPS4  = 4,
  AVI_COMPASS = 5,
} AVI_CANBUS_SEQID_GPS;

#if 0
typedef enum {
    AVIDRONE_MSGID_SERVO_NONE    = 0,
    AVIDRONE_MSGID_SERVO_INFO    = 1,
    AVIDRONE_MSGID_SERVO_PN      = 2,
    AVIDRONE_MSGID_LIDAR         = 3,
    AVIDRONE_MSGID_CASTLE_0      = 4,
    AVIDRONE_MSGID_CASTLE_1      = 5,
    AVIDRONE_MSGID_CASTLE_2      = 6,
    AVIDRONE_MSGID_CASTLE_3      = 7,
    AVIDRONE_MSGID_CASTLE_4      = 8,
    AVIDRONE_MSGID_SERVO_CFG     = 9,
    AVIDRONE_MSGID_SERVO_ACK     = 10,
    AVIDRONE_MSGID_SERVO_LO_CTRL = 11,
    AVIDRONE_MSGID_SERVO_HI_CTRL = 12,
    AVIDRONE_MSGID_SERVO_MAX     = AVIDRONE_MSGID_SERVO_HI_CTRL,
} AVIDRONE_CAN_SERVO_MSG_IDS;

#define MAX_NUM_GPS_MSG 4

typedef enum {
    AVIDRONE_MSGID_GPS_NONE      = 0,
    AVIDRONE_MSGID_GPS_INFO      = 1,
    AVIDRONE_MSGID_GPS_PN        = 2,
    AVIDRONE_MSGID_GPS_SYNC      = 3,
    AVIDRONE_MSGID_GPS_0         = 4,   /* Info - Id, Sats fix, altitude */
    AVIDRONE_MSGID_GPS_1         = 5,   /* Lat/Lon */
    AVIDRONE_MSGID_GPS_2         = 6,   /* Speed ENU, PDOP */
    AVIDRONE_MSGID_GPS_3         = 7,   /* date/Time */
    AVIDRONE_MSGID_GPS_4         = 8,   /* glitch_flag, glitch_cnt, crc_err_cnt, msg_cnt */
    AVIDRONE_MSGID_GPS_CFG       = 9,
    AVIDRONE_MSGID_GPS_ACK       = 10,
    AVIDRONE_MSGID_COMPASS_XYZ   = 11,
    AVIDRONE_MSGID_GPS_MAX       = AVIDRONE_MSGID_COMPASS_XYZ,
} AVIDRONE_CAN_GPS_MSG_IDS;

typedef enum {
    AVIDRONE_MSGID_PWR_NONE     = 0,
    AVIDRONE_MSGID_PWR_INFO     = 1,
    AVIDRONE_MSGID_PWR_PN       = 2,
    AVIDRONE_PWR_MSGID_LIDAR    = 3,
    AVIDRONE_PWR_MSGID_CASTLE_0 = 4,
    AVIDRONE_PWR_MSGID_CASTLE_1 = 5,
    AVIDRONE_PWR_MSGID_CASTLE_2 = 6,
    AVIDRONE_PWR_MSGID_CASTLE_3 = 7,
    AVIDRONE_PWR_MSGID_CASTLE_4 = 8,
    AVIDRONE_MSGID_PWR_V_I      = 9,
    AVIDRONE_MSGID_PWR_CFG      = 10,
    AVIDRONE_MSGID_PWR_ACK      = 11,
    AVIDRONE_MSGID_PWR_LO_CTRL  = 12,
    AVIDRONE_MSGID_PWR_HI_CTRL  = 13,
    AVIDRONE_MSGID_PWR_SYNC     = 14,
    AVIDRONE_MSGID_PWR_COEFF    = 15,
    AVIDRONE_MSGID_PWR_MAX      = AVIDRONE_MSGID_PWR_SYNC,
} AVIDRONE_CAN_PWR_MSG_IDS;
#endif

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

