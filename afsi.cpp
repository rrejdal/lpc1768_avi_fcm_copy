/**
  ******************************************************************************
  * @file    afsi.cpp
  * @author  AVIDRONE FIRMWARE Team
  * @brief   Provides support for AVI-FCM Serial Interface
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
#include <stddef.h>
#include <stdio.h>
#include "afsi.h"
#include "defines.h"

afsi::afsi(RawSerial *m_serial){
    serial = NULL;
    hfc = NULL;
    pConfig = NULL;
}

void afsi::clearBuffer(){
    afsi_index = 0;
    for (int i = 0;i<200;i++){
        afsi_buffer[i] = 0;
    }
}

void afsi::deleteMsg(){
    delete msg->hdr;
    delete msg->payload;
    delete msg;
    msg = new afsi_msg;
    msg->hdr = new afsi_hdr;
}

bool afsi::CheckCRC(){
    int len = sizeof(msg->data);
    for (int i=0; i<sizeof(msg->data); i++) {
        msg->CK_A = msg->CK_A + msg->data[i];
        msg->CK_B = msg->CK_B + msg->CK_A;
    }

    if (msg->CK_A != msg->data[len+0]) {
        return false;
    }

    if (msg->CK_B != msg->data[len+1]) {
        return false;
    }

    return true;
}

static void afsi::trackMsgState(){
    switch (afsi_state)
    {
        case AFSI_STATE_INIT:
            if (afsi_byte == AFSI_SNC_CH_1){
                afsi_buffer[afsi_index] = afsi_byte;
                afsi_index++;
                afsi_state = AFSI_STATE_SYNC;
                msg->hdr->sync1 = afsi_byte;
            }
            else{
                clearBuffer();
                deleteMsg();
            }
            break;
        case AFSI_STATE_SYNC:
            if (afsi_byte == AFSI_SNC_CH_2){
                afsi_buffer[afsi_index] = afsi_byte;
                afsi_index++;
                afsi_state = AFSI_STATE_CLASS;
                msg->hdr->sync2 = afsi_byte;
            }
            else{
                clearBuffer();
                deleteMsg();
            }
            break;
        case AFSI_STATE_CLASS:
            afsi_buffer[afsi_index] = afsi_byte;
            afsi_index++;
            afsi_state = AFSI_STATE_ID;
            msg->hdr->msg_class = afsi_byte;

            break;
        case AFSI_STATE_ID:
            afsi_buffer[afsi_index] = afsi_byte;
            afsi_index++;
            afsi_state = AFSI_STATE_LEN;
            msg->hdr->id = afsi_byte;

            break;
        case AFSI_STATE_LEN:
            afsi_buffer[afsi_index] = afsi_byte;
            afsi_index++;
            afsi_state = AFSI_STATE_MSG;
            msg->hdr->len = afsi_byte;
            afsi_msgCnt = 0;

            break;
        case AFSI_STATE_MSG:
            if (afsi_msgCnt < msg->hdr->len + 2)
            afsi_buffer[afsi_index] = afsi_byte;
            afsi_index++;
            afsi_state = AFSI_STATE_CLASS;
            msg->hdr->sync2 = afsi_byte;
    }
}
