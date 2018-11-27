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
    delete msg;
    msg = new afsi_msg;
    msg->hdr = new afsi_hdr;
}

bool afsi::checkCRC(uint8_t len,uint8_t c1, uint8_t c2){
    //-2 is added because otherwise the 2 CRC bytes are included
    for (int i=0; i<len-2; i++) {
        c1 += msg->payload[i];
        c2 += c1;
    }

    if (c1 != msg->crc[0]) {
        return false;
    }

    if (c2 != msg->crc[1]) {
        return false;
    }

    return true;
}

//State machine reading bytes into a buffer
static void afsi::trackMsgState(){
    switch (afsi_state){
        case AFSI_STATE_INIT:
            if (afsi_byte == AFSI_SNC_CH_1){
                afsi_buffer[afsi_index] = afsi_byte;
                afsi_index++;
                afsi_state = AFSI_STATE_SYNC;
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

            break;
        case AFSI_STATE_ID:
            afsi_buffer[afsi_index] = afsi_byte;
            afsi_index++;
            afsi_state = AFSI_STATE_LEN;

            break;
        case AFSI_STATE_LEN:
            if (afsi_index == 4){
                msg->hdr->len = (uint16_t)afsi_byte;
            }
            else if (afsi_index == 5){
                msg->hdr->len += ((uint16_t)afsi_byte<<8);
                afsi_state = AFSI_STATE_MSG;
                afsi_msgCnt = 0;
            }

            afsi_buffer[afsi_index] = afsi_byte;
            afsi_index++;

            break;
        case AFSI_STATE_MSG:
            if (afsi_msgCnt < msg->hdr->len+2){
                afsi_state = AFSI_STATE_DONE;
            }
            afsi_buffer[afsi_index] = afsi_byte;
            afsi_index++;
            afsi_msgCnt++;

            break;
    }
}

//Processes byte buffer into message
int afsi::parseMsg (){
    int totalLength = 1+1+1+1+2+msg->hdr->len + 2;

    msg->crc[0] = afsi_buffer[totalLength-1];
    msg->crc[1] = afsi_buffer[totalLength-2];

    if (!afsi::checkCRC(totalLength,msg->crc[0],msg->crc[1])){
        return -1;
    }


    //Populate information from buffer
    msg->hdr->sync1 = afsi_buffer[0];
    msg->hdr->sync2 = afsi_buffer[1];
    msg->hdr->msg_class = afsi_buffer[2];
    msg->hdr->id = afsi_buffer[3];
    for (int i=0; i<msg->hdr->len; i++){
        msg->payload[i] = afsi_buffer[4+i];
    }

    return 0;
}

static void afsi::processMsg(){
    switch(msg->hdr->msg_class){
        case (ASFI_){

        }
    }

}
