/*
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __AUDIO_TX_H__
#define __AUDIO_TX_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "app_common.h"

#define PAYLOAD_FMT_HEADER_SIZE     (128)
#define AUDIO_RECV_SIZE             (512)
#define MAX_AUDIO_BUFF_SIZE         (AUDIO_RECV_SIZE + PAYLOAD_FMT_HEADER_SIZE)

/*!
 * Audio Message Type
 */
typedef enum app_a_msg_type
{
    STATUS_MSG,
    START_A_STREAMING,
    STOP_A_STREAMING

}app_a_msg_type_e;

/*!
 * Audio Task Message
 */
typedef struct audio_task_msg
{
    app_task_id_e       task_id;
    app_a_msg_type_e    msg_id;
    cc_s32              msg;

}a_task_msg_s;

/*!
 * Audio Data
 */
typedef struct a_data
{
    cc_u8               a_data_buffer[MAX_AUDIO_BUFF_SIZE];
    ov_astream_info_s   astream_info;
    cc_u32              rx_pkt_size;

}a_data_s;

/*!
 *  Task entry function: Gets the A-Stream data, processes and sends it\
 *  over Wi-Fi
 */
void app_a_tx_task_entry(void *p_args);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __AUDIO_TX_H__ */
