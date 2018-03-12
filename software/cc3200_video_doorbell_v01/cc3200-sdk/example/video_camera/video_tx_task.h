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

#ifndef __VIDEO_TX_H__
#define __VIDEO_TX_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "app_common.h"
#include "ov_sif_if.h"

#define PAYLOAD_FMT_HEADER_SIZE     (128)
#define VIDEO_RECV_SIZE             (10240)
#define MAX_VIDEO_BUFF_SIZE         (VIDEO_RECV_SIZE + PAYLOAD_FMT_HEADER_SIZE)
#define VIDEO_FRAME_RATE            (V_15FPS)

/*!
 */
typedef enum
{
    V_STATUS_MSG,
    START_V_STREAMING,
    STOP_V_STREAMING

}app_v_msg_type_e;

/*!
 */
typedef struct v_task_msg
{
    app_task_id_e       task_id;
    app_v_msg_type_e    msg_id;
    cc_s32              msg;

}v_task_msg_s;

/*!
 */
typedef struct v_data
{
    cc_u8               v_data_buffer[MAX_VIDEO_BUFF_SIZE];
    ov_vstream_info_s   vstream_info_current;
    ov_vstream_info_s   vstream_info_next;
    cc_u32              rx_pkt_size;

}v_data_s;

/*!
 */
void app_v_tx_task_entry(void *p_args);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __VIDEO_TX_H__ */
