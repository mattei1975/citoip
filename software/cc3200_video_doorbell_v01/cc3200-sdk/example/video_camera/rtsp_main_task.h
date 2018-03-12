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

#ifndef __RTSP_MAIN_H__
#define __RTSP_MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "app_common.h"

/*!
 * Types of RTSP-Task messages
 */
typedef enum app_rtsp_msg_type
{
    RTSP_INVALID_MSG,
    RTSP_STATUS_MSG,
    RTSP_V_STOPPED,
    RTSP_CONNECTED_TO_NW,
    RTSP_DISCONNECT_FROM_AP,
    RTSP_SERVER_STARTED,
    RTSP_CLIENT_CONNECTED,
    RTSP_CLIENT_DISCONNECTED,
    RTSP_START_STREAMING,
    RTSP_STOP_STREAMING,

}app_rtsp_msg_type_e;

/*!
 * RTSP-Task message
 */
typedef struct rtsp_task_msg
{
    app_task_id_e       task_id;
    app_rtsp_msg_type_e msg_id;
    cc_s32              msg;

}app_rtsp_task_msg_s;

/*!
 */
typedef enum app_rtsp_rx_msg_type
{
    RTSP_SERVER_START,
    RTSP_SERVER_STOP

}app_rtsp_rx_msg_type_e;

/*!
 * RTSP Recv-Task Message
 */
typedef struct rtsp_recv_task_msg
{
    app_task_id_e task_id;
    app_rtsp_rx_msg_type_e    msg_id;

}app_rtsp_rx_task_msg_s;

/*!
 * Task entry function: Initializes the network processor, triggers the AP
 * connection process and fields the messages on 'app_rtsp_task_q'
 */
void app_main_task_entry(void *p_args);

/*!
 * Task entry function: Fields the messages on 'app_rtsp_rx_task_q'
 */
void app_rtsp_rx_task_entry(void *p_args);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __RTSP_MAIN_H__ */
