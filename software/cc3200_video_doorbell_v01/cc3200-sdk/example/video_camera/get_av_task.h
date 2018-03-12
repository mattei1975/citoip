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
#ifndef __GET_AV_TASK_H__
#define __GET_AV_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "app_common.h"

/*!
 * Error codes
 */
typedef enum app_av_status_code
{
    AV_ERROR_UNKNOWN            = -1,
    AV_ERROR_UNKNOWN_MSGID      = -2,
    STATUS_NULL_DATA_POINTER    = -3

}app_av_status_code_e;

/*!
 * Types of AV Messages
 */
typedef enum app_av_msg_type
{
    INIT_OV788,
    V_CONFIG_SENSOR,
    V_SET_FRAME_RATE,
    V_SET_BITRATE,
    V_SET_ZOOM,
    V_SET_MOTION_DETECT,
    A_CONFIG_AUDIO,
    A_SET_SAMPLE_RATE,
    V_ENABLE,
    V_DISABLE,
    V_GET_INFO,
    V_GET_DATA,
    A_ENABLE,
    A_DISABLE,
    A_GET_INFO,
    A_GET_DATA

}app_av_msg_type_e;

/*!
 * AV Task Message
 */
typedef struct av_task_msg
{
    app_task_id_e       task_id;
    app_av_msg_type_e   msg_id;
    void                *p_data;

}av_task_msg_s;

/*!
 * Get-AV Task's Entry Function
 */
void app_get_av_task_entry(void *p_args);

#ifdef __cplusplus
}
#endif  /* __cplusplus */
#endif  /* __GET_AV_TASK_H__ */
