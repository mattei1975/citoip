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

#ifndef __APP_COMMON_H__
#define __APP_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "cc_pal_app.h"

#ifdef DEBUG_PRINT
extern int Report(const char *format, ...);
#else /* DEBUG_PRINT */
#define Report(...)
#endif /* DEBUG_PRINT */

#define ERR_PRINT(x) Report("Error [%d] at line [%d] in function [%s]  \n\r", x, __LINE__, __FUNCTION__)

#define ASSERT_ON_ERROR(error_code) \
            {\
                /* Handling the error-codes is specific to the application */ \
                if (error_code < 0) \
                { \
                    ERR_PRINT(error_code); \
                    return error_code; \
                } \
                /* else, continue w/ execution */ \
            }

#define LOOP_FOREVER() \
            {\
                while(1); \
            }


/*!
 * Sender-task's ID
 */
typedef enum
{
    ASYNC_EVENT_HANDLER,

    AV_TASK_ID,
    AUDIO_TASK_ID,
    VIDEO_TASK_ID,
    RTSP_TASK_ID,
    RTSP_RECV_TASK_ID,

    MAX_TASK_ID

}app_task_id_e;

/*!
 * Application's state
 */
typedef enum
{
    APP_STATE_START,
    APP_STATE_CONNECTING,
    APP_STATE_CONNECTED,
    APP_STATE_RTSP_SERVER_STARTED,
    APP_STATE_RTSP_CLIENT_CONNECTED,
    APP_STATE_STREAMING,

    APP_STATE_MAX

}app_state_e;

/*!
 * Socket information for an RTSP media stream.!
 */
typedef struct rtp_sock_info
{
    cc_s32          rtp_sock_id;
    SlSockAddrIn_t  rtp_addr;

    cc_s32          rtcp_sock_id;
    SlSockAddrIn_t  rtcp_addr;

    cc_u16          addr_size;
    cc_u8           is_multicast;

}rtp_sock_info_s;

/*!
    \brief      Sets the application's state
    \param      state - Next state
    \return     None
    \note
*/
void app_set_state(app_state_e const state);

/*!
    \brief      Gets the application's state
    \param      None
    \return     Returns the current state of the application
    \note
*/
app_state_e app_get_state();

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __APP_COMMON_H__ */
