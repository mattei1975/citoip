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

#include <stdbool.h>

#include "simplelink.h"

#include "cc_pm.h"
#include "cc_types.h"
#include "cc_pm_ops.h"

#include "rtsp_main_task.h"
#include "video_tx_task.h"
#include "audio_tx_task.h"
#include "provisioning.h"
#include "get_av_task.h"
#include "cc_pal_app.h"
#include "app_common.h"
#include "app_config.h"
#include "rtcp_rtp.h"
#include "rtsp.h"
#include "osi.h"

#define APPLICATION_VERSION "1.0.0"

#define MULTICAST_IP        (0xE00000FB)
#define RTSP_PORT           (8554)
#define RECV_BUFF_SIZE      (1500)  /* Bytes */
#define SEND_BUFF_SIZE      (1500)  /* Bytes */
#define RTSP_RECV_TIMEOUT   (10)    /* Msecs */

/*!
 */
extern void lp3p0_setup_power_policy(int power_policy);

extern OsiMsgQ_t        app_av_task_q;
extern OsiMsgQ_t        app_rtsp_task_q;
extern OsiMsgQ_t        app_a_task_q;
extern OsiMsgQ_t        app_v_task_q;
extern OsiMsgQ_t        app_rtsp_rx_task_q;

/*!
 * RTSP Module config parameter
 */
cc_u8 app_rtsp_describe_msg[] =
{
"v=0\r\n\
o=- 1422939350613525 1 IN IP4 192.168.1.28\r\n\
s=streaming\r\n\
i=A/V\r\n\
t=0 0\r\n\
a=tool:CC3200R1\r\n\
a=type:broadcast\r\n\
a=control:*\r\n\
a=range:npt=0-\r\n\
a=x-qt-text-nam:H.264 Video\r\n\
a=x-qt-text-inf:Video.264\r\n\
m=video 0 RTP/AVP 96\r\n\
c=IN IP4 0.0.0.0\r\n\
b=AS:500\r\n\
a=rtpmap:96 H264/90000\r\n\
a=fmtp:96 packetization-mode=1;profile-level-id=42001F;sprop-parameter-sets=Z00AKKlQFAeyAAAA,aO48gAAAAAAAAAAA\r\n\
a=control:track1\r\n\
m=audio 0 RTP/AVP 97\r\n\
a=rtpmap:97 L16/11025/1\r\n\
a=fmtp:97 emphasis=50-15\r\n\
a=ptime:20\r\n\
a=control:track2\r\n"
};

/*!
 * RTSP Module configuration
 */
rtsp_session_config_s rtsp_session_config =
{
    {
        {
            6970,           /* Server RTP port for media-1 */
            6971            /* Server RTCP port for media-1 */
        },
        {
            6974,           /* Server RTP port  for media-2 */
            6975            /* Server RTCP port for media-2 */
        }
    },
    1000,                   /* Session timeout */
    0x43585430,             /* Session number */
    0,                      /* Time stamp will be updated by application */
    "224.0.0.251",          /* Preferred Multicast IP */
    CUR_DATE                /* Date */
};

rtp_sock_info_s         app_av_soc_info[NUM_OF_MEDIA_STREAMS];

static SlSockAddrIn_t   client_addr = {0};

static cc_u8            rtsp_rx_buffer[RECV_BUFF_SIZE] = {0};
static cc_u8            rtsp_tx_buffer[SEND_BUFF_SIZE] = {0};

static cc_s16           rtsp_listen_sd = -1;
static cc_s16           rtsp_accept_sd = -1;

static ov_v_framerate_e frame_rate;

/*!
 * OV configuration parameters
 */
ov_v_config_s v_config =
{
    V_AUTO,
    4,      /* Brightness */
    3,      /* Contrast */
    2,      /* Saturation */
    0,      /* Flip */
    0       /* Normal mode */
};

/*!
 * OV Audio Configuration
 */
ov_a_config_s a_config =
{
    A_PCM,
    A_11K
};

/*!
 * Displays the application banner
 */
static void display_banner(cc_s8 *p_app_name);

/*!
 * Initializes the OV module
 */
static cc_s32 init_ov();

/*!
 * Closes the RTSP Server
 */
static cc_s16 close_rtsp_conn();

/*!
 * Accepts RTSP client connections
 */
static cc_s16 accept_rtsp_client();

/*!
 * Listens for RTSP clients
 */
static cc_s16 listen_fo_rtsp_clients();

/*!
 * Closes the RTP sockets that were opened for the media streams
 */
static cc_s32 close_rtp_sockets(rtp_sock_info_s *const p_sock_info);

/*!
 * Opens RTP sockets for a media stream
 */
static cc_s32
open_rtp_sockets(rtp_sock_info_s        *const p_sock_info,
                 rtsp_session_config_s  *const p_session_config,
                 rtsp_setup_data_s      setup_data, cc_s32 idx);

/*!
 * Processes data from RTSP clients
 */
static cc_s32 process_rtsp_client_data(app_rtsp_task_msg_s *p_rtsp_msg);

/*!
 * Stops RTSP Rx Task
 */
static inline cc_s32
stop_rtsp_rx_task(app_rtsp_rx_task_msg_s *const p_msg_on_rtsp_rx_task_q);

/*!
 * Starts AV streaming
 */
static inline cc_s32
start_media_streaming(a_task_msg_s *const p_msg_on_a_task_q,\
                      v_task_msg_s *const p_msg_on_v_task_q);
/*!
 * Stops AV streaming
 */
static inline cc_s32
stop_media_streaming(a_task_msg_s *const p_msg_on_a_task_q,\
                     v_task_msg_s *const p_msg_on_v_task_q);

/*!
 * Starts RTSP Server
 */
static cc_s32
start_rtsp_server(app_rtsp_task_msg_s *const p_msg_on_rtsp_task_q,\
                  app_rtsp_rx_task_msg_s *const p_msg_on_rtsp_rx_task_q);

/*!
 * Stops RTSP Server
 */
static inline cc_s32
wait_for_msg_on_rtsp_task_q(app_rtsp_task_msg_s *const p_msg_on_rtsp_task_q);

/*!
 */
void app_main_task_entry(void *p_args)
{
    app_state_e             state = APP_STATE_START;

    app_rtsp_rx_task_msg_s  msg_on_rtsp_rx_task_q;
    app_rtsp_task_msg_s     msg_on_rtsp_task_q;
    v_task_msg_s            msg_on_v_task_q;
    a_task_msg_s            msg_on_a_task_q;

    cc_s32                  ret_val = -1;

    memset(&msg_on_rtsp_rx_task_q, 0, sizeof(msg_on_rtsp_rx_task_q));
    memset(&msg_on_rtsp_task_q, 0, sizeof(msg_on_rtsp_task_q));
    memset(&app_av_soc_info, 0, sizeof(app_av_soc_info));
    memset(&msg_on_v_task_q, 0, sizeof(msg_on_v_task_q));
    memset(&msg_on_a_task_q, 0, sizeof(msg_on_a_task_q));

    display_banner("CC3200-OV788 Video Doorbell Demo");

    /*!
     * Set the Power policy to LPDS
     */
    lp3p0_setup_power_policy(POWER_POLICY_STANDBY);
    cc_app_putoff_pm();

    ret_val = Network_IF_InitDriver();
    if(ret_val < 0) { ERR_PRINT(ret_val); goto exit_app_main_task_entry; }

    ret_val = ConnectToNetwork();
    if(ret_val < 0) { ERR_PRINT(ret_val); goto exit_app_main_task_entry; }

    app_set_state(APP_STATE_CONNECTED);

    ret_val = rtsp_init(app_rtsp_describe_msg);
    if(ret_val < 0) { ERR_PRINT(ret_val); goto exit_app_main_task_entry; }

    ret_val = rtp_init();
    if(ret_val < 0) { ERR_PRINT(ret_val); goto exit_app_main_task_entry; }

    ret_val = init_ov();
    if(ret_val < 0) { ERR_PRINT(ret_val); goto exit_app_main_task_entry; }

    /*!
     * Start RTSP Server
     */
    msg_on_rtsp_rx_task_q.task_id   = RTSP_TASK_ID;
    msg_on_rtsp_rx_task_q.msg_id    = RTSP_SERVER_START;
    osi_MsgQWrite(&app_rtsp_rx_task_q, &msg_on_rtsp_rx_task_q, OSI_NO_WAIT);

    /* Enable PM before entering the loop */
    //APK cc_app_resume_pm();

    /*!
     * Keep waiting for messages on this queue
     */
    while(OSI_OK == \
          osi_MsgQRead(&app_rtsp_task_q, &msg_on_rtsp_task_q, OSI_WAIT_FOREVER))
    {
        //APK - TBD - Check below condition - Required for audio as well?
        state = app_get_state();
        if( (APP_STATE_STREAMING != state) && \
            (RTSP_V_STOPPED == msg_on_rtsp_task_q.msg_id) && \
            (VIDEO_TASK_ID == msg_on_rtsp_task_q.task_id) )
        {
            cc_stop_timestamp_cnt();
            cc_app_resume_pm();
            continue;
        }

        switch(state)
        {
            case APP_STATE_START:
            {
                Report("System State: APP_STATE_START\r\n");
            }
            break;

            case APP_STATE_CONNECTING:
            {
                Report("System State: APP_STATE_CONNECTING\r\n");

                if(RTSP_CONNECTED_TO_NW == msg_on_rtsp_task_q.msg_id)
                {
                    app_set_state(APP_STATE_CONNECTED);

                    /* Trigger the RTSP-Recv task */
                    msg_on_rtsp_rx_task_q.task_id   = RTSP_TASK_ID;
                    msg_on_rtsp_rx_task_q.msg_id    = RTSP_SERVER_START;
                    osi_MsgQWrite(&app_rtsp_rx_task_q, &msg_on_rtsp_rx_task_q,\
                                  OSI_NO_WAIT);
                }
            }
            break;

            case APP_STATE_CONNECTED:
            {
                Report("System State: APP_STATE_INIT_DONE_CONNECTED\r\n");

                if(RTSP_DISCONNECT_FROM_AP == msg_on_rtsp_task_q.msg_id)
                {
                    app_set_state(APP_STATE_CONNECTING);
                    stop_rtsp_rx_task(&msg_on_rtsp_rx_task_q);
                }
                else if(RTSP_SERVER_STARTED == msg_on_rtsp_task_q.msg_id)
                {
                    app_set_state(APP_STATE_RTSP_SERVER_STARTED);
                }
            }
            break;

            case APP_STATE_RTSP_SERVER_STARTED:
            {
                Report("System State: APP_STATE_RTSP_SERVER_STARTED\r\n");

                if(RTSP_DISCONNECT_FROM_AP == msg_on_rtsp_task_q.msg_id)
                {
                    app_set_state(APP_STATE_CONNECTING);
                    stop_rtsp_rx_task(&msg_on_rtsp_rx_task_q);
                }
                else if(msg_on_rtsp_task_q.msg_id == RTSP_CLIENT_CONNECTED)
                {
                    app_set_state(APP_STATE_RTSP_CLIENT_CONNECTED);
                }
            }
            break;

            case APP_STATE_RTSP_CLIENT_CONNECTED:
            {
                Report("System State: APP_STATE_RTSP_CLIENT_CONNECTED\r\n");

                if(msg_on_rtsp_task_q.msg_id == RTSP_START_STREAMING)
                {
                    cc_app_putoff_pm();

                    cc_gpio_config_high(GPIO_WLAN_ON);
                    cc_start_timestamp_cnt();
                    config_interface();

                    start_media_streaming(&msg_on_a_task_q, &msg_on_v_task_q);

                    app_set_state(APP_STATE_STREAMING);
                }
                else if(msg_on_rtsp_task_q.msg_id == RTSP_DISCONNECT_FROM_AP)
                {
                    app_set_state(APP_STATE_CONNECTING);
                    stop_rtsp_rx_task(&msg_on_rtsp_rx_task_q);
                }
            }
            break;

            case APP_STATE_STREAMING:
            {
                Report("System State: APP_STATE_STREAMING\r\n");

                if(msg_on_rtsp_task_q.msg_id == RTSP_STOP_STREAMING)
                {
                    stop_media_streaming(&msg_on_a_task_q, &msg_on_v_task_q);
                    app_set_state(APP_STATE_RTSP_CLIENT_CONNECTED);
                }
                else if(msg_on_rtsp_task_q.msg_id == RTSP_CLIENT_DISCONNECTED)
                {
                    stop_media_streaming(&msg_on_a_task_q, &msg_on_v_task_q);
                    app_set_state(APP_STATE_RTSP_SERVER_STARTED);
                }
                else if(msg_on_rtsp_task_q.msg_id == RTSP_DISCONNECT_FROM_AP)
                {
                    app_set_state(APP_STATE_CONNECTING);
                    stop_rtsp_rx_task(&msg_on_rtsp_rx_task_q);
                    stop_media_streaming(&msg_on_a_task_q, &msg_on_v_task_q);

                    app_set_state(APP_STATE_RTSP_SERVER_STARTED);
                }
            }
            break;
        } /* switch */
    }/* while */

exit_app_main_task_entry:
    LOOP_FOREVER();
}

/**/
void app_rtsp_rx_task_entry(void *p_args)
{
    app_rtsp_rx_task_msg_s      msg_on_rtsp_rx_task_q;
    app_rtsp_task_msg_s         msg_on_rtsp_task_q;

    memset(&msg_on_rtsp_rx_task_q, 0, sizeof(msg_on_rtsp_rx_task_q));
    memset(&msg_on_rtsp_task_q, 0, sizeof(msg_on_rtsp_task_q));

    /*!
     * Wait for messages on this queue
     */
    while(OSI_OK == \
          osi_MsgQRead(&app_rtsp_rx_task_q, &msg_on_rtsp_rx_task_q,\
                       OSI_WAIT_FOREVER))
    {
        if(RTSP_SERVER_START == msg_on_rtsp_rx_task_q.msg_id)
        {
            /*!
             * Ignore the return value, and keep waiting for the next
             * 'RTSP_SERVER_START'
             */
            start_rtsp_server(&msg_on_rtsp_task_q, &msg_on_rtsp_rx_task_q);
        }
    }

    /**/
    LOOP_FOREVER();
}

/**/
static void display_banner(cc_s8 *p_app_name)
{

    Report("\n\n\n\r");
    Report("\t\t ****************************************************************\n\r");
    Report("\t\t    %s Application - v%s      \n\r", p_app_name, APPLICATION_VERSION);
    Report("\t\t ****************************************************************\n\r");
    Report("\n\n\n\r");
}

/**/
static cc_s32 process_rtsp_client_data(app_rtsp_task_msg_s *p_rtsp_msg)
{
    rtsp_session_config_s   *p_session_config = NULL;
    rtsp_pkt_info_s         rtsp_pkt_info;

    cc_s32      ret_val         = -1;
    cc_u16      bytes_to_rx     = 0;
    cc_s16      bytes_to_tx     = -1;
    cc_s16      idx             = -1;

    memset(rtsp_rx_buffer, 0, RECV_BUFF_SIZE);
    memset(&rtsp_pkt_info, 0, sizeof(rtsp_pkt_info));

    ret_val = sl_Recv(rtsp_accept_sd, rtsp_rx_buffer, RECV_BUFF_SIZE, 0);
    if(ret_val <= 0 && ret_val != SL_EAGAIN)
    {
        Report("RTSP client disconnected\r\n");

        p_rtsp_msg->task_id = RTSP_RECV_TASK_ID;
        p_rtsp_msg->msg_id = RTSP_CLIENT_DISCONNECTED;
        osi_MsgQWrite(&app_rtsp_task_q, p_rtsp_msg, OSI_NO_WAIT);

        /* If RTP/RTCP sockets are open, close them */
        for(idx = 0; idx < NUM_OF_MEDIA_STREAMS; idx++)
        {
            ret_val = close_rtp_sockets(&app_av_soc_info[idx]);
            if(ret_val < 0) ERR_PRINT(ret_val);
        }

        sl_Close(rtsp_accept_sd);
        rtsp_accept_sd = -1;
    }
    else if (SL_EAGAIN == ret_val)
    {
        /* No data - Ignore */
    }
    else
    {
        p_session_config = &rtsp_session_config;

        /* Parse data */
        bytes_to_rx                 = ret_val;
        p_session_config->timestamp = 0;

        memset(rtsp_tx_buffer, 0, sizeof(rtsp_tx_buffer));
        ret_val = rtsp_packet_parser(rtsp_rx_buffer, bytes_to_rx,\
                                     p_session_config, SEND_BUFF_SIZE,\
                                     rtsp_tx_buffer, &rtsp_pkt_info);
        if(ret_val <= 0)
        {
            ERR_PRINT(ret_val);
        }
        else
        {
            bytes_to_tx = ret_val;
            ret_val     = sl_Send(rtsp_accept_sd, rtsp_tx_buffer, bytes_to_tx, 0);
            if(ret_val <= 0) ERR_PRINT(ret_val);

            if(SETUP == rtsp_pkt_info.pkt_type)
            {
                /* Setup request */
                ret_val = \
                open_rtp_sockets(&app_av_soc_info[rtsp_pkt_info.setup_count],\
                                 p_session_config,                           \
                                 rtsp_pkt_info.pkt_response.setup_data[(rtsp_pkt_info.setup_count)],\
                                 rtsp_pkt_info.setup_count);
                if(ret_val < 0)
                {
                    ERR_PRINT(ret_val);
                }
            }
            else if(PLAY == rtsp_pkt_info.pkt_type)
            {
                /* Start Streaming */
                p_rtsp_msg->task_id     = RTSP_RECV_TASK_ID;
                p_rtsp_msg->msg_id      = RTSP_START_STREAMING;
                osi_MsgQWrite(&app_rtsp_task_q, p_rtsp_msg, OSI_NO_WAIT);
            }
            else if(TEARDOWN == rtsp_pkt_info.pkt_type)
            {
                /* Client Disconnected */
                p_rtsp_msg->task_id     = RTSP_RECV_TASK_ID;
                p_rtsp_msg->msg_id      = RTSP_STOP_STREAMING;
                osi_MsgQWrite(&app_rtsp_task_q, p_rtsp_msg, OSI_NO_WAIT);

                for(idx = 0; idx < NUM_OF_MEDIA_STREAMS; idx++)
                {
                    ret_val = close_rtp_sockets(&app_av_soc_info[idx]);
                    if(ret_val < 0)
                    {
                        ERR_PRINT(ret_val);
                    }
                }
            }
        }

        memset(rtsp_rx_buffer, 0, RECV_BUFF_SIZE);
    }

    return 0;
}

/**/
static cc_s16 listen_fo_rtsp_clients()
{
    SlSockAddrIn_t  local_addr  = {0};

    cc_u16          addr_size   = 0;
    cc_s16          sock_id     = 0;
    cc_s16          ret_val     = 0;

    local_addr.sin_family       = SL_AF_INET;
    local_addr.sin_port         = sl_Htons((cc_u16)RTSP_PORT);
    local_addr.sin_addr.s_addr  = 0;

    sock_id = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if(sock_id < 0 )
    {
        ERR_PRINT(sock_id);
        return sock_id;
    }

    addr_size   = sizeof(SlSockAddrIn_t);
    ret_val     = sl_Bind(sock_id, (SlSockAddr_t *)&local_addr, addr_size);
    if(ret_val < 0)
    {
        sl_Close(sock_id);
        ERR_PRINT(ret_val);
        return ret_val;
    }

    ret_val = sl_Listen(sock_id, 0);
    if(ret_val < 0)
    {
        sl_Close(sock_id);
        ERR_PRINT(ret_val);
        return ret_val;
    }

    return sock_id;
}

/**/
cc_s16 accept_rtsp_client()
{
    struct SlTimeval_t  time_val    = {0};
    cc_u16              addr_size = 0;
    cc_s16              client_sock_id = 0;

    addr_size       = sizeof(SlSockAddrIn_t);
    client_sock_id  = sl_Accept(rtsp_listen_sd,\
                                (struct SlSockAddr_t *)&client_addr,\
                                (SlSocklen_t*)&addr_size);
    if(client_sock_id < 0)
    {
        sl_Close(rtsp_listen_sd);
        ERR_PRINT(client_sock_id);
    }
    else
    {
        /* Configure the Recv Time out */
        time_val.tv_sec = RTSP_RECV_TIMEOUT;
        time_val.tv_usec = 0;
        sl_SetSockOpt(client_sock_id, SL_SOL_SOCKET, SL_SO_RCVTIMEO,\
                      (_u8 *)&time_val, sizeof(time_val));
    }

    return client_sock_id;
}

/**/
static cc_s16 close_rtsp_conn()
{
    if(rtsp_accept_sd > 0)
    {
        /* Close the client socket */
        sl_Close(rtsp_accept_sd);
    }

    /* Close the server socket  */
    sl_Close(rtsp_listen_sd);

    rtsp_listen_sd = -1;
    rtsp_accept_sd = -1;

    return 0;

}

/**/
static cc_s32 close_rtp_sockets(rtp_sock_info_s *p_sock_info)
{
    if(!(p_sock_info->rtp_sock_id < 0))
    {
        sl_Close(p_sock_info->rtp_sock_id);
        Report("Media/RTP socket [%d] closed..!\n\r", p_sock_info->rtp_sock_id);

        p_sock_info->rtp_sock_id = -1;
    }

    if(!(p_sock_info->rtcp_sock_id < 0))
    {
        sl_Close(p_sock_info->rtcp_sock_id);
        //Report("RTCP socket [%d] closed\r\n", p_sock_info->rtcp_sock_id);

        p_sock_info->rtcp_sock_id = -1;

    }

    rtsp_release_setup();
    return 0;
}

/**/
static cc_s32 \
open_rtp_sockets(rtp_sock_info_s *p_sock_info,              \
                 rtsp_session_config_s *p_session_config,   \
                 rtsp_setup_data_s setup_data, cc_s32 idx)
{
    SlSockNonblocking_t sock_opt    = {0};
    SlSockAddrIn_t      local_addr  = {0};
    cc_s32              ret_val     = 0;

    /* Create RTP socket */
    p_sock_info->rtp_addr.sin_family    = SL_AF_INET;
    p_sock_info->rtp_addr.sin_port      = sl_Htons((_u16)setup_data.rtp_port);
    if(setup_data.is_multicast == 1)
    {
        /* Multicast IP here */
        p_sock_info->rtp_addr.sin_addr.s_addr = MULTICAST_IP;
    }
    else
    {
        /* Client IP here*/
        p_sock_info->rtp_addr.sin_addr.s_addr = client_addr.sin_addr.s_addr;
    }

    p_sock_info->addr_size      = sizeof(SlSockAddrIn_t);
    p_sock_info->rtp_sock_id    = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);
    if(p_sock_info->rtp_sock_id < 0 )
    {
        ERR_PRINT(p_sock_info->rtp_sock_id);
        return p_sock_info->rtp_sock_id;
    }

    Report("Media/RTP socket [%d] opened..!\n\r", p_sock_info->rtp_sock_id);

    local_addr.sin_family       = SL_AF_INET;
    local_addr.sin_port         = sl_Htons((_u16)p_session_config->stream_port[idx].rtp_port);
    local_addr.sin_addr.s_addr  = htonl(INADDR_ANY);

    ret_val = sl_Bind(p_sock_info->rtp_sock_id,     \
                      (SlSockAddr_t *)&local_addr,  \
                      p_sock_info->addr_size);
    if( ret_val < 0 )
    {
        ERR_PRINT(ret_val);
        return ret_val;
    }

    sock_opt.NonblockingEnabled = 1;

    ret_val = sl_SetSockOpt(p_sock_info->rtp_sock_id,           \
                            SL_SOL_SOCKET,SL_SO_NONBLOCKING,    \
                            (_u8 *)&sock_opt, sizeof(sock_opt));
    if(ret_val < 0) ERR_PRINT(ret_val);

    /* Create RTCP socket */
    p_sock_info->rtcp_addr.sin_family   = SL_AF_INET;
    p_sock_info->rtcp_addr.sin_port     = sl_Htons((_u16)setup_data.rtcp_port);
    if(setup_data.is_multicast == 1)
    {
       p_sock_info->rtcp_addr.sin_addr.s_addr = MULTICAST_IP;
    }
    else
    {
        p_sock_info->rtcp_addr.sin_addr.s_addr = client_addr.sin_addr.s_addr;
    }

    p_sock_info->addr_size = sizeof(SlSockAddrIn_t);

    p_sock_info->rtcp_sock_id = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, 0);
    if( p_sock_info->rtcp_sock_id < 0 )
    {
        ERR_PRINT(p_sock_info->rtcp_sock_id);
        return p_sock_info->rtcp_sock_id;
    }

    local_addr.sin_family       = SL_AF_INET;
    local_addr.sin_port         = sl_Htons((_u16)p_session_config->stream_port[idx].rtcp_port);
    local_addr.sin_addr.s_addr  = htonl(INADDR_ANY);

    ret_val = sl_Bind(p_sock_info->rtcp_sock_id, (SlSockAddr_t *)&local_addr,\
                      p_sock_info->addr_size);
    if( ret_val < 0 )
    {
        ERR_PRINT(ret_val);
        return ret_val;
    }

    ret_val = sl_SetSockOpt(p_sock_info->rtcp_sock_id, SL_SOL_SOCKET,\
                            SL_SO_NONBLOCKING, (_u8 *)&sock_opt, sizeof(sock_opt));
    if(ret_val < 0) ERR_PRINT(ret_val);

    return 0;
}

/**/
static cc_s32 init_ov()
{
    app_rtsp_task_msg_s     msg_on_rtsp_task_q;
    av_task_msg_s           msg_on_av_task_q;

    cc_s32                  ret_val = -1;

    msg_on_av_task_q.task_id    = RTSP_TASK_ID;
    msg_on_av_task_q.msg_id     = INIT_OV788;
    osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);

    /**/
    ret_val = wait_for_msg_on_rtsp_task_q(&msg_on_rtsp_task_q);
    if(ret_val < 0) { ERR_PRINT(ret_val); goto exit_init_ov; }

    msg_on_av_task_q.task_id    = RTSP_TASK_ID;
    msg_on_av_task_q.msg_id     = V_CONFIG_SENSOR;
    msg_on_av_task_q.p_data     = (void *)&v_config;
    osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);

    /**/
    ret_val = wait_for_msg_on_rtsp_task_q(&msg_on_rtsp_task_q);
    if(ret_val < 0) { ERR_PRINT(ret_val); goto exit_init_ov; }

    msg_on_av_task_q.task_id    = RTSP_TASK_ID;
    msg_on_av_task_q.msg_id     = V_SET_FRAME_RATE;
    frame_rate                  = VIDEO_FRAME_RATE;
    msg_on_av_task_q.p_data     = (void *)&frame_rate;
    osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);

    /**/
    ret_val = wait_for_msg_on_rtsp_task_q(&msg_on_rtsp_task_q);
    if(ret_val < 0) { ERR_PRINT(ret_val); goto exit_init_ov; }

    msg_on_av_task_q.task_id    = RTSP_TASK_ID;
    msg_on_av_task_q.msg_id     = A_CONFIG_AUDIO;
    msg_on_av_task_q.p_data     = (void *)&a_config;
    osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);

    /**/
    ret_val = wait_for_msg_on_rtsp_task_q(&msg_on_rtsp_task_q);
    if(ret_val < 0) { ERR_PRINT(ret_val); goto exit_init_ov; }

exit_init_ov:
    return ret_val;
}

/**/
static inline cc_s32
wait_for_msg_on_rtsp_task_q(app_rtsp_task_msg_s *const p_msg_on_rtsp_task_q)
{
    do{
        /*!
         * Wait for response
         */
        osi_MsgQRead(&app_rtsp_task_q, p_msg_on_rtsp_task_q, OSI_WAIT_FOREVER);
    }while(RTSP_STATUS_MSG != p_msg_on_rtsp_task_q->msg_id);

    return p_msg_on_rtsp_task_q->msg;
}

/**/
static inline cc_s32
stop_rtsp_rx_task(app_rtsp_rx_task_msg_s *const p_msg_on_rtsp_rx_task_q)
{
    /* Stop the RTSP recv task */
    p_msg_on_rtsp_rx_task_q->task_id   = RTSP_TASK_ID;
    p_msg_on_rtsp_rx_task_q->msg_id    = RTSP_SERVER_STOP;
    osi_MsgQWrite(&app_rtsp_rx_task_q, p_msg_on_rtsp_rx_task_q,\
                  OSI_NO_WAIT);

    return 0;
}

/**/
static inline cc_s32
stop_media_streaming(a_task_msg_s *const p_msg_on_a_task_q,\
                     v_task_msg_s *const p_msg_on_v_task_q)
{
    p_msg_on_a_task_q->task_id     = RTSP_TASK_ID;
    p_msg_on_a_task_q->msg_id      = STOP_A_STREAMING;
    osi_MsgQWrite(&app_a_task_q, p_msg_on_a_task_q, OSI_NO_WAIT);

    p_msg_on_v_task_q->task_id     = RTSP_TASK_ID;
    p_msg_on_v_task_q->msg_id      = STOP_V_STREAMING;
    osi_MsgQWrite(&app_v_task_q, p_msg_on_v_task_q, OSI_NO_WAIT);

    return 0;
}

/**/
static inline cc_s32
start_media_streaming(a_task_msg_s *const p_msg_on_a_task_q,\
                      v_task_msg_s *const p_msg_on_v_task_q)
{
    if(!(app_av_soc_info[0].rtp_sock_id < 0))
    {
        p_msg_on_v_task_q->task_id = RTSP_TASK_ID;
        p_msg_on_v_task_q->msg_id = START_V_STREAMING;
        osi_MsgQWrite(&app_v_task_q, p_msg_on_v_task_q, OSI_NO_WAIT);
    }

    if(!(app_av_soc_info[1].rtp_sock_id < 0))
    {
        p_msg_on_a_task_q->task_id = RTSP_TASK_ID;
        p_msg_on_a_task_q->msg_id = START_A_STREAMING;
        osi_MsgQWrite(&app_a_task_q, p_msg_on_a_task_q, OSI_NO_WAIT);
    }

    return 0;
}

/**/
static cc_s32
start_rtsp_server(app_rtsp_task_msg_s *const p_msg_on_rtsp_task_q,\
                  app_rtsp_rx_task_msg_s *const p_msg_on_rtsp_rx_task_q)
{
    cc_s32  ret_val = 0;
    cc_s8   idx     = 0;

    Report("Listening for RTSP clients..!\r\n");

    rtsp_listen_sd = listen_fo_rtsp_clients();
    if(rtsp_listen_sd < 0) { ERR_PRINT(rtsp_listen_sd); return rtsp_listen_sd; }

    p_msg_on_rtsp_task_q->task_id  = RTSP_RECV_TASK_ID;
    p_msg_on_rtsp_task_q->msg_id   = RTSP_SERVER_STARTED;
    osi_MsgQWrite(&app_rtsp_task_q, p_msg_on_rtsp_task_q, OSI_NO_WAIT);
    while(1)
    {
        /* Wait for client connections */
        if(rtsp_accept_sd <= 0)
        {
            rtsp_accept_sd = accept_rtsp_client();
            if(rtsp_accept_sd <= 0)
            {
                ERR_PRINT(rtsp_accept_sd);
            }
            else
            {
                Report("An RTSP client has connected\r\n");

                p_msg_on_rtsp_task_q->task_id  = RTSP_RECV_TASK_ID;
                p_msg_on_rtsp_task_q->msg_id   = RTSP_CLIENT_CONNECTED;
                osi_MsgQWrite(&app_rtsp_task_q, p_msg_on_rtsp_task_q, OSI_NO_WAIT);
            }
        }
        else
        {
            process_rtsp_client_data(p_msg_on_rtsp_task_q);
        }

        /* Check for msg in queue */
        osi_MsgQRead(&app_rtsp_rx_task_q, p_msg_on_rtsp_rx_task_q, OSI_NO_WAIT);
        if(RTSP_SERVER_STOP == p_msg_on_rtsp_rx_task_q->msg_id)
        {
            /* If RTP/RTCP sockets are open, close them */
            for(idx = 0; idx < NUM_OF_MEDIA_STREAMS; idx++)
            {
                ret_val = close_rtp_sockets(&app_av_soc_info[idx]);
                if(ret_val < 0) ERR_PRINT(ret_val);
            }

            Report("Closing RTSP Sockets\r\n");
            ret_val = close_rtsp_conn();
            if(ret_val < 0)
            {
                ERR_PRINT(ret_val);
                return ret_val;
            }

            break;
        }
    }

    return ret_val;
}
