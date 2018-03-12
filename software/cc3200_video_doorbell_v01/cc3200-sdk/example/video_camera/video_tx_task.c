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
#include "simplelink.h"

#include "video_tx_task.h"
#include "rtsp_main_task.h"
#include "cc_pal_app.h"
#include "get_av_task.h"
#include "app_common.h"
#include "app_config.h"
#include "rtcp_rtp.h"

#include "osi.h"

#define VIDEO_RTP_SSRC              (0xF9143545)

#define PKTS_BW_V_SR_INFO           (20)
#define MAX_UDP_PACKET_SIZE         (1400)
#define V_CLOCK_RATE                (90000)

extern OsiMsgQ_t            app_v_task_q;
extern OsiMsgQ_t            app_av_task_q;
extern OsiMsgQ_t            app_rtsp_task_q;
extern rtp_sock_info_s      app_av_soc_info[2];

/* 0 - Index for Video */
rtp_sock_info_s             *p_v_soc_info = &app_av_soc_info[0];

/*!
 * Sends video stop status
 */
static inline cc_s32 send_v_stop_status();

/* V-Steam Data */
v_data_s                    app_v_data   = {0};

static cc_u8                tx_buffer[MAX_VIDEO_BUFF_SIZE];

static rtp_profile_s        media_profile   = {0};

static av_task_msg_s        msg_on_av_task_q;
static v_task_msg_s         msg_on_v_task_q;

static ov_v_resolution_e    resolution  = V_1280_720;

/*!
 */
void app_v_tx_task_entry(void *p_args)
{
    ov_vstream_info_s   *p_cur_stream_info  = NULL;
    ov_vstream_info_s   *p_next_stream_info = NULL;

    SlSockAddrIn_t  client_addr         = {0};

    cc_u32          prev_frame_num      = 0xFFFFFFFF;
    cc_u32          remaining_bytes     = 0;
    cc_u32          total_bytes_to_tx   = 0;
    cc_u32          pkt_size            = 0;
    cc_u32          pkt_cnt             = 0;
    cc_u32          octet_count         = 0;

    cc_s32          ret_val             = 0;

    cc_u16          bytes_to_tx         = 0;
    cc_u16          seq_num             = 0;

    cc_u8           v_streaming_stopped = 1;
    cc_u8           is_eof = 0;

#ifdef RX_RR_TX_SR
    rtcp_rr_info_s  rr_info     = {0};
    rtcp_sr_info_s  sr_info     = {0};

    cc_s32          addr_size   = 0;
#endif /* RX_RR_TX_SR */

    memset(&msg_on_av_task_q, 0, sizeof(msg_on_av_task_q));
    memset(&msg_on_v_task_q, 0, sizeof(msg_on_v_task_q));
    memset(&media_profile, 0, sizeof(media_profile));
    memset(&client_addr, 0, sizeof(client_addr));

    p_cur_stream_info   = &app_v_data.vstream_info_current;
    p_next_stream_info  = &app_v_data.vstream_info_next;;

    /** Dynamic profile number for video media as listed
      * in 'app_rtsp_describe_msg'
      */
    media_profile.type              = VIDEO;
    media_profile.payload_type      = 96;
    media_profile.payload_format    = H264_AVC;

    /* SSRC - A Random Number */
    media_profile.ssrc  = VIDEO_RTP_SSRC;

    while(1)
    {
        osi_MsgQRead(&app_v_task_q, &msg_on_v_task_q, OSI_WAIT_FOREVER);
        if(START_V_STREAMING == msg_on_v_task_q.msg_id)
        {
            v_streaming_stopped = 0;
            prev_frame_num      = 0xFFFFFFFF;

            msg_on_av_task_q.task_id    = VIDEO_TASK_ID;
            msg_on_av_task_q.msg_id     = V_ENABLE;
            msg_on_av_task_q.p_data     = &resolution;
            osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);

            osi_MsgQRead(&app_v_task_q, &msg_on_v_task_q, OSI_WAIT_FOREVER);
            if(msg_on_v_task_q.msg_id == V_STATUS_MSG && msg_on_v_task_q.msg == 0)
            {
                /* Status message is processed below */
            }
            else if(STOP_V_STREAMING == msg_on_v_task_q.msg_id)
            {
                osi_MsgQRead(&app_v_task_q, &msg_on_v_task_q, OSI_WAIT_FOREVER);

                seq_num     = 0;
                pkt_cnt     = 0;
                octet_count = 0;

                msg_on_av_task_q.task_id    = VIDEO_TASK_ID;
                msg_on_av_task_q.msg_id     = V_DISABLE;
                msg_on_av_task_q.p_data     = &resolution;
                osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);

                osi_MsgQRead(&app_v_task_q, &msg_on_v_task_q, OSI_WAIT_FOREVER);
                send_v_stop_status();
                v_streaming_stopped = 1;
                continue;
            }
            else
            {
                Report("AV TASK Response - Error %ld", msg_on_v_task_q.msg);
                Report("\n\rFunction: %s, Line: %d", __FUNCTION__, __LINE__);
            }

            while(0 == p_next_stream_info->v_nxt_pkt_size)
            {
                msg_on_av_task_q.task_id    = VIDEO_TASK_ID;
                msg_on_av_task_q.msg_id     = V_GET_INFO;
                msg_on_av_task_q.p_data     = (void *)p_next_stream_info;
                osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);

                /* Wait for response */
                osi_MsgQRead(&app_v_task_q, &msg_on_v_task_q, OSI_WAIT_FOREVER);
                if(msg_on_v_task_q.msg_id == V_STATUS_MSG && msg_on_v_task_q.msg == 0)
                {
                    /* Status message is processed below */
                }
                else if(msg_on_v_task_q.msg_id == STOP_V_STREAMING)
                {
                    osi_MsgQRead(&app_v_task_q, &msg_on_v_task_q, OSI_WAIT_FOREVER);

                    seq_num     = 0;
                    pkt_cnt     = 0;
                    octet_count = 0;

                    msg_on_av_task_q.task_id    = VIDEO_TASK_ID;
                    msg_on_av_task_q.msg_id     = V_DISABLE;
                    msg_on_av_task_q.p_data     = &resolution;
                    osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);

                    osi_MsgQRead(&app_v_task_q, &msg_on_v_task_q, OSI_WAIT_FOREVER);
                    v_streaming_stopped = 1;
                    send_v_stop_status();
                    break;
                }
                else
                {
                    Report("AV TASK Response - Error %ld", msg_on_v_task_q.msg);
                    Report("\n\rFunction: %s, Line: %d", __FUNCTION__, __LINE__);
                }

                osi_Sleep(1);
            }

            if(msg_on_v_task_q.msg_id == STOP_V_STREAMING) continue;

            while(1)
            {
                /* Copy next info data to current one */
                memcpy(p_cur_stream_info, p_next_stream_info, sizeof(ov_vstream_info_s));

                /* Get the data from the OV788 */
                memset(&app_v_data.v_data_buffer[0], 0x00, sizeof(app_v_data.v_data_buffer));
                msg_on_av_task_q.task_id    = VIDEO_TASK_ID;
                msg_on_av_task_q.msg_id     = V_GET_DATA;
                osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);

                osi_MsgQRead(&app_v_task_q, &msg_on_v_task_q, OSI_WAIT_FOREVER);
                if(msg_on_v_task_q.msg_id == V_STATUS_MSG && msg_on_v_task_q.msg == 0)
                {
                    /* Status message is processed below */
                }
                else if(msg_on_v_task_q.msg_id == STOP_V_STREAMING)
                {
                    osi_MsgQRead(&app_v_task_q, &msg_on_v_task_q, OSI_WAIT_FOREVER);

                    seq_num     = 0;
                    pkt_cnt     = 0;
                    octet_count = 0;

                    msg_on_av_task_q.task_id    = VIDEO_TASK_ID;
                    msg_on_av_task_q.msg_id     = V_DISABLE;
                    msg_on_av_task_q.p_data     = &resolution;
                    osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);

                    osi_MsgQRead(&app_v_task_q, &msg_on_v_task_q, OSI_WAIT_FOREVER);
                    v_streaming_stopped = 1;
                    send_v_stop_status();
                    break;
                }
                else
                {
                    Report("AV TASK Response - Error %ld", msg_on_v_task_q.msg);
                    Report("\n\rFunction: %s, Line: %d", __FUNCTION__, __LINE__);
                }

                p_next_stream_info->v_nxt_pkt_size  = 0;

                while(0 == p_next_stream_info->v_nxt_pkt_size)
                {
                    msg_on_av_task_q.task_id    = VIDEO_TASK_ID;
                    msg_on_av_task_q.msg_id     = V_GET_INFO;
                    msg_on_av_task_q.p_data     = (void *)p_next_stream_info;
                    osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);

                    osi_MsgQRead(&app_v_task_q, &msg_on_v_task_q, OSI_WAIT_FOREVER);
                    if(msg_on_v_task_q.msg_id == V_STATUS_MSG && msg_on_v_task_q.msg == 0)
                    {
                        /* Status message is processed below */
                    }
                    else if(msg_on_v_task_q.msg_id == STOP_V_STREAMING)
                    {
                        osi_MsgQRead(&app_v_task_q, &msg_on_v_task_q, OSI_WAIT_FOREVER);

                        seq_num     = 0;
                        pkt_cnt     = 0;
                        octet_count = 0;

                        msg_on_av_task_q.task_id    = VIDEO_TASK_ID;
                        msg_on_av_task_q.msg_id     = V_DISABLE;
                        msg_on_av_task_q.p_data     = &resolution;
                        osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);

                        osi_MsgQRead(&app_v_task_q, &msg_on_v_task_q, OSI_WAIT_FOREVER);
                        v_streaming_stopped = 1;
                        send_v_stop_status();
                        break;
                    }
                    else
                    {
                        Report("AV TASK Response - Error %ld", msg_on_v_task_q.msg);
                        Report("\n\rFunction: %s, Line: %d", __FUNCTION__, __LINE__);
                    }
                }

                if(v_streaming_stopped == 1) break;

                if(p_next_stream_info->v_frame_number != prev_frame_num)
                {
                    /*!
                     * New Frame
                     */
                    media_profile.p_data.vProfile.is_new_frame = 1;

                    /*!
                     * Time stamp counts 'TIMER_CNTS_PER_SEC' times in one second
                     * Therefore, timestamp = counter * (V_CLOCK_RATE/TIMER_CNTS_PER_SEC)
                     */
                    media_profile.timestamp = cc_get_timestamp() * \
                                                (V_CLOCK_RATE/TIMER_CNTS_PER_SEC);
                    media_profile.p_data.vProfile.frame_type =\
                                                app_v_data.v_data_buffer[4];

                    remaining_bytes = p_cur_stream_info->v_stream_size;
                }
                else
                {
                    media_profile.p_data.vProfile.is_new_frame = 0;
                }

                remaining_bytes -= p_cur_stream_info->v_nxt_pkt_size;
                if(!remaining_bytes)    is_eof = 1;
                else                    is_eof = 0;

                prev_frame_num      = p_next_stream_info->v_frame_number;
                total_bytes_to_tx   = app_v_data.rx_pkt_size;
                while(total_bytes_to_tx > 0)
                {
                    /* Fragment the packet and send it over UDP */
                    if(total_bytes_to_tx > MAX_UDP_PACKET_SIZE)
                    {
                        pkt_size    = MAX_UDP_PACKET_SIZE;
                        media_profile.p_data.vProfile.is_last_frame = 0;
                    }
                    else
                    {
                        pkt_size = total_bytes_to_tx;
                        if(is_eof == 1)
                            media_profile.p_data.vProfile.is_last_frame = 1;

                    }

                    seq_num += 1;
                    media_profile.seq_num = seq_num;

                    /* Create RTP packet */
                    memset(tx_buffer, 0x00, sizeof(tx_buffer));
                    ret_val = rtp_encode(&app_v_data.v_data_buffer[app_v_data.rx_pkt_size - total_bytes_to_tx],\
                                         pkt_size, &tx_buffer[0],\
                                         MAX_VIDEO_BUFF_SIZE,\
                                         &media_profile);
                    if(ret_val < 0)
                    {
                        ERR_PRINT(ret_val);
                    }
                    else
                    {
                        bytes_to_tx = ret_val;
                        ret_val = sl_SendTo(p_v_soc_info->rtp_sock_id,  \
                                            tx_buffer, bytes_to_tx, 0,  \
                                            (const SlSockAddr_t *)&(p_v_soc_info->rtp_addr),\
                                            p_v_soc_info->addr_size);
                        if(ret_val <= 0 && ret_val != SL_EAGAIN)
                        {
                            ERR_PRINT(ret_val);
                        }
                        else
                        {
                            pkt_cnt         += 1;
                            octet_count     += (bytes_to_tx - sizeof(rtp_header_s));
                        }
                    }

                    total_bytes_to_tx   -= pkt_size;

                    if(1 == media_profile.p_data.vProfile.is_new_frame)
                    {
                        media_profile.p_data.vProfile.is_new_frame = 0;
                    }
                }

#ifdef RX_RR_TX_SR
                memset(&sr_info, 0, sizeof(sr_info));
                sr_info.ssrc = VIDEO_RTP_SSRC;

                /*!
                 */
                memset(tx_buffer, 0x00, sizeof(tx_buffer));
                addr_size   = sizeof(SlSockAddrIn_t);
                ret_val     = sl_RecvFrom(p_v_soc_info->rtcp_sock_id,       \
                                          tx_buffer, MAX_VIDEO_BUFF_SIZE,   \
                                          0, (SlSockAddr_t *)&client_addr,  \
                                          (SlSocklen_t *)&addr_size);
                if(ret_val <= 0 && ret_val != SL_EAGAIN)
                {
                    /* Rx Error */
                }
                else
                {
                    rtp_process_rtcp_pkt(tx_buffer, ret_val, &rr_info);
                }

                /* Send SR-Info every 'n' packets over the RTCP socket */
                if(0 == (pkt_cnt % PKTS_BW_V_SR_INFO))
                {
                    /* Time to send sender Report */
                    sr_info.pkt_count           = pkt_cnt;
                    sr_info.octet_count         = octet_count;

                    sr_info.rtp_timestamp       = cc_get_timestamp() * \
                                                    (V_CLOCK_RATE/TIMER_CNTS_PER_SEC);

                    sr_info.ntp_timestamp.secs  = (cc_get_timestamp()/TIMER_CNTS_PER_SEC) + \
                                                    (CUR_UTC_TIME);

                    sr_info.ntp_timestamp.nsec  = (cc_u32) \
                                                  (cc_get_timestamp() % TIMER_CNTS_PER_SEC) * \
                                                    (TIMER_LOAD_VALUE * TIME_PER_TICK_IN_NSECS);

                    memset(tx_buffer, 0x00, sizeof(tx_buffer));
                    ret_val = rtp_create_sr_pkt(tx_buffer, sizeof(tx_buffer), &sr_info);
                    if(ret_val < 0)
                    {
                        ERR_PRINT(ret_val);
                    }
                    else
                    {
                        bytes_to_tx = ret_val;
                        ret_val = sl_SendTo(p_v_soc_info->rtcp_sock_id,\
                                            tx_buffer, bytes_to_tx, 0,\
                                            (const SlSockAddr_t *)&(p_v_soc_info->rtcp_addr),\
                                            p_v_soc_info->addr_size);
                        if(ret_val <= 0 && ret_val != SL_EAGAIN)
                        {
                            ERR_PRINT(ret_val);
                        }
                    }
                }
#endif /* RX_RR_TX_SR */
            }
        }
        else
        {
            continue;
        }
    }
}

/**/
static inline cc_s32 send_v_stop_status()
{
    app_rtsp_task_msg_s msgData;
    msgData.task_id = VIDEO_TASK_ID;
    msgData.msg_id = RTSP_V_STOPPED;
    osi_MsgQWrite(&app_rtsp_task_q,&msgData,OSI_NO_WAIT);
    return 0;
}
