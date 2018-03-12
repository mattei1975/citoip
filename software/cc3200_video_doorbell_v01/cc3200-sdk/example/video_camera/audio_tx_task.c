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
#include <stdlib.h>

#include "cc_pal_app.h"

#include "simplelink.h"
#include "ov_sif_if.h"
#include "rtcp_rtp.h"
#include "rtsp.h"

#include "audio_tx_task.h"
#include "get_av_task.h"
#include "app_config.h"

#define AUDIO_RTP_SSRC              (0xF9143263)

#define PKTS_BW_A_SR_INFO           (20)
#define MAX_UDP_PACKET_SIZE         (1400)
#define SAMPLING_RATE               (11025)

#define PLAY_BUFFER_SIZE            (50 * AUDIO_RECV_SIZE)
#define PLAY_WATERMARK              (20 * AUDIO_RECV_SIZE)

extern OsiMsgQ_t        app_a_task_q;
extern OsiMsgQ_t        app_av_task_q;
extern rtp_sock_info_s  app_av_soc_info[NUM_OF_MEDIA_STREAMS];

/*!
 * Circular buffer element
 */
typedef struct a_circular_buffer
{
    cc_u8   *p_read;
    cc_u8   *p_write;
    cc_u8   *p_start;
    cc_u8   *p_end;
    cc_u32  buffer_size;

}circular_buffer_s;

/* 1 - Index for Audio */
rtp_sock_info_s         *p_a_soc_info = &app_av_soc_info[1];

/*!
 * Processes the response for a given message ID.!
 */
static cc_s32
process_status_msg(ov_astream_info_s *const p_cur_stream_info);

/*!
 * Gets the 'A-Stream' information
 */
static cc_s32
get_a_info(ov_astream_info_s *const p_cur_stream_info);

/*!
 * Gets the 'A-Stream' data
 */
static cc_s32 get_a_data();

/*!
 * Processes the A-Stream data.! RTP Encoding and Tx over Wi-Fi
 */
static cc_s32 process_a_data();

/*!
 * Creates circular buffer for audio
 */
static circular_buffer_s *
create_circular_buffer(cc_u32 const buffer_size);

/*!
 * Fills the buffer w/ audio data
 */
static cc_s32
fill_buffer(circular_buffer_s *p_circular_buffer,\
            cc_u8 *p_buffer, cc_u32 pkt_size);

/*!
 * Checks if the buffer has space for a copy
 */
static cc_u8
is_buffer_vacant(circular_buffer_s *p_circular_buffer, cc_u32 size);

/*!
 * Gets the buffer size
 */
static cc_u32
get_buffer_size(circular_buffer_s *p_circular_buffer);

/*!
 * Updates the 'write' pointer
 */
static void
update_write_ptr(circular_buffer_s *p_circular_buffer, cc_u32 pkt_size);

/*!
 * Updates the 'read' pointer
 */
static void
update_read_ptr(circular_buffer_s *p_circular_buffer, cc_u32 size);

/*!
 * Checks if the buffer is empty
 */
static cc_u8
is_buffer_empty(circular_buffer_s *p_circular_buffer);

/*!
 * Checks if the buffer is already filled
 */
static cc_u8
is_buffer_filled(circular_buffer_s *p_circular_buffer, cc_u32 size);

/*!
 * Gets the 'read' pointer
 */
static inline cc_u8*
get_read_ptr(circular_buffer_s *p_circular_buffer);

/*!
 * Gets the 'write' pointer
 */
static inline cc_u8*
get_write_ptr(circular_buffer_s *p_circular_buffer);

/* Play buffer */
circular_buffer_s       *p_play_buffer = NULL;

/* A-Steam Data */
a_data_s                app_a_data          = {0};

static cc_u8            tx_buffer[MAX_AUDIO_BUFF_SIZE]  = {0};
static cc_u8            empty_buffer[AUDIO_RECV_SIZE]   = {0xFF};

static rtp_profile_s    media_profile   = {0};

static av_task_msg_s    msg_on_av_task_q;
static a_task_msg_s     msg_on_a_task_q;

static cc_u32           pkt_cnt         = 0;
static cc_u32           octet_count     = 0;
static cc_u16           seq_num         = 0;

/**/
void app_a_tx_task_entry(void *p_args)
{
    ov_astream_info_s   *const p_cur_stream_info = &app_a_data.astream_info;

    memset(&msg_on_av_task_q, 0, sizeof(msg_on_av_task_q));
    memset(&msg_on_a_task_q, 0, sizeof(msg_on_a_task_q));
    memset(&media_profile, 0, sizeof(media_profile));

    p_play_buffer = create_circular_buffer(PLAY_BUFFER_SIZE);

    /** Dynamic profile number for audio meadia as listed
      * in 'app_rtsp_describe_msg'
      */
    media_profile.type              = AUDIO;
    media_profile.payload_type      = 97;
    media_profile.payload_format    = L16_PCM;

    /* SSRC - A Random Number */
    media_profile.ssrc  = AUDIO_RTP_SSRC;

    while(OSI_OK == osi_MsgQRead(&app_a_task_q, &msg_on_a_task_q, OSI_WAIT_FOREVER))
    {
        switch(msg_on_a_task_q.msg_id)
        {
            case START_A_STREAMING:
            {
                msg_on_av_task_q.task_id    = AUDIO_TASK_ID;
                msg_on_av_task_q.msg_id     = A_ENABLE;
                osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);
            }
            break;

            case STOP_A_STREAMING:
            {
                seq_num     = 0;
                pkt_cnt     = 0;
                octet_count = 0;

                msg_on_av_task_q.task_id    = AUDIO_TASK_ID;
                msg_on_av_task_q.msg_id     = A_DISABLE;
                osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);
            }
            break;

            case STATUS_MSG:
            {
                process_status_msg(p_cur_stream_info);
            }
            break;

            default:
            {
                Report("Unidentified message on A-Task-Q: Msg-ID [%ld]", msg_on_a_task_q.msg);
            }
            break;
        }
    }

    /**/
    return;
}

/**/
static cc_s32
process_status_msg(ov_astream_info_s *const p_cur_stream_info)
{
    cc_s32 ret_val = -1;

    switch(msg_on_av_task_q.msg_id)
    {
        case A_ENABLE:
        {
            /** Status message from AV-Task for A_ENABLE
              * Get and process audio info now
              */
            //Report("Track/Audio - RTP Socket ID [%d]\n\r", p_a_soc_info->rtp_sock_id);

            ret_val = get_a_info(p_cur_stream_info);
            if(ret_val < 0) goto exit_process_status_msg;
        }
        break;

        case A_GET_INFO:
        {
            /** Status message from AV-Task for A_GET_INFO
              * Get and process audio data if the ov has valid
              * a-data.!
              */
            if( (0 == p_cur_stream_info->a_nxt_pkt_size) ||\
                (0 == p_cur_stream_info->a_total_size) )
            {
                ret_val = get_a_info(p_cur_stream_info);
            }
            else
            {
                ret_val = get_a_data();
            }

            if(ret_val < 0) goto exit_process_status_msg;
        }
        break;

        case A_GET_DATA:
        {
            ret_val = process_a_data();
            if(ret_val < 0)
            {
                /* Ignore the error - Continue w/ next set of data */
            }

            /** Post status to 'self' w/ av-task's 'msg_id' set to
              * 'A_GET_INFO' to continue the loop
              */
            p_cur_stream_info->a_nxt_pkt_size   = 0;
            p_cur_stream_info->a_total_size     = 0;
            msg_on_av_task_q.msg_id             = A_GET_INFO;

            msg_on_a_task_q.task_id = AV_TASK_ID;
            msg_on_a_task_q.msg_id  = STATUS_MSG;
            msg_on_a_task_q.msg     = 0;
            osi_MsgQWrite(&app_a_task_q, &msg_on_a_task_q, OSI_NO_WAIT);
        }
        break;

        case A_DISABLE:
        {
            /* Status message from AV-Task for A_DISABLE */
        }
        break;

        default:
        {
            Report("Status message not understood w.r.t AV Msg-ID [%d]",\
                                                    msg_on_av_task_q.msg_id);
        }
        break;
    }

exit_process_status_msg:
    return ret_val;
}

/**/
static cc_s32
get_a_info(ov_astream_info_s *const p_cur_stream_info)
{
    p_cur_stream_info->a_nxt_pkt_size   = 0;
    p_cur_stream_info->a_total_size     = 0;

    msg_on_av_task_q.task_id    = AUDIO_TASK_ID;
    msg_on_av_task_q.msg_id     = A_GET_INFO;
    msg_on_av_task_q.p_data     = (void *)p_cur_stream_info;
    osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);

    return 0;
}

/**/
static cc_s32 get_a_data()
{
    memset(app_a_data.a_data_buffer, 0xFF, sizeof(app_a_data.a_data_buffer));

    /* Get the data from the OV788 */
    msg_on_av_task_q.task_id    = AUDIO_TASK_ID;
    msg_on_av_task_q.msg_id     = A_GET_DATA;
    osi_MsgQWrite(&app_av_task_q, &msg_on_av_task_q, OSI_NO_WAIT);

    return 0;
}

/**/
static cc_s32 process_a_data()
{
    SlSockAddrIn_t  client_addr         = {0};

    cc_u32          total_bytes_to_tx   = 0;
    cc_u32          pkt_size            = 0;

    cc_s32          bytes_to_send       = 0;
    cc_s32          ret_val             = 0;

    cc_u8           *p_read             = NULL;

#ifdef RX_RR_TX_SR
    rtcp_rr_info_s  rr_info     = {0};
    rtcp_sr_info_s  sr_info     = {0};
    cc_s32          addr_size   = 0;
#endif /* RX_RR_TX_SR */

    memset(&client_addr, 0, sizeof(client_addr));

    media_profile.p_data.aProfile.mode = 2; /* NA */

    /*!
     * Time stamp counts 'TIMER_CNTS_PER_SEC' times in one second
     * Therefore, timestamp = counter * (SAMPLING_RATE/TIMER_CNTS_PER_SEC)
     */
    media_profile.timestamp = cc_get_timestamp() * \
                                (SAMPLING_RATE/TIMER_CNTS_PER_SEC);

    /* Populate the buffer */
    ret_val = fill_buffer(p_play_buffer, app_a_data.a_data_buffer, app_a_data.rx_pkt_size);
    if(ret_val < 0)
    {
        ERR_PRINT(ret_val);
        goto exit_process_a_data;
    }

    total_bytes_to_tx   = AUDIO_RECV_SIZE;
    if(!is_buffer_empty(p_play_buffer) && \
        is_buffer_filled(p_play_buffer, PLAY_WATERMARK))
    {
        p_read = get_read_ptr(p_play_buffer);
        update_read_ptr(p_play_buffer, total_bytes_to_tx);
    }
    else
    {
        p_read = &empty_buffer[0];
    }

    while(total_bytes_to_tx > 0)
    {
        /* Fragment the packet and send it over UDP */
        if(total_bytes_to_tx > MAX_UDP_PACKET_SIZE)
        {
            pkt_size    = MAX_UDP_PACKET_SIZE;
        }
        else
        {
            pkt_size = total_bytes_to_tx;
        }

        /* Sequence */
        seq_num += 1;
        media_profile.seq_num = seq_num;

        /* Create RTP packet */
        memset(tx_buffer, 0x00, sizeof(tx_buffer));
        ret_val = rtp_encode(p_read, pkt_size, &tx_buffer[0],\
                             sizeof(tx_buffer), &media_profile);
        if(ret_val < 0)
        {
            ERR_PRINT(ret_val);
            goto exit_process_a_data;
        }

        bytes_to_send = ret_val;
        ret_val = sl_SendTo(p_a_soc_info->rtp_sock_id, tx_buffer, bytes_to_send,\
                            0, (const SlSockAddr_t *)&(p_a_soc_info->rtp_addr),\
                            p_a_soc_info->addr_size);
        if(ret_val <= 0 && ret_val != SL_EAGAIN)
        {
            ERR_PRINT(ret_val);
            goto exit_process_a_data;
        }

        //Report("[Audio Task] - Data Sent - %d bytes\r\n", bytes_to_send);
        pkt_cnt += 1;
        octet_count += (bytes_to_send - sizeof(rtp_header_s));

        total_bytes_to_tx   -= pkt_size;
    }

#ifdef RX_RR_TX_SR
    memset(&sr_info, 0, sizeof(sr_info));
    sr_info.ssrc = AUDIO_RTP_SSRC;

    memset(tx_buffer, 0x00, sizeof(tx_buffer));
    addr_size   = sizeof(SlSockAddrIn_t);
    ret_val     = sl_RecvFrom(p_a_soc_info->rtcp_sock_id,       \
                              tx_buffer, sizeof(tx_buffer), 0,  \
                              (SlSockAddr_t *)&client_addr,     \
                              (SlSocklen_t*)&addr_size);
    if(ret_val <= 0 && ret_val != SL_EAGAIN)
    {
        ERR_PRINT(ret_val);
        goto exit_process_a_data;
    }

    rtp_process_rtcp_pkt(tx_buffer, ret_val, &rr_info);

    /* Send SR-Info every 'n' packets over the RTCP socket */
    if(0 == (pkt_cnt % PKTS_BW_A_SR_INFO))
    {
        /* Time to send sender Report */
        sr_info.pkt_count           = pkt_cnt;
        sr_info.octet_count         = octet_count;

        sr_info.rtp_timestamp       = cc_get_timestamp() * \
                                        (SAMPLING_RATE/TIMER_CNTS_PER_SEC);

        sr_info.ntp_timestamp.secs  = (cc_get_timestamp()/TIMER_CNTS_PER_SEC) + \
                                        (CUR_UTC_TIME);

        sr_info.ntp_timestamp.nsec  = (cc_u32) \
                                      (cc_get_timestamp() %  TIMER_CNTS_PER_SEC) * \
                                        (TIMER_LOAD_VALUE * TIME_PER_TICK_IN_NSECS);

        memset(tx_buffer, 0x00, sizeof(tx_buffer));
        ret_val = rtp_create_sr_pkt(tx_buffer, sizeof(tx_buffer), &sr_info);
        if(ret_val < 0)
        {
            ERR_PRINT(ret_val);
            goto exit_process_a_data;
        }

        bytes_to_send = ret_val;
        ret_val = sl_SendTo(p_a_soc_info->rtcp_sock_id,     \
                            tx_buffer, bytes_to_send, 0,    \
                            (const SlSockAddr_t *)&(p_a_soc_info->rtcp_addr),\
                            p_a_soc_info->addr_size);
        if(ret_val <= 0 && ret_val != SL_EAGAIN)
        {
            ERR_PRINT(ret_val);
            goto exit_process_a_data;
        }
    }
#endif /* RX_RR_TX_SR */

exit_process_a_data:
    return ret_val;
}

/**/
static circular_buffer_s *
create_circular_buffer(cc_u32 const buffer_size)
{
    circular_buffer_s   *p_temp = NULL;

    p_temp = (circular_buffer_s *)malloc(sizeof(circular_buffer_s));
    if(NULL == p_temp) return NULL;

    p_temp->p_start     = (cc_u8 *)malloc(buffer_size);
    p_temp->p_read      = p_temp->p_start;
    p_temp->p_write     = p_temp->p_start;
    p_temp->buffer_size = buffer_size;
    p_temp->p_end       = (p_temp->p_start + buffer_size);

    return (p_temp);
}

/**/
static cc_s32
fill_buffer(circular_buffer_s *p_circular_buffer,\
            cc_u8 *p_buffer, cc_u32 pkt_size)
{
    cc_s32 offset = -1;
    offset = ( (p_circular_buffer->p_write + pkt_size) -\
                p_circular_buffer->p_end);

    if(is_buffer_vacant(p_circular_buffer, p_circular_buffer->buffer_size))
    {
        if(offset <= 0)
        {
            memcpy(get_write_ptr(p_circular_buffer), p_buffer, pkt_size);
        }
        else
        {
            memcpy(get_write_ptr(p_circular_buffer), p_buffer, (pkt_size - offset));
            memcpy(p_circular_buffer->p_start, ((p_buffer + pkt_size) - offset), offset);
        }

        update_write_ptr(p_circular_buffer, pkt_size);
        return pkt_size;
    }

    return -1;
}

/**/
static cc_u8
is_buffer_vacant(circular_buffer_s *p_circular_buffer, cc_u32 size)
{
    cc_u32  bytes_filled = 0;
    cc_u8   ret_val = 0;

    bytes_filled = get_buffer_size(p_circular_buffer);
    ret_val = (bytes_filled <= size) ? 1 : 0;
    return ret_val;
}

/**/
static cc_u32
get_buffer_size(circular_buffer_s *p_circular_buffer)
{
    cc_u32 bytes_filled = 0;
    if(p_circular_buffer->p_read <= p_circular_buffer->p_write)
    {
        bytes_filled = p_circular_buffer->p_write - p_circular_buffer->p_read;
    }
    else
    {
        bytes_filled = \
            ( (p_circular_buffer->p_write - p_circular_buffer->p_start) +\
              (p_circular_buffer->p_end - p_circular_buffer->p_read) );
    }

    return bytes_filled;
}

/**/
static void
update_write_ptr(circular_buffer_s *p_circular_buffer, cc_u32 pkt_size)
{
    cc_s32 offset = 0;

    offset = ( (p_circular_buffer->p_write + pkt_size) - \
                p_circular_buffer->p_end);
    if(offset <= 0)
    {
        p_circular_buffer->p_write = (p_circular_buffer->p_write + pkt_size);
        if(p_circular_buffer->p_write == p_circular_buffer->p_end)
        {
            p_circular_buffer->p_write = p_circular_buffer->p_start;
        }
    }
    else
    {
        p_circular_buffer->p_write = (p_circular_buffer->p_start + offset);
    }
}

/**/
static void
update_read_ptr(circular_buffer_s *p_circular_buffer, cc_u32 size)
{
    cc_s32 offset = 0;

    offset = ( (p_circular_buffer->p_read + size) - \
                p_circular_buffer->p_end);
    if(offset <= 0)
    {
        p_circular_buffer->p_read = (p_circular_buffer->p_read + size);
        if(p_circular_buffer->p_end == p_circular_buffer->p_read)
        {
            p_circular_buffer->p_read = p_circular_buffer->p_start;
        }
    }
    else
    {
        p_circular_buffer->p_read = (p_circular_buffer->p_start + offset);
    }
}

/**/
static cc_u8
is_buffer_empty(circular_buffer_s *p_circular_buffer)
{
    if(p_circular_buffer->p_read == p_circular_buffer->p_write)
        return 1;

    return 0;
}

/**/
static cc_u8
is_buffer_filled(circular_buffer_s *p_circular_buffer, cc_u32 size)
{
    cc_u32  bytes_filled = 0;
    cc_u8   ret_val = 0;

    bytes_filled = get_buffer_size(p_circular_buffer);
    ret_val = (bytes_filled >= size) ? 1 : 0;
    return ret_val;
}

/**/
static inline cc_u8*
get_read_ptr(circular_buffer_s *p_circular_buffer)
{
    return (p_circular_buffer->p_read);
}

/**/
static inline cc_u8*
get_write_ptr(circular_buffer_s *p_circular_buffer)
{
    return (p_circular_buffer->p_write);
}
