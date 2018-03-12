/******************************************************************************
*
*   Copyright (C) 2014 Texas Instruments Incorporated
*
*   All rights reserved. Property of Texas Instruments Incorporated.
*   Restricted rights to use, duplicate or disclose this code are
*   granted through contract.
*
*   The program may not be used without the written permission of
*   Texas Instruments Incorporated or against the terms and conditions
*   stipulated in the agreement under which this program has been supplied,
*   and under no circumstances can it be used with non-TI connectivity device.
*
******************************************************************************/

#include <string.h>
#include "rtcp_rtp.h"

#ifdef SL_PLATFORM_MULTI_THREADED
#include "osi.h"
#define RTOS_MUTEX_CREATE(x)    osi_LockObjCreate(x)
#define RTOS_MUTEX_ACQUIRE(x)   osi_LockObjLock(x, (OsiTime_t) OSI_WAIT_FOREVER)
#define RTOS_MUTEX_RELEASE(x)   osi_LockObjUnlock(x)
#define RTOS_MUTEX_DELETE(x)    osi_LockObjDelete(x)
OsiLockObj_t    rtp_lock;
#else /* SL_PLATFORM_MULTI_THREADED */
#define RTOS_MUTEX_CREATE(x)
#define RTOS_MUTEX_ACQUIRE(x)
#define RTOS_MUTEX_RELEASE(x)
#define RTOS_MUTEX_DELETE(x)
#endif /* SL_PLATFORM_MULTI_THREADED */

#define RTP_HEADER_PAYLOAD  0x7F
#define RTP_HEADER_MARKER   0x80

#define NAL_MTYPE_MASK      0x1F
#define NAL_NRI_MASK        0x60
#define NAL_F_MASK          0x80

#define FU_FTYPE_MASK       0x1F
#define FU_R_MASK           0x20
#define FU_E_MASK           0x40
#define FU_S_MASK           0x80

#ifdef DEBUG_PRINT
extern int Report(const char *format, ...);
#else
#define Report(...)
#endif

/* Source Description Header */
#ifdef ccs
typedef struct __attribute__ ((__packed__)) rtcp_src_header
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct rtcp_src_header
#endif
{
    nw_u8   version;
    nw_u8   type;
    nw_u16  length;
    nw_u32  ssrc;
    nw_u8   ttype;
    nw_u8   tlength;
    nw_u8   text;
    nw_u8   text1;
    nw_u8   text2;
    nw_u8   text3;
    nw_u8   text4;
    nw_u8   etype;

}rtcp_src_header_s;

/* SR Header */
#ifdef ccs
typedef struct __attribute__ ((__packed__)) rtcp_sr_header
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct rtcp_sr_header
#endif
{
    nw_u8   version;
    nw_u8   type;
    nw_u16  length;
    nw_u32  ssrc;
    nw_u32  timestamp_msw;
    nw_u32  timestamp_lsw;
    nw_u32  timestamp;
    nw_u32  pkt_cnt;
    nw_u32  octet_cnt;

}rtcp_sr_header_s;

/* RR Header */
#ifdef ccs
typedef struct __attribute__ ((__packed__)) rtcp_rr_header
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct rtcp_rr_header
#endif
{
    nw_u8   version;
    nw_u8   type;
    nw_u16  length;
    nw_u32  ssrc;

}rtcp_rr_header_s;

/* RR Block Header */
#ifdef ccs
typedef struct __attribute__ ((__packed__)) rtcp_rr_block
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct rtcp_rr_block
#endif
{
    nw_u32  ssrc;
    nw_u8   lost_fraction;
    nw_u8   lost_pkts[3];
    nw_u32  highest_seq_num;
    nw_u32  jitter;
    nw_u32  last_sr;
    nw_u32  delay_since_last_sr;

}rtcp_rr_block_s;

static nw_u8 tmp_endian_buffer[2048];


/*!
 * Converts 'val' from host byte order to network byte order
 */
static nw_u16 rtp_htons(nw_u16 val);

/*!
 * Converts 'val' from host byte order to network byte order
 */
static nw_u32 rtp_htonl(nw_u32 val);

/*!
 * Video Profile Encoding
 */
static nw_s32
rtp_encode_v_profile(rtp_header_s *p_rtp_header, rtp_profile_s *p_rtp_profile,\
                     nw_u8 *const p_out_buffer, nw_u8 *p_data,\
                     nw_u16 const data_len);

/*!
 * Audio Profile Encoding
 */
static nw_s32
rtp_encode_a_profile(rtp_header_s *p_rtp_header, rtp_profile_s *p_rtp_profile,\
                     nw_u8 *const p_out_buffer, nw_u8 *p_data,\
                     nw_u16 const data_len);

/*!
 * H264 AVC Encoding
 */
static nw_s32
rtp_encode_h264avc(rtp_header_s *p_rtp_header, rtp_profile_s *p_rtp_profile, \
                   nw_u8 *const p_out_buffer, nw_u8 *p_data,                 \
                   nw_u16 const data_len);

/*!
 * L16 Encoding
 */
static nw_s32
rtp_encode_pcm16(rtp_header_s *p_rtp_header, rtp_profile_s *p_rtp_profile,  \
                 nw_u8 *const p_out_buffer, nw_u8 *p_data,                  \
                 nw_u16 const data_len);

/*!
 * Big-E to Little-E - Processing is required before sending L16 audio data
 * over RTP
 */
static nw_s32
rtp_x_endianness(nw_u8 *p_data, nw_u16 const data_len);

/*!
 * Fills the NAL/FU
 */
static inline \
nw_s32 rtp_fill_nal_fu_1(nw_u8 const frame_type, nw_u8 *const p_nal_header, \
                         nw_u8 *const p_fu_header, nw_u32 *packet_size,     \
                         nw_u8 *const p_out_buffer, rtp_header_s *p_rtp_header);

/*!
 * Fills the RTP NAL/FU header
 */
static inline \
nw_s32 rtp_fill_nal_fu_2(nw_u8 const frame_type, nw_u8 *const p_nal_header, \
                         nw_u8 *const p_fu_header, nw_u32 *packet_size,     \
                         nw_u8 *const p_out_buffer, rtp_header_s *p_rtp_header);

/*!
 * Fills the RTP NAL/FU header
 */
static inline \
nw_s32 rtp_fill_nal_fu_3(nw_u8 const frame_type, nw_u8 *const p_nal_header, \
                         nw_u8 *const p_fu_header, nw_u32 *packet_size,     \
                         nw_u8 *const p_out_buffer, rtp_header_s *p_rtp_header);

/**/
nw_s32 rtp_init()
{
    RTOS_MUTEX_CREATE(rtp_lock);
    return 0;
}

/**/
nw_s32 rtp_deinit()
{
    RTOS_MUTEX_DELETE(rtp_lock);
    return 0;
}

/**/
nw_s32 rtp_process_rtcp_pkt(nw_u8 *p_input, nw_u16 const length,\
                            rtcp_rr_info_s *const p_out_data)
{
    rtcp_rr_header_s    rr_header   = {0};
    rtcp_rr_block_s     rr_block    = {0};

    RTOS_MUTEX_ACQUIRE(rtp_lock);

    memcpy(&rr_header, p_input, sizeof(rtcp_rr_header_s));

    if( (201 == rr_header.type) &&
        (0x80 == rr_header.version & 0xC0) ) /* RR type packet */
    {
        p_out_data->sender_ssrc = rr_header.ssrc;

        if((rr_header.version & 0x1F) > 0)
        {
            memcpy(&rr_block, (p_input + sizeof(rtcp_rr_header_s)),\
                   sizeof(rtcp_rr_block_s));

            p_out_data->has_resource    = 1;
            p_out_data->source_ssrc     = rr_block.ssrc;
            p_out_data->jitter          = rr_block.jitter;
            p_out_data->lost_fraction   = rr_block.lost_fraction;
            p_out_data->last_sr         = rr_block.last_sr;
            p_out_data->delay_since_last_sr = rr_block.delay_since_last_sr;

            memcpy(&p_out_data->lost_pkts, rr_block.lost_pkts,\
                   sizeof(rr_block.lost_pkts));
        }
        else
        {
            p_out_data->has_resource    = 0;
        }
    }

    RTOS_MUTEX_RELEASE(rtp_lock);
    return 0;
}

/**/
nw_s32
rtp_encode(nw_u8 *p_data, nw_u16 const data_len, nw_u8 *const p_out_buffer,\
           nw_u16 const out_buffer_len, rtp_profile_s *p_rtp_profile)
{
    rtp_header_s    rtp_header  = {0};
    nw_s32          ret_val     = -1;

    RTOS_MUTEX_ACQUIRE(rtp_lock);

    if(out_buffer_len < (data_len + sizeof(rtp_header_s) + 2))
    {
        ret_val = RTP_LIMITED_BUFF_SIZE_ERROR;
        goto exit_rtp_encode;
    }

    rtp_header.payload_marker = 0;

    rtp_header.version          = 0x80;
    rtp_header.seq_num          = rtp_htons(p_rtp_profile->seq_num);
    rtp_header.ssrc             = rtp_htonl(p_rtp_profile->ssrc);
    rtp_header.timestamp        = rtp_htonl(p_rtp_profile->timestamp);
    rtp_header.payload_marker   |= (RTP_HEADER_PAYLOAD & p_rtp_profile->payload_type);

    switch(p_rtp_profile->type)
    {
        case VIDEO:
        {
            ret_val = rtp_encode_v_profile(&rtp_header, p_rtp_profile,\
                                           p_out_buffer, p_data, data_len);
            if (ret_val < 0)
            {
                Report("Error in 'rtp_encode_v_profile'\n\r");
                goto exit_rtp_encode;
            }
        }
        break;

        case AUDIO:
        {
            ret_val = rtp_encode_a_profile(&rtp_header, p_rtp_profile,\
                                           p_out_buffer, p_data, data_len);
            if (ret_val < 0)
            {
                Report("Error in 'rtp_encode_a_profile'\n\r");
                goto exit_rtp_encode;
            }
        }
        break;

        default:
        {
            Report("Unidentified/unsupported profile type\n\r");
            ret_val = -1;
            goto exit_rtp_encode;
        }
    }

exit_rtp_encode:
    RTOS_MUTEX_RELEASE(rtp_lock);
    return ret_val;
}

/**/
nw_s32 rtp_create_sr_pkt(nw_u8 *const p_out_buffer, nw_u16 const buffer_len,\
                         rtcp_sr_info_s const *const sr_info)
{
    rtcp_src_header_s   src_header  = {0};
    rtcp_sr_header_s    sr_header   = {0};
    nw_s32              ret_val     = -1;

    RTOS_MUTEX_ACQUIRE(rtp_lock);

    /* Sender Report Header */
    sr_header.version       = 0x80;
    sr_header.type          = 200;
    sr_header.length        = rtp_htons(0x6);
    sr_header.ssrc          = rtp_htonl(sr_info->ssrc);
    sr_header.timestamp_msw = rtp_htonl(sr_info->ntp_timestamp.secs);
    sr_header.timestamp_lsw = rtp_htonl(sr_info->ntp_timestamp.nsec);
    sr_header.timestamp     = rtp_htonl(sr_info->rtp_timestamp);

    sr_header.pkt_cnt       = rtp_htonl(sr_info->pkt_count);
    sr_header.octet_cnt     = rtp_htonl(sr_info->octet_count);

    /* Source Description Header */
    src_header.version      = 0x81;
    src_header.type         = 202;
    src_header.length       = rtp_htons(0x3);
    src_header.ssrc         = rtp_htonl(sr_info->ssrc);
    src_header.ttype        = 0x1;
    src_header.tlength      = 5;
    src_header.text         = 'A';
    src_header.text1        = 'A';
    src_header.text2        = 'R';
    src_header.text3        = 'A';
    src_header.text4        = 'K';
    src_header.etype        = 0x0;

    if(buffer_len <= (sizeof(rtcp_sr_header_s)+ sizeof(rtcp_src_header_s)))
    {
        ret_val = RTP_LIMITED_BUFF_SIZE_ERROR;
        goto exit_rtp_create_sr_pkt;
    }

    memcpy(p_out_buffer, &sr_header, sizeof(rtcp_sr_header_s));
    memcpy(&p_out_buffer[0 + sizeof(rtcp_sr_header_s)], &src_header,\
                                            sizeof(rtcp_src_header_s));

    ret_val = sizeof(rtcp_sr_header_s)+ sizeof(rtcp_src_header_s);

exit_rtp_create_sr_pkt:
    RTOS_MUTEX_RELEASE(rtp_lock);
    return ret_val;
}

/**/
static nw_u16 rtp_htons(nw_u16 val)
{
    nw_s16    i   = 1;
    nw_s8     *p  = (signed char *)&i;

    if(p[0] == 1) /* little endian */
    {
        p[0] = ((signed char* )&val)[1];
        p[1] = ((signed char* )&val)[0];
        return i;
    }

    /* big endian */
    return val;
}

/**/
static nw_u32 rtp_htonl(nw_u32 val )
{
    nw_u32        i   = 1;
    signed char   *p  = (signed char *)&i;

    if(p[0] == 1) /* little endian */
    {
        p[0] = ((signed char* )&val)[3];
        p[1] = ((signed char* )&val)[2];
        p[2] = ((signed char* )&val)[1];
        p[3] = ((signed char* )&val)[0];
        return i;
    }

    /* big endian */
    return val;
}

/**/
static nw_s32
rtp_encode_v_profile(rtp_header_s *p_rtp_header, rtp_profile_s *p_rtp_profile,\
                     nw_u8 *const p_out_buffer, nw_u8 *p_data,\
                     nw_u16 const data_len)
{
    nw_s32 ret_val = -1;

    switch(p_rtp_profile->payload_format)
    {
        case H264_AVC:
        {
            ret_val = rtp_encode_h264avc(p_rtp_header, p_rtp_profile,\
                                         p_out_buffer, p_data, data_len);
            if (ret_val < 0)
            {
                Report("H264-AVC Encoding Failed.!\n\r");
                goto exit_rtp_encode_v_profile;
            }
        }
        break;

        default:
        {
            Report("Unidentified/unsupported payload format\n\r");
            ret_val = -1;
            goto exit_rtp_encode_v_profile;
        }
    }

exit_rtp_encode_v_profile:
    return ret_val;
}

/**/
static nw_s32
rtp_encode_a_profile(rtp_header_s *p_rtp_header, rtp_profile_s *p_rtp_profile,\
                     nw_u8 *const p_out_buffer, nw_u8 *p_data,\
                     nw_u16 const data_len)
{
    nw_s32 ret_val = -1;

    switch(p_rtp_profile->payload_format)
    {
        case L16_PCM:
        {
            ret_val = rtp_encode_pcm16(p_rtp_header, p_rtp_profile,\
                                       p_out_buffer, p_data, data_len);
            if (ret_val < 0)
            {
                Report("H264-AVC Encoding Failed.!\n\r");
                goto exit_rtp_encode_v_profile;
            }
        }
        break;

        default:
        {
            Report("Unidentified/unsupported payload format\n\r");
            ret_val = -1;
            goto exit_rtp_encode_v_profile;
        }
    }

exit_rtp_encode_v_profile:
    return ret_val;
}

/*!
 */
static nw_s32
rtp_encode_h264avc(rtp_header_s *p_rtp_header, rtp_profile_s *p_rtp_profile,\
                   nw_u8 *const p_out_buffer, nw_u8 *p_data, nw_u16 const data_len)
{
    nw_u32  packet_size = 0;
    nw_s32  ret_val     = -1;
    nw_u8   nal_header  = 0;
    nw_u8   fu_header   = 0;

    /* NAL F */
    nal_header = NAL_F_MASK;

    /* FU  R */
    fu_header &= ~FU_R_MASK;

    if(p_rtp_profile->p_data.vProfile.is_new_frame == 1)
    {
        if(p_rtp_profile->p_data.vProfile.is_last_frame == 1)
        {
            /** End of frame and beginning.!
              * Do not fragment - Single NAL header and no FU header
              */
            p_rtp_header->payload_marker |= RTP_HEADER_MARKER;

            ret_val = rtp_fill_nal_fu_1(p_rtp_profile->p_data.vProfile.frame_type,\
                                        &nal_header, &fu_header, &packet_size,\
                                        p_out_buffer, p_rtp_header);
            if(ret_val < 0) goto exit_rtp_encode_h264avc;
        }
        else
        {
            /* Not the last frame */
            p_rtp_header->payload_marker &= ~(RTP_HEADER_MARKER);

            /* NAL type FU-A (NType 0x1C) */
            nal_header |= 0x1C;

            /* Fu header S = 1 */
            fu_header |=  FU_S_MASK;

            /* Fu Header E = 0 */
            fu_header &= ~(FU_E_MASK);

            ret_val = rtp_fill_nal_fu_2(p_rtp_profile->p_data.vProfile.frame_type,\
                                        &nal_header, &fu_header, &packet_size,\
                                        p_out_buffer, p_rtp_header);
            if(ret_val < 0) goto exit_rtp_encode_h264avc;
        }
    }
    else
    {
        /* Not a new frame */

        /* NAL type FU-A NType = 0x1C */
        nal_header |= 0x1C;

        if(p_rtp_profile->p_data.vProfile.is_last_frame == 1)
        {
            /* End of fragmented frame sequence */

            p_rtp_header->payload_marker |= RTP_HEADER_MARKER;

            /** S = 0
              * End of frame E = 1
              */
            fu_header &= ~FU_S_MASK;
            fu_header |= FU_E_MASK;

            ret_val = rtp_fill_nal_fu_3(p_rtp_profile->p_data.vProfile.frame_type,\
                                        &nal_header, &fu_header, &packet_size,\
                                        p_out_buffer, p_rtp_header);
            if(ret_val < 0) goto exit_rtp_encode_h264avc;
        }
        else
        {
            /* Inside fragmented frame sequence  */

            /* Not end of frame */
            p_rtp_header->payload_marker &= ~(RTP_HEADER_MARKER);

            /** Fu Header S = 0
              * Fu Header E = 0
              */
            fu_header &= ~FU_S_MASK;
            fu_header &= ~FU_E_MASK;

            ret_val = rtp_fill_nal_fu_3(p_rtp_profile->p_data.vProfile.frame_type,\
                                        &nal_header, &fu_header, &packet_size,\
                                        p_out_buffer, p_rtp_header);
            if(ret_val < 0) goto exit_rtp_encode_h264avc;
        }
    }

    memcpy(p_out_buffer+packet_size, p_data, data_len);
    packet_size += data_len;

    ret_val = packet_size;

exit_rtp_encode_h264avc:
    return ret_val;
}

/**/
static nw_s32
rtp_encode_pcm16(rtp_header_s *p_rtp_header, rtp_profile_s *p_rtp_profile,\
                 nw_u8 *const p_out_buffer, nw_u8 *p_data, nw_u16 const data_len)
{
    nw_u32  packet_size =  0;
    nw_s32  ret_val     = -1;

    ret_val = rtp_x_endianness(p_data, data_len);
    if(ret_val < 0) goto exit_rtp_encode_pcm16;

    p_rtp_header->payload_marker &= ~(RTP_HEADER_MARKER);

    memcpy(p_out_buffer+packet_size, p_rtp_header, sizeof(rtp_header_s));
    packet_size+=sizeof(rtp_header_s);

    memcpy(p_out_buffer+packet_size, &tmp_endian_buffer[0], data_len);
    packet_size += data_len;
    Report("p_out_buffer = 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n\r",
                                    p_out_buffer[packet_size]   , p_out_buffer[packet_size+1],
                                    p_out_buffer[packet_size+2] , p_out_buffer[packet_size+3],
                                    p_out_buffer[packet_size+4] , p_out_buffer[packet_size+5],
                                    p_out_buffer[packet_size+6] , p_out_buffer[packet_size+7]);

    ret_val = data_len;

exit_rtp_encode_pcm16:
    return ret_val;
}

/**/
static nw_s32
rtp_x_endianness(nw_u8 *p_data, nw_u16 const data_len)
{
    nw_u8   *p_temp = NULL;

    nw_u32  idx     = 0;
    nw_u32  var_nl  = 0;
    nw_u32  var_h   = 0;

    p_temp = p_data;
    Report("p_temp = 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n\r",
                            p_temp[0], p_temp[1], p_temp[2], p_temp[3],
                            p_temp[4], p_temp[5], p_temp[6], p_temp[7]);

    memset(tmp_endian_buffer, 0, sizeof(tmp_endian_buffer));

    for(idx = 0; idx < data_len; idx+=sizeof(long))
    {
        var_h = *(p_temp+idx) << 24     |  *(p_temp+idx+1) << 16 |\
                *(p_temp+idx+2) << 8    |  *(p_temp+idx+3);

        var_nl = rtp_htonl(var_h);

        tmp_endian_buffer[idx]      = var_nl >> 24;
        tmp_endian_buffer[idx+1]    = var_nl >> 16;
        tmp_endian_buffer[idx+2]    = var_nl >> 8;
        tmp_endian_buffer[idx+3]    = var_nl;
    }

    return 0;
}

/**/
static inline \
nw_s32 rtp_fill_nal_fu_1(nw_u8 const frame_type, nw_u8 *const p_nal_header, \
                         nw_u8 *const p_fu_header, nw_u32 *p_packet_size,   \
                         nw_u8 *const p_out_buffer, rtp_header_s *p_rtp_header)
{
    nw_s32 ret_val = 0;

    if(0x41 == frame_type)
    {
        /** I frame
          * Non-IDR, NAL type non-IDR slice\
          * NRI 0, NType 1\
          */
        *p_nal_header |= 0x01;
    }
    else if(0x67 == frame_type)
    {
        /** IDR frame
          * IDR frame, NAL type IDR slice
          * NRI 3, NType 5
          */
        *p_nal_header |=  0x65;
    }
    else if(0x01 == frame_type)
    {
        /** P frame
          * Non-IDR, NAL type non-IDR slice
          * NRI 0, NType 1
          */
        *p_nal_header |= 0x01;
    }
    else
    {
        ret_val = -1;
        goto exit_rtp_fill_nal_fu_1;
    }

    memcpy(p_out_buffer + *p_packet_size, p_rtp_header, sizeof(rtp_header_s));
    *p_packet_size += sizeof(rtp_header_s);

    memcpy(p_out_buffer + *p_packet_size, p_nal_header, sizeof(nw_u8));
    *p_packet_size += sizeof(nw_u8);

exit_rtp_fill_nal_fu_1:
    return ret_val;
}

/**/
static inline \
nw_s32 rtp_fill_nal_fu_2(nw_u8 const frame_type, nw_u8 *const p_nal_header, \
                         nw_u8 *const p_fu_header, nw_u32 *p_packet_size,   \
                         nw_u8 *const p_out_buffer, rtp_header_s *p_rtp_header)
{
    nw_s32 ret_val = 0;

    if(0x41 == frame_type)
    {
        /** I frame
          * Non-IDR NRI  = 0
          * NAL type non-IDR slice Ftype = 1
          */
        *p_nal_header &= ~(NAL_NRI_MASK);
        *p_fu_header |= 0x01;
    }
    else if(0x67 == frame_type)
    {
        /** IDR frame
          * IDR NRI = 3
          * NAL type IDR slice FType = 5
          */
        *p_nal_header |= NAL_NRI_MASK;
        *p_fu_header |= 0x05;
    }
    else if(0x01 == frame_type)
    {
        /** P frame
          * Non-IDR NRI = 0
          * NAL type non-IDR slice FType = 1
          */
        *p_nal_header &= ~(NAL_NRI_MASK);
        *p_fu_header |= 0x01;
    }
    else
    {
        ret_val = -1;
        goto exit_rtp_fill_nal_fu_2;
    }

    memcpy(p_out_buffer + *p_packet_size, p_rtp_header, sizeof(rtp_header_s));
    *p_packet_size += sizeof(rtp_header_s);

    memcpy(p_out_buffer + *p_packet_size, p_nal_header, sizeof(nw_u8));
    *p_packet_size += sizeof(nw_u8);

    memcpy(p_out_buffer + *p_packet_size, p_fu_header, sizeof(nw_u8));
    *p_packet_size += sizeof(nw_u8);

exit_rtp_fill_nal_fu_2:
    return ret_val;
}

/**/
static inline \
nw_s32 rtp_fill_nal_fu_3(nw_u8 const frame_type, nw_u8 *const p_nal_header, \
                         nw_u8 *const p_fu_header, nw_u32 *p_packet_size,   \
                         nw_u8 *const p_out_buffer, rtp_header_s *p_rtp_header)
{
    nw_s32 ret_val = 0;

    if(0x41 == frame_type)
    {
        /** I frame
          * Non-IDR NRI = 0
          * NAL type non-IDR slice FType = 1
          */
        *p_nal_header &= ~NAL_NRI_MASK;
        *p_fu_header |= 0x01;
    }
    else if(0x67 == frame_type)
    {
        /** IDR frame
          * IDR  NRI = 3
          * NAL type IDR slice FType = 5
          */
        *p_nal_header |= NAL_NRI_MASK;
        *p_fu_header |= 0x05;
    }
    else if(0x01 == frame_type)
    {
        /** P frame
          * Non-IDR NRI = 0
          * NAL type non-IDR slice FType = 1
          */
        *p_nal_header &= ~NAL_NRI_MASK;
        *p_fu_header |= 0x01;
    }
    else
    {
        ret_val = -1;
        goto exit_rtp_fill_nal_fu_3;
    }

    memcpy(p_out_buffer + *p_packet_size, p_rtp_header, sizeof(rtp_header_s));
    *p_packet_size += sizeof(rtp_header_s);

    memcpy(p_out_buffer + *p_packet_size, p_nal_header, sizeof(nw_u8));
    *p_packet_size += sizeof(nw_u8);

    memcpy(p_out_buffer + *p_packet_size, p_fu_header, sizeof(nw_u8));
    *p_packet_size += sizeof(nw_u8);

exit_rtp_fill_nal_fu_3:
    return ret_val;
}
