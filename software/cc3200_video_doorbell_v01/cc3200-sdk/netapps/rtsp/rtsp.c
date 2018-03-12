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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "rtsp.h"

#ifdef SL_PLATFORM_MULTI_THREADED
    #include "osi.h"
    #define RTOS_MUTEX_CREATE(x)    osi_LockObjCreate(x)
    #define RTOS_MUTEX_ACQUIRE(x)   osi_LockObjLock(x, (OsiTime_t) OSI_WAIT_FOREVER)
    #define RTOS_MUTEX_RELEASE(x)   osi_LockObjUnlock(x)
    #define RTOS_MUTEX_DELETE(x)    osi_LockObjDelete(x)
    OsiLockObj_t    rtsp_lock;
#else /* SL_PLATFORM_MULTI_THREADED */
    #define RTOS_MUTEX_CREATE(x)
    #define RTOS_MUTEX_ACQUIRE(x)
    #define RTOS_MUTEX_RELEASE(x)
    #define RTOS_MUTEX_DELETE(x)
#endif /* SL_PLATFORM_MULTI_THREADED */

#define RTSP_OPTION_REQUEST         "OPTIONS"
#define RTSP_DESCRIBE_REQUEST       "DESCRIBE"
#define RTSP_SETUP_REQUEST          "SETUP"
#define RTSP_PLAY_REQUEST           "PLAY"
#define RTSP_TEARDOWN_REQUEST       "TEARDOWN"
#define RTSP_SET_PARAMETER_REQUEST  "SET_PARAMETER"
#define RTSP_GET_PARAMETER_REQUEST  "SET_PARAMETER"

#define SEQUENCE_STRING             "CSeq:"
#define RESP_BUF_SIZE               1500

#define BASE_10                     10
#define BASE                        BASE_10

#define SETUP_UCAST_RESPONSE_STRING "%s\r\nCSeq: %d\r\n%s\r\nTransport: RTP/AVP;unicast;client_port=%d-%d;server_port=%d-%d\r\nSession: %d;timeout=%d\r\n\r\n"
#define SETUP_MCAST_RESPONSE_STRING "%s\r\nCSeq: %d\r\nTransport: RTP/AVP;multicast;destination=%d;port=%d-%d\r\nSession: %d\r\n\r\n"
#define DESCRIBE_RESPONSE_STRING    "%s\r\nCSeq: %d\r\n%s\r\nContent-Base: %s\r\nContent-Type: application/sdp\r\nContent-Length: %d\r\n\r\n"
#define PLAY_RESPONSE_STRING        "%s\r\nCSeq: %d\r\n%s\r\nRange: npt=0.000-\r\nSession: %d\r\n;rtptime=%ld\r\n\r\n"

#ifdef DEBUG_PRINT
extern int Report(const char *format, ...);
#else
#define Report(...)
#endif

static rm_u8   rtsp_ok_response[]          = "RTSP/1.0 200 OK";
static rm_u8   rtsp_supported_options[]    = "Public: OPTIONS, DESCRIBE, SETUP, TEARDOWN, PLAY";

static rm_s8   resp_buffer[RESP_BUF_SIZE]  = {0};
static rm_u8   *p_stream_info              = NULL;

static rm_s8   setup_count = -1;

/* RTSP Packet Parsers */
typedef rm_s32 \
(*fptr_rtsp_packet_parser)\
      (rm_u8 *p_in_buffer, rtsp_session_config_s *p_rtsp_session_config,\
       rm_u8 *p_out_buffer, rm_u16 rtsp_resp_pkt_len, rtsp_pkt_info_s *pkt_info);

/*!
 */
static rm_s32
unknown_tag(rm_u8 *p_in_buffer, rtsp_session_config_s *p_rtsp_session_config,\
            rm_u8 *p_out_buffer, rm_u16 rtsp_resp_pkt_len, rtsp_pkt_info_s *pkt_info);
static rm_s32
options_parser(rm_u8 *p_in_buffer, rtsp_session_config_s *p_rtsp_session_config,\
               rm_u8 *p_out_buffer, rm_u16 rtsp_resp_pkt_len, rtsp_pkt_info_s *pkt_info);
static rm_s32
describe_parser(rm_u8 *p_in_buffer, rtsp_session_config_s *p_rtsp_session_config,\
                rm_u8 *p_out_buffer, rm_u16 rtsp_resp_pkt_len, rtsp_pkt_info_s *pkt_info);
static rm_s32
setup_parser(rm_u8 *p_in_buffer, rtsp_session_config_s *p_rtsp_session_config,\
             rm_u8 *p_out_buffer, rm_u16 rtsp_resp_pkt_len, rtsp_pkt_info_s *pkt_info);
static rm_s32
play_parser(rm_u8 *p_in_buffer, rtsp_session_config_s *p_rtsp_session_config,\
            rm_u8 *p_out_buffer, rm_u16 rtsp_resp_pkt_len, rtsp_pkt_info_s *pkt_info);
static rm_s32
teardown_parser(rm_u8 *p_in_buffer, rtsp_session_config_s *p_rtsp_session_config,\
                rm_u8 *p_out_buffer, rm_u16 rtsp_resp_pkt_len, rtsp_pkt_info_s *pkt_info);

/*!
 */
static rm_s32
rtsp_pre_process(rm_u8 *p_in_buffer);

/*!
 */
static rm_s32
rtsp_post_process(rm_u16 rtsp_resp_pkt_len, rm_u8 *p_out_buffer);

/*!
 */
static inline \
rm_s32 rtsp_get_seq_num(rm_s16 index, rm_s32 const length,\
                        rm_s8 const *const p_token);

/*!
 */
fptr_rtsp_packet_parser rtsp_parser[] = {
                                            unknown_tag,
                                            options_parser,
                                            describe_parser,
                                            setup_parser,
                                            play_parser,
                                            teardown_parser
                                        };

/*!
 */
rm_s32 rtsp_init(rm_u8 *p_param)
{
    // Create the lock OBJ
    RTOS_MUTEX_CREATE(rtsp_lock);
    p_stream_info = p_param;

    return 0;
}

/*!
 */
rm_s32 rtsp_deinit()
{
    // Create the lock OBJ
    RTOS_MUTEX_DELETE(rtsp_lock);
    p_stream_info = NULL;

    return 0;
}

/*!
 */
rm_s32 rtsp_release_setup()
{
    setup_count = -1;
    return 0;
}

/*!
 */
rm_s32 rtsp_packet_parser(rm_u8 *const p_in_buffer, rm_u16 rtp_pkt_size,\
                          rtsp_session_config_s *p_rtsp_session_config,\
                          rm_u16 rtsp_resp_pkt_len, rm_u8 *const p_out_buffer,\
                          rtsp_pkt_info_s *pkt_info)
{
    rm_s32  ret_val = 0;

    RTOS_MUTEX_ACQUIRE(rtsp_lock);

    memset(resp_buffer, 0, sizeof(resp_buffer));

    /* Parse and process the RTSP packet */
    if(!memcmp(p_in_buffer, RTSP_OPTION_REQUEST, strlen(RTSP_OPTION_REQUEST)))
    {
        Report("[RTSP]-> OPTIONS Packet Received \n\r");
        Report("[<-] %s\n\r", p_in_buffer);

        pkt_info->pkt_type = OPTIONS;
    }
    else if(!memcmp(p_in_buffer, RTSP_DESCRIBE_REQUEST, strlen(RTSP_DESCRIBE_REQUEST)))
    {
        Report("[RTSP]-> DESCRIBE Packet Received \n\r ");
        Report("[<-] %s\n\r", p_in_buffer);

        pkt_info->pkt_type = DESCRIBE;
    }
    else if(!memcmp(p_in_buffer, RTSP_SETUP_REQUEST, strlen(RTSP_SETUP_REQUEST)))
    {
        Report("[RTSP]-> SETUP Packet Received \n\r ");
        Report("[<-] %s\n\r", p_in_buffer);

        pkt_info->pkt_type = SETUP;
    }
    else if(!memcmp(p_in_buffer, RTSP_PLAY_REQUEST, strlen(RTSP_PLAY_REQUEST)))
    {
        Report("[RTSP]-> PLAY Packet Received \n\r ");
        Report("[<-] %s\n\r", p_in_buffer);

        pkt_info->pkt_type = PLAY;
    }
    else if(!memcmp(p_in_buffer, RTSP_TEARDOWN_REQUEST, strlen(RTSP_TEARDOWN_REQUEST)))
    {
        Report("[RTSP]-> TEARDOWN Packet Received \n\r ");
        Report("[<-] %s\n\r", p_in_buffer);

        pkt_info->pkt_type = TEARDOWN;
    }
    else
    {
        Report("[<-] %s\n\r", p_in_buffer);
        pkt_info->pkt_type = UNKNOWN;
    }

    ret_val = rtsp_parser[pkt_info->pkt_type](p_in_buffer, p_rtsp_session_config,\
                                              p_out_buffer, rtsp_resp_pkt_len, pkt_info);
    if(ret_val < 0)
    {
        ret_val = -1;
        goto exit_process_rtsp_packet;
    }

    Report("[->] %s\n\r", p_out_buffer);

exit_process_rtsp_packet:
    RTOS_MUTEX_RELEASE(rtsp_lock);
    return ret_val;
}

/*!
 */
static rm_s32
unknown_tag(rm_u8 *p_in_buffer, rtsp_session_config_s *p_rtsp_session_config,\
            rm_u8 *p_out_buffer, rm_u16 rtsp_resp_pkt_len, rtsp_pkt_info_s *pkt_info)
{
    Report("[RTSP]-> UNRECOGNISED PACKET RECEIVED\r\n");
    return -1;
}

/*!
 */
static rm_s32
options_parser(rm_u8 *p_in_buffer, rtsp_session_config_s *p_rtsp_session_config,\
               rm_u8 *p_out_buffer, rm_u16 rtsp_resp_pkt_len, rtsp_pkt_info_s *pkt_info)
{
    rm_s32  ret_val = 0;

    pkt_info->seq_num = rtsp_pre_process(p_in_buffer);
    if(pkt_info->seq_num < 0)
    {
        Report("[RTSP]-> Sequence number not found\n\r ");
        ret_val = -1;
        goto exit_options_parser;
    }

    sprintf((char *) resp_buffer , "%s\r\nCSeq: %d\r\n%s\r\n%s\r\n\r\n", rtsp_ok_response,
    pkt_info->seq_num, p_rtsp_session_config->date, rtsp_supported_options);

    ret_val = rtsp_post_process(rtsp_resp_pkt_len, p_out_buffer);
    if(ret_val < 0)
    {
        goto exit_options_parser;
    }

exit_options_parser:
    return ret_val;
}

/*!
 */
static rm_s32
describe_parser(rm_u8 *p_in_buffer, rtsp_session_config_s *p_rtsp_session_config,\
                rm_u8 *p_out_buffer, rm_u16 rtsp_resp_pkt_len, rtsp_pkt_info_s *pkt_info)
{
    rm_s8   rtsp_url[64] = {0};
    rm_s8   *p_token = NULL;

    rm_s32  content_length = 0;
    rm_s32  ret_val = 0;
    rm_s32  length = 0;
    rm_s16  index = 0;

    pkt_info->seq_num = rtsp_pre_process(p_in_buffer);
    if(pkt_info->seq_num < 0)
    {
        Report("[RTSP]-> Sequence number not found\n\r ");
        ret_val = -1;
        goto exit_describe_parser;
    }

    /* Get URL for content */
    p_token = (rm_s8 *)strstr((const char *)p_in_buffer, "rtsp");
    if (NULL == p_token)
    {
        Report("[RTSP]-> ERROR DESCRIBE Can't get Content-Base: \n\r ");
        ret_val = -1;
        goto exit_describe_parser;
    }

    /* store the URL */
    index = 0;
    length = strlen((char *)p_token);
    while ( (index < length) && ( p_token[index] != ' ') )
        index++;

    if (index <= length)
    {
        strncpy((char *)rtsp_url , (char *)&p_token[0], index);
        rtsp_url[index] = '\0';
    }

    if(p_stream_info != NULL)
    {
        content_length = strlen((const char *)p_stream_info);
    }
    else
    {
        ret_val = RTSP_NULL_DESCRIPTION_POINTER;
        goto exit_describe_parser;
    }

    sprintf((char *)resp_buffer, DESCRIBE_RESPONSE_STRING,\
            rtsp_ok_response, pkt_info->seq_num, p_rtsp_session_config->date,\
            rtsp_url, content_length);

    strcat((char *)resp_buffer, (const char *)p_stream_info);

    ret_val = rtsp_post_process(rtsp_resp_pkt_len, p_out_buffer);
    if(ret_val < 0)
    {
        goto exit_describe_parser;
    }

exit_describe_parser:
    return ret_val;
}

/*!
 */
static rm_s32
setup_parser(rm_u8 *p_in_buffer, rtsp_session_config_s *p_rtsp_session_config,\
             rm_u8 *p_out_buffer, rm_u16 rtsp_resp_pkt_len, rtsp_pkt_info_s *pkt_info)
{
    rm_s8   *p_token = NULL;

    rm_s32  length = 0;
    rm_s32  ret_val = 0;
    rm_s16  index = 0;

    setup_count += 1;
    pkt_info->setup_count = setup_count;

    pkt_info->seq_num = rtsp_pre_process(p_in_buffer);
    if(pkt_info->seq_num < 0)
    {
        Report("[RTSP]-> Sequence number not found\n\r ");
        ret_val = -1;
        goto exit_setup_parser;
    }

    /* Check if multicast or unicast  #2471*/
    p_token = (rm_s8 *)strstr((const char *)&p_in_buffer[0] , "port=");
    if(NULL != p_token)
    {
        /* skip "port=" */
        index = strlen("port=");
        length = strlen((char *)p_token);

        /* Get first port number */
        pkt_info->pkt_response.setup_data[(setup_count)].rtp_port = \
                            strtol((const char *)&p_token[index], NULL , BASE);

        /* Scan for the second port number */
        while( (index < length) &&
               ( p_token[index] != '-') )
        {
            index++;
        }

        if (index <= length)
        {
            /* Get second UDP port number */
            pkt_info->pkt_response.setup_data[(setup_count)].rtcp_port = \
                            strtol((const char *)&p_token[index+1], NULL, BASE);
        }
    }
    else
    {
        pkt_info->pkt_response.setup_data[(setup_count)].rtp_port = 0;
        pkt_info->pkt_response.setup_data[(setup_count)].rtcp_port = 0;
    }

    if(strstr((const char *)&p_in_buffer[0], "unicast"))
    {
        pkt_info->pkt_response.setup_data[(setup_count)].is_multicast = 0;

        if(pkt_info->pkt_response.setup_data[(setup_count)].rtp_port != 0)
        {
            sprintf((char *)resp_buffer, SETUP_UCAST_RESPONSE_STRING,
                    rtsp_ok_response, pkt_info->seq_num, p_rtsp_session_config->date,
                    pkt_info->pkt_response.setup_data[(setup_count)].rtp_port,
                    pkt_info->pkt_response.setup_data[(setup_count)].rtcp_port,
                    p_rtsp_session_config->stream_port[(setup_count)].rtp_port,
                    p_rtsp_session_config->stream_port[(setup_count)].rtcp_port,
                    p_rtsp_session_config->session_num, p_rtsp_session_config->session_timeout);
        }
        else
        {
            pkt_info->pkt_response.setup_data[(setup_count)].rtp_port = \
                                    p_rtsp_session_config->stream_port[(setup_count)].rtp_port;
            pkt_info->pkt_response.setup_data[(setup_count)].rtcp_port = \
                                    p_rtsp_session_config->stream_port[(setup_count)].rtcp_port;

            sprintf((char *)resp_buffer, SETUP_UCAST_RESPONSE_STRING,
                    rtsp_ok_response, pkt_info->seq_num, p_rtsp_session_config->date,
                    pkt_info->pkt_response.setup_data[(setup_count)].rtp_port,
                    pkt_info->pkt_response.setup_data[(setup_count)].rtcp_port,
                    p_rtsp_session_config->stream_port[(setup_count)].rtp_port,
                    p_rtsp_session_config->stream_port[(setup_count)].rtcp_port,
                    p_rtsp_session_config->session_num, p_rtsp_session_config->session_timeout);
        }
    }
    else
    {
        pkt_info->pkt_response.setup_data[(setup_count)].is_multicast = 1;

        if(pkt_info->pkt_response.setup_data[(setup_count)].rtp_port != 0)
        {
            /* if port is defined by client used it otherwise use the set ports */
            sprintf((char *)resp_buffer, SETUP_MCAST_RESPONSE_STRING,
                    rtsp_ok_response, pkt_info->seq_num, p_rtsp_session_config->multicast_ip,
                    pkt_info->pkt_response.setup_data[(setup_count)].rtp_port,
                    pkt_info->pkt_response.setup_data[(setup_count)].rtcp_port,
                    p_rtsp_session_config->session_num);
        }
        else
        {
            pkt_info->pkt_response.setup_data[(setup_count)].rtp_port = \
                                    p_rtsp_session_config->stream_port[(setup_count)].rtp_port;
            pkt_info->pkt_response.setup_data[(setup_count)].rtcp_port = \
                                    p_rtsp_session_config->stream_port[(setup_count)].rtcp_port;

            /* if port is defined by client used it otherwise use the set ports */
            sprintf((char *)resp_buffer, SETUP_MCAST_RESPONSE_STRING,
                    rtsp_ok_response, pkt_info->seq_num, p_rtsp_session_config->multicast_ip,
                    p_rtsp_session_config->stream_port[(setup_count)].rtp_port,
                    p_rtsp_session_config->stream_port[(setup_count)].rtcp_port,
                    p_rtsp_session_config->session_num);
        }
    }

    ret_val = rtsp_post_process(rtsp_resp_pkt_len, p_out_buffer);
    if(ret_val < 0)
    {
        goto exit_setup_parser;
    }

exit_setup_parser:
    return ret_val;
}

/*!
 */
static rm_s32
play_parser(rm_u8 *p_in_buffer, rtsp_session_config_s *p_rtsp_session_config,\
            rm_u8 *p_out_buffer, rm_u16 rtsp_resp_pkt_len, rtsp_pkt_info_s *pkt_info)
{
    rm_s32  ret_val = 0;

    pkt_info->seq_num = rtsp_pre_process(p_in_buffer);
    if(pkt_info->seq_num < 0)
    {
        Report("[RTSP]-> Sequence number not found\n\r ");
        ret_val = -1;
        goto exit_play_parser;
    }

    sprintf((char *) &resp_buffer[0], PLAY_RESPONSE_STRING,
            rtsp_ok_response, pkt_info->seq_num, p_rtsp_session_config->date,
            p_rtsp_session_config->session_num, p_rtsp_session_config->timestamp);

    ret_val = rtsp_post_process(rtsp_resp_pkt_len, p_out_buffer);
    if(ret_val < 0)
    {
        goto exit_play_parser;
    }

exit_play_parser:
    return ret_val;
}

/*!
 */
static rm_s32
teardown_parser(rm_u8 *p_in_buffer, rtsp_session_config_s *p_rtsp_session_config,\
                rm_u8 *p_out_buffer, rm_u16 rtsp_resp_pkt_len, rtsp_pkt_info_s *pkt_info)
{
    rm_s32  ret_val = 0;

    pkt_info->seq_num = rtsp_pre_process(p_in_buffer);
    if(pkt_info->seq_num < 0)
    {
        Report("[RTSP]-> Sequence number not found\n\r ");
        ret_val = -1;
        goto exit_teardown_parser;
    }

    sprintf((char *) resp_buffer, "%s\r\nCSeq: %d\r\n\r\n", rtsp_ok_response, pkt_info->seq_num);

    ret_val = rtsp_post_process(rtsp_resp_pkt_len, p_out_buffer);
    if(ret_val < 0)
    {
        goto exit_teardown_parser;
    }

exit_teardown_parser:
    return ret_val;
}

/*!
 */
static rm_s32
rtsp_pre_process(rm_u8 *p_in_buffer)
{
    rm_s8   *p_token = NULL;

    rm_s32  ret_val = 0;
    rm_s32  length = 0;
    rm_s16  index = 0;

    p_token = (rm_s8 *) strstr((const char *)p_in_buffer, SEQUENCE_STRING);
    if(NULL == p_token)
    {
        Report("[RTSP]-> ERROR Sequence NO not found \n\r ");
        ret_val = -1;
        goto exit_rtsp_pre_process;
    }

    /* Skip SEQUENCE_STRING */
    index = strlen(SEQUENCE_STRING);
    length = strlen((char *)p_token);

    /* skip non number characters ' ' */
    ret_val = rtsp_get_seq_num(index, length, p_token);
    if(ret_val < 0)
    {
        ret_val = -1;
        goto exit_rtsp_pre_process;
    }

exit_rtsp_pre_process:
    return ret_val;

}

/*!
 */
static rm_s32
rtsp_post_process(rm_u16 rtsp_resp_pkt_len, rm_u8 *p_out_buffer)
{
    rm_s32  ret_val = -1;
    rm_u16  resp_len = 0;

    resp_len = strlen((char *)resp_buffer);
    if(resp_len > rtsp_resp_pkt_len)
    {
        /* limited buffer size */
        ret_val = RTSP_LIMITED_BUFF_SIZE_ERROR;
        goto exit_rtsp_post_process;
    }

    memcpy(p_out_buffer, resp_buffer, resp_len);
    ret_val = resp_len;

exit_rtsp_post_process:
    return ret_val;

}

/*!
 */
static inline \
rm_s32 rtsp_get_seq_num(rm_s16 index, rm_s32 const length,\
                        rm_s8 const *const p_token)
{
    rm_s32  ret_val = -1;

    /* skip non number characters ' ' */
    while( (index < length) &&
           ((p_token[index] < '0') || (p_token[index] > '9')) )
    {
        index++;
    }

    if (index <= length) ret_val = strtol((const char *)&p_token[index], NULL, BASE);
    else ret_val = -1;

    return ret_val;
}
