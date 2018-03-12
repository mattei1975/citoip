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

#ifndef __RTSP_H__
#define __RTSP_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*!
    \addtogroup RTSP_IF
    @{
*/

#define NUM_OF_MEDIA_STREAMS    2

#ifndef rm_u32
typedef unsigned long   rm_u32;
#endif

#ifndef rm_u16
typedef unsigned short  rm_u16;
#endif

#ifndef rm_u8
typedef unsigned char   rm_u8;
#endif

#ifndef rm_s32
typedef signed long     rm_s32;
#endif

#ifndef rm_s16
typedef signed short    rm_s16;
#endif

#ifndef rm_s8
typedef signed char     rm_s8;
#endif

/* Error Codes */
typedef enum rtsp_status_code
{
    RTSP_GENERAL_ERROR              = -1,
    RTSP_LIMITED_BUFF_SIZE_ERROR    = -2,
    RTSP_NULL_DESCRIPTION_POINTER   = -3

}rtsp_status_code_e;

/* RTSP Packet Types */
typedef enum packet_type
{
    UNKNOWN,
    OPTIONS,
    DESCRIBE,
    SETUP,
    PLAY,
    TEARDOWN,
    GET_PARAMETER,
    SET_PARAMETER

}rtsp_packet_type_e;

/* RTP Port Pair */
typedef struct rtsp_stream_port
{
    rm_u16 rtp_port;
    rm_u16 rtcp_port;

}rtsp_stream_port_s;

/* RTSP Session Configuration Structure */
typedef struct rtsp_session_config
{
    rtsp_stream_port_s stream_port[NUM_OF_MEDIA_STREAMS];

    rm_s32  session_timeout;
    rm_s32  session_num;

    rm_u32  timestamp;

    rm_u8   multicast_ip[16];
    rm_u8   date[64];

}rtsp_session_config_s;

/* RTSP Setup Data*/
typedef struct rtsp_setup_data
{
    rm_u16  rtp_port;
    rm_u16  rtcp_port;
    rm_s8   is_multicast;
    rm_s8   padding[3];

}rtsp_setup_data_s;
typedef union
{
    rtsp_setup_data_s setup_data[NUM_OF_MEDIA_STREAMS];

}rtsp_pkt_response_u;

/* RTSP Packet Info */
typedef struct rtsp_packet_info
{
    rtsp_packet_type_e      pkt_type;
    rtsp_pkt_response_u     pkt_response;
    rm_s32                  seq_num;
    rm_s32                  setup_count;

}rtsp_pkt_info_s;

/*!
    \brief      Inintilize the RTSP library
    \param[out] p_param: Pointer to buffer containing the source description
                         information
    \return     0 on success, negative error-code on error

    \note
*/
rm_s32  rtsp_init(rm_u8 *p_param);

/*!
    \brief      Deinintilize the RTSP library
    \param      None
    \return     0 on success, negative error-code on error
    \note
*/
rm_s32 rtsp_deinit();

/*!
    \brief      Resets the setup-counter that was incremented on receiving an
                SETUP packet
    \param      None
    \return     0 on success, negative error-code on error
    \note
 */
rm_s32 rtsp_release_setup();

/*!
    \brief      Process the RTSP packet and create the RTSP response packet

    \param[in]  p_in_buffer             :   Pointer to buffer containing RTSP packet
    \param[in]  rtp_pkt_size            :   Size of the RTSP packet
    \param[in]  p_rtsp_session_config   :   Pointer to structure containing the information
                                            required to create RTSP response packet
    \param[out] p_out_buffer            :   Pointer to buffer for RTSP response packet
    \param[in]  rtsp_resp_pkt_len       :   RTSP response packet buffer length
    \param[out] pkt_info                :   RTSP packet informmation

    \return     Size of data in output buffer, negative error-code on error

    \note
*/
rm_s32  \
rtsp_packet_parser(rm_u8 *const p_in_buffer, rm_u16 rtp_pkt_size,           \
                   rtsp_session_config_s *p_rtsp_session_config,            \
                   rm_u16 rtsp_resp_pkt_len, rm_u8 *const p_out_buffer,     \
                   rtsp_pkt_info_s *pkt_info);

/*!
     Close the Doxygen group.
     @}
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __RTSP_H__ */
