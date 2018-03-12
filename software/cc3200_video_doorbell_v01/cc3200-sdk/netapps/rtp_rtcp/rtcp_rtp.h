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

#ifndef __RTCP_RTP_H__
#define __RTCP_RTP_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*!
    \addtogroup RTP_IF
    @{
*/

#ifndef nw_u32
typedef unsigned long   nw_u32;
#endif

#ifndef nw_u16
typedef unsigned short  nw_u16;
#endif

#ifndef nw_u8
typedef unsigned char   nw_u8;
#endif

#ifndef nw_s32
typedef signed long     nw_s32;
#endif

#ifndef nw_s16
typedef signed short    nw_s16;
#endif

#ifndef nw_s8
typedef signed char     nw_s8;
#endif

/* Error Codes */
typedef enum rtp_status_code
{
    RTP_ERROR_UNKNOWN           = -1,
    RTP_LIMITED_BUFF_SIZE_ERROR = -2

}rtp_status_code_e;

/* Supported Profiles */
typedef enum profile_type
{
    VIDEO,
    AUDIO

}profile_type_e;

/* Supported Payload Formats */
typedef enum payload_format
{
    L16_PCM,
    H264_AVC

}payload_format_e;

/* Construct to refer to a time (absolute or interval) */
typedef struct rtcp_ntp_time
{
    nw_u32  secs;   /* Range: 0 to 0xFFFFFFFF secs */
    nw_u32  nsec;   /* Range: 0 to  999999999 nsec */

}rtcp_ntp_time_s;

/* RTCP Sender Report Information */
typedef struct rtcp_sr_info
{
    rtcp_ntp_time_s     ntp_timestamp;
    nw_u32              rtp_timestamp;
    nw_u32              pkt_count;
    nw_u32              octet_count;
    nw_u32              ssrc;

}rtcp_sr_info_s;

/* RTCP Reader Report Information */
#ifdef ccs
typedef struct __attribute__ ((__packed__)) rtcp_rr_data
#elif __IAR_SYSTEMS_ICC__
//#pragma pack(1)
typedef struct rtcp_rr_data
#endif
{
    nw_u32  delay_since_last_sr;
    nw_u32  sender_ssrc;
    nw_u32  source_ssrc;
    nw_u32  last_sr;
    nw_u32  jitter;

    nw_u8   lost_pkts[3];
    nw_u8   lost_fraction;

    nw_u8   has_resource;

}rtcp_rr_info_s;

/* RTP Header */
#ifdef ccs
typedef struct __attribute__ ((__packed__)) rtp_header
#elif __IAR_SYSTEMS_ICC__
//#pragma pack(1)
typedef struct rtp_header
#endif
{
    nw_u8   version;
    nw_u8   payload_marker;
    nw_u16  seq_num;
    nw_u32  timestamp;
    nw_u32  ssrc;

}rtp_header_s;

/* Video Profile */
typedef struct rtp_v_profile_info
{
    nw_u8   is_new_frame;
    nw_u8   is_last_frame;
    nw_u8   frame_type;

}rtp_v_profile_info_s;

/* Audio Profile */
typedef struct rtp_a_profile
{
    nw_u8   mode;

}rtp_a_profile_info_s;

/**/
typedef union
{
    rtp_a_profile_info_s    aProfile;
    rtp_v_profile_info_s    vProfile;

}rtp_profile_data_s;

/* RTP Stream Profile */
typedef struct rtp_profile
{
    rtp_profile_data_s  p_data;
    profile_type_e      type;
    payload_format_e    payload_format;

    nw_u32              timestamp;
    nw_u32              ssrc;
    nw_u16              seq_num;
    nw_u8               payload_type;

}rtp_profile_s;

/*!
    \brief      Inintilizes the RTP library
    \param      none
    \return     0 on success, negative error-code on error
    \note
*/
nw_s32 rtp_init();

/*!
    \brief      Deinintilizes the RTP library
    \param      none
    \return     0 on success, negative error-code on error
    \note
*/
nw_s32 rtp_deinit();


/*!
    \brief Process the RTCP packet and return the result

    \param[in]  p_input     : Pointer to buffer containing the RTCP packet
    \param[in]  length      : length of the input data
    \param[out] p_out_data  : Pointer to structure for storing the relevant RR
                              packet information

    \return     data size in output buffer, negative error-code on error

    \note
*/
nw_s32 rtp_process_rtcp_pkt(nw_u8 *p_input, nw_u16 const length,\
                            rtcp_rr_info_s *const p_out_data);

/*!
    \brief Create the RTP for video p_rtp_profile

    \param[in]  p_data          : Pointer to data to be send
    \param[in]  data_len        : Size of the data to be send
    \param[out] p_out_buffer    : Pointer to buffer containing the output RTP
                                  data
    \param[in]  out_buffer_len  : Size of the output buffer
    \param[in]  p_rtp_profile         : Pointer to structure containing the information required
                                  to create RTP packet

    \return     Size of data in output buffer, negative error-code on error

    \note
*/
nw_s32
rtp_encode(nw_u8 *p_data, nw_u16 const data_len, nw_u8 *const p_out_buffer,\
           nw_u16 const out_buffer_len, rtp_profile_s *p_rtp_profile);

/*!
    \brief Create the sender report packet

    \param[out] p_input     :   Pointer to buffer containing the output SR
    \param[in]  buffer_len  :   Size of the output buffer
    \param[in]  sr_info     :   Pointer to structure containing the information required
                                to create SR packet

    \return     Size of data in output buffer, negative error-code on error

    \note
*/
nw_s32 rtp_create_sr_pkt(nw_u8 *const p_input, nw_u16 const buffer_len,\
                         rtcp_sr_info_s const *const sr_info);

/*!
     Close the Doxygen group.
     @}
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __RTCP_RTP_H__ */
