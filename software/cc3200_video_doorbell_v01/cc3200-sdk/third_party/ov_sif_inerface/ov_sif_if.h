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

#ifndef __OV788_IF_H__
#define __OV788_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

/*!
    \addtogroup OV_SIF_IF
    @{
*/

#ifndef ov_u32
typedef unsigned long   ov_u32;
#endif

#ifndef ov_u16
typedef unsigned short  ov_u16;
#endif

#ifndef ov_u8
typedef unsigned char   ov_u8;
#endif

#ifndef ov_s32
typedef signed long     ov_s32;
#endif

#ifndef ov_s16
typedef signed short    ov_s16;
#endif

#ifndef ov_s8
typedef signed char     ov_s8;
#endif

/* Error Codes */
typedef enum
{
    OV_SUCCESS                          =   0,
    OV_ERROR_UNKNOWN                    =  -1,
    OV_RESPONSE_MISMATCH_ERROR          =  -2,
    OV_RESPONSE_FLAG_ERROR              =  -3,
    OV_RESPONSE_CLASS_ID_ERROR          =  -4,
    OV_RESPONSE_CMD_ID_ERROR            =  -5,
    OV_RESPONSE_DATA_PACKET_SIZE_ERROR  =  -6,
    OV_RESPONSE_INVALID_PARAM           =  -7,
    OV_RESPONSE_OUT_OF_RANGE_PARAM      =  -8,
    OV_RESPONSE_OUT_OF_RANGE_FWID       =  -9,
    OV_RESPONSE_UNKNOWN_ERROR           = -10,
    INVALID_INPUT                       = -11,
    INVALID_GPIO_RESPONSE               = -12,
    OV_FIRMWARE_DOWNLOAD_ERROR          = -13,
    OV_FIRMWARE_FILE_OPRATION_ERROR     = -14,
    OV_SMALL_BUFFER_SIZE                = -15,

}ov_status_code_e;

/** Response for 'Get_VSTREAM_Getinfo'
  * Refer 'SIF Protocol' document
  */
typedef struct vstream_getinfo
{
    ov_u32  v_stream_size;
    ov_u32  v_nxt_pkt_size;
    ov_u32  v_frame_timestamp;
    ov_u32  v_frame_number;
    ov_u32  v_frame_type;

}ov_vstream_info_s;

/** Response for 'ASTREAM_Getinfo'
  * Refer 'SIF Protocol' document
  */
typedef struct astream_getinfo
{
    ov_u32  a_total_size;
    ov_u32  a_nxt_pkt_size;

}ov_astream_info_s;

/** Represents 'Param0' of 'VCC_Set_Resolution'
  * Refer 'SIF Protocol' document
  */
typedef enum video_resolution
{
    V_640_480,
    V_352_288,
    V_320_230,
    V_176_144,
    V_1280_720,

}ov_v_resolution_e;

/** Represents 'Param0' of 'VCC_Set_Fame_Rate'
  * Refer 'SIF Protocol' document
  */
typedef enum video_framerate
{
    V_30FPS,
    V_20FPS,
    V_15FPS,
    V_5FPS

}ov_v_framerate_e;

/** Represents 'Param0' of 'VCC_Set_Bit_Rate'
  * Refer 'SIF Protocol' document
  */
typedef enum video_bitrate
{
    V_2Mbps,
    V_1Mbps,
    V_512kbps,
    V_128kbps

}ov_v_bitrate_e;

/** Represents 'Param0' of 'SCC_Frequency_Set'
  * Refer 'SIF Protocol' document
  */
typedef enum sensor_frequency
{
    V_AUTO,
    V_50HZ,
    V_60HZ

}ov_v_scc_freq_e;

/* V Config */
typedef struct ov_v_config
{
    ov_v_scc_freq_e frequency;
    ov_u8           brightness;         /* Level value 0~6 */
    ov_u8           contrast;           /* Level value 0~4 */
    ov_u8           saturation;         /* Level value 0~4 */
    ov_u8           flip;               /* Value 0: Normal, 1:flip */
    ov_u8           mode;               /* Value 0: Normal, 1:night mode */

} ov_v_config_s;

/** Represents 'Param0' of 'AENC_setformat'
  * Refer 'SIF Protocol' document
  */
typedef enum audio_format
{
    A_PCM,
    A_ADPCM,
    A_ULAW = 0x05

}ov_a_format_e;

/** Represents 'Param0' of 'AENC_setsamplerate'
  * Refer 'SIF Protocol' document
  */
typedef enum audio_samplerate
{
    /* Response Packet format is correct */
    A_44K,
    A_32K,
    A_24K,
    A_16K,
    A_12K,
    A_11K,
    A_8K

}ov_a_samplerate_e;

/* A Config */
typedef struct ov_a_config
{
    ov_a_format_e       format;
    ov_a_samplerate_e   sampleRate;

}ov_a_config_s;

/**/
typedef enum ov_q_msgs
{
    NO_MESSAGE,
    OV_TI_GPIO_HIGH,
    OV_TI_GPIO_LOW,
    TIMEROUT_EVENT

}ov_q_msgs_e;


/*!
    \brief      Init the interface and configure the OV788 system
    \param      None
    \return     0 on success, negative error-code on error
    \note
*/
ov_s32 ov_init();

/*!
    \brief      Deinit the interface and OV
    \param      None
    \return     0 on success, negative error-code on error
    \note
*/
ov_s32 ov_deinit();


/*!
    \brief Initnialize the interface and config the OV788 system

    \param[in]  fd              :   Interface Descriptor
    \param[in]  out_buffer_size :   Buffer length
    \param[out] p_outbuffer     :   pointer to buffer containing the response
    \param[out] p_length        :   Size of the data in buffer

    \return     0 on success, negative error-code on error

    \note
*/
ov_s32 ov_get_version(ov_u16 const fd, ov_u8 const out_buffer_size,\
                      ov_u8 *p_outbuffer, ov_u8 *p_length);

/*!
    \brief Initnialize the interface and config the OV788 system

    \param[in]  fd              :   Interface Descriptor
    \param[in]  out_buffer_size :   Buffer length
    \param[out] p_outbuffer     :   pointer to buffer containing the response
    \param[out] p_length        :   Size of the data in buffer

    \return     0 on success, negative error-code on error

    \note
*/
ov_s32 ov_get_status(ov_u16 const fd, ov_u8 const out_buffer_size,\
                     ov_u8 *p_outbuffer, ov_u8 *p_length);


/*!
    \brief Configures the bringhtness, contrast, saturation, mode and other
           parameters for the video sensor

    \param[in]  fd      :   Interface Descriptor
    \param[in]  config  :   Structure containing the different configuration
                            parameter value

    \return     0 on success, negative error-code on error

    \note
*/
ov_s32 ov_config_v_sensor(ov_u16 const fd, ov_v_config_s const config);

/*!
    \brief Configure the parameters for audio codec

    \param[in]  fd      :   Interface Descriptor
    \param[in]  config  :   Structure containing the different configuration
                            parameter value

    \return     0 on success, negative error-code on error

    \note
*/
ov_s32 ov_config_a_codec(ov_u16 const fd, ov_a_config_s const config);

/*!
    \brief      Configure the sampling rate for audio

    \param[in]  fd              :   Interface Descriptor
    \param[in]  sampling_rate   :   Sampling rate value

    \return     0 on success, negative error-code on error

    \note
*/
ov_s32 ov_set_sampling_rate(ov_u16 const fd,\
                            ov_a_samplerate_e const sampling_rate);

/*!
    \brief Configure the Frame rate for video

    \param[in]  fd          :   Interface Descriptor
    \param[in]  frame_rate  :   Frame rate Value (Enum)

    \return     0 on success, negative error-code on error

    \note
*/
ov_s32 ov_set_frame_rate(ov_u16 const fd, ov_v_framerate_e const frame_rate);

/*!
    \brief Configure the Bit rate for video

    \param[in]  fd          :   Interface Descriptor
    \param[in]  bit_rate    :   Bit rate rate Value (Enum)

    \return     0 on success, negative error-code on error

    \note
*/
ov_s32 ov_set_bit_rate(ov_u16 const fd, ov_v_bitrate_e const bit_rate);

/*!
    \brief Set the Zoom in and out for the sensor

    \param[in]  fd      : Interface Descriptor
    \param[in]  zoom    : Set zoom value (0 zoom in, 1 zoom out)

    \return     0 on success, negative error-code on error

    \note
*/
ov_s32 ov_set_zoom(ov_u16 const fd, ov_u8 const zoom);

/*!
    \brief Enable/ Disable the motion detect feature

    \param[in]  fd      :   Interface Descriptor
    \param[in]  enable  :   1 Enable, 0 Disable

    \return     0 on success, negative error-code on error

    \note
*/
ov_s32 ov_set_motion_detect(ov_u16 const fd, ov_u8 const enable);

/*!
    \brief Set resolution and enable/disable the video sampling

    \param[in]  fd          :   Interface Descriptor
    \param[in]  resolution  :   Video resolution value (Enum)
    \param[in]  enable      :   Enable(1)/Disable(0) video sampleling

    \return     0 on success, negative error-code on error

    \note
*/
ov_s32 ov_config_v_encoder(ov_u16 const fd, \
                           ov_v_resolution_e const resolution, \
                           ov_u8 const enable);

/*!
    \brief Get the stream info for the next available video stream packet

    \param[in]  fd              : Interface Descriptor
    \param[in]  pkt_size        : Maximum packet size that can be received by
                                  application. This value depend upon the app
                                  buffer for getting the video data
    \param[out] p_vstream_info  : Pointer to structure for the next video packet info

    \return     0 on success, negative error-code on error

    \note   This function should be called before calling the 'ov_get_vstream_data'
*/
ov_s32 ov_get_vstream_info(ov_u16 const fd, ov_u32 const pkt_size,\
                           ov_vstream_info_s *const p_vstream_info);

/*!
    \brief Get the Video data packet

    \param[in]  fd          : Interface Descriptor
    \param[in]  pkt_size    : Packet size to be recevied. This value should be
                              same as 'v_nxt_pkt_size' in previous
                              'ov_get_vstream_info()' function call
    \param[out] p_outbuffer : Pointer to data buffer to recevie the video data

    \return     0 on success, negative error-code on error

    \note
*/
ov_s32 ov_get_vstream_data(ov_u16 const fd, ov_u32 const pkt_size, \
                           ov_u8 *const p_outbuffer);

/*!
    \brief Get the stream info for the next available audio stream packet

    \param[in]  fd              :   Interface Descriptor
    \param[in]  pkt_size        :   Maximum packet size that can be received by
                                    application. This value depend upon the app
                                    buffer for getting the audio data
    \param[out] p_astream_info  :   Pointer to structure for the next audio packet info

    \return     0 on success, negative error-code on error

    \note   This function should be called before calling the 'ov_get_astream_data'
*/
ov_s32 ov_get_astream_info(ov_u16 const fd, ov_u32 const pkt_size,\
                           ov_astream_info_s *const p_astream_info);


/*!
    \brief Get the audio data packet

    \param[in]  fd          : Interface Descriptor
    \param[in]  pkt_size    : Packet size to be recevied.
    \param[out] p_outbuffer : Pointer to data buffer to recevie the audio data

    \return     0 on success, negative error-code on error

    \note
*/
ov_s32 ov_get_astream_data(ov_u16 const descriptor, ov_u32 const pkt_size,
                           ov_u8 *const p_outbuffer);

/*!
    \brief Enable/ Disable the audio status

    \param[in]  fd      :   Interface Descriptor
    \param[in]  enable  :   1 Enable, 0 Disable

    \return     0 on success, negative error-code on error

    \note
*/
ov_s32 ov_config_a_status(ov_u16 const fd, ov_u8 const enable);


/*
 * Note: Below 'ov_<name>' functions shall be registered w/ their corresponding
 *       functions implemented in fs_if.*.
 *
 *       These functions are to program the OV firmware.
 */

/*!
    \brief      Open the file containing the firmware.
                This file is programmed on CC3200's NVM

    \param[in]  fName   : Pointer to file name to be opened
    \param[out] fHdl    : File handler for the opened file

    \return     0 on success, negative error-code on error

    \note
*/
#define ov_OpenFwFile       sl_OpenFile

/*!
    \brief      Get the size of the file

    \param[in]  fName   : Pointer to file name to be opened
    \param[out] fHdl    : Size of the file

    \return     0 on success, negative error-code on error

    \note
*/
#define ov_GetFwSize        sl_GetFileSize

/*!
    \brief Get the data from the file

    \param[in]  fHdl    : File handler
    \param[out] fHdl    : Pointer to the buffer for reading data
    \param[in]  length  : Size of the data to be read
    \param[in]  offset  : Offset location for the data to be read

    \return     0 on success, negative error-code on error

    \note
*/
#define ov_GetData          sl_GetData

/*!
    \brief Close the opened file

    \param[in]  fHdl   : File handler for the opened file

    \return     0 on success, negative error-code on error

    \note
*/
#define ov_CloseFwFile      sl_CloseFile

/*!
     Close the Doxygen group.
     @}
 */

#ifdef  __cplusplus
}
#endif  /*  __cplusplus */
#endif  /* __OV788_IF_H__ */
