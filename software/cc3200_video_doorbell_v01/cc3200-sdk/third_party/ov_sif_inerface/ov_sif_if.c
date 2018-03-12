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

#include <stdio.h>

#include "cc_pal_app.h"
#include "fs_if.h"

#include "ov_sif_if.h"

#define OSD_COMMAND_CLASS_ID                0x03
#define SYSTEM_COMMAND_CLASS_ID             0x06

#define SENSOR_CNTRL_COMMAND_CLASS_ID       0x10
#define VIDEO_CNTRL_COMMAND_CLASS_ID        0x11
#define AUDIO_CNTRL_COMMAND_CLASS_ID        0x12
#define GET_VIDEO_COMMAND_CLASS_ID          0xA0
#define GET_AUDIO_COMMAND_CLASS_ID          0xA1
#define GET_MOTION_IMAGE_COMMAND_CLASS_ID   0xA3

#define OVT_FW_FILE                         "/user/ovt_firmware.bin"
#define OV_CMD_ACK                          0x02

#define MAX_Q_ELEMENTS                      20
#define COMMAND_SIZE                        32
#define FW_BUFF_SIZE                        1024
#define NUM_OF_TRIALS                       5

#ifdef SL_PLATFORM_MULTI_THREADED
    #include "osi.h"
    #define RTOS_MUTEX_CREATE(x)    osi_LockObjCreate(x)
    #define RTOS_MUTEX_ACQUIRE(x)   osi_LockObjLock(x, (OsiTime_t) OSI_WAIT_FOREVER)
    #define RTOS_MUTEX_RELEASE(x)   osi_LockObjUnlock(x)
    #define RTOS_MUTEX_DELETE(x)    osi_LockObjDelete(x)
    OsiLockObj_t    ov_if_lock;
#else /* SL_PLATFORM_MULTI_THREADED */
    #define RTOS_MUTEX_CREATE(x)
    #define RTOS_MUTEX_ACQUIRE(x)
    #define RTOS_MUTEX_RELEASE(x)
    #define RTOS_MUTEX_DELETE(x)
#endif /* SL_PLATFORM_MULTI_THREADED */

#ifdef DEBUG_PRINT
extern int Report(const char *format, ...);
#else
#define Report(...)
#endif

/* OV Response Packets */
typedef enum ov_response
{
    OV_RESP_NO_PAYLOAD,
    OV_RESP_PAYLOAD,
    OV_RESP_CMD_FLAG_ERROR,
    OV_RESP_CLASS_ID_NOT_SUPPORTED,
    OV_RESP_CMD_ID_NOT_SUPPORTED,
    OV_RESP_INCORRECT_DATA_PACKET_SIZE,
    OV_RESP_INVALID_PARAM,
    OV_RESP_PARAM_OUT_OF_RANGE,
    OV_RESP_FWID_OUT_OF_RANGE

}ov_response_e;

/*!
 * Downloads the OV FW stored on NVM and programs the content onto the OV
 */
static ov_s32 ov_download_fw(ov_u16 fd);

/*!
 * Powers off the OV788 system
 */
static ov_s32 ov_poweroff();

/*!
 * Converts the 'val' from host byte order to network byte order.
 */
static ov_u32 ov_htonl(ov_u32 val);

/*!
 * Verifies the ACK from OV
 */
static ov_s32
ov_verify_ack(ov_u8 *const p_command, ov_u8 const *const p_ackbuffer);

/*!
 * Sends the command to OV
 */
static ov_s32 ov_send_command(ov_u8 const cmd_class, ov_u8 const cmd_id,    \
                              ov_u8 const param_len, void *p_param,         \
                              ov_u8 *const p_ack_payload,                   \
                              ov_u16 const payload_buf_len);

/*!
 * Implements the OV's CMD_ACK sequence for sending the command
 * Refer 'SIF Protocol' document for details on the sequence
 */
static ov_s32 ov_execute_cmd_ack_seq(ov_u8 *const p_cmdBuffer,  \
                                     ov_u8 *const p_ackBuffer,  \
                                     ov_u8 *const p_payload,    \
                                     ov_u32 const read_size);

OsiMsgQ_t       ov_q_obj;

static ov_u8    cmd_buffer[COMMAND_SIZE]    = {0};
static ov_u8    ack_buffer[COMMAND_SIZE]    = {0};
static ov_u8    fw_buffer[FW_BUFF_SIZE + 2] = {0};

static ov_u8    ovt_boot_loader_init[]      = {
                                                0x00,0x01,0x30,0x00,
                                                0x00,0x80,0x00,0x00,
                                                0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x03,
                                              };

static const    ov_s8 cmd_header[]          = {
                                                0xFF,0xFF,0xFF,0x00
                                              };

/**/
void ov_irg_handler(void *p_value)
{
    ov_q_msgs_e var     = NO_MESSAGE;
    ov_u8       status  = *((ov_u8 *)p_value);

    var = (1 == status) ? OV_TI_GPIO_HIGH : OV_TI_GPIO_LOW;
    osi_MsgQWrite(&ov_q_obj, &var, OSI_NO_WAIT);
}

/**/
ov_s32 ov_init()
{
    ov_s32 fd = -1;

    RTOS_MUTEX_CREATE(ov_if_lock);
    osi_MsgQCreate(&ov_q_obj, "ov_queue", sizeof(ov_q_msgs_e), MAX_Q_ELEMENTS);

    fd = cc_configure_spi();
    cc_gpio_configure(GPIO_TI_TO_OV);
    cc_gpio_configure(GPIO_OV_TO_TI);
    cc_config_ov_irq(ov_irg_handler);

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    ov_download_fw(fd);

    RTOS_MUTEX_RELEASE(ov_if_lock);
    return fd;
}

/**/
ov_s32 ov_deinit()
{
    ov_poweroff();
    RTOS_MUTEX_DELETE(ov_if_lock);
    return 0;
}

/**/
ov_s32 ov_get_version(ov_u16 const fd, ov_u8 const out_buffer_size,\
                      ov_u8 *p_outbuffer, ov_u8 *p_length)
{
    ov_u8 const payload_size = 4;
    ov_s32 ret_val = 0;

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    if( (out_buffer_size < payload_size) ||
        (NULL == p_outbuffer) )
    {
        *p_length = 0;
        ret_val = OV_SMALL_BUFFER_SIZE;
        goto exit_ov_get_version;
    }

    ret_val = ov_send_command(SYSTEM_COMMAND_CLASS_ID, 0x00, 0, NULL, NULL, 0);
    if(ret_val < 0) goto exit_ov_get_version;

    memcpy((void *)p_outbuffer, &ack_buffer[10], payload_size);
    *p_length = payload_size;

exit_ov_get_version:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
ov_s32 ov_get_status(ov_u16 const fd, ov_u8 const out_buffer_size,\
                     ov_u8 *p_outbuffer, ov_u8 *p_length)
{
    ov_u8 const payload_size = 1;
    ov_s32 ret_val = 0;

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    if( (out_buffer_size < payload_size) ||
        (NULL == p_outbuffer) )
    {
        *p_length = 0;
        ret_val = OV_SMALL_BUFFER_SIZE;
        goto exit_ov_get_status;
    }

    ret_val = ov_send_command(SYSTEM_COMMAND_CLASS_ID, 0x01, 0, NULL, NULL, 0);
    if(ret_val < 0) goto exit_ov_get_status;

    *p_outbuffer = ack_buffer[10];
    *p_length = payload_size;

exit_ov_get_status:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
ov_s32 ov_config_v_sensor(ov_u16 const fd, ov_v_config_s const config)
{
    ov_u8 const param_length = 1;

    ov_s32 ret_val = 0;
    ov_u8  param = 0;

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    /* Configure mode */
    param = config.mode;
    ret_val = ov_send_command(SENSOR_CNTRL_COMMAND_CLASS_ID, 0x07,\
                              param_length, &param, NULL, 0);
    if(ret_val < 0) goto exit_ov_config_v_sensor;

    /* Configure flip */
    param = config.flip;
    ret_val = ov_send_command(SENSOR_CNTRL_COMMAND_CLASS_ID, 0x06,\
                              param_length, &param, NULL, 0);
    if(ret_val < 0) goto exit_ov_config_v_sensor;

    /* Configure frequency */
    param = config.frequency;
    ret_val = ov_send_command(SENSOR_CNTRL_COMMAND_CLASS_ID, 0x05,\
                              param_length, &param, NULL, 0);
    if(ret_val < 0) goto exit_ov_config_v_sensor;

    /* Configure saturation */
    param = config.saturation;
    ret_val = ov_send_command(SENSOR_CNTRL_COMMAND_CLASS_ID, 0x04,\
                              param_length, &param, NULL, 0);
    if(ret_val < 0) goto exit_ov_config_v_sensor;

    /* Configure contrast */
    param = config.contrast;
    ret_val = ov_send_command(SENSOR_CNTRL_COMMAND_CLASS_ID, 0x03,\
                              param_length, &param, NULL, 0);
    if(ret_val < 0) goto exit_ov_config_v_sensor;

    /* Configure brightness */
    param = config.brightness;
    ret_val = ov_send_command(SENSOR_CNTRL_COMMAND_CLASS_ID, 0x02,\
                              param_length, &param, NULL, 0);
    if(ret_val < 0) goto exit_ov_config_v_sensor;

exit_ov_config_v_sensor:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
ov_s32 ov_set_frame_rate(ov_u16 const fd, ov_v_framerate_e const frame_rate)
{
    ov_u8 const param_length = 1;

    ov_s32 ret_val = 0;
    ov_u8  param = 0;

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    param = frame_rate;
    ret_val = ov_send_command(VIDEO_CNTRL_COMMAND_CLASS_ID, 0x01,\
                              param_length, &param, NULL, 0);
    if(ret_val < 0) goto exit_ov_set_frame_rate;

exit_ov_set_frame_rate:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
ov_s32 ov_set_bit_rate(ov_u16 const fd, ov_v_bitrate_e const bit_rate)
{
    ov_u8 const param_length = 1;

    ov_s32 ret_val = 0;
    ov_u8  param = 0;

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    param = bit_rate;
    ret_val = ov_send_command(VIDEO_CNTRL_COMMAND_CLASS_ID, 0x02,\
                              param_length, &param, NULL, 0);
    if(ret_val < 0) goto exit_ov_set_bit_rate;

exit_ov_set_bit_rate:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
ov_s32 ov_set_zoom(ov_u16 const fd, ov_u8 const zoom)
{
    ov_u8 const param_length = 1;

    ov_s32 ret_val = 0;
    ov_u8  param = 0;

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    param = zoom;
    ret_val = ov_send_command(VIDEO_CNTRL_COMMAND_CLASS_ID, 0x03,\
                              param_length, &param, NULL, 0);
    if(ret_val < 0) goto exit_ov_set_zoom;

exit_ov_set_zoom:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
ov_s32 ov_set_motion_detect(ov_u16 const fd, ov_u8 const enable)
{
    ov_u8 const param_length = 1;

    ov_s32 ret_val = 0;
    ov_u8  param = 0;

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    param = enable;
    ret_val = ov_send_command(VIDEO_CNTRL_COMMAND_CLASS_ID, 0x07,\
                              param_length, &param, NULL, 0);
    if(ret_val < 0) goto exit_ov_set_motion_detect;

exit_ov_set_motion_detect:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
ov_s32 ov_config_a_codec(ov_u16 const fd, ov_a_config_s const config)
{
    ov_u8 const param_length = 1;

    ov_s32 ret_val = 0;
    ov_u8  param = 0;

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    /* Configure format */
    param = config.format;
    ret_val = ov_send_command(AUDIO_CNTRL_COMMAND_CLASS_ID, 0x01,\
                              param_length, &param, NULL, 0);
    if(ret_val < 0) goto exit_ov_config_a_codec;

    /* Configure samplerate */
    param = config.sampleRate;
    ret_val = ov_send_command(AUDIO_CNTRL_COMMAND_CLASS_ID, 0x02,\
                              param_length, &param, NULL, 0);
    if(ret_val < 0) goto exit_ov_config_a_codec;

exit_ov_config_a_codec:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
ov_s32 ov_set_sampling_rate(ov_u16 const fd, \
                            ov_a_samplerate_e const sampling_rate)
{
    ov_u8 const param_length = 1;

    ov_s32 ret_val = 0;
    ov_u8  param = 0;

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    /* Configure format */
    param = sampling_rate;
    ret_val = ov_send_command(AUDIO_CNTRL_COMMAND_CLASS_ID, 0x02,\
                              param_length, &param, NULL, 0);
    if(ret_val < 0) goto exit_ov_set_sampling_rate;

exit_ov_set_sampling_rate:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
ov_s32 ov_config_v_encoder(ov_u16 const fd,                     \
                           ov_v_resolution_e const resolution,  \
                           ov_u8 const enable)
{
    ov_u8 const param_length = 2;

    ov_s32 ret_val = 0;
    ov_u8  param[2] = {0};

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    param[0] = resolution;
    param[1] = enable;
    ret_val = ov_send_command(VIDEO_CNTRL_COMMAND_CLASS_ID, 0x00,\
                              param_length, &param[0], NULL, 0);
    if(ret_val < 0) goto exit_ov_config_v_encoder;

exit_ov_config_v_encoder:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
ov_s32 ov_get_vstream_info(ov_u16 const fd, ov_u32 const pkt_size,\
                           ov_vstream_info_s *const p_vstream_info)
{
    ov_u32 param = 0;
    ov_u32 info = 0;
    ov_s32 ret_val = 0;

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    param = ov_htonl(pkt_size);
    ret_val = ov_send_command(GET_VIDEO_COMMAND_CLASS_ID, 0x00,\
                              sizeof(param), (void *)&param, NULL, 0);
    if(ret_val < 0) goto exit_ov_get_vstream_info;

    memcpy((void *)&(info), &ack_buffer[10], sizeof(info));
    p_vstream_info->v_stream_size = ov_htonl(info);

    memcpy((void *)&(info), &ack_buffer[14], sizeof(info));
    p_vstream_info->v_nxt_pkt_size = ov_htonl(info);

    memcpy((void *)&(info), &ack_buffer[18], sizeof(info));
    p_vstream_info->v_frame_timestamp = ov_htonl(info);

    memcpy((void *)&(info), &ack_buffer[22], sizeof(info));
    p_vstream_info->v_frame_number = ov_htonl(info);

    memcpy((void *)&(info), &ack_buffer[26], sizeof(info));
    p_vstream_info->v_frame_type = ov_htonl(info);

exit_ov_get_vstream_info:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
ov_s32 ov_get_vstream_data(ov_u16 const fd, ov_u32 const pkt_size,\
                           ov_u8 *const p_outbuffer)
{
    ov_u32 param = 0;
    ov_u32 read_size = 0;
    ov_s32 ret_val = 0;

    ov_u8  idx = 0;

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    if(0 == pkt_size)
    {
         ret_val = INVALID_INPUT;
         goto exit_ov_get_vstream_data;
    }

    memset(cmd_buffer, 0, sizeof(cmd_buffer));
    memcpy(cmd_buffer, cmd_header, sizeof(cmd_header));
    idx += sizeof(cmd_header);

    cmd_buffer[idx++] = GET_VIDEO_COMMAND_CLASS_ID;
    cmd_buffer[idx++] = 0x01;
    cmd_buffer[idx++] = 0x00;
    cmd_buffer[idx++] = 8;

    idx += 4;                   /* Next 4 bytes are 0 */

    param = ov_htonl(pkt_size);
    memcpy(&cmd_buffer[idx], &param, sizeof(param));
    idx += sizeof(param);

    /* Make it 32 bytes aligned */
    read_size = ((32 -(pkt_size + 18) % 32) + (pkt_size + 18));

    ret_val = ov_execute_cmd_ack_seq(&cmd_buffer[0], &ack_buffer[0],\
                                     p_outbuffer, read_size);
    if(ret_val < 0) goto exit_ov_get_vstream_data;

exit_ov_get_vstream_data:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
ov_s32 ov_get_astream_info(ov_u16 const fd, ov_u32 const pkt_size,
                           ov_astream_info_s *const p_astream_info)
{
    ov_u32 param = 0;
    ov_u32 info = 0;
    ov_s32 ret_val = 0;

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    if(0 == pkt_size)
    {
        ret_val = INVALID_INPUT;
        goto exit_ov_get_astream_info;
    }

    param = ov_htonl(pkt_size);
    ret_val = ov_send_command(GET_AUDIO_COMMAND_CLASS_ID, 0x00,\
                              sizeof(param), (void *)&param, NULL, 0);
    if(ret_val < 0) goto exit_ov_get_astream_info;

    if ((ack_buffer[4] != OV_CMD_ACK) ||
        (cmd_buffer[4] != ack_buffer[7]) ||
        (cmd_buffer[5] != ack_buffer[8]))
    {
        ret_val = OV_RESPONSE_MISMATCH_ERROR;
        goto exit_ov_get_astream_info;
    }

    memcpy((void *)&(info), &ack_buffer[10], sizeof(info));
    p_astream_info->a_total_size = ov_htonl(info);

    memcpy((void *)&(info), &ack_buffer[14], sizeof(info));
    p_astream_info->a_nxt_pkt_size = ov_htonl(info);

exit_ov_get_astream_info:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
ov_s32 ov_get_astream_data(ov_u16 const descriptor, ov_u32 const pkt_size,
                           ov_u8 *const p_outbuffer)
{
    ov_u32 param = 0;
    ov_u32 read_size = 0;
    ov_s32 ret_val = 0;

    ov_u8  idx = 0;

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    if(pkt_size == 0)
    {
         ret_val = INVALID_INPUT;
         goto exit_ov_get_astream_data;
    }

    memset(cmd_buffer, 0, sizeof(cmd_buffer));
    memcpy(cmd_buffer, cmd_header, sizeof(cmd_header));
    idx += sizeof(cmd_header);

    cmd_buffer[idx++] = GET_AUDIO_COMMAND_CLASS_ID;
    cmd_buffer[idx++] = 0x01;
    cmd_buffer[idx++] = 0x00;
    cmd_buffer[idx++] = 8;

    idx += 4;                   /* Next 4 bytes are 0 */

    param = ov_htonl(pkt_size);
    memcpy(&cmd_buffer[idx], &param, sizeof(param));
    idx += sizeof(param);

    /* Make it 32 bytes aligned */
    read_size = ((32 -(pkt_size + 18) % 32) + (pkt_size + 18));

    ret_val = ov_execute_cmd_ack_seq(&cmd_buffer[0], &ack_buffer[0],\
                                     p_outbuffer, read_size);
    if(ret_val < 0) goto exit_ov_get_astream_data;

    if ((ack_buffer[4] != OV_CMD_ACK) ||
        (cmd_buffer[4] != ack_buffer[7]) ||
        (cmd_buffer[5] != ack_buffer[8]))
    {
        /* Response mismatch */
        ret_val = OV_RESPONSE_MISMATCH_ERROR;
        goto exit_ov_get_astream_data;
    }

exit_ov_get_astream_data:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
ov_s32 ov_config_a_status(ov_u16 const fd, ov_u8 const enable)
{
    ov_u8 const param_length = 1;

    ov_s32 ret_val = 0;
    ov_u8  param = 0;

    RTOS_MUTEX_ACQUIRE(ov_if_lock);

    /* Enable/disable Audio stream */
    param = enable;
    ret_val = ov_send_command(AUDIO_CNTRL_COMMAND_CLASS_ID, 0x00,\
                              param_length, &param, NULL, 0);
    if(ret_val < 0) goto exit_ov_config_a_status;

exit_ov_config_a_status:
    RTOS_MUTEX_RELEASE(ov_if_lock);
    return ret_val;
}

/**/
static ov_u32 ov_htonl(ov_u32 val)
{
    unsigned long i = 1;
    signed char *p = (signed char *)&i;

    if (p[0] == 1) /* little endian */
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
static ov_s32 ov_verify_ack(ov_u8 *const p_command, \
                            ov_u8 const *const p_ackbuffer)
{
    ov_s32 ret_val = 0;

    ret_val = p_ackbuffer[9];
    switch (ret_val)
    {
        case OV_RESP_NO_PAYLOAD:
            /* Do nothing */
        break;

        case OV_RESP_PAYLOAD:
            /* Do nothing */
        break;

        case OV_RESP_CMD_FLAG_ERROR:
            ret_val = OV_RESPONSE_FLAG_ERROR;
        break;

        case OV_RESP_CLASS_ID_NOT_SUPPORTED:
            ret_val = OV_RESPONSE_CLASS_ID_ERROR;
        break;

        case OV_RESP_CMD_ID_NOT_SUPPORTED:
            ret_val = OV_RESPONSE_CMD_ID_ERROR;
        break;

        case OV_RESP_INCORRECT_DATA_PACKET_SIZE:
            ret_val = OV_RESPONSE_DATA_PACKET_SIZE_ERROR;
        break;

        case OV_RESP_INVALID_PARAM:
            ret_val = OV_RESPONSE_INVALID_PARAM;
        break;

        case OV_RESP_PARAM_OUT_OF_RANGE:
            ret_val = OV_RESPONSE_OUT_OF_RANGE_PARAM;
        break;

        case OV_RESP_FWID_OUT_OF_RANGE:
            ret_val = OV_RESPONSE_OUT_OF_RANGE_FWID;
        break;

        default:
            ret_val = OV_RESPONSE_UNKNOWN_ERROR;
        break;
    }

    return ret_val;
}

/**/
static ov_s32 ov_send_command(ov_u8 const cmd_class, ov_u8 const cmd_id,    \
                              ov_u8 const param_len, void *p_param,         \
                              ov_u8 *const p_ack_payload,                   \
                              ov_u16 const payload_buf_len)
{
    ov_u8  idx = 0;
    ov_s32 ret_val = 0;

    if(param_len != 0 && p_param == NULL)
    {
        return INVALID_INPUT;
    }

    memset(cmd_buffer, 0, sizeof(cmd_buffer));
    memcpy(cmd_buffer, cmd_header, sizeof(cmd_header));

    idx += sizeof(cmd_header);

    cmd_buffer[idx++] = cmd_class;
    cmd_buffer[idx++] = cmd_id;
    cmd_buffer[idx++] = 0x00;
    cmd_buffer[idx++] = param_len;

    if(0 != param_len)
    {
        memcpy(&cmd_buffer[idx], p_param, param_len);
        idx += param_len;
    }

    ret_val = ov_execute_cmd_ack_seq(&cmd_buffer[0], &ack_buffer[0], p_ack_payload, payload_buf_len);
    if(ret_val < 0) return ret_val;

    ret_val = ov_verify_ack(cmd_buffer, ack_buffer);
    if(ret_val < 0) return ret_val;

    return ret_val;
}

/**/
static ov_s32 ov_execute_cmd_ack_seq(ov_u8 *const p_cmdBuffer,  \
                                     ov_u8 *const p_ackBuffer,  \
                                     ov_u8 *const p_payload,    \
                                     ov_u32 const read_size)
{
    ov_q_msgs_e ov_msg = NO_MESSAGE;
    ov_u8  trial = 0;
    ov_s32 ret_val = 0;

    /* CMD Stage - Start */
    cc_gpio_config_high(GPIO_TI_TO_OV);
    do{
        osi_MsgQRead(&ov_q_obj, &ov_msg, OSI_WAIT_FOREVER);
    }while((ov_msg != OV_TI_GPIO_HIGH) && (trial++ < NUM_OF_TRIALS));
    trial = 0;

    if(ov_msg != OV_TI_GPIO_HIGH) return INVALID_GPIO_RESPONSE;

    /* Send command */
    cc_spi_write(0, p_cmdBuffer, COMMAND_SIZE);

    cc_gpio_config_low(GPIO_TI_TO_OV);
    do
    {
        osi_MsgQRead( &ov_q_obj, &ov_msg, OSI_WAIT_FOREVER);
    }while((ov_msg != OV_TI_GPIO_HIGH) && (trial++ < NUM_OF_TRIALS));
    trial = 0;

    if(ov_msg != OV_TI_GPIO_HIGH) return INVALID_GPIO_RESPONSE;

    /* ACK Stage - Start - A high on OV_TO_TI line indicates that an ACK is
     * already received
     *
     * CMD Stage - End
     */
    cc_gpio_config_high(GPIO_TI_TO_OV);

    if( (NULL != p_payload) &&
        (0 != read_size) )
    {
        cc_spi_read(0, p_ackBuffer, 18);
        cc_spi_read(0, p_payload, (read_size - 18));
    }
    else
    {
        cc_spi_read(0, p_ackBuffer, COMMAND_SIZE);
    }

    cc_gpio_config_low(GPIO_TI_TO_OV);
    do{
         /* Wait for the OV-TI gpio low */
         osi_MsgQRead( &ov_q_obj, &ov_msg, OSI_WAIT_FOREVER);
    }while((ov_msg != OV_TI_GPIO_LOW) && (trial++ < NUM_OF_TRIALS));
    trial = 0;

    if(ov_msg != OV_TI_GPIO_LOW) return INVALID_GPIO_RESPONSE;
    /* ACK Stage - End */

    if ((ack_buffer[0] != 0xFF) || (ack_buffer[1] != 0xFF) ||
        (ack_buffer[2] != 0xFF) || (ack_buffer[3] != 0x01))
    {
        /* Error in response packet format */
        return OV_RESPONSE_MISMATCH_ERROR;
    }

    return ret_val;
}

/**/
static ov_s32 ov_poweroff()
{
    /* Power off the OV788 system */
    cc_gpio_config_low(GPIO_POWER_EN);
    return 0;
}

/**/
static ov_s32 ov_download_fw(ov_u16 fd)
{
    ov_q_msgs_e ov_msg = NO_MESSAGE;
    ov_u32 fw_data_sent = 0;
    ov_u32 f_len_1k_mod = 0;
    ov_u32 f_len_be = 0;
    ov_u32 f_len = 0;
    ov_s32 ret_val = 0;
    ov_s32 f_handle = 0;

    /* Power on the OV788 system */
    cc_gpio_config_high(GPIO_POWER_EN);
    osi_Sleep(100);

    /* Get OVT firmware from file and send it over SPI */
    ret_val = ov_GetFwSize(OVT_FW_FILE, &f_len);
    if(ret_val < 0)
    {
        /* Error */
        Report("GetFileSize -> Error %d", ret_val);
        return OV_FIRMWARE_FILE_OPRATION_ERROR;
    }

    ret_val = ov_OpenFwFile(OVT_FW_FILE, &f_handle);
    if(ret_val < 0)
    {
        /* Error */
        Report("OpenFile -> Error %d", ret_val);
        return OV_FIRMWARE_FILE_OPRATION_ERROR;
    }

    /* FW should in 1K Modem format */
    f_len_1k_mod = (1024 - (f_len % 1024)) + f_len;

    f_len_be = ov_htonl(f_len_1k_mod);

    /* Write Boot loaded init */
    ovt_boot_loader_init[3] = (f_len_be & 0xFF000000) >> 24;
    ovt_boot_loader_init[2] = (f_len_be & 0xFF0000) >> 16;
    ovt_boot_loader_init[1] = (f_len_be & 0xFF00) >> 8;

    cc_spi_write(fd, (unsigned char *)ovt_boot_loader_init, 16);
    osi_Sleep(100);

    while(fw_data_sent < f_len)
    {
        ret_val = ov_GetData(f_handle, fw_buffer, FW_BUFF_SIZE, fw_data_sent);
        if(ret_val < 0)
        {
            /* Error */
            Report("GetData -> Error %d", ret_val);
            ov_CloseFwFile(f_handle);
            return OV_FIRMWARE_FILE_OPRATION_ERROR;
        }
        else
        {
            cc_spi_write(fd, (unsigned char *)fw_buffer, ret_val);
            fw_data_sent += ret_val;
        }
    }

    memset(fw_buffer, 0x1a, (f_len_1k_mod - f_len));
    cc_spi_write(fd, (unsigned char *)fw_buffer, (f_len_1k_mod - f_len));

    ret_val = ov_CloseFwFile(f_handle);
    if(ret_val < 0)
    {
        /* Error */
        Report("CloseFile -> Error %d", ret_val);
    }

    cc_start_timer(3000, ONE_SHOT);

    while(ov_msg != TIMEROUT_EVENT && ov_msg != OV_TI_GPIO_HIGH)
        osi_MsgQRead(&ov_q_obj, &ov_msg, OSI_WAIT_FOREVER);

    if(ov_msg == TIMEROUT_EVENT)
    {
        /* Timeout Error re download the firmwware */
        return OV_FIRMWARE_DOWNLOAD_ERROR;
    }

    cc_stop_timer();

    /* If got the GPIO high trigger, wait for the
     * gpio low
     */
    if(ov_msg == OV_TI_GPIO_HIGH)
    {
        while(ov_msg != OV_TI_GPIO_LOW)
        {
            osi_MsgQRead( &ov_q_obj, &ov_msg, OSI_WAIT_FOREVER);
        }
    }

    return 0;
}
