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
#include "ov_sif_if.h"

#include "video_tx_task.h"
#include "audio_tx_task.h"
#include "rtsp_main_task.h"
#include "app_common.h"
#include "get_av_task.h"

/*!
 * Task Q Handlers
 */
extern OsiMsgQ_t    app_rtsp_task_q;
extern OsiMsgQ_t    app_av_task_q;
extern OsiMsgQ_t    app_v_task_q;
extern OsiMsgQ_t    app_a_task_q;
extern v_data_s     app_v_data;
extern a_data_s     app_a_data;

cc_s16  fd = 0;

/*!
 * Sends status/response to the sender of the current command
 */
static cc_s32
send_status(app_task_id_e const task_id, cc_s32 const status);

/*!
 * Entry function
 */
void app_get_av_task_entry(void *p_args)
{
    av_task_msg_s   msg_on_av_task_q;
    cc_s32          status  = -1;

    memset(&msg_on_av_task_q, 0, sizeof(msg_on_av_task_q));

    /*!
     * Wait for the message and parse it
     */
    while(OSI_OK == osi_MsgQRead( &app_av_task_q, &msg_on_av_task_q, OSI_WAIT_FOREVER))
    {
        switch(msg_on_av_task_q.msg_id)
        {
            case INIT_OV788:
            {
                Report("Processing 'INIT_OV788' message\r\n");

                status = ov_init();
                if(status < 0) ERR_PRINT(status);

                fd = status;
            }
            break;

            case V_CONFIG_SENSOR:
            {
                ov_v_config_s v_config_data;

                Report("Processing 'V_CONFIG_SENSOR' message\r\n");

                if(NULL == msg_on_av_task_q.p_data)
                {
                    status = STATUS_NULL_DATA_POINTER;
                    break;
                }

                memcpy((void *)&v_config_data, msg_on_av_task_q.p_data,\
                       sizeof(ov_v_config_s));

                status = ov_config_v_sensor(fd, v_config_data);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            case V_SET_FRAME_RATE:
            {
                ov_v_framerate_e frame_rate;

                Report("Processing 'V_SET_FRAME_RATE' message\r\n");

                if(NULL == msg_on_av_task_q.p_data)
                {
                    status = STATUS_NULL_DATA_POINTER;
                    break;
                }

                memcpy((void *)&frame_rate, msg_on_av_task_q.p_data,\
                       sizeof(ov_v_framerate_e));

                status = ov_set_frame_rate(fd, frame_rate);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            case V_SET_BITRATE:
            {
                ov_v_bitrate_e bit_rate;

                Report("Processing 'V_SET_BITRATE' message\r\n");

                if(NULL == msg_on_av_task_q.p_data)
                {
                    status = STATUS_NULL_DATA_POINTER;
                    break;
                }

                memcpy((void *)&bit_rate, msg_on_av_task_q.p_data,\
                       sizeof(ov_v_bitrate_e));

                status = ov_set_bit_rate(fd, bit_rate);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            case V_SET_ZOOM:
            {
                cc_u8 zoom = 0;

                Report("Processing 'V_SET_ZOOM' message\r\n");

                if(NULL == msg_on_av_task_q.p_data)
                {
                    status = STATUS_NULL_DATA_POINTER;
                    break;
                }

                memcpy((void *)&zoom, msg_on_av_task_q.p_data, 1);
                status = ov_set_zoom(fd, zoom);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            case V_SET_MOTION_DETECT:
            {
                cc_u8 enable = 0;

                Report("Processing 'V_SET_MOTION_DETECT' message\r\n");

                if(NULL == msg_on_av_task_q.p_data)
                {
                    status = STATUS_NULL_DATA_POINTER;
                    break;
                }

                enable = *((cc_u8 *)msg_on_av_task_q.p_data);
                status = ov_set_motion_detect(fd, enable);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            case A_CONFIG_AUDIO:
            {
                ov_a_config_s a_config_data;
                Report("Processing 'A_CONFIG_AUDIO' message\r\n");

                if(NULL == msg_on_av_task_q.p_data)
                {
                    status = STATUS_NULL_DATA_POINTER;
                    break;
                }

                memcpy((void *)&a_config_data, msg_on_av_task_q.p_data,\
                        sizeof(ov_a_config_s));

                status = ov_config_a_codec(fd,a_config_data);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            case A_SET_SAMPLE_RATE:
            {
                ov_a_samplerate_e sample_rate;

                Report("Processing 'A_SET_SAMPLE_RATE' message\r\n");

                if(NULL == msg_on_av_task_q.p_data)
                {
                    status = STATUS_NULL_DATA_POINTER;
                    break;
                }

                memcpy((void *)&sample_rate, msg_on_av_task_q.p_data,\
                       sizeof(ov_a_samplerate_e));

                status = ov_set_sampling_rate(fd,sample_rate);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            case V_ENABLE:
            {
                ov_v_resolution_e resolution;
                cc_u8 enable = 1;

                Report("Processing 'V_ENABLE' message\r\n");

                if(NULL == msg_on_av_task_q.p_data)
                {
                    status = STATUS_NULL_DATA_POINTER;
                    break;
                }

                memcpy((void *)&resolution, msg_on_av_task_q.p_data,\
                       sizeof(ov_v_resolution_e));
                status = ov_config_v_encoder(fd, resolution, enable);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            case V_DISABLE:
            {
                ov_v_resolution_e resolution;
                cc_u8 enable = 0;

                Report("Processing 'V_DISABLE' messqage\r\n");

                if(NULL == msg_on_av_task_q.p_data)
                {
                    status = STATUS_NULL_DATA_POINTER;
                    break;
                }

                memcpy((void *)&resolution, msg_on_av_task_q.p_data,\
                       sizeof(ov_v_resolution_e));
                status = ov_config_v_encoder(fd, resolution, enable);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            case V_GET_INFO:
            {
                ov_vstream_info_s *p_stream_info = NULL;

                //Report("Processing 'V_GET_INFO' message\r\n");
                if(NULL == msg_on_av_task_q.p_data)
                {
                    status = STATUS_NULL_DATA_POINTER;
                    break;
                }

                p_stream_info = (ov_vstream_info_s *)msg_on_av_task_q.p_data;
                status = ov_get_vstream_info(fd, VIDEO_RECV_SIZE, p_stream_info);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            case V_GET_DATA:
            {
                //Report("Processing 'V_GET_DATA' message\r\n");

                if(app_v_data.vstream_info_current.v_nxt_pkt_size > VIDEO_RECV_SIZE)
                {
                    app_v_data.rx_pkt_size = VIDEO_RECV_SIZE;
                }
                else
                {
                    app_v_data.rx_pkt_size = app_v_data.vstream_info_current.v_nxt_pkt_size;
                }

                status = ov_get_vstream_data(fd, app_v_data.rx_pkt_size,\
                                             app_v_data.v_data_buffer);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            case A_ENABLE:
            {
                cc_u8 enable = 1;

                Report("Processing 'A_ENABLE' message\r\n");

                status = ov_config_a_status(fd, enable);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            case A_DISABLE:
            {
                cc_u8 enable = 0;

                Report("Processing 'A_DISABLE' message\r\n");

                status = ov_config_a_status(fd, enable);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            case A_GET_INFO:
            {
                ov_astream_info_s *p_stream_info = NULL;

                //Report("Processing 'A_GET_INFO' message\r\n");

                if(NULL == msg_on_av_task_q.p_data)
                {
                    status = STATUS_NULL_DATA_POINTER;
                    break;
                }

                p_stream_info = (ov_astream_info_s *)msg_on_av_task_q.p_data;
                status = ov_get_astream_info(fd, AUDIO_RECV_SIZE, p_stream_info);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            case A_GET_DATA:
            {
                //Report("Processing 'A_GET_DATA' message\r\n");
                //Report("Packet size  %d\r\n", app_a_data.astream_info.a_nxt_pkt_size);
                if(app_a_data.astream_info.a_nxt_pkt_size > AUDIO_RECV_SIZE)
                {
                    app_a_data.rx_pkt_size = AUDIO_RECV_SIZE;
                }
                else
                {
                    app_a_data.rx_pkt_size = app_a_data.astream_info.a_nxt_pkt_size;
                }

                status = ov_get_astream_data(fd, app_a_data.rx_pkt_size, app_a_data.a_data_buffer);
                if(status < 0) ERR_PRINT(status);
            }
            break;

            default:
            {
                Report("Unknown message on AV Task-Q\r\n");
                status = AV_ERROR_UNKNOWN_MSGID;
            }
            break;
        }

        /*!
         * Send status back to unblock the Sender-Task
         */
        send_status(msg_on_av_task_q.task_id, status);
    }

    return;
}

/*!
 */
static cc_s32
send_status(app_task_id_e const task_id, cc_s32 const status)
{
    switch(task_id)
    {
        case AUDIO_TASK_ID:
        {
            a_task_msg_s  msg_on_a_task_q;

            msg_on_a_task_q.task_id = AV_TASK_ID;
            msg_on_a_task_q.msg_id  = STATUS_MSG;
            msg_on_a_task_q.msg     = status;

            osi_MsgQWrite(&app_a_task_q, &msg_on_a_task_q, OSI_NO_WAIT);
        }
        break;

        case VIDEO_TASK_ID:
        {
            v_task_msg_s  msg_on_v_task_q;

            msg_on_v_task_q.task_id = AV_TASK_ID;
            msg_on_v_task_q.msg_id  = V_STATUS_MSG;
            msg_on_v_task_q.msg     = status;

            osi_MsgQWrite(&app_v_task_q, &msg_on_v_task_q, OSI_NO_WAIT);
        }
        break;

        case RTSP_TASK_ID:
        {
            app_rtsp_task_msg_s msg_on_rtsp_task_q;

            msg_on_rtsp_task_q.task_id  = AV_TASK_ID;
            msg_on_rtsp_task_q.msg_id   = RTSP_STATUS_MSG;
            msg_on_rtsp_task_q.msg      = status;

            osi_MsgQWrite(&app_rtsp_task_q, &msg_on_rtsp_task_q, OSI_NO_WAIT);
        }
        break;

        default:
        break;
    }

    return 0;
}
