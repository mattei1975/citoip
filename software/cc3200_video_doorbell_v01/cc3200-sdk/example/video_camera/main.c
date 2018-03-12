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
 *    from this software0 without specific prior written permission.
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
#include "hw_types.h"

#include "simplelink.h"

#include "uart_if.h"
#include "cc_pm.h"

#include "rtsp_main_task.h"
#include "video_tx_task.h"
#include "audio_tx_task.h"
#include "get_av_task.h"
#include "app_common.h"
#include "pinmux.h"

#define STACK_SIZE              (2048)
#define TOTAL_Q_ELEMENTS        (10)

#define TASK_PRIORITY_L         (2)
#define TASK_PRIORITY_M         (TASK_PRIORITY_L + 1)
#define TASK_PRIORITY_H         (TASK_PRIORITY_M + 1)
#define TASK_PRIORITY_SL        (TASK_PRIORITY_H + 1)

/*!
 */
extern int platform_init();

#if defined(ccs) || defined (gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

/*!
 */
OsiMsgQ_t   app_rtsp_rx_task_q;
OsiMsgQ_t   app_rtsp_task_q;
OsiMsgQ_t   app_av_task_q;
OsiMsgQ_t   app_v_task_q;
OsiMsgQ_t   app_a_task_q;

/*!
 */
app_state_e curr_state = APP_STATE_START;

/*!
 * Board Initialization & Configuration
 */
static void board_init(void);

#ifdef USE_FREERTOS
/*!
 * \brief Application defined hook (or callback) function - assert
 *
 * \param[in]  pcFile - Pointer to the File Name
 * \param[in]  ulLine - Line Number
 *
 * \return none
 */
void vAssertCalled(const cc_s8 *pcFile, cc_u32 ulLine)
{
    /* Handle Assert here */
    while(1)
        ;
}

/*!
 * \brief Application defined idle task hook
 * \param  none
 * \return none
 */
void vApplicationIdleHook(void)
{
    /* Handle Idle Hook for Profiling, Power Management etc */
    cc_idle_task_pm();
}

/*!
 * \brief Application defined malloc failed hook
 * \param  none
 * \return none
 */
void vApplicationMallocFailedHook()
{
    /* Handle Memory Allocation Errors */
    while(1)
        ;
}

/*!
 * \brief Application defined stack overflow hook
 * \param  none
 * \return none
 */
void vApplicationStackOverflowHook(OsiTaskHandle *pxTask, cc_s8 *pcTaskName)
{
    /* Handle FreeRTOS Stack Overflow */
    while(1)
        ;
}
#endif /* USE_FREERTOS */

/*!
 * \brief callback function for gpio interrupt handler
 * \param gpio_num is the gpio number which has triggered the interrupt
 * \return 0
 */
cc_s32 gpio_intr_hndlr(cc_s32 gpio_num)
{
    //Report("\r\n\r\n[PM]-> GPIO Trigger \r\n\r\n");
    return 0;
}

/*!
 */
static void board_init(void)
{
    /*!
     * In case of TI-RTOS vector table is initialize by OS itself
     */
    #ifndef USE_TIRTOS
        /*!
         * Set vector table base
         */
        #if defined(ccs) || defined(gcc)
            MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
        #endif

        #if defined(ewarm)
            MAP_IntVTableBaseSet((unsigned long)&__vector_table);
        #endif
    #endif /* USE_TIRTOS */

    /* Enable Processor */
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

/*!
 */
void app_set_state(app_state_e const state)
{
    curr_state = state;
}

/*!
 */
app_state_e app_get_state()
{
    return curr_state;
}

/*!
 */
void main(int argc, char **argv)
{
    OsiReturnVal_e ret_val = OSI_FAILURE;

    /*!
     * Initialize the board & platform configurations
     */
    board_init();
    pinmux_config();
    platform_init();

    #ifdef DEBUG_PRINT
    InitTerm();
    #endif /* DEBUG_PRINT */

    /*!
     * Enable Hystarisis
     */
    HWREG(0x4402E144) = HWREG(0x4402e144) | (3 << 2);

    cc_gpio_configure(GPIO_LED);
    cc_gpio_configure(GPIO_WLAN_ON);
    cc_gpio_configure(GPIO_POWER_EN);

    cc_gpio_config_low(GPIO_LED);
    cc_gpio_config_high(GPIO_WLAN_ON);
    cc_gpio_config_low(GPIO_POWER_EN);

    /*
     * Start the SimpleLink Spawn Task
     */
    VStartSimpleLinkSpawnTask(TASK_PRIORITY_SL);

    /*!
     * Create the required queues for the application
     */
    ret_val = osi_MsgQCreate(&app_rtsp_rx_task_q, "RTSP_RX_TASK_Q",\
                             sizeof(app_rtsp_rx_task_msg_s), TOTAL_Q_ELEMENTS);
    if(OSI_OK != ret_val)  LOOP_FOREVER();

    ret_val = osi_MsgQCreate(&app_rtsp_task_q, "RTSP_TASK_Q",\
                             sizeof(app_rtsp_task_msg_s), TOTAL_Q_ELEMENTS);
    if(OSI_OK != ret_val)  LOOP_FOREVER();

    ret_val = osi_MsgQCreate(&app_av_task_q, "AV_TASK_Q",\
                             sizeof(av_task_msg_s), TOTAL_Q_ELEMENTS);
    if(OSI_OK != ret_val) LOOP_FOREVER();

    ret_val = osi_MsgQCreate(&app_a_task_q, "A_TASK_Q",\
                             sizeof(a_task_msg_s), TOTAL_Q_ELEMENTS);
    if(OSI_OK != ret_val)  LOOP_FOREVER();

    ret_val = osi_MsgQCreate(&app_v_task_q, "V_TASK_Q",\
                             sizeof(v_task_msg_s), TOTAL_Q_ELEMENTS);
    if(OSI_OK != ret_val)  LOOP_FOREVER();

    /*!
     * Create the required tasks for the application
     */
    ret_val = osi_TaskCreate(app_main_task_entry, (const cc_s8 *)"RTSP_MAIN_TASK",\
                             STACK_SIZE, NULL, TASK_PRIORITY_H, NULL);
    if(OSI_OK != ret_val) LOOP_FOREVER();

    ret_val = osi_TaskCreate(app_rtsp_rx_task_entry, (const cc_s8 *)"RTSP_RX_TASK",\
                             STACK_SIZE, NULL, TASK_PRIORITY_M, NULL);
    if(OSI_OK != ret_val) LOOP_FOREVER();

    ret_val = osi_TaskCreate(app_get_av_task_entry, (const cc_s8 *)"AV_TASK",\
                             STACK_SIZE, NULL, TASK_PRIORITY_M, NULL);
    if(OSI_OK != ret_val) LOOP_FOREVER();

    ret_val = osi_TaskCreate(app_a_tx_task_entry, (const cc_s8 *)"A_TASK",\
                             STACK_SIZE, NULL, TASK_PRIORITY_L, NULL);
    if(OSI_OK != ret_val) LOOP_FOREVER();

    ret_val = osi_TaskCreate(app_v_tx_task_entry, (const cc_s8 *)"V_TASK",\
                             STACK_SIZE, NULL, TASK_PRIORITY_L, NULL);
    if(OSI_OK != ret_val) LOOP_FOREVER();

    /*!
     * Start the task scheduler
     */
    osi_start();
}
