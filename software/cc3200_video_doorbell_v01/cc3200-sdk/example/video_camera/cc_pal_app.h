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
#ifndef _CC_PAL_APP_H__
#define _CC_PAL_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>

#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "interrupt.h"
#include "timer.h"
#include "gpio.h"

#include "osi.h"

#ifndef cc_u32
typedef unsigned long   cc_u32;
#endif

#ifndef cc_u16
typedef unsigned short  cc_u16;
#endif

#ifndef cc_u8
typedef unsigned char   cc_u8;
#endif

#ifndef cc_s32
typedef signed long     cc_s32;
#endif

#ifndef cc_s16
typedef signed short    cc_s16;
#endif

#ifndef cc_s8
typedef signed char     cc_s8;
#endif

#define GPIO_TI_TO_OV   7
#define GPIO_OV_TO_TI   8
#define GPIO_SPI_CS     17
#define GPIO_POWER_EN   13
#define GPIO_LED        22
#define GPIO_WLAN_ON    11

#define _10_POW_9               (1000000000)
#define SYS_CLK_FREQ            (80000000)
#define TIMER_CNTS_PER_SEC      (50)

#define TIMER_LOAD_VALUE        (SYS_CLK_FREQ/TIMER_CNTS_PER_SEC)
#define TIME_PER_TICK_IN_NSECS  (1/SYS_CLK_FREQ) * (_10_POW_9)

/*!
    \addtogroup CC_PAL_IF
    @{
*/
typedef void (*P_OV_EVENT_HANDLER)(void* pValue);


/*!
 *  \brief  Timer modes
 */
typedef enum
{
    ONE_SHOT,
    PERIODIC

}e_TimerModes;

/*!
    \brief Start the TIMERA0

    \param[in]  ulValue :   Pointer to file name to be opened
    \param[in]  type    :   Timer type (ONE_SHOT/PERIODIC)

    \note
*/
void cc_start_timer(cc_u32 ulValue, e_TimerModes type);

/*!
    \brief Stop the configured TIMERA0

    \note
*/
void cc_stop_timer();

/*!
    \brief Configure the interrupt on OV RDY GPIO pin

    \param[in] InterruptHdl :   Interrupt handler

    \note
*/
void cc_config_ov_irq(P_OV_EVENT_HANDLER fptr_intr_hndl);

/*!
    \brief Configure the GPIO

    \param[in]  ucPins  :   GPIO to be configured

    \note
*/
void cc_gpio_configure(cc_u8 pins);

/*!
    \brief Configure the GPIO to be high

    \param[in]  ucPins  :   GPIO to be configured

    \note
*/
void cc_gpio_config_high(char ucPins);

/*!
    \brief Configure the GPIO to be low

    \param[in]  ucPins  :   GPIO to be configured

    \note
*/
void cc_gpio_config_low(char ucPins);

/*!
    \brief Get the GPIO status

    \param[in] ucGPIONum : GPIO number

    \note
*/
cc_u8 cc_gpio_if_status(cc_u8 ucGPIONum);

/*!
    \brief Configure the SPI interface

    \return     the Interface handler , negative error-code on error

    \note
*/
cc_s32 cc_configure_spi();

/*!
    \brief Get the data over SPI

    \param[in]  fd   :   interface handler
    \param[out] buff :   pointer to buffer to recv the data
    \param[in]  len  :   size of the data to be read

    \return     size of the data recevied, negative error-code on error

    \note
*/
cc_s32 cc_spi_read(cc_u16 fd, cc_u8 *buff, cc_u32 len);

/*!
    \brief Get the size of the file

    \param[in]  fd   :   interface handler
    \param[in]  buff :   pointer to buffer containg data to be send
    \param[in]  len  :   size of the data to be send

    \return     size of the data send, negative error-code on error

    \note
*/
cc_s32 cc_spi_write(cc_u16 fd, cc_u8 *buff, cc_u32 len);

/*!
    \brief Configure the interface used to communicate with OV788

    \return  None

    \note
*/
void config_interface();

/*!
    \brief Get the timestamp timer value

    \return  None

    \note
*/
cc_u32 cc_get_timestamp();

/*!
    \brief Start the timestamp timer

    This timer generate 60 count value in one second

    \return  None

    \note
*/
void cc_start_timestamp_cnt();

/*!
    \brief Stop the timestamp value

    \return  None

    \note
*/
void cc_stop_timestamp_cnt();

/*!

 Close the Doxygen group.
 @}

 */

#ifdef  __cplusplus
}
#endif /*  __cplusplus */
#endif  /* _CC_PAL_APP_H__ */

