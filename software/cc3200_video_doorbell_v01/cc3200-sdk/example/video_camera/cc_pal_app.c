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

#include "cc_pal_app.h"
#include "timer_if.h"
#include "ov_sif_if.h"

#define TEMP_BUFF_SIZE          100
#define SPI_IF_BIT_RATE         10000000    /* 10MHz */

extern OsiMsgQ_t ov_q_obj;
extern void ov_irg_handler(void *pValue);

static cc_u32 ulReg[] =
{
    GPIOA0_BASE,
    GPIOA1_BASE,
    GPIOA2_BASE,
    GPIOA3_BASE,
    GPIOA4_BASE
};

cc_u32 timestamp_counter = 0;

/* SPI Read Buffer */
cc_u8 spi_buffer[TEMP_BUFF_SIZE] = {0};

/*
 * Variables to store TIMER Port,Pin values
 */
cc_s32 ti_ov_port       = 0;
cc_s32 ov_ti_port       = 0;
cc_s32 spi_cs_port      = 0;
cc_s32 pwr_enable_port  = 0;
cc_s32 led_port         = 0;
cc_s32 wlan_enable_port = 0;

cc_u8 ti_ov_pin         = 0;
cc_u8 ov_ti_pin         = 0;
cc_u8 spi_cs_pin        = 0;
cc_u8 pwr_enable_pin    = 0;
cc_u8 led_pin           = 0;
cc_u8 wlan_enable_pin   = 0;

P_OV_EVENT_HANDLER      pIraEventHandler = 0;

/*!
 * Get port n pin
 */
static void
cc_gpio_get_port_n_pin(cc_u8 ucPin, cc_s32 *puiGPIOPort, cc_u8 *pucGPIOPin);

/*!
 * Set a value to the specified GPIO pin
 */
static void
cc_gpio_if_set(cc_u8 ucPin, cc_s32 uiGPIOPort, cc_u8 ucGPIOPin, cc_u8 ucGPIOValue);

/**/
void cc_timer_a0_int_handler()
{
    ov_q_msgs_e var = TIMEROUT_EVENT;
    Timer_IF_InterruptClear(TIMERA0_BASE);
    osi_MsgQWrite(&ov_q_obj,&var,OSI_NO_WAIT);
}

/**/
void cc_start_timer(cc_u32 ulValue, e_TimerModes type)
{
    /* Configuring the timers */
    if(type == ONE_SHOT)
    {
        Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_ONE_SHOT, TIMER_A, 0);
    }
    else
    {
        Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    }

    /* Setup the interrupts for the timer timeouts. */
    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, cc_timer_a0_int_handler);

    /* Turn on the timers feeding values in mSec */
    Timer_IF_Start(TIMERA0_BASE, TIMER_A, ulValue);
}

/**/
void cc_stop_timer()
{
    Timer_IF_Stop(TIMERA0_BASE, TIMER_A);
    Timer_IF_DeInit(TIMERA0_BASE, TIMER_A);
}

/**/
void cc_timer_a2_int_handler()
{
    cc_u32 intr_status;

    intr_status = MAP_TimerIntStatus(TIMERA2_BASE, true);
    MAP_TimerIntClear(TIMERA2_BASE, intr_status);
    timestamp_counter++;
}

/**/
cc_u32 cc_get_timestamp()
{
    return timestamp_counter;
}

/**/
void cc_start_timestamp_cnt()
{
    timestamp_counter = 0;

    /* Initialize GPT A0 (in 32 bit mode) as periodic down counter. */
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA2, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_TIMERA2);
    MAP_TimerConfigure(TIMERA2_BASE,TIMER_CFG_PERIODIC);
    MAP_TimerPrescaleSet(TIMERA2_BASE,TIMER_BOTH , 0);

    /* Setup the interrupts for the timer timeouts. */
#if defined(USE_TIRTOS) || defined(USE_FREERTOS) || defined(SL_PLATFORM_MULTI_THREADED)
    /*
     * USE_TIRTOS: if app uses TI-RTOS (either networking/non-networking)
     * USE_FREERTOS: if app uses Free-RTOS (either networking/non-networking)
     * SL_PLATFORM_MULTI_THREADED: if app uses any OS + networking(simplelink)
     */
    osi_InterruptRegister(INT_TIMERA2A, cc_timer_a2_int_handler,\
                          INT_PRIORITY_LVL_1);
    osi_InterruptRegister(INT_TIMERA2B, cc_timer_a2_int_handler,\
                          INT_PRIORITY_LVL_1);
#else
    MAP_IntPrioritySet(GetPeripheralIntNum(TIMERA2_BASE, TIMER_A), INT_PRIORITY_LVL_1);
    MAP_TimerIntRegister(TIMERA2_BASE, TIMER_BOTH, cc_timer_a2_int_handler);
#endif


    MAP_TimerIntEnable(TIMERA2_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT);
    MAP_TimerLoadSet(TIMERA2_BASE,TIMER_BOTH, TIMER_LOAD_VALUE);

    /* Enable the GPT */
    MAP_TimerEnable(TIMERA2_BASE, TIMER_BOTH);
}

/**/
void cc_stop_timestamp_cnt()
{
    /* Disable the GPT */
    MAP_TimerDisable(TIMERA2_BASE,TIMER_BOTH);
    timestamp_counter = 0;
}

/**/
void cc_gpio_a1_irq_handler()
{
    cc_u8 status        = 0;
    cc_u32 ulPinState   =  GPIOIntStatus(ov_ti_port, 0XFF);

    MAP_GPIOIntClear(ov_ti_port, ov_ti_pin);

    status = cc_gpio_if_status(GPIO_OV_TO_TI);
    if(ulPinState & ov_ti_pin)
    {
        if(pIraEventHandler != NULL) pIraEventHandler(&status);
    }
}

/**/
void cc_config_ov_irq(P_OV_EVENT_HANDLER fptr_intr_hndl)
{
    /* Set Interrupt Type for GPIO */
    MAP_IntDisable(INT_GPIOA1);
    MAP_GPIOIntTypeSet(GPIOA1_BASE,GPIO_PIN_0, GPIO_BOTH_EDGES);
    pIraEventHandler = fptr_intr_hndl;
    MAP_GPIOIntClear(GPIOA1_BASE,GPIO_PIN_0);
    MAP_IntPendClear(INT_GPIOA1);

    /* Register Interrupt handler */
#if defined(USE_TIRTOS) || defined(USE_FREERTOS) || defined(SL_PLATFORM_MULTI_THREADED)
   /*
    * USE_TIRTOS: if app uses TI-RTOS (either networking/non-networking)
    * USE_FREERTOS: if app uses Free-RTOS (either networking/non-networking)
    * SL_PLATFORM_MULTI_THREADED: if app uses any OS + networking(simplelink)
    */
    osi_InterruptRegister(INT_GPIOA1,(P_OSI_INTR_ENTRY)cc_gpio_a1_irq_handler,\
                          INT_PRIORITY_LVL_1);
#else
    MAP_IntPrioritySet(INT_GPIOA1, INT_PRIORITY_LVL_1);
    MAP_GPIOIntRegister(GPIOA1_BASE, cc_gpio_a1_irq_handler);
#endif

    /* Enable Interrupt */
    MAP_IntEnable(INT_GPIOA1);
    MAP_GPIOIntEnable(GPIOA1_BASE,GPIO_INT_PIN_0);
}

/**/
void cc_gpio_configure(cc_u8 pins)
{
      if(pins & GPIO_TI_TO_OV)
      {
        cc_gpio_get_port_n_pin(GPIO_TI_TO_OV,
                            &ti_ov_port,
                            &ti_ov_pin);
      }

      if(pins & GPIO_OV_TO_TI)
      {
        cc_gpio_get_port_n_pin(GPIO_OV_TO_TI,
                      &ov_ti_port,
                      &ov_ti_pin);
      }

      if(pins & GPIO_SPI_CS)
      {
        cc_gpio_get_port_n_pin(GPIO_SPI_CS,
                      &spi_cs_port,
                      &spi_cs_pin);
      }

      if(pins & GPIO_POWER_EN)
      {
        cc_gpio_get_port_n_pin(GPIO_POWER_EN,
                      &pwr_enable_port,
                      &pwr_enable_pin);
      }

      if(pins & GPIO_LED)
      {
        cc_gpio_get_port_n_pin(GPIO_LED,
                      &led_port,
                      &led_pin);
      }

      if(pins & GPIO_WLAN_ON)
      {
        cc_gpio_get_port_n_pin(GPIO_WLAN_ON,
                      &wlan_enable_port,
                      &wlan_enable_pin);
      }
}

/**/
void cc_gpio_config_high(char pins)
{
    switch(pins)
    {
        case GPIO_TI_TO_OV:
        {
          cc_gpio_if_set(GPIO_TI_TO_OV, ti_ov_port, ti_ov_pin, 1);
          break;
        }
        case GPIO_OV_TO_TI:
        {
          cc_gpio_if_set(GPIO_OV_TO_TI, ov_ti_port, ov_ti_pin, 1);
          break;
        }
        case GPIO_SPI_CS:
        {
          cc_gpio_if_set(GPIO_OV_TO_TI, spi_cs_port, spi_cs_pin, 1);
          break;
        }
        case GPIO_POWER_EN:
        {
            cc_gpio_if_set(GPIO_POWER_EN, pwr_enable_port, pwr_enable_pin, 1);
            break;
        }
        case GPIO_LED:
        {
            cc_gpio_if_set(GPIO_LED, led_port, led_pin, 1);
            break;
        }
        case GPIO_WLAN_ON:
        {
            cc_gpio_if_set(GPIO_WLAN_ON, wlan_enable_port, wlan_enable_pin, 1);
            break;
        }

        default:
          break;
    }
}

/**/
void cc_gpio_config_low(char pins)
{
    switch(pins)
    {
        case GPIO_TI_TO_OV:
        {
          cc_gpio_if_set(GPIO_TI_TO_OV, ti_ov_port, ti_ov_pin, 0);
          break;
        }
        case GPIO_OV_TO_TI:
        {
          cc_gpio_if_set(GPIO_OV_TO_TI, ov_ti_port, ov_ti_pin, 0);
          break;
        }
        case GPIO_SPI_CS:
        {
          cc_gpio_if_set(GPIO_OV_TO_TI, spi_cs_port, spi_cs_pin, 0);
          break;
        }
        case GPIO_POWER_EN:
        {
            cc_gpio_if_set(GPIO_POWER_EN, pwr_enable_port, pwr_enable_pin, 0);
            break;
        }
        case GPIO_LED:
        {
            cc_gpio_if_set(GPIO_LED, led_port, led_pin, 0);
            break;
        }
        case GPIO_WLAN_ON:
        {
            cc_gpio_if_set(GPIO_WLAN_ON, wlan_enable_port, wlan_enable_pin, 0);
            break;
        }

        default:
          break;
    }
}

/**/
cc_u8 cc_gpio_if_get(cc_u8 ucPin, cc_s32 uiGPIOPort, cc_u8 ucGPIOPin)
{
    cc_u8 ucGPIOValue;
    long lGPIOStatus;

    /*
     * Invoke the API to Get the value
     */
    lGPIOStatus =  MAP_GPIOPinRead(uiGPIOPort,ucGPIOPin);

    /*
     * Set the corresponding bit in the bitmask
     */
    ucGPIOValue = lGPIOStatus >> (ucPin % 8);
    return ucGPIOValue;
}

/**/
cc_u8 cc_gpio_if_status(cc_u8 ucGPIONum)
{
  cc_u8 ucGPIOStatus;

  switch(ucGPIONum)
  {
      case GPIO_OV_TO_TI:
      {
          ucGPIOStatus = cc_gpio_if_get(GPIO_OV_TO_TI, ov_ti_port, ov_ti_pin);
        break;
      }

      default:
        break;
  }

  return ucGPIOStatus;
}

/**/
cc_s32 cc_configure_spi()
{
    /* Reset SPI */
    MAP_SPIReset(GSPI_BASE);

    /* Configure SPI interface */
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_3PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));

    /* Enable SPI */
    MAP_SPIEnable(GSPI_BASE);

    cc_gpio_configure(GPIO_SPI_CS);
    cc_gpio_config_high(GPIO_SPI_CS);

    return 0;
}

/**/
cc_s32 cc_spi_read(cc_u16 fd, cc_u8 *buff, cc_u32 len)
{
    /* Assert CS */
    cc_gpio_config_low(GPIO_SPI_CS);

    MAP_SPITransfer(GSPI_BASE, buff, buff, len,
                    SPI_CS_ENABLE|SPI_CS_DISABLE);

    /* Deassert CS */
    cc_gpio_config_high(GPIO_SPI_CS);

    return len;
}

/**/
cc_s32 cc_spi_write(cc_u16 fd, cc_u8 *buff, cc_u32 len)
{
    cc_s32 wSize = len;

    /* Assert CS */
    cc_gpio_config_low(GPIO_SPI_CS);

    while(wSize > 0)
    {
        if(wSize > TEMP_BUFF_SIZE)
        {
           MAP_SPITransfer(GSPI_BASE, &buff[len - wSize], spi_buffer,\
                           TEMP_BUFF_SIZE, SPI_CS_ENABLE|SPI_CS_DISABLE);

           wSize -= TEMP_BUFF_SIZE;
        }
        else
        {
               MAP_SPITransfer(GSPI_BASE, &buff[len - wSize], spi_buffer,\
                               wSize, SPI_CS_ENABLE|SPI_CS_DISABLE);

               wSize = 0;
        }
    }

    /* Deassert CS */
    cc_gpio_config_high(GPIO_SPI_CS);
    return len;
}

/**/
void config_interface()
{
    cc_configure_spi();
    cc_gpio_configure(GPIO_TI_TO_OV);
    cc_gpio_configure(GPIO_OV_TO_TI);

    cc_config_ov_irq(ov_irg_handler);
}

/**/
static void
cc_gpio_get_port_n_pin(cc_u8 ucPin, cc_s32 *puiGPIOPort, cc_u8 *pucGPIOPin)
{
    /*
     * Get the GPIO pin from the external Pin number
     */
    *pucGPIOPin = 1 << (ucPin % 8);

    /*
     * Get the GPIO port from the external Pin number
     */
    *puiGPIOPort = (ucPin / 8);
    *puiGPIOPort = ulReg[*puiGPIOPort];
}

/*!
 * Set a value to the specified GPIO pin
 */
static void
cc_gpio_if_set(cc_u8 ucPin, cc_s32 uiGPIOPort, cc_u8 ucGPIOPin, cc_u8 ucGPIOValue)
{
    /*
     * Set the corresponding bit in the bitmask
     */
    ucGPIOValue = ucGPIOValue << (ucPin % 8);

    /*
     * Invoke the API to set the value
     */
    MAP_GPIOPinWrite(uiGPIOPort,ucGPIOPin,ucGPIOValue);
}
