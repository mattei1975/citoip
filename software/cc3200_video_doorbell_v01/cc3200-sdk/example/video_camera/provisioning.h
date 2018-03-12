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

#ifndef __NETWORK_IF_H__
#define __NETWORK_IF_H__

#include "simplelink.h"

typedef enum
{
	NETWORK_ERROR_UNKNOWN = -1,
	NETWORK_ERRORPROVISIONING_AP_FAILED  = NETWORK_ERROR_UNKNOWN - 1,

}e_NetworkStatusCode;

/* Status bits - These are used to set/reset the corresponding bits in a 'status_variable' */
typedef enum{
    STATUS_BIT_CONNECTION =  0, /* If this bit is:
                                 *      1 in a 'status_variable', the device is connected to the AP
                                 *      0 in a 'status_variable', the device is not connected to the AP
                                 */

    STATUS_BIT_STA_CONNECTED,   /* If this bit is:
                                 *      1 in a 'status_variable', client is connected to device
                                 *      0 in a 'status_variable', client is not connected to device
                                 */

    STATUS_BIT_IP_ACQUIRED,     /* If this bit is:
                                 *      1 in a 'status_variable', the device has acquired an IP
                                 *      0 in a 'status_variable', the device has not acquired an IP
                                 */

    STATUS_BIT_IP_LEASED,       /* If this bit is:
                                 *      1 in a 'status_variable', the device has leased an IP
                                 *      0 in a 'status_variable', the device has not leased an IP
                                 */

    STATUS_BIT_CONNECTION_FAILED,   /* If this bit is:
                                     *      1 in a 'status_variable', failed to connect to device
                                     *      0 in a 'status_variable'
                                     */

    STATUS_BIT_P2P_NEG_REQ_RECEIVED,/* If this bit is:
                                     *      1 in a 'status_variable', connection requested by remote wifi-direct device
                                     *      0 in a 'status_variable',
                                     */
    STATUS_BIT_SMARTCONFIG_DONE,    /* If this bit is:
                                     *      1 in a 'status_variable', smartconfig completed
                                     *      0 in a 'status_variable', smartconfig event couldn't complete
                                     */

    STATUS_BIT_SMARTCONFIG_STOPPED  /* If this bit is:
                                     *      1 in a 'status_variable', smartconfig process stopped
                                     *      0 in a 'status_variable', smartconfig process running
                                     */

}e_StatusBits;

#define SET_STATUS_BIT(status_variable, bit)    status_variable |= ((unsigned long)1<<(bit))
#define CLR_STATUS_BIT(status_variable, bit)    status_variable &= ~((unsigned long)1<<(bit))
#define GET_STATUS_BIT(status_variable, bit)    (0 != (status_variable & ((unsigned long)1<<(bit))))

#define IS_CONNECTED(status_variable)             GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_CONNECTION)
#define IS_STA_CONNECTED(status_variable)         GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_STA_CONNECTED)
#define IS_IP_ACQUIRED(status_variable)           GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_IP_ACQUIRED)
#define IS_IP_LEASED(status_variable)             GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_IP_LEASED)
#define IS_CONNECTION_FAILED(status_variable)     GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_CONNECTION_FAILED)
#define IS_P2P_NEG_REQ_RECEIVED(status_variable)  GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_P2P_NEG_REQ_RECEIVED)
#define IS_SMARTCONFIG_DONE(status_variable)      GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_SMARTCONFIG_DONE)
#define IS_SMARTCONFIG_STOPPED(status_variable)   GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_SMARTCONFIG_STOPPED)


typedef enum provisioning_opt
{
	PROVISIONING_USING_AP_MODE,
	PROVISIONING_USING_SMART_CONFIG,
	PROVISIONING_USING_WPA,  /* Not supported */
	PROVISIONING_USING_WAC   /* Not supported */
}e_ProvisioningOpt;

// Network App specific status/error codes which are used only in this file
typedef enum{
     // Choosing this number to avoid overlap w/ host-driver's error codes
    DEVICE_NOT_IN_STATION_MODE = -0x7F0,
    DEVICE_NOT_IN_AP_MODE = DEVICE_NOT_IN_STATION_MODE - 1,
    DEVICE_NOT_IN_P2P_MODE = DEVICE_NOT_IN_AP_MODE - 1,

    STATUS_CODE_MAX = -0xBB8
}e_NetAppErroCodes;


/*!
    \brief Initialize the NWP and host driver and configure in STA mode

    \return     0 on success, negative error-code on error

    \note
*/
long Network_IF_InitDriver();

/*!
    \brief Deinitialize the NWP and host driver

    \return     0 on success, negative error-code on error

    \note
*/
long Network_IF_DeInitDriver();

/*!
    \brief Connect the deive to network using app based provisiong

    \return     0 on success, negative error-code on error

    \note
*/
long ConnectToNetwork();


#endif   //__NETWORK_IF_H__
