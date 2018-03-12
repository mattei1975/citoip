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

#include "provisioning.h"
#include "app_common.h"
#include "rtsp_main_task.h"
#include "cc_pal_app.h"
#include "timer_if.h"

#ifndef ENABLE_LEGECY_PROVISIONING
#include "provisioning_api.h"
#include "provisioning_defs.h"
#endif

#define PROVISIONING_TIMEOUT 300

#define SL_PARAM_PRODUCT_VERSION_DATA "R1.0"


#define SET_STATUS_BIT(status_variable, bit)    status_variable |= ((unsigned long)1<<(bit))
#define CLR_STATUS_BIT(status_variable, bit)    status_variable &= ~((unsigned long)1<<(bit))
#define GET_STATUS_BIT(status_variable, bit)    (0 != (status_variable & ((unsigned long)1<<(bit))))

/* Use bit 32: Lower bits of status variable are used for NWP events
 *
 *      1 in a 'status_variable', profile is added form web page
 *      0 in a 'status_variable', profile is not added form web page
 */
#define STATUS_BIT_PROFILE_ADDED 31
/* Use bit 31
 *      1 in a 'status_variable', successfully connected to AP
 *      0 in a 'status_variable', AP connection was not successful
 */
#define STATUS_BIT_CONNECTED_TO_CONF_AP (STATUS_BIT_PROFILE_ADDED - 1)
/* Use bit 30
 *      1 in a 'status_variable', AP provisioning process completed
 *      0 in a 'status_variable', AP provisioning process yet to complete
 */
#define STATUS_BIT_PROVISIONING_DONE (STATUS_BIT_CONNECTED_TO_CONF_AP - 1)

#define IS_PROFILE_ADDED(status_variable)          GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_PROFILE_ADDED)
#define IS_CONNECTED_TO_CONF_AP(status_variable)          GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_CONNECTED_TO_CONF_AP)
#define IS_PROVISIONING_DONE(status_variable)          GET_STATUS_BIT(status_variable, \
                                                               STATUS_BIT_PROVISIONING_DONE)

/*
 * GLOBAL VARIABLES -- Start
 */
unsigned long    g_ulStatus = 0;
unsigned char isProvisioning = 0;
unsigned char volatile g_TimerATimedOut;

extern OsiMsgQ_t app_rtsp_task_q;

/*
 * GLOBAL VARIABLES -- End
 */

/*!
    \brief This function handles WLAN events

    \param[in]      pWlanEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pSlWlanEvent)
{
    if(pSlWlanEvent == NULL)
    {
        return;
    }

     switch(((SlWlanEvent_t*)pSlWlanEvent)->Event)
     {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION_FAILED);

            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'slWlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * slWlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &pSlWlanEvent->EventData.STAandP2PModeWlanConnected;
             *
             */

        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            app_rtsp_task_msg_s     rtspMsg;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_ACQUIRED);

            if(isProvisioning == 0)
            {
                rtspMsg.task_id = ASYNC_EVENT_HANDLER;
                rtspMsg.msg_id = RTSP_DISCONNECT_FROM_AP;

                osi_MsgQWrite(&app_rtsp_task_q,&rtspMsg,OSI_NO_WAIT);
            }

            pEventData = &pSlWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request,
             * 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
            }
            else
            {
            }
        }
        break;

        case SL_WLAN_STA_CONNECTED_EVENT:
        {
            /* when device is in AP mode and any client connects to device cc3xxx */
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION_FAILED);

            /*
             * Information about the connected client (like SSID, MAC etc) will
             * be available in 'slPeerInfoAsyncResponse_t' - Applications
             * can use it if required
             *
             * slPeerInfoAsyncResponse_t *pEventData = NULL;
             * pEventData = &pSlWlanEvent->EventData.APModeStaConnected;
             */

        }
        break;

        case SL_WLAN_STA_DISCONNECTED_EVENT:
        {
            /* when client disconnects from device (AP) */
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            /*
             * Information about the connected client (like SSID, MAC etc) will
             * be available in 'slPeerInfoAsyncResponse_t' - Applications
             * can use it if required
             *
             * slPeerInfoAsyncResponse_t *pEventData = NULL;
             * pEventData = &pSlWlanEvent->EventData.APModestaDisconnected;
             */
        }
        break;

        case SL_WLAN_SMART_CONFIG_COMPLETE_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_SMARTCONFIG_DONE);

            /*
             * Information about the SmartConfig details (like Status, SSID,
             * Token etc) will be available in 'slSmartConfigStartAsyncResponse_t'
             * - Applications can use it if required
             *
             * slSmartConfigStartAsyncResponse_t *pEventData = NULL;
             * pEventData = &pSlWlanEvent->EventData.smartConfigStartResponse;
             */
        }
        break;

        case SL_WLAN_SMART_CONFIG_STOP_EVENT:
        {
            /* SmartConfig operation finished */
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_SMARTCONFIG_DONE);

            /*
             * Information about the SmartConfig details (like Status, padding
             * etc) will be available in 'slSmartConfigStopAsyncResponse_t' -
             * Applications can use it if required
             *
             * slSmartConfigStopAsyncResponse_t *pEventData = NULL;
             * pEventData = &pSlWlanEvent->EventData.smartConfigStopResponse;
             */
        }
        break;

        case SL_WLAN_CONNECTION_FAILED_EVENT:
        {
            /* If device gets any connection failed event */
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION_FAILED);
        }
        break;

        default:
        {
            Report("[WLAN EVENT] Unexpected event \n\r");
        }
        break;
     }

}

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication

    \param[in]      pNetAppEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
    {
        return;
    }

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        case SL_NETAPP_IPV6_IPACQUIRED_EVENT:
        {
            app_rtsp_task_msg_s     rtspMsg;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_ACQUIRED);
            Report("[NETAPP EVENT] IP acquired by the device\n\r");


            if(isProvisioning == 0)
            {
                rtspMsg.task_id = ASYNC_EVENT_HANDLER;
                rtspMsg.msg_id = RTSP_CONNECTED_TO_NW;

                osi_MsgQWrite(&app_rtsp_task_q, &rtspMsg, OSI_NO_WAIT);
            }

            Report("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
            "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));

        }
        break;

        case SL_NETAPP_IP_LEASED_EVENT:
        {
            unsigned long  g_ulStaIp = 0;
            SlIpLeasedAsync_t *pEventData = NULL;
            pEventData = &pNetAppEvent->EventData.ipLeased;
            g_ulStaIp = pEventData->ip_address;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);
            Report("[NETAPP EVENT] IP Leased to Client: IP=%d.%d.%d.%d \n\r",
                                                SL_IPV4_BYTE(g_ulStaIp,3), SL_IPV4_BYTE(g_ulStaIp,2),
                                                SL_IPV4_BYTE(g_ulStaIp,1), SL_IPV4_BYTE(g_ulStaIp,0));
        }
        break;

        case SL_NETAPP_IP_RELEASED_EVENT:
        {
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);
            Report("[NETAPP EVENT] IP Released for Client\n\r");
        }
        break;

        default:
        {
            Report("[NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if(pDevEvent == NULL)
    {
        return;
    }

    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */
    Report("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\r",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}

/*!
    \brief This function handles socket events indication

    \param[in]      pSock is the event passed to the handler

    \return         None
*/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
    {
        return;
    }

    /*
     * This application doesn't work w/ socket - Events are not expected
     */
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status)
            {
                case SL_ECLOSE:
                    Report("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\r",
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default:
                    Report("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\r",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        case SL_SOCKET_ASYNC_EVENT:

             switch(pSock->socketAsyncEvent.SockAsyncData.type)
             {
             case SSL_ACCEPT:/*accept failed due to ssl issue ( tcp pass)*/
                 Report("[SOCK ERROR] - close socket (%d) operation"
                             "accept failed due to ssl issue\n\r",
                             pSock->socketAsyncEvent.SockAsyncData.sd);
             case RX_FRAGMENTATION_TOO_BIG:
                 Report("[SOCK ERROR] -close scoket (%d) operation"
                             "connection less mode, rx packet fragmentation\n\r"
                             "> 16K, packet is being released",
                             pSock->socketAsyncEvent.SockAsyncData.sd);
             case OTHER_SIDE_CLOSE_SSL_DATA_NOT_ENCRYPTED:
                 Report("[SOCK ERROR] -close socket (%d) operation"
                             "remote side down from secure to unsecure\n\r",
                            pSock->socketAsyncEvent.SockAsyncData.sd);
             default:
                 Report("Unknown sock async event: %d\n\r",
                             pSock->socketAsyncEvent.SockAsyncData.type);
             }
            break;
        default:
            Report("[SOCK EVENT] - Unexpected Event [%x0x]\n\r",pSock->Event);
          break;
    }
}

/*!
    \brief This function handles callback for the HTTP server events

    \param[in]      pEvent - Contains the relevant event information
    \param[in]      pResponse - Should be filled by the user with the
                    relevant response information

    \return         None

    \note

    \warning
*/
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pEvent,
                                  SlHttpServerResponse_t *pResponse)
{
    if(pEvent == NULL || pResponse == NULL)
    {
        Report(" [HTTP EVENT] NULL Pointer Error \n\r");
        return;
    }
    //Report("[HTTP Event]\r\n");
}


void printNWPVer()
{
    SlVersionFull   ver = {0};
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;

    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
                                &ucConfigLen, (unsigned char *)(&ver));

    Report("Host Driver version: %s\n\r",SL_DRIVER_VERSION);
    Report("Build version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);
}

long switchRole(unsigned long mode)
{
    long lRetVal = -1;

    lRetVal = sl_WlanSetMode(mode);
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(0xFF);

    lRetVal = sl_Start(0, 0, 0);

    return lRetVal;
}


long
Network_IF_InitDriver()
{
    long lRetVal = -1;

    lRetVal = sl_Start(NULL,NULL,NULL);

    if (lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        return lRetVal;
    }

    if(lRetVal != ROLE_STA)
        lRetVal = switchRole(ROLE_STA);

    if(lRetVal != ROLE_STA)
    {
        Report("Failed to start in Startion\r\n");
        return DEVICE_NOT_IN_STATION_MODE;
    }

    /* Print the SP version */
    printNWPVer();

    return 0;
}

long Network_IF_DeInitDriver()
{
    long lRetVal = -1;

    lRetVal = sl_Stop(0xFF);

    return lRetVal;
}

long ConnectToNetwork()
{
    long                retVal = -1;
    unsigned short      counter = 0;

#ifndef ENABLE_LEGECY_PROVISIONING
    slExtLibProvCfg_t   cfg;
    SlFsFileInfo_t FsFileInfo;
    _i32 FileHandle = 0;

    /* Initializes configuration */
    cfg.IsBlocking         = 1;    /* Unused */
    cfg.AutoStartEnabled   = 0;
    cfg.Timeout10Secs      = PROVISIONING_TIMEOUT/10;
    cfg.ModeAfterFailure   = ROLE_STA;
    cfg.ModeAfterTimeout   = ROLE_STA;

#endif /* ENABLE_LEGECY_PROVISIONING */

    isProvisioning = 1;

    /* Wait for device to connect to network or timeout */
    while((!IS_IP_ACQUIRED(g_ulStatus))  && counter < 100)
    {
        cc_gpio_config_high(GPIO_LED);
        osi_Sleep(50);
        cc_gpio_config_low(GPIO_LED);
        osi_Sleep(50);
        counter++;
    }

    if(!IS_CONNECTED(g_ulStatus))
    {
        /* Delete any existing profile */
        sl_WlanProfileDel(0xFF);

        cc_gpio_config_high(GPIO_LED);

#ifndef ENABLE_LEGECY_PROVISIONING
        while(retVal < 0)
        {
            /* Creating the param_product_version.txt file once */
            if (SL_FS_ERR_FILE_NOT_EXISTS == sl_FsGetInfo(SL_FILE_PARAM_PRODUCT_VERSION, 0 , &FsFileInfo))
            {
                sl_FsOpen(SL_FILE_PARAM_PRODUCT_VERSION, FS_MODE_OPEN_CREATE(100, _FS_FILE_OPEN_FLAG_COMMIT), NULL, &FileHandle);
                sl_FsWrite(FileHandle, NULL, SL_PARAM_PRODUCT_VERSION_DATA, strlen(SL_PARAM_PRODUCT_VERSION_DATA));
                sl_FsClose(FileHandle, NULL, NULL, NULL);
            }

            /* Creating the config result file once */
            if (SL_FS_ERR_FILE_NOT_EXISTS == sl_FsGetInfo(SL_FILE_PARAM_CFG_RESULT, 0 , &FsFileInfo))
            {
                sl_FsOpen(SL_FILE_PARAM_CFG_RESULT, FS_MODE_OPEN_CREATE(100, _FS_FILE_OPEN_FLAG_COMMIT), NULL, &FileHandle);
                sl_FsWrite(FileHandle, NULL, GET_CFG_RESULT_TOKEN, strlen(GET_CFG_RESULT_TOKEN));
                sl_FsClose(FileHandle, NULL, NULL, NULL);
            }

            /* Creating the param device name file once */
            if (SL_FS_ERR_FILE_NOT_EXISTS == sl_FsGetInfo(SL_FILE_PARAM_DEVICE_NAME, 0 , &FsFileInfo))
            {

                sl_FsOpen(SL_FILE_PARAM_DEVICE_NAME, FS_MODE_OPEN_CREATE(100, _FS_FILE_OPEN_FLAG_COMMIT), NULL, &FileHandle);
                sl_FsWrite(FileHandle, NULL, GET_DEVICE_NAME_TOKEN, strlen(GET_DEVICE_NAME_TOKEN));
                sl_FsClose(FileHandle, NULL, NULL, NULL);
            }

            /* Creating the netlist name file once */
            if (SL_FS_ERR_FILE_NOT_EXISTS == sl_FsGetInfo(SL_FILE_NETLIST, 0 , &FsFileInfo))
            {
                sl_FsOpen(SL_FILE_NETLIST, FS_MODE_OPEN_CREATE(100, _FS_FILE_OPEN_FLAG_COMMIT), NULL, &FileHandle);
                sl_FsWrite(FileHandle, NULL, SL_SET_NETLIST_TOKENS, strlen(GET_NETWORKS_TOKEN_PREFIX));
                sl_FsClose(FileHandle, NULL, NULL, NULL);
            }


            /* Starts Provisioning Process (Blocking) */
            retVal = sl_extlib_ProvisioningStart(ROLE_STA, &cfg);

            Report("Provisioning Finished\n\r");

            if (retVal < 0)
            {
                /* Signal end of provisioning */
                Report("Provisioning ended with error! RetVal=%d", retVal);
            }
        }
#else /* ENABLE_LEGECY_PROVISIONING */
        retVal = sl_WlanSmartConfigStart(0,                /*groupIdBitmask*/
                               SMART_CONFIG_CIPHER_NONE,    /*cipher*/
                               0,                           /*publicKeyLen*/
                               0,                           /*group1KeyLen*/
                               0,                           /*group2KeyLen */
                               NULL,                        /*publicKey */
                               NULL,                        /*group1Key */
                               NULL);                       /*group2Key*/
        if (retVal < 0)
        {
            /* Signal end of provisioning */
            Report("Provisioning ended with error! RetVal=%d", retVal);
        }
#endif /* ENABLE_LEGECY_PROVISIONING */

        /* Wait for device to connect to network */
        while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
        {
            osi_Sleep(50);
#ifndef SL_PLATFORM_MULTI_THREADED
            _SlNonOsMainLoopTask();
#endif
        }

        Report("Provisioning ended. RetVal=%d", retVal);
    }

    cc_gpio_config_low(GPIO_LED);

    isProvisioning = 0;

    return 0;
}

#ifndef ENABLE_LEGECY_PROVISIONING
void timeoutHandler(void)
{
    Timer_IF_InterruptClear(TIMERA1_BASE);
    g_TimerATimedOut++;
}

/*****************************************************************************/
/*                   Provisioning Callbacks                                  */
/*****************************************************************************/
_i8 sl_extlib_ProvEventTimeoutHdl(_u8* event, _i32 timeout)
{
    if(timeout == SL_EXT_PROV_WAIT_FOREVER)
    {
        /* Waiting forever, no timeout */
        while(*event == FALSE)
        {
            /* waiting... */
#ifndef SL_PLATFORM_MULTI_THREADED
            _SlNonOsMainLoopTask();
#endif
        }
    }
    else
    {
        /*
         * On CC3200, a value greater than 53687 will overflow the buffer,
         * therefore divide the timeout into smaller pieces.
         */
        int divider = 10;

        /* Initializes & Starts timer */
        Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
        Timer_IF_IntSetup(TIMERA1_BASE, TIMER_A, timeoutHandler);

        g_TimerATimedOut = 0;

        Timer_IF_Start(TIMERA1_BASE, TIMER_A, timeout/divider);

        /* Check event or wait until timeout */
        while(*event == FALSE && g_TimerATimedOut != divider)
        {
            /* waiting... */
#ifndef SL_PLATFORM_MULTI_THREADED
            _SlNonOsMainLoopTask();
#endif
        }

        /* Stops timer */
        Timer_IF_Stop(TIMERA1_BASE, PRCM_TIMERA1);
        Timer_IF_DeInit(TIMERA1_BASE, PRCM_TIMERA1);

        if(g_TimerATimedOut == divider)
        {
            return -1;
        }
    }

    return 0;
}

void sl_extlib_ProvWaitHdl(_i32 timeout)
{
    osi_Sleep(timeout);
}
#endif /* ENABLE_LEGECY_PROVISIONING */
