//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************


//*****************************************************************************
//
// Application Name     -   SSL Demo
// Application Overview -   This is a sample application demonstrating the
//                          use of secure sockets on a CC3200 device.The
//                          application connects to an AP and
//                          tries to establish a secure connection to the
//                          Google server.
// Application Details  -
// docs\examples\CC32xx_SSL_Demo_Application.pdf
// or
// http://processors.wiki.ti.com/index.php/CC32xx_SSL_Demo_Application
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup ssl
//! @{
//
//*****************************************************************************

#include <pin_mux_config.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "hw_nvic.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_apps_rcm.h"
#include "gpio.h"
#include "spi.h"
#include "uart.h"
#include "Adafruit_SSD1351.h"
#include "test.h"
#include "pin_mux_config.h"
#include "timer_if.h"
#include "timer.h"


//Common interface includes
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"
#include "i2c_if.h"

# define EXCITED 4
# define HAPPY 3
# define SAD 2
# define ANGRY 1
# define HUNGRY -1

int currentEmotion = HAPPY;
int prevEmotion = HAPPY;

#define SPI_IF_BIT_RATE  400000
#define TR_BUFF_SIZE     100

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif


extern void (* const g_pfnVectors[])(void);



#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1


#define APPLICATION_NAME        "SSL"
#define APPLICATION_VERSION     "1.1.1.EEC.Spring2018"
#define SERVER_NAME             "a23ovi9b4bb7vu-ats.iot.us-east-2.amazonaws.com"
#define GOOGLE_DST_PORT         8443

#define SL_SSL_CA_CERT "/cert/rootCA.der" //starfield class2 rootca (from firefox) // <-- this one works
#define SL_SSL_PRIVATE "/cert/private.der"
#define SL_SSL_CLIENT  "/cert/client.der"


//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                22    /* Current Date */
#define MONTH               2     /* Month 1-12 */
#define YEAR                2024  /* Current year */
#define HOUR                12    /* Time - hours */
#define MINUTE              43    /* Time - minutes */
#define SECOND              25     /* Time - seconds */

#define POSTHEADER "POST /things/mlu_CC3200_Board/shadow HTTP/1.1\r\n"
#define GETHEADER "GET /things/mlu_CC3200_Board/shadow HTTP/1.1\r\n"
#define HOSTHEADER "Host: a23ovi9b4bb7vu-ats.iot.us-east-2.amazonaws.com\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

#define MAX_BUFFER  80
#define SAMPLE_SIZE 410
#define LOWER_BOUND  20000
#define HIGHER_BOUND  40000

char DATA1[100] = "\0";
unsigned char ucDevAddr = 0x18;
unsigned char ucRegOffsetX = 0x3;
unsigned char aucRdDataBufX[256];
unsigned char ucRegOffsetY = 0x5;
unsigned char aucRdDataBufY[256];
int xPos = 64;
int yPos = 64;
int oldX = 64;
int oldY = 64;
int color;
volatile double activityScore;

long int power_all[8];
unsigned char flag;
unsigned char startProcess;

int sample_num;
unsigned short sample_count;
signed long sample_buffer[400];
int new_digit = 0;
int num;
int bottom_x, bottom_y = 61;
long int coeff_array[8]= {32743, 32667, 32541, 32365, 32138, 31863, 31538, 31164};
int frequencies[8] = {100, 200, 300, 400, 500, 600, 700, 800};


// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

typedef struct
{
   /* time */
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   /* date */
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day; //not required
   unsigned long tm_year_day; //not required
   unsigned long reserved[3];
}SlDateTime;


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulPingPacketsRecv = 0; //Number of Ping Packets received
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
signed char    *g_Host = SERVER_NAME;
SlDateTime g_time;
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End: df
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static long WlanConnect();
static int set_time();
static void BoardInit(void);
static long InitializeAppVariables();
static int tls_connect();
static int connectToAccessPoint();
static int http_post(int);
static int http_get(int);

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    if(!pWlanEvent) {
        return;
    }

    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT: {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'.
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                       g_ucConnectionSSID,g_ucConnectionBSSID[0],
                       g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                       g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                       g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT: {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                    "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default: {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    if(!pNetAppEvent) {
        return;
    }

    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT: {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
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

        default: {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse) {
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
    if(!pDevEvent) {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    if(!pSock) {
        return;
    }

    switch( pSock->Event ) {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status) {
                case SL_ECLOSE: 
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n", 
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default: 
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End breadcrumb: s18_df
//*****************************************************************************


//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    0 on success else error code
//!
//! \return None
//!
//*****************************************************************************
static long InitializeAppVariables() {
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    g_Host = SERVER_NAME;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    return SUCCESS;
}


//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState() {
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode 
    if (ROLE_STA != lMode) {
        if (ROLE_AP == lMode) {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while(!IS_IP_ACQUIRED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
            }
        }

        // Switch to STA role and restart 
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again 
        if (ROLE_STA != lRetVal) {
            // We don't want to proceed if the device is not coming up in STA-mode 
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }
    
    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);
    
    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig 
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, 
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore 
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal) {
        // Wait
        while(IS_CONNECTED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, 
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();
    
    return lRetVal; // Success
}


//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  None
//!
//! \return  0 on success else error code
//!
//! \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect() {
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    UART_PRINT("Attempting connection to access point: ");
    UART_PRINT(SSID_NAME);
    UART_PRINT("... ...");
    lRetVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT(" Connected!!!\n\r");


    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        // Toggle LEDs to Indicate Connection Progress
        _SlNonOsMainLoopTask();
        //GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
        _SlNonOsMainLoopTask();
        //GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
    }

    return SUCCESS;

}




long printErrConvenience(char * msg, long retVal) {
    UART_PRINT(msg);
    //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    return retVal;
}


//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! This function demonstrates how certificate can be used with SSL.
//! The procedure includes the following steps:
//! 1) connect to an open AP
//! 2) get the server name via a DNS request
//! 3) define all socket options and point to the CA certificate
//! 4) connect to the server via TCP
//!
//! \param None
//!
//! \return  0 on success else error code
//! \return  LED1 is turned solid in case of success
//!    LED2 is turned solid in case of failure
//!
//*****************************************************************************
static int tls_connect() {
    SlSockAddrIn_t    Addr;
    int    iAddrSize;
    unsigned char    ucMethod = SL_SO_SEC_METHOD_TLSV1_2;
    unsigned int uiIP;
//    unsigned int uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA;
    unsigned int uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256;
// SL_SEC_MASK_SSL_RSA_WITH_RC4_128_SHA
// SL_SEC_MASK_SSL_RSA_WITH_RC4_128_MD5
// SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_DHE_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_RC4_128_SHA
// SL_SEC_MASK_TLS_RSA_WITH_AES_128_CBC_SHA256
// SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA256
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256
// SL_SEC_MASK_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256 // does not work (-340, handshake fails)
    long lRetVal = -1;
    int iSockID;

    lRetVal = sl_NetAppDnsGetHostByName(g_Host, strlen((const char *)g_Host),
                                    (unsigned long*)&uiIP, SL_AF_INET);

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't retrieve the host name \n\r", lRetVal);
    }

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(GOOGLE_DST_PORT);
    Addr.sin_addr.s_addr = sl_Htonl(uiIP);
    iAddrSize = sizeof(SlSockAddrIn_t);
    //
    // opens a secure socket
    //
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
    if( iSockID < 0 ) {
        return printErrConvenience("Device unable to create secure socket \n\r", lRetVal);
    }

    //
    // configure the socket as TLS1.2
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECMETHOD, &ucMethod,\
                               sizeof(ucMethod));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
    //
    //configure the socket as ECDHE RSA WITH AES256 CBC SHA
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &uiCipher,\
                           sizeof(uiCipher));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }



/////////////////////////////////
// START: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
    //
    //configure the socket with CA certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                           SL_SO_SECURE_FILES_CA_FILE_NAME, \
                           SL_SSL_CA_CERT, \
                           strlen(SL_SSL_CA_CERT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
// END: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
/////////////////////////////////


    //configure the socket with Client Certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME, \
                                    SL_SSL_CLIENT, \
                           strlen(SL_SSL_CLIENT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //configure the socket with Private Key - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
            SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME, \
            SL_SSL_PRIVATE, \
                           strlen(SL_SSL_PRIVATE));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }


    /* connect to the peer device - Google server */
    lRetVal = sl_Connect(iSockID, ( SlSockAddr_t *)&Addr, iAddrSize);

    if(lRetVal >= 0) {
        UART_PRINT("Device has connected to the website:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }
    else if(lRetVal == SL_ESECSNOVERIFY) {
        UART_PRINT("Device has connected to the website (UNVERIFIED):");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }
    else if(lRetVal < 0) {
        UART_PRINT("Device couldn't connect to server:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
        return printErrConvenience("Device couldn't connect to server \n\r", lRetVal);
    }

    //GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    //GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    return iSockID;
}



int connectToAccessPoint() {
    long lRetVal = -1;
//    GPIO_IF_LedConfigure(LED1|LED3);
//
//    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
//    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    lRetVal = InitializeAppVariables();
    ASSERT_ON_ERROR(lRetVal);

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0) {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
          UART_PRINT("Failed to configure the device in its default state \n\r");

      return lRetVal;
    }

    UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    ///
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    UART_PRINT("Opening sl_start\n\r");
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal) {
        UART_PRINT("Failed to start the device \n\r");
        return lRetVal;
    }

    UART_PRINT("Device started as STATION \n\r");

    //
    //Connecting to WLAN AP
    //
    lRetVal = WlanConnect();
    if(lRetVal < 0) {
        UART_PRINT("Failed to establish connection w/ an AP \n\r");
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }

    UART_PRINT("Connection established w/ AP and IP is aquired \n\r");
    return 0;
}

static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(DATA1);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, DATA1);
    pcBufHeaders += strlen(DATA1);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

void parseJson(const char *json, const char *key) {
    const char *start = strstr(json, key);

    if (start == NULL) {
        fprintf(stderr, "Key not found: %s\n", key);
        return;
    }

    start += strlen(key) + 3; // Move to the value part

    const char *end = strchr(start, '"');
    if (end == NULL) {
        fprintf(stderr, "Value not found for key: %s\n", key);
        return;
    }

    // Calculate the length of the value
    size_t length = end - start;

    // Allocate memory for the value
    char *value = malloc(length + 1);
    if (value == NULL) {
        fprintf(stderr, "Memory allocation error\n");
        return;
    }

    // Copy the value to the allocated memory
    strncpy(value, start, length);
    value[length] = '\0'; // Null-terminate the string

    if (strcmp(value, "HAPPY") == 0) {
        currentEmotion = HAPPY;
        prevEmotion = HAPPY;
    } else if (strcmp(value, "EXCITED") == 0) {
        currentEmotion = EXCITED;
        prevEmotion = EXCITED;
    } else if (strcmp(value, "SAD") == 0) {
        currentEmotion = SAD;
        prevEmotion = SAD;
    } else if (strcmp(value, "ANGRY") == 0) {
        currentEmotion = ANGRY;
        prevEmotion = ANGRY;
    } else if (strcmp(value, "HUNGRY") == 0) {
        currentEmotion = HUNGRY;
        prevEmotion = HUNGRY;
    }
}


static int http_get(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    //char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, GETHEADER);
    pcBufHeaders += strlen(GETHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("GET failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
        parseJson(acRecvbuff, "emotion");
    }

    return 0;
}

void sendNewState(long lRetVal) {
    //Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();
    //Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }
    //Connect to the website with TLS encryption
    lRetVal = tls_connect();
    if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
    }
    http_post(lRetVal);

    sl_Stop(SL_STOP_TIMEOUT);
    //LOOP_FOREVER();
}

void getState(long lRetVal) {
    //Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();
    //Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }
    //Connect to the website with TLS encryption
    lRetVal = tls_connect();
    if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
    }
    http_get(lRetVal);

    sl_Stop(SL_STOP_TIMEOUT);
    //LOOP_FOREVER();
}

void sendNewEmotion (int lRetVal) {
    strcat(DATA1, "{\"state\": {\r\n\"desired\" : {\r\n\"emotion\" : \""); //concat the DATA1 to SMS format
    if (currentEmotion == HAPPY) {
        strcat(DATA1, "HAPPY");
    } else if (currentEmotion == EXCITED) {
        strcat(DATA1, "EXCITED");
    } else if (currentEmotion == HUNGRY) {
        strcat(DATA1, "HUNGRY");
    } else if (currentEmotion == SAD) {
        strcat(DATA1, "SAD");
    } else if (currentEmotion == ANGRY) {
        strcat(DATA1, "ANGRY");
    }
    strcat(DATA1, "\"\r\n}}}\r\n\r\n");
    sendNewState(lRetVal);
    memset(DATA1, 0, sizeof(DATA1));
}


long goertzel(long coeff)
{
    //initialize variables to be used in the function
    int Q, Q_prev, Q_prev2,i;
    long prod1,prod2,prod3,power;

    Q_prev = 0;         //set delay element1 Q_prev as zero
    Q_prev2 = 0;        //set delay element2 Q_prev2 as zero
    power=0;            //set power as zero

    for (i=0; i<400; i++) // loop 400 times and calculate Q, Q_prev, Q_prev2 at each iteration
        {

            Q = (sample_buffer[i]) + ((coeff* Q_prev)>>14) - (Q_prev2); // >>14 used as the coeff was used in Q15 format
            Q_prev2 = Q_prev;                                    // shuffle delay elements
            Q_prev = Q;
        }

    //calculate the three products used to calculate power
    prod1=((long) Q_prev*Q_prev);
    prod2=((long) Q_prev2*Q_prev2);
    prod3=((long) Q_prev *coeff)>>14;
    prod3=(prod3 * Q_prev2);

    power = ((prod1+prod2-prod3))>>8; //calculate power using the three products and scale the result down
    //printf("pwr: %ld", power);
    return power;
}


int post_test(void) // post_test() function from the Github example
{

    int max_power = 0; // Initialize max_power to the first power
    int max_power_index = 0; // Initialize max_power_index to the index of the first power

     // Find the index of the maximum power
    int i;
     for (i = 0; i < 8; i++) {
         if (power_all[i] > max_power) {
             max_power = power_all[i];
             max_power_index = i;
         }
     }

     // Determine the detected frequency based on the index of the maximum power
     int detected_frequency = frequencies[max_power_index];

     return detected_frequency;
}


unsigned short
readADC(void)
{
    unsigned char data[2];
    GPIOPinWrite(GPIOA0_BASE, 0x20, 0x00);
    MAP_SPITransfer(GSPI_BASE, 0, data, 0x2, SPI_CS_ENABLE|SPI_CS_DISABLE); // SPI
    GPIOPinWrite(GPIOA0_BASE, 0x20, 0x20);
    return (data[0] << 5) | ((0xf8 & data[1]) >> 3);
}

static void
TimerA0IntHandler(void)
{
    unsigned long ulStatus;
    ulStatus = MAP_TimerIntStatus(TIMERA0_BASE, true);
    MAP_TimerIntClear(TIMERA0_BASE, ulStatus);

    flag = 1;
    sample_num++;

    if (sample_num > 400)
        sample_num = 0;
        startProcess = 1;
}


void OledInit()
{
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE, MAP_PRCMPeripheralClockGet(PRCM_GSPI),
    SPI_IF_BIT_RATE,
                           SPI_MODE_MASTER, SPI_SUB_MODE_0, (SPI_SW_CTRL_CS |
                           SPI_4PIN_MODE |
                           SPI_TURBO_OFF |
                           SPI_CS_ACTIVEHIGH |
                           SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);
    Adafruit_Init();
}

drawSlime1(int x, int y, int color){
    drawPixel(x, y+5, color);
        drawPixel(x+1, y+5, color);
        drawPixel(x+2, y+5, color);
        drawPixel(x+3, y+4, color);
        drawPixel(x+4, y+4, color);
        drawPixel(x+5, y+3, color);
        drawPixel(x+6, y+2, color);
        drawPixel(x+6, y+1, color);
        drawPixel(x+7, y, color);
        drawPixel(x+7, y-1, color);
        drawPixel(x+7, y-2, color);
        drawPixel(x+7, y-3, color);
        drawPixel(x+6, y-4, color);

        drawPixel(x+5, y-5, color);
        drawPixel(x+4, y-5, color);
        drawPixel(x+3, y-5, color);
        drawPixel(x+2, y-5, color);
        drawPixel(x+1, y-5, color);
        drawPixel(x, y-5, color);

                drawPixel(x-1, y+5, color);
                drawPixel(x-2, y+5, color);
                drawPixel(x-3, y+4, color);
                drawPixel(x-4, y+4, color);
                drawPixel(x-5, y+3, color);
                drawPixel(x-6, y+2, color);
                drawPixel(x-6, y+1, color);
                drawPixel(x-7, y, color);
                drawPixel(x-7, y-1, color);
                drawPixel(x-7, y-2, color);
                drawPixel(x-7, y-3, color);
                drawPixel(x-6, y-4, color);

                drawPixel(x-5, y-5, color);
                drawPixel(x-4, y-5, color);
                drawPixel(x-3, y-5, color);
                drawPixel(x-2, y-5, color);
                drawPixel(x-1, y-5, color);

                drawPixel(x-3, y, color);
                drawPixel(x-3, y+1, color);
                drawPixel(x-3, y-1, color);

                drawPixel(x+3, y, color);
                drawPixel(x+3, y+1, color);
                drawPixel(x+3, y-1, color);

}

drawSlime2(int x, int y, int color){
    drawPixel(x, y+4, color);
        drawPixel(x+1, y+4, color);
        drawPixel(x+2, y+4, color);

        drawPixel(x+3, y+3, color);
        drawPixel(x+4, y+3, color);

        drawPixel(x+5, y+2, color);

        drawPixel(x+6, y+1, color);
        drawPixel(x+6, y, color);

        drawPixel(x+7, y-1, color);
        drawPixel(x+7, y-2, color);
        drawPixel(x+7, y-3, color);
        drawPixel(x+7, y-4, color);

        drawPixel(x+6, y-5, color);

        drawPixel(x+5, y-5, color);
        drawPixel(x+4, y-5, color);
        drawPixel(x+3, y-5, color);
        drawPixel(x+2, y-5, color);
        drawPixel(x+1, y-5, color);
        drawPixel(x, y-5, color);
////////////////////////////////////////////////////////
                drawPixel(x-1, y+4, color);
                drawPixel(x-2, y+4, color);

                drawPixel(x-3, y+3, color);
                drawPixel(x-4, y+3, color);

                drawPixel(x-5, y+2, color);

                drawPixel(x-6, y+1, color);
                drawPixel(x-6, y, color);

                drawPixel(x-7, y-1, color);
                drawPixel(x-7, y-2, color);
                drawPixel(x-7, y-3, color);
                drawPixel(x-7, y-4, color);

                drawPixel(x-6, y-5, color);

                drawPixel(x-5, y-5, color);
                drawPixel(x-4, y-5, color);
                drawPixel(x-3, y-5, color);
                drawPixel(x-2, y-5, color);
                drawPixel(x-1, y-5, color);

                drawPixel(x-3, y, color);
                                drawPixel(x-3, y-1, color);

                                drawPixel(x+3, y, color);
                                drawPixel(x+3, y-1, color);

}
drawSlime3(int x, int y, int color){
    drawPixel(x, y+3, color);
        drawPixel(x+1, y+3, color);
        drawPixel(x+2, y+3, color);

        drawPixel(x+3, y+2, color);
        drawPixel(x+4, y+2, color);

        drawPixel(x+5, y+1, color);

        drawPixel(x+6, y+0, color);
        drawPixel(x+6, y-1, color);

        drawPixel(x+7, y-2, color);
        drawPixel(x+7, y-3, color);
        drawPixel(x+7, y-4, color);
        drawPixel(x+7, y-5, color);

        drawPixel(x+6, y-5, color);

        drawPixel(x+5, y-5, color);
        drawPixel(x+4, y-5, color);
        drawPixel(x+3, y-5, color);
        drawPixel(x+2, y-5, color);
        drawPixel(x+1, y-5, color);
        drawPixel(x, y-5, color);
////////////////////////////////////////////////////////
                drawPixel(x-1, y+3, color);
                drawPixel(x-2, y+3, color);

                drawPixel(x-3, y+2, color);
                drawPixel(x-4, y+2, color);

                drawPixel(x-5, y+1, color);

                drawPixel(x-6, y, color);
                drawPixel(x-6, y-1, color);

                drawPixel(x-7, y-2, color);
                drawPixel(x-7, y-3, color);
                drawPixel(x-7, y-4, color);
                drawPixel(x-7, y-5, color);

                drawPixel(x-6, y-5, color);

                drawPixel(x-5, y-5, color);
                drawPixel(x-4, y-5, color);
                drawPixel(x-3, y-5, color);
                drawPixel(x-2, y-5, color);
                drawPixel(x-1, y-5, color);

                drawPixel(x-3, y, color);
                drawPixel(x+3, y, color);
}


static void BoardInit(void) {
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

void updateEmotion() {
    if (activityScore >= 3.0 && activityScore < 4.0) {
        currentEmotion = HAPPY;
    } else if (activityScore >= 4.0 && activityScore < 5.0) {
        currentEmotion = EXCITED;
    } else if (activityScore >= 2.0 && activityScore < 3.0) {
        currentEmotion = SAD;
    } else if (activityScore >= 1 && activityScore < 2.0) {
        currentEmotion = ANGRY;
    }
    //activityScore += 0.5;
}

void updateColor() {
    if (currentEmotion == HAPPY) {
        color = WHITE;
    } else if (currentEmotion == EXCITED) {
        color = GREEN;
    } else if (currentEmotion == SAD) {
        color = BLUE;
    } else if (currentEmotion == ANGRY) {
        color = RED;
    } else if (currentEmotion == HUNGRY) {
        color = YELLOW;
    }
}

void updateScore(bool positive) {
    if (positive && activityScore < 8.0) {
        activityScore += 0.1;
    } else if (!positive && activityScore > 1.0) {
        activityScore -= 0.2;
    }
    if (activityScore > 8.0) {
        activityScore = 7.0;
    }
    updateEmotion();
}


void updateScoreSound(bool positive) {
    if (positive && activityScore < 8.0) {
        activityScore += 1.3;
    } else if (!positive && activityScore > 1.0) {
        activityScore -= 0.8;
    }
    if (activityScore > 8.0) {
        activityScore = 7.0;
    }
    if (activityScore < 1.0) {
            activityScore = 1.5;
        }
    updateEmotion();


}

//void updateScore(bool positive) {
//    if (positive && activityScore < 5.0) {
//        activityScore += 0.2;
//    } else if (!positive && activityScore > 1.0) {
//        activityScore -= 0.1;
//    }
//    if (activityScore > 5.0) {
//        activityScore = 4.0;
//    }
//    updateEmotion();
//}
//
//
//void updateScoreSound(bool positive) {
//    if (positive && activityScore < 5.0) {
//        activityScore += 0.5;
//    } else if (!positive && activityScore > 1.0) {
//        activityScore -= 0.2;
//    }
//    if (activityScore > 5.0) {
//        activityScore = 4.0;
//    }
//    if (activityScore < 1.0) {
//            activityScore = 1.5;
//        }
//    updateEmotion();
//
//
//}


void slimeAnimation() {
     int i = 0;
     int j = 0;
     int k = 0;

     for(i; i < 4; i++){
         drawSlime1(xPos, yPos, color);
     }
     drawSlime1(xPos, yPos, BLACK);

     for(j; j < 4; j++){
         drawSlime2(xPos, yPos, color);
     }
     drawSlime2(xPos, yPos, BLACK);

     for(k; k < 4; k++){
         drawSlime3(xPos, yPos, color);
     }
     drawSlime3(xPos, yPos, BLACK);
}

void updatePosition() {
    I2C_IF_Write(ucDevAddr,&ucRegOffsetX,1,0);
    I2C_IF_Read(ucDevAddr, &aucRdDataBufX[0], 1);

    I2C_IF_Write(ucDevAddr,&ucRegOffsetY,1,0);
    I2C_IF_Read(ucDevAddr, &aucRdDataBufY[0], 1);

    oldX = xPos;
    oldY = yPos;
    int magX =  -(signed char)aucRdDataBufX[0];
    int magY = (signed char)aucRdDataBufY[0];

    int xScore = abs(magX);
    int yScore = abs(magY);

    //UART_PRINT("X = %d \t Y = %d\n\r", xScore, yScore);
    if (xScore > 20 || yScore > 20) {
        updateScore(true);
    } else {
        updateScore(false);
    }

    xPos += magX;
    yPos += magY;


    if (xPos > 120) {
        xPos = 120;
    } else if (xPos < 7) {
        xPos = 7;
    }
    if (yPos > 121) {
        yPos = 121;
    } else if (yPos < 6) {
        yPos = 6;
    }
}

void cornerColorChange() {
    int upper = 7;
    int lower = 1;

    if(xPos > 119 && yPos > 119){
    int num = (rand() % (upper - lower + 1)) + lower;
    switch(num){
    case(1):
        color = WHITE;
        break;
    case(2):
        color = CYAN;
        break;
    case(3):
        color = GREEN;
        break;
    case(4):
        color = MAGENTA;
        break;
    case(5):
        color = YELLOW;
        break;
    case(6):
        color = RED;
        break;
    case(7):
        color = BLUE;
        break;
    }

   }
}



//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void main() {
    unsigned long ulStatus;
    long lRetVal = -1;
    int new = -1;
    char old = -1;
    int oldoldoldnew = 0;
    int oldoldnew = 0;
    int oldnew = 0;
    int i;
    flag = 0;
    startProcess = 0;
    sample_num = 0;
    //
    // Initialize board configuration
    //
    BoardInit();

    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);


    InitTerm();
    ClearTerm();

    MAP_PRCMPeripheralReset(PRCM_GSPI);
    OledInit();

    // set up Timer interrupt

    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
      MAP_PRCMPeripheralReset(PRCM_TIMERA0);
      MAP_TimerConfigure(TIMERA0_BASE, TIMER_CFG_PERIODIC);
      MAP_TimerLoadSet(TIMERA0_BASE, TIMER_A, 5000);
      MAP_TimerIntRegister(TIMERA0_BASE, TIMER_A, TimerA0IntHandler);
      ulStatus = MAP_TimerIntStatus(TIMERA0_BASE, false);
      MAP_TimerIntClear(TIMERA0_BASE, ulStatus);

      MAP_TimerIntEnable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
         MAP_TimerEnable(TIMERA0_BASE, TIMER_A);


    //
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    fillScreen(BLACK);

    startingUpText();
    //getState(lRetVal);
    //activityScore = (double)currentEmotion;
    activityScore = (double)HAPPY;
    updateColor();

    fillScreen(BLACK);
    sample_num = 0;
    while (1) {
        slimeAnimation();
        updatePosition();

        if (flag == 1) {
            long tmp = ((signed long) readADC()) - 372;
            //printf("index: %d\n", sample_num);

            sample_buffer[sample_num-1] = tmp;

            flag = 0;

        }
        if (startProcess == 1) {
                   startProcess = 0;
                   // disable timer
                   MAP_TimerIntDisable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
                   MAP_TimerDisable(TIMERA0_BASE, TIMER_A);
                   ulStatus = MAP_TimerIntStatus(TIMERA0_BASE, false);
                   MAP_TimerIntClear(TIMERA0_BASE, ulStatus);

                   for (i = 7; i >= 0; i--)
                       power_all[i] = goertzel(coeff_array[i]); // call goertzel


                   oldoldoldnew = oldoldnew;
                   oldoldnew = oldnew;
                   oldnew = new;
                   new = post_test();
                   if(new != oldnew && oldoldnew != oldnew && oldoldoldnew != oldoldnew){
                          //printf("new %d\n", new);
                          if (new >= 400) {
                             updateScoreSound(true);
                         } else {
                             updateScoreSound(false);
                         }

                   }


                   MAP_TimerLoadSet(TIMERA0_BASE, TIMER_A, 5000);
                   MAP_TimerIntEnable(TIMERA0_BASE, TIMER_A);
                   MAP_TimerEnable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
        }

//        printf("score: %f\n", activityScore);
//        printf("emotion %d\n", currentEmotion);

        if (currentEmotion != prevEmotion) {
            //updatingText();
            //sendNewEmotion(lRetVal);
            updateColor();
            prevEmotion = currentEmotion;
            //fillScreen(BLACK);
        }
    }

}

