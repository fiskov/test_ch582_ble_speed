/********************************** (C) COPYRIGHT *******************************
 * File Name          : peripheral.C
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/10
 * Description        : Peripheral slave multi-connection application, initialize 
 *                      broadcast connection parameters, then broadcast, after 
 *                      connecting to the host, request to update connection parameters, 
 *                      and transmit data through custom services
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"
#include "devinfoservice.h"
#include "gattprofile.h"
#include "peripheral.h"
#include "LED.h"
#include <stdio.h>
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define BLINK_PERIOD_MS                 1000
#define BLINK_DUR_MS                    3   

#define SBP_PERIODIC_EVT_PERIOD              (100 * 8 / 5)

// How often to perform read rssi event
#define SBP_READ_RSSI_EVT_PERIOD             (5*1000*8/5)

// Parameter update delay
#define SBP_PARAM_UPDATE_DELAY               6400

// PHY update delay
#define SBP_PHY_UPDATE_DELAY                 2400

// What is the advertising interval when device is discoverable (units of 625us, 80=50ms)
#define DEFAULT_ADVERTISING_INTERVAL         (500 * 8 / 5)

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE            GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 6=7.5ms)
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL    12

// Maximum connection interval (units of 1.25ms, 100=125ms)
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL    100

// Slave latency to use parameter update
#define DEFAULT_DESIRED_SLAVE_LATENCY        0

// Supervision timeout value (units of 10ms, 100=1s)
#define DEFAULT_DESIRED_CONN_TIMEOUT         100

// Company Identifier: WCH
#define WCH_COMPANY_ID                       0x07D7

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8_t Peripheral_TaskID = INVALID_TASK_ID; // Task ID for internal task/event processing

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] = {
    // complete name
    0x05, // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'T',
    'e',
    's',
    't',

    // connection interval range
    0x05, // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL), // 100ms
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL), // 1s
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

    // Tx power level
    0x02, // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0 // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
static uint8_t advertData[] = {
    // Flags; this sets the device to use limited discoverable
    // mode (advertises for 30 seconds at a time) instead of general
    // discoverable mode (advertises indefinitely)
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // service UUID, to notify central devices what services are included
    // in this peripheral
    0x03,                  // length of this data
    GAP_ADTYPE_16BIT_MORE, // some of the UUID's, but not all
    LO_UINT16(SIMPLEPROFILE_SERV_UUID),
    HI_UINT16(SIMPLEPROFILE_SERV_UUID)
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Test Peripheral";

// Connection item list
static peripheralConnItem_t peripheralConnList;

static uint8_t peripheralMTU = ATT_MTU_SIZE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Peripheral_ProcessTMOSMsg(tmos_event_hdr_t *pMsg);
static void peripheralStateNotificationCB(gapRole_States_t newState, gapRoleEvent_t *pEvent);
static void simpleProfileChangeCB(uint8_t paramID, uint8_t *pValue, uint16_t len);
static void peripheralParamUpdateCB(uint16_t connHandle, uint16_t connInterval,
                                    uint16_t connSlaveLatency, uint16_t connTimeout);
static void peripheralInitConnItem(peripheralConnItem_t *peripheralConnList);
static void peripheralRssiCB(uint16_t connHandle, int8_t rssi);
static int peripheralChar4Notify(uint8_t *pValue, uint16_t len);
static void peripheralConnectEventCB( uint32_t timeUs );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t Peripheral_PeripheralCBs = {
    peripheralStateNotificationCB, // Profile State Change Callbacks
    peripheralRssiCB,              // When a valid RSSI is read from controller (not used by application)
    peripheralParamUpdateCB
};

// Broadcast Callbacks
static gapRolesBroadcasterCBs_t Broadcaster_BroadcasterCBs = {
    NULL, // Not used in peripheral role
    NULL  // Receive scan request callback
};

// GAP Bond Manager Callbacks
static gapBondCBs_t Peripheral_BondMgrCBs = {
    NULL, // Passcode callback (not used by application)
    NULL, // Pairing / Bonding state Callback (not used by application)
    NULL  // oob callback
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t Peripheral_SimpleProfileCBs = {
    simpleProfileChangeCB // Characteristic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

static int tx_total_size = (1024*1024), tx_total_pos, tx_pckt_cnt, tx_pckt_size, tx_rate, pckt_max;
static int tx_img_tmr_cnt = 0, tx_cnt[8], er[8];
static unsigned spd = 0, er_cnt = 0, mem_min;

static int tx_img_start(void)
{
    mem_min = 0xFFFF;
    tx_img_tmr_cnt = 0;
    tmos_start_task(Peripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD);
    tx_pckt_cnt = 0;
    pckt_max = 0;

    memset(tx_cnt, 0, sizeof(tx_cnt));
    memset(er, 0, sizeof(er));

    tx_total_pos = tx_total_size;

    HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);

    return 0;
}
static int tx_img_get(uint8_t *bfr, int len)
{
    static uint8_t c;
    tmos_memset(bfr, c++, len);
    bfr[0] = (tx_pckt_cnt >> 8) & 0xFF;
    bfr[1] = tx_pckt_cnt & 0xFF;

    bfr[2] = (tx_total_pos >> 16) & 0xFF;
    bfr[3] = (tx_total_pos >> 8) & 0xFF;
    bfr[4] = tx_total_pos & 0xFF;

    return MIN(len, tx_total_pos);
}
static int tx_img_next(int len)
{
    tx_cnt[2] += 1;

    if (tx_total_pos > 0)
        len = MIN(len, tx_total_pos);

    if (len)
    {
        tx_total_pos -= len;
        tx_pckt_cnt += 1;
    }

    return len;
}
static int tx_img_finish(void)
{
    PFIC_DisableIRQ(TMR2_IRQn);

    HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
    HalLedBlink(HAL_LED_1, 0, BLINK_DUR_MS, BLINK_PERIOD_MS);

    tmos_stop_task(Peripheral_TaskID, SBP_PERIODIC_EVT);


    if (tx_img_tmr_cnt == 0)
        tx_img_tmr_cnt = 1;

    spd = tx_total_size * 8 / tx_img_tmr_cnt / 100; // kbit/s

    PRINT("tmr=%d.%03ds, tmr_work=%dms ",
            tx_img_tmr_cnt/10, tx_img_tmr_cnt % 10
    );

    PRINT("pcktsize=%d, cnt=%d, pckt_max=%d; spd=%d kbit/s\n",
            tx_pckt_size,
            tx_pckt_cnt,
            pckt_max,
            spd
    );

    for (unsigned i=0; i<ARRAY_SIZE(tx_cnt); i++)
        PRINT("%d\t", tx_cnt[i]);
    PRINT("\n");

    for (unsigned i=0; i<ARRAY_SIZE(er); i++)
    {
        PRINT("%d\t", er[i]);
        er_cnt += er[i];
    }
    PRINT("\n");

    return 0;
}
static int tx_remain_bytes(void)
{
    return tx_total_pos;
}

// print free heap memory of OS
void print_mem(void)
{
    PRINT("mem=%d\n", (int)tmos_memory_getlen());
}


/*********************************************************************
 * @fn      Peripheral_Init
 *
 * @brief   Initialization function for the Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by TMOS.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void Peripheral_Init()
{
    Peripheral_TaskID = TMOS_ProcessEventRegister(Peripheral_ProcessEvent);

    // Setup the GAP Peripheral Role Profile
    {
        uint8_t  initial_advertising_enable = TRUE;
        uint16_t desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16_t desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

        // Set the GAP Role Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_advertising_enable);
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
        GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t), &desired_min_interval);
        GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t), &desired_max_interval);
    }

    {
        uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

        // Set advertising interval
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, advInt);

        // Enable scan req notify
        //GAP_SetParamValue(TGAP_ADV_SCAN_REQ_NOTIFY, ENABLE);
    }

    // Setup the GAP Bond Manager
    {
        uint32_t passkey = 0; // passkey "000000"
        uint8_t  pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        uint8_t  mitm = TRUE;
        uint8_t  bonding = TRUE;
        uint8_t  ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        GAPBondMgr_SetParameter(GAPBOND_PERI_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PERI_PAIRING_MODE, sizeof(uint8_t), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_PERI_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_PERI_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_PERI_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    }

    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);           // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
    //DevInfo_AddService();                        // Device Information Service
    SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, sizeof(attDeviceName), attDeviceName);

    // Setup the SimpleProfile Characteristic Values
    {
        uint8_t charValue1[SIMPLEPROFILE_CHAR_TXT_LEN] = {};
        uint8_t charValue2[SIMPLEPROFILE_CHAR_CMD_LEN] = {};
        uint8_t charValue3[SIMPLEPROFILE_CHAR_ANS_LEN] = {};

        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR_TXT, SIMPLEPROFILE_CHAR_TXT_LEN, charValue1);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR_CMD, SIMPLEPROFILE_CHAR_CMD_LEN, charValue2);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR_ANS, SIMPLEPROFILE_CHAR_ANS_LEN, charValue3);
    }

    // Init Connection Item
    peripheralInitConnItem(&peripheralConnList);

    // Register callback with SimpleGATTprofile
    SimpleProfile_RegisterAppCBs(&Peripheral_SimpleProfileCBs);

    // Register receive scan request callback
    GAPRole_BroadcasterSetCB(&Broadcaster_BroadcasterCBs);

    // Setup a delayed profile startup
    tmos_set_event(Peripheral_TaskID, SBP_START_DEVICE_EVT);

    LL_ConnectEventRegister( peripheralConnectEventCB );

    HalLedBlink(HAL_LED_1, 0, BLINK_DUR_MS, BLINK_PERIOD_MS);
}

/*********************************************************************
 * @fn      peripheralInitConnItem
 *
 * @brief   Init Connection Item
 *
 * @param   peripheralConnList -
 *
 * @return  NULL
 */
static void peripheralInitConnItem(peripheralConnItem_t *peripheralConnList)
{
    peripheralConnList->connHandle = GAP_CONNHANDLE_INIT;
    peripheralConnList->connInterval = 0;
    peripheralConnList->connSlaveLatency = 0;
    peripheralConnList->connTimeout = 0;
}

/*********************************************************************
 * @fn      Peripheral_ProcessEvent
 *
 * @brief   Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16_t Peripheral_ProcessEvent(uint8_t task_id, uint16_t events)
{
    (VOID)task_id; // TMOS required parameter that isn't used in this function

    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(Peripheral_TaskID)) != NULL)
        {
            Peripheral_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);
            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & SBP_START_DEVICE_EVT)
    {
        // Start the Device
        GAPRole_PeripheralStartDevice(Peripheral_TaskID, &Peripheral_BondMgrCBs, &Peripheral_PeripheralCBs);
        return (events ^ SBP_START_DEVICE_EVT);
    }

    if(events & SBP_PARAM_UPDATE_EVT)
    {
        // Send connect param update request
        GAPRole_PeripheralConnParamUpdateReq(peripheralConnList.connHandle,
                                             DEFAULT_DESIRED_MIN_CONN_INTERVAL,
                                             DEFAULT_DESIRED_MAX_CONN_INTERVAL,
                                             DEFAULT_DESIRED_SLAVE_LATENCY,
                                             DEFAULT_DESIRED_CONN_TIMEOUT,
                                             Peripheral_TaskID);

        return (events ^ SBP_PARAM_UPDATE_EVT);
    }

    if(events & SBP_PHY_UPDATE_EVT)
    {
        // start phy update
        PRINT("PHY Update %x...\n", GAPRole_UpdatePHY(peripheralConnList.connHandle, 0, 
                    GAP_PHY_BIT_LE_2M, GAP_PHY_BIT_LE_2M, GAP_PHY_OPTIONS_NOPRE));

        return (events ^ SBP_PHY_UPDATE_EVT);
    }

    if(events & SBP_READ_RSSI_EVT)
    {
        GAPRole_ReadRssiCmd(peripheralConnList.connHandle);
        tmos_start_task(Peripheral_TaskID, SBP_READ_RSSI_EVT, SBP_READ_RSSI_EVT_PERIOD);
        return (events ^ SBP_READ_RSSI_EVT);
    }
    if(events & SBP_PERIODIC_EVT)
    {
        tx_img_tmr_cnt++;
        tmos_start_task(Peripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD);
        return (events ^ SBP_PERIODIC_EVT);
    }
    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      Peripheral_ProcessGAPMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Peripheral_ProcessGAPMsg(gapRoleEvent_t *pEvent)
{
    switch(pEvent->gap.opcode)
    {
    /*
        case GAP_SCAN_REQUEST_EVENT:
        {
            PRINT("Receive scan req from %x %x %x %x %x %x  ..\n", pEvent->scanReqEvt.scannerAddr[0],
                  pEvent->scanReqEvt.scannerAddr[1], pEvent->scanReqEvt.scannerAddr[2], pEvent->scanReqEvt.scannerAddr[3],
                  pEvent->scanReqEvt.scannerAddr[4], pEvent->scanReqEvt.scannerAddr[5]);
            break;
        }
     */
        case GAP_PHY_UPDATE_EVENT:
        {
            PRINT("Phy update Rx:%x Tx:%x ..\n", pEvent->linkPhyUpdate.connRxPHYS, pEvent->linkPhyUpdate.connTxPHYS);
            break;
        }

        default:
            break;
    }
}

/*********************************************************************
 * @fn      Peripheral_ProcessTMOSMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Peripheral_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch(pMsg->event)
    {
        case GAP_MSG_EVENT:
        {
            Peripheral_ProcessGAPMsg((gapRoleEvent_t *)pMsg);
            break;
        }

        case GATT_MSG_EVENT:
        {
            gattMsgEvent_t *pMsgEvent;

            pMsgEvent = (gattMsgEvent_t *)pMsg;
            if(pMsgEvent->method == ATT_MTU_UPDATED_EVENT)
            {
                peripheralMTU = pMsgEvent->msg.exchangeMTUReq.clientRxMTU;
                PRINT("mtu exchange: %d\n", pMsgEvent->msg.exchangeMTUReq.clientRxMTU);
            }
            break;
        }

        default:
            break;
    }
}

/*********************************************************************
 * @fn      Peripheral_LinkEstablished
 *
 * @brief   Process link established.
 *
 * @param   pEvent - event to process
 *
 * @return  none
 */
static void Peripheral_LinkEstablished(gapRoleEvent_t *pEvent)
{
    gapEstLinkReqEvent_t *event = (gapEstLinkReqEvent_t *)pEvent;

    // See if already connected
    if(peripheralConnList.connHandle != GAP_CONNHANDLE_INIT)
    {
        GAPRole_TerminateLink(pEvent->linkCmpl.connectionHandle);
        PRINT("Connection max...\n");
    }
    else
    {
        peripheralConnList.connHandle = event->connectionHandle;
        peripheralConnList.connInterval = event->connInterval;
        peripheralConnList.connSlaveLatency = event->connLatency;
        peripheralConnList.connTimeout = event->connTimeout;
        peripheralMTU = ATT_MTU_SIZE;

        // Set timer for param update event
        tmos_start_task(Peripheral_TaskID, SBP_PARAM_UPDATE_EVT, SBP_PARAM_UPDATE_DELAY);

        // Start read rssi
        tmos_start_task(Peripheral_TaskID, SBP_READ_RSSI_EVT, SBP_READ_RSSI_EVT_PERIOD);

        PRINT("Conn %d: Int=%d, Lat=%d, tmout=%d\n",
                event->connectionHandle,
                event->connInterval,
                event->connLatency,
                event->connTimeout);
    }
}

/*********************************************************************
 * @fn      Peripheral_LinkTerminated
 *
 * @brief   Process link terminated.
 *
 * @param   pEvent - event to process
 *
 * @return  none
 */
static void Peripheral_LinkTerminated(gapRoleEvent_t *pEvent)
{
    gapTerminateLinkEvent_t *event = (gapTerminateLinkEvent_t *)pEvent;

    if(event->connectionHandle == peripheralConnList.connHandle)
    {
        peripheralConnList.connHandle = GAP_CONNHANDLE_INIT;
        peripheralConnList.connInterval = 0;
        peripheralConnList.connSlaveLatency = 0;
        peripheralConnList.connTimeout = 0;
        tmos_stop_task(Peripheral_TaskID, SBP_READ_RSSI_EVT);

        // Restart advertising
        {
            uint8_t advertising_enable = TRUE;
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertising_enable);
        }
    }
    else
    {
        PRINT("ERR. LinkTerminated\n");
    }
}

/*********************************************************************
 * @fn      peripheralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void peripheralRssiCB(uint16_t connHandle, int8_t rssi)
{
    if (!tx_remain_bytes())
    {
        PRINT("RSSI -%d dB, rate=%d, mem=%d\n", -rssi,
            (int)(tx_rate*(1000*8/5)/(SBP_READ_RSSI_EVT_PERIOD)),
            (int)tmos_memory_getlen());
        tx_rate = 0;
    }
    (void)connHandle;
    (void)rssi;
}

/*********************************************************************
 * @fn      peripheralParamUpdateCB
 *
 * @brief   Parameter update complete callback
 *
 * @param   connHandle - connect handle
 *          connInterval - connect interval
 *          connSlaveLatency - connect slave latency
 *          connTimeout - connect timeout
 *
 * @return  none
 */
static void peripheralParamUpdateCB(uint16_t connHandle, uint16_t connInterval,
                                    uint16_t connSlaveLatency, uint16_t connTimeout)
{
    if(connHandle == peripheralConnList.connHandle)
    {
        peripheralConnList.connInterval = connInterval;
        peripheralConnList.connSlaveLatency = connSlaveLatency;
        peripheralConnList.connTimeout = connTimeout;

        PRINT("Update %d. Interval %dms \n", connHandle, connInterval * 5 / 4);
    }
    else
    {
        PRINT("ERR ParamUpdate\n");
    }
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB(gapRole_States_t newState, gapRoleEvent_t *pEvent)
{
    switch(newState & GAPROLE_STATE_ADV_MASK)
    {
        case GAPROLE_STARTED:
            PRINT("Initialized..\n");
            break;

        case GAPROLE_ADVERTISING:
            if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
            {
                Peripheral_LinkTerminated(pEvent);
                PRINT("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
                PRINT("Advertising..\n");
            }
            else if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Advertising..\n");
            }
            break;

        case GAPROLE_CONNECTED:
            if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                Peripheral_LinkEstablished(pEvent);
                PRINT("Connected..\n");
            }
            break;

        case GAPROLE_CONNECTED_ADV:
            if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Connected Advertising..\n");
            }
            break;

        case GAPROLE_WAITING:
            if(pEvent->gap.opcode == GAP_END_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Waiting for advertising..\n");
            }
            else if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
            {
                Peripheral_LinkTerminated(pEvent);
                PRINT("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
            }
            else if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                if(pEvent->gap.hdr.status != SUCCESS)
                {
                    PRINT("Waiting for advertising..\n");
                }
                else
                {
                    PRINT("Error..\n");
                }
            }
            else
            {
                PRINT("Error..%x\n", pEvent->gap.opcode);
            }
            break;

        case GAPROLE_ERROR:
            PRINT("Error..\n");
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      peripheralChar4Notify
 *
 * @brief   Prepare and send simpleProfileChar4 notification
 *
 * @param   pValue - data to notify
 *          len - length of data
 *
 * @return  none
 */
static int peripheralChar4Notify(uint8_t *pValue, uint16_t len)
{
    // MSG_BUFFER_NOT_AVAIL
    attHandleValueNoti_t noti;
    uint8_t state = SUCCESS;
    if(len > (peripheralMTU - 3))
    {
        PRINT("Too large noti\n");
        return bleInvalidRange;
    }
    noti.len = len;
    noti.pValue = GATT_bm_alloc(peripheralConnList.connHandle, ATT_HANDLE_VALUE_NOTI, noti.len, NULL, 0);
    if(noti.pValue)
    {
        tmos_memcpy(noti.pValue, pValue, noti.len);
        state = simpleProfile_Notify(peripheralConnList.connHandle, &noti);
        if( state != SUCCESS)
        {
            GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
        }
    }
    else {
        return bleMemAllocError;
    }
    return state;
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *          pValue - pointer to data that was changed
 *          len - length of data
 *
 * @return  none
 */
static void simpleProfileChangeCB(uint8_t paramID, uint8_t *pValue, uint16_t len)
{
    switch(paramID)
    {
        case SIMPLEPROFILE_CHAR_CMD:
        {
            uint8_t bfr[SIMPLEPROFILE_CHAR_CMD_LEN] = {};
            tmos_memcpy(bfr, pValue, len);
            tx_pckt_size = MIN(bfr[0], peripheralMTU-3);

            PRINT("Get values: pckt size %d bytes, total size %d MBytes\n", bfr[0], bfr[1]);

            if (bfr[1])
            {
                tx_total_size = bfr[1] * 128 * 1024;
                tx_total_size = MIN(tx_total_size, 1024*1024);
            } else {
                tx_total_size = 1024*1024;
            }

            tx_img_start();
            break;
        }

        default:
            // should not reach here!
            break;
    }
}


/*********************************************************************
 * @fn      peripheralConnectEventCB
 *
 * @brief   Callback from once connect event
 *
 * @param   timeUs - Time to next connect event
 *
 * @return  none
 */
static void peripheralConnectEventCB( uint32_t timeUs )
{
    static uint8_t bfr[256];
    (void)timeUs;

    int len = tx_remain_bytes(), err=0, pckt_cnt_per_cb = 0, pckt_wait = 0;
    tx_rate++;

    if ( len > 0 )
    {
        tx_cnt[0]++;

        while (!err && (len > 0) && (pckt_wait < BLE_TX_NUM_EVENT))
        {
            tx_cnt[1]++;
            len = tx_img_get( bfr, MIN(tx_pckt_size, (int)sizeof(bfr)) );

            err = peripheralChar4Notify(bfr, len);

            if (err == 0)
            {
                pckt_cnt_per_cb++;
                len = tx_img_next(len);
            }
            pckt_wait=(int)LL_GetNumberOfUnAckPacket(peripheralConnList.connHandle);
            unsigned mem_cur = tmos_memory_getlen();
            mem_min = MIN(mem_cur, mem_min);
        } 
        pckt_max = MAX(pckt_max, pckt_cnt_per_cb);

        if (err)
            switch (err){
                case bleInvalidRange:        er[1]++; break;
                case MSG_BUFFER_NOT_AVAIL:   er[2]++; break;
                case bleMemAllocError:       er[3]++; break;
                case bleTimeout:             er[4]++; break;
                case bleIncorrectMode:       er[5]++; break;
                case INVALIDPARAMETER:       er[6]++; break;
                case blePending:             er[7]++; break;
                default: er[0]+=1; break;
            }

        if ( len <= 0 )
            tx_img_finish();
    }
}

void upd_txt(void *bfr_, unsigned sz)
{
    int len = 0;
    char *bfr = (char *)bfr_;

    len = snprintf(bfr, sz, "sz=%dMB(%dp*%d) tm=%d00ms spd=%dkbit/s per_int=%d heap_min=%d\ner_cnt=%d",
        tx_total_size/1024/1024, tx_pckt_cnt, tx_pckt_size, tx_img_tmr_cnt, spd, pckt_max,
        (int)mem_min, er_cnt);
    if (er_cnt)
    {
        len += snprintf(&bfr[len], sz, "\n");
        for (unsigned i=0; i<ARRAY_SIZE(er); i++)
            len += snprintf(&bfr[len], sz, "%d=%d  ", i, er[i]);
    }
}

/*********************************************************************
*********************************************************************/
