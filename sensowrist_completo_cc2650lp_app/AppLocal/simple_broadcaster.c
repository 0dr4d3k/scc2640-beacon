/******************************************************************************

 @file  simple_broadcaster.c

 @brief This file contains the Simple BLE Broadcaster sample application for
        use with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2011-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_01_18
 Release Date: 2016-10-26 15:20:04
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"

#include "broadcaster.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"
#include "icall.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include "util.h"
#include <ti/mw/display/Display.h>
#include "board.h"
#include "board_key.h"

#include "simple_broadcaster.h"

#include <driverlib/aon_batmon.h>


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// Select the beacon type
#define BEACON_WRISTBAND
//#define BEACON_KEYRINGUS


#ifdef BEACON_WRISTBAND
#define PERIODO_ADVERTISING_EN_SEGUNDOS          3
#define PERIODO_ADVERTISING_ALARMA_EN_SEGUNDOS   1

#define EVENTOS_EN_UN_MINUTO                     60/PERIODO_ADVERTISING_ALARMA_EN_SEGUNDOS // Se usa para borrar la alarma
#define LED_BLINK_DURATION_MS                    50 // Flashing led every PERIODO_ADVERTISING_ALARMA_EN_SEGUNDOS time
#endif

#ifdef BEACON_KEYRINGUS
#define PERIODO_ADVERTISING_EN_SEGUNDOS          7
#define PERIODO_ADVERTISING_ALARMA_EN_SEGUNDOS   0

#define EVENTOS_EN_UN_MINUTO                     0 // Alarm no allowed in keyringus mode
#define LED_BLINK_DURATION_MS                    0 // Led on pressing pushbutton
#endif

#define PERIODO_ADV_KEEPALIVE_EN_SEGUNDOS        10

// Wakeup timer (in milliseconds)
#define WAKEUP_TIMER                             10*1000 // Time pressing the button to start advertising

// Key timers (in milliseconds)
#define SHORTKEY_TIMER                           10*1000 // Time pressing the button to enter in keepalive state
#define LONGKEY_TIMER                            20*1000 // Time pressing the button to enter in warehouse state

// Initial led gretting (in milliseconds)
#define HELLOWORLD_TIMER                         5*1000  // Initial led ON timer

// Battery period (in milliseconds)
#define BATTERY_PERIOD                           50*1000 // Battery measure period in seconds

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms), valid values: 32-16384
#define LONG_ADVERTISING_INTERVAL           (PERIODO_ADV_KEEPALIVE_EN_SEGUNDOS*1600)
#define DEFAULT_ADVERTISING_INTERVAL        (PERIODO_ADVERTISING_EN_SEGUNDOS*1600)
#define ALARM_ADVERTISING_INTERVAL          (PERIODO_ADVERTISING_ALARMA_EN_SEGUNDOS*1600)

// Task configuration
#define SBB_TASK_PRIORITY                     1

#ifndef SBB_TASK_STACK_SIZE
#define SBB_TASK_STACK_SIZE                   660
#endif

// Internal Events for RTOS application
#define SBB_STATE_CHANGE_EVT                  0x0001
#define SBB_KEY_CHANGE_EVT                    0x0002
//#define SBB_LONGKEY_TIMEOUT_EVT               0x0004
//#define SBB_SHORTKEY_TIMEOUT_EVT              0x0008
#define SBB_ADV_EVT                    		  0x0080

// Customer NV Items - Range 0x80 - 0x8F -
#define SNV_ID_CONFIG          0x80

// Flags in SNV_CONFIG register
#define FLAG_FIRST_INI         0x01
#define FLAG_WAREHOUSE         0x02

// Application states definitions
#define STATE_WAREHOUSE        0x01
#define STATE_ADV_NORMAL       0x02
#define STATE_ADV_ALARM        0x03
#define STATE_ADV_KEEPALIVE    0x04

#define ADV_STOP               0x01
#define ADV_DEFAULT            0x02
#define ADV_ALARM              0x03
#define ADV_KEEPALIVE          0x04

/*********************************************************************
 * TYPEDEFS
 */
// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // Event header.
} sbbEvt_t;


/*********************************************************************
 * GLOBAL VARIABLES
 */
// Display Interface
Display_Handle dispHandle = NULL;


/*********************************************************************
 * EXTERNAL VARIABLES
 */


/*********************************************************************
 * EXTERNAL FUNCTIONS
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Alarm counter
static uint8_t alarmCounter=0;

// Battery value
static uint8_t batt;

// Systen flag
static bool keyTimeoutShort = false;
static bool keyTimeoutLong  = false;

// No volatile configuration register
static uint8_t snvConfigReg;

// Application Moore automate state.
static uint8_t appState = STATE_WAREHOUSE;

// Task configuration
Task_Struct sbbTask;
Char sbbTaskStack[SBB_TASK_STACK_SIZE];

// GAP - SCAN RSP data (max size = 31 bytes)
// oJo, not used in advertising not connectable mode
static uint8 scanRspData[] =
{
#ifdef BEACON_WRISTBAND
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  's','m','a','r','t','c','a','r','e','-','w','r','i','s','t','b','a','n','d',
#endif

#ifdef BEACON_KEYRINGUS
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  's','m','a','r','t','c','a','r','e','-','k','e','y','r','i','n','g','u','s',
#endif

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED|GAP_ADTYPE_FLAGS_GENERAL,


  // three-byte broadcast of the data "1 2 3"
  0x04,   // length of this data including the data type byte
  GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv. data type
  0x41,
  0, // status
  0  // counter
};

PIN_State  ledCtrlState;
PIN_Config ledCtrlCfg[] =
{
		Board_LED1    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off               */
		PIN_TERMINATE
};
PIN_Handle ledCtrlHandle;

// Timers
static Clock_Struct initialLEDTimer;
static Clock_Struct batteryMeasureTimer;
static Clock_Struct wakeupTimer;
static Clock_Struct shortkeyTimer;
static Clock_Struct longkeyTimer;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SimpleBLEBroadcaster_init(void);
static void SimpleBLEBroadcaster_taskFxn(UArg a0, UArg a1);

static void SimpleBLEBroadcaster_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBLEBroadcaster_processAppMsg(sbbEvt_t *pMsg);
static void SimpleBLEBroadcaster_processStateChangeEvt(gaprole_States_t newState);

static void SimpleBLEBroadcaster_stateChangeCB(gaprole_States_t newState);

void SimpleBLEBroadcaster_keyChangeHandler(uint8 keys);

void SimpleBLEPeripheral_atuomateHandler(uint8 keys);

void setAdvIntData(uint8_t adv_mode);

static void InitialLEDTimingHandler(UArg a0)
{
	PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_OFF);
}

static void BatteryMeasureTimingHandler(UArg a0)
{
    // Battery monitor (bit 10:8 - integer, but 7:0 fraction)
    uint32_t batt_raw = AONBatMonBatteryVoltageGet();

    // Parse and round battery raw data
    uint8_t  intPart = (batt_raw & 0x0300) >> 4;
    uint32_t dPart   = ((batt_raw & 0x00FF) * 100) / 256;
    uint8_t  decPart = (dPart / 10) + (dPart % 10>5);
    if (decPart == 10) {decPart=0; intPart++;}

    // Compose battery
    batt = intPart | decPart;
}


static void longkeyTimingHandler(UArg a0)
{
    keyTimeoutLong = true;
    /*
    sbbEvt_t *pMsg;

    // Create dynamic pointer to message.
    if ((pMsg = ICall_malloc(sizeof(sbbEvt_t))))
    {
      pMsg->hdr.event = SBB_LONGKEY_TIMEOUT_EVT;
      pMsg->hdr.state = true;

      // Enqueue the message.
      Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
    }
    */
}

static void shortkeyTimingHandler(UArg a0)
{
    keyTimeoutShort = true;
    /*
    sbbEvt_t *pMsg;

    // Create dynamic pointer to message.
    if ((pMsg = ICall_malloc(sizeof(sbbEvt_t))))
    {
      pMsg->hdr.event = SBB_SHORTKEY_TIMEOUT_EVT;
      pMsg->hdr.state = true;

      // Enqueue the message.
      Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
    }
    */
}

/*********************************************************************
 * PROFILE CALLBACKS
 */
// GAP Role Callbacks
static gapRolesCBs_t simpleBLEBroadcaster_BroadcasterCBs =
{
  SimpleBLEBroadcaster_stateChangeCB   // Profile State Change Callbacks
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEBroadcaster_createTask
 *
 * @brief   Task creation function for the Simple BLE Broadcaster.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLEBroadcaster_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbbTaskStack;
  taskParams.stackSize = SBB_TASK_STACK_SIZE;
  taskParams.priority = SBB_TASK_PRIORITY;

  Task_construct(&sbbTask, SimpleBLEBroadcaster_taskFxn, &taskParams, NULL);
}


/*********************************************************************
 * @fn      SimpleBLEBroadcaster_init
 *
 * @brief   Initialization function for the Simple BLE Broadcaster App
 *          Task. This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Hard code the DB Address till CC2650 board gets its own IEEE address
  //uint8 bdAddress[B_ADDR_LEN] = { 0x33, 0x33, 0x33, 0x33, 0x33, 0x33 };
  //HCI_EXT_SetBDADDRCmd(bdAddress);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Open LCD
  dispHandle = Display_open(Display_Type_LCD, NULL);

  // Register Key Call Back
  Board_initKeys(SimpleBLEBroadcaster_keyChangeHandler);

  // Setup the GAP Broadcaster Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initial_advertising_enable = FALSE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t gapRole_AdvertOffTime = 0;

/*#ifndef BEACON_FEATURE
    uint8_t advType = GAP_ADTYPE_ADV_SCAN_IND; // use scannable undirected adv
#else*/
    uint8_t advType = GAP_ADTYPE_ADV_NONCONN_IND; // use non-connectable adv
//#endif // !BEACON_FEATURE

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initial_advertising_enable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &gapRole_AdvertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof (scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &advType);
  }

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Start the Device
  HCI_EXT_SetTxPowerCmd( HCI_EXT_TX_POWER_5_DBM);

  VOID GAPRole_StartDevice(&simpleBLEBroadcaster_BroadcasterCBs);

  // Fetch configuration register
  if (osal_snv_read(SNV_ID_CONFIG, sizeof(snvConfigReg), &snvConfigReg) != SUCCESS)
  {
      // First time initialization after a burning procedure
      snvConfigReg = FLAG_FIRST_INI | FLAG_WAREHOUSE;

      // Write first time to initialize SNV ID
      osal_snv_write(SNV_ID_CONFIG, sizeof(snvConfigReg), &snvConfigReg);

      // Initial application state after a burning procedure is warehouse mode
      appState = STATE_WAREHOUSE;
  }
  else
  {
      /*
      // Subsequent initializations the app initialize in previous warehouse state
      appState = (snvConfigReg & FLAG_WAREHOUSE)? STATE_WAREHOUSE:STATE_ADV_NORMAL;

      // Clear first time initialization after a burning procedure
      snvConfigReg &= ~FLAG_FIRST_INI;

      // Write in SNV
      osal_snv_write(SNV_ID_CONFIG, sizeof(snvConfigReg), &snvConfigReg);

      // Launch default advertising interval
      if (appState == STATE_ADV_NORMAL)
        */
      appState = STATE_ADV_NORMAL;
          setAdvIntData(ADV_DEFAULT);
  }

  // First hello world auto start led
  ledCtrlHandle = PIN_open(&ledCtrlState, ledCtrlCfg);
  PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_ON);
  Util_constructClock(&initialLEDTimer,
                      InitialLEDTimingHandler,
                      HELLOWORLD_TIMER, 0, true, 0);

  // Battery measure clock
  Util_constructClock(&batteryMeasureTimer,
                      BatteryMeasureTimingHandler,
                      BATTERY_PERIOD, BATTERY_PERIOD, true, 0);

  // Longkey timer constructor
  Util_constructClock(&longkeyTimer,
                      longkeyTimingHandler,
                      LONGKEY_TIMER, 0, false, 0);

  // Shortkey timer constructor
  Util_constructClock(&shortkeyTimer,
                      shortkeyTimingHandler,
                      SHORTKEY_TIMER, 0, false, 0);

  Display_print0(dispHandle, 0, 0, "BLE Broadcaster");

  HCI_EXT_AdvEventNoticeCmd(selfEntity, SBB_ADV_EVT);
}


/*********************************************************************
 * @fn      SimpleBLEBroadcaster_processEvent
 *
 * @brief   Application task entry point for the Simple BLE Broadcaster.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLEBroadcaster_init();

  // Application main loop
  for (;;)
  {
    // Get the ticks since startup
    uint32_t tickStart = Clock_getTicks();

    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          SimpleBLEBroadcaster_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbbEvt_t *pMsg = (sbbEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          SimpleBLEBroadcaster_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }
  }
}


/*********************************************************************
 * @fn      SimpleBLEBroadcaster_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_processStackMsg(ICall_Hdr *pMsg)
{
	ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;
	// Check for BLE stack events first
	if (pEvt->signature == 0xffff)
	{
		if (pEvt->event_flag & SBB_ADV_EVT)
		{
			if(alarmCounter>0)
			{
				advertData[6]=0x80;
				alarmCounter--;

				PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_ON);
				Util_restartClock(&initialLEDTimer, LED_BLINK_DURATION_MS);

				if(alarmCounter==0)
				{
				    setAdvIntData(ADV_DEFAULT);

                    advertData[6]=0x00;
                    PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_OFF);
				}
			}
			else
			{
				advertData[6]=0x00;
			}

            // Compose and update advertising data
            advertData[6] |= batt; // battery
            advertData[7]++;       // counter

			GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
		}
	}
}


/*********************************************************************
 * @fn      SimpleBLEBroadcaster_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLEBroadcaster_keyChangeHandler(uint8 keys)
{
  sbbEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbbEvt_t))))
  {
    pMsg->hdr.event = SBB_KEY_CHANGE_EVT;
	pMsg->hdr.state = keys;

	// Enqueue the message.
	Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_atuomateHandle
 *
 * @brief   Moore automate implementation for smartcare-beacon
 *
 * @param
 *
 * @return  none
 */
void SimpleBLEPeripheral_atuomateHandler(uint8_t key)
{
  static uint8_t appStateNew;

  switch (appState)
  {

/// warehouse state ///////////////////////////////////// Beacon Automate //////
    case STATE_WAREHOUSE:
    case STATE_ADV_KEEPALIVE:
    {
      // KEY_1 pressed (rising edge interruption handled)
      if (key)
      {
#ifdef BEACON_WRISTBAND
          // Set advertising data
          setAdvIntData(ADV_ALARM);

          // Set alarm counter
          alarmCounter = EVENTOS_EN_UN_MINUTO;

          // Launch alarm led
          PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_ON);
          Util_restartClock(&initialLEDTimer, LED_BLINK_DURATION_MS);

          // Next state
//          appStateNew = STATE_ADV_ALARM;
          appStateNew = STATE_ADV_NORMAL;
#endif

#ifdef BEACON_KEYRINGUS
          // Set advertising data
          setAdvIntData(ADV_DEFAULT);

          // Led on
          PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_ON);

          // Next state
          appStateNew = STATE_ADV_NORMAL;
#endif

      }

      // KEY_1 not pressed (falling edge interruption handled)
      else
      {
          // Next state
          appStateNew = appState;
      }
    }
    break;

/// advertising normal ////////////////////////////////// Beacon Automate //////
    case STATE_ADV_NORMAL:

        // Handle key interruptions
        if (key) // Rising Edge
        {
            // Actions: 1.- restart longkey timer
            // Actions: 1.- restart shortkey timer
            Util_restartClock(&shortkeyTimer, SHORTKEY_TIMER);
            Util_restartClock(&longkeyTimer, LONGKEY_TIMER);

#ifdef BEACON_KEYRINGUS
            // Led on
            PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_ON);
#endif

            // Next state
            appStateNew = appState;

        }

        else // Falling Edge
        {
            // Stop all timers
            Util_stopClock(&shortkeyTimer);
            Util_stopClock(&longkeyTimer);

            // Compute key_time flags
            if (keyTimeoutLong)
            {
                // Stop advertising
                setAdvIntData(ADV_STOP);

                // Launch keepalive led
                PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_ON);
                Util_restartClock(&initialLEDTimer, LED_BLINK_DURATION_MS*40);

                // Next state
                appStateNew = STATE_WAREHOUSE;
            }

            else if (keyTimeoutShort)
            {
                // Set advertising data
                setAdvIntData(ADV_KEEPALIVE);

                // Launch keepalive led
                PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_ON);
                Util_restartClock(&initialLEDTimer, LED_BLINK_DURATION_MS*10);

                // Next state
                appStateNew = STATE_ADV_KEEPALIVE;
            }

            else
            {

#ifdef BEACON_WRISTBAND
              // Set advertising data
              setAdvIntData(ADV_ALARM);

              // Set alarm counter
              alarmCounter = EVENTOS_EN_UN_MINUTO;

              // Launch alarm led
              PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_ON);
              Util_restartClock(&initialLEDTimer, LED_BLINK_DURATION_MS);

              // Next state
//              appStateNew = STATE_ADV_ALARM;
              appStateNew = STATE_ADV_NORMAL;
#endif


#ifdef BEACON_KEYRINGUS
                // Led off
                PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_OFF);

                // Next state
                appStateNew = appState;
#endif

            }

            // Clear flags
            keyTimeoutLong  = false;
            keyTimeoutShort = false;
        }

      break;

/// advertising alarm /////////////////////////////////// Beacon Automate //////
    case STATE_ADV_ALARM:
/*
        if (alarmTimeout)
        {
            // Actions: 1.- clear alarm led

            // Action:  2.- stop advertising packets
            // Action:  3.- set normal advertise data and timing
            // Action:  4.- launch advertising packets

            // Next state
            appStateNew = STATE_ADV_NORMAL;
        }
        else
        {
            // Next state
            appStateNew = appState;
        }
*/
      break;

/// advertising keepalive /////////////////////////////// Beacon Automate //////
/*
    case STATE_ADV_KEEPALIVE:

        if (key)
        {
            // Actions: 1.- launch alarm timeout.
            // Actions: 2.- launch alarm led.

            // Action:  3.- stop advertising packets
            // Action:  4.- set alarm advertise data and timing
            // Action:  5.- launch advertising packets


            // Next state
            appStateNew = STATE_ADV_ALARM;
        }
        else
        {
            // Next state
            appStateNew = appState;
        }

      break;
*/
/// unknow state!! /////////////////////////////////////////////////////////////
    default:
        // Should never get here!
      break;
  }

  // Update appState
  appState = appStateNew;
}


void setAdvIntData(uint8_t adv_mode)
{
    uint8_t initial_advertising_enable;

    // Stop the actual advertising data
    initial_advertising_enable = FALSE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                     &initial_advertising_enable);

    // Stop adverising
    if (adv_mode == ADV_STOP) return;

    uint16_t advInt;
    // Set advertising interval
    switch (adv_mode)
    {
      // Set advertising interval for alarm event
      case ADV_DEFAULT:   advInt = DEFAULT_ADVERTISING_INTERVAL;  break;

      // Set advertising interval for alarm event
      case ADV_ALARM:     advInt = ALARM_ADVERTISING_INTERVAL;    break;

      // Set advertising interval for alarm event
      case ADV_KEEPALIVE: advInt = LONG_ADVERTISING_INTERVAL;     break;

      default: break;
    }

    // Write GAP parameter
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

    // Start advertising data
    initial_advertising_enable = TRUE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initial_advertising_enable);

}




/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_processAppMsg(sbbEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBB_STATE_CHANGE_EVT:
      SimpleBLEBroadcaster_processStateChangeEvt((gaprole_States_t)pMsg->
                                                 hdr.state);
      break;

    case SBB_KEY_CHANGE_EVT:
        SimpleBLEPeripheral_atuomateHandler(pMsg->hdr.state);

/*
        // Wait for Wakeup
        if (!wakeup)
        {
            // KEY_1 pressed (rising edge interruption handled)
            if(pMsg->hdr.state)
            {
               Util_restartClock(&wakeupTimer, WAKEUP_TIMER);
            }

            // KEY_1 not pressed (falling edge interruption handled)
            else
            {
               Util_stopClock(&wakeupTimer);
            }

            break;
        }

        // KEY_1 pressed (rising edge interruption handled)
    	if(pMsg->hdr.state)
    	{
            uint8_t initial_advertising_enable;

    	    // WRISTBAND MODE
    		if (ALARM_ADVERTISING_INTERVAL)
    		{
    		    // Stop the actual advertising data
                initial_advertising_enable = FALSE;
                GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                                 &initial_advertising_enable);

                // Set advertising interval for alarm event
                uint16_t advInt = ALARM_ADVERTISING_INTERVAL;

                GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
                GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
                GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
                GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

                alarmCounter=EVENTOS_EN_UN_MINUTO;

                initial_advertising_enable = TRUE;
                GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                     &initial_advertising_enable);

                // Led flashing every ALARM_ADVERTISING_INTERVAL
                PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_ON);
                Util_restartClock(&initialLEDTimer, LED_BLINK_DURATION_MS);
    		}

            // KEYRINGUS MODE
    		else
    		{
    		    // First advertising when the button has been pressed
                initial_advertising_enable = TRUE;
                GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                     &initial_advertising_enable);

                // Led on
                PIN_setOutputValue(ledCtrlHandle, Board_LED1, !PIN_getOutputValue(Board_LED1));
    		}
    	}

    	// KEY_1 not pressed (falling edge interruption handled)
    	else
    	{
    	    // Led off
            PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_OFF);
    	}
*/
      break;

    default:
      // Do nothing.
      break;
  }
}


/*********************************************************************
 * @fn      SimpleBLEBroadcaster_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_stateChangeCB(gaprole_States_t newState)
{
  sbbEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbbEvt_t))))
  {
    pMsg->hdr.event = SBB_STATE_CHANGE_EVT;
    pMsg->hdr.state = newState;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}


/*********************************************************************
 * @fn      SimpleBLEBroadcaster_processStateChangeEvt
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_processStateChangeEvt(gaprole_States_t newState)
{
  switch (newState)
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // Display device address
        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
        Display_print0(dispHandle, 2, 0, "Initialized");
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        Display_print0(dispHandle, 2, 0, "Advertising");
      }
      break;

    case GAPROLE_WAITING:
      {
        Display_print0(dispHandle, 2, 0, "Waiting");
      }
      break;

    case GAPROLE_ERROR:
      {
        Display_print0(dispHandle, 2, 0, "Error");
      }
      break;

    default:
      {
        Display_clearLine(dispHandle, 2);
      }
      break;
  }
}


/*********************************************************************
*********************************************************************/
