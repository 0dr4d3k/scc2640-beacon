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

#define PERIODO_ADVERTISING_EN_SEGUNDOS 3
#define PERIODO_ADVERTISING_ALARMA_EN_SEGUNDOS 1

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms), valid values: 32-16384
#define DEFAULT_ADVERTISING_INTERVAL         (PERIODO_ADVERTISING_EN_SEGUNDOS*1600)
#define ALARM_ADVERTISING_INTERVAL          (PERIODO_ADVERTISING_ALARMA_EN_SEGUNDOS*1600)
#define EVENTOS_EN_UN_MINUTO				60 // (60/PERIODO_ADVERTISING_ALARMA_EN_SEGUNDOS) // Se usa para borrar la alarma, el periodo de adverising es 1 segundo con alarma
#define LED_BLINK_DURATION_MS				50 //5

// Task configuration
#define SBB_TASK_PRIORITY                     1

#ifndef SBB_TASK_STACK_SIZE
#define SBB_TASK_STACK_SIZE                   660
#endif

// Internal Events for RTOS application
#define SBB_STATE_CHANGE_EVT                  0x0001
#define SBB_KEY_CHANGE_EVT                    0x0002
#define SBB_ADV_EVT                    		  0x0080

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

static uint8_t alarmCounter=0;

// Task configuration
Task_Struct sbbTask;
Char sbbTaskStack[SBB_TASK_STACK_SIZE];

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x15,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S',
  'i',
  'm',
  'p',
  'l',
  'e',
  'B',
  'L',
  'E',
  'B',
  'r',
  'o',
  'a',
  'd',
  'c',
  'a',
  's',
  't',
  'e',
  'r',

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
  GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv data type
  0x41,
  0, // status
  0 // counter

};

PIN_State  ledCtrlState;
PIN_Config ledCtrlCfg[] =
{
		Board_LED1    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off               */
		PIN_TERMINATE
};
PIN_Handle ledCtrlHandle;

static Clock_Struct initialLEDTimer;

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



static void InitialLEDTimingHandler(UArg a0)
{
	PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_OFF);
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

  ledCtrlHandle = PIN_open(&ledCtrlState, ledCtrlCfg);
  PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_ON);
  Util_constructClock(&initialLEDTimer, InitialLEDTimingHandler,5000, 0, true, 0);

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
			// Advertisement ended. Process as desired
			uint32_t batt;
			// Battery monitor (bit 10:8 - integer, but 7:0 fraction)
			batt = AONBatMonBatteryVoltageGet();
			uint8_t intPart = (batt&0x0300)>>4;
			uint32_t dPart = ((batt&0x00FF)*100)/256;
			uint8_t decPart = (dPart/10)+(dPart%10>5);
			if(alarmCounter>0)
			{
				advertData[6]=0x80;
				alarmCounter--;

				PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_ON);
				Util_restartClock(&initialLEDTimer, LED_BLINK_DURATION_MS);

				if(alarmCounter==0)
				{
					uint8_t initial_advertising_enable = FALSE;
					GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
													 &initial_advertising_enable);


					// Set advertising interval
					uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

					GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
					GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
					GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
					GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);


					initial_advertising_enable = TRUE;
					GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
										 &initial_advertising_enable);


					advertData[6]=0x00;
					PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_OFF);

				}
			}
			else
				advertData[6]=0x00;

			advertData[6]|=intPart|decPart;
			advertData[7]++;//counter
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
    	if(pMsg->hdr.state)
    	{
    		uint8_t initial_advertising_enable = FALSE;
    		GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
    										 &initial_advertising_enable);

    		PIN_setOutputValue(ledCtrlHandle, Board_LED1, Board_LED_ON);
    		Util_restartClock(&initialLEDTimer, LED_BLINK_DURATION_MS);

    		// Set advertising interval
			uint16_t advInt = ALARM_ADVERTISING_INTERVAL;

			GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
			GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
			GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
			GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

			alarmCounter=EVENTOS_EN_UN_MINUTO;

			initial_advertising_enable = TRUE;
			GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
								 &initial_advertising_enable);
    	}
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
