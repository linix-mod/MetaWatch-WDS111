/*
 * Copyright (C) 2009-2013 by Matthias Ringwald, Matthias Gruenewald
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY MATTHIAS RINGWALD, MATTHIAS GRUENEWALD
 * AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * MATTHIAS RINGWALD, MATTHIAS GRUENEWALD OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at btstack@ringwald.ch
 * and gruenewald75@yahoo.de
 *
 */

/******************************************************************************/
/*! \file Wrapper.c
*
*/
/******************************************************************************/

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "portmacro.h"

#include "Messages.h"
#include "MessageQueues.h"
#include "BufferPool.h"

#include "hal_lpm.h"
#include "hal_board_type.h"
#include "hal_rtc.h"
#include "hal_miscellaneous.h"
#include "hal_analog_display.h"
#include "hal_battery.h"
#include "hal_miscellaneous.h"
#include "hal_calibration.h"

#include "DebugUart.h"
#include "Adc.h"
#include "Wrapper.h"
#include "Background.h"
#include "Buttons.h"
#include "LcdDisplay.h"
#include "Display.h"
#include "Utilities.h"
#include "Accelerometer.h"
#include "Buttons.h"
#include "Vibration.h"
#include "OneSecondTimers.h"
#include "Statistics.h"

#include "OSAL_Nv.h"
#include "NvIds.h"

#include <string.h>
#include <btstack/hci_cmds.h>
#include <btstack/run_loop.h>
#include <btstack/sdp_util.h>
#include <btstack/version.h>
#include <run_loop_private.h>

#include "hci.h"
#include "l2cap.h"
#include "btstack_memory.h"
#include "remote_device_db.h"
#include "rfcomm.h"
#include "sdp.h"
#include "config.h"
#include "bt_control_cc256x.h"
#include "spp.h"

#include <stdarg.h>
#include <stdio.h>

#define SPP_MSG_QUEUE_LEN   8
#define SPP_STACK_SIZE    (configMINIMAL_STACK_SIZE + 200)
#define SPP_TASK_PRIORITY   (tskIDLE_PRIORITY + 2)

static tMessage SPPMsg;

static etConnectionState ConnectionState = Unknown;
static uint8_t Discoverable=0;

xTaskHandle xSPPTaskHandle;

/*! Sends a connection state message */
void ConnectionStateChanged(etConnectionState CS) {
  tMessage OutgoingMsg;
  ConnectionState=CS;
  SetupMessage(&OutgoingMsg, ConnectionStateChangeMsg, CS);
  RouteMsg(&OutgoingMsg);
}

/*! Sets the visibility of the device */
void SetDiscoverability(unsigned char Value) {
  hci_discoverable_control(Value);
  Discoverable=Value;
}

/*! Handle the messages routed to the SPP task */
static unsigned char SPPMessageHandler(tMessage* pMsg)
{
  tMessage OutgoingMsg;

  switch(pMsg->Type)
  {
    case TriggerBTStackRunLoopMsg:
      run_loop_execute();
      break;

    case TurnRadioOnMsg:
      hci_power_control(HCI_POWER_ON);
      SetDiscoverability(1);
      ConnectionStateChanged(Initializing);
      break;

    case TurnRadioOffMsg:
      SetDiscoverability(0);
      ConnectionStateChanged(Initializing);
      hci_power_control(HCI_POWER_OFF);
      break;

    case GetDeviceTypeResponse:
      btstack_queue_tx_packet(pMsg);
      return 0;

    // TODO: Check how the other states are reached

/*
    case PairingControlMsg:
      switch (pMsg->Options) {
        case PAIRING_CONTROL_OPTION_ENABLE_PAIRING:
          break;
        case PAIRING_CONTROL_OPTION_DISABLE_PAIRING:
          break;
        case PAIRING_CONTROL_OPTION_SAVE_SPP:
          break;
        case PAIRING_CONTROL_OPTION_TOGGLE_SSP:
          break;
        default:
          PrintStringAndHex("<<Unhandled PairingControlMsg Option>> in SPP Task: 0x", pMsg->Options);
          break;
      }
      update hci_discoverable depending on visibility status
      sent ConnectionStateChangeMsg
      break;
*/

/*

      
    case GetInfoStringResponse:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case DiagnosticLoopback:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case EnterShippingModeMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;
      
    case ConnectionTimeoutMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case ReadRssiMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case PairingControlMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case ReadRssiResponseMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case SniffControlMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case GetRealTimeClockResponse:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case StatusChangeEvent:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case NvalOperationResponseMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case GeneralPurposePhoneMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case ButtonEventMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case AccelerometerHostMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case QueryMemoryMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case LowBatteryWarningMsgHost:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case LowBatteryBtOffMsgHost:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case ReadBatteryVoltageResponse:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case ReadLightSensorResponse:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case AdvertisingDataMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case CallbackTimeoutMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case RadioPowerControlMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

    case UpdateConnParameterMsg:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;

*/

    default:
      PrintStringAndHex("<<Unhandled Message>> in SPP Task: Type 0x", pMsg->Type);
      break;
  }
  return 1;
}

/*! Function to implement the SPPTask loop
 *
 * \param pvParameters not used
 *
 */
static void SPPTask(void *pvParameters)
{
  // Check that the queue was created
  if ( QueueHandles[SPP_TASK_QINDEX] == 0 )
  {
    PrintString("SPP Queue not created!\r\n");
  }

  // Init the bluetooth stack
  btstack_init();

  // TODO: Adapt all hal_* files to power down handling (especially dma))

  // Message loop
  for(;;)
  {
    // Check for messages
    if( pdTRUE == xQueueReceive(QueueHandles[SPP_TASK_QINDEX],
                                &SPPMsg, portMAX_DELAY ) )
    {
      PrintMessageType(&SPPMsg);
      if (SPPMessageHandler(&SPPMsg))
        SendToFreeQueue(&SPPMsg);
      CheckStackUsage(xSPPTaskHandle,"SPP Task");
      CheckQueueUsage(QueueHandles[SPP_TASK_QINDEX]);
    }

  }

}

/*! Initialize the serial port profile task.  This should be called from main.
*
* This task opens the stack which in turn creates 3 more tasks that create and
* handle the bluetooth serial port connection.
*/
void InitializeWrapperTask(void)
{
  // Init queue for communication with the firmware
  QueueHandles[SPP_TASK_QINDEX] =
    xQueueCreate( SPP_MSG_QUEUE_LEN, MESSAGE_QUEUE_ITEM_SIZE );

  // prams are: task function, task name, stack len , task params, priority, task handle
  xTaskCreate(SPPTask,
              (const signed char *)"SPP",
              SPP_STACK_SIZE,
              NULL,
              SPP_TASK_PRIORITY,
              &xSPPTaskHandle);
}


/*! Query the serial port profile task if it is okay to put the part into LPM3
*
* This function is called by the idle task.  The idle task puts the
* MSP430 into sleep mode.
*
* \return 0 if micro cannot go into LPM3; 1 if micro can go into LPM3
*/
unsigned char SerialPortReadyToSleep(void)
{
  return 1;
}

/*! Return a pointer to the wrapper version string */
tVersion GetWrapperVersion(void)
{
  tVersion version;
  version.pBtVer = "BTS " BTSTACK_VERSION;
  version.pHwVer = "SPP 2560";
  version.pSwVer = "0.1";
  return version;
}

/*! Determine if the bluetooth link is in the connected state.
* When the phone and watch are connected then data can be sent.
*
* \return 0 when not connected, 1 when connected
*/
unsigned char QueryPhoneConnected(void)
{
  switch(ConnectionState) {
    case BRConnected:
    case LEConnected:
      return 1;
    default:
      return 0;
  }
}

/*! This function is used to determine if the radio is on and will return 1 when
* initialization has successfully completed, but the radio may or may not be
* paired or connected.
*
* \return 1 when stack is in the connected, paired, or radio on state, 0 otherwise
*/
unsigned char QueryBluetoothOn(void)
{
  switch(ConnectionState) {
    case RadioOn:
    case Paired:
    case BRConnected:
    case LEConnected:
      return 1;
    default:
      return 0;
  }
}

/*! This function is used to determine if the connection state of the
 * bluetooth serial port.
 *
 * \return etConnectionState
 */
etConnectionState QueryConnectionState(void)
{
  switch(ConnectionState) {
    case BRConnected:
    case LEConnected:
      return Paired;
    default:
      return ConnectionState;
  }
}

/*! Query Bluetooth Discoverability
 *
 * \return 0 when not discoverable, 1 when discoverable
 */
unsigned char QueryDiscoverable(void)
{
  return Discoverable;
}

/*! Query Bluetooth Connectability
 *
 * \return 0 when not connectable, 1 when connectable
 */
unsigned char QueryConnectable(void)
{
  return 1;
}

/*! Query Bluetooth Secure Simple Pairing Setting
 *
 * \return 0 when SSP is disabled, 1 when SSP is enabled
 */
unsigned char QuerySecureSimplePairingEnabled(void)
{
  return 0;
}

/*! Query Bluetooth pairing information
 *
 * \return 0 when valid pairing does not exist, 1 when valid pairing information
 * exists
 */
unsigned char QueryValidPairingInfo(void)
{
  return 0;
}

/*! Query Link Key Information
 *
 * Fills parameters with values if the Index is valid
 *
 * \param Index is an index into the bluetooth link key structure
 * \param pBluetoothAddress is a pointer to the bluetooth address (13 byte
 *
 * \param pBluetoothName is a pointer to the bluetooth name
 * \param BluetoothNameSize is the size of string pBluetoothName points to
 */
void QueryLinkKeys(unsigned char Index,
                   tString *pBluetoothAddress,
                   tString *pBluetoothName,
                   unsigned char BluetoothNameSize)
{
}

/******************************************************************************/

/*! Query the state of the AutoSniffEnabled register
 * \return 1 if Sniff is Enabled, 0 otherwise
 */
/*
unsigned char QueryAutoSniffEnabled(void)
{
  return 0;
}
*/

/*!
 * \param DelayMs is the delay for entering sniff mode
 */
/*
void SetSniffModeEntryDelay(unsigned int DelayMs)
{
}
*/

/*
etSniffState QuerySniffState(void)
{
  return PhoneNotConnected;
}
*/

/*! Allow access to the 4 sniff parameters
 *
 * \param SniffSlotType is the parameter type
 * \param Slots is the new parameter value in slots
 */
/*
void SetSniffSlotParameter(eSniffSlotType SniffSlotType,unsigned int Slots)
{

}
*/

/*! \return The sniff Slot parameter
 *
 * \param SniffSlotType is the parameter type
 */
unsigned int QuerySniffSlotParameter(eSniffSlotType SniffSlotType)
{
  return 0;
}
