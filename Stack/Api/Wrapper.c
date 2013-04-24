//==============================================================================
//  Copyright 2011 Meta Watch Ltd. - http://www.MetaWatch.org/
//
//  Licensed under the Meta Watch License, Version 1.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.MetaWatch.org/licenses/license-1.0.html
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//==============================================================================

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

#include "hci.h"
#include "l2cap.h"
#include "btstack_memory.h"
#include "remote_device_db.h"
#include "rfcomm.h"
#include "sdp.h"
#include "config.h"
#include "bt_control_cc256x.h"

#include <stdarg.h>
#include <stdio.h>

enum CONNECTION_MODE { ACTIVE=0, HOLD=1, SNIFF=2, PARK=3 } ;
enum STATE {INIT, W4_CONNECTION, W4_CHANNEL_COMPLETE, W4_ACTIVE} ;

static uint8_t   rfcomm_channel_nr = 1;
static uint16_t  rfcomm_mtu;
static uint16_t  rfcomm_connection_handle;
static uint16_t  rfcomm_channel_id;
static enum CONNECTION_MODE rfcomm_connection_mode = ACTIVE;
static uint8_t   spp_service_buffer[100];
static uint16_t  sniff_max_interval = 1600;  // time unit: 0.625us
static uint16_t  sniff_min_interval = 1344;
static uint16_t  sniff_return_timeout = 1000; // time unit: ms
static uint16_t  sniff_attempt = 10;
static uint16_t  sniff_timeout = 10;
static timer_source_t sniff_timer;
static uint8_t   first_time = 1;

static enum STATE state = INIT;

// Flag to enable packet dumps
int btstack_enable_dump_mode=0;

// Next data to sent
uint8_t *tx_data;
uint16_t tx_data_len = 0;
uint16_t tx_data_max_len = 0;

// Activates a new timer
static void run_loop_register_timer(timer_source_t *timer, uint16_t period) {
  run_loop_set_timer(timer, period);
  run_loop_remove_timer(timer);
  run_loop_add_timer(timer);
}

// Sets active mode
static void transit_to_active_mode()
{
  // If sniff mode is on, request active mode
  if (rfcomm_connection_mode == SNIFF) {
    if (hci_can_send_packet_now(HCI_COMMAND_DATA_PACKET)) {
      hci_send_cmd(&hci_exit_sniff_mode,rfcomm_connection_handle);
    }
  }
  // In active mode, ensure that return to sniff timeout is updated
  if (rfcomm_connection_mode == ACTIVE) {
    run_loop_register_timer(&sniff_timer, sniff_return_timeout);
  }
}

// Sets sniff mode
static void transit_to_sniff_mode()
{
  if (rfcomm_connection_mode == ACTIVE) {
    if (hci_can_send_packet_now(HCI_COMMAND_DATA_PACKET)) {
      hci_send_cmd(&hci_sniff_mode,rfcomm_connection_handle,sniff_max_interval,sniff_min_interval,sniff_attempt,sniff_timeout);
    }
    run_loop_register_timer(&sniff_timer,sniff_return_timeout);
  }
}

// Send routine
static void try_to_send(void){

  if (!rfcomm_channel_id) return;
  if (tx_data_len==0) return;

  // Set active mode
  transit_to_active_mode();

  // TODO: split packets according to mtu
  int err = rfcomm_send_internal(rfcomm_channel_id, tx_data, tx_data_len);

  switch (err){
    case 0:
      tx_data_len=0;
    case BTSTACK_ACL_BUFFERS_FULL:
      break;
    default:
      break;
  }
}

// Bluetooth packet handler
static void packet_handler(void * connection, uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
  bd_addr_t event_addr;
  uint8_t event = packet[0];
  uint16_t connection_handle;
  uint8_t current_mode;
  uint8_t status;
  uint16_t interval;

  // Handle data packets
  if (packet_type == RFCOMM_DATA_PACKET) {

    // Set active mode
    transit_to_active_mode();

    // hack: truncate data (we know that the packet is at least on byte bigger
    packet[size] = 0;
    //puts( (const char *) packet);
    if (tx_data) {
      strcpy(tx_data, packet);
      tx_data_len=strlen(tx_data)+1;
      try_to_send();
    }
    rfcomm_grant_credits(rfcomm_channel_id, 1); // get the next packet
    return;
  }

  // handle events, ignore data
  if (packet_type != HCI_EVENT_PACKET) return;

  switch(state) {
    case INIT:
      switch(event){
        case BTSTACK_EVENT_STATE:
          // bt stack activated, get started - set local name
          if (packet[2] == HCI_STATE_WORKING) {
            hci_send_cmd(&hci_write_local_name, "MetaWatch Digital WDS112 (BTStack)");
            btstack_enable_dump_mode=0;
            state=W4_CONNECTION;
            puts("BTStack initialized");
          }
          break;
        default:
          break;
      }
      break;

    case W4_CONNECTION:
      switch(event){
        case HCI_EVENT_PIN_CODE_REQUEST:
          // inform about pin code request
          //printf("Pin code request - using '0000'\n\r");
          bt_flip_addr(event_addr, &packet[2]);
          hci_send_cmd(&hci_pin_code_request_reply, &event_addr, 4, "0000");
          break;
        case RFCOMM_EVENT_INCOMING_CONNECTION:
          // data: event (8), len(8), address(48), channel (8), rfcomm_cid (16)
          bt_flip_addr(event_addr, &packet[2]);
          rfcomm_channel_nr = packet[8];
          rfcomm_channel_id = READ_BT_16(packet, 9);
          //printf("RFCOMM channel %u requested for %s\n\r", rfcomm_channel_nr, bd_addr_to_str(event_addr));
          rfcomm_accept_connection_internal(rfcomm_channel_id);
          state = W4_CHANNEL_COMPLETE;
          break;
        default:
          break;
      }
      break;

    case W4_CHANNEL_COMPLETE:
      if ( event != RFCOMM_EVENT_OPEN_CHANNEL_COMPLETE ) break;

      // data: event(8), len(8), status (8), address (48), handle (16), server channel(8), rfcomm_cid(16), max frame size(16)
      if (packet[2]) {
        //printf("RFCOMM channel open failed, status %u\n\r", packet[2]);
        break;
      }
      rfcomm_connection_handle = READ_BT_16(packet, 9);
      rfcomm_channel_id = READ_BT_16(packet, 12);
      rfcomm_mtu = READ_BT_16(packet, 14);
      tx_data_max_len = rfcomm_mtu;
      if (!(tx_data=malloc(tx_data_max_len))) {
        puts("ERROR: Can not allocate memory for tx buffer!");
      }

      // Setup sniff mode
      rfcomm_connection_mode=ACTIVE;
      transit_to_sniff_mode();

      //printf("\n\rRFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n\r", rfcomm_channel_id, rfcomm_mtu);
      puts("RFCOMM channel opened");
      state = W4_ACTIVE;
      break;

    case W4_ACTIVE:
      switch(event){
        case HCI_EVENT_MODE_CHANGE_EVENT:
          // Todo: find out if sniff mode is active
          // data: event (8), len (8), status (8), handle (16), current_mode (8), interval (16)
          connection_handle = READ_BT_16(packet, 3);
          if (connection_handle==rfcomm_connection_handle) {
            status = packet[2];
            if (status!=0) {
              //puts("ERROR: Mode change command failed!");
            }
            current_mode = packet[5];
            interval = READ_BT_16(packet,6);
            //printf("current_mode=%d interval=%d\n",current_mode,interval);
            switch (current_mode) {
              case 0:
                // Ensure that sniff mode is re-activated after timeout
                rfcomm_connection_mode = ACTIVE;
                run_loop_register_timer(&sniff_timer,sniff_return_timeout);
                break;
              case 2:
                rfcomm_connection_mode = SNIFF;
                if (first_time) {
                  strcpy(tx_data, "Test3");
                  tx_data_len=strlen(tx_data)+1;
                  try_to_send();
                  first_time = 0;
                }
                break;
              default:
                puts("ERROR: Unsupported connection mode!");
            }
          }
          break;
        case DAEMON_EVENT_HCI_PACKET_SENT:
        case RFCOMM_EVENT_CREDITS:
          try_to_send();
          break;
        case RFCOMM_EVENT_CHANNEL_CLOSED:
          rfcomm_channel_id = 0;
          state = W4_CONNECTION;
          free(tx_data);
          tx_data=NULL;
          puts("RFCOMM channel closed");
          break;
        case HCI_EVENT_COMMAND_STATUS:
        case HCI_EVENT_NUMBER_OF_COMPLETED_PACKETS:
        case HCI_EVENT_READ_REMOTE_VERSION_INFORMATION_COMPLETE:
        case 0x73:
          break;
        default:
          printf("WARNING: unknown event 0x%02x received!\n",event);
          break;
      }
      break;

    default:
      break;
  }
}

/*! Initaliaze the serial port profile task.  This should be called from main.
*
* This task opens the stack which in turn creates 3 more tasks that create and
* handle the bluetooth serial port connection.
*/
void InitializeWrapperTask(void)
{
  /// GET STARTED with BTstack ///
  btstack_memory_init();
  run_loop_init(RUN_LOOP_EMBEDDED);

  // init HCI
  hci_transport_t    * transport = hci_transport_h4_dma_instance();
  bt_control_t       * control   = bt_control_cc256x_instance();
  hci_uart_config_t  * config    = hci_uart_config_cc256x_instance();
  remote_device_db_t * remote_db = (remote_device_db_t *) &remote_device_db_memory;
  hci_init(transport, config, control, remote_db);

  // use eHCILL
  bt_control_cc256x_enable_ehcill(1);

  // init L2CAP
  l2cap_init();
  l2cap_register_packet_handler(packet_handler);

  // init RFCOMM
  rfcomm_init();
  rfcomm_register_packet_handler(packet_handler);
  rfcomm_register_service_with_initial_credits_internal(NULL, rfcomm_channel_nr, 100, 1);  // reserved channel, mtu=100, 1 credit

  // init SDP, create record for SPP and register with SDP
  sdp_init();
  memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
  service_record_item_t * service_record_item = (service_record_item_t *) spp_service_buffer;
  sdp_create_spp_service( (uint8_t*) &service_record_item->service_record, 1, "MW-SPP");
  sdp_register_service_internal(NULL, service_record_item);

  // Init sniff mode timer
  sniff_timer.process = &transit_to_sniff_mode;

  // ready - enable irq used in h4 task
  __enable_interrupt();

  // turn on!
  hci_power_control(HCI_POWER_ON);

  // make discoverable
  hci_discoverable_control(1);

  // go!
  run_loop_execute();

  // TODO: Adapt all hal_* files to power down handling (especially dma))
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
  return 0;
}

/*! Return a pointer to the wrapper version string */
tVersion GetWrapperVersion(void)
{
  tVersion version;
  version.pBtVer = "BTSTK ?.?-?";
  version.pHwVer = "SPP 2560";
  version.pSwVer = "?";
  return version;
}

/*! Determine if the bluetooth link is in the connected state.
* When the phone and watch are connected then data can be sent.
*
* \return 0 when not connected, 1 when connected
*/
unsigned char QueryPhoneConnected(void)
{
  return 0;
}

/*! This function is used to determine if the radio is on and will return 1 when
* initialization has successfully completed, but the radio may or may not be
* paired or connected.
*
* \return 1 when stack is in the connected, paired, or radio on state, 0 otherwise
*/
unsigned char QueryBluetoothOn(void)
{
  return 0;
}

/*! This function is used to determine if the connection state of the
 * bluetooth serial port.
 *
 * \return etConnectionState
 */
etConnectionState QueryConnectionState(void)
{
  return Unknown;
}

/*! Query Bluetooth Discoverability
 *
 * \return 0 when not discoverable, 1 when discoverable
 */
unsigned char QueryDiscoverable(void)
{
  return 0;
}

/*! Query Bluetooth Connectability
 *
 * \return 0 when not connectable, 1 when connectable
 */
unsigned char QueryConnectable(void)
{
  return 0;
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
unsigned char QueryAutoSniffEnabled(void)
{
  return 0;
}

/*!
 * \param DelayMs is the delay for entering sniff mode
 */
void SetSniffModeEntryDelay(unsigned int DelayMs)
{
}

etSniffState QuerySniffState(void)
{
  return PhoneNotConnected;
}

/*! Allow access to the 4 sniff parameters
 *
 * \param SniffSlotType is the parameter type
 * \param Slots is the new parameter value in slots
 */
void SetSniffSlotParameter(eSniffSlotType SniffSlotType,unsigned int Slots)
{

}


/*! \return The sniff Slot parameter
 *
 * \param SniffSlotType is the parameter type
 */
unsigned int QuerySniffSlotParameter(eSniffSlotType SniffSlotType)
{
  return 0;
}
