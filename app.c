/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "app.h"

#define ADV_PERIOD_MS                   (1000)

/***************************************************************************************************************************
 *  100ms USER TIMER EVENT HANDLER
 ***************************************************************************************************************************/
/** Timer used for periodic update of the adc measurements. */
static sl_sleeptimer_timer_handle_t main_master_timer;
/** Time (in ms) between periodic updates of the measurements. */
#define MAIN_MASTER_INTERVAL_MS     (100)

#define MAX_PACKET_SIZE   200

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
static uint8_t bond_handle = SL_BT_INVALID_BONDING_HANDLE;

static uint16_t max_packet_size;

uint8_t numOfActiveConn = 0; // number of active connections <= SL_BT_CONFIG_MAX_CONNECTIONS
device_info_t device_list[SL_BT_CONFIG_MAX_CONNECTIONS];
static bool use_ota_function = false;

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////

#ifdef __DEBUG_MSG__
    printf("\r\n");
    printf("===============================================================\r\n");
    printf("   SENS-100K  V 1.0.0   %s , %s\r\n", __DATE__, __TIME__);
    printf("===============================================================\r\n");
    printf("\r\n");
#endif

    IADC_Polling_Init();

    spidrv_app_init();
}


/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/


/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////



}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
    sl_status_t sc;
    uint8_t   i;
    uint8_t   data_send[128];
    uint8_t   data_recv[64];
    size_t   data_recv_len;

    uint16_t ps;
    uint16_t QPos;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
/*****************************************************************************
  *     Print MAC Address / Change System ID and Device Name
  ******************************************************************************/
        bootMessage(&(evt->data.evt_system_boot));
        change_system_id();
        change_device_name();

/*****************************************************************************
  *   Create an advertising set
  ******************************************************************************/
          sc = sl_bt_advertiser_create_set(&advertising_set_handle);
          app_assert_status(sc);

          start_advertisement(advertising_set_handle, ADV_PERIOD_MS);

      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      stop_advertisement(advertising_set_handle);
      bond_handle = evt->data.evt_connection_opened.bonding;
      set_server_connection_id(evt->data.evt_connection_opened.connection);
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // handle of the closed connection
      set_server_connection_id(INVALID_HANDLE);

      start_advertisement(advertising_set_handle, ADV_PERIOD_MS);
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

      /**************************************************************************************************************************
        *   Add additional event handlers here as your application requires!
        **************************************************************************************************************************/
          case sl_bt_evt_system_external_signal_id:
              switch(evt->data.evt_system_external_signal.extsignals)
              {
                  case EXT_SENS_DATA_SEND:
                      for(i=0;i<8;i++) {
                          memset(data_send, 0, sizeof(data_send));

                          ps = 0;
                          data_send[ps++] = OPC_R2_info.mac_address[1];
                          data_send[ps++] = OPC_R2_info.mac_address[0];
                          data_send[ps++] = (uint8_t)OPC_R2_info.vbat_lvl_mv;
                          data_send[ps++] = (uint8_t)((OPC_R2_info.vbat_lvl_mv - (uint8_t)OPC_R2_info.vbat_lvl_mv) * 100);
                          data_send[ps++] = 0;
                          data_send[ps++] = OPC_R2_Connection(i);
                          data_send[ps++] = 0;
                          data_send[ps++] = i+1;

                          QPos = i*64;
                          memcpy(&data_send[ps], &OPC_R2_buff[QPos], 64);

                          send_tx_data(data_send, 72);                   // 시리얼로 수신된 정보를 앱으로 전송 한다..
                      }
                    break;

                  default:
                      break;
              }

              break;

/**************************************************************************************************************************
  *   MTU UPDATE 512 -> 517
  **************************************************************************************************************************/
          case sl_bt_evt_gatt_mtu_exchanged_id:
            /* Calculate maximum data per one notification / write-without-response,
             * this depends on the MTU.
             * up to ATT_MTU-3 bytes can be sent at once  */

            max_packet_size = evt->data.evt_gatt_mtu_exchanged.mtu - 3;

      #ifdef __DEBUG_MSG__
            app_log_info("[BLE]MTU exchanged: %d\r\n", evt->data.evt_gatt_mtu_exchanged.mtu);
      #endif
            break;

          // This event occurs when the remote device enabled or disabled the
          // notification.
        case sl_bt_evt_gatt_server_characteristic_status_id:
            if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_OPC_R2_send)
            {
                if (evt->data.evt_gatt_server_characteristic_status.client_config_flags
                    & sl_bt_gatt_notification)
                {
                    set_OPC_R2_notification(API_TRUE);     // The client just enabled the notification.
#ifdef __DEBUG_MSG__
                       app_log_info("[BLE]cgm_data_tx_notification is enabled\r\n");
#endif
                }

                else   {
                    set_OPC_R2_notification(API_FALSE);

#ifdef __DEBUG_MSG__
                   app_log_info("[BLE]cgm_data_tx_notification is disabled\r\n");
#endif
                }
            }
            break;

          // -------------------------------
          // This event indicates that the value of an attribute in the local GATT
          // database was changed by a remote GATT client.
          case sl_bt_evt_gatt_server_attribute_value_id:
            if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_OPC_R2_read)
            {
              // Read characteristic value.
                sc = sl_bt_gatt_server_read_attribute_value(gattdb_OPC_R2_read,
                                                            0,
                                                            sizeof(data_recv),
                                                            &data_recv_len,
                                                            data_recv);


                  if(sc != SL_STATUS_OK)     {
#ifdef __DEBUG_MSG__
                      app_log_info("[BLE][E: 0x%04x] Failed to receive data(gattdb_cgm_data)\r\n", (int)sc);
#endif
                    break;
                  }

            }
            break;

          case sl_bt_evt_connection_parameters_id:

#ifdef __DEBUG_MSG__
            app_log_info("[BLE]Connection parameters:\r\n");
            app_log_info("\t Interval    = %u units\r\n", evt->data.evt_connection_parameters.interval);
            app_log_info("\t latency    = %d units\r\n", evt->data.evt_connection_parameters.latency);
            app_log_info("\t Tx Size     = %u\r\n", evt->data.evt_connection_parameters.txsize);
#endif



            break;


    // -------------------------------
    // Default event handler.




    default:
      break;
  }
}



/*****************************************************************************
 * data send evt handler
 ******************************************************************************/
sl_status_t send_tx_data(uint8_t *data_send, size_t data_send_len)
{
    sl_status_t sc;

    if(get_server_connection_id() == INVALID_HANDLE)  {
        return SL_STATUS_FAIL;
    }

    if(get_OPC_R2_notification() == INVALID_HANDLE)    {
        return SL_STATUS_INVALID_STATE;
    }


    sc = sl_bt_gatt_server_write_attribute_value(gattdb_OPC_R2_send,
                                     0,
                                     data_send_len,
                                     (const uint8_t*)data_send);

    sc = sl_bt_gatt_server_send_notification(get_server_connection_id(),
                                 gattdb_OPC_R2_send,
                                 data_send_len,
                                 (const uint8_t*)data_send);
    return sc;
}




uint8_t get_numOfActiveConn(void)
{
  return numOfActiveConn;
}

void increase_numOfActiveConn(void)
{
  numOfActiveConn++;
}

device_info_t *get_device_list(void)
{
  return device_list;
}



/***************************************************************************//**
 * @brief Callback from timer ...100ms
 ******************************************************************************/
static void main_master_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;

}

void run_main_master_timer(void)
{
  /***************************************************************************//**
   * Set up periodic battery measurement timer.
   ******************************************************************************/
  sl_sleeptimer_start_periodic_timer_ms(&main_master_timer,
                                        MAIN_MASTER_INTERVAL_MS,
                                        main_master_timer_callback,
                                        NULL, 0, 0);

}


void stop_main_master_timer(void)
{
    sl_sleeptimer_stop_timer(&main_master_timer);
}


