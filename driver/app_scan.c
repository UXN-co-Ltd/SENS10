/*
 * app_scan.c
 *
 *  Created on: 2024. 8. 2.
 *      Author: sodykim
 */

#include "app.h"


extern device_info_t device_list[4];

// For Central connection
//uint8_t serviceUUID[2] = {0x09, 0x18}; // HTM service UUID : 0x1809
//uint8_t serviceUUID[16] = {0xe0, 0x93, 0xf3, 0xb5, 0x00, 0xa3, 0xa9, 0xe5, 0x9e, 0xca, 0x40, 0x01, 0x6e, 0x0e, 0xdc, 0x24};
uint8_t serviceUUID[16] = { 0x24,  0xdc, 0x0e, 0x6e, 0x01, 0x40, 0xca, 0x9e, 0xe5, 0xa9, 0xa3, 0x00, 0xb5, 0xf3,0x93, 0xe0};

//uint8_t characteristicUUID[2] = {0x1c, 0x2A}; // Temperature Measurement characteristics : 0x2A1C
uint8_t characteristicUUID[16] = {0x9b, 0x9f, 0x45, 0xa3, 0x73, 0x5d, 0xf0, 0xba, 0xc1, 0x4f, 0x35, 0x6f, 0x2f, 0xfc, 0x55, 0x2e};



uint8_t device_MAC_address[4][6] = {
#if 0
    {0x58, 0x0D, 0x67, 0x72, 0x02, 0x5C},   // # slot 1
    {0x61, 0x0D, 0x67, 0x72, 0x02, 0x5C},   // # slot 2
    {0x59, 0x0D, 0x67, 0x72, 0x02, 0x5C},   // # slot 3
    {0x62, 0x0D, 0x67, 0x72, 0x02, 0x5C}    // # slot 4
#else
    {0x83, 0x0B, 0x67, 0x72, 0x02, 0x5C},   // # slot 1
    {0x80, 0x0B, 0x67, 0x72, 0x02, 0x5C},   // # slot 2
    {0x69, 0x0B, 0x67, 0x72, 0x02, 0x5C},   // # slot 3
    {0x82, 0x0B, 0x67, 0x72, 0x02, 0x5C}    // # slot 4

#endif
};


uint8_t connecting_handle = 0x00;
uint32_t serviceHandle = 0xFFFFFFFF;
uint16_t characteristicHandle = 0xFFFF;

bool enabling_indications = false;
bool connecting = false;
bool discovering_service = false;
bool discovering_characteristic = false;


sl_bt_gap_phy_type_t scanning_phy = sl_bt_gap_1m_phy;
sl_bt_gap_phy_type_t connection_phy = sl_bt_gap_1m_phy;

#define SCAN_TIMEOUT_SEC                10
#define CONNECT_TIMEOUT_SEC       15

static sl_sleeptimer_timer_handle_t scan_timer;
static sl_sleeptimer_timer_handle_t connection_timeout_timer;

static void scan_timeout_callback(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;

  stop_scan();
}

void connection_timeout_callback(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)data;
  (void)handle;


}

/***************************************************************************//**
 * Central Functions
 ******************************************************************************/
void sl_bt_scan_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bool scan_result;

  uint8_t   data_recv[128];
  size_t    data_recv_len;

  uint8_t   dev_index;
  uint8_t   closed_handle;


  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      /* if connection role is CENTRAL ...*/
      if(evt->data.evt_connection_opened.master == CR_CENTRAL)
      {
        func_central_connection_opened(evt->data.evt_connection_opened.connection);
      }
      break;

    /***************************************************************************//**
     * Central Response
     ******************************************************************************/
    // -------------------------------
    // This event is generated when an advertisement packet or a scan response
    // is received from a peripheral device
    case sl_bt_evt_scanner_scan_report_id:

#if 0
      printf("Scanned Device : %02X-%02X-%02X-%02X-%02X-%02X\r\n",
             evt->data.evt_scanner_scan_report.address.addr[0],
             evt->data.evt_scanner_scan_report.address.addr[1],
             evt->data.evt_scanner_scan_report.address.addr[2],
             evt->data.evt_scanner_scan_report.address.addr[3],
             evt->data.evt_scanner_scan_report.address.addr[4],
             evt->data.evt_scanner_scan_report.address.addr[5]
      );
#endif

#if 1

      device_list->address_type = evt->data.evt_scanner_scan_report.address_type;

        // Handle scan result
        scan_result = func_scanner_scan_report(&(evt->data.evt_scanner_scan_report),
                                               evt->data.evt_scanner_scan_report.address,
                                               evt->data.evt_scanner_scan_report.address_type,
                                               get_device_list(),
                                               get_numOfActiveConn());

      if(scan_result)
      {
        /* Increment numOfActiveConn */
        increase_numOfActiveConn();
      }
#endif

      break;

    case sl_bt_evt_gatt_service_id:
      /* save the service handle for the Health Thermometer service */
      set_serviceHandle(evt->data.evt_gatt_service.service);

      printf("sl_bt_evt_gatt_service_id\r\n");

      break;

    case sl_bt_evt_gatt_characteristic_id:
      /* save the characteristic handle for the Temperature Measurement characteristic */
      set_characteristicHandle(evt->data.evt_gatt_characteristic.characteristic);

      printf("sl_bt_evt_gatt_characteristic_id\r\n");

      break;

    case sl_bt_evt_gatt_procedure_completed_id:
      func_gatt_procedure_completed(evt->data.evt_gatt_procedure_completed.connection);


      break;

    case sl_bt_evt_gatt_characteristic_value_id:

      /* if data was received from a slave... */
      if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_OPC_R2_read)
      {
          // Read characteristic value.
          sc = sl_bt_gatt_server_read_attribute_value(gattdb_OPC_R2_read,
                                0,
                                sizeof(data_recv),
                                &data_recv_len,
                                data_recv);

          if(sc != SL_STATUS_OK)
          {
#if 0
              printf("sl_bt_evt_gatt_server_attribute_value_id : %s\r\n", (int)sc);
#endif
              break;
          }

#if 0
          printf("Recevied Data from Scan : Data = %s(%d)\r\n",
                 evt->data.evt_gatt_server_attribute_value.value.data,
                 evt->data.evt_gatt_server_attribute_value.value.len);
#else

          printf("Scan : Data = [%02x] [%02x] [%02x] [%02x] [%02x] [%02x] / (%d)\r\n",
                 evt->data.evt_gatt_server_attribute_value.value.data[0], evt->data.evt_gatt_server_attribute_value.value.data[1],
                 evt->data.evt_gatt_server_attribute_value.value.data[2],evt->data.evt_gatt_server_attribute_value.value.data[3],
                 evt->data.evt_gatt_server_attribute_value.value.data[3],evt->data.evt_gatt_server_attribute_value.value.data[5],
                 evt->data.evt_gatt_server_attribute_value.value.len);

          memset(data_recv, 0, sizeof(data_recv));
          memcpy(data_recv, evt->data.evt_gatt_server_attribute_value.value.data, evt->data.evt_gatt_server_attribute_value.value.len);

//          send_tx_data((uint8_t *)evt->data.evt_gatt_server_attribute_value.value.data, evt->data.evt_gatt_server_attribute_value.value.len);


#endif



      }


      if (evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)
      {
#if 0
          printf("[BLE][CENTRAL]Received data from handle %d, len=%d ",
                evt->data.evt_gatt_characteristic_value.connection,
                evt->data.evt_gatt_characteristic_value.value.len);

        for(uint8_t i=0; i<evt->data.evt_gatt_characteristic_value.value.len; i++) {
          printf("0x%x ", evt->data.evt_gatt_characteristic_value.value.data[i]);
        }
        printf("\r\n");
#endif
        /* Acknowledge indication */
        sc = sl_bt_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
        app_assert_status(sc);

        /* Send notifications or indications to all connected remote GATT clients */
//        sc = sl_bt_gatt_server_notify_all(temp_measurement_char,
//                                          evt->data.evt_gatt_characteristic_value.value.len,
//                                          evt->data.evt_gatt_characteristic_value.value.data);
        app_assert_status(sc);
      }
      break;

    default:
      break;
  }
}

/* returns true if the remote device address is found in the list of connected device list */
bool found_device(bd_addr bd_address, device_info_t *device_list, uint8_t numOfActiveConn)
{
  for (uint8_t i=0; i<numOfActiveConn; i++)
  {
    if (memcmp(&device_list[i].address, &bd_address, sizeof(bd_addr)) == 0)
    {
      return true; // Found
    }
  }

  return false; // Not found
}

/* returns true if the remote device address is found in the list of connected device list */
bool found_mac_address(bd_addr bd_address)
{
  for (uint8_t i=0; i<4; i++) {
      if(memcmp(device_MAC_address[i],bd_address.addr, 6)==0) {
          return true;
      }
  }

  return false; // Not found
}


bool service_found(struct sl_bt_evt_scanner_scan_report_s *pResp, uint8_t *serviceUUID, uint8_t len_of_serviceUUID)
{
  // decoding advertising
  int i = 0;
  int j;
  uint8_t adv_len;
  uint8_t adv_type;
  int len = 0;


#if 0
  if (!memcmp(serviceUUID, &(pResp->data.data[5]), len_of_serviceUUID))
  {
      printf("UUID :  %d => ", len_of_serviceUUID);
      for(int i=0;i<len_of_serviceUUID;i++) {
          printf("%02X ", pResp->data.data[i+5]);

      }
      printf("\r\n");


#if 0
      printf("success!!!\r\n");
#endif
      return true;
  }


#else
  while (i < pResp->data.len - 1)
  {
    adv_len = pResp->data.data[i];
    adv_type = pResp->data.data[i + 1];


    /* type 0x02 = Incomplete List of 16-bit Service Class UUIDs
     type 0x03 = Complete List of 16-bit Service Class UUIDs */
    if (adv_type == 0x02 || adv_type == 0x03)
    {
        printf("adv_len : %d || adv_type : %d || len_of_serviceUUID : %d\r\n", adv_len, adv_type, len_of_serviceUUID);

        printf("UUID :  %d => ", len_of_serviceUUID);
//        for(i=0;i<len_of_serviceUUID;i++) {
        for(i=0;i<(len_of_serviceUUID*2);i++) {
//            printf("%02X ", pResp->data.data[i+5]);
            printf("%02X ", pResp->data.data[i]);

        }
        printf("\r\n");



        // Look through all the UUIDs looking for HTM service
      j = i + 2; // j holds the index of the first data
      do      {
        if (!memcmp(serviceUUID, &(pResp->data.data[j]), len_of_serviceUUID))
        {
          return true;
        }
        j = j + 2;
      }
      while (j < i + adv_len);
    }
    i = i + adv_len + 1;
  }
#endif

  return false;
}

void start_scan(void)
{
  sl_status_t sc;

#if 0
  // Set passive scanning on 1Mb PHY
  sc = sl_bt_scanner_set_mode(scanning_phy, 0);
  app_assert_status(sc);
  // Set scan interval and scan window:  50%duty cycle
  sc = sl_bt_scanner_set_timing(scanning_phy, 20, 10);
  app_assert_status(sc);
  // Start scanning
  sc = sl_bt_scanner_start(scanning_phy, scanner_discover_generic);
  app_assert_status_f(sc, "Failed to start discovery #1\n");

  #else

//  sc = sl_bt_scanner_set_parameters(sl_bt_scanner_scan_mode_active, 200, 100);
  sc = sl_bt_scanner_set_parameters(sl_bt_scanner_scan_mode_active, 800, 400);    // 500ms & 250ms

#if 0
  app_assert_status(sc);
#endif
  // Start scanning
  sc = sl_bt_scanner_start(scanning_phy, sl_bt_scanner_discover_generic);
#if 0
app_assert_status_f(sc, "Failed to start discovery #1\n");
#endif


#endif

  /***************************************************************************//**
   * Set up periodic battery measurement timer.
   ******************************************************************************/
  sl_sleeptimer_start_timer_ms(&scan_timer,
                               SCAN_TIMEOUT_SEC * 1000,
                               scan_timeout_callback,
                               NULL, 0, 0);
#if 0
  printf("[BLE]Start Scan...\r\n");
#endif

}

void stop_scan(void)
{
  sl_status_t sc;

  sc = sl_bt_scanner_stop();

#if 0
  app_assert_status(sc);
  printf("[BLE]Stop Scan...\r\n");
#endif

}

sl_status_t start_connection_timeout_timer(void)
{
  sl_status_t sc;

  /* Set connection timeout timer */
  sc = sl_sleeptimer_start_timer_ms(&connection_timeout_timer,
                                    CONNECT_TIMEOUT_SEC * 1000,
                                    connection_timeout_callback,
                                    NULL, 0, 0);
  app_assert_status(sc);

  return sc;
}

sl_status_t stop_connection_timeout_timer(void)
{
  sl_status_t sc;

  /* Cancel fail safe connection timer */
  sc = sl_sleeptimer_stop_timer(&connection_timeout_timer);
#if 0
  app_assert_status(sc);
  printf("Connection timeout is cleared.\r\n");
#endif
  return sc;
}

void set_serviceHandle(uint32_t service)
{
  serviceHandle = service;
}

void set_characteristicHandle(uint16_t characteristic)
{
  characteristicHandle = characteristic;
}

void func_gatt_procedure_completed(uint8_t connection)
{
  sl_status_t sc;

  /* if service discovery completed */
  if (discovering_service)
  {
    discovering_service = false;
    /* discover Temperature Measurement characteristic */
    sc = sl_bt_gatt_discover_characteristics_by_uuid(connection, serviceHandle, sizeof(characteristicUUID), characteristicUUID);
#if 0
    printf("sl_bt_gatt_discover_characteristics_by_uuid : %d\r\n", sc);
    app_assert_status(sc);
#endif
    discovering_characteristic = true;


  }
  /* if characteristic discovery completed */
  else if (discovering_characteristic)
  {
    discovering_characteristic = false;
    /* enable indications on the Temperature Measurement characteristic */
    sc = sl_bt_gatt_set_characteristic_notification(connection, characteristicHandle, sl_bt_gatt_notification);
#if 0
    printf("sl_bt_gatt_set_characteristic_notification : %d\r\n", sc);
    app_assert_status(sc);
#endif
    enabling_indications = true;
  }
  else if (enabling_indications)
  {
    enabling_indications = 0;
  }
}

void func_central_connection_opened(uint8_t connection)
{
  sl_status_t sc;

  stop_connection_timeout_timer();

  /* Start discovering the remote GATT database */
  sc = sl_bt_gatt_discover_primary_services_by_uuid(connection, sizeof(serviceUUID), serviceUUID);
  app_assert_status(sc);

  discovering_service = true;

  /* connection process completed. */
  connecting = false;
}

bool func_scanner_scan_report(struct sl_bt_evt_scanner_scan_report_s *pResp,
                              bd_addr    address,
                              uint8_t    address_type,
                              device_info_t *device_list,
                              uint8_t numOfActiveConn)
{
  sl_status_t sc;

  uint8_t i = 0, ad_len, ad_type;
  bool ad_match_found = false;

  char name[32];

  /* Exit event if max connection is reached */
  if (numOfActiveConn == SL_BT_CONFIG_MAX_CONNECTIONS)
    return false;

  /* Exit if device is in connection process (processing another scan response),
   * or service, or characterstics discovery */
  if (connecting || enabling_indications || discovering_service || discovering_characteristic) {
      printf("connecting out\r\n");
    return false;
  }

#if 0
  printf("$scan_address:%02X-%02X-%02X-%02X-%02X-%02X\r\n",
         address.addr[0],
         address.addr[1],
         address.addr[2],
         address.addr[3],
         address.addr[4],
         address.addr[5]);

        return false;


#else

//        if(found_mac_address(address)==false) {
//            return false;
//}
#endif

#if 1

        printf("pResp->data.len:[%d][%d][%d]\r\n",pResp->data.len, pResp->data.data[i], pResp->data.data[i+1]);

        while (i < (pResp->data.len - 1)) {
          ad_len = pResp->data.data[i];
          ad_type = pResp->data.data[i + 1];


//          if ((ad_type == 0x08) || (ad_type == 0x09)) {
          if ((ad_type == 0x02) || (ad_type == 0x03)) {
            // Type 0x08 = Shortened Local Name
            // Type 0x09 = Complete Local Name
            memcpy(name, &(pResp->data.data[i + 2]), ad_len - 1);
            name[ad_len - 1] = 0;
            app_log("%s\r\n", name);
          }
        }
#endif

        printf("$mac_address:%02X-%02X-%02X-%02X-%02X-%02X\r\n",
               address.addr[5],
               address.addr[4],
               address.addr[3],
               address.addr[2],
               address.addr[1],
               address.addr[0]);

#if 0
  /* Exit event if service is not in the scan response*/
  if (!service_found(pResp, serviceUUID, sizeof(serviceUUID)))
    return false;


  printf("$mac_address:%02X-%02X-%02X-%02X-%02X-%02X\r\n",
         address.addr[5],
         address.addr[4],
         address.addr[3],
         address.addr[2],
         address.addr[1],
         address.addr[0]);

#endif

#if 0
  /* Exit event if the scan response is triggered by a device already in the connection list. */
  if (found_device(address, device_list, numOfActiveConn))
    return false;

  /* Max connection isn't reached, device is not in a connection process, new HTM service is found.
   * Continue ...*/

  /* Initiate connection */
  connecting = true;
  sc = sl_bt_connection_open(address, address_type, connection_phy, &connecting_handle);
  app_assert_status(sc);

  /* Update device list. If connection doesn't succeed (due to timeout) the device will be removed from the list in connection closed event handler*/
  device_list[numOfActiveConn].address = address;
  device_list[numOfActiveConn].address_type = address_type;
  device_list[numOfActiveConn].conn_handle = connecting_handle;
  device_list[numOfActiveConn].conn_role = CR_PERIPHERAL; // connection role of the remote device
  device_list[numOfActiveConn].conn_state = CS_CONNECTING;

  /* Set connection timeout timer */
  start_connection_timeout_timer();
#endif

  return true;
}

void func_connection_timeout(device_info_t *device_list,    uint8_t numOfActiveConn)
{
  sl_status_t sc;
  uint8_t dev_index;

  /* Connection fail safe timer triggered, cancel connection procedure and restart scanning/discovery */
#if 0
  printf("Connection timeout!\r\n");
  printf("Cancel connection with device :");
#endif

  dev_index = get_dev_index(connecting_handle, device_list, numOfActiveConn);

#if 0
  print_bd_addr(device_list[dev_index].address);
  printf("\r\n");
  printf("Handle .......: #%d\r\n", connecting_handle);
#endif

  // CANCEL CONNECTION
  sc = sl_bt_connection_close(connecting_handle);
  app_assert_status(sc);
  connecting = false;

}

