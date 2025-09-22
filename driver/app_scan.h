/*
 * app_scan.h
 *
 *  Created on: 2024. 8. 2.
 *      Author: sodykim
 */

#ifndef APP_SCAN_H_
#define APP_SCAN_H_



/***************************************************************************//**
 * BLE Multicentral Multiperipheral Dual Topology Parameters
 ******************************************************************************/
typedef enum
{
  // Connection States (CS)
  CS_CONNECTED,
  CS_CONNECTING,
  CS_CLOSED
} conn_state_t;

/*------------CHANGE THE ABOVE ENUM USING THIS ONE-----------------*/
typedef enum {
  scanning,
  opening,
  discover_services,
  discover_characteristics,
  enable_indication,
  running
} conn_state;
/*----------------------------------------------------------------*/

typedef enum
{
  // Connection Roles (CR)
  CR_PERIPHERAL,
  CR_CENTRAL
} conn_role_t;

/* Struct to store the connecting device address, our device role in the connection, and connection state*/
typedef struct
{
  bd_addr address;
  uint8_t address_type;
  conn_role_t conn_role;
  conn_state_t conn_state;
  uint8_t conn_handle;
  uint8_t bond_handle;
} device_info_t;

// For Central connection
void sl_bt_scan_on_event(sl_bt_msg_t *evt);
bool found_device(bd_addr bd_address, device_info_t *device_list, uint8_t numOfActiveConn);
bool service_found(struct sl_bt_evt_scanner_scan_report_s *pResp, uint8_t *serviceUUID, uint8_t len_of_serviceUUID);
void start_scan(void);
void stop_scan(void);
sl_status_t start_connection_timeout_timer(void);
sl_status_t stop_connection_timeout_timer(void);
void set_serviceHandle(uint32_t service);
void set_characteristicHandle(uint16_t characteristic);
void func_gatt_procedure_completed(uint8_t connection);
void func_central_connection_opened(uint8_t connection);
bool func_scanner_scan_report(struct sl_bt_evt_scanner_scan_report_s *pResp,
                              bd_addr    address,
                              uint8_t    address_type,
                              device_info_t *device_list,
                              uint8_t numOfActiveConn);
void func_connection_timeout(device_info_t *device_list,
                             uint8_t numOfActiveConn);

#endif /* APP_SCAN_H_ */
