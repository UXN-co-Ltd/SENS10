/*
 * app_utility.c
 *
 *  Created on: 2022. 4. 4.
 *      Author: dooker
 */
#include "app.h"

#include "sl_sleeptimer.h"

/*******************************************************************************
 *
 *  below nvm3 area for calibration & rfsense & mode check
 *
 *******************************************************************************/

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/
/* Connection Handle ID */
uint8_t server_connection_id = INVALID_HANDLE;
uint8_t OPC_R2_notification_status = INVALID_HANDLE;

// Maximum number of data objects saved
#define MAX_OBJECT_COUNT    10

// Max and min keys for data objects
#define MIN_DATA_KEY  NVM3_KEY_MIN
#define MAX_DATA_KEY  (MIN_DATA_KEY + MAX_OBJECT_COUNT - 1)

// Key of write counter object
#define WRITE_COUNTER_KEY   MAX_OBJECT_COUNT

// Key of delete counter object
#define DELETE_COUNTER_KEY   (WRITE_COUNTER_KEY + 1)



//sl_status_t sl_sleeptimer_set_datetime(sl_sleeptimer_date_t *date);


/***************************************************************************//**
 * loop delay utility
 ******************************************************************************/


void delay_ms(uint16_t delaytime)
{
    uint32_t curTicks = get_tick_count();
    while ((get_tick_count() - curTicks) < delaytime) {}

}

void sleep_ms(uint16_t time_ms)
{
    sl_sleeptimer_delay_millisecond(time_ms);

}


/***************************************************************************//**
 * Print stack version and local Bluetooth address as boot message
 ******************************************************************************/
void bootMessage(struct sl_bt_evt_system_boot_s *bootevt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;

  app_log_info("[BLE]Stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor, bootevt->patch);
  sc = sl_bt_system_get_identity_address(&address, &address_type);
  app_assert_status(sc);

}

/***************************************************************************//**
 * Change gattdb_system_id
 ******************************************************************************/
void change_system_id(void)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  // Extract unique ID from BT Address.
  sc = sl_bt_system_get_identity_address(&address, &address_type);
  app_assert_status(sc);

  // Pad and reverse unique ID to get System ID.
  system_id[0] = address.addr[5];
  system_id[1] = address.addr[4];
  system_id[2] = address.addr[3];
  system_id[3] = 0xFF;
  system_id[4] = 0xFE;
  system_id[5] = address.addr[2];
  system_id[6] = address.addr[1];
  system_id[7] = address.addr[0];

  sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                               0,
                                               sizeof(system_id),
                                               system_id);
  app_assert_status(sc);
}

/***************************************************************************//**
 * Change gattdb_device_name
 ******************************************************************************/
/* Must match length of APP_DEVNAME_DEFAULT after printf formatting */

#define APP_DEVNAME                  "OPC-R2_%02X%02X"
#define APP_DEVNAME_DEFAULT          "OPC-R2_0000"

#define APP_DEVNAME_LEN              (sizeof(APP_DEVNAME_DEFAULT) - 1)
void change_device_name(void)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;

  char devName[APP_DEVNAME_LEN + 1];

  // Extract unique ID from BT Address.
  sc = sl_bt_system_get_identity_address(&address, &address_type);
  app_assert_status(sc);

  OPC_R2_info.mac_address[1] = address.addr[1];
  OPC_R2_info.mac_address[0] = address.addr[0];

  snprintf(devName, APP_DEVNAME_LEN + 1, APP_DEVNAME, address.addr[1], address.addr[0]);

  sc = sl_bt_gatt_server_write_attribute_value(gattdb_device_name,
                                               0,
                                               strlen(devName),
                                               (uint8_t *)devName);
  app_assert_status(sc);

  printf("[BLE]Change device name = %s\r\n", devName);
}

void start_advertisement(uint8_t adv_handle, uint16_t adv_period)
{
  sl_status_t sc;

  /***************************************************************************//**
   * Generate data for advertising
   ******************************************************************************/
  sc = sl_bt_legacy_advertiser_generate_data(adv_handle,
                                             sl_bt_advertiser_general_discoverable);
  app_assert_status(sc);

  /***************************************************************************//**
   * Set advertising interval to ADV_PERIOD_MS
   ******************************************************************************/
  sc = sl_bt_advertiser_set_timing(
          adv_handle,
          (adv_period*1.6), // min. adv. interval (milliseconds * 1.6)
          (adv_period*1.6), // max. adv. interval (milliseconds * 1.6)
          0,   // adv. duration
          0);  // max. num. adv. events
  app_assert_status(sc);

#ifdef __DEBUG_MSG__
  printf("[BLE]Set Advertisement period to %d(ms)\r\n", adv_period);
#endif

  /***************************************************************************//**
   * Start general advertising and enable connections
   ******************************************************************************/
  sc = sl_bt_legacy_advertiser_start(adv_handle,
                                     sl_bt_advertiser_connectable_scannable);
  app_assert_status(sc);

#ifdef __DEBUG_MSG__
  printf("[BLE]Start Advertisement\r\n");
#endif
}

void stop_advertisement(uint8_t adv_handle)
{
  sl_status_t sc;

  sc = sl_bt_advertiser_stop(adv_handle);

  if(sc != SL_STATUS_OK) {
    app_assert_status(sc);

#ifdef __DEBUG_MSG__
    printf("[BLE][E: 0x%04x] Failed to stop the advertising\r\n", (int)sc);
#endif
  }
}




/*****************************************************************************
 * @brief   AGMS Notification Status
 ******************************************************************************/
void set_OPC_R2_notification(uint8_t status)
{
  OPC_R2_notification_status = status;
}

uint8_t get_OPC_R2_notification(void)
{
    return OPC_R2_notification_status;
}


/*****************************************************************************
 * @brief Current Connection ID
 ******************************************************************************/
uint8_t get_server_connection_id(void)
{
  /* Return HID connection handle ID */
  return server_connection_id;
}

void set_server_connection_id(uint8_t connection_id)
{
    if(connection_id == INVALID_HANDLE) {
        set_OPC_R2_notification(INVALID_HANDLE);
    }

  /* Return SPP connection handle ID */
    server_connection_id = connection_id;
}


/*****************************************************************************
  *
  ******************************************************************************/
#if 0
void set_current_time(uint16_t year, uint8_t month, uint8_t month_day, uint8_t hour, uint8_t min, uint8_t sec)
{
  sl_sleeptimer_date_t set_date;

  // SET currrent time
  set_date.time_zone = 9;   // UTC/GMP +9
  set_date.year = year-1900;  //< Year, based on a 0 Epoch or a 1900 Epoch.
  set_date.month = month - 1;
  set_date.month_day = month_day;
  set_date.hour = hour;
  set_date.min = min;
  set_date.sec = sec;
//  set_date.day_of_week = DAY_WEDNESDAY;
//  set_date.day_of_year = 0;

  app_log_info("[TIME]Set current time to %04d/%02d/%02d %02d:%02d:%02d\r\n",
                          set_date.year+1900,
                          set_date.month+1,
                          set_date.month_day,
                          set_date.hour,
                          set_date.min,
                          set_date.sec);

  sl_sleeptimer_set_datetime(&set_date);
}

void get_current_date_time(void)
{
  sl_sleeptimer_date_t get_date;

  sl_sleeptimer_get_datetime(&get_date);

  current_date_time->year =((get_date.year+1900)-2000);
  current_date_time->month = get_date.month+1;
  current_date_time->day = get_date.month_day;
  current_date_time->hour = get_date.hour;
  current_date_time->min = get_date.min;
  current_date_time->sec = get_date.sec;


#if 1
  {
    char time_str[64];

    memset(time_str, 0, sizeof(time_str));
    // YYYYMMDDHHMMSS
    sprintf(time_str, "[%04d]%04d/%02d/%02d %02d:%02d:%02d\r\n", get_date.year,
            current_date_time->year,current_date_time->month, current_date_time->day,
            current_date_time->hour, current_date_time->min, current_date_time->sec);

    app_log_info("%s", time_str);
  }
#endif


}

#endif


