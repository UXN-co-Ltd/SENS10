/*
 * app_utility.h
 *
 *  Created on: 2022. 4. 4.
 *      Author: dooker
 */

#ifndef APP_UTILITY_H_
#define APP_UTILITY_H_

#include "app.h"

/** Indicates currently there is no active connection using this service. */
#define INVALID_HANDLE         0xFF

#define get_tick_count()  sl_sleeptimer_tick_to_ms(sl_sleeptimer_get_tick_count())

void delay_ms(uint16_t delaytime);
void sleep_ms(uint16_t time_ms);

/* Print boot message */
void bootMessage(struct sl_bt_evt_system_boot_s *bootevt);
/* Change BLe System ID */
void change_system_id(void);
/* Change BLE device name */
void change_device_name(void);
void start_advertisement(uint8_t adv_handle, uint16_t adv_period);
void stop_advertisement(uint8_t adv_handle);


uint8_t get_server_connection_id(void);
void set_server_connection_id(uint8_t connection_id);

void set_OPC_R2_notification(uint8_t status);
uint8_t get_OPC_R2_notification(void);


#endif /* APP_UTILITY_H_ */
