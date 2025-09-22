/***************************************************************************//**
 * @file
 * @brief Application interface provided to main().
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

#ifndef APP_H
#define APP_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <app_log.h>
#include <app_assert.h>
#include <gatt_db.h>

#include <em_chip.h>
#include <em_cmu.h>
#include <em_gpio.h>
#include <em_common.h>
#include <em_iadc.h>
#include <em_wdog.h>
#include <em_rmu.h>

#include <sl_bluetooth.h>
#include <sl_sleeptimer.h>
#include <sl_status.h>
#include <sl_iostream.h>

//#include <tempdrv.h>

#include "nvm3_default.h"
#include "nvm3_default_config.h"

#include "spidrv_master.h"
#include "app_utility.h"
#include "app_scan.h"

#include "adc_read.h"

/**************************************************************************//**
 * User define
 *****************************************************************************/
#define __DEBUG_MSG__

#define API_TRUE    1
#define API_FALSE   0

#define POWER_ON    1
#define POWER_OFF   0


#define EXT_SENS_DATA_SEND                    0x00000001



/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void);

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void);


sl_status_t send_tx_data(uint8_t *data_send, size_t data_send_len);

#endif // APP_H
