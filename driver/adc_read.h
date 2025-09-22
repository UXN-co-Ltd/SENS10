/*
 * adc_read.h
 *
 *  Created on: 2022. 3. 27.
 *      Author: dooker
 */

#ifndef DRIVER_ADC_READ_H_
#define DRIVER_ADC_READ_H_

void HW_ADC_Init(void);
double get_adc_value(void);
void update_adc_result(void);
void run_adc_timer(void);
void get_battery(void);

void IADC_Polling_Init(void);
float read_batt_lvl_mV(void);

#endif /* DRIVER_ADC_READ_H_ */
