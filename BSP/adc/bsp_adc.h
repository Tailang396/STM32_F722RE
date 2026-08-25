//
// Created by asus on 2026/4/4.
//

#ifndef F722RE_BSP_ADC_H
#define F722RE_BSP_ADC_H
#include "main.h"

void bsp_adc_init(void);
float bsp_get_bat_voltage(void);
float bsp_get_bat_soc(void);
float bsp_get_mq7_voltage(void);
float bsp_get_mq4_voltage(void);
float bsp_get_mq5_voltage(void);

#endif //F722RE_BSP_ADC_H