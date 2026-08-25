//
// Created by asus on 2026/3/21.
//

#ifndef F722RE_APP_SENSOR_H
#define F722RE_APP_SENSOR_H
#include "sht40.h"

float app_sensor_GetTMP117Data(void);
unsigned char app_sensor_GetTMP117OSFlag(void);
float app_sensor_GetCO_V(void);
float app_sensor_GetNG_V(void);
float app_sensor_GetLPG_V(void);
float app_sensor_GetAlarmVoltage(void);
SHT40_Data_t *app_sensor_GetSHT40Data(void);

#endif //F722RE_APP_SENSOR_H