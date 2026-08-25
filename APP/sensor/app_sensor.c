//
// Created by asus on 2026/3/21.
//

#include "app_sensor.h"
#include "bsp_adc.h"
#include "bsp_def.h"
#include "bsp_imu.h"
#include "sht40.h"
#include "tmp117.h"

#define ALARM_VOLTAGE 2.0f

static float mq7_CO_v = 0.0f;
static float mq4_NG_v = 0.0f;
static float mq5_LPG_v = 0.0f;
static float tmp117_temp = 0.0f;
static uint8_t tmp117_OS_finsh = 0;
SHT40_Data_t sht40_data;

void app_sensor(void* argument) {
    int8_t status = 0;
    tmp117_OS_finsh = 1;
    status = sht40_Init();
    BSP_ASSERT(status == 0);
    bsp_adc_init();
    imu_data_t* imu_data = bsp_imu_GetData();
    for (;;) {
        sht40_ReadData(&sht40_data, SHT40_PRECISION_HIGH);
        osDelay(3);
        tmp117_temp = tmp117_GetData();
        osDelay(3);
        mq7_CO_v = bsp_get_mq7_voltage();
        mq4_NG_v = bsp_get_mq4_voltage();
        mq5_LPG_v = bsp_get_mq5_voltage();
        // bsp_uart_printf(E_UART_DEBUG, "457: %f, %f, %f\r\n", mq4_NG_v, mq5_LPG_v, mq7_CO_v);
        // bsp_uart_printf(E_UART_DEBUG,"sh: %.2f, %.2f, tmp: %.2f\r\n", sht40_data.humidity, sht40_data.temperature, tmp117_temp);
    }
}


/***************API************************************************************/

float app_sensor_GetTMP117Data(void) {
    return tmp117_temp;
}

unsigned char app_sensor_GetTMP117OSFlag(void) {
    return tmp117_OS_finsh;
}

float app_sensor_GetCO_V(void) {
    return mq7_CO_v;
}

float app_sensor_GetNG_V(void) {
    return mq4_NG_v;
}

float app_sensor_GetLPG_V(void) {
    return mq5_LPG_v;
}

float app_sensor_GetAlarmVoltage(void) {
    return ALARM_VOLTAGE;
}

SHT40_Data_t *app_sensor_GetSHT40Data(void) {
    return &sht40_data;
}