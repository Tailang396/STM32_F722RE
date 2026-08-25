//
// Created by asus on 2026/4/8.
//

#include "app_comm.h"

#include <string.h>

#include "app_sensor.h"
#include "bsp_def.h"
#include "bsp_gps.h"
#include "bsp_imu.h"
#include "bsp_flash.h"
#include "alg_crc.h"
#include "bsp_adc.h"

/* 数据包地址 */
/* IMU 0x81
 * GPS 0x82
 * SHT40 0x83
 * BAT 0x84
 * GAS 0x85
 */

/* HEX 数据构成 */
/* DEV_ID DATA_ID DATA_LEN DATA DATA_CRC */
/*  1        1       2       n     2     */
/* 电池数据 */
/* DEV_ID DATA_ID DATA_LEN VOL SOC DATA_CRC */
/*   1        1       2     4   1    2      */
/* 小端模式发送 */
const uint8_t DEV_ID = 0xF1;

const uint8_t IMU_ID = 0x81;
const uint8_t GPS_ID = 0x82;
const uint8_t SHT40_ID = 0x83;
const uint8_t BAT_ID = 0x84;
const uint8_t GAS_ID = 0x85;


void lora_callback(bsp_uart_e e, uint8_t* s, uint16_t l);

uint16_t data_len;
uint16_t data_crc16;
uint8_t data_tx[128];

static uint8_t rx_data[32];
static comm_send_flag_t send_flag = {0};
static comm_mode_e send_mode = ASCII_MODE;

/* 气体传感器数据  */
GasVoltage_t gas_data;
/* 电池信息 */
static uint8_t batLevel = 0;
static float bat_voltage = 0.0f;

void app_comm(void* argument) {
    bsp_uart_init(E_UART_LORA, &huart5);
    bsp_uart_set_callback(E_UART_LORA, lora_callback);

    gps_data_t* gps_data = bsp_gps_get_data();
    imu_data_t* imu_data = bsp_imu_GetData();
    SHT40_Data_t* sht40_data = app_sensor_GetSHT40Data();

    bsp_flash_read("send_flag", &send_flag, sizeof(send_flag));
    bsp_flash_read("send_mode", &send_mode, sizeof(send_mode));

    for (;;) {
        if (send_flag.imu_flag) {
            if (send_mode == ASCII_MODE) {
                bsp_uart_printf(E_UART_LORA, "imu:%.2f,%.2f,%.2f,%.1f\r\n", imu_data->angle.roll, imu_data->angle.pitch,
                                imu_data->angle.yaw, imu_data->temp.bsp_temp);
            }
            else if (send_mode == HEX_MODE) {
                data_len = sizeof(imu_data_t);
                memcpy(data_tx, imu_data, data_len);
                data_crc16 = crc16_calc((uint8_t*)data_tx, data_len);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&DEV_ID, 1);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&IMU_ID, 1);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&data_len, 2);
                bsp_uart_send(E_UART_LORA, (uint8_t*)data_tx, data_len);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&data_crc16, 2);
            }
        }
        if (send_flag.gps_flag) {
            if (send_mode == ASCII_MODE) {
                bsp_uart_printf(E_UART_LORA, "T:%2d:%2d:%2d,%f,%f,%.2f,%.1f,%2d\r\n", (gps_data->time_hms / 10000) + 8,
                                (gps_data->time_hms / 100) % 100, gps_data->time_hms % 100, gps_data->longitude,
                                gps_data->latitude, gps_data->altitude, gps_data->hdop, gps_data->satellites);
            }
            else if (send_mode == HEX_MODE) {
                data_len = sizeof(gps_data_t);
                memcpy(data_tx, gps_data, data_len);
                data_crc16 = crc16_calc((uint8_t*)data_tx, data_len);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&DEV_ID, 1);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&GPS_ID, 1);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&data_len, 2);
                bsp_uart_send(E_UART_LORA, (uint8_t*)data_tx, data_len);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&data_crc16, 2);
            }
        }
        if (send_flag.sht40_flag) {
            if (send_mode == ASCII_MODE) {
                bsp_uart_printf(E_UART_LORA, "SHT40:%.1f,%.1f%\r\n", sht40_data->temperature, sht40_data->humidity);
            }
            else if (send_mode == HEX_MODE) {
                data_len = sizeof(SHT40_Data_t);
                memcpy(data_tx, sht40_data, data_len);
                data_crc16 = crc16_calc((uint8_t*)data_tx, data_len);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&DEV_ID, 1);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&SHT40_ID, 1);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&data_len, 2);
                bsp_uart_send(E_UART_LORA, (uint8_t*)data_tx, data_len);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&data_crc16, 2);
            }
        }
        if (send_flag.gas_flag) {
            gas_data.co_v = app_sensor_GetCO_V();
            gas_data.ng_v = app_sensor_GetNG_V();
            gas_data.lpg_v = app_sensor_GetLPG_V();
            gas_data.alarm_v = app_sensor_GetAlarmVoltage();
            if (send_mode == ASCII_MODE) {
                bsp_uart_printf(E_UART_LORA, "CO:%.2f,NG:%.2f,LPG:%.2f,ALA:%.2f\r\n", gas_data.co_v, gas_data.ng_v,
                                gas_data.lpg_v, gas_data.alarm_v);
            }
            else if (send_mode == HEX_MODE) {
                data_len = sizeof(GasVoltage_t);
                memcpy(data_tx, &gas_data, data_len);
                data_crc16 = crc16_calc((uint8_t*)data_tx, data_len);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&DEV_ID, 1);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&GAS_ID, 1);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&data_len, 2);
                bsp_uart_send(E_UART_LORA, (uint8_t*)data_tx, data_len);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&data_crc16, 2);
            }
        }
        if (send_flag.bat_flag) {
            bat_voltage = bsp_get_bat_voltage();
            batLevel = (uint8_t)bsp_get_bat_soc();
            if (send_mode == ASCII_MODE) {
                bsp_uart_printf(E_UART_LORA, "BAT:%.2f,%d\r\n", bat_voltage, batLevel);
            }
            else if (send_mode == HEX_MODE) {
                uint8_t bat_buf[5];
                memcpy(&bat_buf[0], &bat_voltage, 4);
                bat_buf[4] = batLevel;
                data_len = 5;
                data_crc16 = crc16_calc(bat_buf, data_len);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&DEV_ID, 1);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&BAT_ID, 1);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&data_len, 2);
                bsp_uart_send(E_UART_LORA, bat_buf, data_len);
                bsp_uart_send(E_UART_LORA, (uint8_t*)&data_crc16, 2);
            }
        }
        osDelay(1000);
    }
}

void lora_callback(bsp_uart_e e, uint8_t* s, uint16_t l) {
    if (e == E_UART_LORA) {
        if (l < 32)
            if (s[0] == '*') {
                memset(rx_data, 0x00, 32);
                memcpy(rx_data, s, l);
                if (memcmp(rx_data, "*ASCII_MODE", 11) == 0)
                    send_mode = ASCII_MODE;
                else if (memcmp(rx_data, "*HEX_MODE", 9) == 0)
                    send_mode = HEX_MODE;
                else if (memcmp(rx_data, "*IMU_EN", 7) == 0)
                    send_flag.imu_flag = 1;
                else if (memcmp(rx_data, "*IMU_DIS", 8) == 0)
                    send_flag.imu_flag = 0;
                else if (memcmp(rx_data, "*GPS_EN", 7) == 0)
                    send_flag.gps_flag = 1;
                else if (memcmp(rx_data, "*GPS_DIS", 8) == 0)
                    send_flag.gps_flag = 0;
                else if (memcmp(rx_data, "*SHT40_EN", 9) == 0)
                    send_flag.sht40_flag = 1;
                else if (memcmp(rx_data, "*SHT40_DIS", 10) == 0)
                    send_flag.sht40_flag = 0;
                else if (memcmp(rx_data, "*BAT_EN", 7) == 0)
                    send_flag.bat_flag = 1;
                else if (memcmp(rx_data, "*BAT_DIS", 8) == 0)
                    send_flag.bat_flag = 0;
                else if (memcmp(rx_data, "*GAS_EN", 7) == 0)
                    send_flag.gas_flag = 1;
                else if (memcmp(rx_data, "*GAS_DIS", 8) == 0)
                    send_flag.gas_flag = 0;
                else if (memcmp(rx_data, "*SAVE_SET", 9) == 0) {
                    while (bsp_flash_write("send_flag", &send_flag, sizeof(send_flag)) != BSP_OK);
                    while (bsp_flash_write("send_mode", &send_mode, sizeof(send_mode)) != BSP_OK);
                    bsp_uart_printf(E_UART_LORA, "*SAVE_OK\r\n");
                }
            }
    }
}
