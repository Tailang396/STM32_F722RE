//
// Created by asus on 2026/4/8.
//

#include "app_gps.h"
#include <string.h>
#include "bsp_def.h"
#include "bsp_gps.h"

#define RX_BUF_LEN 128
static uint8_t rx_flag = 0;
static uint16_t rx_len = 0;
static uint8_t rx_buf[RX_BUF_LEN];

void gps_callback(bsp_uart_e e, uint8_t* s, uint16_t l);

void app_gps(void* argument) {
    bsp_uart_init(E_UART_GPS, &huart3);
    bsp_uart_set_callback(E_UART_GPS, gps_callback);
    // gps_data_t *gps_data = bsp_gps_get_data();
    for (;;) {
        if (rx_flag) {
            if (gps_check_sum_ok_len(rx_buf, rx_len)) {
                gps_parse_gga(rx_buf);
                // bsp_uart_send(E_UART_LORA, rx_buf , rx_len);
                // bsp_uart_printf(E_UART_LORA, "T:%d,E:%f,N:%f,H:%.2f,F:%.1f\r\n", gps_data->time_hms, gps_data->longitude, gps_data->latitude, gps_data->altitude, gps_data->hdop);
            }
            memset(rx_buf, 0, RX_BUF_LEN);
            rx_flag = 0;
        }
        osDelay(250);
    }
}

void gps_callback(bsp_uart_e e, uint8_t* s, uint16_t l) {
    if (l < RX_BUF_LEN) {
        memcpy(rx_buf, s, l);
        rx_buf[l] = '\0';
        rx_len = l + 1;
        rx_flag = 1;
    }
}
