//
// Created by asus on 2026/3/17.
//

#include "app_imu.h"
#include "app_sensor.h"
#include "bsp_imu.h"
#include "PID.h"
#include "tim.h"
static uint8_t count = 0;

void app_imu_timer_callback(void) {
    if (++count >= 4) {
        count = 0;
        bsp_tempctrl_update();
    }
    fusion_update();
}
