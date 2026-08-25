//
// Created by asus on 2026/4/8.
//

#ifndef F722RE_APP_COMM_H
#define F722RE_APP_COMM_H
#include "main.h"

typedef struct {
    uint8_t imu_flag;
    uint8_t gps_flag;
    uint8_t sht40_flag;
    uint8_t bat_flag;
    uint8_t gas_flag;
} comm_send_flag_t;

typedef enum {
    HEX_MODE,
    ASCII_MODE
} comm_mode_e;

typedef struct {
    float co_v;
    float ng_v;
    float lpg_v;
    float alarm_v;
} GasVoltage_t;

#endif //F722RE_APP_COMM_H
