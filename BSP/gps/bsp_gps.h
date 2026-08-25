#ifndef BSP_GPS_H
#define BSP_GPS_H

#include "main.h"

typedef struct
{
    int      time_hms;      // 121919
    float    latitude;      // 十进制度
    float    longitude;     // 十进制度
    float    altitude;      // m
    uint8_t  satellites;    // 卫星数
    uint8_t  valid;         // 0无效 1有效
    float    hdop;          // 置信度
} __attribute__((packed)) gps_data_t;

uint8_t gps_check_sum_ok_len(const uint8_t* buf, uint16_t len);
int gps_parse_gga(const uint8_t* buf);
gps_data_t* bsp_gps_get_data(void);



#endif
