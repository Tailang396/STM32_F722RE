#include "bsp_gps.h"
#include <string.h>
#include <stdio.h>


static gps_data_t s_gps_data;

/* ==================== 内部工具函数 ==================== */

static float gps_nmea_to_degree(float raw) {
    int degree = (int)(raw / 100.0f);
    float minute = raw - (float)degree * 100.0f;
    return (float)degree + minute / 60.0f;
}

static uint8_t gps_hex_to_u8(uint8_t c) {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0xFF;
}


uint8_t gps_check_sum_ok_len(const uint8_t* buf, uint16_t len) {
    uint8_t calc = 0;
    uint8_t recv = 0;
    uint16_t i = 0;
    int16_t star_pos = -1;
    uint8_t hi, lo;
    if (buf == 0) return 0;
    if (len < 4) return 0; /* 至少要像 "$*00" 这样 */
    if (buf[0] != '$') return 0;
    for (i = 1; i < len; i++) {
        if (buf[i] == '*') {
            star_pos = (int16_t)i;
            break;
        }
        /* * 前面不应该出现行结束，否则说明帧不完整 */
        if (buf[i] == '\r' || buf[i] == '\n') {
            return 0;
        }
        calc ^= (uint8_t)buf[i];
    }
    if (star_pos < 0) return 0;
    /* * 后面至少还要有两个十六进制字符 */
    if ((uint16_t)(star_pos + 2) >= len) return 0;
    hi = gps_hex_to_u8(buf[star_pos + 1]);
    lo = gps_hex_to_u8(buf[star_pos + 2]);
    if (hi == 0xFF || lo == 0xFF) return 0;
    recv = (uint8_t)((hi << 4) | lo);
    return (calc == recv) ? 1 : 0;
}

int gps_parse_gga(const uint8_t* buf) {
    int fix = 0;
    if (buf == 0) {
        return 0;
    }
    /* 仅接受 GNGGA / GPGGA */
    if (!((buf[1] == 'G') && (buf[3] == 'G') && (buf[4] == 'G') && (buf[5] == 'A'))) {
        return 0;
    }
    if (buf[7] == ',') {
        if (sscanf((const char*)buf, "$%*[^,],,,,,,%d", &fix) != 1) {
            return 0;
        }
    }
    else {
        if (buf[18] == ',') {
            if (sscanf((const char*)buf, "$%*[^,],%*f,,,,,%d", &fix) != 1) {
                return 0;
            }
        }
        else {
            if (sscanf((const char*)buf, "$%*[^,],%*f,%*f,%*c,%*f,%*c,%d", &fix) != 1) {
                return 0;
            }
        }
    }
    /* 无定位 */
    if (fix == 0) {
        int sats = 0;
        float hdop = 0.0f;
        float utc_time = 0.0f;
        memset(&s_gps_data, 0, sizeof(s_gps_data));
        if (buf[7] == ',') {
            if (sscanf((const char*)buf, "$%*[^,],,,,,,%*d,%d,%f", &sats, &hdop) >= 1) {
                s_gps_data.satellites = (uint8_t)sats;
                s_gps_data.hdop = hdop;
                s_gps_data.time_hms = 0;
            }
        }
        else {
            if (sscanf((const char*)buf, "$%*[^,],%f,,,,,%*d,%d,%f", &utc_time, &sats, &hdop) >= 1) {
                s_gps_data.satellites = (uint8_t)sats;
                s_gps_data.hdop = hdop;
                s_gps_data.time_hms = (int)utc_time;
            }
        }
        s_gps_data.valid = 0;
        return 1;
    }
    else {
        float utc_time = 0.0f;
        float lat_nmea = 0.0f;
        float lon_nmea = 0.0f;
        char lat_dir = 0;
        char lon_dir = 0;
        int sats = 0;
        float hdop = 0.0f;
        float altitude = 0.0f;
        int ret;

        memset(&s_gps_data, 0, sizeof(s_gps_data));
        ret = sscanf((const char*)buf,
                     "$%*[^,],%f,%f,%c,%f,%c,%d,%d,%f,%f",
                     &utc_time,
                     &lat_nmea,
                     &lat_dir,
                     &lon_nmea,
                     &lon_dir,
                     &fix,
                     &sats,
                     &hdop,
                     &altitude);

        if (ret < 9) {
            return 0;
        }
        s_gps_data.time_hms = (int)utc_time;
        s_gps_data.latitude = gps_nmea_to_degree(lat_nmea);
        s_gps_data.longitude = gps_nmea_to_degree(lon_nmea);
        s_gps_data.altitude = altitude;
        s_gps_data.satellites = (uint8_t)sats;
        s_gps_data.valid = (fix > 0) ? 1 : 0;
        s_gps_data.hdop = hdop;
        if (lat_dir == 'S') {
            s_gps_data.latitude = -s_gps_data.latitude;
        }
        if (lon_dir == 'W') {
            s_gps_data.longitude = -s_gps_data.longitude;
        }
        return 1;
    }
}


gps_data_t* bsp_gps_get_data(void) {
    return &s_gps_data;
}
