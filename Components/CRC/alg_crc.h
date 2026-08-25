//
// Created by guizi on 2025/6/11.
//

#ifndef RM_FRAMEWORK_H723_ALG_CRC_H
#define RM_FRAMEWORK_H723_ALG_CRC_H

typedef enum {
    CRC_NO_ERR = 0,
    CRC_VERIFY_ERR = 1,
} crc_status_t;

typedef enum {
    CRC8_0x31 = 0,
    CRC8_0x07 = 1,
    CRC8_0x5e = 2,
} crc8_polynomial_e;


uint16_t crc16_calc(const uint8_t *data, uint32_t len);
crc_status_t crc16_verify(const uint8_t *buf, uint32_t len, uint16_t crc);
uint8_t crc8_calc(const uint8_t *buf, uint32_t len, uint8_t InitCRC, crc8_polynomial_e poly);
crc_status_t crc8_verify(const uint8_t *buf, uint32_t len, uint8_t InitCRC, crc8_polynomial_e poly, uint8_t crc);

#endif //RM_FRAMEWORK_H723_ALG_CRC_H
