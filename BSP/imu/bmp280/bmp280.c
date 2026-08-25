//
// Created by asus on 2026/1/3.
//

#include "bmp280.h"
#include <math.h>
#include "cmsis_os2.h"
#include "i2c.h"

// BMP280 硬件IIC 句柄
static const I2C_HandleTypeDef *bmp280_iic = &hi2c2;
static const uint32_t time_out = 10;
// BMP280 配置参数
static const BMP280_Config_TypeDef bmp280_config = {
    .BMP280_T_OverSamp = BMP280_OVERSAMP_2X,
    .BMP280_P_OverSamp = BMP280_OVERSAMP_2X,
    .BMP280_StandbyTime = BMP280_STANDBY_TIME_0_5MS,
    .BMP280_Filter = BMP280_FILTER_8X,
    .BMP280_PowerMode = BMP280_NORMAL_MODE,
    .BMP280_3SPI = BMP280_3SPI_DISABLE,
};


static BMP280_Calib_TypeDef bmp280_calib_data;

static uint16_t bmp280_read_uint16(uint8_t reg) {
    uint8_t buffer[2];
    HAL_I2C_Mem_Read((I2C_HandleTypeDef *)bmp280_iic, BMP280_I2C_READ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buffer, 2, time_out);
    return ((uint16_t) ((uint16_t)buffer[1] << 8 | (uint16_t)buffer[0]));
}

static int16_t bmp280_read_int16(uint8_t reg) {
    uint8_t buffer[2];
    HAL_I2C_Mem_Read((I2C_HandleTypeDef *)bmp280_iic, BMP280_I2C_READ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buffer, 2, time_out);
    return ((int16_t) ((int16_t)buffer[1] << 8 | (int16_t)buffer[0]));
}

static void bmp280_read_calib_data(void) {
    bmp280_calib_data.dig_T1 = bmp280_read_uint16(BMP280_REG_DIG_T1);
    bmp280_calib_data.dig_T2 = bmp280_read_int16(BMP280_REG_DIG_T2);
    bmp280_calib_data.dig_T3 = bmp280_read_int16(BMP280_REG_DIG_T3);
    bmp280_calib_data.dig_P1 = bmp280_read_uint16(BMP280_REG_DIG_P1);
    bmp280_calib_data.dig_P2 = bmp280_read_int16(BMP280_REG_DIG_P2);
    bmp280_calib_data.dig_P3 = bmp280_read_int16(BMP280_REG_DIG_P3);
    bmp280_calib_data.dig_P4 = bmp280_read_int16(BMP280_REG_DIG_P4);
    bmp280_calib_data.dig_P5 = bmp280_read_int16(BMP280_REG_DIG_P5);
    bmp280_calib_data.dig_P6 = bmp280_read_int16(BMP280_REG_DIG_P6);
    bmp280_calib_data.dig_P7 = bmp280_read_int16(BMP280_REG_DIG_P7);
    bmp280_calib_data.dig_P8 = bmp280_read_int16(BMP280_REG_DIG_P8);
    bmp280_calib_data.dig_P9 = bmp280_read_int16(BMP280_REG_DIG_P9);
}

static uint8_t bmp280_ReadID(void) {
    uint8_t chip_id = 0;
    HAL_I2C_Mem_Read((I2C_HandleTypeDef *)bmp280_iic, BMP280_I2C_READ_ADDR, BMP280_REG_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &chip_id, 1,
                     time_out);
    return chip_id;
}

/**
 * @brief 初始化BMP280传感器
 * @return BMP280_status_t 返回初始化状态
 */
BMP280_status_t bmp280_Init(void) {
    BMP280_status_t status = BMP280_NO_ERR;
    uint8_t ctrl_meas, config;
    status = bmp280_reset();
    if (bmp280_ReadID() != BMP280_DEV_ID) {
        return BMP280_INIT_ERR;
    }
    bmp280_read_calib_data();
    ctrl_meas = (bmp280_config.BMP280_T_OverSamp << 5) |
                (bmp280_config.BMP280_P_OverSamp << 2) |
                (bmp280_config.BMP280_PowerMode);
    config = (bmp280_config.BMP280_StandbyTime << 5) |
             (bmp280_config.BMP280_Filter << 2) |
             (bmp280_config.BMP280_3SPI);
    if (HAL_I2C_Mem_Write((I2C_HandleTypeDef *)bmp280_iic, BMP280_I2C_WRITE_ADDR, BMP280_REG_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1,
                          time_out) != HAL_OK) {
        return BMP280_WRITE_ERR;
    }
    if (HAL_I2C_Mem_Write((I2C_HandleTypeDef *)bmp280_iic, BMP280_I2C_WRITE_ADDR, BMP280_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &config, 1,
                          time_out) != HAL_OK) {
        return BMP280_WRITE_ERR;
    }
    return status;
}

/**
 * @brief 复位BMP280传感器
 * @return BMP280_status_t 复位操作的状态结果
 */
BMP280_status_t bmp280_reset(void) {
    uint8_t cmd = 0xB6;
    BMP280_status_t status =
            HAL_I2C_Mem_Write((I2C_HandleTypeDef *)bmp280_iic, BMP280_I2C_WRITE_ADDR, BMP280_REG_RESET, I2C_MEMADD_SIZE_8BIT, &cmd, 1,
                              time_out)
            == HAL_OK
                ? BMP280_NO_ERR
                : BMP280_WRITE_ERR;
#ifdef BMP280_USE_FREERTOS
    osDelay(10);
#else
    HAL_Delay(10);
#endif
    return status;
}


static int32_t bmp280_GetRawTemperature(void) {
    uint8_t buffer[3];
    HAL_I2C_Mem_Read((I2C_HandleTypeDef *)bmp280_iic, BMP280_I2C_READ_ADDR, BMP280_REG_TEMP_MSB, I2C_MEMADD_SIZE_8BIT, buffer, 3,
                     time_out);
    return (int32_t)(((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((uint32_t)buffer[2] >> 4));
}


static int32_t bmp280_GetRawPressure(void) {
    uint8_t buffer[3];
    HAL_I2C_Mem_Read((I2C_HandleTypeDef *)bmp280_iic, BMP280_I2C_READ_ADDR, BMP280_REG_PRESS_MSB, I2C_MEMADD_SIZE_8BIT, buffer, 3,
                     time_out);
    return (int32_t)(((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((uint32_t)buffer[2] >> 4));
}


/**
 * @brief 从BMP280传感器获取温度、压力和海拔数据
 * @param data 指向BMP280_data_t结构体的指针
 * @return BMP280_status_t 返回操作状态
 */
BMP280_status_t bmp280_GetData(BMP280_data_t *data) {
    static int32_t t_fine;
    int32_t raw_temp = bmp280_GetRawTemperature();
    int32_t var1_T = ((((raw_temp >> 3) - ((int32_t)bmp280_calib_data.dig_T1 << 1))) * ((int32_t)bmp280_calib_data.dig_T2)) >> 11;
    int32_t var2_T = (((((raw_temp >> 4) - ((int32_t)bmp280_calib_data.dig_T1)) *
              ((raw_temp >> 4) - ((int32_t)bmp280_calib_data.dig_T1))) >> 12) *
              ((int32_t)bmp280_calib_data.dig_T3)) >> 14;
    t_fine = var1_T + var2_T;
    int32_t T = (t_fine * 5 + 128) >> 8;
    data->temperature = (float)T /100.0f;

    int32_t raw_press = bmp280_GetRawPressure();
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)bmp280_calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280_calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_calib_data.dig_P3) >> 8) +
           ((var1 * (int64_t)bmp280_calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_calib_data.dig_P1) >> 33;
    if (var1 == 0) {
        data->temperature = 0.0f;
        data->pressure = 0.0f;
        data->altitude = 0.0f;
        return BMP280_READ_ERR;
    }
    int64_t P = 1048576 - raw_press;
    P = (((P << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_calib_data.dig_P9) * (P >> 13) * (P >> 13)) >> 25;
    var2 = (((int64_t)bmp280_calib_data.dig_P8) * P) >> 19;
    P = ((P + var1 + var2) >> 8) + (((int64_t)bmp280_calib_data.dig_P7) << 4);
    data->pressure = (float)P / 25600.0f;

    float altitude = 44330.0f * (1.0f - powf(((float)P / 25600.0f) / 1013.25f, 1.0f / 5.255f));
    data->altitude = altitude;

    return BMP280_NO_ERR;
}

