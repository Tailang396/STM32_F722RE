//
// Created by asus on 2026/1/1.
//

#include "sht40.h"
#include "bsp_def.h"
#include "i2c.h"
#include "alg_crc.h"


/**
 * @brief SHT40温湿度传感器I2C句柄指针
 * 该指针指向SHT40传感器所使用的I2C接口句柄，用于I2C通信操作
 */
static const I2C_HandleTypeDef *sht40_i2c = &hi2c3;
static const uint32_t time_out = 10;


static uint16_t sht40_GetPrecisionDelay(SHT40_precision_e precision) {
    switch (precision) {
        case SHT40_PRECISION_HIGH:
            return 10;
        case SHT40_PRECISION_MED:
            return 6;
        case SHT40_PRECISION_LOW:
            return 3;
        default:
            return 0;
    }
}


/**
 * @brief SHT40温湿度传感器初始化函数，耗时1ms
 * 该函数通过发送软复位命令来初始化SHT40传感器，并等待复位完成
 * @return SHT40_Status_t 初始化状态
 */
SHT40_status_t sht40_Init(void) {
    uint8_t cmd = SHT40_CMD_SOFT_RESET;
    SHT40_status_t status =
        HAL_I2C_Master_Transmit((I2C_HandleTypeDef *)sht40_i2c, SHT40_I2C_WRITE_ADDR, &cmd, 1, time_out)
        == HAL_OK ? SHT40_NO_ERR : SHT40_INIT_ERR;
#ifdef SHT40_USE_FREERTOS
    osDelay(1);
#else
    HAL_Delay(1);
#endif
    return status;
}


/**
 * @brief 读取SHT40传感器数据
 * 高精度测量，耗时10ms；中等精度测量，耗时6ms；低精度测量，耗时3ms
 * @param data 指向SHT40_Data_t结构体的指针，用于存储转换后的温度和湿度数据
 * @param precision 测量精度等级，决定命令字节和等待时间
 * @return SHT40_status_t 返回操作状态，包括成功或各种错误类型
 */
SHT40_status_t sht40_ReadData(SHT40_Data_t *data, SHT40_precision_e precision) {
    uint8_t cmd = precision;
    uint16_t time = sht40_GetPrecisionDelay(precision);
    uint8_t rx_data[6];
    SHT40_status_t status = SHT40_NO_ERR;
    if(HAL_I2C_Master_Transmit((I2C_HandleTypeDef *)sht40_i2c, SHT40_I2C_WRITE_ADDR, &cmd, 1, time_out) != HAL_OK) {
        status = SHT40_WRITE_ERR;
    }

#ifdef SHT40_USE_FREERTOS
    osDelay(time);
#else
    HAL_Delay(time);
#endif
    if (HAL_I2C_Master_Receive((I2C_HandleTypeDef *)sht40_i2c, SHT40_I2C_READ_ADDR, rx_data, 6, time_out) != HAL_OK) {
        status = SHT40_READ_ERR;
    }

#ifdef SHT40_USE_CRC
    if (crc8_verify(&rx_data[0], 2, 0xff, CRC8_0x31, rx_data[2]) != CRC_NO_ERR) {
        return SHT40_CRC_ERR;
    }
    if (crc8_verify(&rx_data[3], 2, 0xff, CRC8_0x31, rx_data[5]) != CRC_NO_ERR) {
        return SHT40_CRC_ERR;
    }
#endif

    uint16_t t_temp = (rx_data[0] << 8) | rx_data[1];
    data->temperature = -45.0f + 175.0f * (float)t_temp / 65535.0;
    uint16_t rh_temp = (rx_data[3] << 8) | rx_data[4];
    data->humidity = -6.0f + 125.0f * (float)rh_temp / 65535.0f;

    if (data->humidity > 100.0) data->humidity = 100.0f;
    if (data->humidity < 0.0) data->humidity = 0.0f;
    return status;
}

/**
 * @brief 读取SHT40传感器加热后读取数据，除霜用，正常测量不需要加热，不可循环加热测量
 * 1S加热，耗时3S；0.1S加热，耗时2S；总耗时=加热时间+测量时间
 * @param data 指向SHT40数据结构体的指针，用于存储读取到的温湿度数据
 * @param precision 精度设置枚举，指定读取数据的精度级别
 * @param heat 加热设置枚举，指定加热功率和加热时间
 * @return SHT40_status_t 返回操作状态
 */
SHT40_status_t sht40_ReadData_heat(SHT40_Data_t *data, SHT40_precision_e precision, SHT40_heat_e heat) {
    uint8_t heat_cmd = heat;
    uint16_t heat_time = 0;
    SHT40_status_t status = SHT40_NO_ERR;
    if (heat == SHT40_HEAT_20MW_1S || heat == SHT40_HEAT_220MW_1S || heat == SHT40_HEAT_110MW_1S)
        heat_time = 3000;
    else if (heat == SHT40_HEAT_20MW_100MS || heat == SHT40_HEAT_220MW_100MS || heat == SHT40_HEAT_110MW_100MS)
        heat_time = 2000;
    if(HAL_I2C_Master_Transmit((I2C_HandleTypeDef *)sht40_i2c, SHT40_I2C_WRITE_ADDR, &heat_cmd, 1, time_out) != HAL_OK) {
        status = SHT40_WRITE_ERR;
    }
#ifdef SHT40_USE_FREERTOS
    osDelay(heat_time);
#else
    HAL_Delay(heat_time);
#endif
    if (sht40_ReadData(data, precision) != SHT40_NO_ERR)
        status = SHT40_READ_ERR;
    return status;
}
