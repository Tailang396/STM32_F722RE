//
// Created by asus on 2026/3/13.
//

#include "tmp117.h"
#include "i2c.h"
#include "main.h"
#include "bsp_def.h"

// BMP280 硬件IIC 句柄
static const I2C_HandleTypeDef* tmp117_iic = &hi2c3;
static const uint32_t time_out = 10;

// 默认阈值
#define TMP117_HIGH_LIMIT 50.0f
#define TMP117_LOW_LIMIT 30.0f

/**
 * @brief TMP117 传感器配置结构体
 */
static const TMP117_Config_TypeDef tmp117_config = {
    .mode = TMP117_MODE_CONTINUOUS,
    .data_rate = TMP117_DATA_RATE_64HZ,
    .avg_samples = TMP117_NO_AVG,
    .alert_pin_mode = TMP117_ALERT_PIN_ALERT_FLAGS,
    .alert_mode = TMP117_ALERT_MODE_THERMOSTAT,
    .alert_polarity = TMP117_ALERT_POLARITY_LOW,
};


/**
 * @brief 从 TMP117 传感器读取寄存器数据
 * @param reg 要读取的寄存器地址（8 位）
 * @return uint16_t 读取到的 16 位寄存器值
 *         高字节在前，低字节在后
 */
static uint16_t TMP117_ReadReg(uint8_t reg) {
    uint8_t buf[2];
    HAL_I2C_Mem_Read((I2C_HandleTypeDef*)tmp117_iic,
                     TMP117_I2C_READ_ADDR,
                     reg,
                     I2C_MEMADD_SIZE_8BIT, buf,
                     2,
                     time_out
    );
    return ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
}


/**
 * @brief 向 TMP117 传感器写入寄存器数据
 * @param reg 要写入的寄存器地址（8 位）
 * @param value 要写入的 16 位数据值
 *              高 8 位和低 8 位会自动拆分并通过 I2C 发送
 * @return TMP117_Status_t 操作状态
 */
static TMP117_Status_t TMP117_WriteReg(uint8_t reg, uint16_t value) {
    uint8_t buf[2] = {(value >> 8) & 0xFF, value & 0xFF};
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(
        (I2C_HandleTypeDef*)tmp117_iic,
        TMP117_I2C_WRITE_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        2,
        time_out
    );
    if (ret != HAL_OK) {
        return TMP117_WRITE_ERR;
    }
    return TMP117_NO_ERR;
}

static uint16_t TMP117_ReadID(void) {
    return TMP117_ReadReg(TMP117_REG_DEVICE_ID);
}

/**
 * @brief 初始化 TMP117 温度传感器，延迟5ms
 *        验证设备 ID，执行软件复位，并根据全局配置结构体设置传感器工作参数
 * @return TMP117_Status_t 初始化状态
 */
TMP117_Status_t tmp117_Init(void) {
    if (TMP117_ReadID() != TMP117_DEV_ID) {
        return TMP117_INIT_ERR;
    }
    if (tmp117_reset() != TMP117_NO_ERR) {
        return TMP117_INIT_ERR;
    }
    if (tmp117_SetHighLimit(TMP117_HIGH_LIMIT) != TMP117_NO_ERR) {
        return TMP117_WRITE_ERR;
    }
    if (tmp117_SetLowLimit(TMP117_LOW_LIMIT) != TMP117_NO_ERR) {
        return TMP117_WRITE_ERR;
    }
    uint16_t config = 0x0000;
    config = (tmp117_config.mode << 10) |
        (tmp117_config.data_rate << 7) |
        (tmp117_config.avg_samples << 5) |
        (tmp117_config.alert_pin_mode << 2) |
        (tmp117_config.alert_mode << 4) |
        (tmp117_config.alert_polarity << 3);
    return TMP117_WriteReg(TMP117_REG_CONFIG, config);
}

/**
 * @brief 软件复位 TMP117 传感器
 *        读取配置寄存器并置位复位标志，等待复位完成,延迟5ms
 * @return TMP117_Status_t 复位操作状态
 */
TMP117_Status_t tmp117_reset(void) {
    uint16_t config_reg = TMP117_ReadReg(TMP117_REG_CONFIG);
    config_reg |= (1 << 1);
    if (TMP117_WriteReg(TMP117_REG_CONFIG, config_reg) != TMP117_NO_ERR) {
        return TMP117_WRITE_ERR;
    };
#ifdef TMP117_USE_FREERTOS
    osDelay(5);
#else
    HAL_Delay(5);
#endif
    return TMP117_NO_ERR;
}

/**
 * @brief 读取 TMP117 传感器的温度数据
 *        从温度寄存器读取原始数据并转换为摄氏度值
 * @return float 温度值（单位：摄氏度）
 */
float tmp117_GetData(void) {
    int16_t raw = (int16_t)TMP117_ReadReg(TMP117_REG_TEMP);
    return (float)raw * 0.0078125f;
}

/**
 * @brief 获取 TMP117 传感器的状态信息
 *        读取配置寄存器并解析各位标志，填充状态结构体
 * @param status 指向状态结构体的指针，用于存储解析后的状态信息
 *               - high_Alert: 高温告警标志
 *               - low_Alert: 低温告警标志
 *               - data_ready: 数据就绪标志
 *               - eeprom_Busy: EEPROM 忙标志
 * @return TMP117_Status_t 操作状态
 */
TMP117_Status_t tmp117_GetStatus(TMP117_Status_TypeDef* status) {
    uint16_t config_reg = 0xffff;
    config_reg = TMP117_ReadReg(TMP117_REG_CONFIG);
    if (config_reg == 0xffff) {
        return TMP117_READ_ERR;
    }
    status->high_Alert = (config_reg >> 15) & 0x01; // Bit 15: HIGH_Alert
    status->low_Alert = (config_reg >> 14) & 0x01; // Bit 14: LOW_Alert
    status->data_ready = (config_reg >> 13) & 0x01; // Bit 13: Data_Ready
    status->eeprom_Busy = (config_reg >> 12) & 0x01; // Bit 12: EEPROM_Busy
    return TMP117_NO_ERR;
}

/**
 * @brief 设置 TMP117 的高温告警阈值
 *        将摄氏度温度值转换为寄存器值并写入高温阈值寄存器
 * @param Hlimit 高温告警阈值（单位：摄氏度）
 * @return TMP117_Status_t
 */
TMP117_Status_t tmp117_SetHighLimit(float Hlimit) {
    int16_t value = (int16_t)(Hlimit / 0.0078125f);
    return TMP117_WriteReg(TMP117_REG_T_HIGH, (uint16_t)value);
}

/**
 * @brief 设置 TMP117 的低温告警阈值
 *        将摄氏度温度值转换为寄存器值并写入低温阈值寄存器
 * @param Llimit 低温告警阈值（单位：摄氏度）
 * @return TMP117_Status_t 写入操作状态
 */
TMP117_Status_t tmp117_SetLowLimit(float Llimit) {
    int16_t value = (int16_t)(Llimit / 0.0078125f);
    return TMP117_WriteReg(TMP117_REG_T_LOW, (uint16_t)value);
}
