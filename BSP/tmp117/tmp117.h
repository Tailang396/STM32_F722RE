//
// Created by asus on 2026/3/13.
//

#ifndef TMP117_H
#define TMP117_H

#include "main.h"

/*使能FREERTOS，Init需要在任务里面进行初始化 */
// #define TMP117_USE_FREERTOS

#define TMP117_I2C_ADDR        0x48          // 默认 ADD0 = GND
#define TMP117_I2C_WRITE_ADDR  (TMP117_I2C_ADDR << 1)
#define TMP117_I2C_READ_ADDR   ((TMP117_I2C_ADDR << 1) | 0x01)
#define TMP117_DEV_ID          0x117

/* === 寄存器地址定义 === */
#define TMP117_REG_TEMP        0x00          // 温度结果寄存器（只读）
#define TMP117_REG_CONFIG      0x01          // 配置寄存器
#define TMP117_REG_T_LOW       0x03          // 低温阈值寄存器
#define TMP117_REG_T_HIGH      0x02          // 高温阈值寄存器
#define TMP117_REG_TEMP_OFFSET 0x07          // 温度偏移寄存器
#define TMP117_REG_DEVICE_ID   0x0F          // 器件 ID 寄存器（固定为 0x117）

#define TMP117_REG_EEPROM_UL   0x04          // EEPROM 解锁锁定寄存器
#define TMP117_REG_EEPROM1     0x05          // EEPROM1 寄存器
#define TMP117_REG_EEPROM2     0x06          // EEPROM2 寄存器
#define TMP117_REG_EEPROM3     0x08          // EEPROM3 寄存器


/**
 * @brief TMP117 工作模式枚举
 */
typedef enum {
    TMP117_MODE_CONTINUOUS   = 0x00,   // MOD[1:0] = 11  连续转换模式
    TMP117_MODE_SHUTDOWN     = 0x01,   // MOD[1:0] = 01  关断模式
    TMP117_MODE_SINGLE       = 0x03,   // MOD[1:0] = 11  单次转换模式
} TMP117_Mode_e;

/**
 * @brief TMP117 转换速率枚举
 */
typedef enum {
    TMP117_DATA_RATE_64HZ = 0,   // CONV[2:0] = 000
    TMP117_DATA_RATE_8HZ = 1,   // CONV[2:0] = 001
    TMP117_DATA_RATE_4HZ = 2,  // CONV[2:0] = 010
    TMP117_DATA_RATE_2HZ = 3,  // CONV[2:0] = 011
    TMP117_DATA_RATE_1HZ = 4,  // CONV[2:0] = 100
} TMP117_DataRate_e;


/**
 * @brief TMP117 平均采样次数枚举
 */
typedef enum {
    TMP117_NO_AVG  = 0,
    TMP117_AVG_8   = 1,
    TMP117_AVG_32  = 2,
    TMP117_AVG_64  = 3,
} TMP117_Avg_e;

/**
 * @brief ALERT 工作模式
 */
typedef enum {
    TMP117_ALERT_MODE_ALERT = 0,      // T/nA = 0  Alert mode (双阈值，锁存)
    TMP117_ALERT_MODE_THERMOSTAT = 1, // T/nA = 1  Thermostat mode (单阈值+迟滞，非锁存)
} TMP117_AlertMode_e;

/**
 * @brief ALERT 引脚极性
 */
typedef enum {
    TMP117_ALERT_POLARITY_LOW = 0,    // POL = 0  Low active (ALERT 低有效)
    TMP117_ALERT_POLARITY_HIGH = 1,   // POL = 1  High active (ALERT 高有效)
} TMP117_AlertPolarity_e;


/**
 * @brief ALERT 引脚模式
 */
typedef enum {
    TMP117_ALERT_PIN_ALERT_FLAGS = 0,  // DR/Alert = 0  映警报标志的状态
    TMP117_ALERT_PIN_DATA_READY = 1,   // DR/Alert = 1  数据就绪标志的状态
} TMP117_AlertPinMode_e;

/**
 * @brief TMP117 错误代码
 */
typedef enum {
    TMP117_NO_ERR     = 0, /**< 无错误 */
    TMP117_WRITE_ERR  = 1, /**< I2C 写错误 */
    TMP117_READ_ERR   = 2, /**< I2C 读错误 */
    TMP117_INIT_ERR   = 3, /**< 初始化失败（ID 不匹配） */
} TMP117_Status_t;


/* === 配置结构体 === */
typedef struct {
    TMP117_Mode_e           mode;                 // 工作模式
    TMP117_DataRate_e       data_rate;            // 转换速率
    TMP117_Avg_e            avg_samples;          // 平均采样次数
    TMP117_AlertPinMode_e   alert_pin_mode;       // ALERT 引脚模式
    TMP117_AlertMode_e      alert_mode;           // ALERT 工作模式
    TMP117_AlertPolarity_e  alert_polarity;       // ALERT 引脚极性
} TMP117_Config_TypeDef;

/* === 状态寄存器结构体 === */
typedef struct {
    uint8_t high_Alert;   // 高阈值报警
    uint8_t low_Alert;    // 低阈值报警,热模式保持0
    uint8_t data_ready;   // 数据就绪
    uint8_t eeprom_Busy;  // EEPROM 忙
} TMP117_Status_TypeDef;

TMP117_Status_t tmp117_Init(void);
float tmp117_GetData(void);
TMP117_Status_t tmp117_reset(void);
TMP117_Status_t tmp117_GetStatus(TMP117_Status_TypeDef* status);
TMP117_Status_t tmp117_SetHighLimit(float Hlimit);
TMP117_Status_t tmp117_SetLowLimit(float Llimit);

#endif //MP117_H