//
// Created by asus on 2026/1/3.
//

#ifndef BMP280_H
#define BMP280_H
#include "main.h"

/*使能FREERTOS，Init需要在任务里面进行初始化 */
// #define BMP280_USE_FREERTOS

#define BMP280_I2C_ADDR        0x76
#define BMP280_I2C_WRITE_ADDR  (BMP280_I2C_ADDR << 1)
#define BMP280_I2C_READ_ADDR   ((BMP280_I2C_ADDR << 1) | 0x01)
#define BMP280_DEV_ID          0x58

/* === 校准寄存器起始地址 === */
#define BMP280_REG_DIG_T1      0x88
#define BMP280_REG_DIG_T2      0x8A
#define BMP280_REG_DIG_T3      0x8C
#define BMP280_REG_DIG_P1      0x8E
#define BMP280_REG_DIG_P2      0x90
#define BMP280_REG_DIG_P3      0x92
#define BMP280_REG_DIG_P4      0x94
#define BMP280_REG_DIG_P5      0x96
#define BMP280_REG_DIG_P6      0x98
#define BMP280_REG_DIG_P7      0x9A
#define BMP280_REG_DIG_P8      0x9C
#define BMP280_REG_DIG_P9      0x9E
#define BMP280_REG_DIG_xx      0xA1

/* === 芯片 ID寄存器 === */
#define BMP280_REG_CHIP_ID     0xD0

/* === 软复位寄存器 === */
#define BMP280_REG_RESET       0xE0

/* === 控制与配置寄存器 === */
#define BMP280_REG_CTRL_MEAS                0xF4
#define BMP280_REG_CONFIG                   0xF5

/* === 数据寄存器 === */
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_XLSB  0xF9
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_XLSB   0xFC

/**
 * @brief BMP280传感器电源模式枚举
 * 定义了BMP280气压传感器的三种电源工作模式：
 * - 睡眠模式：不进行测量，传感器处于低功耗状态
 * - 正常模式：在测量期和非活动待机期之间自动永久循环，只需要读寄存器
 * - 强制模式：只进行一次测量，测量结束后传感器返回休眠模式，再次测量需要重新配置强制模式
 */
typedef enum {
    BMP280_SLEEP_MODE = 0x00,   /**< 睡眠模式：传感器不进行测量，处于休眠状态 */
    BMP280_FORCED_MODE = 0x01,  /**< 强制模式：执行单次测量，完成后返回休眠模式 */
    BMP280_NORMAL_MODE = 0x03,  /**< 正常模式：自动循环测量和待机 */
} BMP280_PowerMode_e;


/**
 * @brief BMP280传感器过采样倍数枚举
 * 该枚举定义了BMP280气压传感器的过采样倍数选项，用于配置传感器的精度和功耗
 * 过采样倍数越高，测量精度越高，但功耗和转换时间也相应增加
 */
typedef enum {
    BMP280_OVERSAMP_NONE = 0x00, /**< 无过采样 */
    BMP280_OVERSAMP_1X = 0x01,   /**< 1倍过采样：最低精度，最低功耗 */
    BMP280_OVERSAMP_2X = 0x02,   /**< 2倍过采样 */
    BMP280_OVERSAMP_4X = 0x03,   /**< 4倍过采样 */
    BMP280_OVERSAMP_8X = 0x04,   /**< 8倍过采样 */
    BMP280_OVERSAMP_16X = 0x05,  /**< 16倍过采样：最高精度，最高功耗 */
} BMP280_OverSamp_e;



/**
 * @brief BMP280传感器数字滤波器配置枚举
 * 该枚举定义了BMP280气压传感器的数字滤波器设置选项，
 * 用于配置传感器输出数据的滤波强度，以减少噪声和提高数据稳定性
 */
typedef enum {
    BMP280_FILTER_OFF = 0x00,   /* 关闭滤波器 */
    BMP280_FILTER_2X = 0x01,    /* 2倍滤波 */
    BMP280_FILTER_4X = 0x02,    /* 4倍滤波 */
    BMP280_FILTER_8X = 0x03,    /* 8倍滤波 */
    BMP280_FILTER_16X = 0x04,   /* 16倍滤波 */
    BMP280_FILTER_32X = 0x05,   /* 32倍滤波 */
} BMP280_Filter_e;



/**
 * @brief BMP280传感器待机时间配置枚举
 * 该枚举定义了BMP280传感器在正常模式下的待机时间选项，
 * 用于控制传感器在测量之间的休眠时间，影响功耗和响应速度
 */
typedef enum {
    BMP280_STANDBY_TIME_0_5MS = 0x00,  /**< 待机时间0.5毫秒 */
    BMP280_STANDBY_TIME_62_5MS = 0x01, /**< 待机时间62.5毫秒 */
    BMP280_STANDBY_TIME_125MS = 0x02,  /**< 待机时间125毫秒 */
    BMP280_STANDBY_TIME_250MS = 0x03,  /**< 待机时间250毫秒 */
    BMP280_STANDBY_TIME_500MS = 0x04,  /**< 待机时间500毫秒 */
    BMP280_STANDBY_TIME_1000MS = 0x05, /**< 待机时间1000毫秒 */
    BMP280_STANDBY_TIME_2000MS = 0x06, /**< 待机时间2000毫秒 */
    BMP280_STANDBY_TIME_4000MS = 0x07, /**< 待机时间4000毫秒 */
} BMP280_StandbyTime_e;

/**
 * @brief BMP280传感器SPI接口使能状态枚举
 * 该枚举定义了BMP280传感器3线SPI接口的使能状态选项
 */
typedef enum  {
    BMP280_3SPI_DISABLE = 0x00,  /**< 3线SPI接口禁用状态 */
    BMP280_3SPI_ENABLE = 0x01,   /**< 3线SPI接口启用状态 */
} BMP280_3SPI_e;

typedef enum {
    BMP280_NO_ERR = 0,
    BMP280_WRITE_ERR = 1,
    BMP280_READ_ERR = 2,
    BMP280_INIT_ERR = 3,
} BMP280_status_t;


/* === 校准参数结构体 === */
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} BMP280_Calib_TypeDef;

typedef struct {
    //ctrl_meas 控制测量寄存器
    BMP280_OverSamp_e BMP280_T_OverSamp;
    BMP280_OverSamp_e BMP280_P_OverSamp;
    BMP280_PowerMode_e BMP280_PowerMode;
    //config 配置寄存器
    BMP280_StandbyTime_e BMP280_StandbyTime;
    BMP280_Filter_e BMP280_Filter;
    BMP280_3SPI_e BMP280_3SPI;
} BMP280_Config_TypeDef;

typedef struct {
    float temperature;
    float pressure;
    float altitude;
} BMP280_data_t;

BMP280_status_t bmp280_Init(void);
BMP280_status_t bmp280_reset(void);
BMP280_status_t bmp280_GetData(BMP280_data_t *data);
#endif //F407VG_JLC_BMP280_H
