//
// Created by asus on 2026/1/21.
//

#ifndef F407VG_JLC_HSCDTD008A_H
#define F407VG_JLC_HSCDTD008A_H
#include "main.h"

// #define HSCDTD008A_USE_FREERTOS

/* I2C Address */
#define HSCDTD008A_I2C_ADDR 0x0C
#define HSCDTD008A_I2C_WRITE_ADDR (HSCDTD008A_I2C_ADDR << 1)
#define HSCDTD008A_I2C_READ_ADDR ((HSCDTD008A_I2C_ADDR << 1) | 0x01)
#define HSCDTD008A_DEV_ID 0x49

/* === Register Addresses === */
#define HSCDTD008A_REG_WIA          0x0F  /* Who Am I */
#define HSCDTD008A_REG_STAT         0x18  /* Status Register */
#define HSCDTD008A_REG_STB          0x0C  /* Self-test Register */

#define HSCDTD008A_REG_CTRL1        0x1B  /* Control Register 1 */
#define HSCDTD008A_REG_CTRL2        0x1C  /* Control Register 2 */
#define HSCDTD008A_REG_CTRL3        0x1D  /* Control Register 3 */
#define HSCDTD008A_REG_CTRL4        0x1E  /* Control Register 4 */

#define HSCDTD008A_REG_TEMP         0x31  /* Temp Register 3 */
#define HSCDTD008A_REG_OUTX_LSB     0x10  /* X-axis Output LSB */
#define HSCDTD008A_REG_OUTX_MSB     0x11  /* X-axis Output MSB */
#define HSCDTD008A_REG_OUTY_LSB     0x12  /* Y-axis Output LSB */
#define HSCDTD008A_REG_OUTY_MSB     0x13  /* Y-axis Output MSB */
#define HSCDTD008A_REG_OUTZ_LSB     0x14  /* Z-axis Output LSB */
#define HSCDTD008A_REG_OUTZ_MSB     0x15  /* Z-axis Output MSB */

#define HSCDTD008A_REG_OFFX_LSB     0x20  /* X-axis Offset LSB */
#define HSCDTD008A_REG_OFFX_MSB     0x21  /* X-axis Offset MSB */
#define HSCDTD008A_REG_OFFY_LSB     0x22  /* Y-axis Offset LSB */
#define HSCDTD008A_REG_OFFY_MSB     0x23  /* Y-axis Offset MSB */
#define HSCDTD008A_REG_OFFZ_LSB     0x24  /* Z-axis Offset LSB */
#define HSCDTD008A_REG_OFFZ_MSB     0x25  /* Z-axis Offset MSB */

/* FIFO Register Addresses */
#define HSCDTD008A_REG_FFPT         0x19  /* FIFO指针状态寄存器 */
#define HSCDTD008A_REG_ITHR_LSB     0x26  /* 比较阈值寄存器 LSB */
#define HSCDTD008A_REG_ITHR_MSB     0x27  /* 比较阈值寄存器 MSB */

/* === HSCDTD008A Power Mode === */
typedef enum {
    HSCDTD008A_POWER_SLEEP = 0x00,   /**< Sleep mode: No measurement, low power */
    HSCDTD008A_POWER_ACTIVE = 0x01,  /**< Active mode: Measurement enabled */
} HSCDTD008A_PowerMode_e;

/* === HSCDTD008A Output Data Rate === */
typedef enum {
    HSCDTD008A_ODR_0_5HZ = 0x00,    /**< 0.5 Hz */
    HSCDTD008A_ODR_10HZ = 0x01,     /**< 10 Hz */
    HSCDTD008A_ODR_20HZ = 0x02,     /**< 20 Hz */
    HSCDTD008A_ODR_100HZ = 0x03,    /**< 100 Hz */
} HSCDTD008A_ODR_e;

/* === HSCDTD008A Force Mode === */
typedef enum {
    HSCDTD008A_FORCE_MODE = 0x01,  /**< Force Mode: Single measurement */
    HSCDTD008A_NORMAL_MODE = 0x00, /**< Normal Mode: Continuous measurement */
} HSCDTD008A_ForceMode_e;

typedef enum {
    HSCDTD008A_NOT_UPDATE = 0,
    HSCDTD008A_UPDATE,
} HSCDTD008A_DataUpdateState_e;

typedef enum {
    HSCDTD008A_NOT_OVERFLOW = 0,
    HSCDTD008A_OVERFLOW,
} HSCDTD008A_DataOverflowState_e;

typedef enum {
    HSCDTD008A_NOT_FULL = 0,
    HSCDTD008A_FULL,
} HSCDTD008A_FIFO_FullState_e;

/**
 * @brief HSCDTD008A传感器配置结构体
 * 该结构体定义了HSCDTD008A传感器的各种配置参数，包括控制寄存器CTRL1和CTRL2的相关设置
 * CTRL1寄存器包含电源模式、输出频率和输出模式配置
 * CTRL2寄存器包含DRY引脚控制、FIFO控制等功能配置
 */
typedef struct {
    //CTRL1
    HSCDTD008A_PowerMode_e POWER_MODE;  /* 电源模式 */
    HSCDTD008A_ODR_e ODR;   /* 输出频率 */
    HSCDTD008A_ForceMode_e FS;  /* 输出模式 */
    //CTRL2
    uint8_t DRY_EnableState;  /* DRY引脚使能状态 */
    uint8_t DRY_ActiveLevel;  /* DRY引脚有效电平 */
    uint8_t FIFO_EnableState; /* FIFO使能状态 */
    uint8_t FIFO_DataStorageMethod; /* FIFO数据储存方法 0=直接储存 1=比较模式 */
    uint8_t FIFO_Comparison;   /* 比较模式 0=OR, 1=AND */
} HSCDTD008A_Config_TypeDef;


/**
 * @brief HSCDTD008A传感器状态结构体
 * 该结构体用于存储HSCDTD008A传感器的各种状态信息，包括数据更新状态、
 * 数据溢出状态和FIFO满状态等关键运行状态标志。
 */
typedef struct {
    HSCDTD008A_DataUpdateState_e update_state;
    HSCDTD008A_DataOverflowState_e overflow_state;
    HSCDTD008A_FIFO_FullState_e fifo_full_state;
} HSCDTD008A_Status_TypeDef;

/**
 * @brief 控制模式枚举类型定义
 * 定义了设备控制寄存器CTRL3的各种工作模式，用于配置传感器的不同操作状态
 */
typedef enum {
    OFFSET_CALI = 0,  /**< 偏移校准模式 - Force State可用 bit0 */
    TEMP_MEAS = 1,    /**< 温度测量模式 bit1 */
    SELF_TEST = 4,    /**< 自检模式 bit4 */
    FORCE_MEAS = 6,   /**< 强制测量模式 bit6 */
    SOFT_RESET = 7,   /**< 软件复位模式 bit7 */
} HSCDTD008A_Ctrl3_Mode_e;



/**
 * @brief HSCDTD008A磁力计数据结构体
 *
 * 该结构体用于存储HSCDTD008A三轴磁力计的测量数据，包括三个轴向的磁场强度
 * 以及传感器温度信息。
 */
typedef struct {
    float mag_x;    /* X-axis magnetic field (µT) */
    float mag_y;    /* Y-axis magnetic field (µT) */
    float mag_z;    /* Z-axis magnetic field (µT) */
    int8_t temperature;
    HSCDTD008A_Status_TypeDef status;
} __attribute__((packed)) HSCDTD008A_data_t;


/**
 * @brief HSCDTD008A设备状态枚举类型定义
 * 定义了HSCDTD008A设备操作的各种状态码，用于标识操作的成功或失败类型
 */
typedef enum {
    HSCDTD008A_NO_ERR = 0,
    HSCDTD008A_WRITE_ERR = 1,
    HSCDTD008A_READ_ERR = 2,
    HSCDTD008A_INIT_ERR = 3,
    HSCDTD008A_BUSY_ERR = 4,
} HSCDTD008A_status_t;

HSCDTD008A_status_t hscdtd008a_Init(void);
void hscdtd008a_GetData(HSCDTD008A_data_t* data);
void hscdtd008a_GetStatus(HSCDTD008A_data_t* data);
HSCDTD008A_status_t hscdtd008a_ForceMeasure(void);
HSCDTD008A_status_t hscdtd008a_reset(void);
uint8_t hscdtd008a_read_id(void);
HSCDTD008A_status_t hscdtd008a_ctrl3(HSCDTD008A_Ctrl3_Mode_e mode);



#endif //F407VG_JLC_HSCDTD008A_H