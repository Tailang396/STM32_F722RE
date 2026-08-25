//
// Created by asus on 2026/1/1.
//

#ifndef BSP_SHT40_H
#define BSP_SHT40_H
#include "bsp_def.h"

/*使能FREERTOS，Init需要在任务里面进行初始化 */
#define SHT40_USE_FREERTOS  // SHT40传感器使用FreeRTOS
#define SHT40_USE_CRC //SHT40传感器CRC校验使能宏定义


#define SHT40_I2C_ADDR  0x44
#define SHT40_I2C_WRITE_ADDR  (SHT40_I2C_ADDR << 1)
#define SHT40_I2C_READ_ADDR  ((SHT40_I2C_ADDR << 1) | 0x01)

#define SHT40_CMD_HIGH_PRECISION      0xFD  // High precision measurement
#define SHT40_CMD_MED_PRECISION       0xF6  // Medium precision measurement
#define SHT40_CMD_LOW_PRECISION       0xE0  // Low precision measurement
#define SHT40_CMD_READ_SERIAL         0x89  // Read serial number
#define SHT40_CMD_SOFT_RESET          0x94  // Soft reset
#define SHT40_CMD_HEAT_220MW_1S       0x39  // Heat 220mW for 1s
#define SHT40_CMD_HEAT_220MW_100MS    0x32  // Heat 220mW for 0.1s
#define SHT40_CMD_HEAT_110MW_1S       0x2F  // Heat 110mW for 1s
#define SHT40_CMD_HEAT_110MW_100MS    0x24  // Heat 110mW for 0.1s
#define SHT40_CMD_HEAT_20MW_1S        0x1E  // Heat 20mW for 1s
#define SHT40_CMD_HEAT_20MW_100MS     0x15  // Heat 20mW for 0.1s


typedef enum {
    SHT40_NO_ERR = 0,
    SHT40_WRITE_ERR = 1,
    SHT40_READ_ERR = 2,
    SHT40_CRC_ERR = 3,
    SHT40_INIT_ERR = 4,
} SHT40_status_t;

typedef enum {
    SHT40_PRECISION_HIGH = 0xFD,
    SHT40_PRECISION_MED = 0xF6,
    SHT40_PRECISION_LOW = 0xE0,
} SHT40_precision_e;

typedef enum {
    SHT40_HEAT_220MW_1S = 0x39,
    SHT40_HEAT_220MW_100MS = 0x32,
    SHT40_HEAT_110MW_1S = 0x2F,
    SHT40_HEAT_110MW_100MS = 0x24,
    SHT40_HEAT_20MW_1S = 0x1E,
    SHT40_HEAT_20MW_100MS = 0x15,
} SHT40_heat_e;

typedef struct {
    float temperature;
    float humidity;
} SHT40_Data_t;

SHT40_status_t sht40_Init(void);
SHT40_status_t sht40_ReadData(SHT40_Data_t *data, SHT40_precision_e precision);
SHT40_status_t sht40_ReadData_heat(SHT40_Data_t *data, SHT40_precision_e precision, SHT40_heat_e heat);

#endif //BSP_SHT40_H
