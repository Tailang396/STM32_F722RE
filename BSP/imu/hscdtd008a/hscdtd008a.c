//
// Created by asus on 2026/1/21.
//

#include "hscdtd008a.h"
#include "bsp_def.h"
#include "i2c.h"


/**
 * @brief SCDTDT008A传感器I2C句柄指针
 * 指向SCDTDT008A传感器所使用的I2C接口句柄，该传感器通过I2C总线进行通信，
 */
static const I2C_HandleTypeDef* hscdtd008a_iic = &hi2c1;
static const uint32_t time_out = 10;

/**
 * @brief HSCDTD008A传感器配置结构体
 * 该结构体定义了HSCDTD008A磁传感器的初始配置参数，包括电源模式、输出数据率、
 * 满量程范围、数据就绪引脚配置和FIFO功能配置等关键参数。
 */
static const HSCDTD008A_Config_TypeDef hscdtd008a_config = {
    .POWER_MODE = HSCDTD008A_POWER_ACTIVE, /**< 电源模式 */
    .ODR = HSCDTD008A_ODR_100HZ, /**< 输出数据率 */
    .FS = HSCDTD008A_NORMAL_MODE, /**< 满量程模式 */
    .DRY_EnableState = 1, /**< 数据就绪功能 */
    .DRY_ActiveLevel = 0, /**< 数据就绪有效电平 */
    .FIFO_EnableState = 0, /**< FIFO功能 */
    .FIFO_Comparison = 0, /**< FIFO比较功能 */
    .FIFO_DataStorageMethod = 0, /**< FIFO存储方式 */
};


static uint8_t hscdtd008a_read_reg8(uint8_t reg) {
    uint8_t data;
    HAL_I2C_Mem_Read((I2C_HandleTypeDef*)hscdtd008a_iic, HSCDTD008A_I2C_READ_ADDR,
                     reg, I2C_MEMADD_SIZE_8BIT, &data, 1, time_out);
    return data;
}

static HSCDTD008A_status_t hscdtd008a_write_reg8(uint8_t reg, uint8_t data) {
    HSCDTD008A_status_t status =
        HAL_I2C_Mem_Write((I2C_HandleTypeDef*)hscdtd008a_iic, HSCDTD008A_I2C_WRITE_ADDR,
                          reg, I2C_MEMADD_SIZE_8BIT, &data, 1, time_out)
        == HAL_OK
            ? HSCDTD008A_NO_ERR
            : HSCDTD008A_WRITE_ERR;
    return status;
}

static uint16_t hscdtd008a_read_reg16(uint8_t reg) {
    uint8_t buffer[2];
    HAL_I2C_Mem_Read((I2C_HandleTypeDef*)hscdtd008a_iic, HSCDTD008A_I2C_READ_ADDR,
                     reg, I2C_MEMADD_SIZE_8BIT, buffer, 2, time_out);
    return (uint16_t)((buffer[1] << 8) | buffer[0]);
}


/**
 * @brief 写入HSCDTD008A传感器的控制寄存器3
 * 该函数用于向HSCDTD008A传感器的CTRL3寄存器写入指定数据。
 * @param data 要写入到CTRL3寄存器的数据字节
 * @return HSCDTD008A_status_t 返回操作状态
 */
static HSCDTD008A_status_t hscdtd008a_write_ctrl3(uint8_t data) {
    HSCDTD008A_status_t status = HSCDTD008A_NO_ERR;
    // 检查CTRL3寄存器是否为空闲状态（值为0x00），如果不空闲则返回忙错误
    if (hscdtd008a_read_reg8(HSCDTD008A_REG_CTRL3) != 0x00) {
        status = HSCDTD008A_BUSY_ERR;
        return status;
    }

    // 向CTRL3寄存器写入指定数据，如果写入失败则更新状态为写入错误
    if (hscdtd008a_write_reg8(HSCDTD008A_REG_CTRL3, data) != HSCDTD008A_NO_ERR) {
        status = HSCDTD008A_WRITE_ERR;
    }
    return status;
}


/**
 * @brief 配置HSCDTD008A传感器的控制寄存器
 * 该函数根据全局配置结构体hscdtd008a_config的设置，
 * 配置HSCDTD008A传感器的CTRL1和CTRL2寄存器，
 * 设置包括电源模式、输出数据率、满量程范围、数据就绪引脚电平、FIFO功能等参数。
 * @return HSCDTD008A_status_t 返回操作状态
 */
static HSCDTD008A_status_t hscdtd008a_congifure(void) {
    uint8_t ctrl1 = 0;
    uint8_t ctrl2 = 0;
    // 构建CTRL1寄存器值：设置电源模式、输出数据率和满量程范围
    ctrl1 |= (hscdtd008a_config.POWER_MODE << 7);
    ctrl1 |= (hscdtd008a_config.ODR << 3);
    ctrl1 |= (hscdtd008a_config.FS << 1);
    // 构建CTRL2寄存器值：设置数据就绪引脚、FIFO相关功能
    ctrl2 |= (hscdtd008a_config.DRY_ActiveLevel << 2);
    ctrl2 |= (hscdtd008a_config.DRY_EnableState << 3);
    ctrl2 |= (hscdtd008a_config.FIFO_EnableState << 4);
    ctrl2 |= (hscdtd008a_config.FIFO_DataStorageMethod << 5);
    ctrl2 |= (hscdtd008a_config.FIFO_Comparison << 6);
    if (hscdtd008a_write_reg8(HSCDTD008A_REG_CTRL1, ctrl1) != HSCDTD008A_NO_ERR) {
        return HSCDTD008A_WRITE_ERR;
    }
    if (hscdtd008a_write_reg8(HSCDTD008A_REG_CTRL2, ctrl2) != HSCDTD008A_NO_ERR) {
        return HSCDTD008A_WRITE_ERR;
    }
    return HSCDTD008A_NO_ERR;
}



/**
 * @brief HSCDTD008A传感器自检函数
 * 该函数执行HSCDTD008A传感器的自检流程，通过读取和验证自检寄存器的值来确认传感器工作状态
 * @return HSCDTD008A_status_t 返回自检结果状态
 */
static HSCDTD008A_status_t hscdtd008a_self_test(void) {
    uint8_t self_test = 0;
    self_test = hscdtd008a_read_reg8(HSCDTD008A_REG_STB);
    if (self_test != 0x55) {
        return HSCDTD008A_INIT_ERR;
    }
    hscdtd008a_ctrl3(SELF_TEST);
    self_test = hscdtd008a_read_reg8(HSCDTD008A_REG_STB);
    if (self_test != 0xAA) {
        return HSCDTD008A_INIT_ERR;
    }
    self_test = hscdtd008a_read_reg8(HSCDTD008A_REG_STB);
    if (self_test != 0x55) {
        return HSCDTD008A_INIT_ERR;
    }
    return HSCDTD008A_NO_ERR;
}

/**
 * @brief 控制HSCDTD008A设备的控制寄存器3
 * 该函数用于设置HSCDTD008A设备的控制寄存器3，通过指定的模式来配置设备的相关功能。
 * 函数将输入的模式转换为对应的位掩码，并写入到控制寄存器3中。
 * @param mode 控制模式枚举值，用于指定控制寄存器3的具体配置模式
 * @return HSCDTD008A_status_t 返回操作状态，成功时返回HSCDTD008A_NO_ERR，失败时返回相应的错误码
 */
HSCDTD008A_status_t hscdtd008a_ctrl3(HSCDTD008A_Ctrl3_Mode_e mode) {
    HSCDTD008A_status_t status = HSCDTD008A_NO_ERR;
    uint8_t data = (1 << mode);
    status = hscdtd008a_write_ctrl3(data);
    return status;
}


/**
 * @brief 读取HSCDTD008A传感器的ID寄存器值
 * 该函数通过读取HSCDTD008A的WIA（Who I Am）寄存器来获取传感器的设备ID，
 * @return uint8_t 返回从WIA寄存器读取的8位设备ID值
 */
uint8_t hscdtd008a_read_id(void) {
    return hscdtd008a_read_reg8(HSCDTD008A_REG_WIA);
}

/**
 * @brief 复位HSCDTD008A传感器
 * 该函数通过发送软复位命令来复位HSCDTD008A传感器，
 * @return HSCDTD008A_status_t 返回操作状态
 */
HSCDTD008A_status_t hscdtd008a_reset(void) {
    return hscdtd008a_ctrl3(SOFT_RESET);
}

/**
 * @brief 初始化HSCDTD008A传感器 需要10ms
 * 该函数执行HSCDTD008A传感器的完整初始化流程，包括复位、ID验证、配置和自检
 * @return HSCDTD008A_status_t 返回初始化状态
 */
HSCDTD008A_status_t hscdtd008a_Init(void) {
    // if (hscdtd008a_reset() != HSCDTD008A_NO_ERR) {
    //     return HSCDTD008A_INIT_ERR;
    // }
#ifdef HSCDTD008A_USE_FREERTOS
    osDelay(10);
#else
    HAL_Delay(10);
#endif
    if (hscdtd008a_read_id() != HSCDTD008A_DEV_ID) {
        return HSCDTD008A_INIT_ERR;
    }
    if (hscdtd008a_congifure() != HSCDTD008A_NO_ERR) {
        return HSCDTD008A_INIT_ERR;
    }
    if (hscdtd008a_self_test() != HSCDTD008A_NO_ERR) {
        return HSCDTD008A_INIT_ERR;
    }
    return HSCDTD008A_NO_ERR;
}


/**
 * @brief 获取HSCDTD008A磁传感器的数据
 * 该函数读取磁传感器的原始数据，并将其转换为实际的磁场强度值（单位：微特斯拉）
 * @param data 指向HSCDTD008A数据结构体的指针，用于存储读取到的磁场强度和温度数据
 *             结构体包含mag_x、mag_y、mag_z三个轴的磁场强度和temperature温度值
 */
void hscdtd008a_GetData(HSCDTD008A_data_t* data) {
    int16_t raw_x, raw_y, raw_z;
    raw_x = (int16_t)hscdtd008a_read_reg16(HSCDTD008A_REG_OUTX_LSB);
    raw_y = (int16_t)hscdtd008a_read_reg16(HSCDTD008A_REG_OUTY_LSB);
    raw_z = (int16_t)hscdtd008a_read_reg16(HSCDTD008A_REG_OUTZ_LSB);
    data->mag_x = (float)raw_x * 0.15f;
    data->mag_y = (float)raw_y * 0.15f;
    data->mag_z = (float)raw_z * 0.15f;
    data->temperature = (int8_t)hscdtd008a_read_reg8(HSCDTD008A_REG_TEMP);
}

/**
 * @brief 获取HSCDTD008A传感器的状态信息
 * @param data 指向HSCDTD008A数据结构的指针，用于存储读取到的状态信息
 *             函数将更新data->status中的update_state、overflow_state和fifo_full_state字段
 */
void hscdtd008a_GetStatus(HSCDTD008A_data_t* data) {
    uint8_t status_data = 0;
    status_data = hscdtd008a_read_reg8(HSCDTD008A_REG_STAT);
    data->status.update_state = (status_data & (1 << 6)) ? HSCDTD008A_UPDATE : HSCDTD008A_NOT_UPDATE;
    data->status.overflow_state = (status_data & (1 << 5)) ? HSCDTD008A_OVERFLOW : HSCDTD008A_NOT_OVERFLOW;
    data->status.fifo_full_state = (status_data & (1 << 2)) ? HSCDTD008A_FULL : HSCDTD008A_NOT_FULL;
}

/**
 * @brief 强制执行测量操作
 * @return HSCDTD008A_status_t 返回操作状态
 */
HSCDTD008A_status_t hscdtd008a_ForceMeasure(void) {
    if (hscdtd008a_ctrl3(FORCE_MEAS) != HSCDTD008A_NO_ERR) {
        return HSCDTD008A_WRITE_ERR;
    }
    if (hscdtd008a_ctrl3(TEMP_MEAS) != HSCDTD008A_NO_ERR) {
        return HSCDTD008A_WRITE_ERR;
    }
    return HSCDTD008A_NO_ERR;
}

