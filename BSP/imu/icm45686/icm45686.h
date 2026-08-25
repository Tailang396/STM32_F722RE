//
// Created by Luminescence on 2025/7/8.
//

#ifndef ICM45686_H
#define ICM45686_H
#include "bsp_uart.h"
#include "inv_imu_defs.h"
#include "inv_imu_driver.h"

#define ICM45686_DEBUG E_UART_DEBUG


/**
 * @brief ICM45686 初始化配置结构体
 * 用于配置 ICM45686 IMU 传感器的所有初始化参数，包括中断、量程、采样率、滤波和工作模式
 */
typedef struct {
    inv_imu_int_pin_config_t int_pin_config;            /* 中断引脚配置 */
    inv_imu_int_state_t int_state_config;               /* 中断状态配置 */
    accel_config0_accel_ui_fs_sel_t acc_range;          /* 加速度计量程配置 */
    gyro_config0_gyro_ui_fs_sel_t gyro_range;           /* 陀螺仪量程配置 */
    accel_config0_accel_odr_t acc_sample_rate;          /* 加速度计采样率配置 */
    gyro_config0_gyro_odr_t gyro_sample_rate;           /* 陀螺仪采样率配置 */
    ipreg_sys2_reg_131_accel_ui_lpfbw_t acc_lpfbw;      /* 加速度计低通滤波器带宽配置 */
    ipreg_sys1_reg_172_gyro_ui_lpfbw_sel_t gyro_lpfbw;  /* 陀螺仪低通滤波器带宽配置 */
    smc_control_0_accel_lp_clk_sel_t low_power_mode;    /* 低功耗模式时钟选择配置 */
    pwr_mgmt0_accel_mode_t acc_mode;                    /* 加速度计工作模式配置 */
    pwr_mgmt0_gyro_mode_t gyro_mode;                    /* 陀螺仪工作模式配置 */
} ICM45686_InitTypeDef;

/**
 * @brief ICM45686 原始数据结构体
 * 存储经过灵敏度转换后的三轴加速度、三轴角速度和温度数据
 */
typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
} imu_data_raw_t;

/**
 * @brief ICM45686 设备状态枚举类型
 * 定义设备的初始化和校准状态
 */
typedef enum {
    ICM45686_ERR = -1,
    ICM45686_NO_ERR = 0,
    ICM45686_INIT_OK,
    ICM45686_CALI_OK,
}imu_status_e;

/**
 * @brief ICM45686 设备参数结构体
 * 存储传感器的灵敏度、采样参数和初始化状态等配置信息
 */
typedef struct {
    float gyro_sensitivity;    // gyro 灵敏度
    float gyro_range;          // gyro 量程
    float acc_sensitivity;     //acc 灵敏度
    float sample_freq;         // 采样频率 单位： Hz
    float sample_time;         // 采样时间 单位： ms
    float deltaTime;             // 积分时间 单位： s
    float gyro_offset[3];       // 陀螺仪偏置x y z
    imu_status_e init_status;  // 初始化状态
}  __attribute__((packed)) imu_parameters_t;

imu_status_e ICM45686_Init(uint16_t sample_times);
int ICM45686_GetRawData(imu_data_raw_t *raw);


// <---------------------------------错误检查----------------------------------------->
#define SI_CHECK_RC(rc)                                                              \
    do {                                                                             \
        if (si_print_error_if_any(rc)) {                                             \
        INV_MSG(INV_MSG_LEVEL_ERROR, "At %s (line %d)", __FILE__, __LINE__);         \
        return rc;                                                                   \
        }                                                                            \
} while (0)
#define INV_MSG(level,msg, ...) bsp_uart_printf(ICM45686_DEBUG, "%d," msg "\r\n", __LINE__, ##__VA_ARGS__)

//error_code
static inline int si_print_error_if_any(int rc) {
    if (rc != 0) {
        switch (rc) {
        case INV_IMU_ERROR:
            bsp_uart_printf(ICM45686_DEBUG, "Unspecified error (%d)", rc);
            break;
        case INV_IMU_ERROR_TRANSPORT:
            bsp_uart_printf(ICM45686_DEBUG, "Error occurred at transport level (%d)", rc);
            break;
        case INV_IMU_ERROR_TIMEOUT:
            bsp_uart_printf(ICM45686_DEBUG, "Action did not complete in the expected time window (%d)", rc);
            break;
        case INV_IMU_ERROR_BAD_ARG:
            bsp_uart_printf(ICM45686_DEBUG, "Invalid argument provided (%d)", rc);
            break;
        case INV_IMU_ERROR_EDMP_BUF_EMPTY:
            bsp_uart_printf(ICM45686_DEBUG, "EDMP buffer is empty (%d)", rc);
            break;
        default:
            bsp_uart_printf(ICM45686_DEBUG, "Unknown error (%d)", rc);
            break;
        }
    }
    return rc;
}

#endif //ICM45686_H
