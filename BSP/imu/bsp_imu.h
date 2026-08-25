//
// Created by Yu_Jie on 2025/12/20.
//

#ifndef BSP_IMU_H
#define BSP_IMU_H
#include "main.h"

#define IMU_DISABLE 0
#define IMU_ENABLE  1

typedef struct {
    float mag_temp;
    float bmp_temp;
    float imu_temp;
    float bsp_temp;
} imu_temp_data_t;

typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
} imu_acc_data_t;

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
} imu_gyro_data_t;

typedef struct {
    float mag_x;
    float mag_y;
    float mag_z;
} imu_mag_data_t;

typedef struct {
    float pressure;
    float altitude;
} imu_bmp_data_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} imu_angle_data_t;

typedef struct {
    uint8_t MAG_STATE;            //是否启用磁力计
    uint16_t CALI_SAMPLE_TIMES;   //校准采样次数 0是不校准  500够用
    uint16_t TMEP_CTRL_STATE;     //温度控制状态
} IMU_InitTypeDef;

typedef struct {
    imu_angle_data_t angle;
    imu_acc_data_t acc;
    imu_gyro_data_t gyro;
    imu_mag_data_t mag;
    imu_bmp_data_t bmp;
    imu_temp_data_t temp;
} imu_data_t;

void bsp_imu_init(TIM_HandleTypeDef *htim);
imu_data_t* bsp_imu_GetData(void);


void bsp_imu_set_mode(void);
void bsp_tempctrl_init(void);
void bsp_tempctrl_update(void);
void fusion_init(void);
void fusion_update(void);

#endif
