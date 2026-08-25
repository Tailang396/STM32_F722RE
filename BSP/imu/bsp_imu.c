/**
 * @file bsp_imu.c
 * @brief BSP_IMU库 包含hscdtd008磁力计 bmp280气压计 icm45686陀螺仪驱动
 * 可以配置是否使用磁力计，磁力计校准参数请在此处设置
 *
 * @author Yu_Jie
 * @date 2026/3/17
 */

#include "bsp_imu.h"
#include <time.h>

#include "app_sensor.h"
#include "Fusion.h"
#include "hscdtd008a.h"
#include "icm45686.h"
#include "bmp280.h"
#include "bsp_def.h"
#include "bsp_rgb.h"
#include "PID.h"
#include "tim.h"
#include "tmp117.h"


/**
 * @brief IMU 初始化配置结构体
 * 该结构体用于配置 IMU 设备的初始化参数，包括校准采样次数、磁力计状态和温度控制状态。
 * 作为静态常量，该配置在程序运行期间保持不变。
 */
static IMU_InitTypeDef imu_init = {
    .CALI_SAMPLE_TIMES = 600,
    .MAG_STATE = IMU_DISABLE,
    .TMEP_CTRL_STATE = IMU_DISABLE
};

#define abs(x) ((x) > 0 ? (x) : -(x))

extern imu_parameters_t imu_param;
imu_data_t imu_data = {0};
uint8_t imu_ok_flag = 0;
uint8_t imu_tempctrl_finsh_flag = 1;
uint8_t imu_init_finsh_flag = 0;

static float tmp117_temp;
static float aim_temp = 40.0f;
static float temp_stable_threshold = 0.1f;
static uint16_t stable_count_required = 200;
PID_TypeDef pid_temp;

imu_data_raw_t imu_data_raw;
HSCDTD008A_data_t mag_data;
BMP280_data_t bmp_data;

// Calibration parameters (replace with actual calibration data)
static const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
static const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
static FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};

static const FusionMatrix softIronMatrix = {0.9744618f,  0.0301350f, 0.00280765f,
                                            0.0301350f,  1.0356935f, -0.0542281f,
                                            0.0028076f,  -0.054228f,  0.9945922f};
static const FusionVector hardIronOffset = {7.6314566f,  -3.095440f, -7.6259514f};

// Initialise structures
static FusionBias bias;
static FusionAhrs ahrs;
static FusionAhrsFlags flags = {
    .initialising = 1
};


/**
 * @brief BSP IMU 初始化函数
 * 必须在OS开启之前调用，定时器中断优先级最高，初始化时间较长
 */

void bsp_imu_init(TIM_HandleTypeDef *htim) {
    int8_t status = 0;
    bsp_imu_set_mode();
    status = tmp117_Init();
    BSP_ASSERT(status == 0);
    if (imu_init.TMEP_CTRL_STATE == IMU_ENABLE) {
        imu_tempctrl_finsh_flag = 0;
        bsp_rgb_set(0,0,255);
        bsp_tempctrl_init();
        HAL_TIM_Base_Start_IT(htim); //算法校准和温度控制需要提前打开定时中断
    }
    while (imu_tempctrl_finsh_flag == 0) {
        HAL_Delay(5);
    }
    bsp_rgb_set(0,255,255);
    status = ICM45686_Init(imu_init.CALI_SAMPLE_TIMES);
    BSP_ASSERT(status >= 0);
    status = hscdtd008a_Init();
    BSP_ASSERT(status == 0);
    status = bmp280_Init();
    BSP_ASSERT(status == 0);
    bsp_rgb_set(0,255,0);
    fusion_init();
    imu_init_finsh_flag = 1;
    HAL_TIM_Base_Start_IT(htim); //算法校准和温度控制需要提前打开定时中断
    while (imu_ok_flag == 0) {
        HAL_Delay(5);
    }
}

imu_data_t* bsp_imu_GetData(void) {
    return &imu_data;
}

void bsp_imu_set_mode(void) {
    if (HAL_GPIO_ReadPin(MOD1_GPIO_Port, MOD1_Pin) == GPIO_PIN_RESET) {
        imu_init.TMEP_CTRL_STATE = IMU_ENABLE;
    } else {
        imu_init.TMEP_CTRL_STATE = IMU_DISABLE;
    }
    if (HAL_GPIO_ReadPin(MOD2_GPIO_Port, MOD2_Pin) == GPIO_PIN_RESET) {
        imu_init.MAG_STATE = IMU_ENABLE;
    } else {
        imu_init.MAG_STATE = IMU_DISABLE;
    }
}

void bsp_tempctrl_init(void) {
    PID_Init(&pid_temp, 150.0f, 0.4f, 0.0f, 400, 500);
    pid_temp.polarity = PID_UNIPOLAR;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

void bsp_tempctrl_update(void) {
    if (imu_init.TMEP_CTRL_STATE == IMU_ENABLE) {
        if (app_sensor_GetTMP117OSFlag() == 0)
            tmp117_temp = tmp117_GetData();
        else
            tmp117_temp = app_sensor_GetTMP117Data();
        if (tmp117_temp == 0) tmp117_temp = aim_temp;
        PID_Calculate(&pid_temp, aim_temp, tmp117_temp);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint16_t)pid_temp.PIDout);
        if (imu_tempctrl_finsh_flag == 0) {
            static uint16_t count = 0;
            bsp_uart_printf(E_UART_DEBUG, "%.1f, %.1f, %f\r\n", tmp117_temp, aim_temp, pid_temp.PIDout);
            if (abs(tmp117_temp - aim_temp) <= temp_stable_threshold) {
                count++;
                if (count >= stable_count_required) {
                    imu_tempctrl_finsh_flag = 1;
                }
            }
            else {
                count = 0;
            }
        }
    }
}

void fusion_init(void) {
    FusionBiasInitialise(&bias, (uint32_t)imu_param.sample_freq);
    FusionAhrsInitialise(&ahrs);
    gyroscopeOffset.array[0] = imu_param.gyro_offset[0];
    gyroscopeOffset.array[1] = imu_param.gyro_offset[1];
    gyroscopeOffset.array[2] = imu_param.gyro_offset[2];
    // Set AHRS settings
    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 0.5f,
        .gyroscopeRange = imu_param.gyro_range, /* replace with actual gyroscope range */
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .recoveryTriggerPeriod = 5 * (uint32_t)imu_param.sample_freq, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);
}


void fusion_update(void) {
    if (imu_init_finsh_flag == 1) {
        bmp280_GetData(&bmp_data);
        ICM45686_GetRawData(&imu_data_raw);
        hscdtd008a_GetStatus(&mag_data);
        if (mag_data.status.update_state)
            hscdtd008a_GetData(&mag_data);

        FusionVector gyroscope = {imu_data_raw.gyro_x, imu_data_raw.gyro_y, imu_data_raw.gyro_z};
        FusionVector accelerometer = {imu_data_raw.acc_x, imu_data_raw.acc_y, imu_data_raw.acc_z};
        FusionVector magnetometer = {mag_data.mag_x, mag_data.mag_y, mag_data.mag_z};

        // Apply calibration
        gyroscope = FusionModelInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        // accelerometer = FusionModelInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
        if (imu_init.MAG_STATE == IMU_ENABLE) {
            magnetometer = FusionModelMagnetic(magnetometer, softIronMatrix, hardIronOffset);
        }

        // Update bias algorithm
        gyroscope = FusionBiasUpdate(&bias, gyroscope);

        // Update AHRS algorithm
        if (imu_init.MAG_STATE == IMU_ENABLE) {
            FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, imu_param.deltaTime);
        } else {
            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, imu_param.deltaTime);
        }

        // Print AHRS outputs
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        // const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
        if (flags.initialising == 1) {
            flags = FusionAhrsGetFlags(&ahrs);
        } else {
            if (imu_ok_flag == 0) imu_ok_flag = 1;
            imu_data.acc.acc_x = imu_data_raw.acc_x, imu_data.acc.acc_y = imu_data_raw.acc_y, imu_data.acc.acc_z = imu_data_raw.acc_z;
            imu_data.gyro.gyro_x = imu_data_raw.gyro_x, imu_data.gyro.gyro_y = imu_data_raw.gyro_y, imu_data.gyro.gyro_z = imu_data_raw.gyro_z;
            imu_data.mag.mag_x = mag_data.mag_x, imu_data.mag.mag_y = mag_data.mag_y, imu_data.mag.mag_z = mag_data.mag_z;
            imu_data.bmp.altitude = bmp_data.altitude, imu_data.bmp.pressure = bmp_data.pressure;
            imu_data.angle.roll = euler.angle.roll, imu_data.angle.pitch = euler.angle.pitch, imu_data.angle.yaw = euler.angle.yaw;
            imu_data.temp.bmp_temp = bmp_data.temperature, imu_data.temp.imu_temp = imu_data_raw.temp, imu_data.temp.mag_temp = mag_data.temperature;
            imu_data.temp.bsp_temp = app_sensor_GetTMP117Data();
        }
    }
}
