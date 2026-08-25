//
// Created by Luminescence on 2025/7/8.
//
#include "icm45686.h"
#include "bsp_sys.h"
#include "inv_imu_driver.h"
#include "spi.h"

// ==========================需要修改的部分=======================================
static const SPI_HandleTypeDef* icm45686_spi = &hspi2;
static const uint32_t time_out = 10;
#define ICM45686_CS_LOW()    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)
#define ICM45686_CS_HIGH()   HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)
static void (*icm45686_delay_ms)(uint32_t ms) = HAL_Delay;

// ===============================变量定义=======================================
inv_imu_device_t imu_dev = {0}; /* Driver structure */
imu_parameters_t imu_param = {0};

// ===============================内部函数声明=======================================
#define ABS(x) ((x) < 0 ? -(x) : (x))
static int icm45686_write_regs(uint8_t reg, const uint8_t* buf, uint32_t len);
static int icm45686_read_regs(uint8_t reg, uint8_t* buf, uint32_t len);
static int ICM45686_Init_Reg(const ICM45686_InitTypeDef* ICM45686_init_Struct);
static void Sensitivity_Config(const ICM45686_InitTypeDef* ICM45686_init_Struct, imu_parameters_t* icm45686_param);
static imu_status_e ICM45686_SoftCalibrate(uint16_t sample_times);


// <==================================================顶层常用函数=====================================================>

/**
 * @brief 初始化 ICM45686 IMU 传感器
 * @return imu_status_e 初始化结果，成功返回 0，失败返回错误码
 */
imu_status_e ICM45686_Init(uint16_t sample_times) {
    int rc = 0;
    uint8_t whoami;
    imu_dev.transport.read_reg = icm45686_read_regs;
    imu_dev.transport.write_reg = icm45686_write_regs;
    imu_dev.transport.sleep_us = delay_u32_us;
    imu_dev.transport.serif_type = UI_SPI4;
    icm45686_delay_ms(100); // 等待ICM45686供电稳定
    inv_imu_soft_reset(&imu_dev);
    icm45686_delay_ms(100);
    rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
    SI_CHECK_RC(rc);
    if (whoami != INV_IMU_WHOAMI)
        return INV_IMU_ERROR;
    ICM45686_InitTypeDef ICM45686_init_Struct = {0};
    // 外部中断引脚配置
    ICM45686_init_Struct.int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH; // 高电平
    ICM45686_init_Struct.int_pin_config.int_drive = INTX_CONFIG2_INTX_DRIVE_PP; // 推挽输出
    ICM45686_init_Struct.int_pin_config.int_mode = INTX_CONFIG2_INTX_MODE_PULSE; // 脉冲模式
    ICM45686_init_Struct.int_state_config.INV_UI_DRDY = INV_IMU_ENABLE; // 外部中断开启

    ICM45686_init_Struct.acc_range = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G; // 加速度计：4g
    ICM45686_init_Struct.gyro_range = GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS; // 角速度计：500dps
    ICM45686_init_Struct.acc_sample_rate = ACCEL_CONFIG0_ACCEL_ODR_200_HZ; // 加速度频率：200hz
    ICM45686_init_Struct.gyro_sample_rate = GYRO_CONFIG0_GYRO_ODR_200_HZ; // 角速度频率：200hz
    ICM45686_init_Struct.acc_lpfbw = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4; // 加速度低通滤波：4
    ICM45686_init_Struct.gyro_lpfbw = IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4; // 角速度低通滤波：4
    ICM45686_init_Struct.low_power_mode = SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC; // 低功耗设置
    ICM45686_init_Struct.acc_mode = PWR_MGMT0_ACCEL_MODE_LN; // 加速度计模式：低噪声
    ICM45686_init_Struct.gyro_mode = PWR_MGMT0_GYRO_MODE_LN; // 角速度计模式：低噪声
    rc |= ICM45686_Init_Reg(&ICM45686_init_Struct);
    Sensitivity_Config(&ICM45686_init_Struct, &imu_param);
    icm45686_delay_ms(1000);
    if (rc != 0) {
        imu_param.init_status = ICM45686_ERR;
    }
    imu_param.init_status = ICM45686_INIT_OK;
    imu_param.gyro_offset[0] = 0.0f, imu_param.gyro_offset[1] = 0.0f, imu_param.gyro_offset[2] = 0.0f;
    if (sample_times != 0)
        imu_param.init_status = ICM45686_SoftCalibrate(sample_times);
    SI_CHECK_RC(rc);
    return imu_param.init_status;
}


/**
 * @brief 获取 ICM45686 原始传感器数据（加速度、角速度、温度）
 * @param raw 存储原始数据的结构体指针，包含三轴加速度、三轴角速度和温度
 * @return int 读取结果，成功返回 0，失败返回错误码
 */
int ICM45686_GetRawData(imu_data_raw_t* raw) {
    int rc = 0;
    if (imu_param.init_status == ICM45686_ERR)
        return INV_IMU_ERROR;
    inv_imu_sensor_data_t d;
    rc |= inv_imu_get_register_data(&imu_dev, &d);
    SI_CHECK_RC(rc);
    raw->acc_x = (float)d.accel_data[0] * imu_param.acc_sensitivity;
    raw->acc_y = (float)d.accel_data[1] * imu_param.acc_sensitivity;
    raw->acc_z = (float)d.accel_data[2] * imu_param.acc_sensitivity;
    raw->gyro_x = (float)d.gyro_data[0] * imu_param.gyro_sensitivity;
    raw->gyro_y = (float)d.gyro_data[1] * imu_param.gyro_sensitivity;
    raw->gyro_z = (float)d.gyro_data[2] * imu_param.gyro_sensitivity;
    raw->temp = (float)(25 + (d.temp_data / 128.0));
    // if (ABS(raw->gyro_z) < 0.03f) {
    //     raw->gyro_z = 0.0f;
    // }
    return rc;
}

/**
 * @brief 配置 ICM45686 的寄存器参数（中断、量程、采样率、滤波等）
 * @param ICM45686_init_Struct 包含所有初始化参数的结构体指针
 * @return int 配置结果，成功返回 0，失败返回错误码
 */
static int ICM45686_Init_Reg(const ICM45686_InitTypeDef* ICM45686_init_Struct) {
    int rc = 0;
    inv_imu_int_pin_config_t int_pin_config;
    inv_imu_int_state_t int_config;

    int_pin_config.int_polarity = ICM45686_init_Struct->int_pin_config.int_polarity;
    int_pin_config.int_mode = ICM45686_init_Struct->int_pin_config.int_mode;
    int_pin_config.int_drive = ICM45686_init_Struct->int_pin_config.int_drive;
    rc |= inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int_pin_config);
    SI_CHECK_RC(rc);

    memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
    int_config.INV_UI_DRDY = ICM45686_init_Struct->int_state_config.INV_UI_DRDY;
    rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int_config);

    rc |= inv_imu_set_accel_fsr(&imu_dev, ICM45686_init_Struct->acc_range);
    rc |= inv_imu_set_gyro_fsr(&imu_dev, ICM45686_init_Struct->gyro_range);

    rc |= inv_imu_set_accel_frequency(&imu_dev, ICM45686_init_Struct->acc_sample_rate);
    rc |= inv_imu_set_gyro_frequency(&imu_dev, ICM45686_init_Struct->gyro_sample_rate);

    rc |= inv_imu_set_accel_ln_bw(&imu_dev, ICM45686_init_Struct->acc_lpfbw);
    rc |= inv_imu_set_gyro_ln_bw(&imu_dev, ICM45686_init_Struct->gyro_lpfbw);

    rc |= inv_imu_select_accel_lp_clk(&imu_dev, ICM45686_init_Struct->low_power_mode);
    rc |= inv_imu_set_accel_mode(&imu_dev, ICM45686_init_Struct->acc_mode);
    rc |= inv_imu_set_gyro_mode(&imu_dev, ICM45686_init_Struct->gyro_mode);
    SI_CHECK_RC(rc);
    return rc;
}


/**
 * @brief 配置 ICM45686 的灵敏度参数（陀螺仪、加速度计量程、采样频率等）
 * @param ICM45686_init_Struct 包含初始化配置的结构体指针
 * @param icm45686_param 存储计算后灵敏度参数的结构体指针
 */
static void Sensitivity_Config(const ICM45686_InitTypeDef* ICM45686_init_Struct, imu_parameters_t* icm45686_param) {
    switch (ICM45686_init_Struct->gyro_range) {
    case GYRO_CONFIG0_GYRO_UI_FS_SEL_15_625_DPS:
        icm45686_param->gyro_sensitivity = 15.625f / 32768.0f;
        icm45686_param->gyro_range = 15.625f;
        break;
    case GYRO_CONFIG0_GYRO_UI_FS_SEL_31_25_DPS:
        icm45686_param->gyro_sensitivity = 31.25f / 32768.0f;
        icm45686_param->gyro_range = 31.25f;
        break;
    case GYRO_CONFIG0_GYRO_UI_FS_SEL_62_5_DPS:
        icm45686_param->gyro_sensitivity = 62.5f / 32768.0f;
        icm45686_param->gyro_range = 62.5f;
        break;
    case GYRO_CONFIG0_GYRO_UI_FS_SEL_125_DPS:
        icm45686_param->gyro_sensitivity = 125.0f / 32768.0f;
        icm45686_param->gyro_range = 125.0f;
        break;
    case GYRO_CONFIG0_GYRO_UI_FS_SEL_250_DPS:
        icm45686_param->gyro_sensitivity = 250.0f / 32768.0f;
        icm45686_param->gyro_range = 250.0f;
        break;
    case GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS:
        icm45686_param->gyro_sensitivity = 500.0f / 32768.0f;
        icm45686_param->gyro_range = 500.0f;
        break;
    case GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS:
        icm45686_param->gyro_sensitivity = 1000.0f / 32768.0f;
        icm45686_param->gyro_range = 1000.0f;
        break;
    case GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS:
        icm45686_param->gyro_sensitivity = 2000.0f / 32768.0f;
        icm45686_param->gyro_range = 2000.0f;
        break;
    default:
        icm45686_param->gyro_sensitivity = 0.0f;
        icm45686_param->gyro_range = 0.0f;
    }
    switch (ICM45686_init_Struct->acc_range) {
    case ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G: icm45686_param->acc_sensitivity = 2.0f / 32768.0f;
        break;
    case ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G: icm45686_param->acc_sensitivity = 4.0f / 32768.0f;
        break;
    case ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G: icm45686_param->acc_sensitivity = 8.0f / 32768.0f;
        break;
    case ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G: icm45686_param->acc_sensitivity = 16.0f / 32768.0f;
        break;
    default: icm45686_param->acc_sensitivity = 0.0f;
    }
    switch (ICM45686_init_Struct->acc_sample_rate) {
    case ACCEL_CONFIG0_ACCEL_ODR_1_5625_HZ: icm45686_param->sample_freq = 1.5625f;
        break;
    case ACCEL_CONFIG0_ACCEL_ODR_3_125_HZ: icm45686_param->sample_freq = 3.125f;
        break;
    case ACCEL_CONFIG0_ACCEL_ODR_6_25_HZ: icm45686_param->sample_freq = 6.25f;
        break;
    case ACCEL_CONFIG0_ACCEL_ODR_12_5_HZ: icm45686_param->sample_freq = 12.5f;
        break;
    case ACCEL_CONFIG0_ACCEL_ODR_25_HZ: icm45686_param->sample_freq = 25.0f;
        break;
    case ACCEL_CONFIG0_ACCEL_ODR_50_HZ: icm45686_param->sample_freq = 50.0f;
        break;
    case ACCEL_CONFIG0_ACCEL_ODR_100_HZ: icm45686_param->sample_freq = 100.0f;
        break;
    case ACCEL_CONFIG0_ACCEL_ODR_200_HZ: icm45686_param->sample_freq = 200.0f;
        break;
    case ACCEL_CONFIG0_ACCEL_ODR_400_HZ: icm45686_param->sample_freq = 400.0f;
        break;
    case ACCEL_CONFIG0_ACCEL_ODR_800_HZ: icm45686_param->sample_freq = 800.0f;
        break;
    case ACCEL_CONFIG0_ACCEL_ODR_1600_HZ: icm45686_param->sample_freq = 1600.0f;
        break;
    case ACCEL_CONFIG0_ACCEL_ODR_3200_HZ: icm45686_param->sample_freq = 3200.0f;
        break;
    case ACCEL_CONFIG0_ACCEL_ODR_6400_HZ: icm45686_param->sample_freq = 6400.0f;
        break;
    default: icm45686_param->sample_freq = 0;
    }
    icm45686_param->deltaTime = 1.0f / icm45686_param->sample_freq;
    icm45686_param->sample_time = icm45686_param->deltaTime * 1000;
}

static imu_status_e ICM45686_SoftCalibrate(uint16_t sample_times) {
	float sum_gx = 0.00f, sum_gy = 0.00f, sum_gz = 0.00f;
	imu_data_raw_t raw;
	for (int i = 0; i < sample_times; i++) {
		ICM45686_GetRawData(&raw);
		sum_gx += raw.gyro_x,sum_gy += raw.gyro_y,sum_gz += raw.gyro_z;
		icm45686_delay_ms((uint16_t)imu_param.sample_time);
	}
	imu_param.gyro_offset[0] = sum_gx / (float)sample_times;
	imu_param.gyro_offset[1] = sum_gy / (float)sample_times;
    imu_param.gyro_offset[2] = sum_gz / (float)sample_times;
	return ICM45686_CALI_OK;
}

// <===========================================底层驱动函数====================================================>

/**
 * @brief 通过 SPI 接口向一个 ICM45686 寄存器写入数据
 * @param reg 要写入的寄存器地址
 * @param value 要写入寄存器的值
 * @return int 写入结果，成功返回 INV_IMU_OK，失败返回 INV_IMU_ERROR_TRANSPORT
 */
static int icm45686_write_reg(uint8_t reg, uint8_t value) {
    uint8_t read_buf_null = 0;
    ICM45686_CS_LOW();
    if (HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)icm45686_spi, &reg, &read_buf_null, 1, time_out) != HAL_OK)
        return INV_IMU_ERROR_TRANSPORT;
    if (HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)icm45686_spi, &value, &read_buf_null, 1, time_out) != HAL_OK)
        return INV_IMU_ERROR_TRANSPORT;
    ICM45686_CS_HIGH();
    return INV_IMU_OK;
}

/**
 * @brief 向 ICM45686 连续写入多个寄存器数据
 * @param reg 起始寄存器地址
 * @param buf 要写入的数据缓冲区指针
 * @param len 要写入的数据长度（字节数）
 * @return int 写入结果，全部成功返回 INV_IMU_OK，任一失败返回 INV_IMU_ERROR_TRANSPORT
 */
static int icm45686_write_regs(uint8_t reg, const uint8_t* buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        if (icm45686_write_reg(reg + i, buf[i]) != INV_IMU_OK) {
            return INV_IMU_ERROR_TRANSPORT;
        }
    }
    return INV_IMU_OK;
}

/**
 * @brief 从 ICM45686 连续读取多个寄存器数据
 * @param reg 起始寄存器地址
 * @param buf 读取数据存储缓冲区指针
 * @param len 要读取的数据长度（字节数）
 * @return int 读取结果，成功返回 INV_IMU_OK，失败返回 INV_IMU_ERROR_TRANSPORT
 */
static int icm45686_read_regs(uint8_t reg, uint8_t* buf, uint32_t len) {
    uint8_t read_buf_null = 0;
    reg |= 0x80;
    ICM45686_CS_LOW();
    if (HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)icm45686_spi, &reg, &read_buf_null, 1, time_out) != HAL_OK)
        return INV_IMU_ERROR_TRANSPORT;
    while (len--) {
        if (HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)icm45686_spi, &reg, buf, 1, time_out) != HAL_OK)
            return INV_IMU_ERROR_TRANSPORT;
        buf++;
    }
    ICM45686_CS_HIGH();
    return INV_IMU_OK;
}
