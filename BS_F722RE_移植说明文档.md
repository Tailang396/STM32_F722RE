# BS_F722RE 传感器驱动与姿态解算 移植说明文档

> 适用工程：`C:\Users\asus\Desktop\BS_F722RE`
> 编写目的：说明本项目包含的传感器驱动、Flash 存储（SFUD + EasyFlash）移植方式，以及姿态解算（Fusion 库）在移植到其他项目时所需修改 / 删除的接口。

---

## 1. 项目简介

本项目为**本科毕业设计**，主控为 **STM32F722RE**，通过 **SPI / I2C** 总线驱动多种传感器，并移植了 **SFUD**（串行 Flash 通用驱动）、**EasyFlash**（轻量级 KV 存储）和 **Fusion**（开源姿态解算库）等第三方库。

**硬件外设资源分配：**

| 外设 | 挂载设备 | 说明 |
|---|---|---|
| SPI1 + CS | W25Q128JV（Flash） | 存储，16MB |
| SPI2 + CS | ICM45686（六轴 IMU） | 加速度计 + 陀螺仪 |
| I2C1 | HSCDTD008A（磁力计） | 三轴磁场，地址 `0x0C` |
| I2C2 | BMP280（气压计） | 气压 / 高度 / 温度，地址 `0x76` |
| I2C3 | TMP117、SHT40 | TMP117 高精度温度（`0x48`）；SHT40 温湿度 |
| TIM3 CH4 PWM | 加热负载 | 恒温加热控制 |
| TIM7 | — | `delay_us()` 微秒延时基准 |
| TIM14 | — | 采样定时中断（驱动姿态解算） |
| MOD1 / MOD2 GPIO | 拨码开关 | 选择"恒温加热" / "磁力计"是否使能 |
| UART1 | 调试串口 | 日志输出（`E_UART_DEBUG`） |
| RGB LED | — | 初始化状态指示 |

**可输出数据：**
- **六轴模式**（仅 IMU）：加速度、角速度、姿态角（roll / pitch / yaw）
- **九轴模式**（IMU + 磁力计）：在上述基础上加入磁场数据并参与解算，得到带磁航向的姿态角
- 附加：气压高度、IMU / 磁力计 / 板载温度
- 特性：启动时可选**恒温加热**（TMP117 测温 + PID 控制加热到 40℃，使陀螺零偏稳定）

---

## 2. 工程分层结构

```
BS_F722RE/
├── Core/                 STM32CubeMX 生成：main.c、中断回调、启动文件
├── Drivers/              CMSIS 核心头文件 + STM32F7xx HAL 标准库
├── Middlewares/          FreeRTOS 实时内核 + CMSIS-RTOS V2 封装
├── BSP/                  板级支持包（本工程核心，全部业务驱动在此）
│   ├── imu/
│   │   ├── icm45686/           六轴 IMU 驱动（含 InvenSense BasicDriver 官方库）
│   │   ├── hscdtd008a/         三轴磁力计驱动
│   │   ├── bmp280/             气压计驱动
│   │   ├── fusion/             Fusion 姿态解算库（xioTechnologies，官方源码）
│   │   ├── bsp_imu.c/.h        传感器汇总 + 解算 + 恒温加热 + 九/六轴切换
│   ├── flash/
│   │   ├── sfud/               SFUD 库（src/ 源码，port/ 移植接口）
│   │   ├── easyflash/          EasyFlash 库（src/ 源码，port/ 移植接口）
│   │   └── bsp_flash.c/.h      基于 EasyFlash 的 KV 读写封装
│   ├── tmp117/  sht40/         温度 / 温湿度传感器驱动
│   ├── sys/  uart/  rgb/       系统延时 / 串口 / LED
│   ├── can/  gps/  adc/        扩展外设
│   └── lcd_ST7789/             LCD 显示驱动
├── APP/                   应用层（FreeRTOS 任务）
│   ├── sys/    app_sys.c        系统初始化（调用 bsp_flash_init、bsp_imu_init）
│   ├── imu/    app_imu.c        定时器中断回调 → fusion_update / 温度控制
│   ├── sensor/ app_sensor.c     传感器采样任务（SHT40 / TMP117 / ADC）
│   ├── comm/   gps/  lcdshow/   通信 / GPS / 显示任务
└── Components/            组件算法（如 CRC）
```

**依赖关系（分层明确）：**

```
APP（应用层，管理任务与初始化顺序）
  │  调用
  ▼
BSP（板级支持包，驱动具体芯片）
  │  调用
  ▼
Drivers / Middlewares（HAL 库 + FreeRTOS）
```

- **BSP 层内部再分层**：`bsp_xxx`（整合层，如 `bsp_imu`）→ 单传感器驱动（`icm45686.c` 等）→ 第三方库源码（`fusion/`、`sfud/`、`easyflash/`）。
- **移植时**：APP 层基本不用动，主要改 BSP 层各驱动文件顶部的"硬件接口"部分，以及 CubeMX 生成的外设句柄。

---

## 3. 传感器驱动清单与移植接口

> 所有传感器驱动都有同一个移植套路：**改总线句柄 / CS 引脚 / I2C 地址 / 延时函数 / RTOS 宏**。下面逐个说明。

### 3.1 ICM45686 六轴 IMU —— SPI2

**对外接口（`BSP/imu/icm45686/icm45686.h`）：**

```c
imu_status_e ICM45686_Init(uint16_t sample_times);   // 初始化 + 软校准（0 表示不校准）
int ICM45686_GetRawData(imu_data_raw_t *raw);         // 读加计/陀螺/温度（已换算物理量）
```

**移植需要修改的地方**（`icm45686.c` 顶部第 9~14 行，"需要修改的部分"段）：

```c
static const SPI_HandleTypeDef* icm45686_spi = &hspi2;            // ① SPI 句柄
#define ICM45686_CS_LOW()  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)
#define ICM45686_CS_HIGH() HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)
static void (*icm45686_delay_ms)(uint32_t ms) = HAL_Delay;        // ③ 毫秒延时
```

以及在 `ICM45686_Init()` 内绑定的底层传输回调：

```c
imu_dev.transport.read_reg  = icm45686_read_regs;     // SPI 寄存器读
imu_dev.transport.write_reg = icm45686_write_regs;    // SPI 寄存器写
imu_dev.transport.sleep_us  = delay_u32_us;           // 微秒延时（bsp_sys 提供）
imu_dev.transport.serif_type = UI_SPI4;               // 4 线 SPI；若用 I2C 改成 UI_I2C
```

> `icm45686_read_regs / write_regs` 内部用 `HAL_SPI_TransmitReceive()` 实现，本质就是"片选拉低 → 传地址 → 传/收数据 → 片选拉高"，换平台时重写这两个函数即可。
>
> `BasicDriver/` 目录是 InvenSense 官方寄存器驱动库，**无需修改**，随目录拷贝即可。

**可配置的初始化参数**（量程、采样率、滤波等）：见 `ICM45686_InitTypeDef` 结构体，常用修改点：

```c
ICM45686_init_Struct.acc_range        = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G;     // 加计量程
ICM45686_init_Struct.gyro_range       = GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS;  // 陀螺量程
ICM45686_init_Struct.acc_sample_rate  = ACCEL_CONFIG0_ACCEL_ODR_200_HZ;        // 采样率
ICM45686_init_Struct.gyro_sample_rate = GYRO_CONFIG0_GYRO_ODR_200_HZ;
```

**软校准**：`sample_times != 0` 时，静止采集若干次陀螺数据取平均，得到零偏存入 `imu_param.gyro_offset[3]`（供 Fusion 库使用）。

### 3.2 HSCDTD008A 三轴磁力计 —— I2C1

**对外接口（`BSP/imu/hscdtd008a/hscdtd008a.h`）：**

```c
HSCDTD008A_status_t hscdtd008a_Init(void);                  // 初始化（含自检）
void   hscdtd008a_GetData(HSCDTD008A_data_t* data);         // 读三轴磁场 + 温度（单位 µT）
void   hscdtd008a_GetStatus(HSCDTD008A_data_t* data);       // 读数据更新/溢出状态
HSCDTD008A_status_t hscdtd008a_ForceMeasure(void);          // 单次强制测量（一次写入同时触发磁场+温度）
```

**移植要改的地方：**

```c
static const I2C_HandleTypeDef* hscdtd008a_iic = &hi2c1;    // ① I2C 句柄
#define HSCDTD008A_I2C_ADDR 0x0C                            // ② 器件地址（接线不同要改）
#define HSCDTD008A_DEV_ID   0x49                            // ③ 器件 ID（WHO_AM_I）
// #define HSCDTD008A_USE_FREERTOS                            // ④ 在 RTOS 任务内初始化则打开
```

> 底层使用 `HAL_I2C_Mem_Read / Mem_Write`，换平台改 I2C 句柄即可。

### 3.3 BMP280 气压计 —— I2C2

**对外接口：** `bmp280_Init()` / `bmp280_GetData(BMP280_data_t*)` / `bmp280_reset()`

**移植要改的地方：**

```c
static const I2C_HandleTypeDef *bmp280_iic = &hi2c2;        // ① I2C 句柄
#define BMP280_I2C_ADDR 0x76                                // ② 器件地址（SDO 接法决定 0x76/0x77）
#define BMP280_DEV_ID   0x58
```

### 3.4 TMP117 高精度温度计 —— I2C3

**对外接口：** `tmp117_Init()` / `tmp117_GetData()` / `tmp117_reset()` / `tmp117_GetStatus()` / `tmp117_SetHighLimit()` / `tmp117_SetLowLimit()`

**移植要改的地方：**

```c
static const I2C_HandleTypeDef* tmp117_iic = &hi2c3;        // ① I2C 句柄
#define TMP117_I2C_ADDR 0x48                                // ② 器件地址（ADD0 引脚决定）
```

> 本工程中 TMP117 同时用于两处：**恒温加热的温控反馈** 和 **板温数据显示**。

### 3.5 SHT40 温湿度 —— I2C3

**对外接口：** `sht40_Init()` / `sht40_ReadData(&data, SHT40_PRECISION_HIGH)`

**移植要改的地方：**

```c
static const I2C_HandleTypeDef *sht40_i2c = &hi2c3;         // I2C 句柄
#define SHT40_I2C_ADDR 0x44
```

### 3.6 传感器移植点汇总表

| 传感器 | 总线 | 句柄变量 | 地址 | 需改的核心文件 |
|---|---|---|---|---|
| ICM45686 | SPI2 | `&hspi2` + `SPI2_CS_*` | — | `icm45686.c` 顶部 |
| HSCDTD008A | I2C1 | `&hi2c1` | `0x0C` | `hscdtd008a.c` 顶部 |
| BMP280 | I2C2 | `&hi2c2` | `0x76` | `bmp280.c` 顶部 |
| TMP117 | I2C3 | `&hi2c3` | `0x48` | `tmp117.c` 顶部 |
| SHT40 | I2C3 | `&hi2c3` | `0x44` | `sht40.c` 顶部 |

**公共依赖（换平台必须提供）：**
1. CubeMX 生成的 `main.h` / `i2c.h` / `spi.h` / `tim.h` 中的 HAL 句柄；
2. 调试串口打印 `bsp_uart_printf()`（驱动报错用，可注释掉）；
3. 若在 FreeRTOS 任务内初始化，打开对应驱动的 `xxx_USE_FREERTOS` 宏（内部用 `osDelay` 代替 `HAL_Delay`）。

---

## 4. Flash 存储移植（SFUD + EasyFlash + bsp_flash）

本工程用 **SFUD** 识别并驱动 W25Q128JV（SPI1），在其上运行 **EasyFlash** 提供"键值对（KV）"存储，再用 **bsp_flash** 封装成简易 API。

```
bsp_flash（KV 读写封装）
   │ 调用
EasyFlash（ENV 键值存储、磨损均衡）
   │ 调用
SFUD（识别/擦/写/读 Flash 芯片）
   │ 调用
HAL SPI（SPI1 + CS）
```

### 4.1 SFUD 移植 —— 需改两个文件

**`BSP/flash/sfud/inc/sfud_cfg.h`（配置文件）：**

```c
enum {
    SFUD_W25Q128JV_DEVICE_INDEX = 0,      // ① 设备序号：换 Flash 芯片可在此增加
};

#define SFUD_FLASH_DEVICE_TABLE \
{                                                                     \
    [SFUD_W25Q128JV_DEVICE_INDEX] = {.name = "W25Q128JV", .spi.name = "SPI1"}, \
}

#define SFUD_USING_SFDP    // 使用芯片 SFDP 表自动识别参数（新芯片推荐保留）
// #define SFUD_USING_FLASH_INFO_TABLE   // 老芯片不支持 SFDP 时用内置表格
// #define SFUD_USING_FAST_READ
// #define SFUD_USING_QSPI               // 用 QSPI 则打开
```

**`BSP/flash/sfud/port/sfud_port.c`（硬件接口）：**

```c
// ① 定义 SPI 接口（换 SPI 总线 / CS 引脚改这里）
static spi_user_data spi_w25q128jv = {
    .spix = &hspi1, .cs_port = SPI1_CS_GPIO_Port, .cs_pin = SPI1_CS_Pin
};

// ② 底层收发函数（HAL_SPI_Transmit/Receive，CS 拉低→收发→CS 拉高）
static sfud_err spi_write_read(const sfud_spi *spi, const uint8_t *write_buf,
                               size_t write_size, uint8_t *read_buf, size_t read_size);

// ③ 设备绑定：switch(flash->index) 把 ① ② 挂到对应 Flash 上
sfud_err sfud_spi_port_init(sfud_flash *flash) {
    switch (flash->index) {
    case SFUD_W25Q128JV_DEVICE_INDEX:
        flash->spi.wr = spi_write_read;        // 必填
        flash->spi.user_data = &spi_w25q128jv; // 必填
        flash->retry.delay = retry_delay_100us; // 重试延时
        flash->retry.times = 10000;             // 必填
        break;
    default:
        result = SFUD_ERR_NOT_FOUND;
    }
    return result;
}
```

**SFUD 移植需要额外提供的两样东西：**
1. **`delay_us()` 微秒延时函数**（sfud_port 的 `retry_delay_100us` 调用它）。本工程在 `bsp_sys.c` 中用 **TIM7 计数器**实现，换平台可改用 DWT 或 SysTick 空闲计数：
   ```c
   void delay_us(uint16_t us);
   void delay_u32_us(uint32_t us);
   ```
2. 串口日志 `bsp_uart_printf()`（`sfud_log_debug / sfud_log_info`），不需要可注释掉宏 `SFUD_LOG_PRINTF`。

### 4.2 EasyFlash 移植 —— 需改两个文件

**`BSP/flash/easyflash/inc/ef_cfg.h`（配置文件）：**

```c
#define EF_ERASE_MIN_SIZE 4096              // ① 最小擦除单元（= Flash 扇区大小，W25Q128JV 为 4KB）
#define EF_WRITE_GRAN     1                 // ② 写粒度：NOR Flash 为 1(bit)
#define EF_START_ADDR     0                 // ③ 备份区起始地址（相对 Flash 的偏移）
#define ENV_AREA_SIZE     (4096 * EF_ERASE_MIN_SIZE)  // ④ ENV 区大小，至少 2 个扇区（1 个留给 GC）
#define EF_ENV_VER_NUM    0                 // ⑤ 环境变量版本号，新增默认变量后 +1
#define EF_USING_ENV                          // 只用了 ENV 功能
// #define EF_USING_IAP
// #define EF_USING_LOG
```

> **注意**：修改 `ENV_AREA_SIZE` 后，EasyFlash 需要重新格式化 ENV 区，**首次初始化耗时会明显变长**（代码注释里也强调了）。

**`BSP/flash/easyflash/port/ef_port.c`（硬件接口）：**

```c
// ① 默认环境变量表（芯片校验 SN 等）
static const ef_env default_env_set[] = {
    {"SN", "SN0000-W25Q128JV", 32 * sizeof(uint8_t)}
};

// ② 初始化：内部完成 sfud_init → 取设备 → 设备初始化
EfErrCode ef_port_init(ef_env const **default_env, size_t *default_env_size);

// ③ 三个必须实现的读写擦函数（本质是转发给 SFUD）
EfErrCode ef_port_read (uint32_t addr, uint32_t *buf, size_t size);
EfErrCode ef_port_erase(uint32_t addr, size_t size);
EfErrCode ef_port_write(uint32_t addr, const uint32_t *buf, size_t size);

// ④ 全片擦除
EfErrCode ef_chip_erase(void);

// ⑤ ENV 缓存锁（单线程可留空）
void ef_port_env_lock(void);
void ef_port_env_unlock(void);
```

`ef_port_erase / write / read` 内部就是：

```c
sfud_read(flash_w25q128, addr, size, (uint8_t*)buf);       // 读
sfud_erase(flash_w25q128, addr, size);                     // 擦
sfud_write(flash_w25q128, addr, size, (const uint8_t*)buf);// 写
```

### 4.3 bsp_flash 封装层

对外只暴露 KV 接口（`BSP/flash/bsp_flash.c`）：

| 函数 | 功能 | 底层 |
|---|---|---|
| `bsp_flash_init()` | 初始化（**须在 RTOS 启动前调用**） | `easyflash_init()` |
| `bsp_flash_write(key, buf, len)` | 写 KV | `ef_set_env_blob()` |
| `bsp_flash_read(key, buf, len)` | 读 KV | `ef_get_env_blob()` |
| `bsp_flash_delete(key)` | 删除 KV | `ef_del_env()` |
| `bsp_flash_Erase(addr, size)` | 按地址擦除 | `ef_port_erase()` |
| `bsp_flash_ChipErase()` | 全片擦除（耗时长，勿频繁调用） | `ef_chip_erase()` |
| `bsp_flash_print_status()` | 打印全部 ENV | `ef_print_env()` |
| `bsp_flash_SetSN()` | 写设备序列号 | `ef_set_env_blob("SN", ...)` |

**使用示例（本工程 `app_sys.c` 的用法）：**

```c
bsp_flash_init();
bsp_flash_read("SN", data, 32);   // 读 SN
bsp_flash_Write("SN", data, 32);  // 写 SN
```

**移植注意点：**
1. `bsp_flash_init()` 必须在 FreeRTOS 调度器启动前调用（本工程在 `sys_main_init()` 里、`osKernelStart` 之前）；
2. `bsp_flash_ChipErase()` 擦大芯片耗时很长，期间会返回 `BSP_ERR`，但 Flash 仍在擦除，**不要据此判断失败**；
3. `bsp_flash_read` 返回实际读到的字节数，**读不到返回 0**，可据此判断 KV 是否存在。

---

## 5. 姿态解算（Fusion 库）与 bsp_imu.c

### 5.1 Fusion 库接入

本工程使用开源 **Fusion** 库（xioTechnologies，Madgwick 姿态解算算法的 C 实现），源码在 `BSP/imu/fusion/`，用到了两个模块：

- **FusionAhrs**：由陀螺 + 加计（六轴）或再加磁力计（九轴）输出四元数 → 欧拉角；
- **FusionBias**：静止时估计陀螺零偏（在线），与 ICM45686 软校准的静态零偏配合。

**`fusion_init()`（bsp_imu.c）配置：**

```c
FusionBiasInitialise(&bias, (uint32_t)imu_param.sample_freq);   // 零偏估计器
FusionAhrsInitialise(&ahrs);
const FusionAhrsSettings settings = {
    .convention = FusionConventionNwu,           // 北西天坐标系
    .gain = 0.5f,                                // 融合增益（越大越信加计）
    .gyroscopeRange = imu_param.gyro_range,      // 陀螺量程（来自 IMU 配置，1000dps）
    .accelerationRejection = 10.0f,              // 加速度拒斥阈值
    .magneticRejection = 10.0f,                  // 磁场拒斥阈值
    .recoveryTriggerPeriod = 5 * sample_freq,    // 5 秒恢复触发周期
};
FusionAhrsSetSettings(&ahrs, &settings);
```

**`fusion_update()` 数据流水线：**

```
ICM45686_GetRawData ──► 陀螺/加计
HSCDTD008A_GetData ────► 磁力计（仅九轴）
BMP280_GetData ────────► 气压/温度
         │
         ▼
陀螺 = FusionModelInertial(原始, 安装误差, 灵敏度, 零偏)      // 陀螺标定
磁力 = FusionModelMagnetic(原始, 软磁矩阵, 硬磁偏移)          // 磁标定（九轴才用）
陀螺 = FusionBiasUpdate(&bias, 陀螺)                          // 在线零偏
         │
         ├─ 九轴: FusionAhrsUpdate(&ahrs, 陀螺, 加计, 磁力, dt)
         └─ 六轴: FusionAhrsUpdateNoMagnetometer(&ahrs, 陀螺, 加计, dt)
         │
         ▼
FusionAhrsGetQuaternion → FusionQuaternionToEuler → imu_data.angle(roll/pitch/yaw)
```

**磁力计校准参数**（`bsp_imu.c` 顶部）就是上面 `FusionModelMagnetic` 用的硬磁/软磁参数：

```c
static const FusionMatrix softIronMatrix = { 0.9744f, 0.0301f, ..., ... };  // 软磁矩阵
static const FusionVector hardIronOffset  = { 7.63f, -3.09f, -7.62f };       // 硬磁偏移
```

> 这两个参数需要专门做磁校准采集，移植到新板子后**必须换成自己的标定值**，否则航向会偏。

### 5.2 启动流程与恒温加热

`bsp_imu_init(&htim14)` 的启动顺序：

```
① bsp_imu_set_mode()        读 MOD1/MOD2 拨码 → 决定是否"恒温加热""启用磁力计"
② tmp117_Init()             初始化温控反馈用温度计
③ 若恒温加热使能:
   RGB蓝 + bsp_tempctrl_init()   （PID 初始化 + TIM3 PWM 启动）
   启动 TIM14 定时中断 → 进入恒温加热等待
   while(imu_tempctrl_finsh_flag == 0) 死等升温到目标温度
④ RGB青 → ICM45686_Init(600)   IMU 初始化 + 600 次软校准
⑤ hscdtd008a_Init() + bmp280_Init()
⑥ RGB绿 → fusion_init()
⑦ 启动 TIM14 定时中断 → app_imu_timer_callback() 周期性执行解算
⑧ while(imu_ok_flag == 0) 等第一帧解算完成
```

**恒温加热原理**（`bsp_tempctrl_init / bsp_tempctrl_update`）：

- 目标温度 `aim_temp = 40.0℃`，稳定判据：`|当前温度 − 目标| ≤ 0.1℃` 且连续累计 200 次；
- 控制量由 PID（`PID_TypeDef pid_temp`，参数 P=150, I=0.4, D=0）算出，经 `TIM3_CH4 PWM` 输出给加热负载；
- 加热过程中持续通过调试串口打印 `温度, 目标, PWM占空比`；
- 达到稳定温度后置 `imu_tempctrl_finsh_flag = 1`，才继续初始化 IMU —— **目的是让陀螺工作在恒温环境下，零偏更稳定**。

**周期性驱动**（`APP/imu/app_imu.c`）：

```c
void app_imu_timer_callback(void) {   // 由 TIM14 中断调用（main.c HAL_TIM_PeriodElapsedCallback）
    if (++count >= 4) {               // 每 4 次（约 50Hz）做一次温控
        count = 0;
        bsp_tempctrl_update();
    }
    fusion_update();                  // 每次都做一次解算（约 200Hz）
}
```

> 解算频率取决于 TIM14 的定时周期，与 `imu_param.deltaTime = 1/sample_freq ≈ 5ms`（IMU 200Hz 采样）匹配。

### 5.3 九轴 / 六轴切换

**硬件开关**：`bsp_imu_set_mode()` 读取拨码：

```c
MOD1 拨码（低有效）→ imu_init.TMEP_CTRL_STATE = IMU_ENABLE/DISABLE   // 恒温加热开关
MOD2 拨码（低有效）→ imu_init.MAG_STATE       = IMU_ENABLE/DISABLE   // 九轴/六轴开关
```

**软件分支**（`fusion_update()` 内）：

```c
if (imu_init.MAG_STATE == IMU_ENABLE) {           // 九轴：带磁力计
    magnetometer = FusionModelMagnetic(magnetometer, softIronMatrix, hardIronOffset);
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, imu_param.deltaTime);
} else {                                           // 六轴：无磁力计
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, imu_param.deltaTime);
}
```

**两种模式差异：**
| | 六轴（默认，MOD2 悬空） | 九轴（MOD2 接 GND） |
|---|---|---|
| 数据来源 | IMU 陀螺 + 加计 | 陀螺 + 加计 + 磁力计 |
| 函数 | `FusionAhrsUpdateNoMagnetometer` | `FusionAhrsUpdate` |
| 输出 | roll/pitch/yaw（yaw 会缓慢漂移） | roll/pitch/yaw（yaw 由磁力计锁定） |
| 前置条件 | 无 | 磁力计已校准（软/硬磁参数） |

### 5.4 移植到其他项目时的删除清单

按"精简到只需原始数据 / 六轴姿态 / 九轴姿态"三种程度，分步删除：

#### A. 删除恒温加热（推荐，除非需要恒温陀螺）

| 位置 | 删除内容 |
|---|---|
| `bsp_imu_init()` | `tmp117_Init()` 及其断言；`if (TMEP_CTRL_STATE == IMU_ENABLE)` 整个分支（含 RGB 蓝、`bsp_tempctrl_init()`、`HAL_TIM_Base_Start_IT`、等待循环） |
| `bsp_tempctrl_init()` | 整个函数（含 `PID_Init`、`HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4)`） |
| `bsp_tempctrl_update()` | 整个函数 |
| `app_imu.c` | `bsp_tempctrl_update()` 调用、`count` 计数逻辑（保留 `fusion_update()` 即可） |
| 全局变量 | `pid_temp`、`tmp117_temp`、`aim_temp`、`temp_stable_threshold`、`stable_count_required`、`imu_tempctrl_finsh_flag` |
| include | `PID.h`、`tim.h`（若 TIM3/TIM14 不再用）、`tmp117.h` |
| `bsp_imu_set_mode()` | 删除 MOD1 相关的 `TMEP_CTRL_STATE` 判断 |
| 静态配置 | `imu_init.TMEP_CTRL_STATE = IMU_DISABLE` |

> 删除后，启动流程简化为：`bsp_imu_set_mode → ICM45686_Init → hscdtd008a_Init → bmp280_Init → fusion_init → 启动 TIM14`。

#### B. 只要六轴、不要磁力计

| 位置 | 删除内容 |
|---|---|
| `bsp_imu_init()` | `hscdtd008a_Init()` |
| `fusion_update()` | `hscdtd008a_GetStatus / GetData`；`FusionModelMagnetic` 分支；`FusionAhrsUpdate`（带磁）分支，只留 `FusionAhrsUpdateNoMagnetometer` |
| 全局变量 | `mag_data`、`softIronMatrix`、`hardIronOffset` |
| `bsp_imu_set_mode()` | 删除 MOD2 相关的 `MAG_STATE` 判断 |
| 静态配置 | `imu_init.MAG_STATE = IMU_DISABLE` |
| 输出 | `imu_data.mag.*`、`imu_data.temp.mag_temp` |

#### C. 不要气压计

| 位置 | 删除内容 |
|---|---|
| `bsp_imu_init()` | `bmp280_Init()` |
| `fusion_update()` | `bmp280_GetData()` |
| 输出 | `imu_data.bmp.*`、`imu_data.temp.bmp_temp` |

#### D. 只要原始数据、完全不要姿态解算

- 删除整个 `fusion_init / fusion_update`、`Fusion.h` include、`FusionBias/FusionAhrs` 相关变量；
- 保留 `ICM45686_Init + ICM45686_GetRawData`（以及需要的磁力计/气压计），把 `fusion_update()` 替换成"直接读原始数据并发布"即可。

#### E. 保留的最小核心（六轴姿态，无加热无磁力无气压）

```c
bsp_imu_init()  精简为：
    1. ICM45686_Init(CALI_SAMPLE_TIMES)      // 含软校准
    2. fusion_init()                          // FusionAhrs + FusionBias
    3. 启动 TIM14 定时中断
fusion_update() 精简为：
    1. ICM45686_GetRawData(&raw)
    2. gyro = FusionModelInertial(gyro, 安装误差, 灵敏度, gyroscopeOffset)
    3. gyro = FusionBiasUpdate(&bias, gyro)
    4. FusionAhrsUpdateNoMagnetometer(&ahrs, gyro, acc, deltaTime)
    5. euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs))
```

**精简后必须保留的硬件依赖**：TIM14（采样中断）、SPI2 + CS（IMU）、UART（打印）；去掉加热后 **TIM3 PWM / TMP117 / MOD1** 可一并去掉。

---

## 6. 移植常见坑点速查

| 现象 | 原因 / 对策 |
|---|---|
| `bsp_flash_init` 首次运行很久 | 更换 Flash 芯片或改 `ENV_AREA_SIZE` 后 ENV 区重建，属正常 |
| Flash 全片擦除返回 `BSP_ERR` | 擦除耗时长被误判，实际仍在擦除，稍后校验即可 |
| 姿态 yaw 漂移 | 六轴模式正常现象；需要稳定 yaw 用九轴并**重新标定**软/硬磁参数 |
| 加热过程卡在初始化 | 检查 TMP117 接线（I2C3）、TIM3 PWM 是否输出、目标温度是否可达 |
| SPI 通信失败 | 检查 `icm45686_spi`、CS 引脚宏、`delay_us`（TIM7）是否初始化 |
| I2C NACK | 检查对应 `hi2cX` 句柄与器件地址（尤其 SDO/ADD0 引脚接法） |
| 在 RTOS 任务里初始化驱动挂死 | 打开对应驱动的 `xxx_USE_FREERTOS` 宏（把 `HAL_Delay` 换成 `osDelay`） |
