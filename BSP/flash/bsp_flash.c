/**
 * @file bsp_flash.c
 * @brief BSP_FLASH库 基于EasyFLASH库和SFUD库 用于FLASH的基本操作 读 写 删 擦 查看信息
 * 更多功能请查看esayflash.h和sfud.h中的函数，调用底层函数实现更多功能
 *
 * Please open the file with UTF-8 encoding.
 *
 * 如果要更换FLASH 主控 或者进行调试, 需要修改一下文件
 * sfud_cfg.h 添加或者修改INDEX 修改宏定义
 * sfud_port.c 修改串口枚举 添加或者修改spi接口 修改或者添加sfud_spi_port_init,该文件需要delay_us(us)进行延时
 * ef_cfg.h 修改宏定义,修改ENV_AREA_SIZE后,需要重新查询之前环境变量的数据，初始化较长
 * ef_port.c 修改函数接口 修改sfud设备接口 修改默认环境变量
 *
 * @author Yu_Jie
 * @date 2025/12/30
 */

#include "bsp_flash.h"
#include "easyflash.h"
#include "sfud.h"


/**
 * @brief 初始化Flash存储功能
 *
 * 需要再RTOS之前初始化
 * @return bsp_status_t 返回初始化结果
 *         - BSP_OK: 初始化成功
 *         - BSP_ERR: 初始化失败
 */
bsp_status_t bsp_flash_init(void) {
    bsp_status_t result =
            easyflash_init() == EF_NO_ERR ? BSP_OK : BSP_ERR;
    return result;
}


/**
 * @brief 设置设备序列号到Flash存储中
 *
 * @return bsp_status_t 返回操作结果状态
 *         - BSP_OK: 操作成功
 *         - BSP_ERR: 操作失败
 */
bsp_status_t bsp_flash_SetSN(void) {
    bsp_status_t result =
            ef_set_env_blob("SN", FLASH_SN, 32 * sizeof(uint8_t)) == EF_NO_ERR ? BSP_OK : BSP_ERR;
    return result;
}



/**
 * @brief 从Flash中读取指定键值的环境变量数据
 *w
 * @param key 环境变量的键值字符串，用于标识要读取的数据
 * @param buf 用于存储读取数据的目标缓冲区指针
 * @param len 目标缓冲区的大小，限制读取数据的最大长度
 *
 * @return uint32_t 实际读取的数据大小，如果读取失败则返回0
 */
uint32_t bsp_flash_read(const char *key, void *buf, size_t len) {
    uint32_t size =
            ef_get_env_blob(key, buf, len, NULL);
    return size;
}


/**
 * @brief 写入数据到Flash存储中
 *
 * @param key 存储数据的键名，用于后续读取和识别数据
 * @param buf 要写入的数据缓冲区指针
 * @param len 要写入的数据长度（字节数）
 *
 * @return bsp_status_t 返回操作状态
 *         - BSP_OK: 写入成功
 *         - BSP_ERR: 写入失败
 */
bsp_status_t bsp_flash_write(const char *key, void *buf, size_t len) {
    bsp_status_t result =
            ef_set_env_blob(key, buf, len) == EF_NO_ERR ? BSP_OK : BSP_ERR;
    return result;
}


/**
 * @brief 删除Flash中指定键值的环境变量
 *
 * @param key 要删除的环境变量的键名，不能为空指针
 * @return bsp_status_t 返回操作状态
 *         - BSP_OK: 删除成功
 *         - BSP_ERR: 删除失败
 */
bsp_status_t bsp_flash_delete(const char *key) {
    bsp_status_t result =
            ef_del_env(key) == EF_NO_ERR ? BSP_OK : BSP_ERR;
    return result;
}


/**
 * @brief 对Flash芯片执行全片擦除操作，格式化芯片
 * 才函数目前有小问题，当擦除较大芯片时，会占用大量时间，此时会返回BSP_ERR，但flash仍然在擦除。
 * 每次全片擦除操作耗时较长，请勿频繁调用，且格式结束，flash需要重新校验扇区，时间较长
 *
 * @return bsp_status_t 返回操作结果状态
 */
bsp_status_t bsp_flash_ChipErase(void) {
    bsp_status_t result =
        ef_chip_erase() == EF_NO_ERR ? BSP_OK : BSP_ERR;
    return result;
}

/**
 * @brief 擦除指定地址范围的flash存储区域
 * @param addr 要擦除的起始地址
 * @param size 要擦除的字节数大小
 * @return bsp_status_t 返回操作状态，BSP_OK表示成功，BSP_ERR表示失败
 */
bsp_status_t bsp_flash_Erase(uint32_t addr, size_t size) {
    bsp_status_t result =
        ef_port_erase(addr, size) == EF_NO_ERR ? BSP_OK : BSP_ERR;
    return result;
}


/**
 * @brief 打印Flash状态信息
 *
 * 该函数用于打印当前Flash的环境信息，包括存储状态、配置信息等
 * 调用该函数需要进行宏定义 PRINT_INFO 并且修改串口枚举
 *
 * @param void 无参数
 * @return void 无返回值
 */
void bsp_flash_print_status(void) {
    ef_print_env();
}
