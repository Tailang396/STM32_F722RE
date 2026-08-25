//
// Created by Yu_Jie on 2025/12/20.
//

#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include "bsp_def.h"

#ifdef __cplusplus
extern "C" {

#endif

//小于32个字节
#define FLASH_SN "SN0000-W25Q128JV"

bsp_status_t bsp_flash_init(void);
bsp_status_t bsp_flash_SetSN(void);
uint32_t bsp_flash_read(const char *key, void *buf, size_t len);
bsp_status_t bsp_flash_write(const char *s, void *buf, size_t len);
bsp_status_t bsp_flash_delete(const char *key);
bsp_status_t bsp_flash_Erase(uint32_t addr, size_t size);
bsp_status_t bsp_flash_ChipErase(void);
void bsp_flash_print_status(void);

#ifdef __cplusplus
}
#endif
#endif //BSP_FLASH_H
