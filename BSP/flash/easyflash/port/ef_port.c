/*
 * This file is part of the EasyFlash Library.
 *
 * Copyright (c) 2015-2019, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-01-16
 */

#include <easyflash.h>
#include <stdarg.h>
#include "sfud.h"
#include "bsp_uart.h"

static sfud_flash *flash_w25q128 = NULL;

/* default environment variables set for user */
#define EF_UART_DEBUG E_UART_DEBUG
static char log_buf[128];
static const ef_env default_env_set[] = {
    {"SN", "SN0000-W25Q128JV", 32 * sizeof(uint8_t)}
};

/**
 * Flash port for hardware initialize.
 *
 * @param default_env default ENV set for user
 * @param default_env_size default ENV size
 *
 * @return result
 */
EfErrCode ef_port_init(ef_env const **default_env, size_t *default_env_size) {
    EfErrCode result = EF_NO_ERR;

    *default_env = default_env_set;
    *default_env_size = sizeof(default_env_set) / sizeof(default_env_set[0]);

    sfud_err sfud_status[2];
    sfud_status[0] = sfud_init();
    flash_w25q128 = sfud_get_device(SFUD_W25Q128JV_DEVICE_INDEX);
    sfud_status[1] = sfud_device_init(flash_w25q128);
    if (sfud_status[0] != SFUD_SUCCESS || sfud_status[1] != SFUD_SUCCESS)
        result = EF_ENV_INIT_FAILED;

    return result;
}

/**
 * Read data from flash.
 * @note This operation's units is word.
 *
 * @param addr flash address
 * @param buf buffer to store read data
 * @param size read bytes size
 *
 * @return result
 */
EfErrCode ef_port_read(uint32_t addr, uint32_t *buf, size_t size) {
    EfErrCode result = EF_NO_ERR;
    /* You can add your code under here. */
    if (flash_w25q128 == NULL) {
        result = EF_ENV_INIT_FAILED;
        return result;
    }
    result =
        sfud_read(flash_w25q128, addr, size, (uint8_t *)buf) == SFUD_SUCCESS ? EF_NO_ERR : EF_READ_ERR;
    return result;
}

/**
 * Erase data on flash.
 * @note This operation is irreversible.
 * @note This operation's units is different which on many chips.
 *
 * @param addr flash address
 * @param size erase bytes size
 *
 * @return result
 */
EfErrCode ef_port_erase(uint32_t addr, size_t size) {
    EfErrCode result = EF_NO_ERR;

    /* make sure the start address is a multiple of EF_ERASE_MIN_SIZE */
    EF_ASSERT(addr % EF_ERASE_MIN_SIZE == 0);
    /* You can add your code under here. */
    if (flash_w25q128 == NULL) {
        result = EF_ENV_INIT_FAILED;
        return result;
    }
    result =
        sfud_erase(flash_w25q128, addr, size) == SFUD_SUCCESS ? EF_NO_ERR : EF_ERASE_ERR;
    return result;
}

/**
 * Write data to flash.
 * @note This operation's units is word.
 * @note This operation must after erase. @see flash_erase.
 *
 * @param addr flash address
 * @param buf the write data buffer
 * @param size write bytes size
 *
 * @return result
 */
EfErrCode ef_port_write(uint32_t addr, const uint32_t *buf, size_t size) {
    EfErrCode result = EF_NO_ERR;
    /* You can add your code under here. */
    if (flash_w25q128 == NULL) {
        result = EF_ENV_INIT_FAILED;
        return result;
    }
    result =
        sfud_write(flash_w25q128, addr, size, (const uint8_t *)buf) == SFUD_SUCCESS ? EF_NO_ERR : EF_WRITE_ERR;
    return result;
}

/**
 * @brief 芯片擦除函数
 *
 * 该函数用于擦除Flash芯片的全部内容
 * @return EfErrCode 擦除操作的错误码
 */
EfErrCode ef_chip_erase(void) {
    EfErrCode result = EF_NO_ERR;
    if (flash_w25q128 == NULL) {
        result = EF_ENV_INIT_FAILED;
        return result;
    }
    result =
        sfud_chip_erase(flash_w25q128) == SFUD_SUCCESS ? EF_NO_ERR : EF_ERASE_ERR;
    return result;
}

/**
 * lock the ENV ram cache
 */
void ef_port_env_lock(void) {
    /* You can add your code under here. */
    // __disable_irq();
}

/**
 * unlock the ENV ram cache
 */
void ef_port_env_unlock(void) {
    /* You can add your code under here. */
    // __enable_irq();
}


/**
 * This function is print flash debug info.
 *
 * @param file the file which has call this function
 * @param line the line number which has call this function
 * @param format output format
 * @param ... args
 *
 */
void ef_log_debug(const char *file, const long line, const char *format, ...) {
#ifdef PRINT_DEBUG
    va_list args;
    /* args point to the first variable parameter */
    va_start(args, format);
    ef_print("[EasyFlash]");
    /* You can add your code under here. */
    vsprintf(log_buf, format, args);
    ef_print("%s", log_buf);
    va_end(args);
#endif
}

/**
 * This function is print flash routine info.
 *
 * @param format output format
 * @param ... args
 */
void ef_log_info(const char *format, ...) {
#ifdef PRINT_DEBUG
    va_list args;
    /* args point to the first variable parameter */
    va_start(args, format);
    vsprintf(log_buf, format, args);
    ef_print("%s", log_buf);
    /* You can add your code under here. */
    va_end(args);
#endif
}

/**
 * This function is print flash non-package info.
 *
 * @param format output format
 * @param ... args
 */
void ef_print(const char *format, ...) {
    va_list args;
    /* args point to the first variable parameter */
    va_start(args, format);
    /* You can add your code under here. */
    vsprintf(log_buf, format, args);
    bsp_uart_printf(EF_UART_DEBUG,"%s", log_buf);
    va_end(args);
}
