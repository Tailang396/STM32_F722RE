//
// Created by asus on 2025/7/14.
//

#include "app_sys.h"

#include <string.h>

#include "adc.h"
#include "app_imu.h"
#include "app_sensor.h"
#include "bsp_def.h"
#include "bsp_flash.h"
#include "can.h"
#include "bsp_can.h"
#include "bsp_imu.h"
#include "bsp_rgb.h"
#include "lcd.h"
#include "tim.h"
#include "tmp117.h"

#define abs(x) ((x) > 0 ? (x) : -(x))

void debug_callback(bsp_uart_e e, uint8_t *s, uint16_t l);
static void LCD_Begin(void);
uint8_t data[32] = {0};

void sys_main_init(void)
{
    bsp_rgb_init();
    bsp_rgb_set(255, 0, 0);
    HAL_Delay(1000);
    LCD_Init();
    LCD_Begin();
    bsp_uart_init(E_UART_DEBUG, &huart1);
    bsp_uart_set_callback(E_UART_DEBUG,debug_callback);
    bsp_flash_init();
    bsp_flash_read("SN", data, 32);
    bsp_uart_printf(E_UART_DEBUG,"%s\r\n", data);
    bsp_imu_init(&htim14);
    bsp_flash_print_status();
}
void app_sys(void *argument)
{
    int8_t r = 0, g = 0, b = 0;
    static uint16_t count = 0;
    for(;;)
    {
        if (++count == 20) {
            count = 0;
            HAL_GPIO_TogglePin(led_GPIO_Port,led_Pin);
            // bsp_uart_printf(E_UART_LORA, "LORA\r\n");
        }
        // bsp_uart_printf(E_UART_DEBUG,"angle: %.2f, %.2f, %.2f, %.1f\r\n", imu_data->angle.roll, imu_data->angle.pitch, imu_data->angle.yaw,temp);
        // bsp_uart_printf(E_UART_DEBUG, "mag: %.2f, %.2f, %.2f\r\n", imu_data->mag.mag_x, imu_data->mag.mag_y, imu_data->mag.mag_z);
        // bsp_uart_printf(E_UART_DEBUG, "bmp: %.2f, %.2f\r\n", imu_data->bmp.altitude, imu_data->bmp.pressure);
        // bsp_uart_printf(E_UART_DEBUG, "adc: %d\r\n", adc2_value);
        // bsp_uart_printf(E_UART_DEBUG, "adc: %d, %d, %d, %d\r\n", adc1_value[0], adc1_value[1], adc1_value[2], adc1_value[3]);

        bsp_rgb_set(abs(r), abs(g), abs(b));
        if(++r > 100) r = -100;
        if(++g > 100) g = -100;
        if(++b > 100) b = -100;
        osDelay(10);
    }
}

void debug_callback(bsp_uart_e e, uint8_t *s, uint16_t l)
{
    bsp_uart_send(e,s,l);
}

static void LCD_Begin(void) {
    LCD_Fill(0,0,LCD_W, LCD_H,WHITE);
    LCD_ShowPicture(0, 10, 280, 140, gImage_pic);
    LCD_ShowChinese(76, 155, "00", BROWN, WHITE, 32, 0);
    LCD_ShowChinese(108, 155, "01", BRRED, WHITE, 32, 0);
    LCD_ShowChinese(140, 155, "02", 0x586D, WHITE, 32, 0);
    LCD_ShowChinese(172, 155, "03", GRAY, WHITE, 32, 0);
    LCD_ShowString(204, 171, "...", LBBLUE, WHITE, 16, 0);

    uint8_t mode = 0x00;
    mode = (HAL_GPIO_ReadPin(MOD1_GPIO_Port, MOD1_Pin) << 1) | HAL_GPIO_ReadPin(MOD2_GPIO_Port, MOD2_Pin);
    if(mode == 0x00) {
        LCD_ShowChinese(60, 197, "07", BLUE, WHITE, 32, 0);
        LCD_ShowChinese(100, 197, "08", GRAYBLUE, WHITE, 32, 0);
        LCD_ShowChinese(140, 197, "09", MAGENTA, WHITE, 32, 0);
        LCD_ShowChinese(180, 197, "10", BRRED, WHITE, 32, 0);
    } else if(mode == 0x02) {
        LCD_ShowChinese(100, 197, "07", LIGHTBLUE, WHITE, 32, 0);
        LCD_ShowChinese(140, 197, "08", MAGENTA, WHITE, 32, 0);
    } else if(mode == 0x01) {
        LCD_ShowChinese(60, 197, "06", BLUE, WHITE, 32, 0);
        LCD_ShowChinese(100, 197, "08", GRAYBLUE, WHITE, 32, 0);
        LCD_ShowChinese(140, 197, "09", MAGENTA, WHITE, 32, 0);
        LCD_ShowChinese(180, 197, "10", BRRED, WHITE, 32, 0);
    } else if(mode == 0x03) {
        LCD_ShowChinese(100, 197, "06", LIGHTBLUE, WHITE, 32, 0);
        LCD_ShowChinese(140, 197, "08", MAGENTA, WHITE, 32, 0);
    }
}