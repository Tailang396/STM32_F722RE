//
// Created by asus on 2026/1/25.
//

#include "app_lcdshow.h"

#include "app_sensor.h"
#include "bsp_adc.h"
#include "bsp_def.h"
#include "bsp_imu.h"
#include "lcd.h"
#include "sht40.h"

static void LCD_ShowInit(uint8_t BatLevel);

void app_lcdshow(void* argument) {
    osDelay(10);
    static uint8_t BatLevel = 0;
    static uint8_t last_bat_level = 0;
    static float co_v;
    static float ng_v;
    static float lpg_v;
    static float alarm_v;
    BatLevel = (uint8_t)(bsp_get_bat_soc() + 0.5f);
    last_bat_level = BatLevel;
    LCD_ShowInit(BatLevel);
    static uint16_t count = 10;
    imu_data_t* imu_data = bsp_imu_GetData();
    SHT40_Data_t* sht40_data = app_sensor_GetSHT40Data();
    for (;;) {
        if (++count >= 20) {
            count = 0;
            BatLevel = (uint8_t)(bsp_get_bat_soc() + 0.5f);
            if (BatLevel != last_bat_level) {
                last_bat_level = BatLevel;
                LCD_ShowIntNum(200, 0, BatLevel, 3, BLUE, WHITE, 24);

                if (BatLevel < 20)
                    LCD_ShowChinese(164, 0, "94", RED, WHITE, 24, 0);
                else if (BatLevel < 40)
                    LCD_ShowChinese(164, 0, "93", GRED, WHITE, 24, 0);
                else if (BatLevel < 70)
                    LCD_ShowChinese(164, 0, "92", BLUE, WHITE, 24, 0);
                else if (BatLevel < 90)
                    LCD_ShowChinese(164, 0, "91", BLUE, WHITE, 24, 0);
            }
        }
        LCD_ShowFloatNum(36, 29, imu_data->angle.roll, 3, 1, YELLOW, BLACK, 32);
        LCD_ShowFloatNum(36, 61, imu_data->angle.pitch, 3, 1, BLUE, BLACK, 32);
        LCD_ShowFloatNum(36, 93, imu_data->angle.yaw, 3, 1, GREEN, BLACK, 32);
        LCD_ShowFloatNum(36, 125, imu_data->bmp.altitude, 5, 0, LIGHTGREEN, BLACK, 32);
        LCD_ShowFloatNum(174, 64, imu_data->temp.bsp_temp, 3, 1, GRAYBLUE, BLACK, 24);
        LCD_ShowFloatNum(180, 128, bsp_get_bat_voltage(), 1, 2, GRAYBLUE, BLACK, 24);
        LCD_ShowFloatPositiveNum(75, 163, sht40_data->temperature, 2, 1, YELLOW, BLACK, 24);
        LCD_ShowFloatPositiveNum(210, 163, sht40_data->humidity, 2, 2, YELLOW, BLACK, 24);

        co_v = app_sensor_GetCO_V();
        ng_v = app_sensor_GetNG_V();
        lpg_v = app_sensor_GetLPG_V();
        alarm_v = app_sensor_GetAlarmVoltage();

        if (co_v < alarm_v)
            LCD_ShowFloatPositiveNum(44, 189, co_v, 1, 2, BLUE, GREEN, 24);
        else
            LCD_ShowFloatPositiveNum(44, 189, co_v, 1, 2, BLUE, RED, 24);
        if (ng_v < alarm_v)
            LCD_ShowFloatPositiveNum(137, 189, ng_v, 1, 2, BLUE, GREEN, 24);
        else
            LCD_ShowFloatPositiveNum(137, 189, ng_v, 1, 2, BLUE, RED, 24);
        if (lpg_v < alarm_v)
            LCD_ShowFloatPositiveNum(231, 189, lpg_v, 1, 2, BLUE, GREEN, 24);
        else
            LCD_ShowFloatPositiveNum(231, 189, lpg_v, 1, 2, BLUE, RED, 24);
        osDelay(50);
    }
}

static void LCD_ShowInit(uint8_t BatLevel) {
    LCD_Fill(0, 0,LCD_W, 25,WHITE);
    LCD_Fill(0, 25,LCD_W, LCD_H,BLACK);
    LCD_ShowString(32, 0, "TGU_GYJ", BLUE, WHITE, 24, 0);

    if (BatLevel > 90)
        LCD_ShowChinese(164, 0, "90", BLUE, WHITE, 24, 0);
    else if (BatLevel > 70)
        LCD_ShowChinese(164, 0, "91", BLUE, WHITE, 24, 0);
    else if (BatLevel > 40)
        LCD_ShowChinese(164, 0, "92", BLUE, WHITE, 24, 0);
    else if (BatLevel > 20)
        LCD_ShowChinese(164, 0, "93", GRED, WHITE, 24, 0);
    else
        LCD_ShowChinese(164, 0, "94", RED, WHITE, 24, 0);
    LCD_ShowIntNum(200, 0, BatLevel, 3, BLUE, WHITE, 24);
    LCD_ShowChar(236, 0, '%', BLUE, WHITE, 24, 0);

    LCD_DrawRectangle(2, 27, 134, 159, GBLUE);
    LCD_DrawRectangle(1, 28, 135, 160, GBLUE);
    LCD_ShowString(4, 29, "R:", YELLOW, BLACK, 32, 0);
    LCD_ShowString(4, 61, "P:", BLUE, BLACK, 32, 0);
    LCD_ShowString(4, 93, "Y:", GREEN, BLACK, 32, 0);
    LCD_ShowString(4, 125, "H:", LIGHTGREEN, BLACK, 32, 0);

    LCD_Fill(141, 27, 278, 59,GRAYBLUE);
    LCD_DrawRectangle(141, 27, 278, 91, GRAYBLUE);
    LCD_ShowString(162, 31, "IMU-TEMP", BLACK, GRAYBLUE, 24, 0);

    LCD_Fill(141, 92, 278, 124,GRAYBLUE);
    LCD_DrawRectangle(141, 92, 278, 160, GRAYBLUE);
    LCD_ShowString(162, 96, "BAT-VOLT", BLACK, GRAYBLUE, 24, 0);

    LCD_Fill(1, 162, 61, 187,YELLOW);
    LCD_DrawRectangle(1, 162, 135, 187, YELLOW);
    LCD_ShowString(7, 163, "TEMP", BLACK, YELLOW, 24, 0);
    LCD_ShowFloatPositiveNum(75, 163, 99.9, 2, 1, YELLOW, BLACK, 24);

    LCD_Fill(141, 162, 201, 187,YELLOW);
    LCD_DrawRectangle(141, 162, 278, 187, YELLOW);
    LCD_ShowString(153, 163, "HUM", BLACK, YELLOW, 24, 0);
    LCD_ShowFloatPositiveNum(210, 163, 99.99, 2, 2, YELLOW, BLACK, 24);

    LCD_Fill(1, 189, 44, 213,BLUE);
    LCD_ShowString(9, 189, "CO", WHITE, BLUE, 24, 0);
    LCD_ShowFloatPositiveNum(44, 189, 9.99, 1, 2, BLUE, GREEN, 24);

    LCD_Fill(94, 189, 137, 213,BLUE);
    LCD_ShowString(102, 189, "NG", WHITE, BLUE, 24, 0);
    LCD_ShowFloatPositiveNum(137, 189, 9.99, 1, 2, BLUE, GREEN, 24);

    LCD_Fill(187, 189, 231, 213,BLUE);
    LCD_ShowString(189, 189, "LPG", WHITE, BLUE, 24, 0);
    LCD_ShowFloatPositiveNum(231, 189, 9.99, 1, 2, BLUE, GREEN, 24);


    uint8_t mode = 0x00;
    mode = (HAL_GPIO_ReadPin(MOD1_GPIO_Port, MOD1_Pin) << 1) | HAL_GPIO_ReadPin(MOD2_GPIO_Port, MOD2_Pin);
    if (mode == 0x00) {
        LCD_ShowChinese(76, 215, "07", BLUE, BLACK, 24, 0);
        LCD_ShowChinese(108, 215, "08", GRAYBLUE, BLACK, 24, 0);
        LCD_ShowChinese(140, 215, "09", MAGENTA, BLACK, 24, 0);
        LCD_ShowChinese(172, 215, "10", BRRED, BLACK, 24, 0);
    }
    else if (mode == 0x02) {
        LCD_ShowChinese(108, 215, "07", LIGHTBLUE, BLACK, 24, 0);
        LCD_ShowChinese(140, 215, "08", MAGENTA, BLACK, 24, 0);
    }
    else if (mode == 0x01) {
        LCD_ShowChinese(76, 215, "06", BLUE, BLACK, 24, 0);
        LCD_ShowChinese(108, 215, "08", GRAYBLUE, BLACK, 24, 0);
        LCD_ShowChinese(140, 215, "09", MAGENTA, BLACK, 24, 0);
        LCD_ShowChinese(172, 215, "10", BRRED, BLACK, 24, 0);
    }
    else if (mode == 0x03) {
        LCD_ShowChinese(108, 215, "06", LIGHTBLUE, BLACK, 24, 0);
        LCD_ShowChinese(140, 215, "08", MAGENTA, BLACK, 24, 0);
    }
}
