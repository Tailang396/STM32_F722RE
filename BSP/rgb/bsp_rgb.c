//
// Created by asus on 2026/3/31.
//

#include "bsp_rgb.h"
#include "tim.h"

void bsp_rgb_init(void) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
}

void bsp_rgb_set(uint8_t r, uint8_t g, uint8_t b) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)(r * 1000 / 255));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint16_t)(g * 1000 / 255));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)(b * 1000 / 255));
}
