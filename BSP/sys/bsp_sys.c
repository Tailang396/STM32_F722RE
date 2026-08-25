#include "bsp_sys.h"
#include "tim.h"
#include <stdint.h>

void delay_us(uint16_t us) {
    uint16_t differ = 0xffff - us - 5;
    __HAL_TIM_SET_COUNTER(&htim7, differ);
    HAL_TIM_Base_Start(&htim7);
    while (differ < 0xffff - 5) {
        differ = __HAL_TIM_GET_COUNTER(&htim7);
    }
    HAL_TIM_Base_Stop(&htim7);
}

void delay_u32_us(uint32_t us) {
    uint32_t remaining = us;
    const uint16_t BOUNDARY = 65000;
    while (remaining > BOUNDARY) {
        delay_us(BOUNDARY);
        remaining -= BOUNDARY;
    }
    if (remaining > 0) {
        delay_us((uint16_t)remaining);
    }
}

void delay_ms(uint16_t ms) {
    while (ms--) {
        delay_us(1000);
    }
}
