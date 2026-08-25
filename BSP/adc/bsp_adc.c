//
// Created by asus on 2026/4/4.
//

#include "bsp_adc.h"
#include "adc.h"
#include "tim.h"
#include "math.h"

static uint16_t adc2_value[20];
static uint16_t adc1_value[30];

static const float voltage_soc[12][2] = {
    {4.15f, 100.0f},
    {4.06f, 90.0f},
    {3.98f, 80.0f},
    {3.90f, 70.0f},
    {3.80f, 60.0f},
    {3.75f, 50.0f},
    {3.70f, 40.0f},
    {3.65f, 30.0f},
    {3.60f, 20.0f},
    {3.50f, 10.0f},
    {3.40f, 5.0f},
    {3.20f, 0.0f}
};

void bsp_adc_init(void) {
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_value, 20); // 单通道
    HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_value, 30);
    HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
}

float bsp_get_bat_voltage(void) {
    uint32_t adc_sum = 0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < 20; i++) {
        if (adc2_value[i] > 2000) {
            adc_sum += adc2_value[i];
            count++;
        }
    }
    return ((float)adc_sum * 3.3f * 197) / (4095 * (float)count * 150) + 0.01f;
}

float bsp_get_bat_soc(void) {
    float voltage = bsp_get_bat_voltage();
    if (voltage >= voltage_soc[0][0]) {
        return 100.0f;
    }
    if (voltage <= voltage_soc[11][0]) {
        return 0.0f;
    }
    for (uint8_t i = 0; i < 11; i++) {
        float v1 = voltage_soc[i][0];
        float v2 = voltage_soc[i+1][0];
        if (voltage >= v2 && voltage <= v1) {
            float soc1 = voltage_soc[i][1];
            float soc2 = voltage_soc[i+1][1];
            float soc = soc1 + (soc2 - soc1) * (voltage - v1) / (v2 - v1);
            return soc;
        }
    }
    return 0.0f;
}


float bsp_get_mq7_voltage(void) {
    uint32_t adc_sum = 0;
    for (uint8_t i = 1; i < 30; i = i + 3) {
        adc_sum += adc1_value[i];
    }
    return ((float)adc_sum * 3.3f * 138) / (4095 * 10 * 91);
}

float bsp_get_mq4_voltage(void) {
    uint32_t adc_sum = 0;
    for (uint8_t i = 0; i < 30; i = i + 3) {
        adc_sum += adc1_value[i];
    }
    return ((float)adc_sum * 3.3f * 138) / (4095 * 10 * 91);
}

float bsp_get_mq5_voltage(void) {
    uint32_t adc_sum = 0;
    for (uint8_t i = 2; i < 30; i = i + 3) {
        adc_sum += adc1_value[i];
    }
    return ((float)adc_sum * 3.3f * 138) / (4095 * 10 * 91);
}