#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef enum
{
    PID_UNIPOLAR = 0,
    PID_BIPOLAR = 1
} PID_Polarity;

typedef struct
{
    float kp, ki, kd;
    float Aim, Real;
    float err;
    float last1_err;
    float last2_err;
    float sum_err;
    float max_sum;
    float Pout, Iout, Dout;
    float PIDout;
    float max_out;
    PID_Polarity polarity;
    uint8_t active;
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float maxsum, float maxout);
void PID_SetMode(PID_TypeDef *pid, PID_Polarity polarity);
float PID_Calculate(PID_TypeDef *pid, float aim, float real);
void PID_over_zero(float *real, float aim, float max, float min);

#endif
