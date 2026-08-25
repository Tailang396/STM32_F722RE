#include "main.h"
#include "PID.h"

/**
 * PID控制器初始化函数
 *
 * @param pid PID控制器的结构体指针，用于存储PID控制器的相关参数和状态
 * @param Kp 比例增益，决定比例项的反应速度
 * @param Ki 积分增益，决定积分项累加速度和消除稳态误差的能力
 * @param Kd 微分增益，决定微分项对误差变化率的反应速度，用以抑制超调和提高系统的稳定性
 * @param maxsum 积分饱和上限，防止积分项过大导致系统不稳定
 * @param maxout PID输出饱和上限，确保输出在合理范围内
 *
 * 本函数用于初始化PID控制器的各项参数和状态，包括比例、积分、微分增益，积分和输出的饱和上限，
 * 以及用于内部计算的变量，如输出、误差和历史误差等。初始化完成后，PID控制器处于激活状态，准备进行控制运算。
 */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float maxsum, float maxout)
{
    pid->kp = Kp;
    pid->ki = Ki;
    pid->kd = Kd;
    pid->max_sum = maxsum;
    pid->max_out = maxout;
    pid->PIDout = 0;
    pid->sum_err = 0;
    pid->err = 0;
    pid->last1_err = 0;
    pid->last2_err = 0;
    pid->active = 1;
    pid->polarity = PID_BIPOLAR;
}

/**
 * PID_SetMode 函数用于设置 PID 控制器的工作极性模式
 * @param pid PID 控制器结构体指针，指向需要设置模式的 PID 控制器实例
 * @param polarity 极性模式参数，可选值为 PID_UNIPOLAR（单极性）或 PID_BIPOLAR（双极性）
 */
void PID_SetMode(PID_TypeDef *pid, PID_Polarity polarity) {
    pid->polarity = polarity;
}

/**
 * PID控制算法计算函数
 *
 * 该函数实现了一个标准的PID控制算法，用于根据目标值和实际值计算控制输出
 * PID控制是反馈控制的一种，通过比例、积分、微分三个环节的计算，来调整控制量
 *
 * @param pid PID控制结构体指针，包含PID控制所需的参数和状态变量
 * @param aim 目标值，即期望的设定值
 * @param real 实际值，即系统当前的测量值
 * @return 返回计算得到的PID控制输出值
 */
float PID_Calculate(PID_TypeDef *pid, float aim, float real)
{
    if (pid->active)
    {
        pid->Aim = aim;
        pid->Real = real;
        pid->err = pid->Aim - pid->Real;
        pid->sum_err = pid->sum_err + pid->err;

        if (pid->polarity == PID_UNIPOLAR) {
            if (pid->sum_err >= pid->max_sum) pid->sum_err = pid->max_sum;
            else if (pid->sum_err <= 0) pid->sum_err = 0;
        } else {
            if (pid->sum_err >= pid->max_sum) pid->sum_err = pid->max_sum;
            else if (pid->sum_err <= -pid->max_sum) pid->sum_err = -pid->max_sum;
        }
        pid->last2_err = pid->last1_err;
        pid->last1_err = pid->err;
        pid->Pout = pid->kp * pid->err;
        pid->Iout = pid->ki * pid->sum_err;
        pid->Dout = pid->kd * (pid->err - 2 * pid->last1_err + pid->last2_err);
        pid->PIDout = pid->Pout + pid->Iout + pid->Dout;

        if (pid->polarity == PID_UNIPOLAR) {
            if (pid->PIDout >= pid->max_out) pid->PIDout = pid->max_out;
            else if (pid->PIDout <= 0) pid->PIDout = 0;
        } else {
            if (pid->PIDout >= pid->max_out) pid->PIDout = pid->max_out;
            else if (pid->PIDout <= -pid->max_out) pid->PIDout = -pid->max_out;
        }
        return pid->PIDout;
    } else {
        pid->sum_err = 0;
        pid->last1_err = 0;
        pid->last2_err = 0;
        pid->err = 0;
        pid->PIDout = 0;
        return 0;
    }
}

/**
 * PID_over_zero函数用于根据目标值和实际值的差异，调整实际值以接近目标值。
 * 该函数主要解决的是在PID控制中，当目标值跨越零点时，如何有效调整实际值的问题。
 * 过零处理，一般处理imu，绝对值角度传感器的零点突变问题，进行劣弧优化。
 *
 * @param real 指向实际值的指针，实际值将根据目标值进行调整。
 * @param aim 目标值，函数旨在将实际值调整到接近此值。
 * @param max 实际值和目标值之间调整的最大范围。
 * @param min 实际值和目标值之间调整的最小范围。
 */
void PID_over_zero(float *real, float aim, float max, float min)
{
    if (aim - *real > (max - min) / 2)
        *real = *real + (max - min);
    else if (aim - *real < -(max - min) / 2)
        *real = *real - (max - min);
    else
        __NOP();
}