/**
 * @file alg_filter.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 滤波器
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-09-25 1.1 可自定义滤波器阶数
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_filter.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化滤波器
 *
 * @param __Value_Constrain_Low 滤波器最小值, 全0不限制
 * @param __Value_Constrain_High 滤波器最大值, 全0不限制
 * @param __Filter_Fourier_Type 滤波器类型
 * @param __Frequency_Low 滤波器特征低频, 非高通有效
 * @param __Frequency_High 滤波器特征高频, 非低通有效
 * @param __Sampling_Frequency 滤波器采样频率
 */
void Class_Filter_Fourier::Init(float __Value_Constrain_Low, float __Value_Constrain_High, Enum_Filter_Fourier_Type __Filter_Fourier_Type, float __Frequency_Low, float __Frequency_High, float __Sampling_Frequency, int __Filter_Fourier_Order)
{
    Value_Constrain_Low = __Value_Constrain_Low;
    Value_Constrain_High = __Value_Constrain_High;
    Filter_Fourier_Type = __Filter_Fourier_Type;
    Frequency_Low = __Frequency_Low;
    Frequency_High = __Frequency_High;
    Sampling_Frequency = __Sampling_Frequency;
    Filter_Fourier_Order = __Filter_Fourier_Order;

    // 将所有计算所得值进行softmax操作成和为1的值
    float system_function_sum = 0.0f;
    // 特征低角速度
    float omega_low;
    // 特征高角速度
    float omega_high;

    omega_low = 2.0f * PI * Frequency_Low / Sampling_Frequency;
    omega_high = 2.0f * PI * Frequency_High / Sampling_Frequency;

    // 计算滤波器系统

    switch (Filter_Fourier_Type)
    {
    case (Filter_Fourier_Type_LOWPASS):
    {
        for (int i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = omega_low / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_low);
        }

        break;
    }
    case (Filter_Fourier_Type_HIGHPASS):
    {
        for (int i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = Math_Sinc((i - Filter_Fourier_Order / 2.0f) * PI) - omega_high / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_high);
        }

        break;
    }
    case (Filter_Fourier_Type_BANDPASS):
    {
        for (int i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = omega_high / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_high) - omega_low / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_low);
        }

        break;
    }
    case (Filter_Fourier_Type_BANDSTOP):
    {
        for (int i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = Math_Sinc((i - Filter_Fourier_Order / 2.0f) * PI) + omega_low / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_low) - omega_high / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_high);
        }

        break;
    }
    }

    for (int i = 0; i < Filter_Fourier_Order + 1; i++)
    {
        system_function_sum += System_Function[i];
    }

    for (int i = 0; i < Filter_Fourier_Order + 1; i++)
    {
        System_Function[i] /= system_function_sum;
    }
}

/**
 * @brief 滤波器调整值, 周期与采样周期相同
 *
 */
void Class_Filter_Fourier::TIM_Calculate_PeriodElapsedCallback()
{
    Out = 0.0f;

    // 执行卷积操作
    for (int i = 0; i < Filter_Fourier_Order + 1; i++)
    {
        Out += System_Function[i] * Input_Signal[(Signal_Flag + i) % (Filter_Fourier_Order + 1)];
    }
}

/**
 * @brief 初始化Kalman滤波器
 * 
 * @param __Error_Measure 测量误差
 * @param __Value 当前值
 * @param __Error_Estimate 估计误差
 */
void Class_Filter_Kalman::Init(float __Error_Measure, float __Now, float __Error_Estimate)
{
    Error_Measure = __Error_Measure;

    Now = __Now;
    Error_Estimate = __Error_Estimate;
}

/**
 * @brief 滤波器调整值, 周期与采样周期相同
 *
 */
void Class_Filter_Kalman::TIM_Calculate_PeriodElapsedCallback()
{
    Kalman_Gain = Error_Estimate / (Error_Estimate + Error_Measure);

    Out = Out + Kalman_Gain * (Now - Out);

    Error_Estimate = (1.0f - Kalman_Gain) * Error_Estimate;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
