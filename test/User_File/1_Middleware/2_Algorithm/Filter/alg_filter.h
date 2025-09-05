/**
 * @file alg_filter.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 滤波器
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-09-25 1.1 可自定义滤波器阶数
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef ALG_FILTER_H
#define ALG_FILTER_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/1_Driver/Math/drv_math.h"

/* Exported macros -----------------------------------------------------------*/

// 滤波器阶数
#define FILTER_FOURIER_ORDER (500)
// 采样频率
#define SAMPLING_FREQUENCY (1000.0f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 滤波器类型
 *
 */
enum Enum_Filter_Fourier_Type
{
    Filter_Fourier_Type_LOWPASS = 0,
    Filter_Fourier_Type_HIGHPASS,
    Filter_Fourier_Type_BANDPASS,
    Filter_Fourier_Type_BANDSTOP,
};

/**
 * @brief Reusable, Fourier滤波器算法
 *
 */
class Class_Filter_Fourier
{
public:
    void Init(float __Value_Constrain_Low = 0.0f, float __Value_Constrain_High = 1.0f, Enum_Filter_Fourier_Type __Filter_Fourier_Type = Filter_Fourier_Type_LOWPASS, float __Frequency_Low = 0.0f, float __Frequency_High = SAMPLING_FREQUENCY / 2.0f, float __Sampling_Frequency = SAMPLING_FREQUENCY, int Filter_Fourier_Order = 5);

    inline float Get_Out();

    inline void Set_Now(float __Now);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 输入限幅
    float Value_Constrain_Low;
    float Value_Constrain_High;
    int Filter_Fourier_Order;

    // 滤波器类型
    Enum_Filter_Fourier_Type Filter_Fourier_Type;
    // 滤波器特征低频
    float Frequency_Low;
    // 滤波器特征高频
    float Frequency_High;
    // 滤波器采样频率
    float Sampling_Frequency;

    // 常量

    // 内部变量

    // 卷积系统函数向量
    float System_Function[FILTER_FOURIER_ORDER + 1];

    // 输入信号向量
    float Input_Signal[FILTER_FOURIER_ORDER + 1];

    // 新数据指示向量
    uint8_t Signal_Flag = 0;

    // 读变量

    // 输出值
    float Out = 0;

    // 写变量

    // 内部函数
};

/**
 * @brief Reusable, Kalman滤波器算法
 *
 */
class Class_Filter_Kalman
{
public:
    void Init(float __Error_Measure = 1.0f, float __Now = 0.0f, float __Error_Estimate = 1.0f);

    inline float Get_Out();

    inline void Set_Now(float __Now);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 测量误差
    float Error_Measure;

    // 常量

    // 内部变量

    // 估计误差
    float Error_Estimate = 1.0f;
    // 增益
    float Kalman_Gain = 0.0f;

    // 读变量

    // 输出值
    float Out = 0.0f;

    // 写变量

    // 当前值
    float Now = 0.0f;

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
inline float Class_Filter_Fourier::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定当前值
 *
 * @param __Now 当前值
 */
inline void Class_Filter_Fourier::Set_Now(float __Now)
{
    // 输入限幅, 全0为不限制
    if (Value_Constrain_Low != 0.0f || Value_Constrain_High != 0.0f)
    {
        Math_Constrain(&__Now, Value_Constrain_Low, Value_Constrain_High);
    }

    // 将当前值放入被卷积的信号中
    Input_Signal[Signal_Flag] = __Now;
    Signal_Flag++;

    // 若越界则轮回
    if (Signal_Flag == Filter_Fourier_Order + 1)
    {
        Signal_Flag = 0;
    }
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
inline float Class_Filter_Kalman::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定当前值
 *
 * @param __Now 当前值
 */
inline void Class_Filter_Kalman::Set_Now(float __Now)
{
    Now = __Now;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
