/**
 * @file dvc_sampler.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief ADC采样器
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef DVC_SAMPLER_H
#define DVC_SAMPLER_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/1_Driver/ADC/drv_adc.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, 采样器, 后续可以增加多对一采样并匹配滤波等算法
 *
 */
class Class_Sampler
{
public:

    void Init(ADC_HandleTypeDef *hadc, uint16_t __Sampler_Serial);

    inline float Get_Value();

    void TIM_Sampler_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //绑定的UART
    Struct_ADC_Manage_Object *ADC_Manage_Object;
    //绑定的采样通道
    uint16_t Sampler_Serial;
    //对应采样数据指针
    uint16_t *ADC_Value;

    //常量

    //内部变量

    //读变量

    float Value = 0.0f;

    //写变量

    //读写变量

    //内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取采样值, 归化到0~1
 *
 * @return float 采样值, 归化到0~1
 */
inline float Class_Sampler::Get_Value()
{
    return (Value);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
