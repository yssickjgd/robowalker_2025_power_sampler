/**
 * @file dvc_sampler.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief ADC采样器
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_sampler.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 采样器初始化
 * 
 * @param hadc 指定的ADC
 * @param __Sampler_Serial 采样点序列号, 从0开始
 */
void Class_Sampler::Init(ADC_HandleTypeDef *hadc, uint16_t __Sampler_Serial)
{
    if (hadc->Instance == ADC1)
    {
        ADC_Manage_Object = &ADC1_Manage_Object;
    }
    Sampler_Serial = __Sampler_Serial;

    ADC_Value = &ADC_Manage_Object->ADC_Data[Sampler_Serial];
}

/**
 * @brief TIM定时器中断计算采样值函数
 *
 */
void Class_Sampler::TIM_Sampler_PeriodElapsedCallback()
{
    Value = *ADC_Value;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
