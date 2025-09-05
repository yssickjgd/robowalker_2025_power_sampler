/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/**
 * @brief 注意, 每个类的对象分为专属对象Specialized, 同类可复用对象Reusable以及通用对象Generic
 *
 * 专属对象:
 * 单对单来独打独
 * 比如交互类的底盘对象, 只需要交互对象调用且全局只有一个, 这样看来, 底盘就是交互类的专属对象
 * 这种对象直接封装在上层类里面, 初始化在上层类里面, 调用在上层类里面
 *
 * 同类可复用对象:
 * 各调各的
 * 比如电机的对象, 底盘可以调用, 云台可以调用, 而两者调用的是不同的对象, 这种就是同类可复用对象
 * 电机的pid对象也算同类可复用对象, 它们都在底盘类里初始化
 * 这种对象直接封装在上层类里面, 初始化在最近的一个上层专属对象的类里面, 调用在上层类里面
 *
 * 通用对象:
 * 多个调用同一个
 * 比如底盘陀螺仪对象, 底盘类要调用它做小陀螺, 云台要调用它做方位感知, 因此底盘陀螺仪是通用对象.
 * 这种对象以指针形式进行指定, 初始化在包含所有调用它的上层的类里面, 调用在上层类里面
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"

#include "2_Device/Serialplot/dvc_serialplot.h"
#include "2_Device/Sampler/dvc_sampler.h"
#include "1_Middleware/1_Driver/TIM/drv_tim.h"
#include "1_Middleware/2_Algorithm/Filter/alg_filter.h"

/* Private macros ------------------------------------------------------------*/

#define DEBUG_FLAG 0

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 全局初始化完成标志位
bool init_finished = false;

// 串口绘图
Class_Serialplot serialplot;
// 电流采样器
Class_Sampler sampler_current;
// 电压采样器
Class_Sampler sampler_voltage;
// 电流滤波器
Class_Filter_Fourier filter_current;
// 电压滤波器
Class_Filter_Fourier filter_voltage;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief UART1串口绘图回调函数
 *
 * @param Buffer UART2收到的消息
 * @param Length 长度
 */
void Serialplot_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    serialplot.UART_RxCpltCallback(Buffer, Length);
}

/**
 * @brief TIM4任务回调函数
 *
 */
void Task100us_TIM2_Callback()
{
    sampler_current.TIM_Sampler_PeriodElapsedCallback();
    sampler_voltage.TIM_Sampler_PeriodElapsedCallback();

    filter_current.Set_Now(sampler_current.Get_Value());
    filter_voltage.Set_Now(sampler_voltage.Get_Value());

    filter_current.TIM_Calculate_PeriodElapsedCallback();
    filter_voltage.TIM_Calculate_PeriodElapsedCallback();
}

/**
 * @brief TIM5任务回调函数
 *
 */
void Task1ms_TIM5_Callback()
{
    float final_current_value, final_voltage_value, final_power;
    final_current_value = 0.001600687f * filter_current.Get_Out() - 3.27319976f;
    final_voltage_value = 0.008482093f * filter_voltage.Get_Out() - 0.057714471f;
    // final_current_value = 0.00161133f * filter_current.Get_Out() - 3.3f;
    // final_voltage_value = 0.00809692f * filter_voltage.Get_Out();
    final_power = final_current_value * final_voltage_value;
    serialplot.Set_Data(3, &final_current_value, &final_voltage_value, &final_power);
    serialplot.TIM_Write_PeriodElapsedCallback();
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    // 驱动层初始化

    UART_Init(&huart1, Serialplot_UART1_Callback, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);
    // 定时器初始化
    TIM_Init(&htim2, Task100us_TIM2_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);
    // ADC初始化
    ADC_Init(&hadc1, 2);

    // 算法层初始化
    filter_current.Init(0.0f, 0.0f, Filter_Fourier_Type_LOWPASS, 0.5f, 0.0f, 10000.0f, 20);
    filter_voltage.Init(0.0f, 0.0f, Filter_Fourier_Type_LOWPASS, 0.5f, 0.0f, 10000.0f, 20);

    // 设备层初始化

    // 串口绘图初始化
    serialplot.Init(&huart1, Serialplot_Checksum_8_ENABLE);
    // 电流采样器初始化
    sampler_current.Init(&hadc1, 0);
    // 电压采样器初始化
    sampler_voltage.Init(&hadc1, 1);

    // 使能调度时钟
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim5);
    // 标记初始化完成
    init_finished = true;

    // 等待系统
    HAL_Delay(1000);
}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{

}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
