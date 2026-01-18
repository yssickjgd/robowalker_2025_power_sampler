/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// ADC采样缓冲区
uint16_t adc_buffer[2];
// 电流, 电压, 功率值
float now_current = 0.0f;
float now_voltage = 0.0f;
float now_power = 0.0f;
// UART通信缓冲区, 0xAB float:电流 float:电压 float:功率 uint8:校验和
uint8_t uart_tx_buffer[14];
// CAN通信缓冲区, float:电流 float:电压
uint8_t can_tx_buffer[8];
// USB通信缓冲区, 0xAB float:电流 float:电压 float:功率 uint8:校验和
uint8_t usb_tx_buffer[14];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_IWDG_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  while (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
  };
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 2);
  HAL_CAN_Start(&hcan);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // TIM2 CH4的呼吸灯, 周期1s
    for (int16_t pwm = 0; pwm <= 10000; pwm += 20)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm);
      HAL_Delay(0);
    }
    for (int16_t pwm = 10000; pwm >= 20; pwm -= 20)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm);
      HAL_Delay(0);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
    now_voltage = (float)(adc_buffer[0]) / 4096.0f * 3.3f / 50.0f * 10010.0f / 10.0f;
    now_current = ((float)(adc_buffer[1] - 2048.0f) / 4096.0f * 3.3f) / 50.0f / 0.005f;
    now_power = now_voltage * now_current;

    uart_tx_buffer[0] = 0xAB;
    memcpy(&uart_tx_buffer[1], &now_current, 4);
    memcpy(&uart_tx_buffer[5], &now_voltage, 4);
    memcpy(&uart_tx_buffer[9], &now_power, 4);
    uart_tx_buffer[13] = 0;
    for (int i = 1; i < 13; i++)
    {
      uart_tx_buffer[13] += uart_tx_buffer[i];
    }
    HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, 14);

    memcpy(&can_tx_buffer[0], &now_current, 4);
    memcpy(&can_tx_buffer[4], &now_voltage, 4);
    CAN_TxHeaderTypeDef can_tx_header;
    can_tx_header.StdId = 0x114;
    can_tx_header.ExtId = 0;
    can_tx_header.IDE = 0;
    can_tx_header.RTR = 0;
    can_tx_header.DLC = 8;
    uint32_t can_tx_mailbox;
    HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_buffer, &can_tx_mailbox);

    usb_tx_buffer[0] = 0xAB;
    memcpy(&usb_tx_buffer[1], &now_current, 4);
    memcpy(&usb_tx_buffer[5], &now_voltage, 4);
    memcpy(&usb_tx_buffer[9], &now_power, 4);
    usb_tx_buffer[13] = 0;
    for (int i = 1; i < 13; i++)
    {
      usb_tx_buffer[13] += usb_tx_buffer[i];
    }
    CDC_Transmit_FS(usb_tx_buffer, 14);
    HAL_IWDG_Refresh(&hiwdg);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
