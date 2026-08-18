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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "bsp_tmc2209.h"
#include "bsp_motor.h"
#include "bsp_adc.h"
#include "protocol.h"
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
  MX_GPIO_Init();                    /* 步骤1 */
  MX_DMA_Init();                     /* 步骤2 */
  MX_TIM3_Init();                    /* 步骤3: TIM3 PWM (STEP 脉冲), 已修复 AFIO 重映射顺序 */
  MX_USART1_UART_Init();             /* 步骤4 */
  MX_ADC1_Init();                    /* 步骤5 */
  /* USER CODE BEGIN 2 */
  BSP_GPIO_Init();
  BSP_UART_Init();
  USART1->CR1 |= USART_CR1_RE;
  BSP_ADC_Init();

#ifdef U7_DEBUG
  /* 诊断: 发送 USART1 寄存器值 */
  {
      uint16_t cr1 = USART1->CR1, sr = USART1->SR;
      char hex[] = "0123456789ABCDEF";
      USART1->DR = 'C'; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = '1'; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = '='; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = hex[(cr1>>12)&0xF]; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = hex[(cr1>>8)&0xF]; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = hex[(cr1>>4)&0xF]; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = hex[cr1&0xF]; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = ' '; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = 'S'; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = '='; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = hex[(sr>>8)&0xF]; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = hex[(sr>>4)&0xF]; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = hex[sr&0xF]; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = '\r'; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = '\n'; while(!(USART1->SR&USART_SR_TC));
  }

  /* ADC 验证最先输出 */
  {
      uint8_t ok; uint16_t r = BSP_ADC_ReadRaw(ADC_CH_ADC0, &ok);
      USART1->DR = '\r'; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = '\n'; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = 'A'; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = 'D'; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = 'C'; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = '='; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = ok ? '1' : '0'; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = '\r'; while(!(USART1->SR&USART_SR_TC));
      USART1->DR = '\n'; while(!(USART1->SR&USART_SR_TC));
  }
#endif /* U7_DEBUG */

  /* [2026-08-17] M6 修复: 恢复电机/TMC2209 初始化 (此前被注释, TMC 靠上电默认值运行,
   * 细分不确定; 此处按备份版本恢复: 检测到 TMC 在线则显式配置 16 细分等参数)
   * [2026-08-18] 排障开关: 若怀疑 TMC 初始化导致 U7 异常, 定义 U7_SKIP_TMC_CFG
   * 可跳过 (电机退化为 TMC 上电默认配置) */
#ifndef U7_SKIP_TMC_CFG
  BSP_TMC_Init();
  if (BSP_TMC_IsAlive()) {
      BSP_TMC_SetDefaults();   /* GCONF/IHOLD_IRUN/CHOPCONF(mres=4→16细分)/PWMCONF */
  }
#endif /* U7_SKIP_TMC_CFG */
  BSP_Motor_Init();

  PROTO_Init();
  while (1) {
    PROTO_Run();
  }
  /* USER CODE END 2 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
