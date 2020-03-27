/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_usart.h"
#include "bsp_beep.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <string.h>

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
void start_task(void *pvParameters);
void low_task(void *pvParameters);
void middle_task(void *pvParameters);
void high_task(void *pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 任务相关参数 */
#define START_TASK_SIZE     256
#define START_TASK_PRIO     1
TaskHandle_t start_Task_Handle;

#define LOW_TASK_SIZE       256
#define LOW_TASK_PRIO       2
TaskHandle_t low_task_Handle;

#define MIDDLE_TASK_SIZE    256
#define MIDDLE_TASK_PRIO    3
TaskHandle_t middle_task_Handle;

#define HIGH_TASK_SIZE      256
#define HIGH_TASK_PRIO      4
TaskHandle_t high_task_Handle;

SemaphoreHandle_t mutex_semphr = NULL;    // 互斥信号量句柄

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // 创建开始任务
  xTaskCreate((TaskFunction_t )start_task,
              (char *         )"start_task",
              (uint16_t       )START_TASK_SIZE,
              (void *         )NULL,
              (UBaseType_t    )START_TASK_PRIO,
              (TaskHandle_t * )&start_Task_Handle);

  vTaskStartScheduler(); // 开启调度器

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
}

/* USER CODE BEGIN 4 */

void start_task(void *pvParameters)
{
  taskENTER_CRITICAL();

  mutex_semphr = xSemaphoreCreateMutex();  // 创建互斥信号量
  if (mutex_semphr != NULL)
  {
    printf("\n互斥信号量创建成功");
    xSemaphoreGive(mutex_semphr);    // 释放信号量
  }
  else
  {
    printf("\n互斥信号量创建失败");
  }

  // 创建低优先级任务
  xTaskCreate((TaskFunction_t )low_task,
              (char *         )"low_task",
              (uint16_t       )LOW_TASK_SIZE,
              (void *         )NULL,
              (UBaseType_t    )LOW_TASK_PRIO,
              (TaskHandle_t * )&low_task_Handle);
  // 创建中优先级任务
  xTaskCreate((TaskFunction_t )middle_task,
              (char *         )"middle_task",
              (uint16_t       )MIDDLE_TASK_SIZE,
              (void *         )NULL,
              (UBaseType_t    )MIDDLE_TASK_PRIO,
              (TaskHandle_t * )&middle_task_Handle);
  // 创建高优先级任务
  xTaskCreate((TaskFunction_t )high_task,
              (char *         )"high_task",
              (uint16_t       )HIGH_TASK_SIZE,
              (void *         )NULL,
              (UBaseType_t    )HIGH_TASK_PRIO,
              (TaskHandle_t * )&high_task_Handle);
  taskEXIT_CRITICAL();
  // 删除开始任务
  vTaskDelete(start_Task_Handle);
}

void low_task(void *pvParameters)
{
  BaseType_t error_state;
  uint32_t limit;
  for (;;)
  {
    if (mutex_semphr != NULL)
    {
      // 一直等待互斥信号量
      error_state = xSemaphoreTake(mutex_semphr, portMAX_DELAY);   
      if (error_state == pdTRUE)
      {
        printf("\n低优先级任务获取到信号量，正在占用...");
      }
      else
      {
        printf("\n低优先级任务获取信号量失败");
      }
      #if 0
      // 模拟低优先级占用信号量
      for (limit = 0; limit < 5000000; limit++)
      {
        taskYIELD();      // 一直调用任务切换 
      }
      #else
      HAL_Delay(10000);
      #endif
      // 释放互斥信号量
      printf("\n低优先级任务占用信号量结束，即将释放");
      xSemaphoreGive(mutex_semphr);
    }
    vTaskDelay(500);
  }
}

void middle_task(void *pvParameters)
{
  for (;;)
  {
    printf("\n中优先级任务正在运行");
    vTaskDelay(1000);
  }
}

void high_task(void *pvParameters)
{
  for (;;)
  {
    if (mutex_semphr != NULL)
    {
      printf("\n高优先级任务正在等待信号量...");
      // 一直等待互斥信号量
      xSemaphoreTake(mutex_semphr, portMAX_DELAY);   
      printf("\n高优先级任务获取到信号量，即将释放...");
      xSemaphoreGive(mutex_semphr);    // 释放信号量
    }
    vTaskDelay(1000);
  }
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
