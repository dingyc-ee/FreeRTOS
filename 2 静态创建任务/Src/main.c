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

#include "FreeRTOS.h"
#include "task.h"

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

// 开始任务
#define START_TASK_PRIO     1     // 开始任务优先级
#define START_TASK_SIZE     128   // 开始任务堆栈大小
StackType_t Start_Task_Stack[START_TASK_SIZE];  // 开始任务堆栈
StaticTask_t StartTaskTCB;            // 开始任务控制块
TaskHandle_t Start_Task_Handle;       // 开始任务句柄
void start_task(void *pvParameters);  // 开始任务函数

// 任务1
#define TASK1_TASK_PRIO     2     
#define TASK1_TASK_SIZE     128       
StackType_t Task1_Stack[TASK1_TASK_SIZE]; 
StaticTask_t Task1TaskTCB;          
TaskHandle_t Task1_Handle;       
void task1_task(void *pvParameters);  

// 任务2
#define TASK2_TASK_PRIO     3     
#define TASK2_TASK_SIZE     128       
StackType_t Task2_Stack[TASK2_TASK_SIZE]; 
StaticTask_t Task2TaskTCB;          
TaskHandle_t Task2_Handle;       
void task2_task(void *pvParameters);

static StaticTask_t IdleTaskTCB;                              // 创建空闲任务控制块
static StackType_t IdleTaskStack[configMINIMAL_STACK_SIZE];   // 创建空闲任务堆栈   

// 空闲任务所需内存
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &IdleTaskTCB;
  *ppxIdleTaskStackBuffer = IdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

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
  Start_Task_Handle = xTaskCreateStatic((TaskFunction_t ) start_task,
                                        (char *         ) "start_task",
                                        (uint32_t       ) START_TASK_SIZE,
                                        (void *         ) NULL,
                                        (UBaseType_t    ) START_TASK_PRIO,
                                        (StackType_t *  ) Start_Task_Stack,
                                        (StaticTask_t * ) &StartTaskTCB);

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
  // 创建任务1
  Task1_Handle = xTaskCreateStatic((TaskFunction_t ) task1_task,
                                   (char *         ) "task1_task",
                                   (uint32_t       ) TASK1_TASK_SIZE,
                                   (void *         ) NULL,
                                   (UBaseType_t    ) TASK1_TASK_PRIO,
                                   (StackType_t *  ) Task1_Stack,
                                   (StaticTask_t * ) &Task1TaskTCB);

  // 创建任务2
  Task2_Handle = xTaskCreateStatic((TaskFunction_t ) task2_task,
                                   (char *         ) "task2_task",
                                   (uint32_t       ) TASK2_TASK_SIZE,
                                   (void *         ) NULL,
                                   (UBaseType_t    ) TASK2_TASK_PRIO,
                                   (StackType_t *  ) Task2_Stack,
                                   (StaticTask_t * ) &Task2TaskTCB);

  vTaskDelete(Start_Task_Handle);     // 删除开始任务
}

void task1_task(void *pvParameters)
{
  uint16_t count1 = 0;
  for (;;)
  {
    count1++;
    LED_B_TOGGLE;
    printf("Task1 running %d\n", count1);
    if (count1 == 5)
    {
      vTaskDelete(Task2_Handle);  // 任务1执行5次，则删除任务2
      printf("delete task2!!!\n");
    }

    vTaskDelay(1000);
  }
}

void task2_task(void *pvParameters)
{
  uint16_t count2 = 0;
  for (;;)
  {
    count2++;
    printf("Task2 running %d\n", count2);
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
