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
void start_task(void *pvParameters);
void led_task(void *pvParameters);
void query_task(void *pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define START_TASK_SIZE 128
#define START_TASK_PRIO 1
TaskHandle_t Start_Task_Handle;

#define LED_TASK_SIZE 128
#define LED_TASK_PRIO 2
TaskHandle_t led_task_Handle;

#define QUERY_TASK_SIZE 256
#define QUERY_TASK_PRIO 3
TaskHandle_t query_task_Handle;

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
  xTaskCreate((TaskFunction_t)start_task,
              (char *)"start_task",
              (uint16_t)START_TASK_SIZE,
              (void *)NULL,
              (UBaseType_t)START_TASK_PRIO,
              (TaskHandle_t *)&Start_Task_Handle);

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  // 创建led任务
  xTaskCreate((TaskFunction_t)led_task,
              (char *)"led_task",
              (uint16_t)LED_TASK_SIZE,
              (void *)NULL,
              (UBaseType_t)LED_TASK_PRIO,
              (TaskHandle_t *)&led_task_Handle);
  // 创建查询任务
  xTaskCreate((TaskFunction_t)query_task,
              (char *)"query_task",
              (uint16_t)QUERY_TASK_SIZE,
              (void *)NULL,
              (UBaseType_t)QUERY_TASK_PRIO,
              (TaskHandle_t *)&query_task_Handle);
  // 删除开始任务
  vTaskDelete(Start_Task_Handle);
}

void led_task(void *pvParameters)
{
  for (;;)
  {
    LED_G_TOGGLE;
    vTaskDelay(500);
  }
}

void query_task(void *pvParameters)
{
  UBaseType_t task_priority;
  UBaseType_t i, arraySize;
  TaskStatus_t *StatusArray; // 定义任务状态指针
  uint32_t totalRunTime;

  /* 查询任务优先级 */
  printf("\n**************查询任务优先级****************\n");
  task_priority = uxTaskPriorityGet(led_task_Handle); // 打印led任务优先级
  printf("led_task priority = %ld\n", task_priority);

  task_priority = uxTaskPriorityGet(query_task_Handle); // 打印查询任务优先级
  printf("query_task priority = %ld\n", task_priority);

  arraySize = uxTaskGetNumberOfTasks();                         // 获取任务个数
  StatusArray = pvPortMalloc(arraySize * sizeof(TaskStatus_t)); // 指针分配内存
  /* 内存分配成功 */
  if (StatusArray != NULL)
  {
    arraySize = uxTaskGetSystemState((TaskStatus_t *)StatusArray,
                                     (UBaseType_t)arraySize,
                                     (uint32_t *)&totalRunTime);

    printf("\n**************获取任务有关信息****************\n");
    printf("TaskName\t\tPriority\t\tTaskNumber\t\tStackHighWater\t\t\r\n");
    for (i = 0; i < arraySize; i++)
    {
      // 串口打印出获取到的系统任务有关信息：如任务名称、任务优先级、任务编号、剩余堆栈
      printf("%s\t\t%d\t\t\t%d\t\t\t%d\t\t\t\r\n",
             StatusArray[i].pcTaskName,
             StatusArray[i].uxCurrentPriority,
             StatusArray[i].xTaskNumber,
             StatusArray[i].usStackHighWaterMark);
    }
    vPortFree(StatusArray);
  }

  // 查询单个任务状态信息
  TaskStatus_t test_task_status;
  printf("\n************查询单个任务的状态信息**************\n");
  vTaskGetInfo(query_task_Handle, &test_task_status, !pdFALSE, eInvalid);
  printf("任务名                  %s\n", test_task_status.pcTaskName);
  printf("任务编号                %d\n", test_task_status.xTaskNumber);
  printf("任务状态                %d\n", test_task_status.eCurrentState);
  printf("任务当前优先级          %d\n", test_task_status.uxCurrentPriority);
  printf("任务堆栈基地址          %p\n", test_task_status.pxStackBase);
  printf("任务历史剩余堆栈        %d\n", test_task_status.usStackHighWaterMark);

  // 通过任务名查询任务句柄
  TaskHandle_t testHandle;
  testHandle = xTaskGetHandle("query_task");

  printf("\n************通过任务名查询任务句柄**************\n");
  printf("test_handle = %p\n", testHandle);
  printf("query_task_handle = %p\n", query_task_Handle);

  printf("\n************查询任务历史剩余堆栈大小**************\n");
  UBaseType_t left_stack;
  left_stack = uxTaskGetStackHighWaterMark(query_task_Handle); // 获取剩余堆栈
  printf("query_task left %ld stack\n", left_stack);

  printf("\n*****************查询任务状态*******************\n");
  eTaskState task_state;
  task_state = eTaskGetState(led_task_Handle);
  printf("led_task state = %d\n", task_state);

  task_state = eTaskGetState(query_task_Handle);
  printf("query_task state = %d\t", task_state);

  switch (task_state)
  {
  case eRunning:
    printf("running\n");
    break;

  case eReady:
    printf("ready\n");
    break;

  case eBlocked:
    printf("blocked\n");
    break;

  case eSuspended:
    printf("suspended\n");
    break;

  case eDeleted:
    printf("deleted\n");
    break;

  case eInvalid:
    printf("invalid\n");
    break;

  default:
    break;
  }

  // 测试打印列表函数
  printf("\n*****************测试列表打印函数*******************\n");
  char *plist;
  plist = pvPortMalloc(1024 * 2 * sizeof(char));  // 申请1024 * 2字节内存
  vTaskList(plist);
  printf("%s", plist);
  vPortFree(plist);     // 释放内存

  for (;;)
  {
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
  if (htim->Instance == TIM2)
  {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
