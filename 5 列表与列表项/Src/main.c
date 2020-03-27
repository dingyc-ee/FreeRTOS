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
#include "list.h"

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
void list_task(void *pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define START_TASK_SIZE 128
#define START_TASK_PRIO 1
TaskHandle_t Start_Task_Handle;

#define LED_TASK_SIZE 128
#define LED_TASK_PRIO 2
TaskHandle_t led_task_Handle;

#define LIST_TASK_SIZE 128
#define LIST_TASK_PRIO 3
TaskHandle_t list_task_Handle;

// 定义一个列表和三个列表项
List_t testList;
ListItem_t ListItem1;
ListItem_t ListItem2;
ListItem_t ListItem3;

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
  taskENTER_CRITICAL();
  // 创建led任务
  xTaskCreate((TaskFunction_t)led_task,
              (char *)"led_task",
              (uint16_t)LED_TASK_SIZE,
              (void *)NULL,
              (UBaseType_t)LED_TASK_PRIO,
              (TaskHandle_t *)&led_task_Handle);
  // 创建列表任务
  xTaskCreate((TaskFunction_t)list_task,
              (char *)"list_task",
              (uint16_t)LIST_TASK_SIZE,
              (void *)NULL,
              (UBaseType_t)LIST_TASK_PRIO,
              (TaskHandle_t *)&list_task_Handle);
  // 删除开始任务
  vTaskDelete(Start_Task_Handle);
  taskEXIT_CRITICAL();
}

void led_task(void *pvParameters)
{
  for (;;)
  {
    LED_R_TOGGLE;
    vTaskDelay(500);
  }
}

void list_task(void *pvParameters)
{
  vListInitialise(&testList);      // 初始化列表
  vListInitialiseItem(&ListItem1); // 初始化列表项1
  vListInitialiseItem(&ListItem2); // 初始化列表项2
  vListInitialiseItem(&ListItem3); // 初始化列表项3

  ListItem1.xItemValue = 40;
  ListItem2.xItemValue = 60;
  ListItem3.xItemValue = 50;

  // 打印列表和其他列表项的地址
  printf("\n/*********************列表和列表项地址************************/\n");
  printf("项目                            地址                          \n");
  printf("testList                        %#X                          \n", (int)&testList);
  printf("testList->pxIndex               %#X                          \n", (int)testList.pxIndex);
  printf("testList->xListEnd              %#X                          \n", (int)&testList.xListEnd);
  printf("ListItem1                       %#X                          \n", (int)&ListItem1);
  printf("ListItem2                       %#X                          \n", (int)&ListItem2);
  printf("ListItem3                       %#X                          \n", (int)&ListItem3);
  printf("/***************************结束*****************************/\n");
  printf("按下K1按键继续！\r\n\r\n");
  while (key_scan(KEY1_GPIO_Port, KEY1_Pin) != KEY_ON);

  // testList插入ListItem1
  vListInsert(&testList, &ListItem1); // 插入列表项1
  printf("\n/*******************添加列表项ListItem1**********************/\n");
  printf("项目                            地址                          \n");
  printf("testList->xListEnd->pxNext      %#X                          \n", (int)testList.xListEnd.pxNext);
  printf("ListItem1->pxNext               %#X                          \n", (int)ListItem1.pxNext);
  printf("/**********************前后向连接分割线***********************/\n");
  printf("testList->xListEnd->pxPrevious  %#X                          \n", (int)testList.xListEnd.pxPrevious);
  printf("ListItem1->pxPrevious           %#X                          \n", (int)ListItem1.pxPrevious);
  printf("/***************************结束*****************************/\n");
  printf("按下K1按键继续！\r\n\r\n");
  while (key_scan(KEY1_GPIO_Port, KEY1_Pin) != KEY_ON);

  // testList插入ListItem2
  vListInsert(&testList, &ListItem2); // 插入列表项2
  printf("\n/*******************添加列表项ListItem2**********************/\n");
  printf("项目                            地址                          \n");
  printf("testList->xListEnd->pxNext      %#X                          \n", (int)testList.xListEnd.pxNext);
  printf("ListItem1->pxNext               %#X                          \n", (int)ListItem1.pxNext);
  printf("ListItem2->pxNext               %#X                          \n", (int)ListItem2.pxNext);
  printf("/**********************前后向连接分割线***********************/\n");
  printf("testList->xListEnd->pxPrevious  %#X                          \n", (int)testList.xListEnd.pxPrevious);
  printf("ListItem1->pxPrevious           %#X                          \n", (int)ListItem1.pxPrevious);
  printf("ListItem2->pxPrevious           %#X                          \n", (int)ListItem2.pxPrevious);
  printf("/***************************结束*****************************/\n");
  printf("按下K1按键继续！\r\n\r\n");
  while (key_scan(KEY1_GPIO_Port, KEY1_Pin) != KEY_ON);

  // testList插入ListItem3
  vListInsert(&testList, &ListItem3); // 插入列表项3
  printf("\n/*******************添加列表项ListItem3**********************/\n");
  printf("项目                            地址                          \n");
  printf("testList->xListEnd->pxNext      %#X                          \n", (int)testList.xListEnd.pxNext);
  printf("ListItem1->pxNext               %#X                          \n", (int)ListItem1.pxNext);
  printf("ListItem3->pxNext               %#X                          \n", (int)ListItem3.pxNext);
  printf("ListItem2->pxNext               %#X                          \n", (int)ListItem2.pxNext);
  printf("/**********************前后向连接分割线***********************/\n");
  printf("testList->xListEnd->pxPrevious  %#X                          \n", (int)testList.xListEnd.pxPrevious);
  printf("ListItem1->pxPrevious           %#X                          \n", (int)ListItem1.pxPrevious);
  printf("ListItem3->pxPrevious           %#X                          \n", (int)ListItem3.pxPrevious);
  printf("ListItem2->pxPrevious           %#X                          \n", (int)ListItem2.pxPrevious);
  printf("/***************************结束*****************************/\n");
  printf("按下K1按键继续！\r\n\r\n");
  while (key_scan(KEY1_GPIO_Port, KEY1_Pin) != KEY_ON);

  // testList删除ListItem2
  uxListRemove(&ListItem2);
  printf("\n/*******************删除列表项ListItem2**********************/\n");
  printf("项目                            地址                          \n");
  printf("testList->xListEnd->pxNext      %#X                          \n", (int)testList.xListEnd.pxNext);
  printf("ListItem1->pxNext               %#X                          \n", (int)ListItem1.pxNext);
  printf("ListItem3->pxNext               %#X                          \n", (int)ListItem3.pxNext);
  printf("/**********************前后向连接分割线***********************/\n");
  printf("testList->xListEnd->pxPrevious  %#X                          \n", (int)testList.xListEnd.pxPrevious);
  printf("ListItem1->pxPrevious           %#X                          \n", (int)ListItem1.pxPrevious);
  printf("ListItem3->pxPrevious           %#X                          \n", (int)ListItem3.pxPrevious);
  printf("/***************************结束*****************************/\n");
  printf("按下K1按键继续！\r\n\r\n");
  while (key_scan(KEY1_GPIO_Port, KEY1_Pin) != KEY_ON);

  // pxIndex向后移一项，这样pxIndex就会指向ListItem1
  testList.pxIndex = testList.pxIndex->pxNext;
  vListInsertEnd(&testList, &ListItem2);	  // 列表末尾添加列表项ListItem2
  printf("\n/*****************在末尾添加列表项ListItem2******************/\n");
  printf("项目                            地址                          \n");
  printf("testList.pxIndex                %#X                          \n", (int)testList.pxIndex);
  printf("testList->xListEnd->pxNext      %#X                          \n", (int)testList.xListEnd.pxNext);
  printf("ListItem2->pxNext               %#X                          \n", (int)ListItem2.pxNext);
  printf("ListItem1->pxNext               %#X                          \n", (int)ListItem1.pxNext);
  printf("ListItem3->pxNext               %#X                          \n", (int)ListItem3.pxNext);
  printf("/**********************前后向连接分割线***********************/\n");
  printf("testList->xListEnd->pxPrevious  %#X                          \n", (int)testList.xListEnd.pxPrevious);
  printf("ListItem2->pxPrevious           %#X                          \n", (int)ListItem2.pxPrevious);
  printf("ListItem1->pxPrevious           %#X                          \n", (int)ListItem1.pxPrevious);
  printf("ListItem3->pxPrevious           %#X                          \n", (int)ListItem3.pxPrevious);
  printf("/***************************结束*****************************/\n");
  printf("按下K1按键继续！\r\n\r\n");
  while (key_scan(KEY1_GPIO_Port, KEY1_Pin) != KEY_ON);
  printf("list_task开始任务调度\n");
  
  for (;;)
  {
    printf("list_task run!\n");
    vTaskDelay(500);
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
