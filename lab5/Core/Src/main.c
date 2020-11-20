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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef unsigned char tByte;
typedef unsigned int  tWord;
typedef unsigned long tLong;
typedef unsigned int  bit;
typedef struct {
	// Pointer to the task (must be a 'void (void)' function)
	void ( * pTask)(void);
	// Delay (ticks) until the function will (next) be run
	tWord Delay;
	// Interval (ticks) between subsequent runs.
	tWord Period;
	// Incremented (by scheduler) when task is due to execute
	tByte RunMe;
} sTask;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCH_MAX_TASKS 5
enum code_G {FALSE, ERROR_SCH_TOO_MANY_TASKS,
			ERROR_SCH_CANNOT_DELETE_TASK,
			ERROR_SCH_WAITING_FOR_SLAVE_TO_ACK,
			ERROR_SCH_WAITING_FOR_START_COMMAND_FROM_MASTER,
			ERROR_SCH_ONE_OR_MORE_SLAVES_DID_NOT_START,
			ERROR_SCH_LOST_SLAVE,
			ERROR_SCH_CAN_BUS_ERROR,
			ERROR_I2C_WRITE_BYTE_AT24C64};
#ifndef TRUE
#define TRUE (!FALSE)
#endif
#define RETURN_NORMAL (bit) 0
#define RETURN_ERROR (bit) 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
tByte Error_code_G = 0;
char uart_buf[50];
int uart_buf_len;
sTask SCH_tasks_G[SCH_MAX_TASKS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/*Scheduler initialization function.  Prepares scheduler
  data structures and sets up timer interrupts at required rate. */
void SCH_Init_T2(void);
/*This function will be updated the remaining time of each
tasks that are added to a queue. It will be called in the interrupt timer, for
example 10 ms.*/
void SCH_Update(void);
/* This function will get the task in the queue to
run.*/
void SCH_Dispatch_Tasks(void);
/*This function is used to add a task to the queue. It should return an ID that
is corresponding with the added task. */
tByte SCH_Add_Task(void (* pFunction)(), const tWord DELAY, const tWord PERIOD);
/* This function is used to delete the task based on its ID*/
bit SCH_Delete_Task(const tByte taskID);
/*To report errors at any part of the scheduled application*/
void SCH_Report_Status(void);
// to call scheduler enters idle mode
void SCH_Go_To_Sleep(void);
/* 5 Tasks to schedule periodically*/
void TaskA(void);
void TaskB(void);
void TaskC(void);
void TaskD(void);
void TaskE(void);
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
  /*Initialize the Scheduler */
  SCH_Init_T2();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize the timer */
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  SCH_Dispatch_Tasks();
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|LD2_Pin|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 LD2_Pin PA6
                           PA7 PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|LD2_Pin|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void SCH_Init_T2(void) {
   tByte i;
   for (i = 0; i < SCH_MAX_TASKS; i++)
      {
	   SCH_Delete_Task(i);
      }
   // Reset the global error variable
   // - SCH_Delete_Task() will generate an error code,
   //   (because the task array is empty)
   Error_code_G = 0;
   uart_buf_len = sprintf(uart_buf, "ADD SUCCESFUL \r\n");
   HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);;
   SCH_Add_Task(TaskA,1,1);
   SCH_Add_Task(TaskB,2,1);
   SCH_Add_Task(TaskC,3,1);
   SCH_Add_Task(TaskD,4,1);
   SCH_Add_Task(TaskE,5,1);
}

void TaskA(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_SET);
	uart_buf_len = sprintf(uart_buf, "Task 0 is processing...");
	HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
	HAL_Delay(1000);
	uart_buf_len = sprintf(uart_buf, "Done\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_RESET);
}

void TaskB(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,GPIO_PIN_SET);
	uart_buf_len = sprintf(uart_buf, "Task 1 is processing...");
	HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
	HAL_Delay(1000);
	uart_buf_len = sprintf(uart_buf, "Done\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,GPIO_PIN_RESET);
}

void TaskC(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
	uart_buf_len = sprintf(uart_buf, "Task 2 is processing...");
	HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
	HAL_Delay(1000);
	uart_buf_len = sprintf(uart_buf, "Done\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
}

void TaskD(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);
	uart_buf_len = sprintf(uart_buf, "Task 3 is processing...");
	HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
	HAL_Delay(1000);
	uart_buf_len = sprintf(uart_buf, "Done\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);
}

void TaskE(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_SET);
	uart_buf_len = sprintf(uart_buf, "Task 4 is processing...");
	HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
	HAL_Delay(1000);
	uart_buf_len = sprintf(uart_buf, "Done\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_RESET);
}

void SCH_Update(void) {
	tByte Index;
// NOTE: calculations are in *TICKS* (not milliseconds)
	for (Index = 0; Index < SCH_MAX_TASKS; Index++) {
		// Check if there is a task at this location
		if (SCH_tasks_G[Index].pTask) {
			uart_buf_len = sprintf(uart_buf, "Task %d. Remaining %d \r\n",Index,SCH_tasks_G[Index].Delay);
			HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
			if (SCH_tasks_G[Index].Delay == 0) {
				// The task is due to run
				SCH_tasks_G[Index].RunMe += 1; // Inc. the 'RunMe' flag
				if (SCH_tasks_G[Index].Period) {
					// Schedule periodic tasks to run again
					SCH_tasks_G[Index].Delay = SCH_tasks_G[Index].Period;
				}
			} else {
				// Not yet ready to run: just decrement the delay
				SCH_tasks_G[Index].Delay -= 1;
			}
		}
	}
}

tByte SCH_Add_Task(void (* pFunction)(), const tWord DELAY, const tWord PERIOD) {
	tByte Index = 0;
// First find a gap in the array (if there is one)
	while ((SCH_tasks_G[Index].pTask != 0) && (Index < SCH_MAX_TASKS)) {
		Index++;
	}
	if (Index == SCH_MAX_TASKS) {// Have we reached the end of the list?
// Task list is full, Set the global error variable
		Error_code_G = ERROR_SCH_TOO_MANY_TASKS;
		return SCH_MAX_TASKS; // Also return an error code
	}
// If we're here, there is a space in the task array
	SCH_tasks_G[Index].pTask = pFunction;
	SCH_tasks_G[Index].Delay = DELAY;
	SCH_tasks_G[Index].Period = PERIOD;
	SCH_tasks_G[Index].RunMe = 0;
	return Index; // return position of task (to allow later deletion)
}

void SCH_Dispatch_Tasks(void) {
	tByte Index;
	// Dispatches (runs) the next task (if one is ready)
	for (Index = 0; Index < SCH_MAX_TASKS; Index++) {
		if (SCH_tasks_G[Index].RunMe > 0) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);
			(*SCH_tasks_G[Index].pTask)(); // Run the task
			SCH_tasks_G[Index].RunMe -= 1; // Reset / reduce RunMe flag
			// Periodic tasks will automatically run again
			// - if this is a 'one shot' task, remove it from the array
			if (SCH_tasks_G[Index].Period == 0) SCH_Delete_Task(Index);
		}
	}
	SCH_Report_Status(); // Report system status
	SCH_Go_To_Sleep(); // The scheduler enters idle mode at this point
}

bit SCH_Delete_Task(const tByte TASK_INDEX) {
   bit Return_code;
   if (SCH_tasks_G[TASK_INDEX].pTask == 0){
      // No task at this location...
      // Set the global error variable
	   uart_buf_len = sprintf(uart_buf, "ERROR_SCH_CANNOT_DELETE_TASK \r\n");
	   HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
	   Error_code_G = ERROR_SCH_CANNOT_DELETE_TASK;
	   // ...also return an error code
	   Return_code = RETURN_ERROR;
      } else{
    	  Return_code = RETURN_NORMAL;
      }
   uart_buf_len = sprintf(uart_buf, "TASK %d DELETE \r\n", TASK_INDEX);
   HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
   SCH_tasks_G[TASK_INDEX].pTask   = 0x0000;
   SCH_tasks_G[TASK_INDEX].Delay   = 0;
   SCH_tasks_G[TASK_INDEX].Period  = 0;
   SCH_tasks_G[TASK_INDEX].RunMe   = 0;
   return Return_code;       // return status
}

void SCH_Report_Status(void) {
	#ifdef SCH_REPORT_ERRORS // ONLY APPLIES IF WE ARE REPORTING ERRORS
	// Check for a new error code
	if (Error_code_G != Last_error_code_G) {
		// Negative logic on LEDs assumed
		Error_port = 255 - Error_code_G;
		Last_error_code_G = Error_code_G;
		if (Error_code_G != 0) {
			Error_tick_count_G = 60000;
		} else {
			Error_tick_count_G = 0;
		}
	} else {
		if (Error_tick_count_G != 0) {
			if (--Error_tick_count_G == 0) {
				Error_code_G = 0; // Reset error code
			}
		}
	}
#endif
	//uart_buf_len = sprintf(uart_buf, "ERROR CODE, %d \r\n", Error_code_G);
	//HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
}

void SCH_Go_To_Sleep(void){
	//PCON
	// INTERUPT HTIM4
	//uart_buf_len = sprintf(uart_buf, "HET VIEC, DI NGU THOI \r\n");
	//HAL_UART_Transmit(&huart2, (uint8_t *) uart_buf, uart_buf_len, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	SCH_Update();
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
