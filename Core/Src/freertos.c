/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "rover_firmware.h"
#include "test_comunication.h"
#include "canParser.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotorControlTas */
osThreadId_t MotorControlTasHandle;
uint32_t encoderReadBuffer[ 128 ];
osStaticThreadDef_t encoderReadControlBlock;
const osThreadAttr_t MotorControlTas_attributes = {
  .name = "MotorControlTas",
  .cb_mem = &encoderReadControlBlock,
  .cb_size = sizeof(encoderReadControlBlock),
  .stack_mem = &encoderReadBuffer[0],
  .stack_size = sizeof(encoderReadBuffer),
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for canTxTask */
osThreadId_t canTxTaskHandle;
uint32_t canTxTaskBuffer[ 256 ];
osStaticThreadDef_t canTxTaskControlBlock;
const osThreadAttr_t canTxTask_attributes = {
  .name = "canTxTask",
  .cb_mem = &canTxTaskControlBlock,
  .cb_size = sizeof(canTxTaskControlBlock),
  .stack_mem = &canTxTaskBuffer[0],
  .stack_size = sizeof(canTxTaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for MPUCanTxTask */
osThreadId_t MPUCanTxTaskHandle;
uint32_t MPUCanTxTaskBuffer[ 128 ];
osStaticThreadDef_t MPUCanTxTaskControlBlock;
const osThreadAttr_t MPUCanTxTask_attributes = {
  .name = "MPUCanTxTask",
  .cb_mem = &MPUCanTxTaskControlBlock,
  .cb_size = sizeof(MPUCanTxTaskControlBlock),
  .stack_mem = &MPUCanTxTaskBuffer[0],
  .stack_size = sizeof(MPUCanTxTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);

	rover_t* const rover = rover_get_instance();

	HAL_CAN_GetRxMessage(hcan, rover->can_config.rx_fifo, &rover->can_manager.config->rx_header, rover->can_manager.rx_data);

	if (rover->can_manager.config->rx_header.StdId == CAN_MESSAGES_REFERENCE_MSG_ID && rover->can_manager.config->rx_header.DLC == CAN_MESSAGES_REFERENCE_DLC_BYTES) {
		can_messages_reference_t ref;
		if (can_msg_unpack_reference_frame(rover->can_manager.rx_data, &ref) == CAN_MESSAGES_OK) {

			rover->reference_fl_rpm = saturate_rpm(ref.front_left_rpm);
			rover->reference_rl_rpm = saturate_rpm(ref.rear_left_rpm);
			rover->reference_fr_rpm = saturate_rpm(ref.front_right_rpm);
			rover->reference_rr_rpm = saturate_rpm(ref.rear_right_rpm);

			rover->can_manager.message_received = CAN_MANAGER_RECEIVED_NEW_MESSAGE;
		}
	}

	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void startMotorControl(void *argument);
void StartCanTxTask(void *argument);
void MPUCanTxFunc(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	if(rover_init() != ROVER_OK ){
		Error_Handler();
	}
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MotorControlTas */
  MotorControlTasHandle = osThreadNew(startMotorControl, NULL, &MotorControlTas_attributes);

  /* creation of canTxTask */
  canTxTaskHandle = osThreadNew(StartCanTxTask, NULL, &canTxTask_attributes);

  /* creation of MPUCanTxTask */
  MPUCanTxTaskHandle = osThreadNew(MPUCanTxFunc, NULL, &MPUCanTxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_startMotorControl */
/**
* @brief Function implementing the MotorControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startMotorControl */
void startMotorControl(void *argument)
{
  /* USER CODE BEGIN startMotorControl */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(ENCODER_SAMPLING_TIME * 1000);
	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	for(;;)
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
		if( rover_enc_can_tx_step()!= ROVER_OK){
			Error_Handler();
		}

		//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
	}
  /* USER CODE END startMotorControl */
}

/* USER CODE BEGIN Header_StartCanTxTask */
/**
* @brief Function implementing the canTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTxTask */
void StartCanTxTask(void *argument)
{
  /* USER CODE BEGIN StartCanTxTask */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(CAN_TX_PERIOD_MS);
	xLastWakeTime = xTaskGetTickCount();
	for (;;) {
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		if (rover_can_tx_step() != ROVER_OK) {
			Error_Handler();
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	}
  /* USER CODE END StartCanTxTask */
}

/* USER CODE BEGIN Header_MPUCanTxFunc */
/**
* @brief Function implementing the MPUCanTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MPUCanTxFunc */
void MPUCanTxFunc(void *argument)
{
  /* USER CODE BEGIN MPUCanTxFunc */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(MPU_CAN_TX_PERIOD_MS );
	xLastWakeTime = xTaskGetTickCount();
	for (;;){
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		if (rover_imu_can_tx_step()!= ROVER_OK) {
			Error_Handler();
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	}
  /* USER CODE END MPUCanTxFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

