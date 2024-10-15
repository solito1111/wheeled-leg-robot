/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for buzzerTask */
osThreadId_t buzzerTaskHandle;
const osThreadAttr_t buzzerTask_attributes = {
  .name = "buzzerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for remoteTask */
osThreadId_t remoteTaskHandle;
const osThreadAttr_t remoteTask_attributes = {
  .name = "remoteTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for communicateTask */
osThreadId_t communicateTaskHandle;
const osThreadAttr_t communicateTask_attributes = {
  .name = "communicateTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for chassisTask */
osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for communicate2Tas */
osThreadId_t communicate2TasHandle;
const osThreadAttr_t communicate2Tas_attributes = {
  .name = "communicate2Tas",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for judgeUnpackTask */
osThreadId_t judgeUnpackTaskHandle;
const osThreadAttr_t judgeUnpackTask_attributes = {
  .name = "judgeUnpackTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for stuinteractiveT */
osThreadId_t stuinteractiveTHandle;
const osThreadAttr_t stuinteractiveT_attributes = {
  .name = "stuinteractiveT",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for powerTask */
osThreadId_t powerTaskHandle;
const osThreadAttr_t powerTask_attributes = {
  .name = "powerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for communicate3Tas */
osThreadId_t communicate3TasHandle;
const osThreadAttr_t communicate3Tas_attributes = {
  .name = "communicate3Tas",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for lostCounterTask */
osThreadId_t lostCounterTaskHandle;
const osThreadAttr_t lostCounterTask_attributes = {
  .name = "lostCounterTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for can1Queue */
osMessageQueueId_t can1QueueHandle;
const osMessageQueueAttr_t can1Queue_attributes = {
  .name = "can1Queue"
};
/* Definitions for can2Queue */
osMessageQueueId_t can2QueueHandle;
const osMessageQueueAttr_t can2Queue_attributes = {
  .name = "can2Queue"
};
/* Definitions for RemoteSem */
osSemaphoreId_t RemoteSemHandle;
const osSemaphoreAttr_t RemoteSem_attributes = {
  .name = "RemoteSem"
};
/* Definitions for refereeEvent */
osEventFlagsId_t refereeEventHandle;
const osEventFlagsAttr_t refereeEvent_attributes = {
  .name = "refereeEvent"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Led_Task(void *argument);
void Buzzer_Task(void *argument);
void Gyro_task(void *argument);
void Remote_Task(void *argument);
void Communicate_Task(void *argument);
void Chassis_Task(void *argument);
void Communicate2_Task(void *argument);
void Judge_Unpack_Task(void *argument);
void StuInteractive_Task(void *argument);
void PowerTask(void *argument);
void Communicate3_Task(void *argument);
void LostCounter_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of RemoteSem */
  RemoteSemHandle = osSemaphoreNew(1, 1, &RemoteSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of can1Queue */
  can1QueueHandle = osMessageQueueNew (512, sizeof(size10_t), &can1Queue_attributes);

  /* creation of can2Queue */
  can2QueueHandle = osMessageQueueNew (512, sizeof(size10_t), &can2Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(Led_Task, NULL, &ledTask_attributes);

  /* creation of buzzerTask */
  buzzerTaskHandle = osThreadNew(Buzzer_Task, NULL, &buzzerTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(Gyro_task, NULL, &imuTask_attributes);

  /* creation of remoteTask */
  remoteTaskHandle = osThreadNew(Remote_Task, NULL, &remoteTask_attributes);

  /* creation of communicateTask */
  communicateTaskHandle = osThreadNew(Communicate_Task, NULL, &communicateTask_attributes);

  /* creation of chassisTask */
  chassisTaskHandle = osThreadNew(Chassis_Task, NULL, &chassisTask_attributes);

  /* creation of communicate2Tas */
  communicate2TasHandle = osThreadNew(Communicate2_Task, NULL, &communicate2Tas_attributes);

  /* creation of judgeUnpackTask */
  judgeUnpackTaskHandle = osThreadNew(Judge_Unpack_Task, NULL, &judgeUnpackTask_attributes);

  /* creation of stuinteractiveT */
  stuinteractiveTHandle = osThreadNew(StuInteractive_Task, NULL, &stuinteractiveT_attributes);

  /* creation of powerTask */
  powerTaskHandle = osThreadNew(PowerTask, NULL, &powerTask_attributes);

  /* creation of communicate3Tas */
  communicate3TasHandle = osThreadNew(Communicate3_Task, NULL, &communicate3Tas_attributes);

  /* creation of lostCounterTask */
  lostCounterTaskHandle = osThreadNew(LostCounter_Task, NULL, &lostCounterTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of refereeEvent */
  refereeEventHandle = osEventFlagsNew(&refereeEvent_attributes);

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

/* USER CODE BEGIN Header_Led_Task */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Led_Task */
__weak void Led_Task(void *argument)
{
  /* USER CODE BEGIN Led_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Led_Task */
}

/* USER CODE BEGIN Header_Buzzer_Task */
/**
* @brief Function implementing the buzzerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Buzzer_Task */
__weak void Buzzer_Task(void *argument)
{
  /* USER CODE BEGIN Buzzer_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Buzzer_Task */
}

/* USER CODE BEGIN Header_Gyro_task */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gyro_task */
__weak void Gyro_task(void *argument)
{
  /* USER CODE BEGIN Gyro_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gyro_task */
}

/* USER CODE BEGIN Header_Remote_Task */
/**
* @brief Function implementing the remoteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remote_Task */
__weak void Remote_Task(void *argument)
{
  /* USER CODE BEGIN Remote_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Remote_Task */
}

/* USER CODE BEGIN Header_Communicate_Task */
/**
* @brief Function implementing the communicateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Communicate_Task */
__weak void Communicate_Task(void *argument)
{
  /* USER CODE BEGIN Communicate_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Communicate_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the chassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void *argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_Communicate2_Task */
/**
* @brief Function implementing the communicate2Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Communicate2_Task */
__weak void Communicate2_Task(void *argument)
{
  /* USER CODE BEGIN Communicate2_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Communicate2_Task */
}

/* USER CODE BEGIN Header_Judge_Unpack_Task */
/**
* @brief Function implementing the judgeUnpackTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Judge_Unpack_Task */
__weak void Judge_Unpack_Task(void *argument)
{
  /* USER CODE BEGIN Judge_Unpack_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Judge_Unpack_Task */
}

/* USER CODE BEGIN Header_StuInteractive_Task */
/**
* @brief Function implementing the stuinteractiveT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StuInteractive_Task */
__weak void StuInteractive_Task(void *argument)
{
  /* USER CODE BEGIN StuInteractive_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StuInteractive_Task */
}

/* USER CODE BEGIN Header_PowerTask */
/**
* @brief Function implementing the powerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PowerTask */
__weak void PowerTask(void *argument)
{
  /* USER CODE BEGIN PowerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PowerTask */
}

/* USER CODE BEGIN Header_Communicate3_Task */
/**
* @brief Function implementing the communicate3Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Communicate3_Task */
__weak void Communicate3_Task(void *argument)
{
  /* USER CODE BEGIN Communicate3_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Communicate3_Task */
}

/* USER CODE BEGIN Header_LostCounter_Task */
/**
* @brief Function implementing the lostCounterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LostCounter_Task */
__weak void LostCounter_Task(void *argument)
{
  /* USER CODE BEGIN LostCounter_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LostCounter_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

