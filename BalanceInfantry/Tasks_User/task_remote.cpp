//
// Created by 45441 on 2023/9/23.
//
#include "cmsis_os2.h"
#include "bsp_usart.h"
#include "driver_remote.h"

extern osSemaphoreId_t RemoteSemHandle;
uint8_t remoteFlag = DISABLE;
void Remote_Task(void *argument)
{
    /* USER CODE BEGIN Remote_Task */
    /* Infinite loop */
    for(;;)
    {

        osDelay(1);
    }
    /* USER CODE END Remote_Task */
}