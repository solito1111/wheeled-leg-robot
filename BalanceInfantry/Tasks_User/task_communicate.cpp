//
// Created by 45441 on 2023/9/30.
//

#include "driver_communicate.h"
#include "driver_remote.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "driver_chassis.h"


uint32_t count;
void Communicate_Task(void *argument)
{
    /* USER CODE BEGIN Communicate_Task */
    /* Infinite loop */
    for(;;)
    {
        if(motor_OK==1) {
            Can1CalculateSend(remoteFlag);
//            osDelay(1);
        }
        else{
            osDelay(1);
        }
    }
    /* USER CODE END Communicate_Task */
}

void Communicate2_Task(void *argument)
{
    /* USER CODE BEGIN Communicate2_Task */
    /* Infinite loop */
    for(;;)
    {
        if(motor_OK==1) {
            Can2CalculateSend(remoteFlag);
            SC_send_message(0);
            osDelay(1);
        }
        else{
            SC_send_message(0);
            osDelay(1);
        }

    }
    /* USER CODE END Communicate2_Task */
}

void Communicate3_Task(void *argument)
{
    /* USER CODE BEGIN Communicate3_Task */
    /* Infinite loop */
    for(;;)
    {
        SendtoGimbalMessage();
        osDelay(1);
    }
    /* USER CODE END Communicate3_Task */
}