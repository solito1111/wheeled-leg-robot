//
// Created by 45441 on 2023/7/5.
//
#include "bsp_buzzer.h"
#include "cmsis_os.h"
extern uint8_t motor_OK;

void Buzzer_Task(void *argument)
{
    /* USER CODE BEGIN Buzzer_Task */
    /* Infinite loop */
    for(;;)
    {
          if(!motor_OK){
//              playMusic();
            osDelay(300);
          }else{
            buzzer_on(0,0);
          }
        osDelay(100);
    }
    /* USER CODE END Buzzer_Task */
}