//
// Created by 45441 on 2023/7/5.
//
#include"driver_led.h"
#include "cmsis_os2.h"
#include "bsp_buzzer.h"
void Led_Task(void *argument) {
    /* USER CODE BEGIN Led_Task */
    /* Infinite loop */
    for (;;) {
        LED_R_Light_On();
        osDelay(200);
        LED_G_Light_On();
        osDelay(200);
        LED_B_Light_On();
        osDelay(200);
    }
    /* USER CODE END Led_Task */
}
