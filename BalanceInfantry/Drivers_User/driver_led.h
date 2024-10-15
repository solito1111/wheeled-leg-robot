//
// Created by 45441 on 2023/7/5.
//

#ifndef INFANTRYGIMBAL_C_DRIVER_LED_H
#define INFANTRYGIMBAL_C_DRIVER_LED_H
#include "cmsis_os2.h"
#include "main.h"


#define LED_GPIO_PORT GPIOH
#define LED_R_PIN GPIO_PIN_12
#define LED_G_PIN GPIO_PIN_11
#define LED_B_PIN GPIO_PIN_10
#ifdef __cplusplus
extern "C" {
#endif
//C

void LED_R_Light_On();
void LED_G_Light_On();
void LED_B_Light_On();
void Led_Task(void *argument);
#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBAL_C_DRIVER_LED_H
