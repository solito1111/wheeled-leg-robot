//
// Created by 45441 on 2023/7/5.
//

#include "driver_led.h"
void LED_R_Light_On(){
    HAL_GPIO_TogglePin(LED_GPIO_PORT,LED_R_PIN);
}

void LED_G_Light_On(){
    HAL_GPIO_TogglePin(LED_GPIO_PORT,LED_G_PIN);
}

void LED_B_Light_On(){
    HAL_GPIO_TogglePin(LED_GPIO_PORT,LED_B_PIN);
}