//
// Created by 45441 on 2023/7/5.
//

#ifndef INFANTRYGIMBALC_BSP_BUZZER_H
#define INFANTRYGIMBALC_BSP_BUZZER_H
#include "main.h"
#include "tim.h"
#include "cmsis_os2.h"
#define MIN_BUZZER_PWM (10000)
#define MAX_BUZZER_PWM (20000)
#define MAX_PSC (1000)
#ifdef __cplusplus
extern "C" {
#endif
//C
void buzzer_on(uint16_t psc, uint16_t pwm);
void Buzzer_On();
    void Buzzer_Task(void *argument);
void playMusic();
#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_BSP_BUZZER_H
