//
// Created by 45441 on 2023/9/15.
//

#ifndef INFANTRYGIMBALC_BSP_PWM_H
#define INFANTRYGIMBALC_BSP_PWM_H
#include "stdint.h"
#include "tim.h"
#ifdef __cplusplus
extern "C" {
#endif
//C


void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value);

#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_BSP_PWM_H
