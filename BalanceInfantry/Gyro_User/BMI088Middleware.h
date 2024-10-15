//
// Created by 45441 on 2023/9/15.
//

#ifndef INFANTRYGIMBALC_BMI088MIDDLEWARE_H
#define INFANTRYGIMBALC_BMI088MIDDLEWARE_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
//C
extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

extern SPI_HandleTypeDef *BMI088_SPI;

#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_BMI088MIDDLEWARE_H
