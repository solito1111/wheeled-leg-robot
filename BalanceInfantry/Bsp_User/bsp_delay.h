//
// Created by 45441 on 2023/7/6.
//

#ifndef INFANTRYGIMBALC_BSP_DELAY_H
#define INFANTRYGIMBALC_BSP_DELAY_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
//C
extern void delay_init(void);
extern void delay_us(uint16_t nus);
extern void delay_ms(uint16_t nms);
#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_BSP_DELAY_H
