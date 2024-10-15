//
// Created by 45441 on 2023/11/6.
//

#ifndef INFANTRYGIMBALC_PROTOCOL_H
#define INFANTRYGIMBALC_PROTOCOL_H
#ifdef __cplusplus
#include "stm32f4xx_hal.h"

extern "C" {
#endif
//C
uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);

void append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
void append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);

#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_PROTOCOL_H
