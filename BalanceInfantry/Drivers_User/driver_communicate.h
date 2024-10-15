//
// Created by 45441 on 2023/9/30.
//

#ifndef INFANTRYGIMBALC_DRIVER_COMMUNICATE_H
#define INFANTRYGIMBALC_DRIVER_COMMUNICATE_H
#include "cmsis_os2.h"
#include "main.h"
#include "driver_power.h"
#ifdef __cplusplus
extern "C" {
#endif
//C




    void SendtoGimbalMessage();
    void Can1CalculateSend(uint8_t remote);
    void Can2CalculateSend(uint8_t remote);
    void Communicate_Task(void *argument);
    void Communicate2_Task(void *argument);
    void Communicate3_Task(void *argument);

void SC_send_message(Super_Cap_t *super_cap);


#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_DRIVER_COMMUNICATE_H
