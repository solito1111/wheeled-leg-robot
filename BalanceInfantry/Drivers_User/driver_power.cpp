//
// Created by 45441 on 2024/1/21.
//

#include "driver_power.h"
enum SuperCapState super_cap_state = READY;

Super_Cap_t superCap ;//= {
//        .scSet.enable = 1
//};
uint8_t errorFlag=0;
void SC_recv_message(uint8_t *data)
{
    superCap.scSet.enable = 1;
    float V_SC;

    superCap.power_fb.p_sc = (int16_t)(data[0] | (data[1] << 8)) / 50.f;           //0.02W/LSB
    superCap.power_fb.p_wheel = (int16_t)(data[2] | (data[3] << 8)) / 50.f;        //0.02W/LSB
    V_SC = (int16_t)(data[4] | (data[5] << 8)) / 1000.f;                           //mV -> V
    superCap.scFeedback.V_SC = superCap.scFeedback.V_SC * 0.9f + V_SC * 0.1f;      //低通
    super_cap_state = (SuperCapState) data[6];
}
