//
// Created by 45441 on 2024/1/21.
//

#ifndef INFANTRYGIMBALC_DRIVER_POWER_H
#define INFANTRYGIMBALC_DRIVER_POWER_H
#include "main.h"
#include "cmsis_os2.h"
#ifdef __cplusplus
extern "C" {
#endif
//C
#define POWER_BUFFER_MAX (60)
#define POWER_CHARGE_MAX      (120)          // 电容充电限制80w 保守了 记得改高
#define POWER_DISCHARGE_MAX   (180)         //  电容放电限制150w
#define MAX_FULL_CAP_VOLTAGE       26.0f         // 电容最大电压
#define MIN_FULL_CAP_VOLTAGE       25.5f
#define MIN_CAP_VOLTAGE       8.0f          // 电容最小电压

#define Power_delta 50.f //30.f
#define super_cap_ref  55.f //55.f

//电容PID控制的参数
#define SC_KP                   0.2f
#define SC_KI                   0.0000f
#define SC_KD                   0.f

//send can id 312
//recv can id 311
typedef struct {        // 单位
    float V_IN;         // V
    float V_SC;         // V
    float I_IN_SC;      // A
    float I_CHASSIS;    // A
}SC_feedback_t;

typedef struct {
    uint8_t enable;
    float set_power_charge;     //发送给电容控制板数据，给电容充电为正，放电为负（单位 W）
}SC_set_t;

typedef struct {
    SC_feedback_t scFeedback;
    SC_set_t scSet;
    struct {
        float p_sc;			//电容发送回来的电容功率数据  充电为正
        float p_wheel;		//四轮电机功率
        float last_p_wheel;
    } power_fb;
} Super_Cap_t;

enum SuperCapState{    //电容充放电情况	0 电容没电	1 电容可用	2 电容满电
    FULL = 0,
    READY = 1,
    EMPTY = 2,
    SUPERC_ERROR = 3,
    SUPERC_DEBUG = 4
};
extern uint16_t robot_power_limit ;
extern  float capPower;
extern Super_Cap_t superCap ;

void SC_recv_message(uint8_t *data);
void PowerTask(void *argument);

#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_DRIVER_POWER_H
