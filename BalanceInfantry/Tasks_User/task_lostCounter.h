//
// Created by 45441 on 2024/3/4.
//

#ifndef INFANTRYGIMBALC_TASK_LOSTCOUNTER_H
#define INFANTRYGIMBALC_TASK_LOSTCOUNTER_H
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "main.h"
#include "cmsis_os2.h"
#include "bsp_watchdog.h"

#define LOST_COUNTER_TIME_MS (5)

#define NUMBERS_OF_COUNT		18
enum LostCounterType
{
    REMOTE_LOST_COUNT                 = 0,
    JUDGEMENT_LOST_COUNT			  = 1,
    OVER_SPEED_LOST					  = 2,
    OVER_HEAT_LOST					  = 3,
    FEEDMOTOR_LOST_COUNT 			  = 4,
    JOINT_MOTOR_0					  = 5,
    JOINT_MOTOR_1					  = 6,
    JOINT_MOTOR_2					  = 7,
    JOINT_MOTOR_3					  = 8,
    FRICTION_LOST_COUNT				  = 9,
    GIMBAL_MOTOR_PITCH				  = 10,
    GIMBAL_MOTOR_YAW				  = 11,
    SUPERC_LOST_COUNT				  = 12,
    VISION_LOST_COUNT				  = 13,
    COMMUNICATE_LOST_COUNT			  = 14,
    SUPERC_OUTPUTLOST_COUNT           = 15,
    WHEEL_MOTOR_0                     = 16,
    WHEEL_MOTOR_1                     = 17,
};

#define CHASSIS_LOST_TOLERANCE_MS			400
#define GIMBAL_LOST_TOLERANCE_MS			200
#define REMOTE_LOST_TOLERANCE_MS			200
#define VISION_LOST_TOLERANCE_MS			200
#define JUDGEMENT_LOST_TOLERANCE_MS		    200
#define FEEDMOTOR_LOST_TOLERANCE_MS		    200
#define COMMUNICATE_LOST_TOLERANCE_MS       50
#define SUPERC_LOST_TOLERANCE_MS            200
#define SUPERC_OUTPUTLOST_TOLERANCE_MS      200

extern int32_t LostCounterCountNumber[NUMBERS_OF_COUNT];

void LostCounterInit(void);
void LostCounterFeed(uint8_t i);
void LostCounterControl(uint16_t SystemErrorStatus);
void LostCounterCount(void);
uint16_t GetErrorState(void);
uint32_t * GetLostCounterData(void);
void LostCounter_Task(void *argument);

#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_TASK_LOSTCOUNTER_H
