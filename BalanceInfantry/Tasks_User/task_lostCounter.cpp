
// Created by 45441 on 2024/3/4.
//

#include "task_lostCounter.h"
#include "driver_chassis.h"

int32_t LostCounterCountNumber[NUMBERS_OF_COUNT];

void LostCounterInit(void)											// LostCounterCountNumber[i]初始化值为1000
{																						// 在main.c初始化
    uint8_t i;
    for(i = 0; i < NUMBERS_OF_COUNT; i++){
        LostCounterCountNumber[i]=1000;
    }
}

void LostCounterFeed(uint8_t i)
{
    LostCounterCountNumber[i] = 0;
}
void LostCounterControl(uint16_t SystemErrorStatus)
{
    if((SystemErrorStatus >> REMOTE_LOST_COUNT) & (1 << REMOTE_LOST_COUNT))
    {
        //GimbalReceive.LostCounter = 1;//关控保护
    }
}

/********************************************************************************************
	SystemErrorStatus:
  				|	    		|   	      	|   	      	|   	   		|				|				|   备注
 				|		1		|   	12  	|   	11  	|   	10		|		9		|		8		|   优先级
		15		|		14		|   	13  	|   	12  	|   	11		|		10		|		9		|   ID
				|	        	|	  Vision 	|	  SuperC	|      Yaw 		|	  Pitch 	|	FRICTION	|   LostCounter
				|	云台底盘通信	|	   图传		|	   电容		|	 Yaw电机		|	Pitch电机	| 	  摩擦轮	    |   失联位置
---------------------------------------------------------------------------------------------
					两个及以上失联优先级为7（底盘）					|	    		| 	    		|	    		|	        	|	   	    	|   备注
   		6		|		6		|		6		|   	6		|   	5  		|   	4   	|   	3   	|		2   	|   	1  		|   优先级
  		8		|		7		|		6		|   	5		|   	4  		|   	3   	|   	2   	|		1   	|   	0  		|   ID
	  MOTOR-3	|	 MOTOR-2	| 	MOTOR-1 	|	  MOTOR-0	|	FEED_MOTOR	|	   HEAT 	|	  BSPEED	|  	JUDGEMENT  	|	  REMOTE	|   LostCOunter
							底盘电机							    |	  拨弹		| 	   热量		|	   射速		|	 裁判系统	|	   遥控		|   失联位置
********************************************************************************************/

void LostCounterCount(void)
{
    uint8_t i;

    for(i = 0; i < NUMBERS_OF_COUNT; i++){		// NUMBERS_OF_COUNT 10
        LostCounterCountNumber[i]++;
    }
}

uint16_t GetErrorState(void)
{
    uint16_t SystemErrorStatus = 0;

    if(LostCounterCountNumber[JOINT_MOTOR_0]>(CHASSIS_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << JOINT_MOTOR_0;

    if(LostCounterCountNumber[JOINT_MOTOR_1]>(CHASSIS_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << JOINT_MOTOR_1;

    if(LostCounterCountNumber[JOINT_MOTOR_2]>(CHASSIS_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << JOINT_MOTOR_2;

    if(LostCounterCountNumber[JOINT_MOTOR_3]>(CHASSIS_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << JOINT_MOTOR_3;

    if(LostCounterCountNumber[GIMBAL_MOTOR_PITCH]>(GIMBAL_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << GIMBAL_MOTOR_PITCH;

    if(LostCounterCountNumber[GIMBAL_MOTOR_YAW]>(GIMBAL_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << GIMBAL_MOTOR_YAW;

    if(LostCounterCountNumber[FEEDMOTOR_LOST_COUNT]>(FEEDMOTOR_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << FEEDMOTOR_LOST_COUNT;

    if(LostCounterCountNumber[REMOTE_LOST_COUNT]>(REMOTE_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << REMOTE_LOST_COUNT;

    if(LostCounterCountNumber[VISION_LOST_COUNT]>(VISION_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << VISION_LOST_COUNT;

    if(LostCounterCountNumber[JUDGEMENT_LOST_COUNT]>(JUDGEMENT_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << JUDGEMENT_LOST_COUNT;

    if(LostCounterCountNumber[COMMUNICATE_LOST_COUNT]>(COMMUNICATE_LOST_TOLERANCE_MS/(LOST_COUNTER_TIME_MS)))
        SystemErrorStatus |= 1 << COMMUNICATE_LOST_COUNT;

    if(LostCounterCountNumber[OVER_SPEED_LOST]>(JUDGEMENT_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << OVER_SPEED_LOST;

    if(LostCounterCountNumber[OVER_HEAT_LOST]>(JUDGEMENT_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << OVER_HEAT_LOST;

    if(LostCounterCountNumber[FRICTION_LOST_COUNT]>(JUDGEMENT_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << FRICTION_LOST_COUNT;

    if(LostCounterCountNumber[SUPERC_LOST_COUNT]>(SUPERC_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << SUPERC_LOST_COUNT;

    if(LostCounterCountNumber[SUPERC_OUTPUTLOST_COUNT] > SUPERC_OUTPUTLOST_TOLERANCE_MS / LOST_COUNTER_TIME_MS)
        SystemErrorStatus |= 1 << SUPERC_OUTPUTLOST_COUNT;

    if(LostCounterCountNumber[WHEEL_MOTOR_0]>(CHASSIS_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << WHEEL_MOTOR_0;

    if(LostCounterCountNumber[WHEEL_MOTOR_1]>(CHASSIS_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        SystemErrorStatus |= 1 << WHEEL_MOTOR_1;

    LostCounterControl(SystemErrorStatus);
    return SystemErrorStatus;
}

void LostCounter_Task(void *argument)
{
    LostCounterInit();
    for(;;)
    {
        LostCounterControl(GetErrorState());
        LostCounterCount();
        if(!controlSignal.controlFlag.chassisInitFlag){
            FeedIndependentWatchDog();
        }
        osDelay(50);
    }
}

//uint32_t * GetLostCounterData(void)
//{
//    return LostCounterCountNumber;
//}
