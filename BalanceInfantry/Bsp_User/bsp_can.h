//
// Created by 45441 on 2023/9/23.
//

#ifndef INFANTRYGIMBALC_BSP_CAN_H
#define INFANTRYGIMBALC_BSP_CAN_H
#include "main.h"
#include "can.h"
#ifdef __cplusplus
extern "C" {
#endif
//C
/*定义关节电机的控制模式*/
#define CMD_MOTOR_MODE      0x05
#define CMD_RESET_MODE      0x06
#define CMD_ZERO_POSITION   0x07

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#define CAN1_FILTER_ID_FMI0	(0x205)//pitch
#define CAN1_FILTER_ID_FMI1	(0x206)//feedmotor
#define CAN1_FILTER_ID_FMI2	(0x207)//friction
#define CAN1_FILTER_ID_FMI3	(0x208)//friction

typedef struct
{

    //自瞄识别到的目标位置，默认向右x，向下y
    int16_t target_location_x;
    int16_t target_location_y;
    uint16_t target_location_z;
}GimbalReceiveStruct;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern uint8_t CAN2RecevieChassisMessageBuffer1[8];
extern float setSpeedX,setSpeedY;
extern uint8_t axialFlag;
extern uint8_t spinFlag;
extern uint8_t jumpFlag;
extern GimbalReceiveStruct GimbalReceive;

void CanInit(void);
unsigned char UserCan1FilterConfig(void);
unsigned char UserCan2FilterConfig(void);



void LL_CAN_Receive(CAN_HandleTypeDef* hcan);

void Can1_Send_Msg(uint8_t *buf,uint8_t len,uint32_t motor_id);

void Can2_Send_Msg(uint8_t *buf,uint8_t len,uint32_t motor_id);
void CanComm_ControlCmd(uint8_t cmd,uint32_t motor_id,CAN_HandleTypeDef *hcan);
void CanComm_SendControlPara(CAN_HandleTypeDef *hcan,float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t motor_id);
#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_BSP_CAN_H
