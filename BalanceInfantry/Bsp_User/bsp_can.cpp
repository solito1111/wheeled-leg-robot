//
// Created by 45441 on 2023/9/23.
//

#include "bsp_can.h"
#include "stm32f4xx_hal_can.h"
#include "cmsis_os2.h"
#include "driver_chassis.h"
#include "driver_remote.h"
#include "driver_power.h"
#include "bsp_usart.h"
/**
  * @brief  Enable CAN
  * @retval None
  */

void CanInit(void)
{
    UserCan1FilterConfig();
    UserCan2FilterConfig();
}


unsigned char UserCan1FilterConfig()
{
    CAN_FilterTypeDef  CAN1_FilerConf;
    CAN1_FilerConf.FilterIdHigh=0x0000;     //32位ID
    CAN1_FilerConf.FilterIdLow=0x0000;
    CAN1_FilerConf.FilterMaskIdHigh=0x0000; //32位MASK
    CAN1_FilerConf.FilterMaskIdLow=0x0000;
    CAN1_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤器0关联到FIFO0
    CAN1_FilerConf.FilterBank=0;          //过滤器0
    CAN1_FilerConf.FilterMode=CAN_FILTERMODE_IDMASK;
    CAN1_FilerConf.FilterScale=CAN_FILTERSCALE_32BIT;
    CAN1_FilerConf.FilterActivation=ENABLE; //激活滤波器0
    CAN1_FilerConf.SlaveStartFilterBank=0;


    if(HAL_CAN_ConfigFilter(&hcan1,&CAN1_FilerConf)!=HAL_OK) return 1;//滤波器初始化

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    //__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//FIFO0消息挂起中断允许.

    return 0;
}

unsigned char UserCan2FilterConfig()
{
    CAN_FilterTypeDef  CAN2_FilerConf;
    CAN2_FilerConf.FilterIdHigh=0x0000;     //32位ID
    CAN2_FilerConf.FilterIdLow=0x0000;
    CAN2_FilerConf.FilterMaskIdHigh=0x0000; //32位MASK
    CAN2_FilerConf.FilterMaskIdLow=0x0000;
    CAN2_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤器0关联到FIFO0
    CAN2_FilerConf.FilterBank=14;          //过滤器0
    CAN2_FilerConf.FilterMode=CAN_FILTERMODE_IDMASK;
    CAN2_FilerConf.FilterScale=CAN_FILTERSCALE_32BIT;
    CAN2_FilerConf.FilterActivation=ENABLE; //激活滤波器0
    CAN2_FilerConf.SlaveStartFilterBank=14;

    if(HAL_CAN_ConfigFilter(&hcan2,&CAN2_FilerConf)!=HAL_OK) return 1;//滤波器初始化
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    //__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//FIFO0消息挂起中断允许.

    return 0;
}
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    float offset = x_min;

    return (uint16_t) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
}

/**
  * @brief  converts unsigned int to float, given range and number of bits
  * @param
  * @retval
  */
static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

 uint8_t jumpFlagTemp;
 uint8_t jumpFlagTrigger;
uint8_t cntjump;
uint32_t gimbalcnt;
float gimabalFre;
GimbalReceiveStruct GimbalReceive;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    CAN_RxHeaderTypeDef RxMessage;
    uint8_t rx_Data[8];
    if(hcan==&hcan1){
       // __HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
        HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMessage,rx_Data);
        if(RxMessage.StdId == 0){
            static uint8_t j = 0;
            if (rx_Data[0] == 0x01 || rx_Data[0] == 0x02 || rx_Data[0] == 0x03 || rx_Data[0] == 0x04) {
                j = rx_Data[0] - 0x01;
                LostCounterFeed(j+5);
                joint_motors_array[j].detect = joint_motors_array[j].get_motor_measure(rx_Data);
            }else if (rx_Data[0] == 0x05) {
                LostCounterFeed(WHEEL_MOTOR_0);
                wheel_motors_array[0].detect = wheel_motors_array[0].get_motor_measure(rx_Data);
            }else if(rx_Data[0]==0x06){
                LostCounterFeed(WHEEL_MOTOR_1);
                wheel_motors_array[1].detect=wheel_motors_array[1].get_motor_measure(rx_Data);
            }
        }
    }else if(hcan==&hcan2){
        //__HAL_CAN_ENABLE_IT(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
        HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMessage,rx_Data);
         if(RxMessage.StdId ==0x213){
                gimabalFre = 1/ DWT_GetDeltaT(&gimbalcnt);
                static uint8_t spin_cnt = 0;
                float vxInit=(float)((rx_Data[0]<<8)|(rx_Data[1]&0xff));
                float vyInit=(float)((rx_Data[2]<<8)|(rx_Data[3]&0xff));
                controlSignal.controlFlag.superCFlag = (rx_Data[4]>>7)&1;
                controlSignal.controlFlag.spinFlag = (rx_Data[4]>>6)&1;
                if((controlSignal.controlFlag.spinFlag == 0)&&(spin_cnt == 1)){
                    spin_inverse=!spin_inverse;
                    spin_cnt = 0;
                }
                if(controlSignal.controlFlag.spinFlag){
                    spin_cnt =1;
                }
//                controlSignal.controlFlag.rollFlag = rx_Data[4]&1;
                controlSignal.controlFlag.chassisInitFlag = (rx_Data[4]>>5)&1;
                controlSignal.controlFlag.followFlag = !((rx_Data[4]>>4)&1);
                jumpFlagTemp = (rx_Data[4] >> 3) & 1;
                if((jumpFlagTemp == 1) && (jumpFlagTrigger == 0)&&(tof_distance<800)){
                    controlSignal.controlFlag.jumpFlag = 1;//(rx_Data[4] >> 4) & 1;
                    jump_finish = 0;
                    jumpFlagTrigger = 1;
                }else if(jumpFlagTemp == 0){
                    jump_finish = 1;
                    jumpFlagTrigger = 0;
                    controlSignal.controlFlag.jumpFlag = 0;//(rx_Data[4] >> 4) & 1;
                }
                controlSignal.controlFlag.friFlag = (rx_Data[4]>>2)&1;
                controlSignal.controlFlag.balanceFlag = (rx_Data[4]>>1)&1;
                controlSignal.controlFlag.axialFlag = rx_Data[4]&1;
                controlSignal.controlFlag.legLengthFlag = rx_Data[5];
                controlSignal.controlFlag.visionFlag = rx_Data[6];
                int lostnum=0;
                for(int i =0;i<8;i++){
                    if(rx_Data[i]==0){
                        lostnum++;
                    }
                }
                if(lostnum == 8){
                    remoteFlag=0;
                }else {
                    remoteFlag=1;
                }
                if(vxInit>363.f&&vxInit<1685.f){
                    controlSignal.legLength = -(vxInit-1024.f)/660.f/1.f;
                }
                if(vyInit>363.f&&vyInit<1685.f){
                    controlSignal.speed = -(vyInit-1024.f)/660.f/1.f;
                }

             if (!(rx_Data[7] & 0x01))//No OverSpeed
                 LostCounterFeed(OVER_SPEED_LOST);
             if (!((rx_Data[7] & 0x02) >> 1))//No OverHeat
                 LostCounterFeed(OVER_HEAT_LOST);
             if (!((rx_Data[7] & 0x04) >> 2))//FeedMotor
                 LostCounterFeed(FEEDMOTOR_LOST_COUNT);
             if (!((rx_Data[7] & 0x08) >> 3))//Friction
                 LostCounterFeed(FRICTION_LOST_COUNT);
             if (!((rx_Data[7] & 0x10) >> 4))//Pitch
                 LostCounterFeed(GIMBAL_MOTOR_PITCH);
             if (!((rx_Data[7] & 0x20) >> 5))//Vision
                 LostCounterFeed(VISION_LOST_COUNT);
             if(remoteFlag==0){
                 controlSignal.speed = 0;
             }else{
                 LostCounterFeed(REMOTE_LOST_COUNT);
             }
         }else if(RxMessage.StdId ==0x214){
//                float vy2Init = (float)((rx_Data[0]<<8)|(rx_Data[1]&0xff));
//                controlSignal.speed2 = -(vy2Init-1024.f)/660.f/1.f;
//                float yawInit = (float)((rx_Data[2]<<8)|(rx_Data[3]&0xff));
//                controlSignal.yaw = - (yawInit-1024.f)/660.f;
             //目标位置信息
             GimbalReceive.target_location_x= (int16_t)(((rx_Data[0] << 8) | (rx_Data[1] & 0xFF)) / 10);
             GimbalReceive.target_location_y= (int16_t)(((rx_Data[2] << 8) | (rx_Data[3] & 0xFF)) / 10);
             GimbalReceive.target_location_z= (uint16_t)(((rx_Data[4] << 8) | (rx_Data[5] & 0xFF)) / 10);
             LostCounterFeed(COMMUNICATE_LOST_COUNT);
         }else if(RxMessage.StdId ==0x206){
            YawMotor.detect = YawMotor.get_motor_measure(rx_Data);
             LostCounterFeed(GIMBAL_MOTOR_YAW);
         }else if(RxMessage.StdId == 0x311){
             SC_recv_message(rx_Data);
             LostCounterFeed(SUPERC_LOST_COUNT);
             LostCounterFeed(SUPERC_OUTPUTLOST_COUNT);
         }
        }
    }



 void Can1_Send_Msg(uint8_t *buf,uint8_t len,uint32_t motor_id){
    CAN_TxHeaderTypeDef TxHead;
    uint32_t canTxMailBox;

    if((buf != NULL) && (len != 0))
    {
        TxHead.StdId    = motor_id;         /* 指定标准标识符，该值在0x01-0x04 */
        TxHead.IDE      = CAN_ID_STD;       /* 指定将要传输消息的标识符类型 */
        TxHead.RTR      = CAN_RTR_DATA;     /* 指定消息传输帧类型 */
        TxHead.DLC      = len;              /* 指定将要传输的帧长度 */
    }
    if(HAL_CAN_AddTxMessage(&hcan1,&TxHead,buf,(uint32_t*)canTxMailBox) ==HAL_OK);
}

 void Can2_Send_Msg(uint8_t *buf,uint8_t len,uint32_t motor_id){
    CAN_TxHeaderTypeDef TxHead;
    uint32_t canTxMailBox;

    if((buf != NULL) && (len != 0))
    {
        TxHead.StdId    = motor_id;         /* 指定标准标识符，该值在0x01-0x04 */
        TxHead.IDE      = CAN_ID_STD;       /* 指定将要传输消息的标识符类型 */
        TxHead.RTR      = CAN_RTR_DATA;     /* 指定消息传输帧类型 */
        TxHead.DLC      = len;              /* 指定将要传输的帧长度 */
    }
    if(HAL_CAN_AddTxMessage(&hcan2,&TxHead,buf,(uint32_t*)canTxMailBox) ==HAL_OK);
}

/**
  * @brief          发送模式数据去设置关节电机的控制模式
  * @param[in]      cmd：模式指令，电机模式(0x05)、失效模式(0x06)、零位设置模式(0x07)
  * @param[in]      motor_id: 电机的ID，从0x01到0x04
  * @retval         none
  */
void CanComm_ControlCmd(uint8_t cmd,uint32_t motor_id,CAN_HandleTypeDef *hcan)
        {
            uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
            switch(cmd)
            {
                case CMD_MOTOR_MODE:
                    buf[7] = 0xFC;
                    break;

                case CMD_RESET_MODE:
                    buf[7] = 0xFD;
                    break;

                case CMD_ZERO_POSITION:
                    buf[7] = 0xFE;
                    break;

                default:
                    return; /* 直接退出函数 */
            }
            if(hcan == &hcan1){
                Can1_Send_Msg(buf,8, motor_id);
            }else if(hcan == &hcan2){
        Can2_Send_Msg(buf,8, motor_id);

    }
}


/**
  * @brief          发送关节电机控制参数(0x01,0x02,0x03,0x04)
  * @param[in]      f_p: 目标位置，范围 [-95.5,95.5] rad
  * @param[in]      f_v: 目标转速，范围 [-45,45] rad/s
  * @param[in]      f_kp: kp参数， 范围 [0，500] N.m/rad
  * @param[in]      f_kd: kd参数,  范围 [0,5] N.m/rad/s
  * @param[in]      f_t: 目标力矩, 范围 [-18,18] N.m
  * @param[in]      motor_id: 电机的ID，从0x01到0x04
  * @retval         none
  */
extern osMessageQueueId_t can1QueueHandle;
extern osMessageQueueId_t can2QueueHandle;
osStatus_t statuS;
size10_t txBuf1,txBuf2,txBuf3,txBuf4,txBuf;
void CanComm_SendControlPara(CAN_HandleTypeDef *hcan,float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t motor_id)
{
    uint16_t p, v, kp, kd, t;
    uint8_t buf[8];

    /* 限制输入的参数在定义的范围内 */
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);

    /* 根据协议，对float参数进行转换 */
    p = float_to_uint(f_p,   P_MIN,  P_MAX,  16);
    v = float_to_uint(f_v,   V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,   T_MIN,  T_MAX,  12);

    /* 根据传输协议，把数据转换为CAN命令数据字段 */
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;
    /* 通过CAN接口把buf中的内容发送出去 */
    if(hcan == &hcan1){
        Can1_Send_Msg(buf,8,motor_id);

    }else if(hcan == &hcan2){
        Can2_Send_Msg(buf,8,motor_id);
    }
}