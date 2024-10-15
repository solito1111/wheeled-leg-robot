//
// Created by 45441 on 2023/9/30.
//

#include "driver_communicate.h"
#include "bsp_can.h"
#include "driver_chassis.h"
#include "bsp_dwt.h"
#include "driver_judge.h"
extern osMessageQueueId_t can1QueueHandle;
extern osMessageQueueId_t can2QueueHandle;
float kd1=0.5;
float kd2=0.15f;
float abc;
float Can1Fre,Can2Fre;
uint32_t Can1Cnt,Can2Cnt;

void Can1CalculateSend(uint8_t remote){
    if(jumping){
        kd1 = 0;
    }else{
        kd1 = 0.5f;
    }
//    LIMIT_MIN_MAX(joint_motors_array[0].torque_ref,-3,3);
//    LIMIT_MIN_MAX(joint_motors_array[1].torque_ref,-3,3);
//    LIMIT_MIN_MAX(joint_motors_array[2].torque_ref,-3,3);
//    LIMIT_MIN_MAX(joint_motors_array[3].torque_ref,-3,3);

    if(remote&&(!motor_error)){
        CanComm_SendControlPara(&hcan1,0,0,0
                ,kd1,joint_motors_array[0].torque_ref,joint_motors_array[0].motor_id);
        CanComm_SendControlPara(&hcan1,0,0,0
                ,kd1,joint_motors_array[1].torque_ref,joint_motors_array[1].motor_id);
        osDelay(1);
        CanComm_SendControlPara(&hcan1,0,0,0
                ,kd1,joint_motors_array[2].torque_ref,joint_motors_array[2].motor_id);
        CanComm_SendControlPara(&hcan1,0,0,0
                ,kd1,joint_motors_array[3].torque_ref,joint_motors_array[3].motor_id);
        osDelay(1);

        Can1Fre = 1.f/DWT_GetDeltaT(&Can1Cnt);
    }else{
        CanComm_SendControlPara(&hcan1,0,0,0,0,0,joint_motors_array[2].motor_id);
        CanComm_SendControlPara(&hcan1,0,0,0,0,0,joint_motors_array[3].motor_id);
        osDelay(1);
        CanComm_SendControlPara(&hcan1,0,0,0,0,0,joint_motors_array[1].motor_id);
        CanComm_SendControlPara(&hcan1,0,0,0,0,0,joint_motors_array[0].motor_id);
        osDelay(1);


    }
}
float kd2_last=0;
float asd=0;
void Can2CalculateSend(uint8_t remote){
    if(jumping){
        kd2 = 0.75f;
    }else{
        kd2 = 0.25f;
    }
    if(remote&&(!motor_error)){
        CanComm_SendControlPara(&hcan1,0,0,0
                ,kd2,wheel_motors_array[1].torque_ref,wheel_motors_array[1].motor_id);
        osDelay(1);
        CanComm_SendControlPara(&hcan1,0,0,0
                ,kd2,wheel_motors_array[0].torque_ref,wheel_motors_array[0].motor_id);
        Can2Fre = 1.f/DWT_GetDeltaT(&Can2Cnt);

    }else{
        CanComm_SendControlPara(&hcan1,0,0,0,0,0,wheel_motors_array[1].motor_id);
        osDelay(1);
        CanComm_SendControlPara(&hcan1,0,0,0,0,0,wheel_motors_array[0].motor_id);
    }
}
void SC_send_message(Super_Cap_t *super_cap)
{
    uint8_t send_buf[8];

    // 电容总开关 1开 0关
    send_buf[0] = super_cap->scSet.enable;
    super_cap->scSet.set_power_charge = capPower;
    // 限幅
    super_cap->scSet.set_power_charge = super_cap->scSet.set_power_charge > POWER_CHARGE_MAX ? POWER_CHARGE_MAX : super_cap->scSet.set_power_charge;
    super_cap->scSet.set_power_charge = super_cap->scSet.set_power_charge < -POWER_DISCHARGE_MAX ? -POWER_DISCHARGE_MAX : super_cap->scSet.set_power_charge;
    // 电容功率单位换算 W -> mW -> 0.1mW
    int32_t power_charge = super_cap->scSet.set_power_charge * 1000 * 10;   // 0.1 mW/LSB

    send_buf[1] = (uint8_t)(power_charge & 0xff);
    send_buf[2] = (uint8_t)((power_charge >> 8) & 0xff);
    send_buf[3] = (uint8_t)((power_charge >> 16) & 0xff);
    send_buf[4] = (uint8_t)((power_charge >> 24) & 0xff);
    send_buf[5] = 0;//(Status.Use_AutoC & (1 << 0));

    Can2_Send_Msg(send_buf, 8, 0x312);
}
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    float offset = x_min;

    return (uint16_t) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
}
extern judgement_protection_struct judgement_protection;

void SendtoGimbalMessage(){
    u8 CAN2SendGimbalMessegeBuffer1[8];
    u8 CAN2SendGimbalMessegeBuffer2[8];
    //当裁判系统数据未丢失时发送裁判系统数据，当裁判系统丢失时发送保存数据
    if(judgement_protection.judgement_lost_flag == 0)
    {
        CAN2SendGimbalMessegeBuffer1[0] = ((int16_t)(judge_rece_mesg.robot_status_data.shooter_barrel_heat_limit)
                >> 8);//枪口热量上限
        CAN2SendGimbalMessegeBuffer1[1] = (
                (int16_t)(judge_rece_mesg.robot_status_data.shooter_barrel_heat_limit) & 0x00ff);
        CAN2SendGimbalMessegeBuffer1[2] = ((int16_t)(judge_rece_mesg.robot_status_data.shooter_barrel_cooling_value)
                >> 8);//枪口冷却值
        CAN2SendGimbalMessegeBuffer1[3] = ((int16_t)(judge_rece_mesg.robot_status_data.shooter_barrel_cooling_value) &
                                           0x00ff);
        CAN2SendGimbalMessegeBuffer1[4] = ((int16_t)(30)
                >> 8);//枪口射速上限
        CAN2SendGimbalMessegeBuffer1[5] = ((int16_t)(30) &
                                           0x00ff);
        CAN2SendGimbalMessegeBuffer1[6] = (int8_t)(judge_rece_mesg.robot_status_data.robot_id);//机器人id
    }else
    {
        CAN2SendGimbalMessegeBuffer1[0] = ((int16_t)(judgement_protection.shooter_barrel_heat_limit)
                >> 8);//枪口热量上限
        CAN2SendGimbalMessegeBuffer1[1] = (
                (int16_t)(judgement_protection.shooter_barrel_heat_limit) & 0x00ff);
        CAN2SendGimbalMessegeBuffer1[2] = ((int16_t)(judgement_protection.shooter_barrel_cooling_value)
                >> 8);//枪口冷却值
        CAN2SendGimbalMessegeBuffer1[3] = ((int16_t)(judgement_protection.shooter_barrel_cooling_value) &
                                           0x00ff);
        CAN2SendGimbalMessegeBuffer1[4] = ((int16_t)(30)
                >> 8);//枪口射速上限
        CAN2SendGimbalMessegeBuffer1[5] = ((int16_t)(30) &
                                           0x00ff);
        CAN2SendGimbalMessegeBuffer1[6] = (int8_t)(judgement_protection.robot_id);//机器人id
    }

    if(LostCounterCountNumber[JUDGEMENT_LOST_COUNT]>(JUDGEMENT_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
        CAN2SendGimbalMessegeBuffer1[7] = (CAN2SendGimbalMessegeBuffer1[7] & (~(1 << 0)));
    else
        CAN2SendGimbalMessegeBuffer1[7] = (CAN2SendGimbalMessegeBuffer1[7] | (1 << 0));
    if(judge_rece_mesg.robot_status_data.mains_power_shooter_output)
        CAN2SendGimbalMessegeBuffer1[7] = (CAN2SendGimbalMessegeBuffer1[7] | (1 << 1));
    else
        CAN2SendGimbalMessegeBuffer1[7] = (CAN2SendGimbalMessegeBuffer1[7] & (~(1 << 1)));
    // TODO: 裁判系统断联后无法估计云台热量数据，如何提醒云台？UI中报警？
    CAN2SendGimbalMessegeBuffer2[0] =
            ((int16_t)(judge_rece_mesg.power_heat_data.shooter_17mm_1_barrel_heat)) >> 8;//枪口热量1
    CAN2SendGimbalMessegeBuffer2[1] =
            ((int16_t)(judge_rece_mesg.power_heat_data.shooter_17mm_1_barrel_heat)) & 0x00ff;
    CAN2SendGimbalMessegeBuffer2[4] =
            ((int16_t)(judge_rece_mesg.power_heat_data.shooter_17mm_2_barrel_heat)) >> 8;//枪口热量2
    CAN2SendGimbalMessegeBuffer2[5] =
            ((int16_t)(judge_rece_mesg.power_heat_data.shooter_17mm_2_barrel_heat)) & 0x00ff;

    CAN2SendGimbalMessegeBuffer2[2] = ((int16_t)(judge_rece_mesg.shoot_data.bullet_speed * 100)) >> 8;//子弹速度
    CAN2SendGimbalMessegeBuffer2[3] = ((int16_t)(judge_rece_mesg.shoot_data.bullet_speed * 100)) & 0x00ff;
    CAN2SendGimbalMessegeBuffer2[6] = ((int16_t)(judge_rece_mesg.shoot_data.bullet_speed * 100)) >> 8;//子弹速度
    CAN2SendGimbalMessegeBuffer2[7] = ((int16_t)(judge_rece_mesg.shoot_data.bullet_speed * 100)) & 0x00ff;

    Can2_Send_Msg(CAN2SendGimbalMessegeBuffer1, 8, 0x221);
    Can2_Send_Msg(CAN2SendGimbalMessegeBuffer2,8,0x222);

}

