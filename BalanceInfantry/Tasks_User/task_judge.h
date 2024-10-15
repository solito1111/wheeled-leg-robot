//
// Created by 45441 on 2023/11/6.
//

#ifndef INFANTRYGIMBALC_TASK_JUDGE_H
#define INFANTRYGIMBALC_TASK_JUDGE_H
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "cmsis_os2.h"
#include "data_fifo.h"
#include "driver_remote.h"
#include "bsp_usart.h"
#include "protocol.h"
#include "driver_judge.h"
#ifdef __cplusplus
extern "C" {
#endif
//C


#define UART_TX_SIGNAL      ( 1 << 2 )
#define UART_IDLE_SIGNAL    ( 1 << 1 )

extern osEventFlagsId_t refereeEventHandle;
//————————————————————————————雷达裁判系统测试部分————————————————————————————————————————————//
typedef struct {
    uint16_t target_robot_id;
    float    target_position_x;
    float    target_position_y;
    float    target_state_point;
} lidar_send_struct;

/*********************************************************************************
 * 					协议部分——公用协议
 *********************************************************************************/

#define DN_REG_ID    0xA5  //裁判系统通信
#define HEADER_LEN   5     //帧头长度为5个字节
#define CMD_LEN      2     //命令帧长度2个字节
#define CRC_LEN      2     //CRC16校验长度2个字节
#define INTERACTIVE_HEADER_LEN 6 //交互帧头

#define PROTOCOL_DATA_MAX_SIZE  128

void Judge_Unpack_Task(void *argument);
void StuInteractive_Task(void *argument);
#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_TASK_JUDGE_H
