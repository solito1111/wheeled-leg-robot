//
// Created by 45441 on 2023/11/6.
//

#include "task_judge.h"

/* 接收到的数据包 */
extern unpack_data_t     judge_unpack_obj;
/* 裁判系统数据保护结构体 */
extern judgement_protection_struct judgement_protection;
/**
  * @brief  解包任务.
  * @param  无
  * @retval 无
  */
void Judge_Unpack_Task(void *argument)
{
    uint32_t event;
    /* open judge uart receive it */
    judgement_uart_init();
    while (1) {
        //当接收到数据时事件set,开启数据解包处理
        event = osEventFlagsWait(refereeEventHandle,
                                 UART_IDLE_SIGNAL,
                                 osFlagsWaitAny,
                                 500);
        //当超时时（0.5s）尝试重启整个串口
        if(event == osErrorTimeout)
        {
            judgement_protection.judgement_lost_flag = 1; //裁判系统丢失
            HAL_UART_MspDeInit(&huart6);
            HAL_UART_MspInit(&huart6);
            judgement_uart_init();
        }
        //git test
            //串口空闲中断完成，FIFO正常收到数据，开始解包
        else if(event & UART_IDLE_SIGNAL) {
            unpack_fifo_data(&judge_unpack_obj, DN_REG_ID);
        }
            //对除超时外的其他错误情况进行保护，重启串口
        else{
            judgement_protection.judgement_lost_flag = 1; //裁判系统丢失
            HAL_UART_MspDeInit(&huart6);
            HAL_UART_MspInit(&huart6);
            judgement_uart_init();
        }
        //send data to judge system
    }
}



/**
  * @brief  选手端与机器人交互
  * @param  无
  * @retval 无
  */
void StuInteractive_Task(void *argument)
{
    /* USER CODE BEGIN StulnteractiveTask */
    /* Infinite loop */
    uint32_t event;
    for(;;)
    {
        StuInteractiveData();
        event = osEventFlagsWait(refereeEventHandle,
                                 UART_TX_SIGNAL,
                                 osFlagsWaitAny,
                                 osWaitForever);
        if (event & UART_TX_SIGNAL) {
            //开始发送数据
            send_packed_fifo_data(DN_REG_ID);
        }
        //TODO:上限10Hz?
        osDelay(50);
    }
    /* USER CODE END StulnteractiveTask */
}