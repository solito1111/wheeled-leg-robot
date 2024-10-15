//
// Created by 45441 on 2023/9/23.
//

#include "bsp_usart.h"
#include "cmsis_os2.h"
#include "driver_judge.h"
extern osSemaphoreId_t RemoteSemHandle;
#define JUDGE_FIFO_BUFLEN 1250
//该参数应大于等于UART_RX_DMA_SIZE，避免遗漏数据

extern uint8_t   judge_rxdata_buf[JUDGE_FIFO_BUFLEN];
extern usart_param_struct judgement_usart;
extern osSemaphoreId_t RemoteSemHandle;

/**
  * @brief  Enable USART3
  * @param  DMA_Memory0BaseAddr : Usart1DMAMemoryBaseAddress
  * @param  DMA_BufferSize : Amount of data elements (uint8_t or uint16_t) to be received.
  * @retval None
  */
void USART3ConfigEnable(uint8_t *DMA_Memory0BaseAddr, uint8_t DMA_BufferSize)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*)DMA_Memory0BaseAddr, DMA_BufferSize);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}
uint8_t Uart1_Receive_buf[1];          //串口5接收中断数据存放的缓冲区

char USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
uint16_t point1 ;
LidarPointTypedef Pack_Data[12];/* 雷达接收的数据储存在这个变量之中 */
LidarPointTypedef Pack_sum;     /* 输出结果储存 */
uint16_t receive_cnt;
uint8_t confidence;
uint16_t distance,noise,reftof;
uint32_t peak,intg;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    static uint8_t state = 0;            //状态位
    static uint8_t crc = 0;                //校验和
    static uint8_t cnt = 0;                //用于一帧12个点的计数
    static uint8_t PACK_FLAG = 0;  //命令标志位
    static uint8_t data_len = 0;  //数据长度
    static uint32_t timestamp = 0; //时间戳
    static uint8_t state_flag = 1; //转入数据接收标志位
    uint8_t temp_data;
    if (huart->Instance == USART1) {
        temp_data = Uart1_Receive_buf[0];
        if (state <
            4)                                                                                     /* 起始符验证 前4个数据均为0xAA */
        {
            if (temp_data == HEADER) state++;
            else state = 0;
        } else if (state < 10 && state > 3) {
            switch (state) {
                case 4:
                    if (temp_data == device_address)              /* 设备地址验证 */
                    {
                        state++;
                        crc = crc + temp_data;
                        break;
                    } else state = 0, crc = 0;
                case 5:
                    if (temp_data == PACK_GET_DISTANCE)                     /* 获取测量数据命令 */
                    {
                        PACK_FLAG = PACK_GET_DISTANCE;
                        state++;
                        crc = crc + temp_data;
                        break;
                    } else if (temp_data == PACK_RESET_SYSTEM)         /* 复位命令 */
                    {
                        PACK_FLAG = PACK_RESET_SYSTEM;
                        state++;
                        crc = crc + temp_data;
                        break;
                    } else if (temp_data == PACK_STOP)                             /* 停止测量数据传输命令 */
                    {
                        PACK_FLAG = PACK_STOP;
                        state++;
                        crc = crc + temp_data;
                        break;
                    } else if (temp_data == PACK_ACK)                             /* 应答码命令 */
                    {
                        PACK_FLAG = PACK_ACK;
                        state++;
                        crc = crc + temp_data;
                        break;
                    } else if (temp_data == PACK_VERSION)                     /* 获取传感器信息命令 */
                    {
                        PACK_FLAG = PACK_VERSION,
                                state++,
                        crc = crc + temp_data;
                        break;
                    } else state = 0, crc = 0;
                case 6:
                    if (temp_data == chunk_offset)          /* 偏移地址 */
                    {
                        state++;
                        crc = crc + temp_data;
                        break;
                    } else state = 0, crc = 0;
                case 7:
                    if (temp_data == chunk_offset) {
                        state++;
                        crc = crc + temp_data;
                        break;
                    } else state = 0, crc = 0;
                case 8:
                    data_len = (u16) temp_data;                                 /* 数据长度低八位 */
                    state++;
                    crc = crc + temp_data;
                    break;
                case 9:
                    data_len = data_len + ((u16) temp_data << 8);             /* 数据长度高八位 */
                    state++;
                    crc = crc + temp_data;
                    break;
                default:
                    break;
            }
        }
        else if (state == 10)
            state_flag = 0;                    /*由switch跳出来时state为10，但temp_data仍为距离长度高八位数据，需跳过一次中断*/
        if (PACK_FLAG == PACK_GET_DISTANCE && state_flag == 0)      /* 获取一帧数据并校验 */
        {
            if (state > 9) {
                if (state < 190) {
                    static uint8_t state_num;
                    state_num = (state - 10) % 15;
                    switch (state_num) {
                        case 0:
                            Pack_Data[cnt].distance = (uint16_t) temp_data;                 /* 距离数据低八位 */
                            crc = crc + temp_data;
                            state++;
                            break;
                        case 1:
                            Pack_Data[cnt].distance = ((u16) temp_data << 8) + Pack_Data[cnt].distance;     /* 距离数据 */
                            crc = crc + temp_data;
                            state++;
                            break;
                        case 2:
                            Pack_Data[cnt].noise = (u16) temp_data;                 /* 环境噪音低八位 */
                            crc = crc + temp_data;
                            state++;
                            break;
                        case 3:
                            Pack_Data[cnt].noise =
                                    ((u16) temp_data << 8) + Pack_Data[cnt].noise;                 /* 环境噪音 */
                            crc = crc + temp_data;
                            state++;
                            break;
                        case 4:
                            Pack_Data[cnt].peak = (u32) temp_data;                                                         /* 接受强度信息低八位 */
                            crc = crc + temp_data;
                            state++;
                            break;
                        case 5:
                            Pack_Data[cnt].peak = ((u32) temp_data << 8) + Pack_Data[cnt].peak;
                            crc = crc + temp_data;
                            state++;
                            break;
                        case 6:
                            Pack_Data[cnt].peak = ((u32) temp_data << 16) + Pack_Data[cnt].peak;
                            crc = crc + temp_data;
                            state++;
                            break;
                        case 7:
                            Pack_Data[cnt].peak =
                                    ((u32) temp_data << 24) + Pack_Data[cnt].peak;                    /* 接受强度信息 */
                            crc = crc + temp_data;
                            state++;
                            break;
                        case 8:
                            Pack_Data[cnt].confidence = temp_data;                 /* 置信度 */
                            crc = crc + temp_data;
                            state++;
                            break;
                        case 9:
                            Pack_Data[cnt].intg = (u32) temp_data;                                                            /* 积分次数低八位 */
                            crc = crc + temp_data;
                            state++;
                            break;
                        case 10:
                            Pack_Data[cnt].intg = ((u32) temp_data << 8) + Pack_Data[cnt].intg;
                            crc = crc + temp_data;
                            state++;
                            break;
                        case 11:
                            Pack_Data[cnt].intg = ((u32) temp_data << 16) + Pack_Data[cnt].intg;
                            crc = crc + temp_data;
                            state++;
                            break;
                        case 12:
                            Pack_Data[cnt].intg =
                                    ((u32) temp_data << 24) + Pack_Data[cnt].intg;                     /* 积分次数 */
                            crc = crc + temp_data;
                            state++;
                            break;
                        case 13:
                            Pack_Data[cnt].reftof = (int16_t) temp_data;                                                 /* 温度表征值低八位 */
                            crc = crc + temp_data;
                            state++;
                            break;
                        case 14:
                            Pack_Data[cnt].reftof =
                                    ((int16_t) temp_data << 8) + Pack_Data[cnt].reftof;            /* 温度表征值 */
                            crc = crc + temp_data;
                            state++;
                            cnt++;                             /* 进入下一个测量点 */
                            break;
                        default:
                            break;
                    }
                }
                /* 时间戳 */
                if (state == 190)
                    timestamp = temp_data, state++, crc = crc + temp_data;
                else if (state == 191)
                    timestamp = ((u32) temp_data << 8) + timestamp, state++, crc = crc + temp_data;
                else if (state == 192) timestamp = ((u32) temp_data << 16) + timestamp, state++, crc = crc + temp_data;
                else if (state == 193) timestamp = ((u32) temp_data << 24) + timestamp, state++, crc = crc + temp_data;
                else if (state == 194) {
                    if (temp_data == crc)   /* 校验成功 */
                    {
                        data_process();     /* 数据处理函数，完成一帧之后可进行数据处理 */
                        receive_cnt++;         /* 输出接收到正确数据的次数 */
                    }
                    tof_distance = Pack_Data[0].distance;
                    crc = 0;
                    state = 0;
                    state_flag = 1;
                    cnt = 0;                             /* 复位*/
                }

            }
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Uart1_Receive_buf,sizeof(Uart1_Receive_buf));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
    if (huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*)remote_Rx_Buffer, RC_FRAME_LENGTH);
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
        osSemaphoreRelease(RemoteSemHandle);
    }
    if(huart->Instance == USART6)
    {
        //暂停DMA数据传输
        //HAL_UART_DMAStop(huart);

        //将DMA　buffer中数据传给FIFO
        /*
         * 理想情况下，每帧数据均从DMA数组首位开始存放（即第一位为A5），实际上由于各个包传输频率不同，
         * 串口空闲中断不明显，可能存在数据接收出错，导致包头A5前仍有其他杂数据
         *
         * */
        usart_rx_processed( &judgement_usart, Size);

        /*
         * 开启DMA数据传输
         * judgement_usart.buff为传输地址，当空闲中断发生或DMA传输数据达到buff_size时进入中断
         * */
        HAL_UARTEx_ReceiveToIdle_DMA(judgement_usart.huart,judgement_usart.buff,judgement_usart.buff_size);

        //为避免接收数据错误，不在此处喂狗，而且数据校验解算后喂狗
        return;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    //if(huart == judgement_usart.huart)
    judgement_usart.tx_finish_flag = 1;
}

void usart_dma_send(usart_param_struct *usart_param, const uint8_t *pData, uint32_t _size)
{

    if(usart_param->tx_finish_flag){
        HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(usart_param->huart, pData, _size);
        if(status==HAL_OK)
        {
            usart_param->tx_finish_flag = 0;
        }
        else
        {
            usart_param->tx_finish_flag = 0;
        }
        __HAL_DMA_DISABLE_IT(usart_param->huart->hdmatx, DMA_IT_HT);
    }
}

void usart_communicate_config(usart_param_struct *usart_param)
{
    //再次开启DMA数据传输
    HAL_UARTEx_ReceiveToIdle_DMA(usart_param->huart,
                                 usart_param->buff,
                                 usart_param->buff_size);
    __HAL_DMA_DISABLE_IT(usart_param->huart->hdmarx, DMA_IT_TC);
    __HAL_DMA_DISABLE_IT(usart_param->huart->hdmarx, DMA_IT_HT);
}

uint16_t tof_distance;

void data_process(void)/*数据处理函数，完成一帧之后可进行数据处理*/
{
    /* 计算距离 */
    static u8 cnt = 0;
    u8 i;
    static u16 count = 0;
    static u32 sum = 0;
    LidarPointTypedef Pack_sum;
    for(i=0;i<12;i++)									/* 12个点取平均 */
    {
        if(Pack_Data[i].distance != 0)  /* 去除0的点 */
        {
            count++;
            Pack_sum.distance += Pack_Data[i].distance;
            Pack_sum.noise += Pack_Data[i].noise;
            Pack_sum.peak += Pack_Data[i].peak;
            Pack_sum.confidence += Pack_Data[i].confidence;
            Pack_sum.intg += Pack_Data[i].intg;
            Pack_sum.reftof += Pack_Data[i].reftof;
        }
    }
    if(count !=0)
    {

        tof_distance = Pack_sum.distance/count;
        noise = Pack_sum.noise/count;
        peak = Pack_sum.peak/count;
        confidence = Pack_sum.confidence/count;
        intg = Pack_sum.intg/count;
        reftof = Pack_sum.reftof/count;
        Pack_sum.distance = 0;
        Pack_sum.noise = 0;
        Pack_sum.peak = 0;
        Pack_sum.confidence = 0;
        Pack_sum.intg = 0;
        Pack_sum.reftof = 0;
        count = 0;
    }
}