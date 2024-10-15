//
// Created by 45441 on 2023/9/23.
//

#ifndef INFANTRYGIMBALC_DRIVER_REMOTE_H
#define INFANTRYGIMBALC_DRIVER_REMOTE_H
#include "main.h"
#define u8 uint8_t
#define u16 uint16_t
#ifdef __cplusplus
extern "C" {
#endif
//C
/**
  * @brief Remote channel enumeration
  */
typedef enum
{
    RC_CH_VALUE_MIN = 364U,
    RC_CH_VALUE_OFFSET = 1024U,
    RC_CH_VALUE_MAX = 1684U
} RC_Channel_Enum;



/**
  * @brief Control data structure definition
  */
typedef struct
{
    float	ChassisSpeedX;
    float	ChassisSpeedY;
    float PitchIncrement;
    float YawIncrement;

    u8	Friction;
    u8	FeedMotor;
    u8	Magazine;
    u8	Laser;//激光
    u8	ShakeEnable;
    u8	FlagChangeFollow;
    u8	Uphill;//检测爬坡
    u8  Downhill;
    u8  YawLocationInit;//yaw初始化
    u8 	SuperCFlag;//电容
    u8	SpinFlag;//小陀螺
    u8	CancelFollow;//取消跟随
    u8	SentryFlag;//哨兵画图没啥用了
    u8	AutoCFlag;//自动电容
    u8	FlagGimbalPitchLock;//pitch初始化
    u8	ChassisInitFlag;//底盘初始化
    u8	RampFlag;//斜坡
    u8  Axialflag;
    u8  Balance;
    u8 Control;//控制方式
}RemoteDataPortStruct;

//***********************新加入的的的的的的的的的的的的對對對是的****************************************************************??//


/**
  * @brief Remote switch enumeration
  */
typedef enum
{
    SwitchUp=1,
    SwitchDown=2,
    SwitchMid=3,
}SwitchEnum;

typedef enum
{
    False=0,
    True=1,
}MousePressEnum;

/**
  * @brief Remote data structure definition
  */
typedef struct
{
    //摇杆
    float Channel_0;
    float Channel_1;
    float Channel_2;
    float Channel_3;

    SwitchEnum RightSwitch;
    SwitchEnum LeftSwitch;

    float MouseX;
    float MouseY;
    float MouseZ;

    MousePressEnum LeftMousePress;
    MousePressEnum RightMousePress;

    uint8_t KeyW:1;
    uint8_t KeyS:1;
    uint8_t KeyA:1;
    uint8_t KeyD:1;
    uint8_t KeyShift:1;
    uint8_t KeyCtrl:1;
    uint8_t KeyQ:1;
    uint8_t KeyE:1;
    uint8_t KeyR:1;
    uint8_t KeyF:1;
    uint8_t KeyG:1;
    uint8_t KeyZ:1;
    uint8_t KeyX:1;
    uint8_t KeyC:1;
    uint8_t KeyV:1;
    uint8_t KeyB:1;

    uint16_t Roller;
    uint8_t FlagValidity;
}RemoteDataProcessedStruct;

#pragma pack(1)
typedef union
{
    struct{

        struct{
            uint16_t Ch0:11;
            uint16_t Ch1:11;
            //此处占用22个字节，接下来补齐
            uint16_t Ch2:11;
            uint16_t Ch3:11;
            uint8_t	s1:2;
            uint8_t	s2:2;
        }RCValue;

        struct{
            int16_t x;
            int16_t y;
            int16_t z;
            uint8_t Press_l;
            uint8_t Press_r;
        }Mouse;

        struct
        {
            uint8_t KeyW:1;
            uint8_t KeyS:1;
            uint8_t KeyA:1;
            uint8_t KeyD:1;
            uint8_t KeyShift:1;
            uint8_t KeyCtrl:1;
            uint8_t KeyQ:1;
            uint8_t KeyE:1;
            uint8_t KeyR:1;
            uint8_t KeyF:1;
            uint8_t KeyG:1;
            uint8_t KeyZ:1;
            uint8_t KeyX:1;
            uint8_t KeyC:1;
            uint8_t KeyV:1;
            uint8_t KeyB:1;
        }Key;
        uint16_t resv;
    }RemoteDataProcessed;
    uint8_t RemoteDataRaw[18];
}RemoteDataUnion;
#pragma pack()

typedef struct{
    u8	CancelFollow;
    u8	Magazine;
    u8  YawLocationInit;
    u8	ChassisInitFlag;
    u8	RampFlag;
}RemoteControlFlagSetPortStruct;//一些附加功能标志位







/* Exported constants --------------------------------------------------------*/



#define RC_FRAME_LENGTH (18u)  /* 遥控器接收字节长度 */
#define MOUSEERRORTOLERANCE (0.00003f)

#define YAW_REMOTE_SENSITIVITY (0.001f)  /* yaw轴遥控器灵敏度 */
#define PITCH_REMOTE_SENSITIVITY (0.0004f)  /* pitch轴遥控器灵敏度 */
/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
extern uint8_t remote_Rx_Buffer[RC_FRAME_LENGTH];
extern RemoteDataPortStruct RemoteDataPort;
extern float test_current;
extern uint8_t AutoSmallBuffShot;
extern uint8_t AutoBigBuffShot;
extern uint8_t Game;
extern uint8_t autoShot;
extern  uint8_t shoot_continue;
extern uint8_t remoteFlag ;
//extern RemoteDataPortStruct RemoteData;

/* Exported functions --------------------------------------------------------*/
int RC_Update(void);
RemoteDataProcessedStruct RemoteDataProcess(RemoteDataUnion RemoteDataRaw);
void RockerDataConvert(float *x,float *y);

void Remote_Task(void *argument);
#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_DRIVER_REMOTE_H
