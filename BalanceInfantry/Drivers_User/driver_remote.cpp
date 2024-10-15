
/**
  ******************************************************************************
  * @file    driver_remote.c
  * @author  XJTU ROBOMASTER Team
  * @brief   remote driver
  *          这个文件提供函数对遥控器数据进行处理得到拨杆和键鼠控制量，对机器人控制信号进行更新
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
    [..]
      (#) Update the remote data by implementing the RC_Update():
          (++) Update the remote parameters, including:
		       (+++) RC_Ctl

      (#) Update the remote data by implementing the RemoteControl():
          (++) Update the remote parameters, including:
		       (+++) RemoteData

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * 函数 RC_Update() 已经封装好，遥杆和键鼠的数据接收均为正确的不需要改
  * 可以根据需要更改结构体 RemoteDataPortStruct 中的成员变量，并更改函数 RemoteControl()
  *
  * Please add comments after adding or deleting functions to ensure code
  * specification.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "driver_remote.h"
#include "arm_math.h"

/* ----------------------- Internal Data ----------------------------------- */
/** @brief The data received from the Dbus, is the memory address of the serial
  *        port DMA transfer
  */
uint8_t remote_Rx_Buffer[RC_FRAME_LENGTH];//原始数据位置
float test_current;
uint8_t AutoSmallBuffShot;
uint8_t AutoBigBuffShot;
uint8_t autoShot;
/** @brief According to the DT7 & DR16 2.4Ghz manual, 18 bytes of data are computed and
  *        stored in the struct variable
  */
//RC_Ctl_t RC_Ctl;
RemoteDataUnion RemoteData;//原始数据结构体
RemoteDataPortStruct RemoteDataPort;
enum
{
    REMOTE_MODE=1,
    KEYBOARD_MODE=2,
    AUTO_MODE=3,
}RemoteControlMode;


/** @brief All variables related to robot control
  */
#define KEY_MIN_SPEED (0.2f)
#define KEY_MAX_SPEED (1.0f)
#define KEY_MID_SPEED	(0.7f)

#define MOUSE_Y_SENTIVIVE (-40)
#define MOUSE_X_SENTIVIVE (-12)
#define MOUSE_Y_SENTIVIVE_SLOW (-1500)
#define MOUSE_X_SENTIVIVE_SLOW (-700)
#define AUTO_TIME_BEGINNING (850)
#define AUTO_TIME_MOVE (815)
#define AUTO_TIME_SHOOT (690)
#define AUTO_TIME_BACK (590)

#define AutoAimMode 0
#define AutoSmallBuffMode 1
#define AutoBigBuffMode 3

RemoteControlFlagSetPortStruct RemoteControlFlagSetPort = {0};//特殊需求标志位
//uint8_t BigBulletIncreFlag=0,NoBigBulletFlag=0;
uint8_t feedmotor_count = 0;
uint8_t AutoSmallBuffTrigger=0;
uint8_t AutoBigBuffTrigger = 0;
uint8_t shoot_continue;
/* ----------------------- Function Implements ---------------------------- */
RemoteDataPortStruct RemoteModeProcessData(RemoteDataProcessedStruct	RemoteDataReceive)
{
    static RemoteDataPortStruct	RemoteDataPortTemp={0};

    RemoteDataPortTemp.ChassisSpeedX	=	-	RemoteDataReceive.Channel_2;
    RemoteDataPortTemp.ChassisSpeedY	=	-	RemoteDataReceive.Channel_3;

    RemoteDataPortTemp.PitchIncrement	=		RemoteDataReceive.Channel_1/32;
    RemoteDataPortTemp.YawIncrement		=	-	RemoteDataReceive.Channel_0/16;

    if(RemoteDataReceive.Roller >1284)
        RemoteDataPortTemp.Magazine = ENABLE;
    else if(RemoteDataReceive.Roller<764)
        RemoteDataPortTemp.Magazine = DISABLE;
    else
        RemoteDataPortTemp.Magazine = RemoteDataPortTemp.Magazine;

    RockerDataConvert(&(RemoteDataPortTemp.ChassisSpeedX),&(RemoteDataPortTemp.ChassisSpeedY));

    //AimAutoData.AutoAim = 1;

    switch(RemoteDataReceive.RightSwitch)
    {
        case 1:RemoteDataPortTemp.Friction=DISABLE;
            RemoteDataPortTemp.FeedMotor=DISABLE;
            RemoteDataPortTemp.SuperCFlag=0;
            feedmotor_count = 0;
            break;
        case 3:RemoteDataPortTemp.Friction=ENABLE;
            RemoteDataPortTemp.FeedMotor=DISABLE;
            RemoteDataPortTemp.SuperCFlag=1;
            feedmotor_count = 0;
            break;
        case 2:RemoteDataPortTemp.Friction=ENABLE;
            if(feedmotor_count == 0) {
                RemoteDataPortTemp.FeedMotor = ENABLE;
                feedmotor_count = 1;
            }else
                RemoteDataPortTemp.FeedMotor = DISABLE;
            RemoteDataPortTemp.SuperCFlag=0;
            break;
        default:
            break;
    }
    RemoteDataPortTemp.Laser=RemoteDataPortTemp.Friction;

    return RemoteDataPortTemp;
}
uint8_t KeyQTrigger = 0;
uint8_t KeyETrigger =0;
//RemoteDataPortStruct KeyboardModeProcessData(RemoteDataProcessedStruct	RemoteDataReceive) {
//    static RemoteDataPortStruct RemoteDataPortTemp = {0};
//
//    static u8 FlagMagazineOpenAndClose = 0;
//    static u16 MouseRCount = 0, MouseLCount, MouseLCountDelay, MagazineCount = 0;
//
//    static float MouseXSentitiveValue = 0;
//    static float MouseYSentitiveValue = 0;
//    static u8 MagazineState = 0;
//
//    static float TurnARoundRef;
//    static u8 FlagTurning180 = 0, FlagTurningRight90 = 0, FlagTurningLeft90 = 0;
//    u8 FlagTurningTemp;
//
//    static u16 ShiftCount = 0;
//    static u16 CountKeyB = 0;
//    static u16 ResetCount = 0;
//
//    static u8 CtrlBResetFlag = 0;
//    static u8 CtrlZResetFlag = 0;
//
//    static u16 KeyBCount = 0, KeyZCount = 0, KeyRCount = 0, KeyCCount = 0, KeyVCount = 0,
//            KeyQCount = 0, KeyECount = 0, KeyXCount = 0, KeyFCount = 0, SentryCount = 0,
//            KeyShiftGCount = 0, KeyCtrlGCount = 0, KeyCtrlFCount = 0, KeyCtrlVCount = 0, KeyCtrlBCount = 0,
//            KeyCtrlWcount = 0, KeyCtrlScount = 0, KeyCtrlDcount = 0, KeyCtrlAcount = 0, KeyCtrlZCount = 0;
///* ----------------------------------------------------------------------- */
//    RemoteDataPortTemp.ChassisSpeedY = (float) (RemoteDataReceive.KeyS - RemoteDataReceive.KeyW) * (KEY_MAX_SPEED);
//    RemoteDataPortTemp.ChassisSpeedX = (float) (RemoteDataReceive.KeyA - RemoteDataReceive.KeyD) * (KEY_MAX_SPEED);
//    MouseYSentitiveValue = MOUSE_Y_SENTIVIVE;
//    MouseXSentitiveValue = MOUSE_X_SENTIVIVE;
//
//    RemoteDataPortTemp.PitchIncrement = MouseYSentitiveValue * RemoteDataReceive.MouseY;
//    RemoteDataPortTemp.YawIncrement = MouseXSentitiveValue * RemoteDataReceive.MouseX;
/* ----------------------------------------------------------------------- */
//    if(RemoteData.RemoteDataProcessed.Key.KeyShift)
//    {   RemoteDataPortTemp.SuperCFlag=1;//SuperC
//        RemoteDataPortTemp.AutoCFlag = 0;
//    }
//    else
//        RemoteDataPortTemp.SuperCFlag=0;
//    if(Yellow_Duck){
//        if(RemoteData.RemoteDataProcessed.Key.KeyCtrl){
//            RemoteDataPortTemp.Uphill=1;
//        }else{
//            RemoteDataPortTemp.Uphill =0;
//        }
//    }else {
//        if ((RemoteData.RemoteDataProcessed.Key.KeyCtrl)) {
//            RemoteDataPortTemp.AutoCFlag = 1;
//        } else {
//            RemoteDataPortTemp.AutoCFlag = 0;
//        }
//    }
/* --------------------------------Spin--------------------------------- */
//    if (RemoteData.RemoteDataProcessed.Key.KeyG)//Spin
//    {
//        RemoteDataPortTemp.SpinFlag = 1;
//        RemoteDataPortTemp.CancelFollow = 0;
//        RemoteDataPortTemp.Axialflag = 0;
//    }
//    if (RemoteData.RemoteDataProcessed.Key.KeyC) {
//        RemoteDataPortTemp.SpinFlag = 0;
//        RemoteDataPortTemp.CancelFollow = 0;
//        RemoteDataPortTemp.Axialflag = 0;
//
//    }
//    if(Yellow_Duck){
//        if(RemoteData.RemoteDataProcessed.Key.KeyX)
//        {
//            RemoteDataPortTemp.Axialflag = 1;
//            RemoteDataPortTemp.CancelFollow = 0;
//            RemoteDataPortTemp.SpinFlag=0;
//
//        }
//    }
/* ------------------------------Autoaim status-------------------------------- */
//    if(RemoteData.RemoteDataProcessed.Key.KeyCtrl&&RemoteData.RemoteDataProcessed.Key.KeyC)
//    {
//        RemoteDataPortTemp.CancelFollow=0;
//        AimAutoData.AutoAimStatus=0;//autoaim
//        GimbalSetLocationDataTemp.FlagPitchUseEncoder = 1;
//
//    }
//    if(RemoteData.RemoteDataProcessed.Key.KeyCtrl&&RemoteData.RemoteDataProcessed.Key.KeyZ)
//    {
//        RemoteDataPortTemp.CancelFollow=1;
//        AimAutoData.AutoAimStatus=1;//small buff
//        RemoteDataPortTemp.SpinFlag=0;
//        GimbalSetLocationDataTemp.FlagPitchUseEncoder = 1;
//
//    }
//
//    if(RemoteData.RemoteDataProcessed.Key.KeyCtrl&&RemoteData.RemoteDataProcessed.Key.KeyX)	{
//        RemoteDataPortTemp.CancelFollow=1;
//        AimAutoData.AutoAimStatus=3;//large buff
//        RemoteDataPortTemp.SpinFlag=0;
//        GimbalSetLocationDataTemp.FlagPitchUseEncoder = 1;
//
//    }
///* ----------------------------------------------------------------------- */
//    if (RemoteDataReceive.KeyZ)//Ramp need to be close after usage
//    {
//        RemoteDataPortTemp.Balance = 0;
//    } else {
//        RemoteDataPortTemp.Balance = 1;
//    }
//


/* ----------------------------------------------------------------------- */

//
//    switch(AimAutoData.AutoAimStatus){
//        case AutoAimMode:
//            AutoSmallBuffShot = DISABLE;
//            AutoBigBuffShot = DISABLE;
//            if(RemoteDataReceive.RightMousePress)//Aim Control Permit
//            {
//                AimAutoData.AutoAim = 1;
//            }
//            else{
//                AimAutoData.AutoAim = 0;
//            };
//            break;
//        case AutoSmallBuffMode:
//            if(WAIT_ENERGY_BUFF){
//                if((RemoteDataReceive.RightMousePress)&&(AutoSmallBuffTrigger==0)){
//                    AutoSmallBuffShot = ENABLE;
//                    AutoSmallBuffTrigger = 1;
//                }else if(!RemoteDataReceive.RightMousePress){
//                    AutoSmallBuffShot = DISABLE;
//                    AutoSmallBuffTrigger = 0;
//                }else {
//                    AutoSmallBuffTrigger = 1;
//                }
//            }else{
//                AutoSmallBuffShot = ENABLE;
//                if(RemoteDataReceive.RightMousePress){
//                    autoShot=1;
//                }else{
//                    autoShot=0;
//                }
//            }
//            break;
//        case AutoBigBuffMode:
//            if(WAIT_ENERGY_BUFF){
//                if((RemoteDataReceive.RightMousePress)&&(AutoBigBuffTrigger==0)){
//                    AutoBigBuffShot = ENABLE;
//                    AutoBigBuffTrigger = 1;
//                }else if(!RemoteDataReceive.RightMousePress){
//                    AutoBigBuffShot = DISABLE;
//                    AutoBigBuffTrigger = 0;
//                }else {
//                    AutoBigBuffTrigger = 1;
//                }
//            }else{
//                AutoBigBuffShot = ENABLE;
//                if(RemoteDataReceive.RightMousePress){
//                    autoShot=1;
//                }else{
//                    autoShot=0;
//                }
//            }
//            break;
//    }
/* ----------------------------friction control----------------------------- */
//    if(((!RemoteData.RemoteDataProcessed.Key.KeyCtrl)&&RemoteData.RemoteDataProcessed.Key.KeyV))
//    {
//        RemoteDataPortTemp.Friction=1;
//        RemoteDataPortTemp.Laser=1;
//    }
//
//    if((!RemoteData.RemoteDataProcessed.Key.KeyCtrl)&&RemoteData.RemoteDataProcessed.Key.KeyB)
//    {
//        RemoteDataPortTemp.Friction=0;
//        RemoteDataPortTemp.Laser=0;
//    }
///* ----------------------------------------------------------------------- */
//
//    if(RemoteDataPortTemp.Friction)
//    {   if(ChassisReceiveData.Judge == ENABLE) {
//            if (RemoteDataReceive.LeftMousePress) {
//                MouseLCount++;
//            } else {
//                MouseLCountDelay = 0;
//                MouseLCount = 0;
//            }
//            if ((MouseLCount > 1) && (MouseLCount < 20)) {
//                RemoteDataPortTemp.FeedMotor = RemoteDataReceive.LeftMousePress;
//                MouseLCountDelay++;
//                if (MouseLCountDelay > 1) {
//                    RemoteDataPortTemp.FeedMotor = 0;//如果需要成单发模式，则去掉注释
//                }
//            } else if (MouseLCount >= 20) {
//                RemoteDataPortTemp.FeedMotor = RemoteDataReceive.LeftMousePress;
//            } else {
//                RemoteDataPortTemp.FeedMotor = 0;
//            }
//        }else{
//            if((RemoteDataReceive.LeftMousePress)&&(MouseLCount==0)){
//                RemoteDataPortTemp.FeedMotor = 1;
//                MouseLCount=1;
//            }else if(RemoteDataReceive.LeftMousePress&&(MouseLCount!=0)){
//                RemoteDataPortTemp.FeedMotor = 0;
//            }else{
//                RemoteDataPortTemp.FeedMotor = 0;
//                MouseLCount = 0;
//            }
//
//        }
//    }
//    else
//    {
//        MouseLCount=0;
//        RemoteDataPortTemp.FeedMotor = DISABLE;
//    }
//
///* -----------------------Turning control logic------------------------------- */
//    FlagTurningTemp = FlagTurningRight90 + FlagTurningLeft90 ;
//    if((!RemoteDataReceive.KeyCtrl)&&(RemoteDataReceive.KeyQ)&&(KeyQTrigger==0))
//    {
//        if(FlagTurningTemp==0)
//        {
//            FlagTurningRight90=1;
//            TurnARoundRef=GetYawGyroValue()+0.25f;
//        }
//        KeyQTrigger = 1;
//    } else if((!RemoteDataReceive.KeyCtrl)&&(!RemoteDataReceive.KeyQ)){
//        KeyQTrigger = 0;
//    }
//    FlagTurningTemp = FlagTurningRight90 + FlagTurningLeft90 ;
//    if((!RemoteDataReceive.KeyCtrl)&&(RemoteDataReceive.KeyE)&&(KeyETrigger==0))
//    {
//        if(FlagTurningTemp==0)
//        {
//            FlagTurningLeft90=1;
//            TurnARoundRef=GetYawGyroValue()-0.25f;
//        }
//        KeyETrigger = 1;
//    }else if((!RemoteDataReceive.KeyCtrl)&&(!RemoteDataReceive.KeyE)){
//        KeyETrigger =0 ;
//    }
///* ----------------------------------------------------------------------- */
//    if(RemoteDataReceive.KeyV&&RemoteDataReceive.KeyCtrl)//ChassisInit
//    {
//        RemoteDataPortTemp.ChassisInitFlag=1;
//        ResetCount++;
//        if(ResetCount==100)
//        {
//            GimbalDataInit();
//        }
//        if(ResetCount==200)
//        {
//            NVIC_SystemReset();
//        }
//    }
//    else
//    {
//        ResetCount=0;
//        RemoteDataPortTemp.ChassisInitFlag=0;
//    }
//
///* ----------------------------------------------------------------------- */
//    if ((!RemoteDataReceive.KeyCtrl) && RemoteDataReceive.KeyF)//close magazine
//    {
//        FlagTurningLeft90 = 0;
//        FlagTurningRight90 = 0;
//        RemoteDataPortTemp.Magazine = DISABLE;
//        RemoteDataPortTemp.FlagGimbalPitchLock = 0;
//    }
//    /* ----------------------------------------------------------------------- */
//    if ((!RemoteDataReceive.KeyCtrl) && RemoteDataReceive.KeyR)//open magazine
//    {
//        RemoteDataPortTemp.FlagGimbalPitchLock = 1;
//        MagazineCount = 0;
//        MagazineState = RemoteDataPortTemp.Friction;
//    }
//
//    if (MagazineCount > 10) {
//        RemoteDataPortTemp.Magazine = ENABLE;
//        MagazineCount = 0;
//    }
//    if (RemoteDataPortTemp.FlagGimbalPitchLock) {
//        FlagTurningLeft90 = 0;
//        FlagTurningRight90 = 0;
//        RemoteDataPortTemp.PitchIncrement = 0;
//
//        MagazineCount++;
//    }
///* ----------------------------------------------------------------------- */
//    if(FlagTurningLeft90)
//    {
//        RemoteDataPortTemp.YawIncrement = -3.57142f/2.0f;//3*(TurnARoundRef-GetYawGyroValue());
//        FlagTurningLeft90 = 0;
//        TurnARoundRef = 0;
//    }
//    if(FlagTurningRight90)
//    {
//        RemoteDataPortTemp.YawIncrement = 3.57142f/2.0f;//3*(TurnARoundRef-GetYawGyroValue());
//        FlagTurningRight90 = 0;
//        TurnARoundRef = 0;
//    }
///* ----------------------------------------------------------------------- */
//    return RemoteDataPortTemp;
//}
    RemoteDataPortStruct RemoteDataCalculate(RemoteDataProcessedStruct RemoteDataReceive) {
        RemoteDataPortStruct RemoteDataPortTemp;

//        RemoteControlMode = RemoteDataReceive.LeftSwitch;

       // switch (RemoteControlMode) {
           // case REMOTE_MODE://1
           RemoteDataPortTemp = RemoteModeProcessData(RemoteDataReceive);
//                RemoteDataPortTemp.Magazine = RemoteDataPortTemp.Magazine;
//                RemoteDataPortTemp.YawLocationInit = RemoteControlFlagSetPort.YawLocationInit;
//                RemoteDataPortTemp.ChassisInitFlag = RemoteControlFlagSetPort.ChassisInitFlag;
//                RemoteDataPortTemp.RampFlag = RemoteControlFlagSetPort.RampFlag;
//            AimAutoData.AutoAim = 1;
//                RemoteDataPortTemp.Control = 0;
//                if (RemoteDataReceive.RightSwitch == SwitchDown) {
//                    RemoteDataPortTemp.SpinFlag = 1;
//                    RemoteDataPortTemp.CancelFollow = RemoteControlFlagSetPort.CancelFollow;
//                    AutoSmallBuffShot = 0;
//                    AutoBigBuffShot = 0;
//                } else if (RemoteDataReceive.RightSwitch == SwitchUp) {
//                    RemoteDataPortTemp.Friction = ENABLE;
//                    RemoteDataPortTemp.CancelFollow = 1;
//                    RemoteDataPortTemp.SpinFlag = 0;
//
//                    AutoSmallBuffShot = 1;
//                    autoShot = RemoteDataPortTemp.Magazine;
//                } else if (RemoteDataReceive.RightSwitch == SwitchMid) {
//                    RemoteDataPortTemp.Friction = ENABLE;
//                    RemoteDataPortTemp.CancelFollow = 1;
//                    RemoteDataPortTemp.SpinFlag = 0;
//
//                    AutoBigBuffShot = 1;
//                    autoShot = RemoteDataPortTemp.Magazine;
//
//                }
//                break;
//            case KEYBOARD_MODE://2
////                RemoteDataPortTemp = KeyboardModeProcessData(RemoteDataReceive);
//                RemoteDataPortTemp.Control = 1;
//                break;
//            case AUTO_MODE://3
//                RemoteDataPortTemp = RemoteModeProcessData(RemoteDataReceive);
//                RemoteDataPortTemp.SpinFlag = 0;
//                RemoteDataPortTemp.CancelFollow = RemoteControlFlagSetPort.CancelFollow;
//                RemoteDataPortTemp.Magazine = RemoteDataPortTemp.Magazine;
//                RemoteDataPortTemp.YawLocationInit = RemoteControlFlagSetPort.YawLocationInit;
//                RemoteDataPortTemp.ChassisInitFlag = RemoteControlFlagSetPort.ChassisInitFlag;
//                RemoteDataPortTemp.RampFlag = RemoteControlFlagSetPort.RampFlag;
////                AimAutoData.AutoAim = 1;
////                AimAutoData.AutoAimStatus = 0;//large buff
//                AutoSmallBuffShot = 0;
//                AutoBigBuffShot = 0;
//                RemoteDataPortTemp.Control = 0;
//                break;
        //}
        return RemoteDataPortTemp;
    }



/**
  * @brief  Solve the DR16 Remote data
  * @param  remote_Rx_Buffer : original received data
  * @param  RC_Ctl : processed data
  * @retval None
  * 测试内容：首先检查RemoteData.RemoteDataRaw[i]中是否是DR16的原始数据，是否会根据遥控器的波动而改变
  * 当检查完数据之后，便检查RemoteData联合体中的摇杆及键盘数据是否正确。是否能够按照键鼠的改变而改变
  */
RemoteDataProcessedStruct RemoteDataReceive;        //建立虚拟控制器

    int RC_Update(void) {


        //Step	1	:	Receive remote raw data from buffer
        for (int i = 0; i < 18; i++)
            RemoteData.RemoteDataRaw[i] = remote_Rx_Buffer[i];//遥控器内容输入
        RemoteDataReceive = RemoteDataProcess(RemoteData);//检查数据正确并压缩到虚拟控制器中

        //Step	2	:	Judge Validity
        if (RemoteDataReceive.FlagValidity) {

            //Step	3	Process	remote data	and	Save into RemoteDataPort
            RemoteDataPort = RemoteDataCalculate(RemoteDataReceive);//重点：进行主要控制
            return 0;
        }
        return 1;
    }
/**
  * @brief  Get all the control signals needed for robot control
  * @param  RC_Ctl : data before processing
  * @param  RemoteData : the processed data
  * @retval None
  * 这个函数的作用是做整个遥控器数据的控制与分发 并不参与到具体的控制中。所有的控制任务从这个结构体中取出数据并在自身文件中进行处理。
  */
//0,正常
//1，数据有误，不要用
    uint8_t JudgeDataValidity(uint16_t data) {
        if (data <= 1685 && data >= 363)
            return 0;
        else
            return 1;
    }
    RemoteDataProcessedStruct RemoteDataProcess(RemoteDataUnion RemoteDataRaw) {
        RemoteDataProcessedStruct RemoteDataProcessed;


        uint8_t JudgeDataValiditySum = 0;
        float MouseX, MouseY, MouseZ;
//**********************************摇杆***********************************************

        JudgeDataValiditySum = JudgeDataValidity(RemoteDataRaw.RemoteDataProcessed.RCValue.Ch0) +
                               JudgeDataValidity(RemoteDataRaw.RemoteDataProcessed.RCValue.Ch1) +
                               JudgeDataValidity(RemoteDataRaw.RemoteDataProcessed.RCValue.Ch1) +
                               JudgeDataValidity(RemoteDataRaw.RemoteDataProcessed.RCValue.Ch3);

        if (JudgeDataValiditySum)                    //检查数据合法性
        {
            RemoteDataProcessed.FlagValidity = 0;
            return RemoteDataProcessed;
        }

        //基本量偏移消除
        if (RemoteDataRaw.RemoteDataProcessed.RCValue.Ch0 > 1030 ||
            RemoteDataRaw.RemoteDataProcessed.RCValue.Ch0 < 1018)
            RemoteDataProcessed.Channel_0 =
                    ((float) (RemoteDataRaw.RemoteDataProcessed.RCValue.Ch0 - RC_CH_VALUE_OFFSET)) /
                    (RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
        else
            RemoteDataProcessed.Channel_0 = 0;

        //基本量偏移消除
        if (RemoteDataRaw.RemoteDataProcessed.RCValue.Ch1 > 1030 ||
            RemoteDataRaw.RemoteDataProcessed.RCValue.Ch1 < 1018)
            RemoteDataProcessed.Channel_1 =
                    ((float) (RemoteDataRaw.RemoteDataProcessed.RCValue.Ch1 - RC_CH_VALUE_OFFSET)) /
                    (RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
        else
            RemoteDataProcessed.Channel_1 = 0;

        if (RemoteDataRaw.RemoteDataProcessed.RCValue.Ch2 > 1030 ||
            RemoteDataRaw.RemoteDataProcessed.RCValue.Ch2 < 1018)
            RemoteDataProcessed.Channel_2 =
                    ((float) (RemoteDataRaw.RemoteDataProcessed.RCValue.Ch2 - RC_CH_VALUE_OFFSET)) /
                    (RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
        else
            RemoteDataProcessed.Channel_2 = 0;

        if (RemoteDataRaw.RemoteDataProcessed.RCValue.Ch3 > 1030 ||
            RemoteDataRaw.RemoteDataProcessed.RCValue.Ch3 < 1018)
            RemoteDataProcessed.Channel_3 =
                    ((float) (RemoteDataRaw.RemoteDataProcessed.RCValue.Ch3 - RC_CH_VALUE_OFFSET)) /
                    (RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
        else
            RemoteDataProcessed.Channel_3 = 0;

//**********************************拨扭***********************************************
//        RemoteDataProcessed.LeftSwitch = RemoteDataRaw.RemoteDataProcessed.RCValue.s2;
//        RemoteDataProcessed.RightSwitch = RemoteDataRaw.RemoteDataProcessed.RCValue.s1;

//**********************************鼠标***********************************************
        MouseX = RemoteDataRaw.RemoteDataProcessed.Mouse.x / 32768.0f;
        MouseY = RemoteDataRaw.RemoteDataProcessed.Mouse.y / 32768.0f;
        MouseZ = RemoteDataRaw.RemoteDataProcessed.Mouse.z / 32768.0f;

        if (MouseX >= MOUSEERRORTOLERANCE || MouseX <= -MOUSEERRORTOLERANCE)
            RemoteDataProcessed.MouseX = MouseX;
        else
            RemoteDataProcessed.MouseX = 0;

        if (MouseY >= MOUSEERRORTOLERANCE || MouseY <= -MOUSEERRORTOLERANCE)
            RemoteDataProcessed.MouseY = MouseY;
        else
            RemoteDataProcessed.MouseY = 0;

        if (MouseZ >= MOUSEERRORTOLERANCE || MouseZ <= -MOUSEERRORTOLERANCE)
            RemoteDataProcessed.MouseZ = MouseZ;
        else
            RemoteDataProcessed.MouseZ = 0;

//        RemoteDataProcessed.LeftMousePress = RemoteDataRaw.RemoteDataProcessed.Mouse.Press_l;
//        RemoteDataProcessed.RightMousePress = RemoteDataRaw.RemoteDataProcessed.Mouse.Press_r;

        RemoteDataProcessed.KeyW = RemoteDataRaw.RemoteDataProcessed.Key.KeyW;
        RemoteDataProcessed.KeyS = RemoteDataRaw.RemoteDataProcessed.Key.KeyS;
        RemoteDataProcessed.KeyA = RemoteDataRaw.RemoteDataProcessed.Key.KeyA;
        RemoteDataProcessed.KeyD = RemoteDataRaw.RemoteDataProcessed.Key.KeyD;
        RemoteDataProcessed.KeyShift = RemoteDataRaw.RemoteDataProcessed.Key.KeyShift;
        RemoteDataProcessed.KeyCtrl = RemoteDataRaw.RemoteDataProcessed.Key.KeyCtrl;
        RemoteDataProcessed.KeyQ = RemoteDataRaw.RemoteDataProcessed.Key.KeyQ;
        RemoteDataProcessed.KeyE = RemoteDataRaw.RemoteDataProcessed.Key.KeyE;
        RemoteDataProcessed.KeyR = RemoteDataRaw.RemoteDataProcessed.Key.KeyR;
        RemoteDataProcessed.KeyF = RemoteDataRaw.RemoteDataProcessed.Key.KeyF;
        RemoteDataProcessed.KeyG = RemoteDataRaw.RemoteDataProcessed.Key.KeyG;
        RemoteDataProcessed.KeyZ = RemoteDataRaw.RemoteDataProcessed.Key.KeyZ;
        RemoteDataProcessed.KeyX = RemoteDataRaw.RemoteDataProcessed.Key.KeyX;
        RemoteDataProcessed.KeyC = RemoteDataRaw.RemoteDataProcessed.Key.KeyC;
        RemoteDataProcessed.KeyV = RemoteDataRaw.RemoteDataProcessed.Key.KeyV;
        RemoteDataProcessed.KeyB = RemoteDataRaw.RemoteDataProcessed.Key.KeyB;

        RemoteDataProcessed.Roller = RemoteDataRaw.RemoteDataProcessed.resv;
        RemoteDataProcessed.FlagValidity = 1;
        return RemoteDataProcessed;
    }
void RockerDataConvert(float *x,float *y)
{
    if(fabsf(*x)+fabsf(*y)>1)
    {
        if(*y>0 && *x>0)				//1象限
        {
            *y=*y/(*y+*x);
            *x=*x/(*y+*x);
        }
        else if(*y>0 && *x<0)		//2象限
        {
            *x=*x/(*y-*x);
            *y=*y/(*y-*x);
        }
        else if(*y<0 && *x>0)		//4象限
        {
            *x=-*x/(*y-*x);
            *y=-*y/(*y-*x);
        }
        else										//3象限
        {
            *x=-*x/(*x+*y);
            *y=-*y/(*x+*y);
        }
    }
}