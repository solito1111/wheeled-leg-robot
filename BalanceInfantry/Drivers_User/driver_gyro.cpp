//
// Created by 45441 on 2023/7/6.
//

#include "driver_gyro.h"
#include "bsp_struct_typedef.h"
#include "task_gyro.h"
#include "bsp_dwt.h"
volatile IMU imu;

void GyroDataConvert(){
    //imu.yaw_offset = YAWOFFSET;
    imu.pitch = INS.Pitch;
    imu.roll = INS.Roll;//*0.1f+imu.roll*0.9f;
    imu.yaw = INS.Yaw;//
    //imu.yaw -= imu.yaw_offset*osKernelGetTickCount();
    imu.pitch_speed = INS.Gyro[0];
    imu.roll_speed = INS.Gyro[1];
    imu.yaw_speed = INS.Gyro[2];

    imu.ax = INS.Accel[0];
    imu.ay = INS.Accel[1];
    imu.az = INS.Accel[2];

    float YawGyroDataTemp;
    static float YawGyroDataLast=0;
    static int YawGyroCount=0;
    static float FilterK=0.1;//低通滤波系数
    YawGyroDataTemp	=	imu.yaw;
    YawGyroDataTemp	=	YawGyroDataTemp	/	360.0f;

    if			(YawGyroDataTemp	-	YawGyroDataLast	>	0.8f)
    {YawGyroCount--;YawGyroDataLast++;}
    else if	(YawGyroDataTemp	-	YawGyroDataLast	<	-0.8f)
    {YawGyroCount++;YawGyroDataLast--;}
    imu.yaw_sum	=	YawGyroDataTemp*FilterK+YawGyroDataLast*(1-FilterK) + YawGyroCount;
    YawGyroDataLast 	= YawGyroDataTemp ;



}
