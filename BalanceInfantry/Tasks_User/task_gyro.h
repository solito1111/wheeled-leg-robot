//
// Created by 45441 on 2023/7/6.
//

#ifndef INFANTRYGIMBALC_TASK_GYRO_H
#define INFANTRYGIMBALC_TASK_GYRO_H
#include "stdint.h"
#include "BMI088driver.h"
#include "QuaternionEKF.h"
#include "bsp_struct_typedef.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "driver_gyro.h"
#ifdef __cplusplus
extern "C" {
#endif
//C
#define X_T 0
#define Y_T 1
#define Z_T 2

#define INS_TASK_PERIOD 1

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_t;

extern INS_t INS;


/**
 * @brief 用于修正安装误差的参数,demo中可无视
 *
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;


extern INS_t INS;
void INS_Init(void);
void INS_Task(void);
void IMU_Temperature_Ctrl(void);

void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

void Gyro_task(void *argument);




#ifdef __cplusplus
}
#endif
//C++
 //INFANTRYGIMBALC_TASK_GYRO_H
#endif