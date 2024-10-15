//
// Created by 45441 on 2023/7/6.
//

#ifndef INFANTRYGIMBALC_DRIVER_GYRO_H
#define INFANTRYGIMBALC_DRIVER_GYRO_H
#include "cmsis_os2.h"
#include "main.h"
#include "task_gyro.h"
#define YAWOFFSET (0.f)
#ifdef __cplusplus
extern "C" {
#endif

//C
typedef struct {
    volatile float pitch;
    volatile float roll;
    volatile float yaw;
    volatile float yaw_last;
    volatile float yaw_sum;
    volatile float yaw_init;
    volatile float yaw_offset;
    volatile float pitch_speed;
    volatile float roll_speed;
    volatile float yaw_speed;

    volatile float ax;
    volatile float ay;
    volatile float az;
}IMU;
void GyroDataConvert();
extern volatile IMU imu;

#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_DRIVER_GYRO_H
