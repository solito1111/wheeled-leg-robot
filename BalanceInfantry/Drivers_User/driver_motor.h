//
// Created by 45441 on 2023/11/27.
//

#ifndef INFANTRYGIMBALC_DRIVER_MOTOR_H
#define INFANTRYGIMBALC_DRIVER_MOTOR_H
#include "main.h"
#include "bsp_dwt.h"
#include <array>
#include "cmsis_os2.h"
#include "bsp_can.h"
#include "kalman_filter.h"
#include "task_lostCounter.h"
#ifdef __cplusplus
extern "C" {
#endif
//C
/*定义关节电机的控制模式*/
#define CMD_MOTOR_MODE      0x05
#define CMD_RESET_MODE      0x06
#define CMD_ZERO_POSITION   0x07

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
/*定义关节电机的控制参数幅值*/
#define P_MIN (-95.5f)    // Radians
#define P_MAX 95.5f
#define V_MIN (-45.0f)    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN (-18.0f)
#define T_MAX (18.0f)
class Motor{
public:
    //电机反馈数据
    volatile float angle;
    volatile float velocity;
    volatile float velocity_temp;
    volatile float current;
    //解算数据
    volatile float angle_last;
    volatile float velocity_watch;
    volatile float velocity_last;
    volatile float position;
    volatile float position_last;
    //控制帧数据
     float torque_ref;//力矩或者电流
     float p;
     float v;
     float kp;
     float kv;
    volatile float radio;//比例系数,控制电机正反转
    //实时监测帧率与工作状态
    volatile float frequency;
    uint32_t motor_cnt;
    uint8_t detect;
    uint32_t lost_counter_count_number;
    Motor() {};
     void Motor_Init();
     uint8_t get_motor_measure(const uint8_t data[8]);
     void torque_set();
     void ZeroPosition();
    float get_position(float angle_temp){
        delta = angle - angle_last;
        if(delta > angle_temp){
            delta = delta - (2*angle_temp);
        }else if(delta <  -angle_temp){
            delta = delta + (2*angle_temp);
        }
        angle_sum+= delta;
        return angle_sum;
    }
private:
    float delta;
    float angle_sum;
};
class HT_Motor:public Motor{
public:
    float velocity_hat;
    uint32_t motor_id;
    uint8_t send_data[8];
    uint8_t get_motor_measure(const uint8_t data[8]);
    kalman_filter_t motor_kalman={0};
    kalman_filter_init_t motor_kalman_filter_para = {
            .xhat_data = {0, 0},
            .P_data = {1, 0, 0, 1},
            .A_data = {1, 0.001f, 0, 1},
            .H_data = {1, 0, 0, 1},
            .Q_data = {1, 0, 0, 1},
            .R_data = {3000, 0, 0, 1500}
    };
    void get_send_data();
    uint8_t init_buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};

private:
    uint16_t p_temp;
    uint16_t v_temp;
    uint16_t kp_temp;
    uint16_t kv_temp;
    uint16_t torque_temp;
    uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
        float span = x_max - x_min;
        float offset = x_min;

        return (uint16_t) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
    }
    float uint_to_float(int x_int, float x_min, float x_max, int bits) {
        float span = x_max - x_min;
        float offset = x_min;
        return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
    }

};
class DJI_Motor:public Motor{
public:
//    float angle_max;
//    float velocity_max;
//    float torque_max;
    void motor_init();
    uint8_t get_motor_measure(const uint8_t data[8]);
private:
    float velocity_max=9600.f;
    float angle_max = 8191.f;
    float torque_max = 30000.f;
};
//根据手册自行修改
class Motor3508:public DJI_Motor{
};
class Motor6020:public DJI_Motor{
};
class Motor2006:public DJI_Motor{
};
class Joint_Motor:public HT_Motor{
public:
    Joint_Motor(){};
    void Motor_Init();
    void RollBack_Init();
    void ZeroPosition();
    void torque_set();

};

class Wheel_Motor:public HT_Motor{
public:
    Wheel_Motor(){};
    void Motor_Init();
    void RollBack_Init();
    void ZeroPosition();
    void torque_set();


};
#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_DRIVER_MOTOR_H
