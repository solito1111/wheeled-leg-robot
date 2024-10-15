//
// Created by 45441 on 2023/10/21.
//

#ifndef INFANTRYGIMBALC_DRIVER_CHASSIS_H
#define INFANTRYGIMBALC_DRIVER_CHASSIS_H
#include "cmsis_os2.h"
#include "driver_remote.h"
#include "bsp_dwt.h"
#include "main.h"
#include "user_lib.h"
#include "driver_motor.h"
#include "kalman_filter_whx.h"
#ifdef __cplusplus
extern "C" {
#endif
//C
//enum StandupState{
//    StandupState_None,
//    StandupState_Prepare,
//    StandupState_Standup,
//}StandupState;
#define l1 0.15f
#define l2 0.26f//0.26f
#define l3 0.26f//0.26f
#define l4 0.15f//0.15f
#define l5 0.15f//0.15f

#define M  15.f
#define Mp 1.1f
#define l 0.011f
#define g 9.7592433f
#define WHEEL_RADIUS 0.1f

#define LEG_LENGTH_MAX 0.35f
#define LEG_LENGTH_MIN 0.1f
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef enum {
    StandupState_None,
    StandupState_Prepare,
    StandupState_Standup,
}StandupState;
typedef enum{
    lengthDown,
    lengthMid,
    lengthUp,
}LengthStatus;
typedef struct{
    uint8_t	followFlag;
    uint8_t axialFlag;
    uint8_t jumpFlag;
    uint8_t balanceFlag;
    uint8_t spinFlag;
    uint8_t rollFlag;
    uint8_t legLengthFlag;
    uint8_t friFlag;
    uint8_t superCFlag;
    uint8_t chassisInitFlag;
    uint8_t visionFlag;
}ControlFlag;//一些附加功能标志位
class ControlInput{
public:
    float speed;
    float speed2;
    float legLength;
    float yaw;
    float rollRef;
    ControlFlag controlFlag;
};
class Leg_Posture{
public:
    std::array<Joint_Motor,2> joint_leg_motors;
    volatile float length;
    volatile float angle;
    volatile float dLength;
    volatile float ddLength;
    volatile float ddLength_last;
    volatile float dLength_last;
    volatile float dAngle;
    volatile float dAngle_watch;
    volatile float dAngle_last;
    volatile float Tp;//髋关节输出扭矩
    volatile float Force;//左右腿推力
    volatile float Tp_div;//算出来的反馈力
    volatile float Force_div;
    volatile float TLeg;
    volatile float T[2];
    volatile float supportForce;
    volatile bool isGround;
    volatile bool isGroundLast;
    volatile bool isCuchioning;
    volatile bool isCuchioningLast;
    volatile float torque[2];
    volatile uint32_t lastTouchTime;
    Leg_Posture(){};
    void leg_feedback_update(Joint_Motor *jointMotor1,Joint_Motor *jointMotor2);
    void leg_vmc_calculate();
    void leg_vmc_calculate_div(Joint_Motor motor1,Joint_Motor motor2);

public:
    float phi1;
    float phi2;
    float phi3;
    float phi4;
    float phi0;
    float phi0_last;
    float x_b;
    float y_b;
    float x_d;
    float y_d;
    float x_c;
    float y_c;
    float A;
    float B;
    float C;
    float L_BD;
    float phi1_dot;
    float phi4_dot;
    float x_b_dot;
    float x_d_dot;
    float y_b_dot;
    float y_d_dot;
    float phi2_dot;
    float phi0_dot;
private:
    float phi0_spd(float phi_1,float phi_4,float phi1_spd,float phi4_spd);
};
class StateVar{
public:
    float phi;
    float phi_dot;
    float x_left;
    float x_right;
    float x;
    float x_dot;
    float x_dot_right;
    float x_dot_right_last;
    float x_dot_left;
    float x_dot_left_last;
    float theta;
    float theta_left;
    float theta_right;
    float theta_dot;
    float theta_dot_left;
    float theta_dot_right;
    float theta_dot_left_last;
    float theta_dot_right_last;
    float z_w_ddot_left;
    float z_w_ddot_right;
    float legLength;
    float robot_ax;
    float robot_vx;
    float x_dot_left_hat;
    float x_dot_right_hat;
    bool isCuchioning;
    bool isCuchiongingLast;
    bool isCuchioningFinish;
    bool isGround;
    bool isGroundLast;
    float theta_left_hat[5];
    float theta_right_hat[5];
    float theta_left_dot_hat[5];
    float theta_right_dot_hat[5];
    float x_left_hat[5];
    float x_right_hat[5];
    float x_left_dot_hat[5];
    float x_right_dot_hat[5];
    float phi_hat[5];
    float phi_dot_hat[5];
    StateVar(){};
    void StateVar_Update();

};
class LQR{
public:
    float K[2][6];
    float radio[2][6];
    float legLength;
    float legLength2;
    float legLength3;
    float outputT[2];
    float outputTp[2];
    LQR(){};
    void K_Calculate(float leg_length,bool ground);
    float T_Calculate();
    float Tp_Calculate();

};
typedef struct
{
    float Input;

    float h0;///调参
    float r;

    float x;
    float dx;
    float ddx;

    float last_dx;
    float last_ddx;

    uint32_t DWT_CNT;
    float dt;
} TD_t;
extern float torque_ref_follow;
extern ControlInput controlSignal;;
extern float target_leglength;
extern volatile float target_leglength_left;
extern volatile float target_leglength_right;
extern uint8_t motor_OK;
extern std::array<Leg_Posture,2> leg_posture_array;
extern std::array<Joint_Motor,4> joint_motors_array;
extern std::array<Wheel_Motor,2> wheel_motors_array;
extern StateVar state_var;
extern LQR lqr;
extern float target_speed;
extern float target_position;
extern Motor6020 YawMotor;
extern uint8_t motor_error;
extern uint8_t jump_finish;
extern  uint8_t jumpFlagTemp;
extern kalman_filter_t phi_kalman;
extern kalman_filter_t x_dot_kalman;
extern kalman_filter_t x_dot_right_kalman;
extern float target_yaw;
extern float x_dot_left_watch;
extern uint8_t jumping;
extern Ordinary_Least_Squares_t xOls;
extern TD_t left_leg_td,right_leg_td;
extern float robot_m;
extern KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体
extern uint8_t down;
extern uint8_t spin_inverse ;
void ChassisInit();
void chassisFollowCalculate(uint8_t follow_flag,uint8_t axial_flag);
void legLengthCalculate(uint8_t jump_flag);
void forceLegCalculate();
void ifGroundDetect();
void tpLegCalculate();
void Chassis_Task(void *argument);

void TD_Init(TD_t *td, float r, float h0);
float TD_Calculate(TD_t *td, float input);
//void get_tof();

//#define USART_REC_LEN  			200  	//定义最大接收字节数 200
//#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
//
//extern char  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符
//extern uint16_t USART_RX_STA;         		//接收状态标记
//
//#define u8 uint8_t
//#define u16 uint16_t
//#define u32 uint32_t
//
//#define false 0
//#define true 1
//
//#define HEADER 0xAA							/* 起始符 */
//#define device_address 0x00     /* 设备地址 */
//#define chunk_offset 0x00       /* 偏移地址命令 */
//#define PACK_GET_DISTANCE 0x02 	/* 获取测量数据命令 */
//#define PACK_RESET_SYSTEM 0x0D 	/* 复位命令 */
//#define PACK_STOP 0x0F 				  /* 停止测量数据传输命令 */
//#define PACK_ACK 0x10           /* 应答码命令 */
//#define PACK_VERSION 0x14       /* 获取传感器信息命令 */
//
//typedef struct {
//    int16_t distance;  						/* 距离数据：测量目标距离单位 mm */
//    uint16_t noise;		 						/* 环境噪声：当前测量环境下的外部环境噪声，越大说明噪声越大 */
//    uint32_t peak;								/* 接收强度信息：测量目标反射回的光强度 */
//    uint8_t confidence;						/* 置信度：由环境噪声和接收强度信息融合后的测量点的可信度 */
//    uint32_t intg;     						/* 积分次数：当前传感器测量的积分次数 */
//    int16_t reftof;   						/* 温度表征值：测量芯片内部温度变化表征值，只是一个温度变化量无法与真实温度对应 */
//}LidarPointTypedef;
//
//struct AckResultData{
//    uint8_t ack_cmd_id;						/* 答复的命令 id */
//    uint8_t result; 							/* 1表示成功,0表示失败 */
//};
//
//struct LiManuConfig
//{
//    uint32_t version; 						/* 软件版本号 */
//    uint32_t hardware_version; 		/* 硬件版本号 */
//    uint32_t manufacture_date; 		/* 生产日期 */
//    uint32_t manufacture_time; 		/* 生产时间 */
//    uint32_t id1; 								/* 设备 id1 */
//    uint32_t id2; 								/* 设备 id2 */
//    uint32_t id3; 								/* 设备 id3 */
//    uint8_t sn[8]; 								/* sn */
//    uint16_t pitch_angle[4]; 			/* 角度信息 */
//    uint16_t blind_area[2]; 			/* 盲区信息 */
//    uint32_t frequence; 					/* 数据点频 */
//};
//
//extern LidarPointTypedef Pack_Data[12];
//extern LidarPointTypedef Pack_sum;
//extern char rxdatabufer;
//extern uint16_t receive_cnt;
//extern uint8_t confidence;
//extern uint16_t tof_distance,noise,reftof;
//extern uint32_t peak,intg;
//
//void data_process(void);
//
//void CLR_Buf(void);
//uint8_t Hand(char *a);
//void clrStruct(void);
#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_DRIVER_CHASSIS_H
