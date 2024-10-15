//
// Created by 45441 on 2023/10/21.
//

#include "driver_chassis.h"
#include "user_lib.h"
#include "driver_gyro.h"
#include "bsp_usart.h"
uint8_t ChassisInitFlag=1;
StandupState standupState;
float task_chassis_frequency;
uint32_t task_chassis_cnt;
uint8_t motor_OK=0;
uint8_t motor_error=0;
kalman_filter_t x_dot_kalman={0};
kalman_filter_t x_dot_right_kalman={0};
kalman_filter_init_t x_dot_kalman_filter_para = {
        .xhat_data = {0, 0},
        .P_data = {1, 0, 0, 1},
        .A_data = {1, 0.001f, 0, 1},
        .H_data = {1, 0, 0, 1},
        .Q_data = {0.55, 0, 0, 0.1},
        .R_data = {1500, 0, 0, 1000}
};
kalman_filter_init_t x_dot_right_kalman_filter_para = {
        .xhat_data = {0, 0},
        .P_data = {1, 0, 0, 1},
        .A_data = {1, 0.001f, 0, 1},
        .H_data = {1, 0, 0, 1},
        .Q_data = {1, 0, 0, 1},
        .R_data = {3000, 0, 0, 50}
};


float robot_m;

void Chassis_Task(void *argument)
{
    /* USER CODE BEGIN Chassis_Task */
    //初始化设置关节电机ID
    for(int i =0;i<joint_motors_array.size();++i){
        joint_motors_array[i].motor_id = i+1;
    }
    //初始化设置轮电机ID
    for(int i=0;i<wheel_motors_array.size();++i){
        wheel_motors_array[i].motor_id = i+5;
    }
    osDelay(50);
//    //检测关节电机是否启动，初始化，校准 电机零位
    while(joint_motors_array[0].detect==0||joint_motors_array[1].detect==0||
          joint_motors_array[2].detect==0||joint_motors_array[3].detect==0) {
        for (auto &joint_motor: joint_motors_array) {
            joint_motor.Motor_Init();
            osDelay(1);
        }
    }
    for(int i=0;i<300;i++){
        for (auto &joint_motor: joint_motors_array){
            joint_motor.RollBack_Init();
            osDelay(1);
        }
    }
    for (auto &joint_motor: joint_motors_array) {
        joint_motor.ZeroPosition();
        osDelay(1);
    }
    //  校准轮电机是否启动，初始化，校准
    while(wheel_motors_array[0].detect==0||wheel_motors_array[1].detect==0){
        for(auto &wheel_motor:wheel_motors_array){
            wheel_motor.Motor_Init();
            osDelay(1);
        }
    }
    osDelay(30);

    ChassisInit();
    mat_init(&x_dot_kalman .Q,2,2, x_dot_kalman_filter_para.Q_data);
    mat_init(&x_dot_kalman.R,2,2, x_dot_kalman_filter_para.R_data);
    kalman_filter_init(&x_dot_kalman, &x_dot_kalman_filter_para);


    /* Infinite loop */
    for(;;)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Uart1_Receive_buf,sizeof(Uart1_Receive_buf));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        task_chassis_frequency = 1.f/DWT_GetDeltaT(&task_chassis_cnt);
        if(!remoteFlag){
            target_position = 0;
            target_speed = 0;
            target_leglength = 0.18f;
            target_leglength_left = target_leglength;
            target_leglength_right = target_leglength;
            controlSignal.controlFlag.jumpFlag = 0;
            target_yaw =imu.yaw;
            state_var.x_dot_left_hat = 0;
            state_var.x_dot_right_hat = 0;

        }
        motor_OK = 1;
        //双腿位置、速度、加速度更新
        for(int i=0;i<leg_posture_array.size();i++){
            leg_posture_array[i].leg_feedback_update(&joint_motors_array[2*i],&joint_motors_array[2*i+1]);
        }
        robot_m = M;
        //状态空间矩阵更新
        state_var.StateVar_Update();
        //转向环
        chassisFollowCalculate(!controlSignal.controlFlag.spinFlag,controlSignal.controlFlag.axialFlag);
        //腿长计算
        legLengthCalculate(controlSignal.controlFlag.jumpFlag);
        //腿部F计算
        forceLegCalculate();
        //离地检测
        ifGroundDetect();
        //计算髋关节力矩
        tpLegCalculate();
        //VMC解算获得关节力矩
        for(auto &leg_posture:leg_posture_array){
            leg_posture.leg_vmc_calculate();
        }

        //关节电机扭矩设定
        for(auto &joint_motor:joint_motors_array){
            joint_motor.torque_set();
        }

        //轮电机扭矩设定
        for(auto &wheel_motor:wheel_motors_array){
            wheel_motor.torque_set();
        }
        for (auto &joint_motor: joint_motors_array) {
            joint_motor.lost_counter_count_number++;
        }
        for(auto &wheel_motor:wheel_motors_array){
            wheel_motor.lost_counter_count_number++;
        }
        if(joint_motors_array[0].lost_counter_count_number>=40||joint_motors_array[1].lost_counter_count_number>=40||
           joint_motors_array[2].lost_counter_count_number>=40||joint_motors_array[3].lost_counter_count_number>=40||
           wheel_motors_array[0].lost_counter_count_number>=40||wheel_motors_array[1].lost_counter_count_number>=40){
            motor_error=1;
        }else{
            motor_error=0;
        }
        osDelay(1);
    }
    /* USER CODE END Chassis_Task */
}