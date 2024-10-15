//
// Created by 45441 on 2023/10/21.
//

#include "driver_chassis.h"
#include "pid.h"
#include "task_gyro.h"
#include "driver_judge.h"
#include "driver_power.h"
#include "bsp_usart.h"
//关节电机数组，依次为左前，左后，右前，右后
std::array<Joint_Motor,4> joint_motors_array;
//轮电机数组，依次为左，右
std::array<Wheel_Motor,2> wheel_motors_array;
//腿姿态数组，分别为左，右
std::array<Leg_Posture,2> leg_posture_array;
StateVar state_var;
LQR lqr;
ControlInput controlSignal;
float torque_ref_follow;
Ordinary_Least_Squares_t xOls;
//转向环pid
PositionPid yawPid{80,0,10,3.5,-3.5,0.001,NONE,0,0,0,0,0};
//双腿协调pid
PositionPid legDeltaPid{-200,0,0,5,-5,0,NONE,0,0,0,0,0};
//腿长roll轴pid
PositionPid rollPid{0.00005,0,0.00004,0.0001,-0.0001,1.,NONE,0,0,0,0,0};
//roll轴补偿pid
PositionPid rollComPid{0,0,20,35,-35,0.0001,DerivativeFilter|Derivative_On_Measurement,0,0,0,0.05,0};
//腿长控制pid
PositionPid leftLegLengthPid{-1800,-0,-40,400,-550,0.0001,Changing_Integral_Rate|Trapezoid_Integral|DerivativeFilter|Derivative_On_Measurement|Integral_Limit,0.01,0.02,0,0.08,10};
PositionPid rightLegLengthPid{-1800,-0,-40,400,-550,0.0001,Changing_Integral_Rate|Trapezoid_Integral|DerivativeFilter|Derivative_On_Measurement|Integral_Limit,0.01,0.02,0,0.08,10};
float feedforwarRoll;
float loopTime;//积分步长
uint32_t loopCnt;
float target_speed=0;
float target_position=0;
float target_yaw=0;
Motor6020 YawMotor;
float target_leglength=0.18f;
volatile float target_leglength_left=0.18f;
volatile float target_leglength_right=0.18f;
TD_t left_leg_td,right_leg_td;
void ChassisInit(){

    wheel_motors_array[0].radio = 0.75f;//1.28
    wheel_motors_array[1].radio = -0.75f;
    joint_motors_array[0].radio = 0.3;
    joint_motors_array[1].radio = 0.3f;
    joint_motors_array[2].radio = -0.3f;
    joint_motors_array[3].radio = -0.3f;

    lqr.radio[0][0] = 1;
    lqr.radio[0][1] = 1;
    lqr.radio[0][2] = 1;
    lqr.radio[0][3] = 1;
    lqr.radio[0][4] = 1;
    lqr.radio[0][5] = 1;
    lqr.radio[1][0] = 1;
    lqr.radio[1][1] = 1;
    lqr.radio[1][2] = 1;
    lqr.radio[1][3] = 1;
    lqr.radio[1][4] = 1;
    lqr.radio[1][5] = 1;



    target_position= (wheel_motors_array[0].angle-wheel_motors_array[1].angle)/2.f*WHEEL_RADIUS;
    target_yaw = 6020;

    // YawMotor.Motor_Init();

}
uint32_t dlength_cnt;
void Leg_Posture::leg_feedback_update(Joint_Motor *jointMotor1,Joint_Motor *jointMotor2){
    //phi1
    if(jointMotor1->motor_id ==1){
        phi4 = (-jointMotor1->angle) - 30/180.f*PI;//0.349066f;
        phi4_dot = -jointMotor1->velocity;
        //phi4
        phi1 = PI - jointMotor2->angle + 30/180.f*PI;//0.349066f;
        phi1_dot = -jointMotor2->velocity;
    }
    else {
        phi4 = jointMotor1->angle-30/180.f*PI;//0.349066f;
        phi4_dot = jointMotor1->velocity;
        phi1 = PI+jointMotor2->angle+30/180.f*PI;//0.349066f;
        phi1_dot = jointMotor2->velocity;
    }
//正运动学解算，由关节电机反馈值算出腿长和角度
    x_b = l1*arm_cos_f32(phi1);
    y_b = l1*arm_sin_f32(phi1);
    x_d = l5+l4*arm_cos_f32(phi4);
    y_d = l4*arm_sin_f32(phi4);

    A = 2*l2*(x_d-x_b);
    B = 2*l2*(y_d-y_b);
    L_BD = sqrtf((x_b-x_d)*(x_b-x_d)+(y_b-y_d)*(y_b-y_d));
    C = l2*l2+L_BD*L_BD-l3*l3;

    phi2 = 2*atan2f(B+sqrtf(A*A+B*B-C*C),A+C);
    x_c = x_b+l2*arm_cos_f32(phi2);
    y_c = y_b+l2*arm_sin_f32(phi2);

    phi3 = atan2f(y_c-y_d,x_c-x_d);
    length = sqrtf((x_c-l5/2)*(x_c-l5/2)+y_c*y_c);
    //theta
    phi0 = atan2f(y_c,x_c-(l5/2));
    angle = phi0;

    x_d_dot = -phi4_dot*y_d;
    x_b_dot = -phi1_dot*y_b;
    y_d_dot = phi4_dot*(x_d-l5);
    y_b_dot = phi1_dot*x_b;

    phi2_dot = ((x_d_dot-x_b_dot)*arm_cos_f32(phi3)+(y_d_dot-y_b_dot)*arm_sin_f32(phi3))/l2/arm_sin_f32(phi3-phi2);

    phi0_dot = ((x_b*phi1_dot+(x_c-x_b)*phi2_dot)*(x_c-l5/2)+
                (y_b*phi1_dot+(y_c-y_b)*phi2_dot)*y_c)/
               (y_c*y_c+(x_c-l5/2)*(x_c-l5/2));
    //x_dot
    dLength = (l1*arm_sin_f32(phi3)*arm_sin_f32(phi1-phi2))/arm_sin_f32(phi2-phi3)*phi1_dot+
              (l4*arm_sin_f32(phi2)*arm_sin_f32(phi3-phi4)/arm_sin_f32(phi2-phi3))*phi4_dot;
    ddLength = 1000*(dLength-dLength_last);// DWT_GetDeltaT(&dlength_cnt);
    ddLength = ddLength_last*0.9f+ddLength*0.1f;
    dLength_last = dLength;
    ddLength_last = ddLength;
    dAngle = phi0_dot;// phi0_spd(phi1,phi4,phi1_dot,phi4_dot);
    phi0_last = phi0;
}
float x_dot_filter;
uint32_t state_var_cnt=0;
void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度

    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    x_dot_filter = EstimateKF->FilteredValue[0];
//    for (uint8_t i = 0; i < 2; i++)
//    {
//        vel_acc[i] = EstimateKF->FilteredValue[i];
//    }
}
float T_hat[2],Tp_hat[2];

void StateVar::StateVar_Update(){
    if(remoteFlag){
//        theta_left_hat = theta_left * 1.0001f + 0.001f * theta_dot_left;
//        theta_right_hat = theta_right * 1.0001f + 0.001f * theta_dot_right;
        x_dot_left_hat = x_dot_left+(-0.28f*theta_left+(0.0055f)*lqr.outputT[0] -0.0018*lqr.outputTp[0]);
        x_dot_right_hat = x_dot_right+(-0.028f*theta_right+(0.00055)*lqr.outputT[1] -0.0018*lqr.outputTp[1]);
    }else{
//        theta_left_hat = 0;
//        theta_right_hat = 0;
        x_dot_left_hat = 0;
        x_dot_right_hat = 0;
    }

    phi = imu.pitch/180.f*PI;
    phi_dot =imu.pitch_speed;//*0.1f+phi_dot*0.9f;
    theta_left = (float)M_PI_2-leg_posture_array[0].phi0-phi;
    theta_right = (float)M_PI_2-leg_posture_array[1].phi0-phi;

    theta_dot_left = -leg_posture_array[0].dAngle-phi_dot;
    theta_dot_right =  -leg_posture_array[1].dAngle-phi_dot;

    theta_dot =(theta_dot_left+theta_dot_right)/2.f;

    robot_ax =((imu.ay-9.7592433f* arm_sin_f32(phi))*arm_cos_f32(phi)+(imu.az-9.7592433f* arm_cos_f32(phi))* arm_sin_f32(-phi));

    x_left = wheel_motors_array[0].angle*WHEEL_RADIUS+leg_posture_array[0].length* arm_sin_f32(theta_left);
    x_right = -wheel_motors_array[1].angle*WHEEL_RADIUS+leg_posture_array[1].length* arm_sin_f32(theta_right);
   // x = (x_left+x_right)/2.f;
    //车相对于轮的速度+轮相对于地的速度
    x_dot_left = (wheel_motors_array[0].velocity-imu.pitch_speed-leg_posture_array[0].dAngle)*WHEEL_RADIUS;//+leg_posture_array[0].dLength* arm_sin_f32(theta_left)+leg_posture_array[0].length* arm_cos_f32(theta_left)*theta_dot_left;
    //x_dot_left_last = x_dot_left;
    x_dot_right = (-wheel_motors_array[1].velocity-imu.pitch_speed-leg_posture_array[1].dAngle)*WHEEL_RADIUS;//+leg_posture_array[1].dLength* arm_sin_f32(theta_right)+leg_posture_array[1].length* arm_cos_f32(theta_right)*theta_dot_right;
    //x_dot_right_last = x_dot_right;
    x_dot = (x_dot_left+x_dot_right)/2.f;
    kalman_filter_calc(&x_dot_kalman,x_dot,robot_ax);
//    xvEstimateKF_Update(&vaEstimateKF,robot_ax,x_dot);
    //x_dot_filter = x_dot_kalman.filtered_value[0];
    x += x_dot_kalman.filtered_value[0]* DWT_GetDeltaT(&state_var_cnt);
    //竖直方向运动加速度
    z_w_ddot_left=imu.az* arm_cos_f32(state_var.phi)+imu.ay* arm_sin_f32(state_var.phi)-g - leg_posture_array[0].ddLength* arm_cos_f32(theta_left);//+2*leg_posture_array[0].dLength*theta_dot_left*arm_sin_f32(theta_left)+leg_posture_array[0].length*theta_dot_left*theta_dot_left*arm_cos_f32(theta_left);
    z_w_ddot_right=imu.az* arm_cos_f32(state_var.phi)+imu.ay* arm_sin_f32(state_var.phi) -g- leg_posture_array[1].ddLength* arm_cos_f32(theta_right);//+2*leg_posture_array[1].dLength*theta_dot_right*arm_sin_f32(theta_right)+leg_posture_array[1].length*theta_dot_right*theta_dot_right*arm_cos_f32(theta_right);
    legLength = (leg_posture_array[0].length+leg_posture_array[1].length)/2.f;
    theta_dot_left_last = theta_dot_left;
    theta_dot_right_last = theta_dot_right;

    if(remoteFlag){
        theta_left_hat[0] = 1.0001f*theta_left + 0.001f*theta_dot_left;
        theta_right_hat[0] = 1.0001f*theta_right + 0.001f*theta_dot_right;
        theta_left_dot_hat[0] = 0.1928f * theta_left + 1.0001f*theta_dot_left + 0.0001f*phi - 0.0312f*lqr.outputT[0] + 0.0127f*lqr.outputTp[0];
        theta_right_dot_hat[0] = 0.1928f * theta_right + 1.0001f*theta_dot_right + 0.0001f*phi - 0.0312f*lqr.outputT[1] + 0.0127f* lqr.outputTp[1];
        x_left_hat[0] =  x_left + 0.001f*x_dot_left;
        x_right_hat[0] =  x_right + 0.001f*x_dot_right;
        x_left_dot_hat[0] = -0.0283f*theta_left + x_dot_left+0.0055f*lqr.outputT[0] -0.0018f*lqr.outputTp[0];
        x_right_dot_hat[0] = -0.0283f*theta_right + x_dot_right+0.0055f*lqr.outputT[1] -0.0018f*lqr.outputTp[1];
        phi_hat[0] = phi + 0.001f*phi_dot;
        phi_dot_hat[0] = 0.0015f*theta_left + 0.0023f*phi + phi_dot + 0.0028f*lqr.outputTp[0];
        for(int i=1;i<3;i++){
            T_hat[0] = -(-28.2554f*theta_left_hat[i-1] - 4.121f*theta_left_dot_hat[i-1] - 10.7098f*x_left_hat[i-1]- 11.5265f*x_left_dot_hat[i-1] + 43.5147f*phi_hat[i-1] +5.94566f*phi_dot_hat[i-1]);
            T_hat[1] = -(-28.2554f*theta_right_hat[i-1] - 4.121f*theta_right_dot_hat[i-1] - 10.7098f*x_right_hat[i-1]- 11.5265f*x_right_dot_hat[i-1] + 43.5147f*phi_hat[i-1] +5.94566f*phi_dot_hat[i-1]);
            Tp_hat[0] = -(46.5749f*theta_left_hat[i-1] + 7.9748f*theta_left_dot_hat[i-1] + 30.6327f*x_left_hat[i-1] + 29.3272f*x_left_dot_hat[i-1] + 90.0892f*phi_hat[i-1] +2.8101f*phi_dot_hat[i-1]);
            Tp_hat[1] = -(46.5749f*theta_right_hat[i-1] + 7.9748f*theta_right_dot_hat[i-1] + 30.6327f*x_right_hat[i-1] + 29.3272f*x_right_dot_hat[i-1] + 90.0892f*phi_hat[i-1] +2.8101f*phi_dot_hat[i-1]);
            theta_left_hat[i] = 1.0001f*theta_left_hat[i-1] + 0.001f*theta_left_dot_hat[i-1] ;
            theta_right_hat[i] = 1.0001f*theta_right_hat[i-1] + 0.001f*theta_right_dot_hat[i-1] ;
            theta_left_dot_hat[i] = 0.1928f * theta_left_hat[i-1] + 1.0001f*theta_left_dot_hat[i-1] + 0.0001f*phi_hat[i-1] - 0.0312f*T_hat[0] + 0.0127f*Tp_hat[0];
            theta_right_dot_hat[i] = 0.1928f * theta_right_hat[i-1] + 1.0001f*theta_right_dot_hat[i-1] + 0.0001f*phi_hat[i-1] - 0.0312f*T_hat[1] + 0.0127f* Tp_hat[1];
            x_left_hat[i] =  x_left_hat[i-1] + 0.001f*x_left_dot_hat[i-1];
            x_right_hat[i] = x_right_hat[i-1] + 0.001f*x_right_dot_hat[i-1];
            x_left_dot_hat[i] = -0.0283f*theta_left_hat[i-1] + x_left_dot_hat[i-1]+0.0055f*T_hat[0] - 0.0018f*Tp_hat[0];
            x_right_dot_hat[i] = -0.0283f*theta_right_hat[i-1] + x_right_dot_hat[i-1]+0.0055f*T_hat[1] - 0.0018f*Tp_hat[1];
            phi_hat[i] = phi_hat[i-1] + 0.001f*phi_dot_hat[i-1];
            phi_dot_hat[i] = 0.0015f*theta_left_hat[i-1] + 0.0023f*phi_hat[i-1] + phi_dot_hat[i-1] + 0.0028f*Tp_hat[0];
        }
    }else{
        theta_left_hat[4] = theta_left;
        theta_right_hat[4] = theta_right;
        theta_left_dot_hat[4] = theta_dot_left;
        theta_right_dot_hat[4] = theta_dot_right;
        x_left_hat[4] = x_left;
        x_right_hat[4] = x_right;
        x_left_dot_hat[4] = x_dot_left;
        x_right_dot_hat[4] = x_dot_right;
        phi_hat[4] = phi;
        phi_dot_hat[4] = phi_dot;
    }
//    x_dot_left_hat = x_left_dot_hat[4];
//    x_dot_right_hat = x_right_dot_hat[4];
}
uint8_t zig_flag=0;
float yaw_temp_imu;
uint8_t spin_inverse = 0;
void chassisFollowCalculate(uint8_t follow_flag,uint8_t axial_flag){

    if(leg_posture_array[0].isGround&&leg_posture_array[1].isGround) {
        if (follow_flag) {
            if (axial_flag) {
                target_yaw = 0.824f-0.25;
                yaw_temp_imu = imu.yaw;
            } else {
                if (YawMotor.angle > 0.824f - 0.25 || YawMotor.angle < 0.824f + 0.25 - 1) {
                    target_yaw = 0.824f;
                } else {
                    target_yaw = 0.824f - 0.5;
                }
                yaw_temp_imu = imu.yaw;
            }
            if(jumpFlagTemp){
                target_yaw = 0.824f;
            }
            float delta_yaw = -target_yaw + YawMotor.angle;
            if(controlSignal.controlFlag.visionFlag !=0){
                target_yaw = yaw_temp_imu;
                delta_yaw = -target_yaw + imu.yaw;
            }
//            target_yaw -=controlSignal.yaw/4.f;
//            if(target_yaw>180){
//                target_yaw = target_yaw-360;
//            }else if(target_yaw<-180){
//                target_yaw = target_yaw+360;
//            }
//            float delta_yaw = target_yaw - imu.yaw;
//            if(delta_yaw>180){
//                delta_yaw = delta_yaw-360.f;
//            }else if(delta_yaw<-180){
//                delta_yaw = delta_yaw+360.f;
//            }
//            torque_ref_follow = -yawPid.PidCalculate(0,delta_yaw/360.f);

            if (delta_yaw > 0.5) {
                delta_yaw = delta_yaw - 1;
            } else if (delta_yaw < -0.5) {
                delta_yaw = 1 + delta_yaw;
            }
            if(fabsf(delta_yaw)>0.2){
                zig_flag = 1;
            }else{
                zig_flag = 0;
            }
            torque_ref_follow = yawPid.PidCalculate(delta_yaw, 0);
        } else {
            if(spin_inverse == 1){
                torque_ref_follow = 6.5f;
            }else{
                torque_ref_follow = -6.5f;

            }
        }
    }else{
        torque_ref_follow = 0;
    }

}
uint8_t mid=0;
uint8_t up=0;
uint8_t down=0;
uint8_t jump_finish=1;
uint8_t jumping;
float ground_theta;
float tar_length;
float tar_length_temp;
float tar_roll=0;
float k_leg=0.5f;
void legLengthCalculate(uint8_t jump_flag){

    if((!jump_flag)&&(jump_finish)) {
        leftLegLengthPid.Kp = -1800;
        leftLegLengthPid.Kd = -40;
        rightLegLengthPid.Kp = -1800;
        rightLegLengthPid.Kd = -40;
        leftLegLengthPid.outputMin = -60;
        rightLegLengthPid.outputMin = -60;
        leftLegLengthPid.outputMax = 60;
        rightLegLengthPid.outputMax = 60;

        mid = 0;
        up = 0;
        down = 0;
        jumping = 0;
        if(leg_posture_array[0].isGround==DISABLE&&leg_posture_array[1].isGround==DISABLE){
            return;
        }else {

//        if(controlSignal.controlFlag.rollFlag){
//            target_leglength = 0.18f;
//            tar_roll = controlSignal.legLength*0.5;
//        }else{
            if (controlSignal.controlFlag.legLengthFlag == 0) {
                target_leglength = 0.1f;
            } else if (controlSignal.controlFlag.legLengthFlag == 1) {
                target_leglength = 0.18f;
            } else {
                target_leglength = 0.32f;
            }
//            target_leglength -=controlSignal.legLength/1500.f;
            tar_roll = 0;
//        }
//            if(target_leglength == 0.18f){
//                if(fabsf(torque_ref_follow)>3&&fabsf(target_speed)>2){
//                    target_leglength = 0.15f;
//                }else{
//                    target_leglength = 0.18f;
//                }
//            }
            //计算斜坡角度
            ground_theta = atan2f(leg_posture_array[0].length -
                                  (leg_posture_array[1].length - 0.5f * arm_sin_f32(imu.roll / 180.f * PI - tar_roll)),
                                  0.5f *arm_cos_f32(imu.roll / 180.f * PI - tar_roll)) / PI * 180.f;
            if (fabsf(ground_theta) < 3) {
                ground_theta = 0;
            }
            LIMIT_MIN_MAX(target_leglength, LEG_LENGTH_MIN, LEG_LENGTH_MAX);
            target_leglength_left = target_leglength + k_leg * arm_sin_f32(ground_theta / 180.f * PI) / 2.f;
            target_leglength_right = target_leglength - k_leg * arm_sin_f32(ground_theta / 180.f * PI) / 2.f;
            LIMIT_MIN_MAX(target_leglength_left, LEG_LENGTH_MIN, LEG_LENGTH_MAX);
            LIMIT_MIN_MAX(target_leglength_right, LEG_LENGTH_MIN, LEG_LENGTH_MAX);
            if(fabsf(target_leglength_left-target_leglength_right)<0.02f){
                target_leglength_right = target_leglength_left;
            }

        }
   }else{
        //收腿
        jumping=1;
//        leftLegLengthPid.Kp = -5000;
//        rightLegLengthPid.Kp = -5000;
//        leftLegLengthPid.outputMin = -450;
//        rightLegLengthPid.outputMin = -450;
//        leftLegLengthPid.outputMax = 600;
//        rightLegLengthPid.outputMax = 600;
//        if(fabsf(state_var.phi)>0.2f){
//            target_leglength_left = target_leglength_right = 0.1f;
//        }
        leftLegLengthPid.Kp = -2500;
        rightLegLengthPid.Kp = -2500;
        leftLegLengthPid.outputMin = -450;
        rightLegLengthPid.outputMin = -450;
        leftLegLengthPid.outputMax = 600;
        rightLegLengthPid.outputMax = 600;
//        if(mid == 0){
//            target_leglength_left = 0.12f;
//            target_leglength_right = 0.12f;
//        }
//        if((mid == 0)&&(fabsf(leg_posture_array[0].length-0.12f)<0.01f)&&(fabsf(leg_posture_array[1].length-0.12f)<0.01f)){
//            mid=1;
//        }
        mid = 1;
        //伸腿
        if(mid==1){
            leftLegLengthPid.Kp = -7000;
            leftLegLengthPid.Kd = 0;//-20;
            rightLegLengthPid.Kp = -7000;
            rightLegLengthPid.Kd = 0;//-20;

            target_leglength_left = 0.35f;
            target_leglength_right = 0.35f;
        }
        if((up==0)&&(fabsf(leg_posture_array[0].length-0.35f)<0.01f)&&(fabsf(leg_posture_array[1].length-0.35f)<0.01f)){
            up=1;
        }
        //收腿
        if(up==1){
            leftLegLengthPid.Kp = -8000;
            leftLegLengthPid.Kd = 0;
            rightLegLengthPid.Kp = -8000;
            rightLegLengthPid.Kd = 0;
            leftLegLengthPid.outputMin = -450;
            rightLegLengthPid.outputMin = -450;
            leftLegLengthPid.outputMax = 500;
            rightLegLengthPid.outputMax = 500;
            target_leglength_left = 0.13f;
            target_leglength_right = 0.13f;
            target_leglength = 0.18f;
        }
        if((up==1)&&(fabsf(leg_posture_array[0].length-0.13f)<0.01f)&&(fabsf(leg_posture_array[1].length-0.13f)<0.01f)){
            down=1;
            jump_finish=1;
            controlSignal.controlFlag.jumpFlag = 0;
        }

    }

}
float k_feed=0.6f;
void forceLegCalculate(){
    //ROLL轴前馈

    if(controlSignal.controlFlag.spinFlag){
        feedforwarRoll =0;//(robot_m/2.f+1.5f*Mp)*state_var.legLength/2/WHEEL_RADIUS*state_var.x_dot*imu.yaw_speed;
    }else{
        feedforwarRoll =(k_feed*robot_m/1.6f+1.5f*Mp)*state_var.legLength/2/WHEEL_RADIUS*state_var.x_dot*imu.yaw_speed;
    }

    if(leg_posture_array[0].isGround == DISABLE && leg_posture_array[1].isGround==DISABLE){
        leg_posture_array[0].Force = leftLegLengthPid.PidCalculate(leg_posture_array[0].length,0.25f);//-(robot_m/2+Mp)*g;
        leg_posture_array[1].Force = rightLegLengthPid.PidCalculate(leg_posture_array[1].length,0.25f);//-(robot_m/2+Mp)*g;
    }else{
        leg_posture_array[0].Force = -feedforwarRoll+rollComPid.PidCalculate(imu.roll/180.f*PI,tar_roll)+leftLegLengthPid.PidCalculate(leg_posture_array[0].length,target_leglength_left)-(robot_m/2+Mp)*g;
                                     M/2.f*(g*arm_cos_f32(state_var.theta_left)-(leg_posture_array[0].length)*state_var.theta_dot_left*state_var.theta_dot_left
                                            -l*(state_var.phi_dot*state_var.phi_dot*(arm_cos_f32(state_var.theta_left)*arm_cos_f32(state_var.phi)-arm_sin_f32(state_var.theta_left)*arm_sin_f32(state_var.phi))))-
                                     Mp*(g*arm_cos_f32(state_var.theta_left)-leg_posture_array[0].length/2*state_var.theta_dot_left*state_var.theta_dot_left);
        leg_posture_array[1].Force = feedforwarRoll-rollComPid.PidCalculate(imu.roll/180.f*PI,tar_roll)+rightLegLengthPid.PidCalculate(leg_posture_array[1].length,target_leglength_right)-(robot_m/2+Mp)*g;
                                     M/2.f*(g*arm_cos_f32(state_var.theta_right)-(leg_posture_array[1].length)*state_var.theta_dot_right*state_var.theta_dot_right
                                            -l*(state_var.phi_dot*state_var.phi_dot*(arm_cos_f32(state_var.theta_right)*arm_cos_f32(state_var.phi)-arm_sin_f32(state_var.theta_right)*arm_sin_f32(state_var.phi))))-
                                     Mp*(g*arm_cos_f32(state_var.theta_right)-leg_posture_array[1].length/2*state_var.theta_dot_right*state_var.theta_dot_right);
    }

}
void Leg_Posture::leg_vmc_calculate_div(Joint_Motor motor1,Joint_Motor motor2){
    float sig1=l4* arm_cos_f32(phi0-phi2)* arm_sin_f32(phi0-phi3)* arm_sin_f32(phi3-phi4)-
               l4* arm_cos_f32(phi0-phi3)*arm_sin_f32(phi0-phi2)*arm_sin_f32(phi3-phi4);
    float sig2=l1* arm_cos_f32(phi0-phi2)* arm_sin_f32(phi0-phi3)* arm_sin_f32(phi1-phi2)-
               l1* arm_cos_f32(phi0-phi3)*arm_sin_f32(phi0-phi2)*arm_sin_f32(phi1-phi2);

    Force_div = (motor2.current* arm_cos_f32(phi0-phi2)* arm_sin_f32(phi3-phi2)/sig2+
                 motor1.current* arm_cos_f32(phi0-phi3)*arm_sin_f32(phi2-phi3)/sig1);
    Tp_div = (motor2.current* length*arm_sin_f32(phi0-phi2)* arm_sin_f32(phi2-phi3)/sig2-
              motor1.current*length* arm_sin_f32(phi0-phi3)*arm_sin_f32(phi2-phi3)/sig1);

}
uint16_t left_leg_count = 0;
uint16_t right_leg_count = 0;
void ifGroundDetect(){

    //计算地面支持力
    leg_posture_array[0].leg_vmc_calculate_div(joint_motors_array[0],joint_motors_array[1]);
    leg_posture_array[1].leg_vmc_calculate_div(joint_motors_array[2],joint_motors_array[3]);
    leg_posture_array[0].supportForce = 4.5f*(-leg_posture_array[0].Force_div* arm_cos_f32(state_var.theta_left)-
                                              leg_posture_array[0].Tp_div*arm_sin_f32(state_var.theta_left)/leg_posture_array[0].length)+Mp*g+Mp*state_var.z_w_ddot_left;
    leg_posture_array[1].supportForce = 4.5f*(+leg_posture_array[1].Force_div* arm_cos_f32(state_var.theta_right)+
                                              leg_posture_array[1].Tp_div*arm_sin_f32(state_var.theta_right)/leg_posture_array[1].length)+Mp*g+Mp*state_var.z_w_ddot_right;
    //离地检测
    if((leg_posture_array[0].supportForce<=20)){
        left_leg_count++;
    }else{
        left_leg_count = 0;
    }


    if((leg_posture_array[1].supportForce<=20)){
        right_leg_count++;
    }else{
        right_leg_count = 0;
    }
    if(osKernelGetTickCount()<3000){
        if(left_leg_count>=1){
            leg_posture_array[0].isGround = DISABLE;
        }else{
            leg_posture_array[0].isGround = ENABLE;
        }
        if(right_leg_count>=1){
            leg_posture_array[1].isGround = DISABLE;
        }else{
            leg_posture_array[1].isGround = ENABLE;
        }
    }else{
        if(left_leg_count>=500){
            leg_posture_array[0].isGround = DISABLE;
        }else{
            leg_posture_array[0].isGround = ENABLE;
        }
        if(right_leg_count>=500){
            leg_posture_array[1].isGround = DISABLE;
        }else{
            leg_posture_array[1].isGround = ENABLE;
        }
    }


}

void LQR::K_Calculate(float leg_length,bool ground) {
    legLength = leg_length;
    legLength2 = leg_length * legLength;
    legLength3 = legLength * legLength * legLength;
    if (ground == ENABLE ) {
        //T
        K[0][0] = radio[0][0]*(174.196*legLength3 - 45.0022*legLength2 - 153.416*legLength - 1.93323);
        K[0][1] = radio[0][1]*(79.4466*legLength3 - 77.1504*legLength2 - 10.7699*legLength - 0.47805);
        K[0][2] = radio[0][2]*(- 47.9022*legLength3 + 101.178*legLength2 - 64.334*legLength - 1.18417);
        K[0][3] = radio[0][3]*(8.34459*legLength3 + 55.485*legLength2 - 59.6771*legLength - 3.06313);
        K[0][4] = radio[0][4]*(303.988*legLength3 - 150.597*legLength2 - 58.3557*legLength + 58.8656);
        K[0][5] = radio[0][5]*(- 23.1748*legLength3 + 27.5383*legLength2 - 17.8431*legLength + 8.68258);

        //Tp
        K[1][0] = radio[1][0]*(1230.07*legLength3 - 1238.53*legLength2 + 434.845*legLength + 8.96923);
        K[1][1] = radio[1][1]*(74.6601*legLength3 - 106.967*legLength2 + 63.0749*legLength + 1.08734);
        if(K[1][1]>8.5){
            K[1][1] = 8.5;
        }
        K[1][2] = radio[1][2]*(260.076*legLength3 - 139.298*legLength2 - 34.6913*legLength + 41.005);
        K[1][3] = radio[1][3]*(228.532*legLength3 - 130.515*legLength2 - 28.3412*legLength + 42.5488);
        K[1][4] = radio[1][4]*(646.152*legLength3 - 1020.13*legLength2 + 567.798*legLength + 11.5812);
        K[1][5] = radio[1][5]*( 161.629*legLength3 - 199.049*legLength2 + 95.2849*legLength - 9.98302);
    } else {

        K[0][0] = 0;
        K[0][1] = 0;
        K[0][2] = radio[0][2]*(- 47.9022*legLength3 + 101.178*legLength2 - 64.334*legLength - 1.18417);
        K[0][3] = radio[0][3]*(8.34459*legLength3 + 55.485*legLength2 - 59.6771*legLength - 3.06313);
        K[0][4] = 0;
        K[0][5] = 0;

        K[1][0] = radio[1][0]*(1230.07*legLength3 - 1238.53*legLength2 + 434.845*legLength + 8.96923);
        K[1][1] = radio[1][1]*(74.6601*legLength3 - 106.967*legLength2 + 63.0749*legLength + 1.08734);
        K[1][2] = 0;
        K[1][3] = 0;
        K[1][4] = 0;
        K[1][5] = 0;
    }
}
float tar_theta_left=0;
float tar_theta_right=0;
float K_measure=0.35;
float speed_max;
float Ky[4][10];
float tar_spd;
float tar_yaw= -0.17f;
void tpLegCalculate(){
    //修改平衡点
    loopTime= DWT_GetDeltaT(&loopCnt);
    speed_max = K_measure * sqrtf(robot_power_limit);
    if (controlSignal.controlFlag.superCFlag){
        LIMIT_MIN_MAX(speed_max,3.3f,3.3f);
    }else{
        LIMIT_MIN_MAX(speed_max,2.5f,2.5f);
    }
    if(jumpFlagTemp) {
        LIMIT_MIN_MAX(speed_max, 1.7f, 1.7f);
    }
    if(controlSignal.controlFlag.spinFlag){
        if(target_yaw == 0.824f){
            target_speed = -sqrtf(powf(controlSignal.speed,2)+ powf(controlSignal.legLength,2))*speed_max* arm_cos_f32((YawMotor.angle-target_yaw-0.2f+0.17f)*2*PI-
                                                                                                                       atan2f(-controlSignal.speed,controlSignal.legLength));
        }else{
            target_speed = sqrtf(powf(controlSignal.speed,2)+ powf(controlSignal.legLength,2))*speed_max* arm_cos_f32((YawMotor.angle-target_yaw-0.2f+0.17f)*2*PI-
                                                                                                                       atan2f(-controlSignal.speed,controlSignal.legLength));
        }
        if(spin_inverse == 0){
            if(target_yaw == 0.824f){
                target_speed =  -sqrtf(powf(controlSignal.speed,2)+ powf(controlSignal.legLength,2))*speed_max* arm_cos_f32((YawMotor.angle-target_yaw-0.2f-0.17f)*2*PI-
                                                                                                                            atan2f(-controlSignal.speed,controlSignal.legLength));
            }else{
                target_speed = sqrtf(powf(controlSignal.speed,2)+ powf(controlSignal.legLength,2))*speed_max* arm_cos_f32((YawMotor.angle-target_yaw-0.2f-0.17f)*2*PI-
                                                                                                                          atan2f(-controlSignal.speed,controlSignal.legLength))
                                                                                                                                 ;
            }
        }
    }else if(controlSignal.controlFlag.axialFlag){
        if(zig_flag){
            target_speed = 0;
        }else{
            target_speed = -controlSignal.legLength*speed_max;
        }
    }else{
        if(zig_flag){
            target_speed=0;
        }else{
            if(target_yaw == 0.824f){
                target_speed = controlSignal.speed * speed_max;
            }else{
                target_speed = -controlSignal.speed * speed_max;
            }
        }
    }
    tar_theta_left = 0;
    tar_theta_right = 0;

    target_position+=(state_var.x_dot-target_speed)*loopTime;

    float x[2][6]={state_var.theta_left-tar_theta_left,state_var.theta_dot_left,state_var.x,x_dot_kalman.filtered_value[0],state_var.phi,state_var.phi_dot,
                   state_var.theta_right-tar_theta_right,state_var.theta_dot_right,state_var.x,x_dot_kalman.filtered_value[0],state_var.phi,state_var.phi_dot};
    float y[10] = {state_var.x,state_var.x_dot,0,0,state_var.theta_left,state_var.theta_dot_left,state_var.theta_right,state_var.theta_dot_right,state_var.phi,state_var.phi_dot};
    LIMIT_MIN_MAX(target_position,-3.3f,3.3f);


    if(leg_posture_array[0].isGround==DISABLE&&leg_posture_array[1].isGround==DISABLE){
        // target_speed = state_var.x_dot;
        target_position = 0;
    }

        x[0][2] = target_position;
        x[1][2] = target_position;


    float legDeltaOutput= legDeltaPid.PidCalculate(state_var.theta_left,state_var.theta_right);
    //计算LQR矩阵增益 u=-Kx
    lqr.K_Calculate(leg_posture_array[0].length,leg_posture_array[0].isGround);
    lqr.outputT[0] = -(x[0][0] * lqr.K[0][0]+x[0][1] * lqr.K[0][1] + x[0][2]*lqr.K[0][2] + x[0][3]*lqr.K[0][3]+x[0][4]*lqr.K[0][4]+x[0][5]*lqr.K[0][5]);
    lqr.outputTp[0] = -(x[0][0] * lqr.K[1][0]+x[0][1] * lqr.K[1][1] + x[0][2]*lqr.K[1][2] + x[0][3]*lqr.K[1][3]+x[0][4]*lqr.K[1][4]+x[0][5]*lqr.K[1][5]);
    lqr.K_Calculate(leg_posture_array[1].length,leg_posture_array[1].isGround);
    lqr.outputT[1] = -(x[1][0] * lqr.K[0][0]+x[1][1] * lqr.K[0][1] + x[1][2]*lqr.K[0][2] + x[1][3]*lqr.K[0][3]+x[1][4]*lqr.K[0][4]+x[1][5]*lqr.K[0][5]);
    lqr.outputTp[1] = -(x[1][0] * lqr.K[1][0]+x[1][1] * lqr.K[1][1] + x[1][2]*lqr.K[1][2] + x[1][3]*lqr.K[1][3]+x[1][4]*lqr.K[1][4]+x[1][5]*lqr.K[1][5]);

    //双腿Tp赋值
    leg_posture_array[0].Tp = lqr.outputTp[0]-legDeltaOutput;
    leg_posture_array[1].Tp = lqr.outputTp[1]+legDeltaOutput;
}
//由F,T_p解算出关节电机力矩 VMC
void Leg_Posture::leg_vmc_calculate(){
    T[1] = l1*arm_sin_f32(phi0-phi3)*arm_sin_f32(phi1-phi2)/arm_sin_f32(phi3-phi2)*Force+
           l1*arm_cos_f32(phi0-phi3)*arm_sin_f32(phi1-phi2)/length/arm_sin_f32(phi3-phi2)*Tp;
    T[0] = l4*arm_sin_f32(phi0-phi2)*arm_sin_f32(phi3-phi4)/arm_sin_f32(phi3-phi2)*Force+
           l4*arm_cos_f32(phi0-phi2)*arm_sin_f32(phi3-phi4)/length/arm_sin_f32(phi3-phi2)*Tp;
}
//关节电机扭矩设定
void Joint_Motor::torque_set() {
    torque_ref = radio * leg_posture_array[motor_id/3].T[(motor_id-1)%2];
}
//轮电机扭矩设定
float k_adp=0;
float yawspeed;
void Wheel_Motor::torque_set() {
    yawspeed = imu.yaw_speed;
    if(motor_id==5) {
        if (leg_posture_array[0].isGround == DISABLE||leg_posture_array[1].isGround == DISABLE) {
            torque_ref = radio * (lqr.outputT[0]);
        } else {
            if (fabsf(state_var.x_dot_left_hat - (state_var.x_dot_left - yawspeed * 0.26f)) < 0.2 ) {
                torque_ref = radio * (lqr.outputT[0]) - torque_ref_follow;
            } else {
                torque_ref = radio * (lqr.outputT[0]) - torque_ref_follow -
                             k_adp * (state_var.x_dot_left_hat - (state_var.x_dot_left ));
            }
        }
    }
    if(motor_id==6) {
        if (leg_posture_array[1].isGround == DISABLE||leg_posture_array[0].isGround == DISABLE) {
            torque_ref = radio * (lqr.outputT[1]);
        } else {
            if (fabsf(state_var.x_dot_right_hat - (state_var.x_dot_right + yawspeed * 0.26f)) < 0.2 ) {
                torque_ref = radio * (lqr.outputT[1]) - torque_ref_follow;
            } else {
                torque_ref = radio * (lqr.outputT[1]) - torque_ref_follow +
                             k_adp * (state_var.x_dot_right_hat - (state_var.x_dot_right ));
            }
        }
    }
}

