//
// Created by 45441 on 2024/1/21.
//
#include "driver_power.h"
#include "driver_judge.h"
#include "pid.h"

PositionPid powerPID{3,0,0,150,-180,0.1,NONE,0,0,0,0,0};
 float capPower;
uint16_t robot_power_limit ;
float task_cap_frequency;
uint32_t task_cap_cnt;
float power_f;
float power_pid;
void PowerTask(void *argument)
{
    /* USER CODE BEGIN PowerTask */
    /* Infinite loop */
    for(;;)
    {
        powerPID.Kp = 10;
        powerPID.outputMax=1;
        powerPID.outputMin = -1;
        task_cap_frequency = 1.f/DWT_GetDeltaT(&task_cap_cnt);
        robot_power_limit = judge_rece_mesg.robot_status_data.chassis_power_limit;
        if(robot_power_limit<45){
            robot_power_limit = 45;
        }

        power_f = (float)robot_power_limit - superCap.power_fb.p_wheel;

        //电容放电时
        if(power_f<=0){
            if(judge_rece_mesg.power_heat_data.chassis_power_buffer>40){
                power_pid=-100*powerPID.PidCalculate((float)judge_rece_mesg.power_heat_data.chassis_power_buffer/POWER_BUFFER_MAX,58.f/POWER_BUFFER_MAX);
            }else{
                power_pid=-150*powerPID.PidCalculate((float)judge_rece_mesg.power_heat_data.chassis_power_buffer/POWER_BUFFER_MAX,58.f/POWER_BUFFER_MAX);
            }
            if(power_pid>0){
                power_pid = 0;
            }
            capPower = power_pid + 0.9f*power_f;
        }else{
            power_pid = -150*powerPID.PidCalculate((float)judge_rece_mesg.power_heat_data.chassis_power_buffer/POWER_BUFFER_MAX,45.f/POWER_BUFFER_MAX);
            capPower = power_pid + power_f;
        }
        if(jumpFlagTemp){
            capPower = -40;
        }
        if(capPower > (float)robot_power_limit + 20){
            capPower = (float)robot_power_limit + 20;
        }else if(capPower < -superCap.power_fb.p_wheel - 40){
            capPower = -superCap.power_fb.p_wheel - 40;
        }
        //capPower = powerPID.PidCalculate(judge_rece_mesg.power_heat_data.chassis_power+(float)(60-judge_rece_mesg.power_heat_data.chassis_power_buffer),judge_rece_mesg.robot_status_data.chassis_power_limit);
        osDelay(1);

    }
    /* USER CODE END PowerTask */
}