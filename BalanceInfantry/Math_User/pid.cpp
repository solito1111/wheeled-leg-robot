/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"
#include "arm_math.h"
#include "bsp_dwt.h"
void PID::trapezoidIntergral() {
    Iterm = Ki*(err+errLast)/2*dt;
}
void PID::changingIntegrationRate() {
    if(err*Iout>0){
        if(fabsf(err)<=ScalarB) {
            return;
        }
        if(fabsf(err)<=ScalarA+ScalarB){
            Iterm *= (ScalarA- fabsf(err)+ScalarB)/ScalarA;
        }else{
            Iterm = 0;
        }
    }
}
void PID::integralLimit() {
    static float temp_Output,temp_Iout;
    temp_Iout = Iout+Iterm;
    temp_Output = Pout+Iout+Dout;
    if(fabsf(temp_Output)>outputMax){
        if(err*Iout>0){
            Iterm=0;
        }
    }
    if(temp_Iout>IntegralLimit){
        Iterm = 0;
        Iout = IntegralLimit;
    }else if(temp_Iout<-IntegralLimit){
        Iterm = 0;
        Iout = -IntegralLimit;
    }
}
void PID::derivativeOnMeasurement() {
    Dout = Kd*(fdbLast-fdb)/dt;
}
void PID::derivateFilter() {
    Dout = Dout*dt/(Derivative_LPF_RC+dt)+DoutLast*Derivative_LPF_RC/(Derivative_LPF_RC+dt);
}
void PID::outputFilter() {
    output = output*dt/(Output_LPF_RC+dt)+outputLast*Output_LPF_RC/(Output_LPF_RC+dt);
}
void PID::outputLimit() {
    if(output > outputMax){
        output = outputMax;
    }else if(output < outputMin){
        output = outputMin;
    }
}

float PositionPid::PidCalculate(float fdb, float ref)  {

    dt = DWT_GetDeltaT(&Dwt_Cnt);
    this->fdb = fdb;
    this->ref = ref;
    //Compute the error
    err = this->ref - this->fdb;

    //change-structure PID
//    if (Ap != 0 || Bp != 0 || Cp != 0)
//        Kp = Ap + Bp * (1 - exp(-Cp * fabsf(err)));
    //如果在死区外，则计算pid
    if(fabsf(err)>DeadBand){
        Pout = Kp * err;
        Iterm = Ki * err * dt;
        Dout = Kd*(err - errLast)/dt;
        //梯形积分
        if(improve & Trapezoid_Integral){
            trapezoidIntergral();
        }
        //变速积分
        if(improve & Changing_Integral_Rate){
            changingIntegrationRate();
        }
        //微分先行
        if(improve & Derivative_On_Measurement){
            derivativeOnMeasurement();
        }
        //微分滤波器
        if(improve & DerivativeFilter){
            derivateFilter();
        }
        //积分限幅
        if(improve & Integral_Limit){
            integralLimit();
        }
        Iout += Iterm;
        output = Pout + Iout + Dout;

        if(improve & OutputFilter){
            outputFilter();
        }
        outputLimit();
    }else{
        output = 0;
        Iterm = 0;
    }
    fdbLast = fdb;
    outputLast = output;
    DoutLast = Dout;
    errLast = err;
    ItermLast = Iterm;
    return output;
}
//float DeltaPid::PidCalculate(float fdb, float ref) {
//    this->fdb = fdb;
//    this->ref = ref;
//    //Compute the error
//    err = ref - fdb;
//    //change-structure PID
//    if (Ap != 0 || Bp != 0 || Cp != 0)
//        Kp =Ap + Bp * (1 - exp(-Cp * fabsf(err)));
//    // Compute the proportional output
//    Up = Kp * (err-errLast);
//    // Compute the integral output
//    Ui = Ki * err;
//    // Compute the derivative output
//    Ud = Kd * ((err-errLast)-(errLast-errLLast));
//    // Compute the pre-saturated output
//    outPre = Up+Ui+Ud+out;
//    //Update the previous proportional output
//    errLast = err;
//    errLLast = errLast;
//    //Limit output
//    if(outPre > outMax){
//        out = outMax;
//    }else if(outPre < outMin){
//        out = outMin;
//    }else{
//        out = outPre;
//    }
//    return out;
//}

float PID::PidCalculate() {
    return 0;
}
