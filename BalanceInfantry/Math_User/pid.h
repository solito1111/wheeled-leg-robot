//
// Created by 45441 on 2023/7/6.
//

#ifndef INFANTRYGIMBALC_PID_H
#define INFANTRYGIMBALC_PID_H
#ifdef __cplusplus
#include "iostream"
#include "main.h"
#include "bsp_struct_typedef.h"
using namespace std;
extern "C" {
#endif
//C
enum improvePid{
    NONE=0b00000000,
    Integral_Limit=0b00000001,//积分限幅
    Derivative_On_Measurement=0b00000010,//微分先行
    Trapezoid_Integral=0b00000100,//梯形积分
    Changing_Integral_Rate=0b00001000,//变积分
    DerivativeFilter = 0b00010000,//微分滤波
    OutputFilter=0b00100000,//输出滤波
};
class PID{
public:
    float fdb;
    float fdbLast;
    float ref;

    float err;
    float errLast;

    float Iterm;
    float ItermLast;
    float Pout;
    float Iout;
    float Dout;
    float DoutLast;
    float Up;
    float Ud;
    float Ui;
    float output;
    float outputLast;
    float outputPre;
    float outputMax;
    float outputMin;
    uint32_t Dwt_Cnt;
    float dt;
public:
    float Kp;
    float Ap;
    float Bp;
    float Cp;
    float Ki;
    float Kd;
    float DeadBand;
    float ScalarA;//变积分公式参数 ITerm = err*((A-fabs(err)+B/A) when B<|err|<A+B
    float ScalarB;
    float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
    float Derivative_LPF_RC; // 微分滤波器系数
    float IntegralLimit;     // 积分限幅
    uint8_t improve;

    PID(float Kp,float Ki,float Kd,float outputMax,float outputMin,float DeadBand,uint8_t improve,
        float ScalarA,float ScalarB,float Output_LPF_RC,float Derivative_LPF_RC,float IntegralLimit):
        Kp(Kp),Ki(Ki),Kd(Kd),outputMax(outputMax),outputMin(outputMin),DeadBand(DeadBand),improve(improve),
        ScalarA(ScalarA),ScalarB(ScalarB),Output_LPF_RC(Output_LPF_RC),Derivative_LPF_RC(Derivative_LPF_RC),IntegralLimit(IntegralLimit){};
    virtual float PidCalculate() ;

protected:
    void trapezoidIntergral();//梯形积分
    void changingIntegrationRate();//变速积分
    void integralLimit();//积分限幅
    void derivativeOnMeasurement();//微分先行
    void derivateFilter();//微分滤波
    void outputLimit();
    void outputFilter();
};
class PositionPid : public PID{
public:
    PositionPid(float Kp,float Ki,float Kd,float outputMax,float outputMin,float DeadBand,uint8_t improve,
                float ScalarA,float ScalarB,float Output_LPF_RC,float Derivative_LPF_RC,float IntegralLimit): PID( Kp, Ki, Kd,outputMax, outputMin, DeadBand, improve,
     ScalarA, ScalarB, Output_LPF_RC, Derivative_LPF_RC, IntegralLimit){};
    float PidCalculate(float fdb,float ref);
};
class DeltaPid : public PID{
public:
    DeltaPid(float Kp,float Ki,float Kd,float outputMax,float outputMin,float DeadBand,uint8_t improve,
             float ScalarA,float ScalarB,float Output_LPF_RC,float Derivative_LPF_RC,float IntegralLimit): PID( Kp, Ki, Kd,outputMax, outputMin, DeadBand, improve,
                                                                                                                ScalarA, ScalarB, Output_LPF_RC, Derivative_LPF_RC, IntegralLimit){};
    float PidCalculate(float fdb,float ref);
private:
    float errLLast;
};
#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_PID_H
