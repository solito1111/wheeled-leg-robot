//
// Created by 45441 on 2023/7/5.
//

#include "bsp_buzzer.h"

#define playNote_A  220
#define playNote_3A 110
#define playNote_5A 440
#define playNote_sA 233  //233.082
#define playNote_B  247  //246.942
#define playNote_3B  123  //123.471
#define playNote_5B  494  //493.883
#define playNote_C  262  //261.626
#define playNote_5C  523  //523.251
#define playNote_sC 277  //277.183
#define playNote_D  294  //293.665
#define playNote_sD 311  //311.127
#define playNote_5D 587  //587.33
#define playNote_3sD 156  //155.563
#define playNote_E  330  //329.629
#define playNote_3E  165  //164.814
#define playNote_F  349  //349.228
#define playNote_3F  175  //174.614
#define playNote_sF 370  //369.994
#define playNote_3sF 185  //184.997
#define playNote_G  392  //391.995
#define playNote_sG 415  //415.305
#define playNote_3G 196  //195.998
#define playNote_5sG 831  //830.609
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
#define MAX_PSC                 1000

#define MAX_BUZZER_PWM      20000
#define MIN_BUZZER_PWM      10000

uint16_t psc = 0;
uint16_t pwm = MIN_BUZZER_PWM;
void Buzzer_On(){
    pwm++;
    psc++;

    if(pwm > MAX_BUZZER_PWM)
    {
        pwm = MIN_BUZZER_PWM;
    }
    if(psc > MAX_PSC)
    {
        psc = 0;
    }
    buzzer_on(psc, pwm);
}
void playNote(uint16_t frequency, uint16_t duration)
{
    __HAL_TIM_SET_AUTORELOAD(&htim4, (HAL_RCC_GetPCLK1Freq() / 84) / frequency); // 设置自动重装载值以产生特定频率的音调
    HAL_Delay(duration*200); // 持续时间
}
void playMusic()
{
    float t=0.2;

//    playNote( playNote_5B , 1 );    //前奏
//    playNote( playNote_G , 1 );
//    playNote( playNote_5B , 2 );
//    playNote( playNote_G , 1 );
//    playNote( playNote_5B ,  2);
//    playNote( playNote_G , 1 );
//    playNote( playNote_5D ,  2);
//    playNote( playNote_G , 1 );
//    playNote( playNote_5C , 1 );
//    playNote( playNote_5C , 1 );
//    playNote( playNote_G , 1 );
//    playNote( playNote_5B , 1 );
//    playNote( playNote_5C , 1 );
//
//    playNote( playNote_5B , 1 );
//    playNote( playNote_G , 1 );
//    playNote( playNote_5B , 2 );
//    playNote( playNote_G , 1 );
//    playNote( playNote_5B ,  2);
//    playNote( playNote_G , 1 );
//    playNote( playNote_5D ,  2);
//    playNote( playNote_G , 1 );
//    playNote( playNote_5C , 1 );
//    playNote( playNote_5C , 1 );
//    playNote( playNote_G , 1 );
//    playNote( playNote_5B , 1 );
//    playNote( playNote_5C , 1 );
//
//    playNote( playNote_5B , 1 );
//    playNote( playNote_G , 1 );
//    playNote( playNote_5B , 2 );
//    playNote( playNote_G , 1 );
//    playNote( playNote_5B ,  2);
//    playNote( playNote_G , 1 );
//    playNote( playNote_5D ,  2);
//    playNote( playNote_G , 1 );
//    playNote( playNote_5C , 1 );
//    playNote( playNote_5C , 1 );
//    playNote( playNote_G , 1 );
//    playNote( playNote_5B , 1 );
//    playNote( playNote_5C , 1 );
//
//    playNote( playNote_5B , 1 );
//    playNote( playNote_G , 1 );
//    playNote( playNote_5B , 2 );
//    playNote( playNote_G , 1 );
//    playNote( playNote_5B , 2 );
//    playNote( playNote_G , 1 );
//    playNote( playNote_5D ,  2);
//    playNote( playNote_G , 1 );
//    playNote( playNote_5C , 1 );
//    playNote( playNote_5C , 1 );
//    playNote( playNote_G , 1 );
//    playNote( playNote_D , 2 );
//
//    playNote( playNote_E , 6 );
//    playNote( playNote_D , 2 );
//    playNote( playNote_E , 2 );
//    playNote( playNote_D , 2 );
//    playNote( playNote_E , 2 );
//    playNote( playNote_G , 4 );
//
//    playNote( playNote_5C , 4 );
//    playNote( playNote_5B , 4 );
//    playNote( playNote_E , 4 );
//    playNote( playNote_D , 2 );
//
//    playNote( playNote_E , 6 );
//    playNote( playNote_D , 2 );
//    playNote( playNote_E , 2 );
//    playNote( playNote_D , 2 );
//    playNote( playNote_E , 2 );
//    playNote( playNote_5C , 8 );
//
//    playNote( playNote_5B , 2 );
//    playNote( playNote_5D , 4 );
//    playNote( playNote_E , 10 );
//
//    playNote( playNote_D , 2 );
//    playNote( playNote_E , 2 );
//    playNote( playNote_D , 2 );
//    playNote( playNote_E , 2 );
//    playNote( playNote_5B , 4 );
//
//    playNote( playNote_5C , 4 );
//    playNote( playNote_5B , 4 );
//    playNote( playNote_E , 4 );
//    playNote( playNote_D , 2 );
//
//    playNote( playNote_E , 6 );
//    playNote( playNote_D , 2 );
//    playNote( playNote_E , 2 );
//    playNote( playNote_D , 2 );
//    playNote( playNote_E , 2 );
//    playNote( playNote_5B , 4 );
//
//    playNote( playNote_5C , 4 );
//    playNote( playNote_5D , 10 );

    playNote( 0 , 4 );      //我一直追寻着你
    playNote( playNote_E , 2 );
    playNote( 0 , t );
    playNote( playNote_E , 2 );
    playNote( playNote_D , 1 );
    playNote( playNote_E , 3 );
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 2 );

    playNote( 0 , 2 );      //你好像不远也不近
    playNote( playNote_E , 2 );
    playNote( 0 , t );
    playNote( playNote_E , 2 );
    playNote( playNote_D , 1 );
    playNote( playNote_E , 3 );
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 4 );
    playNote( playNote_C , 6 );

    playNote( 0 , 2 );      //却总保持着距离
    playNote( playNote_E , 2 );
    playNote( 0 , t );
    playNote( playNote_E , 2 );
    playNote( playNote_D , 1 );
    playNote( playNote_E , 3 );
//	playNote( playNote_D , 2 );
    playNote( playNote_D , 2 );
//	playNote( playNote_C , 1 );
    playNote( playNote_C , 2 );
    playNote( playNote_D , 2 );

    playNote( 0 , 2 );
    playNote( playNote_E , 2 );  //我一直幻想着你
    playNote( 0 , t );
    playNote( playNote_E , 2 );
    playNote( playNote_D , 1 );
    playNote( playNote_E , 3 );
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 2 );

    playNote( 0 , 2 );      //在我身边在我怀里
    playNote( playNote_E , 2 );
    playNote( 0 , t );
    playNote( playNote_E , 2 );
    playNote( playNote_D , 1 );
    playNote( playNote_E , 3 );
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_5C , 4 );
    playNote( playNote_C , 6 );

    playNote( 0 , 2 );      //让我欢笑让我哭泣
    playNote( playNote_E , 2 );
    playNote( 0 , t );
    playNote( playNote_E , 2 );
    playNote( playNote_D , 1 );
    playNote( playNote_E , 3 );
    playNote( playNote_D , 2 );
    playNote( playNote_B , 2 );
    playNote( playNote_A , 1 );
    playNote( playNote_3G , 5 );

    playNote( 0 , 1 );      //你是我灵魂的旋律
    playNote( playNote_3G , 1 );
    playNote( 0 , t );
    playNote( playNote_3G , 1 );
    playNote( 0 , t );
    playNote( playNote_3G , 1 );
    playNote( playNote_G , 4 );
    playNote( playNote_E , 3 );
    playNote( playNote_D , 1 );
    playNote( playNote_C , 2 );
    playNote( 0 , t );
    playNote( playNote_C , 4 );

    playNote( 0 , 1 );
    playNote( playNote_C , 2 );      //春日的细雨
    playNote( 0 , 0.05 );
    playNote( playNote_C , 2 );
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_A , 6 );

    playNote( 0 , 2 );          //墓碑的雏菊
    playNote( playNote_A , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_D , 2 );
    playNote( playNote_C , 2 );
    playNote( playNote_D , 6 );
    playNote( 0 , 2 );

    playNote( playNote_E , 4 );      //我从来不会计算代价
    playNote( playNote_F , 4 );
    playNote( playNote_G , 6 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 2 );
    playNote( playNote_E , 1 );
    playNote( playNote_G , 3 );
    playNote( playNote_5B , 4 );
    playNote( playNote_5C , 6 );

    playNote( playNote_C , 2 );     //为了你可以纵身无底悬崖
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 4 );
    playNote( playNote_5A , 4 );
    playNote( playNote_G , 1 );
    playNote( playNote_5A , 3 );
    playNote( playNote_G , 1 );
    playNote( 0 , 0.1 );
    playNote( playNote_G , 3 );
    playNote( 0 , 0.1 );
    playNote( playNote_G , 4 );
    playNote( playNote_D , 8 );

    playNote( playNote_E , 4 );     //像条狗更像一个笑话
    playNote( playNote_F , 4 );
    playNote( playNote_G , 6 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 2 );
    playNote( playNote_E , 1 );
    playNote( playNote_G , 3 );
    playNote( playNote_5B , 4 );
    playNote( playNote_5C , 6 );

    playNote( playNote_C , 2 );     //也许我很傻但我不会怕
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_5B , 4 );
    playNote( playNote_5A , 6 );
    playNote( 0 , t );
    playNote( playNote_5A , 2 );
    playNote( playNote_G , 1 );
    playNote( playNote_5A , 3 );
    playNote( playNote_5C , 4 );
    playNote( playNote_5D , 6 );

    playNote( 0 , 2 );
    playNote( playNote_G , 2 );   //我愿意呀
    playNote( playNote_5C , 2 );
    playNote( playNote_5B , 1 );
    playNote( playNote_5C , 12 );

    playNote( 0 , 4 );

    playNote( 0 , 2 );      //人们都追寻着你
    playNote( playNote_E , 2 );
    playNote( 0 , t );
    playNote( playNote_E , 2 );
    playNote( playNote_D , 1 );
    playNote( playNote_E , 3 );
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 2 );

    playNote( 0 , 2 );      //都曾把你当作唯一
    playNote( playNote_E , 2 );
    playNote( 0 , t );
    playNote( playNote_E , 2 );
    playNote( playNote_D , 1 );
    playNote( playNote_E , 3 );
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 4 );
    playNote( playNote_C , 6 );

//	playNote( 0 , 4 );

    playNote( 0 , 2 );      //最后却无能为力
    playNote( playNote_E , 2 );
    playNote( 0 , t );
    playNote( playNote_E , 2 );
    playNote( playNote_D , 1 );
    playNote( playNote_E , 3 );
//	playNote( playNote_D , 2 );
    playNote( playNote_D , 2 );
//	playNote( playNote_C , 1 );
    playNote( playNote_C , 2 );
    playNote( playNote_D , 2 );

    playNote( 0 , 2 );      //人们都幻想着你
    playNote( playNote_E , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_D , 1 );
    playNote( playNote_E , 3 );
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 2 );

    playNote( 0 , 2 );      //幻想你依偎他怀里
    playNote( playNote_E , 2 );
    playNote( 0 , t );
    playNote( playNote_E , 2 );
    playNote( playNote_D , 1 );
    playNote( playNote_E , 3 );
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_5C , 4 );
    playNote( playNote_C , 6 );

//	playNote( 0 , 4 );

    playNote( 0 , 2 );      //一朝拥有一劳永逸
    playNote( playNote_E , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_D , 1 );
    playNote( playNote_E , 3 );
    playNote( playNote_D , 2 );
    playNote( playNote_B , 2 );
    playNote( playNote_A , 1 );
    playNote( playNote_3G , 5 );

    playNote( 0 , 1 );      //可是你不为谁守候
    playNote( playNote_3G , 1 );
    playNote( 0 , t );
    playNote( playNote_3G , 1 );
    playNote( 0 , t );
    playNote( playNote_3G , 1 );
    playNote( playNote_G , 4 );
    playNote( playNote_E , 3 );
    playNote( playNote_D , 1 );
    playNote( playNote_C , 2 );
    playNote( 0 , t );
    playNote( playNote_C , 4 );

    playNote( 0 , 1 );
    playNote( playNote_C , 2 );      //不承诺永久
    playNote( 0 , t );
    playNote( playNote_C , 2 );
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_A , 6 );

    playNote( 0 , 2 );          //不轻易停留
    playNote( playNote_A , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_D , 2 );
    playNote( playNote_C , 2 );
    playNote( playNote_D , 10 );

    playNote( 0 , 4 );
    playNote( playNote_E , 4 );      //我知道只有不断出发
    playNote( playNote_F , 4 );
    playNote( playNote_G , 6 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 2 );
    playNote( playNote_E , 1 );
    playNote( playNote_G , 3 );
    playNote( playNote_5B , 4 );
    playNote( playNote_5C , 6 );

    playNote( playNote_C , 2 );     //才能够紧随你纵情的步伐
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 4 );
    playNote( playNote_5A , 5 );
    playNote( playNote_G , 1 );
    playNote( playNote_5A , 2 );
    playNote( playNote_G , 1 );
    playNote( 0 , t );
    playNote( playNote_G , 3 );
    playNote( 0 , 0.1 );
    playNote( playNote_D , 2 );
    playNote( 0 , 0.1 );
    playNote( playNote_D , 8 );

    playNote( playNote_E , 4 );     //就算是海角至天涯
    playNote( playNote_F , 4 );
    playNote( playNote_G , 4 );
    playNote( 0 , 0.5 );
    playNote( playNote_G , 2 );
    playNote( playNote_E , 1 );
    playNote( playNote_G , 3 );
    playNote( playNote_5D , 4 );
    playNote( playNote_5C , 6 );

    playNote( playNote_C , 2 );    //青丝变白发
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_5C , 4 );
    playNote( playNote_5A , 5 );

    playNote( playNote_G , 1 );     //只等着你回答
    playNote( playNote_5A , 2 );
    playNote( playNote_G , 1 );
    playNote( playNote_5A , 3 );
    playNote( playNote_5C , 4 );
    playNote( playNote_5D , 6 );

    playNote( 0 , 2 );
    playNote( playNote_D , 2 );   //我愿意呀
    playNote( playNote_E , 2 );
    playNote( playNote_D , 1 );
    playNote( playNote_C , 12 );

    //间奏略

    playNote( 0 , 4 );
    playNote( playNote_E , 4 );      //我从来不会计算代价
    playNote( playNote_F , 4 );
    playNote( playNote_G , 6 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 2 );
    playNote( playNote_E , 1 );
    playNote( playNote_G , 3 );
    playNote( playNote_5B , 4 );
    playNote( playNote_5C , 6 );

    playNote( playNote_C , 2 );     //为了你可以纵身无底悬崖
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 4 );
    playNote( playNote_5A , 5 );
    playNote( playNote_G , 1 );
    playNote( playNote_5A , 2 );
    playNote( playNote_G , 1 );
    playNote( 0 , 0.1 );
    playNote( playNote_G , 3 );
    playNote( 0 , 0.1 );
    playNote( playNote_G , 4 );
    playNote( playNote_D , 8 );

    playNote( playNote_E , 4 );     //像条狗更像一个笑话
    playNote( playNote_F , 4 );
    playNote( playNote_G , 6 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 2 );
    playNote( playNote_E , 1 );
    playNote( playNote_G , 3 );
    playNote( playNote_5B , 4 );
    playNote( playNote_5C , 6 );

    playNote( playNote_C , 2 );     //也许我很傻但我不会怕
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_5C , 4 );
    playNote( playNote_5A , 6 );
    playNote( 0 , t );
    playNote( playNote_5A , 2 );
    playNote( playNote_G , 1 );
    playNote( playNote_5A , 3 );
    playNote( playNote_5C , 4 );
    playNote( playNote_5D , 6 );

    playNote( playNote_E , 4 );      //我知道只有不断出发
    playNote( playNote_F , 4 );
    playNote( playNote_G , 6 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 2 );
    playNote( playNote_E , 1 );
    playNote( playNote_G , 3 );
    playNote( playNote_5B , 4 );
    playNote( playNote_5C , 6 );

    playNote( playNote_C , 2 );     //才能够紧随你纵情的步伐
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_G , 4 );
    playNote( playNote_5A , 5 );
    playNote( playNote_G , 1 );
    playNote( playNote_5A , 2 );
    playNote( playNote_G , 1 );
    playNote( 0 , 0.1 );
    playNote( playNote_G , 3 );
    playNote( 0 , 0.1 );
    playNote( playNote_G , 4 );
//	playNote( playNote_D , 2 );
//	playNote( 0 , 0.1 );
    playNote( playNote_D , 8 );

    playNote( playNote_E , 4 );     //就算是海角至天涯
    playNote( playNote_F , 4 );
    playNote( playNote_G , 4 );
    playNote( 0 , 1 );
    playNote( playNote_G , 2 );
    playNote( playNote_E , 1 );
    playNote( playNote_G , 3 );
    playNote( playNote_5D , 4 );
    playNote( playNote_5C , 6 );

    playNote( playNote_C , 2 );    //青丝变白发
    playNote( playNote_D , 2 );
    playNote( playNote_E , 2 );
    playNote( playNote_5C , 4 );
    playNote( playNote_5A , 5 );

    playNote( playNote_G , 1 );     //只等着你回答
    playNote( playNote_5A , 2 );
    playNote( playNote_G , 1 );
    playNote( playNote_5A , 3 );
    playNote( playNote_5C , 4 );
    playNote( playNote_5D , 6 );

    playNote( 0 , 2 );
    playNote( playNote_G , 2 );   //我愿意呀
    playNote( playNote_5C , 2 );
    playNote( playNote_5B , 1 );
    playNote( playNote_5C , 12 );

    //尾
    playNote( 0 , 4 );
    playNote( playNote_E , 4 );
    playNote( playNote_D , 4 );
    playNote( playNote_C , 4 );
    playNote( playNote_G , 4 );
    playNote( playNote_C , 4 );
    playNote( playNote_D , 4 );
    playNote( playNote_E , 4 );
    playNote( playNote_F , 4 );
    playNote( playNote_G , 4 );
    playNote( playNote_F , 4 );

    playNote( playNote_E , 4 );
    playNote( playNote_D , 4 );
    playNote( playNote_C , 4 );
    playNote( playNote_D , 4 );
    playNote( playNote_E , 4 );
    playNote( playNote_F , 4 );
    playNote( playNote_E , 4 );
    playNote( playNote_D , 4 );
    playNote( playNote_C , 4 );
    playNote( playNote_G , 4 );

    playNote( playNote_E , 4 );
    playNote( playNote_D , 4 );
    playNote( playNote_E , 4 );
    playNote( playNote_F , 4 );
    playNote( playNote_G , 4 );
    playNote( playNote_F , 4 );
    playNote( playNote_E , 4 );
    playNote( playNote_D , 4 );

    playNote( playNote_E , 4 );
    playNote( playNote_D , 4 );
    playNote( playNote_E , 4 );
    playNote( playNote_F , 4 );

    playNote( playNote_E , 4 );
    playNote( playNote_D , 4 );
    playNote( playNote_C , 4 );
    playNote( playNote_G , 4 );
    playNote( playNote_C , 4 );
    playNote( playNote_D , 4 );
    playNote( playNote_E , 4 );
    playNote( playNote_F , 4 );
    playNote( playNote_G , 4 );
    playNote( playNote_F , 4 );
    playNote( playNote_E , 4 );
    playNote( playNote_D , 4 );
}

