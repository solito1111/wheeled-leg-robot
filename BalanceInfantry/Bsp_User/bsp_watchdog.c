//
// Created by 45441 on 2024/3/4.
//

#include "bsp_watchdog.h"
/**参数说明：
***独立看门狗
	*IWDG_PR = 0x04：64分频，计算公式:4*2^prer
	*IWDG_RLR = 625 刚好溢出时间为1s
	*溢出时间计算公式:Tout = (4*2^PR)*RLR/40 (ms)
	*如需改变溢出时间，只需要改变PR值或RLR值即可. 默认为1s
***窗口看门狗
	*STM32F405/407下窗口看门狗最长喂狗间隔为59.93ms
	*目前并不使用
*/

void FeedIndependentWatchDog(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}