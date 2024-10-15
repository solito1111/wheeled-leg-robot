//
// Created by 45441 on 2024/3/4.
//

#ifndef INFANTRYGIMBALC_BSP_WATCHDOG_H
#define INFANTRYGIMBALC_BSP_WATCHDOG_H
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "iwdg.h"

/**是否使用看门狗
	* 1  是
	* 0  否
*/
#define CONFIG_USE_IWDG 1
#define CONFIG_USE_WWDG 0

void FeedIndependentWatchDog(void);
#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_BSP_WATCHDOG_H
