#ifndef __DELAY_H__
#define __DELAY_H__
#include <stdint.h>
#include "stm32g4xx_hal.h"

// 不修改 sysTick 的us级延时函数
int delay_init(void);
void delay_us(uint32_t us);

static inline void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

#endif /* __DELAY_H__ */
