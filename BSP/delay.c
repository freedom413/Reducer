
#include "stm32g4xx_hal.h"

static uint32_t usTicks; // 存储每微秒对应的SysTick周期数

/**
  * @brief  初始化微秒延时功能
  * @retval -1:失败; 0:成功
  * @note   需在HAL_Init()之后调用
  */
int delay_init(void) {
    // 检查SysTick是否已被HAL库配置为1ms中断（标准情况）
    if (SysTick->LOAD != (SystemCoreClock / 1000 - 1)) {
        // 如果SysTick已被重配置为非标准值，此方法可能不适用
        return -1;
    }
    // 计算每微秒的系统周期数
    usTicks = SystemCoreClock / 1000000;
    return 0;
}

/**
  * @brief  获取当前的“微秒时间戳”（一个相对计数值）
  * @retval 当前的时间戳计数值，单位是微秒的“刻度”
  */
uint32_t get_microsecond_tick(void) {
    // 确保使用volatile读取，防止编译器优化
    volatile uint32_t ms, cycle_cnt;
    
    do {
        ms = HAL_GetTick();            // 读取当前的毫秒计数
        cycle_cnt = SysTick->VAL;      // 读取SysTick计数器的当前值（递减）
        // 关键检查：如果在读取过程中发生了毫秒进位中断，则重新读取以保证一致性
    } while (ms != HAL_GetTick());     
    
    // 计算自本毫秒开始以来已经过去了多少微秒“刻度”
    // SysTick->LOAD是计数器重装载值，所以 (LOAD - cycle_cnt) 是本毫秒内已递增的周期数
    return (ms * 1000) + ((SysTick->LOAD - cycle_cnt) / usTicks);
}

/**
  * @brief  阻塞式微秒延时
  * @param  us: 需要延时的微秒数
  */
void delay_us(uint32_t us) {
    uint32_t start = get_microsecond_tick();
    // 等待直到达到目标时间差
    while ((get_microsecond_tick() - start) < us) {
        // 此处可以添加__NOP()或留空
    };
}

