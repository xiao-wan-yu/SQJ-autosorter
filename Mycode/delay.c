#include "delay.h"
#include <stdint.h>

/* 私有变量 */
static uint32_t us_ticks = 0;
static uint32_t last_core_clock = 0;
static uint8_t dwt_enabled = 0;  // 新增：跟踪DWT是否已使能

/**
  * @brief  初始化或重新初始化DWT
  */
void DWT_Init(void) 
{
    /* 1. 使能DWT跟踪单元 */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) 
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    
    /* 2. 清零并启用周期计数器 */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    /* 3. 计算并保存参数 */
    us_ticks = SystemCoreClock / 1000000;
    last_core_clock = SystemCoreClock;
    dwt_enabled = 1;  // 标记为已初始化
    
    /* 调试信息（可选） */
    // printf("DWT Init: %lu Hz, us_ticks=%lu\n", SystemCoreClock, us_ticks);
}

/**
  * @brief  检查并确保DWT正确初始化
  */
static void ensure_dwt_initialized(void)
{
    /* 情况1：完全未初始化 */
    if (!dwt_enabled) 
    {
        DWT_Init();
        return;
    }
    
    /* 情况2：系统时钟变化，需要重新初始化 */
    if (SystemCoreClock != last_core_clock) 
    {
        /* 重要：重新初始化整个DWT，不仅仅是更新us_ticks！ */
        DWT_Init();
    }
    
    /* 情况3：检查DWT是否被意外禁用（如低功耗模式后） */
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) 
    {
        /* 重新使能计数器 */
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

/**
  * @brief  微秒延时函数（带自动初始化检查）
  */
void delay_us(uint32_t us) 
{
    /* 1. 确保DWT已正确初始化 */
    ensure_dwt_initialized();
    
    /* 2. 参数检查 */
    if (us == 0) return;
    
    /* 3. 计算需要的时钟周期数 */
    uint32_t start_tick = DWT->CYCCNT;
    uint32_t delay_ticks = us * us_ticks;
    
    /* 4. 延时循环（简化版，足够用于大多数情况） */
    while ((DWT->CYCCNT - start_tick) < delay_ticks) 
    {
        // 空循环
    }
}

/**
  * @brief  毫秒延时函数
  */
void delay_ms(uint32_t ms) 
{
    /* 确保DWT已初始化 */
    ensure_dwt_initialized();
    
    /* 分段处理，避免可能的溢出问题 */
    while (ms > 0) 
    {
        if (ms >= 1000) 
        {
            delay_us(1000000);  // 延时1秒
            ms -= 1000;
        }
        else 
        {
            delay_us(ms * 1000);
            ms = 0;
        }
    }
}
