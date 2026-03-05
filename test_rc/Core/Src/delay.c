#include "delay.h"
#include "cmsis_os.h"

static void delay_dwt_init(void)
{
#if defined(DWT) && defined(CoreDebug)
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
#endif
}

void delay_ms(uint32_t ms)
{
    if (osKernelGetState() == osKernelRunning) {
        osDelay(ms);
        return;
    }
    HAL_Delay(ms);
}

void delay_us(uint32_t us)
{
    delay_dwt_init();
#if defined(DWT) && defined(CoreDebug)
    if (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) {
        const uint32_t start = DWT->CYCCNT;
        const uint32_t ticks = (SystemCoreClock / 1000000UL) * us;
        while ((DWT->CYCCNT - start) < ticks) {
            __NOP();
        }
        return;
    }
#endif
    const uint32_t loops = us * (SystemCoreClock / 1000000UL) / 8U;
    for (uint32_t i = 0; i < loops; ++i) {
        __NOP();
    }
}
