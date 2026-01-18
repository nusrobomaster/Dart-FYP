#define HX711_USE_FREERTOS
#include "hx711.h"

/* ---- Internal helpers --------------------------------------------------- */

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

static inline uint8_t hx711_gain_to_pulses(HX711_Gain g) {
    switch (g) {
        case HX711_GAIN_128: return 1; // A 128
        case HX711_GAIN_64:  return 3; // A 64
        case HX711_GAIN_32:  return 2; // B 32
        default:             return 1;
    }
}

static inline void sck_high(const HX711 *hx) {
    HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_SET);
    hx711_delay_us(hx->sck_delay_us);
}
static inline void sck_low(const HX711 *hx) {
    HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_RESET);
    hx711_delay_us(hx->sck_delay_us);
}
static inline GPIO_PinState dout_read(const HX711 *hx) {
    return HAL_GPIO_ReadPin(hx->dout_port, hx->dout_pin);
}

/* ---- Time base (DWT) ---------------------------------------------------- */
void hx711_timebase_init(void) {
#if defined(DWT) && defined(CoreDebug)
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    }
#endif
}

void hx711_delay_us(uint32_t us) {
#if defined(DWT) && defined(CoreDebug)
    if (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) {
        const uint32_t start = DWT->CYCCNT;
        const uint32_t ticks = (84000000UL / 1000000UL) * us;
        while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
        return;
    }
#endif
    /* Coarse fallback if DWT not present/enabled */
    const uint32_t loops = us * (SystemCoreClock / 1000000UL) / 8U;
    for (uint32_t i = 0; i < loops; ++i) { __NOP(); }
}

/* ---- Millisecond delay: HAL vs FreeRTOS --------------------------------- */
void hx711_delay_ms(uint32_t ms) {
#ifdef HX711_USE_FREERTOS
    /* Use RTOS delay only if scheduler is running; else HAL_Delay */
    BaseType_t running =
    #if (INCLUDE_xTaskGetSchedulerState == 1)
        (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);
    #else
        pdTRUE; /* Assume running if API not available */
    #endif

    if (running) {
        vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }
#endif
    HAL_Delay(ms);
}

/* ---- Public API --------------------------------------------------------- */

void hx711_init(HX711 *hx,
                GPIO_TypeDef *dout_port, uint16_t dout_pin,
                GPIO_TypeDef *sck_port,  uint16_t sck_pin,
                HX711_Gain gain) {
    hx->dout_port    = dout_port;
    hx->dout_pin     = dout_pin;
    hx->sck_port     = sck_port;
    hx->sck_pin      = sck_pin;
    hx->gain_pulses  = hx711_gain_to_pulses(gain);
    hx->scale        = 1.0f;
    hx->offset       = 0;
    hx->sck_delay_us = 30;   // HX711 max clk ~50kHz => ≥1us
    hx->ready		 = false;

    hx711_timebase_init();
    HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_RESET);
}

bool hx711_is_ready(const HX711 *hx) {
    return (dout_read(hx) == GPIO_PIN_RESET);
}

void hx711_set_gain(HX711 *hx, HX711_Gain gain) {
    hx->gain_pulses = hx711_gain_to_pulses(gain);
}

void hx711_wait_ready(const HX711 *hx, uint32_t poll_delay_ms) {
    while (!hx711_is_ready(hx)) {
        hx711_delay_ms(poll_delay_ms);   // <-- RTOS/HAL aware
    }
}

bool hx711_wait_ready_timeout(const HX711 *hx, uint32_t timeout_ms, uint32_t poll_delay_ms) {
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        if (hx711_is_ready(hx)) return true;
        const uint32_t step = (poll_delay_ms == 0) ? 1 : poll_delay_ms;
        hx711_delay_ms(step);             // <-- RTOS/HAL aware
        elapsed += step;
    }
    return false;
}

int32_t hx711_read_raw(HX711 *hx) {
    hx711_wait_ready(hx, 1);

    /* Protect the SCK-high window so it doesn't stretch past ~60us */
#ifdef HX711_USE_FREERTOS
    taskENTER_CRITICAL();
#else
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
#endif
//    hx->weight = 0;
    value = 0;
    for (uint8_t i = 0; i < 24; ++i) {
        sck_high(hx);
        value = (value << 1) | (dout_read(hx) == GPIO_PIN_SET ? 1 : 0);
        sck_low(hx);
        __NOP();
    }

    for (uint8_t i = 0; i < hx->gain_pulses; ++i) {
        sck_high(hx);
        sck_low(hx);
    }

#ifdef HX711_USE_FREERTOS
    taskEXIT_CRITICAL();
#else
    __set_PRIMASK(primask);
#endif

    if (value & 0x800000) { value |= 0xFF000000; }

    return value;
}

int32_t hx711_read_average(HX711 *hx, uint8_t times) {
    if (times == 0) return 0;
    times = MIN(times, 50);
    int64_t sum = 0;
    for (uint8_t i = 0; i < times; ++i) {
        sum += hx711_read_raw(hx);
        __NOP();
    }
    return (int32_t)(sum / (int32_t)times);
}

double hx711_get_value(HX711 *hx, uint8_t times) {
    return (double)hx711_read_average(hx, times) - (double)hx->offset;
}

float hx711_get_units(HX711 *hx, uint8_t times) {
    return (float)(hx711_get_value(hx, times) / (double)hx->scale);
}

void hx711_tare(HX711 *hx, uint8_t times) {
    hx->offset = hx711_read_average(hx, times);
}

void hx711_set_scale(HX711 *hx, float scale) { hx->scale = (scale == 0.0f) ? 1.0f : scale; }
float hx711_get_scale(const HX711 *hx) { return hx->scale; }
void  hx711_set_offset(HX711 *hx, int32_t offset) { hx->offset = offset; }
int32_t hx711_get_offset(const HX711 *hx) { return hx->offset; }

void hx711_power_down(const HX711 *hx) {
    HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_RESET);
    sck_high(hx);
    hx711_delay_us(80); // >60us
}

void hx711_power_up(const HX711 *hx) {
    sck_low(hx);
    /* Needs up to one conversion before DOUT goes low */
}
