#ifndef HX711_H
#define HX711_H

#include "stm32f4xx_hal.h"   // replace xxx with your series, e.g. stm32f4xx_hal.h
#include <stdint.h>
#include <stdbool.h>

/* ==== CONFIG: FreeRTOS integration ========================================

Define HX711_USE_FREERTOS to enable RTOS-aware delays. You may also choose
to use RTOS critical sections during the 24-bit read by defining
HX711_USE_RTOS_CRITICAL. If not defined, bare-metal PRIMASK is used.

Example (in your compiler flags or before including this header):
  -D HX711_USE_FREERTOS
  -D HX711_USE_RTOS_CRITICAL

============================================================================ */
#ifdef HX711_USE_FREERTOS
  #include "FreeRTOS.h"
  #include "task.h"
  /* Optional: use RTOS critical section instead of PRIMASK during read */
  /* #define HX711_USE_RTOS_CRITICAL */
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern int32_t value;

typedef enum {
    HX711_GAIN_128 = 128,   // Channel A, gain 128
    HX711_GAIN_64  = 64,    // Channel A, gain 64
    HX711_GAIN_32  = 32     // Channel B, gain 32
} HX711_Gain;

typedef struct {
    GPIO_TypeDef *dout_port;
    uint16_t      dout_pin;

    GPIO_TypeDef *sck_port;
    uint16_t      sck_pin;

    uint8_t       gain_pulses;   // 1 (128), 3 (64), 2 (32)
    float         scale;         // unit per count
    int32_t       offset;        // zero offset
    uint32_t       sck_delay_us;  // SCK high/low hold (≥1us)
    bool		  ready;
    int32_t		  weight;
} HX711;

/** Initialize DWT-based µs timer (called by hx711_init). */
void hx711_timebase_init(void);

/** Blocking µs delay (DWT precise, coarse fallback if absent). */
void hx711_delay_us(uint32_t us);

/** Blocking ms delay; uses FreeRTOS vTaskDelay when available/running. */
void hx711_delay_ms(uint32_t ms);

/** Initialize HX711 (configure pins in CubeMX). */
void hx711_init(HX711 *hx,
                GPIO_TypeDef *dout_port, uint16_t dout_pin,
                GPIO_TypeDef *sck_port,  uint16_t sck_pin,
                HX711_Gain gain);

/** Status / config */
bool  hx711_is_ready(const HX711 *hx);
void  hx711_set_gain(HX711 *hx, HX711_Gain gain);

/** Ready waits (ms) */
void  hx711_wait_ready(const HX711 *hx, uint32_t poll_delay_ms);
bool  hx711_wait_ready_timeout(const HX711 *hx, uint32_t timeout_ms, uint32_t poll_delay_ms);

/** Read API */
int32_t hx711_read_raw(HX711 *hx);
int32_t hx711_read_average(HX711 *hx, uint8_t times);
double  hx711_get_value(HX711 *hx, uint8_t times);
float   hx711_get_units(HX711 *hx, uint8_t times);

/** Calibration helpers */
void  hx711_tare(HX711 *hx, uint8_t times);
void  hx711_set_scale(HX711 *hx, float scale);
float hx711_get_scale(const HX711 *hx);
void   hx711_set_offset(HX711 *hx, int32_t offset);
int32_t hx711_get_offset(const HX711 *hx);

/** Power management */
void hx711_power_down(const HX711 *hx);
void hx711_power_up(const HX711 *hx);

#ifdef __cplusplus
}
#endif

#endif // HX711_H
