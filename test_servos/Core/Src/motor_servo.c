#include "motor_servo.h"

#include "stm32f4xx_hal.h"

static uint16_t g_servo_us[MOTOR_SERVO_MAX];

__weak void motor_servo_platform_write(uint8_t channel, uint16_t pulse_us)
{
    (void)channel;
    (void)pulse_us;
}

void motor_servo_init(void)
{
    for (uint32_t i = 0; i < MOTOR_SERVO_MAX; ++i) {
        g_servo_us[i] = 1500U;
    }
}

void motor_servo_set_us(uint8_t channel, uint16_t pulse_us)
{
    if (channel >= MOTOR_SERVO_MAX) {
        return;
    }
    g_servo_us[channel] = pulse_us;
}

void motor_servo_tick(void)
{
#if MOTOR_SERVO_ENABLE
    for (uint32_t i = 0; i < MOTOR_SERVO_MAX; ++i) {
        motor_servo_platform_write((uint8_t)i, g_servo_us[i]);
    }
#endif
}
