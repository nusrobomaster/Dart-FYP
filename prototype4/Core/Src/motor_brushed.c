#include "motor_brushed.h"

#include "stm32f4xx_hal.h"

static int16_t g_brushed_duty = 0;

__weak void motor_brushed_platform_write(int16_t duty)
{
    (void)duty;
}

void motor_brushed_init(void)
{
    g_brushed_duty = 0;
}

void motor_brushed_set_duty(int16_t duty)
{
    if (duty > 1000) {
        duty = 1000;
    } else if (duty < -1000) {
        duty = -1000;
    }
    g_brushed_duty = duty;
}

int16_t motor_brushed_get_duty(void)
{
    return g_brushed_duty;
}

void motor_brushed_tick(void)
{
#if MOTOR_BRUSHED_ENABLE
    motor_brushed_platform_write(g_brushed_duty);
#endif
}
