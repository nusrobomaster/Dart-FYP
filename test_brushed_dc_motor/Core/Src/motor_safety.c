#include "motor_safety.h"

#include <stdint.h>

static volatile uint8_t g_motor_safety_active = 0;

void motor_safety_set(bool enabled)
{
    g_motor_safety_active = enabled ? 1U : 0U;
}

bool motor_safety_is_active(void)
{
    return g_motor_safety_active != 0U;
}
