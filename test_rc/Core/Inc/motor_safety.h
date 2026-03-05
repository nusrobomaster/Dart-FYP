#ifndef MOTOR_SAFETY_H
#define MOTOR_SAFETY_H

#include <stdbool.h>

void motor_safety_set(bool enabled);
bool motor_safety_is_active(void);

#endif /* MOTOR_SAFETY_H */
