#ifndef MOTOR_SERVO_H
#define MOTOR_SERVO_H

#include <stdint.h>

#ifndef MOTOR_SERVO_ENABLE
#define MOTOR_SERVO_ENABLE 0
#endif

#ifndef MOTOR_SERVO_MAX
#define MOTOR_SERVO_MAX 4U
#endif

void motor_servo_init(void);
void motor_servo_set_us(uint8_t channel, uint16_t pulse_us);
void motor_servo_tick(void);

#endif /* MOTOR_SERVO_H */
