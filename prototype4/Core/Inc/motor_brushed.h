#ifndef MOTOR_BRUSHED_H
#define MOTOR_BRUSHED_H

#include <stdint.h>

#ifndef MOTOR_BRUSHED_ENABLE
#define MOTOR_BRUSHED_ENABLE 0
#endif

void motor_brushed_init(void);
void motor_brushed_set_duty(int16_t duty);
int16_t motor_brushed_get_duty(void);
void motor_brushed_tick(void);

#endif /* MOTOR_BRUSHED_H */
