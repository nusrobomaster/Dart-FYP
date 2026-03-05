#ifndef MOTOR_DM_H
#define MOTOR_DM_H

#include "bsp_damiao.h"
#include "stm32f4xx_hal_can.h"

#include <stdint.h>

#ifndef DM_MOTOR_MAX
#define DM_MOTOR_MAX 12U
#endif

void dm_motor_manager_init(void);
int dm_motor_manager_register(dm_motor_t *motor, CAN_HandleTypeDef *hcan);
dm_motor_t *dm_motor_manager_get_by_id(uint16_t id);
uint32_t dm_motor_manager_count(void);

void dm_motor_manager_set_velocity(dm_motor_t *motor, float vel_rad_s);
void dm_motor_manager_set_torque(dm_motor_t *motor, float torq_nm);

void dm_motor_manager_tick(void);
void dm_motor_manager_on_can_rx(uint32_t std_id, const uint8_t *data);

#endif /* MOTOR_DM_H */
