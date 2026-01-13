#include "motor_dm.h"

#include "motor_safety.h"

#include <string.h>

typedef struct {
    dm_motor_t *motor;
    CAN_HandleTypeDef *hcan;
} dm_motor_slot_t;

static dm_motor_slot_t g_dm_slots[DM_MOTOR_MAX];
static uint32_t g_dm_count = 0;

void dm_motor_manager_init(void)
{
    memset(g_dm_slots, 0, sizeof(g_dm_slots));
    g_dm_count = 0;
}

int dm_motor_manager_register(dm_motor_t *motor, CAN_HandleTypeDef *hcan)
{
    if (motor == NULL || hcan == NULL) {
        return -1;
    }
    for (uint32_t i = 0; i < g_dm_count; ++i) {
        if (g_dm_slots[i].motor == motor) {
            return (int)i;
        }
    }
    if (g_dm_count >= DM_MOTOR_MAX) {
        return -1;
    }
    g_dm_slots[g_dm_count].motor = motor;
    g_dm_slots[g_dm_count].hcan = hcan;
    g_dm_count++;
    return (int)(g_dm_count - 1U);
}

dm_motor_t *dm_motor_manager_get_by_id(uint16_t id)
{
    for (uint32_t i = 0; i < g_dm_count; ++i) {
        dm_motor_t *motor = g_dm_slots[i].motor;
        if (motor && motor->id == id) {
            return motor;
        }
    }
    return NULL;
}

uint32_t dm_motor_manager_count(void)
{
    return g_dm_count;
}

void dm_motor_manager_set_velocity(dm_motor_t *motor, float vel_rad_s)
{
    if (!motor) {
        return;
    }
    motor->ctrl.vel_set = vel_rad_s;
}

void dm_motor_manager_set_torque(dm_motor_t *motor, float torq_nm)
{
    if (!motor) {
        return;
    }
    motor->ctrl.tor_set = torq_nm;
}

void dm_motor_manager_tick(void)
{
    const bool safety = motor_safety_is_active();
    for (uint32_t i = 0; i < g_dm_count; ++i) {
        dm_motor_t *motor = g_dm_slots[i].motor;
        CAN_HandleTypeDef *hcan = g_dm_slots[i].hcan;
        if (!motor || !hcan) {
            continue;
        }
        if (safety) {
            dm4310_clear_para(motor);
        }
        dm4310_ctrl_send(hcan, motor);
    }
}

void dm_motor_manager_on_can_rx(uint32_t std_id, const uint8_t *data)
{
    if (std_id != 0x00 || data == NULL) {
        return;
    }
    uint8_t motor_id_nibble = data[0] & 0x0F;
    for (uint32_t i = 0; i < g_dm_count; ++i) {
        dm_motor_t *motor = g_dm_slots[i].motor;
        if (!motor) {
            continue;
        }
        if ((motor->id & 0x0F) == motor_id_nibble) {
            dm4310_fbdata(motor, (uint8_t *)data);
        }
    }
}
