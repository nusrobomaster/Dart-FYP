#ifndef BRITERENCODER_H
#define BRITERENCODER_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

typedef struct {
    uint8_t id;
    uint32_t value;
    int32_t velocity;
    uint8_t last_func;
    uint8_t last_status;
    uint32_t last_rx_ms;
    uint8_t pending_id;
} briterencoder_t;

void briterencoder_init(briterencoder_t *enc, uint8_t id);

HAL_StatusTypeDef briterencoder_read_value(CAN_HandleTypeDef *hcan, briterencoder_t *enc);
HAL_StatusTypeDef briterencoder_read_velocity(CAN_HandleTypeDef *hcan, briterencoder_t *enc);

HAL_StatusTypeDef briterencoder_set_id(CAN_HandleTypeDef *hcan, briterencoder_t *enc, uint8_t new_id);
HAL_StatusTypeDef briterencoder_set_baud(CAN_HandleTypeDef *hcan, briterencoder_t *enc, uint8_t baud_code);
HAL_StatusTypeDef briterencoder_set_mode(CAN_HandleTypeDef *hcan, briterencoder_t *enc, uint8_t mode);
HAL_StatusTypeDef briterencoder_set_auto_period_us(CAN_HandleTypeDef *hcan, briterencoder_t *enc, uint16_t period_us);
HAL_StatusTypeDef briterencoder_set_zero(CAN_HandleTypeDef *hcan, briterencoder_t *enc);
HAL_StatusTypeDef briterencoder_set_direction(CAN_HandleTypeDef *hcan, briterencoder_t *enc, uint8_t direction);
HAL_StatusTypeDef briterencoder_set_speed_calc_time_ms(CAN_HandleTypeDef *hcan, briterencoder_t *enc, uint16_t time_ms);
HAL_StatusTypeDef briterencoder_set_midpoint(CAN_HandleTypeDef *hcan, briterencoder_t *enc);
HAL_StatusTypeDef briterencoder_set_value(CAN_HandleTypeDef *hcan, briterencoder_t *enc, uint32_t value);
HAL_StatusTypeDef briterencoder_set_five_turns(CAN_HandleTypeDef *hcan, briterencoder_t *enc);

void briterencoder_on_can_rx(briterencoder_t *enc, uint32_t std_id, const uint8_t *data, uint8_t dlc);

#endif /* BRITERENCODER_H */
