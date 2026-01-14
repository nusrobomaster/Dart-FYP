#include "briterencoder.h"

#include <string.h>

enum {
    BRITERENCODER_FUNC_READ_VALUE = 0x01,
    BRITERENCODER_FUNC_SET_ID = 0x02,
    BRITERENCODER_FUNC_SET_BAUD = 0x03,
    BRITERENCODER_FUNC_SET_MODE = 0x04,
    BRITERENCODER_FUNC_SET_AUTO_PERIOD = 0x05,
    BRITERENCODER_FUNC_SET_ZERO = 0x06,
    BRITERENCODER_FUNC_SET_DIR = 0x07,
    BRITERENCODER_FUNC_READ_VELOCITY = 0x0A,
    BRITERENCODER_FUNC_SET_VEL_TIME = 0x0B,
    BRITERENCODER_FUNC_SET_MID = 0x0C,
    BRITERENCODER_FUNC_SET_VALUE = 0x0D,
    BRITERENCODER_FUNC_SET_FIVE_TURNS = 0x0F,
};

static HAL_StatusTypeDef briterencoder_send(CAN_HandleTypeDef *hcan,
                                            uint8_t id,
                                            uint8_t func,
                                            const uint8_t *payload,
                                            uint8_t payload_len)
{
    if (!hcan || payload_len > 4U) {
        return HAL_ERROR;
    }

    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    uint32_t mailbox = 0;
    uint8_t len = (uint8_t)(3U + payload_len);

    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = len;
    header.StdId = id;

    data[0] = len;
    data[1] = id;
    data[2] = func;
    if (payload_len && payload) {
        memcpy(&data[3], payload, payload_len);
    }

    return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

static void briterencoder_u16_to_le(uint16_t val, uint8_t *out)
{
    out[0] = (uint8_t)(val & 0xFFU);
    out[1] = (uint8_t)((val >> 8) & 0xFFU);
}

static void briterencoder_u32_to_le(uint32_t val, uint8_t *out)
{
    out[0] = (uint8_t)(val & 0xFFU);
    out[1] = (uint8_t)((val >> 8) & 0xFFU);
    out[2] = (uint8_t)((val >> 16) & 0xFFU);
    out[3] = (uint8_t)((val >> 24) & 0xFFU);
}

static uint32_t briterencoder_u32_from_le(const uint8_t *in)
{
    return (uint32_t)in[0]
        | ((uint32_t)in[1] << 8)
        | ((uint32_t)in[2] << 16)
        | ((uint32_t)in[3] << 24);
}

static int32_t briterencoder_i32_from_le(const uint8_t *in)
{
    return (int32_t)briterencoder_u32_from_le(in);
}

void briterencoder_init(briterencoder_t *enc, uint8_t id)
{
    if (!enc) {
        return;
    }
    memset(enc, 0, sizeof(*enc));
    enc->id = id;
}

HAL_StatusTypeDef briterencoder_read_value(CAN_HandleTypeDef *hcan, briterencoder_t *enc)
{
    uint8_t payload = 0x00;
    if (!enc) {
        return HAL_ERROR;
    }
    enc->last_func = BRITERENCODER_FUNC_READ_VALUE;
    return briterencoder_send(hcan, enc->id, BRITERENCODER_FUNC_READ_VALUE, &payload, 1);
}

HAL_StatusTypeDef briterencoder_read_velocity(CAN_HandleTypeDef *hcan, briterencoder_t *enc)
{
    uint8_t payload = 0x00;
    if (!enc) {
        return HAL_ERROR;
    }
    enc->last_func = BRITERENCODER_FUNC_READ_VELOCITY;
    return briterencoder_send(hcan, enc->id, BRITERENCODER_FUNC_READ_VELOCITY, &payload, 1);
}

HAL_StatusTypeDef briterencoder_set_id(CAN_HandleTypeDef *hcan, briterencoder_t *enc, uint8_t new_id)
{
    if (!enc) {
        return HAL_ERROR;
    }
    enc->last_func = BRITERENCODER_FUNC_SET_ID;
    enc->pending_id = new_id;
    return briterencoder_send(hcan, enc->id, BRITERENCODER_FUNC_SET_ID, &new_id, 1);
}

HAL_StatusTypeDef briterencoder_set_baud(CAN_HandleTypeDef *hcan, briterencoder_t *enc, uint8_t baud_code)
{
    if (!enc) {
        return HAL_ERROR;
    }
    enc->last_func = BRITERENCODER_FUNC_SET_BAUD;
    return briterencoder_send(hcan, enc->id, BRITERENCODER_FUNC_SET_BAUD, &baud_code, 1);
}

HAL_StatusTypeDef briterencoder_set_mode(CAN_HandleTypeDef *hcan, briterencoder_t *enc, uint8_t mode)
{
    if (!enc) {
        return HAL_ERROR;
    }
    enc->last_func = BRITERENCODER_FUNC_SET_MODE;
    return briterencoder_send(hcan, enc->id, BRITERENCODER_FUNC_SET_MODE, &mode, 1);
}

HAL_StatusTypeDef briterencoder_set_auto_period_us(CAN_HandleTypeDef *hcan,
                                                   briterencoder_t *enc,
                                                   uint16_t period_us)
{
    uint8_t payload[2];
    if (!enc) {
        return HAL_ERROR;
    }
    briterencoder_u16_to_le(period_us, payload);
    enc->last_func = BRITERENCODER_FUNC_SET_AUTO_PERIOD;
    return briterencoder_send(hcan, enc->id, BRITERENCODER_FUNC_SET_AUTO_PERIOD, payload, 2);
}

HAL_StatusTypeDef briterencoder_set_zero(CAN_HandleTypeDef *hcan, briterencoder_t *enc)
{
    uint8_t payload = 0x00;
    if (!enc) {
        return HAL_ERROR;
    }
    enc->last_func = BRITERENCODER_FUNC_SET_ZERO;
    return briterencoder_send(hcan, enc->id, BRITERENCODER_FUNC_SET_ZERO, &payload, 1);
}

HAL_StatusTypeDef briterencoder_set_direction(CAN_HandleTypeDef *hcan, briterencoder_t *enc, uint8_t direction)
{
    if (!enc) {
        return HAL_ERROR;
    }
    enc->last_func = BRITERENCODER_FUNC_SET_DIR;
    return briterencoder_send(hcan, enc->id, BRITERENCODER_FUNC_SET_DIR, &direction, 1);
}

HAL_StatusTypeDef briterencoder_set_speed_calc_time_ms(CAN_HandleTypeDef *hcan,
                                                       briterencoder_t *enc,
                                                       uint16_t time_ms)
{
    uint8_t payload[2];
    if (!enc) {
        return HAL_ERROR;
    }
    briterencoder_u16_to_le(time_ms, payload);
    enc->last_func = BRITERENCODER_FUNC_SET_VEL_TIME;
    return briterencoder_send(hcan, enc->id, BRITERENCODER_FUNC_SET_VEL_TIME, payload, 2);
}

HAL_StatusTypeDef briterencoder_set_midpoint(CAN_HandleTypeDef *hcan, briterencoder_t *enc)
{
    uint8_t payload = 0x01;
    if (!enc) {
        return HAL_ERROR;
    }
    enc->last_func = BRITERENCODER_FUNC_SET_MID;
    return briterencoder_send(hcan, enc->id, BRITERENCODER_FUNC_SET_MID, &payload, 1);
}

HAL_StatusTypeDef briterencoder_set_value(CAN_HandleTypeDef *hcan, briterencoder_t *enc, uint32_t value)
{
    uint8_t payload[4];
    if (!enc) {
        return HAL_ERROR;
    }
    briterencoder_u32_to_le(value, payload);
    enc->last_func = BRITERENCODER_FUNC_SET_VALUE;
    return briterencoder_send(hcan, enc->id, BRITERENCODER_FUNC_SET_VALUE, payload, 4);
}

HAL_StatusTypeDef briterencoder_set_five_turns(CAN_HandleTypeDef *hcan, briterencoder_t *enc)
{
    uint8_t payload = 0x01;
    if (!enc) {
        return HAL_ERROR;
    }
    enc->last_func = BRITERENCODER_FUNC_SET_FIVE_TURNS;
    return briterencoder_send(hcan, enc->id, BRITERENCODER_FUNC_SET_FIVE_TURNS, &payload, 1);
}

void briterencoder_on_can_rx(briterencoder_t *enc, uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
    if (!enc || !data || dlc < 3U) {
        return;
    }
    if (std_id != enc->id && std_id != enc->pending_id) {
        return;
    }
    if (data[0] > dlc) {
        return;
    }

    uint8_t func = data[2];
    enc->last_func = func;
    enc->last_rx_ms = HAL_GetTick();

    if (func == BRITERENCODER_FUNC_READ_VALUE && dlc >= 7U) {
        enc->value = briterencoder_u32_from_le(&data[3]);
        return;
    }
    if (func == BRITERENCODER_FUNC_READ_VELOCITY && dlc >= 7U) {
        enc->velocity = briterencoder_i32_from_le(&data[3]);
        return;
    }

    if (dlc >= 4U) {
        enc->last_status = data[3];
        if (func == BRITERENCODER_FUNC_SET_ID && enc->last_status == 0U && enc->pending_id) {
            enc->id = enc->pending_id;
            enc->pending_id = 0U;
        }
    }
}
