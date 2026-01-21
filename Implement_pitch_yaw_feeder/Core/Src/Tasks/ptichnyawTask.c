/*
 * ptichnyawtTask.c
 *
 *  Created on: Jan 18, 2026
 *      Author: Barry Ang
 */
#include "main.h"
#include "bsp_damiao.h"
#include <math.h>
#include <stdbool.h>
#include "briterencoder.h"



static inline float deg_to_rad(float deg)
{
    return deg * 0.017453292519943295f;
}

#define YAW_KP_SET 10.0f
#define YAW_KD_SET 1.5f

#define FWD_GPIO_Port GPIOC
#define FWD_Pin       GPIO_PIN_3

#define BWD_GPIO_Port GPIOB
#define BWD_Pin       GPIO_PIN_1

extern TIM_HandleTypeDef htim3;

#define PWM_TIM     htim3
#define PWM_CHANNEL TIM_CHANNEL_3

extern dm_motor_t dm_yaw_motor;
extern CAN_HandleTypeDef hcan1;

volatile briterencoder_t pitch_encoder;

float yaw_angle = 0.0f;
float pitch_deg = 30.0f; // example command in-range

#define DEG2RAD (0.01745329251994329577f)

#define ANGLE_MIN_DEG 0.0f
#define ANGLE_MAX_DEG 360.0f

#define ANGLE_MIN_RAD (ANGLE_MIN_DEG * DEG2RAD)
#define ANGLE_MAX_RAD (ANGLE_MAX_DEG * DEG2RAD)
#define PWM_MAX_COUNTS 4199u   // if ARR = 4199


static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* Wrap to [-pi, pi] */
static float wrap_to_pi(float x)
{
    while (x >  (float)M_PI) x -= 2.0f * (float)M_PI;
    while (x < -(float)M_PI) x += 2.0f * (float)M_PI;
    return x;
}

static inline void Motor_SetDir(bool forward)
{
    // Replace with your GPIO write:
    // HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    (void)forward;
}

static inline void Motor_Stop(void)
{
    // PWM to 0
    __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_CHANNEL, 0);

    // both direction pins low
    HAL_GPIO_WritePin(FWD_GPIO_Port, FWD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BWD_GPIO_Port, BWD_Pin, GPIO_PIN_RESET);
}

/* Set direction safely (never both high) */
static inline void Motor_SetDirection(bool forward)
{
    if (forward) {
        HAL_GPIO_WritePin(BWD_GPIO_Port, BWD_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(FWD_GPIO_Port, FWD_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(FWD_GPIO_Port, FWD_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BWD_GPIO_Port, BWD_Pin, GPIO_PIN_SET);
    }
}

static inline void Motor_SetPwmCounts(uint16_t duty)
{
    if (duty > PWM_MAX_COUNTS) duty = PWM_MAX_COUNTS;
    __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_CHANNEL, duty);
}

/* --------- Position control (20–45 deg) --------- */
#define DEG2RAD 0.01745329251994329577f
#define ANGLE_MIN_RAD (0.0f * DEG2RAD)
#define ANGLE_MAX_RAD (360.0f * DEG2RAD)

//void Motor_PositionControlStep(PID *pos_pid,
//                               float target_angle_rad,
//                               float dt,
//                               float deadzone_rad)
//{
//    // 1) clamp command to allowed window
//    float target = clampf(target_angle_rad, ANGLE_MIN_RAD, ANGLE_MAX_RAD);
//
//    // 2) read encoder
//    float angle = briterencoder_u32_to_rad(pitch_encoder.value);
//
////    // 3) if you ever go out-of-bounds, force target back in-range
//    if (angle < ANGLE_MIN_RAD) target = ANGLE_MIN_RAD;
//    if (angle > ANGLE_MAX_RAD) target = ANGLE_MAX_RAD;
////
////    // 4) error (shortest path)
//    float err = wrap_to_pi(target - angle);
//
//    // 5) drive your PID using the wrapped error (setpoint trick)
//    PID_Compute(pos_pid, err, 0.0f, dt, deadzone_rad);
//    float u = pos_pid->output; // signed command
//
//    // 6) stop near target to avoid buzzing
//    if (fabsf(err) <= deadzone_rad) {
//        Motor_Stop();
//        return;
//    }
//
//    // 7) direction + magnitude -> PWM
//    bool forward = (u >= 0.0f);
//    Motor_SetDirection(forward);
//
//    float mag = fabsf(u);
//    if (mag > (float)PWM_MAX_COUNTS) mag = (float)PWM_MAX_COUNTS;
//    Motor_SetPwmCounts((uint16_t)mag);
//}


void PitchnYawTask(void *argument)
{
    /* USER CODE BEGIN YawTask */
	briterencoder_init(&pitch_encoder, 3);

    dm_yaw_motor.ctrl.pos_set = 0.0f;
    dm_yaw_motor.ctrl.vel_set = 0.0f;
    dm_yaw_motor.ctrl.kp_set  = YAW_KP_SET;
    dm_yaw_motor.ctrl.kd_set  = YAW_KD_SET;
    dm_yaw_motor.ctrl.tor_set = 0.0f;


    PID pos = {0};
	pos.kp = 200.0f;
	pos.ki = 0.0f;
	pos.kd = 10.0f;
	pos.min_output = -(float)PWM_MAX_COUNTS;
	pos.max_output =  (float)PWM_MAX_COUNTS;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	HAL_TIM_PWM_Start(&PWM_TIM, PWM_CHANNEL);
	Motor_Stop();

	float dt = 0.001f;                 // 1 kHz control loop
	float deadzone = 0.25f * DEG2RAD;  // 0.25 deg
	briterencoder_set_mode(&hcan1, &pitch_encoder, 0xAA);
	briterencoder_set_auto_period_us(&hcan1, &pitch_encoder, 100);

    /* Infinite loop */
    for (;;)
    {
//    	briterencoder_read_value(&hcan1, &pitch_encoder);
//    	Motor_PositionControlStep(&pos, pitch_deg * DEG2RAD, dt, deadzone);
    	float cur_pitch_rad_angle = briterencoder_u32_to_rad(pitch_encoder.value);
//    	if (cur_pitch_rad_angle > 3.14 + 0.1){
//    		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
//    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
//    	} else if (cur_pitch_rad_angle < 3.14 - 0.1){
//    		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
//    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
//
//    	} else {
//    		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
//    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
//    	}

    	dm_yaw_motor.ctrl.pos_set = deg_to_rad(yaw_angle);
        dm4310_ctrl_send(&hcan1, &dm_yaw_motor);
        osDelay(pdMS_TO_TICKS(1));
    }

    /* USER CODE END YawTask */
}

