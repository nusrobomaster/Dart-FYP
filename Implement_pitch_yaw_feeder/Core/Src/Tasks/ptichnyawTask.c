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
#include "../ui_interface.h"



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

extern volatile dm_motor_t dm_yaw_motor;
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

CascadePID Pitch_PID;



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

void Motor_PositionControlStep(CascadePID *pos_pid,
                               float target_angle_rad,
                               float dt,
                               float deadzone_rad)
{
	PID_CascadeCalc(pos_pid,target_angle_rad, pitch_encoder.value, pitch_encoder.velocity, dt);
}


void PitchnYawTask(void *argument)
{
    /* USER CODE BEGIN YawTask */
	briterencoder_init(&pitch_encoder, 3);
	taskENTER_CRITICAL();
    dm_yaw_motor.ctrl.pos_set = 0.0f;
    dm_yaw_motor.ctrl.vel_set = 0.0f;
    dm_yaw_motor.ctrl.kp_set  = YAW_KP_SET;
    dm_yaw_motor.ctrl.kd_set  = YAW_KD_SET;
    dm_yaw_motor.ctrl.tor_set = 0.0f;
    taskEXIT_CRITICAL();


    PID_Init(&Pitch_PID.outer, 200.0f, 0.0f, 10.0f, 10.0f, 350.0f);
    PID_Init(&Pitch_PID.inner, 200.0f, 0.0f, 10.0f, -PWM_MAX_COUNTS, PWM_MAX_COUNTS);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);

	HAL_TIM_PWM_Start(&PWM_TIM, PWM_CHANNEL);
	Motor_Stop();

//	float dt = 0.001f;                 // 1 kHz control loop

	static TickType_t last = 0;
	TickType_t now = xTaskGetTickCount();

	float dt = (now - last) * portTICK_PERIOD_MS * 0.001f;
	last = now;
	bool velocity = true;


//	briterencoder_set_mode(&hcan1, &pitch_encoder, 0xAA);
//	briterencoder_set_auto_period_us(&hcan1, &pitch_encoder, 100);


//	briterencoder_set_mode(&hcan1, &pitch_encoder, 0x00);
//	briterencoder_read_velocity(&hcan1, &pitch_encoder);
	/* Infinite loop */
    for (;;)
    {
		/* Get set values from UI interface */
		yaw_angle = ui_interface_get_set_yaw();
		pitch_deg = ui_interface_get_set_pitch();
		
		Motor_SetPwmCounts(3000);
		Motor_SetDirection(false);


		/* Get cur values from Sensors */
		float cur_pitch_rad_angle = briterencoder_u32_to_rad(pitch_encoder.value);
		float cur_pitch_deg_angle = cur_pitch_rad_angle / DEG2RAD;
		float cur_yaw_deg_angle = dm_yaw_motor.para.pos / DEG2RAD;
		ui_interface_update_current_values(cur_pitch_deg_angle, cur_yaw_deg_angle);

		/* Send controls to motors */
		taskENTER_CRITICAL();
    	dm_yaw_motor.ctrl.pos_set = deg_to_rad(yaw_angle);
    	taskEXIT_CRITICAL();
        dm4310_ctrl_send(&hcan1, &dm_yaw_motor);

//        if (velocity){
//        	 briterencoder_read_velocity(&hcan1, &pitch_encoder);
//        	 velocity = false;
//        } else {
//
//        	 velocity = true;
//        }
//        briterencoder_read_value(&hcan1, &pitch_encoder);


//        Motor_PositionControlStep(&Pitch_PID, pitch_deg * DEG2RAD, dt, 0);
        osDelay(pdMS_TO_TICKS(1));
    }

    /* USER CODE END YawTask */
}

