/*
 * ptichnyawtTask.c
 *
 *  Created on: Jan 18, 2026
 *      Author: Barry Ang
 */

#include <DART_CONFIG.h>
#include "main.h"
#include "bsp_damiao.h"
#include "pitchnyawTask.h"
#include <math.h>
#include <stdbool.h>
#include "../ui_interface.h"
#include "remote_control.h"
#include <stdint.h>
#include "PID.h"

typedef enum {
    MODE_BREAKING = 0,
    MODE_FORWARD = 1,
    MODE_BACKWARD = 2,
	MODE_COASTING = 3
} Mode_t;

Mode_t mode = MODE_BREAKING;

extern volatile dm_motor_t dm_yaw_motor;
extern CAN_HandleTypeDef hcan1;


float yaw_angle = 0.0f;
float pitch_angle = 30.0f; // example command in-range

PID Pitch_PID;

extern RC_ctrl_t rc_ctrl;

float yaw_threshold_velocity = 4.0f;

extern bool op_sen_yaw_45deg;
extern bool op_sen_yaw_35deg;

extern TIM_HandleTypeDef htim2;

float cur_pitch_deg_angle;
float yaw_velocity;


static inline void Motor_Stop(void)
{
    // PWM to 0
    __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_CHANNEL, 0);

    // both direction pins low
    HAL_GPIO_WritePin(FWD_GPIO_Port, FWD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BWD_GPIO_Port, BWD_Pin, GPIO_PIN_RESET);
}

/* Set direction safely (never both high) */
static inline void Motor_SetDirection(Mode_t direction)
{
  switch (direction) {
	  case MODE_COASTING:
		  HAL_GPIO_WritePin(FWD_GPIO_Port, FWD_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(BWD_GPIO_Port, BWD_Pin, GPIO_PIN_RESET);
		  break;
	  case MODE_FORWARD:
		HAL_GPIO_WritePin(FWD_GPIO_Port, FWD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BWD_GPIO_Port, BWD_Pin, GPIO_PIN_RESET);
		break;
	  case MODE_BACKWARD:
		HAL_GPIO_WritePin(FWD_GPIO_Port, FWD_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BWD_GPIO_Port, BWD_Pin, GPIO_PIN_SET);
		break;
	  case MODE_BREAKING:
		HAL_GPIO_WritePin(FWD_GPIO_Port, FWD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BWD_GPIO_Port, BWD_Pin, GPIO_PIN_SET);
		break;
	  default:
		HAL_GPIO_WritePin(FWD_GPIO_Port, FWD_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BWD_GPIO_Port, BWD_Pin, GPIO_PIN_RESET);
		break;
  }
}

static inline void Motor_SetPwmCounts(uint16_t duty)
{
    if (duty > PWM_MAX_COUNTS) duty = PWM_MAX_COUNTS;
    __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_CHANNEL, duty);
}

#include <stdint.h>

static inline void Motor_SetTorque(float torque_cmd)
{
    float abs_torque;
    uint16_t duty;

    if (torque_cmd > 0.0f) {
        Motor_SetDirection(MODE_FORWARD);
        abs_torque = torque_cmd;
    }
    else if (torque_cmd < 0.0f) {
        Motor_SetDirection(MODE_BACKWARD);
        abs_torque = -torque_cmd;
    }
    else {
        Motor_SetDirection(MODE_COASTING);
        Motor_SetPwmCounts(0);
        return;
    }

    /* Clamp torque magnitude to max allowed */
    if (abs_torque > TORQUE_MAX) {
        abs_torque = TORQUE_MAX;
    }

    /* Map torque to PWM counts */
    duty = (uint16_t)((abs_torque / TORQUE_MAX) * PWM_MAX_COUNTS);

    Motor_SetPwmCounts(duty);
}

void PitchnYawTask(void *argument)
{
    /* USER CODE BEGIN YawTask */

	// Initialize the encoder TIM2 for the pitch motor
	HAL_TIM_Encoder_Start(&htim2,  TIM_CHANNEL_ALL);
	taskENTER_CRITICAL();
#if TESTING | TESTING_WOUT_YAW
	dm_yaw_motor.ctrl.pos_set = 0.0f;
	dm_yaw_motor.ctrl.vel_set = 0.0f;
	dm_yaw_motor.ctrl.kp_set  = 0.0f;
	dm_yaw_motor.ctrl.kd_set  = YAW_KD_SET;
	dm_yaw_motor.ctrl.tor_set = 0.0f;
#else
    dm_yaw_motor.ctrl.pos_set = 0.0f;
    dm_yaw_motor.ctrl.vel_set = 0.0f;
    dm_yaw_motor.ctrl.kp_set  = YAW_KP_SET;
    dm_yaw_motor.ctrl.kd_set  = YAW_KD_SET;
    dm_yaw_motor.ctrl.tor_set = 0.0f;
#endif;
    taskEXIT_CRITICAL();


    /* Pitch single-loop PID: angle -> torque */
    PID_Init(&Pitch_PID, 200.0f, 0.0f, 10.0f, -TORQUE_MAX, TORQUE_MAX);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);

	HAL_TIM_PWM_Start(&PWM_TIM, PWM_CHANNEL);
	Motor_Stop();

	static TickType_t last = 0;

	#if !(TESTING | TESTING_WOUT_YAW)
		yaw_angle = CENTRE_ANGLE;
		dm_yaw_motor.ctrl.pos_set = yaw_angle * DEG2RAD;
		dm_ctrl_send(&hcan1, &dm_yaw_motor);
		while (!op_sen_yaw_35deg || !op_sen_yaw_45deg){
			//need check direction
			Motor_SetTorque(125);
		}
		if (op_sen_yaw_35deg){
			__HAL_TIM_SET_COUNTER(&htim2, 0);
		}

	#endif

	/* Infinite loop */
    for (;;)
    {
		#if TESTING | TESTING_WOUT_YAW
    		testing_pitch_n_yaw();
    	#else

		/* Get set values from UI interface */

    	if (ui_interface_get_selection() == SELECT_RC){
    		//TODO
    		if (rc_ctrl.rc.s[0] != 0){
				yaw_angle += rc_ctrl.rc.ch[2] / 100;
				//NEED TO CHECK FOR INVERSION
				if (yaw_angle > RIGHT_YAW_LIMIT){
					yaw_angle = RIGHT_YAW_LIMIT;
				} else if (yaw_angle < LEFT_YAW_LIMIT) {
					yaw_angle = LEFT_YAW_LIMIT;
				}

				pitch_angle += rc_ctrl.rc.ch[3] / 100;
				if (pitch_angle > UPPER_PITCH_LIMIT){
					pitch_angle = UPPER_PITCH_LIMIT;
				} else if (pitch_angle < LOWER_PITCH_LIMIT){
					pitch_angle = LOWER_PITCH_LIMIT;
				}
				ui_interface_apply_value_direct(SELECT_PITCH_ANGLE, pitch_angle);
				ui_interface_apply_value_direct(SELECT_YAW_ANGLE, yaw_angle);
    		}
    	} else if (ui_interface_get_selection() == SELECT_AUTO){
    		//place holder for vj
    		if (true){
    			// read from the preset values
    			yaw_angle = PRESET_YAW_ANGLE;
    			pitch_angle = PRESET_PITCH_ANGLE;
    		} else {
    			// read from the RPI

    		}
    		ui_interface_apply_value_direct(SELECT_PITCH_ANGLE, pitch_angle);
    		ui_interface_apply_value_direct(SELECT_YAW_ANGLE, yaw_angle);
    	} else {
    		yaw_angle = ui_interface_get_set_yaw();
			pitch_angle = ui_interface_get_set_pitch();
    	}



		/* Get cur values from Sensors */
		// reductiongear ratio is 1:5000 and there is 17 basic pulse per revolution with a 4x ab encoder
		// so 1 revolution = 360 degrees
		// so TIM2->CNT * (360.0f / (17.0f * 4.0f * 5000.0f)) = current pitch angle in degrees
		cur_pitch_deg_angle = TIM2->CNT * (360.0f / (17.0f * 4.0f * 5000.0f)) + LOWER_PITCH_LIMIT;


		// convert the yaw angle from radians to degrees
		float cur_yaw_deg_angle = dm_yaw_motor.para.pos / DEG2RAD;
		ui_interface_update_current_values(cur_pitch_deg_angle, cur_yaw_deg_angle);

		/* Time step for PID (s) */
		TickType_t now = xTaskGetTickCount();
		float dt = (now - last) * portTICK_PERIOD_MS * 0.001f;
		if (dt <= 0.0f) {
			dt = 0.001f;
		}
		last = now;
		/* Single-loop PID: angle (deg) -> torque */
		PID_Compute(&Pitch_PID, pitch_angle, cur_pitch_deg_angle, dt, 0.0f);

		// apply torque command from PID
		Motor_SetTorque(Pitch_PID.output);
		/* Send controls to motors */
    	dm_yaw_motor.ctrl.pos_set = yaw_angle * DEG2RAD;
        dm_ctrl_send(&hcan1, &dm_yaw_motor);


		#endif
    }

    /* USER CODE END YawTask */
}


void testing_pitch_n_yaw(void){
	// Set velocity and direction for the brushed dc motor (pitch motor)
	if (op_sen_yaw_35deg){
		op_sen_yaw_35deg = 0;
		__HAL_TIM_SET_COUNTER(&htim2, 0);
	}

	cur_pitch_deg_angle = TIM2->CNT * (360.0f / (17.0f * 4.0f * 5000.0f)) + LOWER_PITCH_LIMIT;

	//turn on controller
	if (rc_ctrl.rc.s[0] != 1){
		int16_t velocity = rc_ctrl.rc.ch[3];
		if (velocity == 0){
			Motor_SetDirection(MODE_COASTING);
		}
		else if (velocity > 0){
			Motor_SetDirection(MODE_BACKWARD);

		}
		else {
			// negative velocity
			Motor_SetDirection(MODE_FORWARD);
		}
		Motor_SetPwmCounts(abs(velocity*5));

	// Set Velocity for the brushless motor (yaw motor)
		yaw_velocity = rc_ctrl.rc.ch[2] / 5;

		if (yaw_velocity > yaw_threshold_velocity) {
			yaw_velocity = yaw_threshold_velocity;
		}
		else if (yaw_velocity < -yaw_threshold_velocity) {
			yaw_velocity = -yaw_threshold_velocity;
		}

		dm_yaw_motor.ctrl.vel_set = yaw_velocity;
		dm_ctrl_send(&hcan1, &dm_yaw_motor);
	} else {
		// turn off controller
		Motor_SetPwmCounts(0);
		Motor_SetDirection(MODE_COASTING);

		yaw_velocity = 0;
		dm_yaw_motor.ctrl.vel_set = yaw_velocity;
		dm_ctrl_send(&hcan1, &dm_yaw_motor);

	}
}
