/*
 * feederTask.c
 *
 *  Created on: Jan 17, 2026
 *      Author: Barry
 */

#include "main.h"
#include "bsp_damiao.h"
#include <math.h>
#include <stdbool.h>


extern dm_motor_t dm_pitch_motor;
extern CAN_HandleTypeDef hcan1;

extern TIM_HandleTypeDef htim5;

extern float target_rpm;          /* Desired speed setpoint in RPM */
extern float commanded_rpm;       /* Internally ramped RPM command */
extern float measured_rpm;
extern float pid_int;
extern float pid_prev_err;

extern float target_position;
extern float current_position;
extern int   round_counter;
extern float tolerance;
extern float prev_pos;

extern float pos_kp;
extern float pos_kd;



#include <stdint.h>


typedef enum {
  POS_1,
  UNLOAD_1,
  POS_2,
  UNLOAD_2,
  POS_3,
  UNLOAD_3
} feeder_state_t;

feeder_state_t FeederState = POS_1;

#define UNLOAD_MS  500;
#define KP_SET 10.0;
#define KD_SET 1.5;
static const float LOAD_ANGLE = 0.0f;
static const float UNLOAD_ANGLE = 270.0f;
#define UNLOAD_DURATION  1000
#define LOOP_DELAY 100;

bool ready = false;

#define DEG_TO_RAD        (3.14159265358979323846f / 180.0f)

#define POS_DEG_INC       90.0f

#define POS_DEG_TOL	      10.0f
#define POS_DEG_1         0.0f
#define POS_DEG_2         (POS_DEG_1 + POS_DEG_INC)
#define POS_DEG_3         (POS_DEG_2 + POS_DEG_INC)

#define POS_RAD_TOL       (POS_DEG_TOL * DEG_TO_RAD)
#define POS_RAD_1         (POS_DEG_1 * DEG_TO_RAD)
#define POS_RAD_2         (POS_DEG_2 * DEG_TO_RAD)
#define POS_RAD_3         (POS_DEG_3 * DEG_TO_RAD)

float servo_1 = 0.0f;
float servo_2 = 0.0f;
float servo_3 = 0.0f;
float pos_angle = 0.0f;



void servo_set_angle(TIM_HandleTypeDef *htim,
                     uint32_t channel,
                     float angle_deg)
{
    const float MIN_ANGLE = 0.0f;
    const float MAX_ANGLE = 270.0f;
    const float MIN_PULSE = 500.0f;   // µs
    const float MAX_PULSE = 2500.0f;  // µs

    // clamp angle
    if (angle_deg < MIN_ANGLE) angle_deg = MIN_ANGLE;
    if (angle_deg > MAX_ANGLE) angle_deg = MAX_ANGLE;

    // angle → pulse width (µs)
    float pulse_us = MIN_PULSE +
                     (angle_deg - MIN_ANGLE) *
                     (MAX_PULSE - MIN_PULSE) /
                     (MAX_ANGLE - MIN_ANGLE);

    // write directly to timer CCR
    __HAL_TIM_SET_COMPARE(htim, channel, (uint16_t)pulse_us);
}




void FeederTask(void *argument)
{
  /* USER CODE BEGIN FeederTask */
	servo_set_angle(&htim5, TIM_CHANNEL_4, LOAD_ANGLE);
	servo_set_angle(&htim5, TIM_CHANNEL_3, LOAD_ANGLE);
	servo_set_angle(&htim5, TIM_CHANNEL_2, LOAD_ANGLE);

	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    vTaskDelay(1000);
    dm4310_motor_init();
    dm_pitch_motor.ctrl.pos_set = 0;
	dm_pitch_motor.ctrl.vel_set = 0;
	dm_pitch_motor.ctrl.kp_set = KP_SET;
	dm_pitch_motor.ctrl.kd_set = KD_SET;
	dm_pitch_motor.ctrl.tor_set = 0;
    TickType_t loop_period_ms = 10;    /* Control loop period */
  /* Infinite loop */
  for(;;)
  {
	  switch (FeederState) {

	  case POS_1:
	      servo_1 = LOAD_ANGLE;
	      servo_2 = LOAD_ANGLE;
	      servo_3 = LOAD_ANGLE;
	      pos_angle = POS_RAD_1;

	      if ((dm_pitch_motor.para.pos > (POS_RAD_1 - POS_RAD_TOL)) &&
	          (dm_pitch_motor.para.pos < (POS_RAD_1 + POS_RAD_TOL)) &&
			   ready)
	      {
	    	  ready = false;
	          FeederState = UNLOAD_1;
	      }
	      break;

	  case UNLOAD_1:
	      loop_period_ms = UNLOAD_DURATION;

	      servo_1 = UNLOAD_ANGLE;
	      servo_2 = LOAD_ANGLE;
	      servo_3 = LOAD_ANGLE;
	      pos_angle = POS_RAD_1;

	      if (ready) {
	          ready = false;
	          loop_period_ms = LOOP_DELAY;
	          FeederState = POS_2;
	      }
	      break;

	  case POS_2:
	      servo_1 = LOAD_ANGLE;
	      servo_2 = LOAD_ANGLE;
	      servo_3 = LOAD_ANGLE;
	      pos_angle = POS_RAD_2;

	      if ((dm_pitch_motor.para.pos > (POS_RAD_2 - POS_RAD_TOL)) &&
	          (dm_pitch_motor.para.pos < (POS_RAD_2 + POS_RAD_TOL))&&
			  ready)
	      {
	    	  ready = false;
	          FeederState = UNLOAD_2;
	      }
	      break;

	  case UNLOAD_2:
	      loop_period_ms = UNLOAD_DURATION;

	      servo_1 = LOAD_ANGLE;
	      servo_2 = UNLOAD_ANGLE;
	      servo_3 = LOAD_ANGLE;
	      pos_angle = POS_RAD_2;

	      if (ready) {
	          ready = false;
	          loop_period_ms = LOOP_DELAY;
	          FeederState = POS_3;
	      }
	      break;

	  case POS_3:
	      servo_1 = LOAD_ANGLE;
	      servo_2 = LOAD_ANGLE;
	      servo_3 = LOAD_ANGLE;
	      pos_angle = POS_RAD_3;

	      if ((dm_pitch_motor.para.pos > (POS_RAD_3 - POS_RAD_TOL)) &&
	          (dm_pitch_motor.para.pos < (POS_RAD_3 + POS_RAD_TOL))&&
			  ready)
	      {
	    	  ready = false;
	          FeederState = UNLOAD_3;
	      }
	      break;

	  case UNLOAD_3:
	      loop_period_ms = UNLOAD_DURATION;

	      servo_1 = LOAD_ANGLE;
	      servo_2 = LOAD_ANGLE;
	      servo_3 = UNLOAD_ANGLE;
	      pos_angle = POS_RAD_3;

	      if (ready) {
	          ready = false;
	          loop_period_ms = LOOP_DELAY;
	          FeederState = POS_1;
	      }
	      break;

	  default:
	      loop_period_ms = LOOP_DELAY;
	      FeederState = POS_1;
	      ready = false;
	      break;
	  }



	//move motors
	dm_pitch_motor.ctrl.pos_set = pos_angle;
	servo_set_angle(&htim5, TIM_CHANNEL_4, servo_3);
	servo_set_angle(&htim5, TIM_CHANNEL_3, servo_2);
	servo_set_angle(&htim5, TIM_CHANNEL_2, servo_1);
    dm4310_ctrl_send(&hcan1, &dm_pitch_motor);
    vTaskDelay(loop_period_ms);
  }
  /* USER CODE END FeederTask */
}


