/*
 * launcher.c
 *
 *  Created on: Jan 19, 2026
 *      Author: Barry Ang
 */


#include <DART_CONFIG.h>
#include "main.h"
#include "bsp_damiao.h"
#include "launcherTask.h"
#include "PID.h"
#include <math.h>
#include <stdbool.h>
#include "../ui_interface.h"
#include "remote_control.h"



 /* Feeder--------------------------------------------------------*/
typedef enum {
	POS_1_0,
	POS_1_1,
	UNLOAD_1,
	POS_2_0,
	POS_2_1,
	UNLOAD_2,
	POS_3_0,
	POS_3_1,
	UNLOAD_3,
	POS_4_0
} feeder_state_t;

feeder_state_t FeederState = POS_1_0;

#define UNLOAD_MS  500
#define KP_SET     10.0f
#define KD_SET     1.5f
static const float LOAD_ANGLE = 180.0f;
static const float UNLOAD_ANGLE = 90.0f;
#define UNLOAD_DURATION  1000
#define LOOP_DELAY       0
TickType_t loop_period_ms;

bool ready = false;
bool done = false;

#define DEG_TO_RAD        (3.14159265358979323846f / 180.0f)

#define POS_DEG_INC       60.0f

#define POS_DEG_TOL	      10.0f
#define POS_DEG_1_0		  0.0f
#define POS_DEG_1_1         (POS_DEG_1_0 + POS_DEG_INC)
#define POS_DEG_2_0         (POS_DEG_1_1 + POS_DEG_INC)
#define POS_DEG_2_1         (POS_DEG_2_0 + POS_DEG_INC)
#define POS_DEG_3_0         (POS_DEG_2_1 + POS_DEG_INC)
#define POS_DEG_3_1         (POS_DEG_3_0 + POS_DEG_INC)
#define POS_DEG_4_0         (POS_DEG_3_1 + POS_DEG_INC)

#define POS_RAD_TOL       	(POS_DEG_TOL * DEG_TO_RAD)
#define POS_RAD_1_0         (POS_DEG_1_0 * DEG_TO_RAD)
#define POS_RAD_1_1         (POS_DEG_1_1 * DEG_TO_RAD)
#define POS_RAD_2_0         (POS_DEG_2_0 * DEG_TO_RAD)
#define POS_RAD_2_1         (POS_DEG_2_1 * DEG_TO_RAD)
#define POS_RAD_3_0         (POS_DEG_3_0 * DEG_TO_RAD)
#define POS_RAD_3_1         (POS_DEG_3_1 * DEG_TO_RAD)
#define POS_RAD_4_0			(POS_DEG_4_0 * DEG_TO_RAD)

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

 /*--------------------------------------------------------*/

/* Launcher--------------------------------------------------------*/

typedef enum {
LAUNCHER_WAIT,
SEEK_LOCK_CW,
LOCKED,
FEED,
REVERSE_TO_WEIGHT,
HOLD
} launcher_state_t;

launcher_state_t LauncherState = LAUNCHER_WAIT;

float target = 7.0f, tol = 0.5f;

enum LaunchernFeederState {
	WAIT = 0,
	FIRING = 1
};

int dart_count;
int dart_fire;
enum LaunchernFeederState LnF_state;

/* HOLD: release servo sequence (release angle -> delay -> rest angle) */
static uint8_t hold_release_phase = 0;  /* 0=need release, 1=waiting, 2=done */
static TickType_t hold_release_ticks = 0;

#define KP_SET_FEEDER 10.0f
#define KD_SET_FEEDER 1.5f

extern dm_motor_t dm_launching_motor;
extern dm_motor_t dm_feeder_motor;
extern RC_ctrl_t rc_ctrl;
extern CAN_HandleTypeDef hcan1;
bool lock = false;
bool op_sen_feeder = false;
bool op_sen_launcher_limits = false;
PID reverse_to_weight_pid;


void LauncherTask(void *argument)
{
  /* USER CODE BEGIN StartTask04 */

	dart_count = 4;
	dart_fire = 0;
	LnF_state = WAIT;

	FeederState = POS_1_0;
	LauncherState = LAUNCHER_WAIT;
	NVIC_DisableIRQ(EXTI9_5_IRQn);
	PID_Init(&reverse_to_weight_pid, REVERSE_TORQUE_PID_KP, REVERSE_TORQUE_PID_KI, REVERSE_TORQUE_PID_KD, REVERSE_V_MIN, REVERSE_V_MAX);

   /* Infinite loop */
   for(;;)
   {
	#if TESTING == 1
		testing_launcher();
	#else
		switch (LnF_state){
		case WAIT:
			dm_launching_motor.ctrl.pos_set = 0;
			dm_launching_motor.ctrl.vel_set = 0;
			dm_launching_motor.ctrl.kp_set  = LAUNCHER_IDLE_KP;
			dm_launching_motor.ctrl.kd_set  = LAUNCHER_IDLE_KD;
			dm_launching_motor.ctrl.tor_set = LAUNCHER_IDLE_TOR;

			dm_feeder_motor.ctrl.pos_set = 0;
			dm_feeder_motor.ctrl.vel_set = 0;
			dm_feeder_motor.ctrl.kp_set  = KP_SET_FEEDER;
			dm_feeder_motor.ctrl.kd_set  = KD_SET_FEEDER;
			dm_feeder_motor.ctrl.tor_set = 0;
			
			dart_fire = 0;
			dart_count = 4;

			servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO1_CHANNEL, LOAD_ANGLE);
			servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO2_CHANNEL, LOAD_ANGLE);
			servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO3_CHANNEL, LOAD_ANGLE);
			servo_set_angle(&LAUNCHER_SERVO_TIM, LAUNCHER_SERVO_CHANNEL, LOAD_ANGLE);
			HAL_TIM_PWM_Start(&FEEDER_SERVO_TIM, FEEDER_SERVO1_CHANNEL);
			HAL_TIM_PWM_Start(&FEEDER_SERVO_TIM, FEEDER_SERVO2_CHANNEL);
			HAL_TIM_PWM_Start(&FEEDER_SERVO_TIM, FEEDER_SERVO3_CHANNEL);
			HAL_TIM_PWM_Start(&LAUNCHER_SERVO_TIM, LAUNCHER_SERVO_CHANNEL);
			set_var_firing(false);

			//TODO make the button on changable for different numbers of dart
			if (ui_interface_get_auto_dart_count() > 0){
				LnF_state = FIRING;
				set_var_firing(true);
			}
			break;
		case FIRING:

		/*Launcher State Machine--------------------------------------------------------*/
			switch (LauncherState){
			case LAUNCHER_WAIT:

			dm_launching_motor.ctrl.pos_set = 0;
			dm_launching_motor.ctrl.vel_set = 0;
			dm_launching_motor.ctrl.kp_set  = LAUNCHER_WAIT_KP;
			dm_launching_motor.ctrl.kd_set  = LAUNCHER_WAIT_KD;
			dm_launching_motor.ctrl.tor_set = LAUNCHER_WAIT_TOR;

			if (dart_count == 4){
				LauncherState = REVERSE_TO_WEIGHT;
			} else if (dart_count > 0) {
				LauncherState = SEEK_LOCK_CW;
			} else {
				ui_interface_set_auto_dart_count(0);
				LauncherState = LAUNCHER_WAIT;
				LnF_state = WAIT;
			}
			break;

			case SEEK_LOCK_CW:

			dm_launching_motor.ctrl.pos_set = 0;
			dm_launching_motor.ctrl.vel_set = LAUNCHER_SEEK_LOCK_VEL;
			dm_launching_motor.ctrl.kp_set  = LAUNCHER_SEEK_LOCK_KP;
			dm_launching_motor.ctrl.kd_set  = LAUNCHER_SEEK_LOCK_KD;
			dm_launching_motor.ctrl.tor_set = LAUNCHER_SEEK_LOCK_TOR;
			if (lock){
				lock = false;
				LauncherState = LOCKED;
				__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
				NVIC_EnableIRQ(EXTI9_5_IRQn);
				ready = true;
			}
			break;

			case LOCKED:
			if (((FeederState == POS_1_1) || (FeederState == POS_2_1) || (FeederState == POS_3_1)) && done){
				dm_launching_motor.ctrl.pos_set = 0;
				dm_launching_motor.ctrl.vel_set = LAUNCHER_LOCKED_VEL;
				dm_launching_motor.ctrl.kp_set  = LAUNCHER_LOCKED_KP;
				dm_launching_motor.ctrl.kd_set  = LAUNCHER_LOCKED_KD;
				dm_launching_motor.ctrl.tor_set = LAUNCHER_LOCKED_TOR;
			} else {
				dm_launching_motor.ctrl.pos_set = 0;
				dm_launching_motor.ctrl.vel_set = 0;
				dm_launching_motor.ctrl.kp_set  = LAUNCHER_LOCKED_KP;
				dm_launching_motor.ctrl.kd_set  = LAUNCHER_LOCKED_KD;
				dm_launching_motor.ctrl.tor_set = LAUNCHER_LOCKED_TOR;
			}
			if (op_sen_feeder){
				LauncherState = FEED;

			}
			break;
			case FEED:
				dm_launching_motor.ctrl.pos_set = 0;
				dm_launching_motor.ctrl.vel_set = 0;
				dm_launching_motor.ctrl.kp_set  = LAUNCHER_FEED_KP;
				dm_launching_motor.ctrl.kd_set  = LAUNCHER_FEED_KD;
				dm_launching_motor.ctrl.tor_set = LAUNCHER_FEED_TOR;


				if ((FeederState == POS_2_0  || FeederState == POS_3_0  || FeederState == POS_4_0) && done ){
					LauncherState = REVERSE_TO_WEIGHT;
					NVIC_DisableIRQ(EXTI9_5_IRQn);
					op_sen_feeder = false;
				}
				break;

			case REVERSE_TO_WEIGHT: {
				#if LAUNCH_CONTROL == 0
				/* Torque PID: e = T_target - T_measured -> v_cmd (motor speed command) */
				PID_Compute(&reverse_to_weight_pid, target, dm_launching_motor.para.tor, 0.01f, 0.0f);
				dm_launching_motor.ctrl.pos_set = 0;
				dm_launching_motor.ctrl.vel_set = reverse_to_weight_pid.output;
				dm_launching_motor.ctrl.kp_set  = LAUNCHER_REVERSE_KP;
				dm_launching_motor.ctrl.kd_set  = LAUNCHER_REVERSE_KD;
				dm_launching_motor.ctrl.tor_set = LAUNCHER_REVERSE_TOR;
				if (dm_launching_motor.para.tor > target - tol && dm_launching_motor.para.tor < target + tol) {
					LauncherState = HOLD;
				}
				break;
				#else if LAUNCH_CONTROL == 1
					dm_launching_motor.ctrl.pos_set = 0;
					dm_launching_motor.ctrl.vel_set = LAUNCHER_REVERSE_VEL;
					dm_launching_motor.ctrl.kp_set  = LAUNCHER_REVERSE_KP;
					dm_launching_motor.ctrl.kd_set  = LAUNCHER_REVERSE_KD;
					dm_launching_motor.ctrl.tor_set = LAUNCHER_REVERSE_TOR;

					if (dm_launching_motor.para.tor > target) {
						LauncherState = HOLD;
					}
				#endif
			}

			case HOLD: {
				dm_launching_motor.ctrl.pos_set = 0;
				dm_launching_motor.ctrl.vel_set = 0;
				dm_launching_motor.ctrl.kp_set  = LAUNCHER_HOLD_KP;
				dm_launching_motor.ctrl.kd_set  = LAUNCHER_HOLD_KD;
				dm_launching_motor.ctrl.tor_set = LAUNCHER_HOLD_TOR;
				servo_set_angle(&LAUNCHER_SERVO_TIM, LAUNCHER_SERVO_CHANNEL, LAUNCHER_SERVO_ANGLE_RELEASE);

				if (HAL_GPIO_ReadPin(LOCK_GPIO_Port, LOCK_Pin)){
					servo_set_angle(&LAUNCHER_SERVO_TIM, LAUNCHER_SERVO_CHANNEL, LAUNCHER_SERVO_ANGLE_REST);
					LauncherState = WAIT;
					dart_count--;
				}
				break;
			}
			}
		

	/*Feeder State Machine--------------------------------------------------------*/
			switch (FeederState){
			  case POS_1_0:
				  loop_period_ms = LOOP_DELAY;
				  servo_1 = LOAD_ANGLE;
				  servo_2 = LOAD_ANGLE;
				  servo_3 = LOAD_ANGLE;
				  pos_angle = POS_RAD_1_0;

				  if ((dm_feeder_motor.para.pos > (POS_RAD_1_0 - POS_RAD_TOL)) &&
				  	(dm_feeder_motor.para.pos < (POS_RAD_1_0 + POS_RAD_TOL))){
					done = true;
				  } else {
					done = false;
				  }
				  
				  if (ready && done){
					  FeederState = POS_1_1;
					  ready = false;
				  }
				  break;
			  case POS_1_1:
				  loop_period_ms = LOOP_DELAY;
			      servo_1 = LOAD_ANGLE;
			      servo_2 = LOAD_ANGLE;
			      servo_3 = LOAD_ANGLE;
			      pos_angle = POS_RAD_1_1;

			      if ((dm_feeder_motor.para.pos > (POS_RAD_1_1 - POS_RAD_TOL)) &&
			          (dm_feeder_motor.para.pos < (POS_RAD_1_1 + POS_RAD_TOL)))
			      {
			    	  done = true;
			      } else {
			    	  done = false;
			      }

			      if ((LauncherState == FEED) && done){
 			          FeederState = UNLOAD_1;
			      }
			      break;

			  case UNLOAD_1:
				  loop_period_ms = UNLOAD_DURATION;
			      servo_1 = UNLOAD_ANGLE;
			      servo_2 = LOAD_ANGLE;
			      servo_3 = LOAD_ANGLE;
			      pos_angle = POS_RAD_1_1;
			      break;

			  case POS_2_0:
				  loop_period_ms = LOOP_DELAY;
				  servo_1 = LOAD_ANGLE;
				  servo_2 = LOAD_ANGLE;
				  servo_3 = LOAD_ANGLE;
				  pos_angle = POS_RAD_2_0;

				  if ((dm_feeder_motor.para.pos > (POS_RAD_2_0 - POS_RAD_TOL)) &&
				  	(dm_feeder_motor.para.pos < (POS_RAD_2_0 + POS_RAD_TOL))){
					done = true;
				  } else {
					done = false;
				  }
                
				  if (ready && done){
					  FeederState = POS_2_1;
					  ready = false;
				  }
				  break;

			  case POS_2_1:
				  loop_period_ms = LOOP_DELAY;
			      servo_1 = LOAD_ANGLE;
			      servo_2 = LOAD_ANGLE;
			      servo_3 = LOAD_ANGLE;
			      pos_angle = POS_RAD_2_1;

			      if ((dm_feeder_motor.para.pos > (POS_RAD_2_1 - POS_RAD_TOL)) &&
				   (dm_feeder_motor.para.pos < (POS_RAD_2_1 + POS_RAD_TOL)) &&
				   (LauncherState == FEED)){
			    	  FeederState = UNLOAD_2;
				  }
			      break;

			  case UNLOAD_2:
			      loop_period_ms = UNLOAD_DURATION;

			      servo_1 = LOAD_ANGLE;
			      servo_2 = UNLOAD_ANGLE;
			      servo_3 = LOAD_ANGLE;
			      pos_angle = POS_RAD_2_1;
			      FeederState = POS_3_0;
			      break;

			  case POS_3_0:
				  loop_period_ms = LOOP_DELAY;
			      servo_1 = LOAD_ANGLE;
			      servo_2 = LOAD_ANGLE;
			      servo_3 = LOAD_ANGLE;
			      pos_angle = POS_RAD_3_0;

			      if ((dm_feeder_motor.para.pos > (POS_RAD_3_0 - POS_RAD_TOL)) &&
			          (dm_feeder_motor.para.pos < (POS_RAD_3_0 + POS_RAD_TOL)) &&
					  (LauncherState == FEED))
			      {
			          FeederState = UNLOAD_3;
			      }
			      break;

			  case UNLOAD_3:
			      loop_period_ms = UNLOAD_DURATION;
			      servo_1 = LOAD_ANGLE;
			      servo_2 = LOAD_ANGLE;
			      servo_3 = UNLOAD_ANGLE;
			      pos_angle = POS_RAD_3_0;

			      FeederState = POS_4_0;
			      break;

			  case POS_4_0:
				  loop_period_ms = LOOP_DELAY;
				  servo_1 = LOAD_ANGLE;
				  servo_2 = LOAD_ANGLE;
				  servo_3 = LOAD_ANGLE;
				  pos_angle = POS_RAD_4_0;

				  if ((dm_feeder_motor.para.pos > (POS_RAD_4_0 - POS_RAD_TOL)) &&
					(dm_feeder_motor.para.pos < (POS_RAD_4_0 + POS_RAD_TOL))){
					done = true;
				  } else {
					done = false;
				  }

				  if (ready && done){
					  ready = false;
					  FeederState = POS_1_0;
				  }
				  break;

			  default:
			      FeederState = POS_1_0	;
			      ready = false;
			      break;
			}



			dm_feeder_motor.ctrl.pos_set = pos_angle;
			servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO3_CHANNEL, servo_3);
			servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO2_CHANNEL, servo_2);
			servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO1_CHANNEL, servo_1);
			dm_ctrl_send(&hcan1, &dm_feeder_motor);
			vTaskDelay(1);
			dm_ctrl_send(&hcan1, &dm_launching_motor);

			vTaskDelay(loop_period_ms);
		}
	#endif
	}
}

void testing_launcher(void){
	if (rc_ctrl.rc.s[0] != 1){
		if (rc_ctrl.rc.s[1] == 1){
			servo_set_angle(&LAUNCHER_SERVO_TIM, LAUNCHER_SERVO_CHANNEL, LAUNCHER_SERVO_ANGLE_RELEASE);
		} else {
			servo_set_angle(&LAUNCHER_SERVO_TIM, LAUNCHER_SERVO_CHANNEL, LAUNCHER_SERVO_ANGLE_REST);
		}

		float velocity = rc_ctrl.rc.ch[1]/10;

		if (velocity > MAX_VELOCITY) {
			velocity = MAX_VELOCITY;
		} else if (velocity < -MAX_VELOCITY) {
			velocity = -MAX_VELOCITY;
		}

		dm_launching_motor.ctrl.pos_set = 0;
		dm_launching_motor.ctrl.vel_set = velocity;
		dm_launching_motor.ctrl.kp_set  = LAUNCHER_IDLE_KP;
		dm_launching_motor.ctrl.kd_set  = LAUNCHER_TEST_KD;
		dm_launching_motor.ctrl.tor_set = LAUNCHER_IDLE_TOR;
		dm_ctrl_send(&hcan1, &dm_launching_motor);
	} else {
		dm_launching_motor.ctrl.pos_set = 0;
		dm_launching_motor.ctrl.vel_set = 0;
		dm_launching_motor.ctrl.kp_set  = LAUNCHER_IDLE_KP;
		dm_launching_motor.ctrl.kd_set  = LAUNCHER_TEST_KD;
		dm_launching_motor.ctrl.tor_set = LAUNCHER_IDLE_TOR;
	}
}
