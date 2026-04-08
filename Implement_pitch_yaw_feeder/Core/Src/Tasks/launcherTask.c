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
static feeder_state_t FeederState = POS_1_0;

TickType_t loop_period_ms;

float servo_1 = LOAD_ANGLE_1;
float servo_2 = LOAD_ANGLE_2;
float servo_3 = LOAD_ANGLE_3;
float servo_4 = LAUNCHER_SERVO_ANGLE_REST;
float pos_angle = POS_RAD_1_0;
float kp_set_feed =  KP_SET_FEEDER;
float kd_set_feed =  KD_SET_FEEDER;
float pos_rad_3_1_0 = POS_RAD_3_1_0;
float launcher_reverse_tor = LAUNCHER_REVERSE_TOR;
float dist_tol = DIST_TOL;
float pause_3_dist = PAUSE_3_DIST;

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
bool hit_op = false;
static launcher_state_t LauncherState = LAUNCHER_WAIT;
float rev_to_feed = 0.0f;
float target = 6.0f, tol = 0.5f;
enum LaunchernFeederState LnF_state;
int dart_count;
int dart_fire;
extern dm_motor_t dm_launching_motor;
extern dm_motor_t dm_feeder_motor;
extern CAN_HandleTypeDef hcan1;
//bool lock = false;
bool op_sen_feeder = false;
bool op_sen_launcher_limits = false;
float op_endstop_position = 0.0f;
#if LAUNCH_CONTROL == 2
static PID reverse_pos_pid;
#endif

float feeding_max_velocity = 4.0f;
float firing_max_velocity = 10.0f;

float extra_front = PAUSE_3_DIST_PID;

void LauncherTask(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
	bool final = false;
	bool done = false;
	bool ready = false;
	dart_count = 4;
	dart_fire = 0;
	LnF_state = WAIT;



	LauncherState = LAUNCHER_WAIT;
	#if TESTING != 0
	NVIC_DisableIRQ(EXTI9_5_IRQn);
	#endif
	#if LAUNCH_CONTROL == 2
	PID_Init(&reverse_pos_pid, LAUNCHER_REVERSE_POS_KP, LAUNCHER_REVERSE_POS_KI, LAUNCHER_REVERSE_POS_KD, -LAUNCHER_REVERSE_POS_VEL_MAX, LAUNCHER_REVERSE_POS_VEL_MAX);
	#endif
	servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO1_CHANNEL, LOAD_ANGLE_1);
	servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO2_CHANNEL, LOAD_ANGLE_2);
	servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO3_CHANNEL, LOAD_ANGLE_3);
	servo_set_angle(&LAUNCHER_SERVO_TIM, LAUNCHER_SERVO_CHANNEL, LAUNCHER_SERVO_ANGLE_REST);
	HAL_TIM_PWM_Start(&FEEDER_SERVO_TIM, FEEDER_SERVO1_CHANNEL);
	HAL_TIM_PWM_Start(&FEEDER_SERVO_TIM, FEEDER_SERVO2_CHANNEL);
	HAL_TIM_PWM_Start(&FEEDER_SERVO_TIM, FEEDER_SERVO3_CHANNEL);
	HAL_TIM_PWM_Start(&LAUNCHER_SERVO_TIM, LAUNCHER_SERVO_CHANNEL);
	dm_feeder_motor.ctrl.pos_set = 0;
	dm_feeder_motor.ctrl.vel_set = 0;
	dm_feeder_motor.ctrl.kp_set  = kp_set_feed;
	dm_feeder_motor.ctrl.kd_set  = kd_set_feed;
	dm_feeder_motor.ctrl.tor_set = 0;
	dm_launching_motor.ctrl.pos_set = 0;
	dm_launching_motor.ctrl.vel_set = 0;
	dm_launching_motor.ctrl.kp_set  = LAUNCHER_IDLE_KP;
	dm_launching_motor.ctrl.kd_set  = LAUNCHER_IDLE_KD;
	dm_launching_motor.ctrl.tor_set = LAUNCHER_IDLE_TOR;

	servo_1 = LOAD_ANGLE_1;
	servo_2 = LOAD_ANGLE_2;
	servo_3 = LOAD_ANGLE_3;
	servo_4 = LAUNCHER_SERVO_ANGLE_REST;
	pos_angle = POS_RAD_1_0;
	dm_feeder_motor.ctrl.pos_set = pos_angle;
	dm_feeder_motor.ctrl.kp_set  = kp_set_feed;
	dm_feeder_motor.ctrl.kd_set  = kd_set_feed;
	dm_ctrl_send(&hcan1, &dm_feeder_motor);
	vTaskDelay(pdMS_TO_TICKS(15000));
	#if TESTING_FEEDER == 0
		#if TESTING_SERVO == 0
			FeederState = POS_1_0;
		#elif TESTING_SERVO == 1
			FeederState = POS_1_0;
		#elif TESTING_SERVO == 2
			FeederState = POS_2_0;
		#elif TESTING_SERVO == 3
			FeederState = POS_3_0;
		#endif
	#else
		#if TESTING_SERVO == 0
			FeederState = POS_1_0;
		#elif TESTING_SERVO == 1
			FeederState = POS_1_1;
		#elif TESTING_SERVO == 2
			FeederState = POS_2_1;
		#elif TESTING_SERVO == 3
			FeederState = POS_3_1;
		#endif
	#endif
	/* Infinite loop */
   for(;;)
   {
	   RC_ctrl_t rc;
	   get_remote_control_snapshot(&rc);
	#if TESTING == 1
		testing_launcher();
		test_servo_dm_feeder();
	#else


		dm_motor_t feeder_snap, launching_snap;
		dm_motor_snapshot(&feeder_snap, (const volatile dm_motor_t *)&dm_feeder_motor);
		dm_motor_snapshot(&launching_snap, (const volatile dm_motor_t *)&dm_launching_motor);
		if (rc.rc.s[1] == 1) {
		switch (LnF_state){
		case WAIT:
			dm_launching_motor.ctrl.pos_set = 0;
			dm_launching_motor.ctrl.vel_set = 0;
			dm_launching_motor.ctrl.kp_set  = LAUNCHER_IDLE_KP;
			dm_launching_motor.ctrl.kd_set  = LAUNCHER_IDLE_KD;
			dm_launching_motor.ctrl.tor_set = LAUNCHER_IDLE_TOR;

			dm_feeder_motor.ctrl.pos_set = 0;
			dm_feeder_motor.ctrl.vel_set = 0;
			dm_feeder_motor.ctrl.kp_set  = kp_set_feed;
			dm_feeder_motor.ctrl.kd_set  = kd_set_feed;
			dm_feeder_motor.ctrl.tor_set = 0;
			

			dart_fire = 0;
			#if TESTING_SERVO == 0
				dart_count = 4;
			#else
				dart_count = 3;
			#endif
			servo_1 = LOAD_ANGLE_1;
			servo_2 = LOAD_ANGLE_2;
			servo_3 = LOAD_ANGLE_3;
			servo_4 = LAUNCHER_SERVO_ANGLE_REST;
			pos_angle = POS_RAD_1_0;
			ready = false;
			#if TESTING_WOUT_YAW && TESTING_SERVO != 0
				if (rc.rc.s[0] == 1) {
					LnF_state = FIRING;
				}
			break;
			#else
				LnF_state = FIRING;
			#endif

				set_var_firing(false);
			//TODO make the button on changable for different numbers of dart
			if (ui_interface_get_auto_dart_count() > 0){
				LnF_state = FIRING;
				set_var_firing(true);
			}
			break;
		case FIRING: {
		/*Launcher State Machine--------------------------------------------------------*/
			switch (LauncherState){
			case LAUNCHER_WAIT:
			dm_launching_motor.ctrl.pos_set = 0;
			dm_launching_motor.ctrl.vel_set = 0;
			dm_launching_motor.ctrl.kp_set  = LAUNCHER_WAIT_KP;
			dm_launching_motor.ctrl.kd_set  = LAUNCHER_WAIT_KD;
			dm_launching_motor.ctrl.tor_set = LAUNCHER_WAIT_TOR;
			servo_4 = LAUNCHER_SERVO_ANGLE_REST;

			#if TESTING_WOUT_YAW && TESTING_SERVO != 0
				if (rc.rc.s[0] == 3) {
					LauncherState = SEEK_LOCK_CW;
				}
			#else
			if (dart_count == 4 && !HAL_GPIO_ReadPin(LOCK_GPIO_Port, LOCK_Pin)){
				LauncherState = SEEK_LOCK_CW;
			} else if (dart_count > 0) {
				LauncherState = SEEK_LOCK_CW;
			} else {
				ui_interface_set_auto_dart_count(0);
				LauncherState = LAUNCHER_WAIT;
				LnF_state = WAIT;
			}
			#endif
//			lock = false;
			break;

			case SEEK_LOCK_CW:
			dm_launching_motor.ctrl.pos_set = 0;
			dm_launching_motor.ctrl.vel_set = LAUNCHER_SEEK_LOCK_VEL;
			dm_launching_motor.ctrl.kp_set  = LAUNCHER_SEEK_LOCK_KP;
			dm_launching_motor.ctrl.kd_set  = LAUNCHER_SEEK_LOCK_KD;
			dm_launching_motor.ctrl.tor_set = LAUNCHER_SEEK_LOCK_TOR;
			if (dart_count != 4){
				ready = true;
			}

			static TickType_t lock_hold_start_tick = 0;
			static uint8_t lock_hold_active = 0;

			if (!HAL_GPIO_ReadPin(LOCK_GPIO_Port, LOCK_Pin)) {
				if (!lock_hold_active) {
					lock_hold_active = 1;
					lock_hold_start_tick = xTaskGetTickCount();

				}
			}

			/* After 300 ms from first entering in-position window, finish shot */
			if (!HAL_GPIO_ReadPin(LOCK_GPIO_Port, LOCK_Pin) &&
				(xTaskGetTickCount() - lock_hold_start_tick) >= pdMS_TO_TICKS(2000)) {
				ready = false;
				lock_hold_active = 0;
//				lock = false;
				dm_launching_motor.ctrl.pos_set = 0;
				dm_launching_motor.ctrl.vel_set = 0;
				dm_launching_motor.ctrl.kp_set  = 0;
				dm_launching_motor.ctrl.kd_set  = 0;
				dm_launching_motor.ctrl.tor_set = 0;

				LauncherState = LOCKED;
				__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
				NVIC_EnableIRQ(EXTI9_5_IRQn);
			}
			break;

			case LOCKED:

			if (dart_count == 4){
				dm_launching_motor.ctrl.pos_set = 0;
				dm_launching_motor.ctrl.vel_set = 4;
				dm_launching_motor.ctrl.kp_set  = LAUNCHER_LOCKED_KP;
				dm_launching_motor.ctrl.kd_set  = LAUNCHER_LOCKED_KD;
				dm_launching_motor.ctrl.tor_set = LAUNCHER_LOCKED_TOR;
			} else {
				if (((FeederState == POS_1_1) || (FeederState == POS_2_1) || (FeederState == POS_3_1)) && done){
					dm_launching_motor.ctrl.pos_set = 0;
					dm_launching_motor.ctrl.vel_set = LAUNCHER_LOCKED_VEL;
					dm_launching_motor.ctrl.kp_set  = LAUNCHER_LOCKED_KP;
					dm_launching_motor.ctrl.kd_set  = LAUNCHER_LOCKED_KD;
					dm_launching_motor.ctrl.tor_set = LAUNCHER_LOCKED_TOR;
//				} else {
//					dm_launching_motor.ctrl.pos_set = 0;
//					dm_launching_motor.ctrl.vel_set = 0;
//					dm_launching_motor.ctrl.kp_set  = LAUNCHER_LOCKED_KP;
//					dm_launching_motor.ctrl.kd_set  = LAUNCHER_LOCKED_KD;
//					dm_launching_motor.ctrl.tor_set = LAUNCHER_LOCKED_TOR;
				}

			}
			if (op_sen_feeder){
				op_sen_feeder = false;
				LauncherState = FEED;
				NVIC_DisableIRQ(EXTI9_5_IRQn);
				op_endstop_position = launching_snap.para.pos_wrap;

			}
			break;
			case FEED:

				dm_launching_motor.ctrl.pos_set = 0;
				dm_launching_motor.ctrl.vel_set = 0;
				dm_launching_motor.ctrl.kp_set  = LAUNCHER_FEED_KP;
				dm_launching_motor.ctrl.kd_set  = LAUNCHER_FEED_KD;
				dm_launching_motor.ctrl.tor_set = launcher_reverse_tor;
				reverse_pos_pid.max_output = feeding_max_velocity;
				reverse_pos_pid.min_output = -reverse_pos_pid.max_output;
				/* Use PID.c position PID (reverse_pos_pid) to generate velocity command */
				const float dt = 0.01f; /* 10 ms loop assumption (matches previous PID_Compute) */
				//TODO configure servo angle for UNLOAD_1 and UNLOAD_3
				if ((FeederState  == PAUSE_1)|| (FeederState  == PAUSE_1_1)||(FeederState  == PAUSE_2_1) || (FeederState  == PAUSE_2_2) || (FeederState  == PAUSE_2_3) || (FeederState  == PAUSE_2_4) || (FeederState  == PAUSE_2_5) ||(FeederState  == PAUSE_3) || (FeederState == PAUSE_3_1)){
					if ((FeederState  == PAUSE_1) || (FeederState  == PAUSE_1_1) ){
						rev_to_feed = PAUSE_1_DIST;
					} else if (FeederState  == PAUSE_2_1 || FeederState  == PAUSE_2_2 || FeederState  == PAUSE_2_3){
						rev_to_feed = PAUSE_2_123DIST;
					} else if ((FeederState  == PAUSE_2_4)  || (FeederState  == PAUSE_2_5)) {
						rev_to_feed = PAUSE_2_4DIST;
					} else if ((FeederState  == PAUSE_3) || (FeederState  == PAUSE_3_1)){
						rev_to_feed = pause_3_dist + extra_front;

					}
					PID_Compute(&reverse_pos_pid,
								(rev_to_feed + op_endstop_position),
								launching_snap.para.pos_wrap,
								dt,
								0.0f);
					float vel_cmd = reverse_pos_pid.output;
					dm_launching_motor.ctrl.pos_set = 0;
					dm_launching_motor.ctrl.vel_set = vel_cmd;
					dm_launching_motor.ctrl.kp_set   = LAUNCHER_REVERSE_KP;
					dm_launching_motor.ctrl.kd_set   = LAUNCHER_REVERSE_KD;
					dm_launching_motor.ctrl.tor_set = launcher_reverse_tor;
				} else if ((FeederState == RELOAD_1)  || (FeederState == RELOAD_2) || (FeederState == RELOAD_3)){
					PID_Compute(&reverse_pos_pid,
								op_endstop_position + OFFSET,
								launching_snap.para.pos_wrap,
								dt,
								0.0f);
					float vel_cmd = reverse_pos_pid.output;
					dm_launching_motor.ctrl.pos_set = 0;
					dm_launching_motor.ctrl.vel_set = vel_cmd;
					dm_launching_motor.ctrl.kp_set   = LAUNCHER_REVERSE_KP;
					dm_launching_motor.ctrl.kd_set   = LAUNCHER_REVERSE_KD;
					dm_launching_motor.ctrl.tor_set = launcher_reverse_tor;
				}



				if (((FeederState == POS_2_0  || FeederState == POS_3_0  || FeederState == POS_4_0) && done) || (dart_count == 4)){
					LauncherState = REVERSE_TO_WEIGHT;

				}
				break;

			case REVERSE_TO_WEIGHT: {
					reverse_pos_pid.max_output = firing_max_velocity ;
					reverse_pos_pid.min_output = -reverse_pos_pid.max_output;
					/* Use PID.c position PID (reverse_pos_pid) to generate velocity command */
					const float dt = 0.01f; /* 10 ms loop assumption (matches previous PID_Compute) */
					PID_Compute(&reverse_pos_pid,
								(LAUNCHER_REVERSE_POS_TARGET + op_endstop_position),
								launching_snap.para.pos_wrap,
								dt,
								0.0f);

					float vel_cmd = reverse_pos_pid.output;

					dm_launching_motor.ctrl.pos_set = 0;
					dm_launching_motor.ctrl.vel_set = vel_cmd;
					dm_launching_motor.ctrl.kp_set   = LAUNCHER_REVERSE_KP;
					dm_launching_motor.ctrl.kd_set   = LAUNCHER_REVERSE_KD;
					dm_launching_motor.ctrl.tor_set = launcher_reverse_tor;

					/* When we reach the reverse position, release for 300 ms then go back to LAUNCHER_WAIT */
					{
						static TickType_t reverse_hold_start_tick = 0;
						static uint8_t reverse_hold_active = 0;

						#if !(TESTING_WOUT_YAW && TESTING_SERVO != 0)
						if ((launching_snap.para.pos_wrap < op_endstop_position + LAUNCHER_REVERSE_POS_TARGET + TOL) &&
							(launching_snap.para.pos_wrap > op_endstop_position + LAUNCHER_REVERSE_POS_TARGET - TOL)) {
							/* First time reaching position: start 300 ms window and release lock */
						#else
						if ((launching_snap.para.pos_wrap < op_endstop_position + LAUNCHER_REVERSE_POS_TARGET + TOL) &&
							(launching_snap.para.pos_wrap > op_endstop_position + LAUNCHER_REVERSE_POS_TARGET - TOL) &&
							(rc.rc.s[0] == 2)) {
						#endif
							if (!reverse_hold_active && HAL_GPIO_ReadPin(LOCK_GPIO_Port, LOCK_Pin)) {
								reverse_hold_active = 1;
								reverse_hold_start_tick = xTaskGetTickCount();
							}
							servo_4 = LAUNCHER_SERVO_ANGLE_RELEASE;
						} 

						/* After 10s from first entering in-position window, finish shot */
						if (reverse_hold_active &&
							(xTaskGetTickCount() - reverse_hold_start_tick) >= pdMS_TO_TICKS(10000)) {
							dm_launching_motor.ctrl.vel_set = 0;
							dm_launching_motor.ctrl.tor_set = 0.0f;
							servo_4 = LAUNCHER_SERVO_ANGLE_REST;
							LauncherState = LAUNCHER_WAIT;
							#if (TESTING_WOUT_YAW && TESTING_SERVO != 0)
								LnF_state = WAIT;
							#endif;
							#if TESTING_SERVO == 0
								dart_count--;
							#endif
							reverse_hold_active = 0;
							if (dart_count == 0){
								final = true;
							}
						}
					}
				}
			break;
			}
		

	/*Feeder State Machine--------------------------------------------------------*/
			static TickType_t feeder_hold_start_tick = 0;
			static uint8_t feeder_hold_active = 0;
			switch (FeederState){
			  case POS_1_0:
				  servo_1 = LOAD_ANGLE_1;
				  servo_2 = LOAD_ANGLE_2;
				  servo_3 = LOAD_ANGLE_3;
				  pos_angle = POS_RAD_1_0;

				  if ((feeder_snap.para.pos > (POS_RAD_1_0 - POS_RAD_TOL)) &&
				  	(feeder_snap.para.pos < (POS_RAD_1_0 + POS_RAD_TOL))){
					done = true;
				  } else {
					done = false;
				  }

				  if (ready && done){
					  FeederState = POS_1_1;
				      ready = false;
				      done = false;
				  }

				  break;
			  case POS_1_1:
			      servo_1 = LOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_1_1_0;
			      kp_set_feed = POS_RAD_1_KP;
			      kd_set_feed = POS_RAD_1_KD;

			      if ((feeder_snap.para.pos > (POS_RAD_1_1 - POS_RAD_TOL)) &&
			          (feeder_snap.para.pos < (POS_RAD_1_1 + POS_RAD_TOL)))
			      {
			    	  done = true;
			      } else {
			    	  done = false;
			      }

				  #if TESTING_FEEDER == 0
			      if ((LauncherState == FEED) && done){
 			          FeederState = UNLOAD_1;
 			          done = false;
			      }
				  #endif
			      break;

			  case UNLOAD_1:
			      servo_1 = UNLOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_1_1_0;
				  if (!feeder_hold_active) {
						feeder_hold_active = 1;
						feeder_hold_start_tick = xTaskGetTickCount();
					}

				if ((xTaskGetTickCount() - feeder_hold_start_tick) >= pdMS_TO_TICKS(4000)) {
					feeder_hold_active = 0;
					FeederState = PAUSE_1;
				}
			     break;

			  case PAUSE_1:
				  servo_1 = UNLOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_1_1_0;
			      //within tolerance of move forward distance will trigger change state to RELOAD
			      //check equivalenc for respective forward distance and the state of the launcher
			      if ((launching_snap.para.pos_wrap < op_endstop_position + PAUSE_1_DIST +dist_tol) && (launching_snap.para.pos_wrap > op_endstop_position + PAUSE_1_DIST - dist_tol)){
			    	  FeederState = PAUSE_1_1;
			      }
				  break;
			  case PAUSE_1_1:
				  servo_1 = LOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_1_1_0;
			      if (!feeder_hold_active) {
					  feeder_hold_active = 1;
					  feeder_hold_start_tick = xTaskGetTickCount();
				  }

				  if ((xTaskGetTickCount() - feeder_hold_start_tick) >= pdMS_TO_TICKS(2000)) {
					feeder_hold_active = 0;
					FeederState = RELOAD_1;
				  }
				  break;

			  case RELOAD_1:
			      servo_1 = LOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_1_1_0;

				  if (!feeder_hold_active) {
					  feeder_hold_active = 1;
					  feeder_hold_start_tick = xTaskGetTickCount();
				  }

				 if (((xTaskGetTickCount() - feeder_hold_start_tick) >= pdMS_TO_TICKS(2000)) && (launching_snap.para.pos_wrap < op_endstop_position + OFFSET + dist_tol) && (launching_snap.para.pos_wrap > op_endstop_position + OFFSET - dist_tol)) {
					feeder_hold_active = 0;
					FeederState = POS_2_0;
				 }
			     break;

			  case POS_2_0:
				  servo_1 = LOAD_ANGLE_1;
				  servo_2 = LOAD_ANGLE_2;
				  servo_3 = LOAD_ANGLE_3;
				  pos_angle = POS_RAD_2_0;

				  if ((feeder_snap.para.pos > (POS_RAD_2_0 - POS_RAD_TOL)) &&
				  	(feeder_snap.para.pos < (POS_RAD_2_0 + POS_RAD_TOL))){
					done = true;
				  } else {
					done = false;
				  }


				  if (ready && done){
					#if TESTING_SERVO == 1
					  FeederState = POS_1_1;
					#else
					  FeederState = POS_2_1;
					#endif
					  ready = false;
					  done = false;
				  }
				  break;


			  case POS_2_1:
			      servo_1 = LOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_2_1_0;
				  kp_set_feed = POS_RAD_2_KP;
				  kd_set_feed = POS_RAD_2_KD;

					  if ((feeder_snap.para.pos > (POS_RAD_2_1 - POS_RAD_TOL)) &&
				   (feeder_snap.para.pos < (POS_RAD_2_1 + POS_RAD_TOL))){
					  done = true;
				  } else {
					  done = false;
				  }
				  #if TESTING_FEEDER == 0
				  if ((LauncherState == FEED) && done){
					  FeederState = UNLOAD_2;
					  done = false;
				  }
				  #endif
				  break;

			  case UNLOAD_2:
			      servo_1 = LOAD_ANGLE_1;
			      servo_2 = UNLOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_2_1_0;
				  if (!feeder_hold_active) {
					  feeder_hold_active = 1;
					  feeder_hold_start_tick = xTaskGetTickCount();
				  }

				  if ((xTaskGetTickCount() - feeder_hold_start_tick) >= pdMS_TO_TICKS(3000)) {
					  feeder_hold_active = 0;
					  FeederState = PAUSE_2_1;
				  }
			      break;

			  case PAUSE_2_1:
				  servo_1 = LOAD_ANGLE_1;
			      servo_2 = UNLOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_2_1_0;
			      //within tolerance of move forward distance will trigger change state to RELOAD
			      //check equivalenc for respective forward distance and the state of the launcher
			      if ((launching_snap.para.pos_wrap < op_endstop_position + PAUSE_2_123DIST +dist_tol) && (launching_snap.para.pos_wrap > op_endstop_position + PAUSE_2_123DIST - dist_tol)){
			    	  FeederState = PAUSE_2_2;
			      }
				  break;
			  case PAUSE_2_2:
				  servo_1 = LOAD_ANGLE_1;
			      servo_2 = UNLOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_2_1_1;
				  if ((feeder_snap.para.pos > (POS_RAD_2_1_1 - POS_RAD_TOL)) &&
				  (feeder_snap.para.pos < (POS_RAD_2_1_1 + POS_RAD_TOL))){
					  done = true;
				  } else {
					  done = false;
				  }

				  if (done) {
					  FeederState = PAUSE_2_3;
				  }
				  break;
			  case PAUSE_2_3:
			      servo_1 = LOAD_ANGLE_1;
			      servo_2 = UNLOAD_ANGLE_2_0;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_2_1_1;
				  if (!feeder_hold_active) {
					  feeder_hold_active = 1;
					  feeder_hold_start_tick = xTaskGetTickCount();
				  }

				  if ((xTaskGetTickCount() - feeder_hold_start_tick) >= pdMS_TO_TICKS(3000)) {
					  feeder_hold_active = 0;
					  FeederState = PAUSE_2_4;
				  }
				  break;
			  case PAUSE_2_4:
				  servo_1 = LOAD_ANGLE_1;
			      servo_2 = UNLOAD_ANGLE_2_0;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_2_1_1;
			      //within tolerance of move forward distance will trigger change state to RELOAD
			      //check equivalenc for respective forward distance and the state of the launcher
			      if ((launching_snap.para.pos_wrap < op_endstop_position + PAUSE_2_4DIST +dist_tol) && (launching_snap.para.pos_wrap > op_endstop_position + PAUSE_2_4DIST - dist_tol)){
			    	  FeederState = PAUSE_2_5;
			      }
				  break;
			  case PAUSE_2_5:
				  servo_1 = LOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_2_1_1;

			      if (!feeder_hold_active) {
					  feeder_hold_active = 1;
					  feeder_hold_start_tick = xTaskGetTickCount();
				  }

				  if ((xTaskGetTickCount() - feeder_hold_start_tick) >= pdMS_TO_TICKS(2000)) {
					feeder_hold_active = 0;
					FeederState = RELOAD_2;
				  }
			      break;

			  case RELOAD_2:
			      servo_1 = LOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_2_1_1;
				  if (!feeder_hold_active) {
					  feeder_hold_active = 1;
					  feeder_hold_start_tick = xTaskGetTickCount();
				  }

				 if ((xTaskGetTickCount() - feeder_hold_start_tick) >= pdMS_TO_TICKS(2000) && (launching_snap.para.pos_wrap < op_endstop_position + OFFSET + dist_tol) && (launching_snap.para.pos_wrap > op_endstop_position + OFFSET - dist_tol)) {
					feeder_hold_active = 0;
					FeederState = POS_3_0;
				 }
			      break;

			  case POS_3_0:
			      servo_1 = LOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_3_0;

				  if ((feeder_snap.para.pos > (POS_RAD_3_0 - POS_RAD_TOL)) &&
				  (feeder_snap.para.pos < (POS_RAD_3_0 + POS_RAD_TOL))){
					  done = true;
				  } else {
					  done = false;
				  }



				  if (ready && done){
					#if TESTING_SERVO == 2
					  FeederState = POS_2_1;
					#else
					  FeederState = POS_3_1;
					#endif
					  ready = false;
					  done = false;
				  }
			      break;

			  case POS_3_1:
			      servo_1 = LOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = pos_rad_3_1_0;
			      kp_set_feed = POS_RAD_3_KP;
			      kd_set_feed = POS_RAD_3_KD;
				  
				  if ((feeder_snap.para.pos > (POS_RAD_3_1 - POS_RAD_TOL)) &&
				  (feeder_snap.para.pos < (POS_RAD_3_1 + POS_RAD_TOL))){
					  done = true;
				  } else {
					  done = false;
				  }
				  if ((LauncherState == FEED) && done){
					  FeederState = UNLOAD_3;
					  done = false;
				  }
				  break;

			  case UNLOAD_3:
			      servo_1 = LOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = UNLOAD_ANGLE_3;
			      pos_angle = pos_rad_3_1_0;
				  if (!feeder_hold_active) {
					  feeder_hold_active = 1;
					  feeder_hold_start_tick = xTaskGetTickCount();
				  }

				  if ((xTaskGetTickCount() - feeder_hold_start_tick) >= pdMS_TO_TICKS(2000)) {
					  feeder_hold_active = 0;
					  FeederState = PAUSE_3;
				  }
			      break;

			  case PAUSE_3:
			      servo_1 = LOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = UNLOAD_ANGLE_3;
			      pos_angle = pos_rad_3_1_0;

				  if ((launching_snap.para.pos_wrap < op_endstop_position + pause_3_dist + dist_tol) && (launching_snap.para.pos_wrap > op_endstop_position + pause_3_dist - dist_tol)){
					  FeederState = PAUSE_3_1;
				  }
				  break;
			  case PAUSE_3_1:
				  servo_1 = LOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = pos_rad_3_1_0;

			      if (!feeder_hold_active) {
					  feeder_hold_active = 1;
					  feeder_hold_start_tick = xTaskGetTickCount();
				  }

				  if ((xTaskGetTickCount() - feeder_hold_start_tick) >= pdMS_TO_TICKS(2000)) {
					feeder_hold_active = 0;
					FeederState = RELOAD_3;
				  }
				  break;
			  case RELOAD_3:
			      servo_1 = LOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = pos_rad_3_1_0;
				  if (!feeder_hold_active) {
					  feeder_hold_active = 1;
					  feeder_hold_start_tick = xTaskGetTickCount();
				  }

				 if (((xTaskGetTickCount() - feeder_hold_start_tick) >= pdMS_TO_TICKS(2000)) &&  (launching_snap.para.pos_wrap < op_endstop_position + OFFSET + dist_tol) && (launching_snap.para.pos_wrap > op_endstop_position + OFFSET - dist_tol)) {
					feeder_hold_active = 0;
					FeederState = POS_4_0;
				 }
			      break;

			  case POS_4_0:
			      servo_1 = LOAD_ANGLE_1;
			      servo_2 = LOAD_ANGLE_2;
			      servo_3 = LOAD_ANGLE_3;
			      pos_angle = POS_RAD_4_0;
				  if ((feeder_snap.para.pos > (POS_RAD_4_0 - POS_RAD_TOL)) &&
					(feeder_snap.para.pos < (POS_RAD_4_0 + POS_RAD_TOL))){
					done = true;
				  } else {
					done = false;
				  }

				  #if TESTING_SERVO == 3
					  if (ready && done){
//						  final = false;
						  done = false;
						  FeederState = POS_3_1;

				  #else
					  if (final && done){
						  final = false;
						  done = false;
						  FeederState = POS_1_0;
				   #endif
				  }
				  break;

			  default:
			      FeederState = POS_1_0	;
			      ready = false;
			      done = false;
			      break;
				}
			}
		}
		} else {
			float velocity = - ((float) rc.rc.ch[1])/10;

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


			if (rc.rc.s[0] == 2) {
				servo_4 = LAUNCHER_SERVO_ANGLE_RELEASE;
			} else {
				servo_4 = LAUNCHER_SERVO_ANGLE_REST;
			}
		}
		dm_feeder_motor.ctrl.pos_set = pos_angle;
		dm_feeder_motor.ctrl.kp_set  = kp_set_feed;
		dm_feeder_motor.ctrl.kd_set  = kd_set_feed;
		servo_set_angle(&LAUNCHER_SERVO_TIM, LAUNCHER_SERVO_CHANNEL, servo_4);
		servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO3_CHANNEL, servo_3);
		servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO2_CHANNEL, servo_2);
		servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO1_CHANNEL, servo_1);
		dm_ctrl_send(&hcan1, &dm_feeder_motor);
		vTaskDelay(1);
		dm_ctrl_send(&hcan1, &dm_launching_motor);
	#endif
	}

}

void testing_launcher(void){
	RC_ctrl_t rc;
	dm_motor_t launching_snap;
	get_remote_control_snapshot(&rc);
	dm_motor_snapshot(&launching_snap, (const volatile dm_motor_t *)&dm_launching_motor);
	float current_position = launching_snap.para.pos_wrap;
	if (op_sen_feeder && !hit_op){
		hit_op = true;
		op_sen_feeder = false;
		op_endstop_position = launching_snap.para.pos_wrap;

	}
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	if (rc.rc.s[0] != 1){
		if (rc.rc.s[1] == 1){
			servo_set_angle(&LAUNCHER_SERVO_TIM, LAUNCHER_SERVO_CHANNEL, LAUNCHER_SERVO_ANGLE_RELEASE);
		} else {
			servo_set_angle(&LAUNCHER_SERVO_TIM, LAUNCHER_SERVO_CHANNEL, LAUNCHER_SERVO_ANGLE_REST);
		}

		if (rc.rc.s[1] == 2  && hit_op){
			const float dt = 0.01f; /* 10 ms loop assumption (matches previous PID_Compute) */
			PID_Compute(&reverse_pos_pid,
						(rev_to_feed + op_endstop_position),
						launching_snap.para.pos_wrap,
						dt,
						0.0f);
			float vel_cmd = reverse_pos_pid.output;
			dm_launching_motor.ctrl.pos_set = 0;
			dm_launching_motor.ctrl.vel_set = vel_cmd;
			dm_launching_motor.ctrl.kp_set   = LAUNCHER_REVERSE_KP;
			dm_launching_motor.ctrl.kd_set   = LAUNCHER_REVERSE_KD;
			dm_launching_motor.ctrl.tor_set = launcher_reverse_tor;
			dm_ctrl_send(&hcan1, &dm_launching_motor);
		} else {
			float velocity = - ((float) rc.rc.ch[1])/10;

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
		}
	} else {
			hit_op = false;
			dm_launching_motor.ctrl.pos_set = 0;
			dm_launching_motor.ctrl.vel_set = 0;
			dm_launching_motor.ctrl.kp_set  = LAUNCHER_IDLE_KP;
			dm_launching_motor.ctrl.kd_set  = LAUNCHER_TEST_KD;
			dm_launching_motor.ctrl.tor_set = LAUNCHER_IDLE_TOR;
			dm_ctrl_send(&hcan1, &dm_launching_motor);
	}
}

void test_servo_dm_feeder(void){
	dm_motor_t feeder_snap;
	dm_motor_snapshot(&feeder_snap, (const volatile dm_motor_t *)&dm_feeder_motor);
	RC_ctrl_t rc;
	get_remote_control_snapshot(&rc);
	if (rc.rc.s[0] == 2){
		dm_feeder_motor.ctrl.pos_set = 0;
		dm_feeder_motor.ctrl.vel_set = 6;
		dm_feeder_motor.ctrl.kp_set  = 0;
		dm_feeder_motor.ctrl.kd_set  = 6;
		dm_feeder_motor.ctrl.tor_set = 0;
		servo_3 = 0;
		servo_2 = 0;
		servo_1 = 0;
	} else {
		dm_feeder_motor.ctrl.pos_set = pos_angle;
		dm_feeder_motor.ctrl.vel_set = 0;
		dm_feeder_motor.ctrl.kp_set  = kp_set_feed;
		dm_feeder_motor.ctrl.kd_set  = kd_set_feed;
		dm_feeder_motor.ctrl.tor_set = 0;
		servo_3 = 0;
		servo_2 = 0;
		servo_1 = 0;
	}

	servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO3_CHANNEL, servo_3);
	servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO2_CHANNEL, servo_2);
	servo_set_angle(&FEEDER_SERVO_TIM, FEEDER_SERVO1_CHANNEL, servo_1);
	dm_ctrl_send(&hcan1, &dm_feeder_motor);
	vTaskDelay(1);
}
