/*
 * launcher.c
 *
 *  Created on: Jan 19, 2026
 *      Author: Barry Ang
 */


#include "main.h"
#include "bsp_damiao.h"
#include <math.h>
#include <stdbool.h>
#include "../ui_interface.h"


#define FORWARD 1;
#define BACKWARD -1;


//with launching elastic band (known load, need find rough load at launching)
#define lAUNCHING_TENS_SET 1000.0f;
#define lAUNCHING_TENS_KP 10.0f;
#define lAUNCHING_TENS_KD 1.0f;
#define lAUNCHING_TENS_TOR 100.0f;
#define lAUNCHING_TENS_VEL 100.0f;	

//for launching outer loop pid controller
#define lAUNCHING_PID_TENS_KP 10.0f;
#define lAUNCHING_PID_TENS_KD 1.0f;
#define lAUNCHING_PID_TENS_KI 0.0f;
#define lAUNCHING_PID_TENS_TOR 100.0f;

//for launching inner loop pid controller
#define lAUNCHING_PID_VEL_VEL 100.0f;
#define lAUNCHING_PID_VEL_KI 0.0f;
#define lAUNCHING_PID_VEL_KP 10.0f;
#define lAUNCHING_PID_VEL_KD 1.0f;


//with feeder elastic band (known load, need find rough load at feeder)
#define FEEDER_TENS_SET 1000.0f;
#define FEEDER_TENS_KP 10.0f;
#define FEEDER_TENS_KD 1.0f;
#define FEEDER_TENS_TOR 100.0f;
#define TENS_VEL 100.0f;



//without elastic band (known load, need find from tuning)
#define LOCK_SET 1000.0f;
#define LOCK_KP 10.0f;
#define LOCK_KD 1.0f;
#define LOCK_TOR 100.0f;
#define LOCK_VEL 100.0f;

float target = 1000.0f, tol = 50.0f;
enum {WAIT, SEEK_LOCK_CW, LOCKED, FEED, REVERSE_TO_WEIGHT, HOLD } state = WAIT;

extern dm_motor_t dm_launching_motor;

bool lock = false;
bool op_sen = false ;

extern bool ready;
extern CAN_HandleTypeDef hcan1;


void LauncherTask(void *argument)
{

  //need to think of state machine for the launcher

/*
 *  motor needs to spin forward to catch the electronic lock
 *
 *  motor spin backward till it hit the optical switch
 *
 *  motor will stop spinning
 *
 *  feeder will then be given a true signal to release the dart
 *
 *  once feeder send a false
 *
 *  motor can spin backwards till force of elastic band (determined by load cell and torque of motor) is achieved
 *
 *  a signal will be sent to the electronic lock to released
 *
 *
 * once released, got back to first state
 *
 */
  /* USER CODE BEGIN StartTask04 */
  PID_Init(&launching_pid_tens_outer, lAUNCHING_PID_TENS_KP, lAUNCHING_PID_TENS_KI, lAUNCHING_PID_TENS_KD, 0, 1000.0f);
  PID_Init(&launching_pid_tens_inner, lAUNCHING_PID_VEL_KP, lAUNCHING_PID_VEL_KI, lAUNCHING_PID_VEL_KD, 0, 1000.0f);
  CascadePID launching_pid_tens;

  launching_pid_tens.outer = launching_pid_tens_outer;
  launching_pid_tens.inner = launching_pid_tens_inner;

   /* Infinite loop */
   for(;;)
   {
	  switch (state){
	  case WAIT:

		  dm_launching_motor.ctrl.pos_set = 0;
		  dm_launching_motor.ctrl.vel_set = 0;
		  dm_launching_motor.ctrl.kp_set  = TENS_SET;
		  dm_launching_motor.ctrl.kd_set  = TENS_KP;
		  dm_launching_motor.ctrl.tor_set = 0;
		  state = WAIT;
		  if (ui_interface_check_fire_request()){
			  state = SEEK_LOCK_CW;
		  }
		  break;

	  case SEEK_LOCK_CW:

		  dm_launching_motor.ctrl.pos_set = 0;
		  dm_launching_motor.ctrl.vel_set = LOCK_VEL;
		  dm_launching_motor.ctrl.kp_set  = LOCK_KP;
		  dm_launching_motor.ctrl.kd_set  = LOCK_KP;
		  dm_launching_motor.ctrl.tor_set = LOCK_TOR;
		  if (lock){
			  state = LOCKED;
		  }
		  break;

	  case LOCKED:
		  dm_launching_motor.ctrl.pos_set = 0;
		  dm_launching_motor.ctrl.vel_set = TENS_VEL;
		  dm_launching_motor.ctrl.kp_set  = TENS_SET;
		  dm_launching_motor.ctrl.kd_set  = TENS_KP
		  dm_launching_motor.ctrl.tor_set = TENS_TOR;
		  if (op_sen){
			  state = FEED;
			  ready = true;

		  }
	  case FEED:
		  //add the pid here
		  dm_launching_motor.ctrl.pos_set = 0;
		  dm_launching_motor.ctrl.vel_set = 0;
		  dm_launching_motor.ctrl.kp_set  = TENS_SET;
		  dm_launching_motor.ctrl.kd_set  = TENS_KP;
		  dm_launching_motor.ctrl.tor_set = 0;


		  if (ready == false){
			  state = REVERSE_TO_WEIGHT;
		  }

	  case REVERSE_TO_WEIGHT:
		  //add the pid here
		  PID_CascadeCalc(&launching_pid_tens, target, dm_launching_motor.para.tor, dm_launching_motor.para.vel, 0.01);
		  dm_launching_motor.ctrl.pos_set = 0;
		  dm_launching_motor.ctrl.vel_set = 0;
		  dm_launching_motor.ctrl.kp_set  = TENS_SET;
		  dm_launching_motor.ctrl.kd_set  = TENS_KP;
		  dm_launching_motor.ctrl.tor_set = 0;

		  if (dm_launching_motor.para.tor > target - tol && dm_launching_motor.para.tor < target + tol){
			  state = HOLD;

		  }
		  break;

	  case HOLD:
		  //add the pid here
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

		  dm_launching_motor.ctrl.pos_set = 0;
		  dm_launching_motor.ctrl.vel_set = 0;
		  dm_launching_motor.ctrl.kp_set  = TENS_SET;
		  dm_launching_motor.ctrl.kd_set  = TENS_KP;
		  dm_launching_motor.ctrl.tor_set = 0;


		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
			  dm_launching_motor.ctrl.pos_set = 0;
			  dm_launching_motor.ctrl.vel_set = 0;
			  dm_launching_motor.ctrl.kp_set  = TENS_SET;
			  dm_launching_motor.ctrl.kd_set  = TENS_KP;
			  dm_launching_motor.ctrl.tor_set = 0;
			  state = WAIT;
		  }
		  break;
	  }
	  dm4310_ctrl_send(&hcan1, &dm_launching_motor);
	  osDelay(1);
   }
}



