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

   /* Infinite loop */
   for(;;)
   {
	  switch (state){

	  case WAIT:
		  if (ui_interface_check_draw_request()){
			  state = SEEK_LOCK_CW;
		  }
		  break;

	  case SEEK_LOCK_CW:

		  if (lock){
			  state = LOCKED;
		  }
		  break;

	  case LOCKED:

		  if (op_sen){
			  state = FEED;
			  ready = true;

		  }
	  case FEED:

		  if (ready == false){
			  state = REVERSE_TO_WEIGHT;
		  }

	  case REVERSE_TO_WEIGHT:

		  if (dm_launching_motor.para.tor > target - tol && dm_launching_motor.para.tor < target + tol){
			  state = HOLD;

		  }
		  break;

	  case HOLD:


		  if (ui_interface_check_fire_request()){
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
			  state = WAIT;
		  }
		  break;
	  }



	  dm4310_ctrl_send(&hcan1, &dm_launching_motor);
	  osDelay(1);
   }
}



