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
//   #define FORWARD 200;
//   #define BACKWARD -200;
//   float target = 1000.0f, tol = 50.0f;
//   enum { SEEK_LOCK_CW, LOCKED, REVERSE_TO_WEIGHT, HOLD } state = SEEK_LOCK_CW;
//
//   float curr_weight_buffer;
//   float curr_weight;
//   osStatus_t st;
//   int16_t loadingMotor = 200;
   /* Infinite loop */
   for(;;)
   {
//	  if (osMessageQueueGetCount(myQueue_weight_loadingHandle) > 0){
//		  st = osMessageQueueGet(myQueue_weight_loadingHandle, &curr_weight_buffer, NULL, osWaitForever);
//		  if (st == osOK) {
//			  curr_weight = curr_weight_buffer;
//		  }
//	  }
//	switch (state) {
//	case SEEK_LOCK_CW:
//		loadingMotor = FORWARD;
//		if (!unlock) state = REVERSE_TO_WEIGHT;
//		break;
//	case REVERSE_TO_WEIGHT:
//		if (curr_weight < target - tol) {
//			loadingMotor = BACKWARD;
//		}
//		else if (curr_weight > target + tol){
//			loadingMotor = FORWARD;
//		}
//		else {
//			loadingMotor = 0;
//		}
//		if (unlock) state = SEEK_LOCK_CW;
//		break;
//	}
//
//	st = osMessageQueuePut(myQueue_loading_motorHandle, &loadingMotor, 0, 0);
	osDelay(1);
   }
}
