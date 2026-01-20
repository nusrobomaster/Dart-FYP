/*
 * controlTask.c
 *
 *  Created on: Jan 19, 2026
 *      Author: Barry Ang
 */

#include "lvgl.h"



void ControlTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */

/*
 * this task should handle the update of the screen as while as the overall state of the robot
 *
 * writing to global variables will be done via this task
 *
 * need to check if I neeed queue for the different variable or just implement a mutex/semaphore
 *
 * the remote controller should be included in this
 */

  /* Infinite loop */
  for(;;)
  {
    uint32_t delay = lv_timer_handler();
    ui_tick();
    if (delay < 5U) {
      delay = 5U;
    }
    osDelay(delay);

  }
  /* USER CODE END 5 */
}
