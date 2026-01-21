/*
 * controlTask.c
 *
 *  Created on: Jan 19, 2026
 *      Author: Barry Ang
 */

#include "lvgl.h"
#include "ui.h"
#include "usb_device.h"
#include "cmsis_os.h"
#include "../ui_interface.h"


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

  /* Initialize UI interface */
  ui_interface_init();

  /* Infinite loop */
  for(;;)
  {
    uint32_t delay = lv_timer_handler();
    ui_tick();
    ui_interface_update_display();
    if (delay < 5U) {
      delay = 5U;
    }
    osDelay(delay);

  }
  /* USER CODE END 5 */
}
