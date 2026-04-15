/*
 * DART_CONFIG.H
 *
 *  Created on: Jan 31, 2026
 *      Author: Barry Ang
 */

#ifndef INC_DART_CONFIG_H_
#define INC_DART_CONFIG_H_

#define TESTING 1
#define TESTING_WOUT_YAW 1
#define TESTING_YAW_ANGLE 1
#define LAUNCH_CONTROL 2
#define TESTING_SERVO 3
#define TESTING_ONLY_YAW 0

/*
 * Pitch/yaw test input when TESTING || TESTING_WOUT_YAW:
 *   0 = RC combined (testing_pitch_n_yaw)
 *   1 = screen yaw only (testing_yaw_screen)
 *   2 = screen pitch only (testing_pitch_screen)
 * Use TESTING_SCREEN 0 so PitchnYawTask runs alongside the UI for modes 1 and 2.
 */
#define TESTING_PITCH_YAW_INPUT 0

/*
 * TESTING_SCREEN: 1 = only controlTask is started (LVGL / USB / ui_interface).
 * Pitch/yaw and launcher RTOS tasks are not created, so the UI is not driven or
 * contended by those loops. Set to 0 for normal full-firmware operation.
 */
#define TESTING_SCREEN 0
#endif /* INC_DART_CONFIG_H_ */
