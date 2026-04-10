/*
 * pitchnyawTask.h
 *
 *  Created on: Jan 18, 2026
 *      Author: user
 */

#ifndef SRC_TASKS_PITCHNYAWTASK_H_
#define SRC_TASKS_PITCHNYAWTASK_H_

/* Yaw PD gains */
#define YAW_KP_SET  20.0f
#define YAW_KD_SET  2.0f

/* Angle conversion and limits */
#define DEG2RAD     (0.01745329251994329577f)
#define ANGLE_MIN_DEG  0.0f
#define ANGLE_MAX_DEG  360.0f
#define ANGLE_MIN_RAD  (ANGLE_MIN_DEG * DEG2RAD)
#define ANGLE_MAX_RAD  (ANGLE_MAX_DEG * DEG2RAD)

/* PWM (ARR = 4199) */
#define PWM_MAX_COUNTS  4199u

#define PRESET_YAW_ANGLE  27.0f //NEED CHANGE
#define PRESET_PITCH_ANGLE 40.0f //NEED CHANGE

#define ZERO 1.86669159f //checking
#define RIGHT_YAW_LIMIT_DEG  ((ZERO / DEG2RAD)  + 10.0f)  //NEED TO CHECK COULD BE INVERTED
#define LEFT_YAW_LIMIT_DEG  ((ZERO / DEG2RAD) - 10.0f) //NEED TO CHECK COULD BE INVERTED


#define TORQUE_MAX 125

#define UPPER_PITCH_LIMIT 45.0f
#define LOWER_PITCH_LIMIT 35.0f

#endif /* SRC_TASKS_PITCHNYAWTASK_H_ */
