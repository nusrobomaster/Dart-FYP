/*
 * launcherTask.h
 *
 *  Created on: Jan 19, 2026
 *      Author: Barry Ang
 */

#ifndef SRC_TASKS_LAUNCHERTASK_H_
#define SRC_TASKS_LAUNCHERTASK_H_

/* Launcher motor: per-state KP, KD, TOR (and VEL where used) for dm_launching_motor */

/* LnF_state WAIT (idle) */
#define LAUNCHER_IDLE_KP    0.0f
#define LAUNCHER_IDLE_KD    0.0f
#define LAUNCHER_IDLE_TOR   0.0f

/* LauncherState WAIT (in FIRING) */
#define LAUNCHER_WAIT_KP    LAUNCHER_IDLE_KP
#define LAUNCHER_WAIT_KD    LAUNCHER_IDLE_KD
#define LAUNCHER_WAIT_TOR   LAUNCHER_IDLE_TOR 

/* SEEK_LOCK_CW */
#define LAUNCHER_SEEK_LOCK_KP   4.0f
#define LAUNCHER_SEEK_LOCK_KD   1.0f
#define LAUNCHER_SEEK_LOCK_TOR  0.0f
#define LAUNCHER_SEEK_LOCK_VEL  -12.0f

/* LOCKED (tensioning) */
#define LAUNCHER_LOCKED_KP   4.0f
#define LAUNCHER_LOCKED_KD   1.0f
#define LAUNCHER_LOCKED_TOR  1.0f
#define LAUNCHER_LOCKED_VEL  12.0f

/* FEED */
#define LAUNCHER_FEED_KP    LAUNCHER_IDLE_KP
#define LAUNCHER_FEED_KD    LAUNCHER_IDLE_KD
#define LAUNCHER_FEED_TOR   0.0f

/* REVERSE_TO_WEIGHT: motor ctrl gains + torque PID for v_cmd */
#if LAUNCH_CONTROL == 1
	#define LAUNCHER_REVERSE_VEL  4.0f
#endif
#define LAUNCHER_REVERSE_KP  4.0f
#define LAUNCHER_REVERSE_KD  1.0f
#define LAUNCHER_REVERSE_TOR 0.0f
/* Torque PID: e = T_target - T_measured, output v_cmd (clamped) */
#define REVERSE_TORQUE_PID_KP  0.5f
#define REVERSE_TORQUE_PID_KI  0.05f
#define REVERSE_TORQUE_PID_KD  0.02f
#define REVERSE_V_MIN  -20.0f
#define REVERSE_V_MAX   20.0f

/* HOLD */
#define LAUNCHER_HOLD_KP    4.0f
#define LAUNCHER_HOLD_KD    1.0f
#define LAUNCHER_HOLD_TOR   0.0f

/* Launcher release servo (TIM4_CH4): angle to fire, then back to rest */
#define LAUNCHER_SERVO_ANGLE_RELEASE  90.0f   /* angle to trigger dart release */
#define LAUNCHER_SERVO_ANGLE_REST     180.0f  /* angle when not firing (idle) */
#define LAUNCHER_SERVO_RELEASE_MS     200     /* hold release this long before moving back */

/* Testing (manual velocity control) */
#define LAUNCHER_TEST_KD    4.0f
#define MAX_VELOCITY        100.0f

#endif /* SRC_TASKS_LAUNCHERTASK_H_ */
