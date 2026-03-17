/*
 * launcherTask.h
 *
 *  Created on: Jan 19, 2026
 *      Author: Barry Ang
 */

#ifndef SRC_TASKS_LAUNCHERTASK_H_
#define SRC_TASKS_LAUNCHERTASK_H_

#define LOAD_ANGLE_1 0.0f
#define UNLOAD_ANGLE_1  90.0f
#define UNLOAD_ANGLE_1_0  150.0f

#define LOAD_ANGLE_2 0.0f
#define UNLOAD_ANGLE_2  90.0f
#define UNLOAD_ANGLE_2_0  150.0f

#define LOAD_ANGLE_3 0.0f
#define UNLOAD_ANGLE_3  90.0f
#define UNLOAD_ANGLE_3_0  150.0f

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
#define LAUNCHER_SEEK_LOCK_KP   0.0f
#define LAUNCHER_SEEK_LOCK_KD   4.0f
#define LAUNCHER_SEEK_LOCK_TOR  0.0f
#define LAUNCHER_SEEK_LOCK_VEL  -4.0f

/* LOCKED (tensioning) */
#define LAUNCHER_LOCKED_KP   0.0f
#define LAUNCHER_LOCKED_KD   4.0f
#define LAUNCHER_LOCKED_TOR  0.0f
#define LAUNCHER_LOCKED_VEL  4.0f

/* FEED */
#define LAUNCHER_FEED_KP    LAUNCHER_IDLE_KP
#define LAUNCHER_FEED_KD    LAUNCHER_IDLE_KD
#define LAUNCHER_FEED_TOR   0.0f

/* REVERSE_TO_WEIGHT: motor ctrl gains + torque PID for v_cmd */
#if LAUNCH_CONTROL == 1
	#define LAUNCHER_REVERSE_VEL  4.0f
#endif
#if LAUNCH_CONTROL == 2
	/* P-only velocity control to reach desired launcher_abs_position (rad) */
	#define LAUNCHER_REVERSE_POS_KP    1.0f   /* proportional gain for position -> velocity */
	#define LAUNCHER_REVERSE_POS_KI	   0.0f   /* integral gain (used in inline PI controller) */
	#define LAUNCHER_REVERSE_POS_KD    0.3f
	#define LAUNCHER_REVERSE_POS_TARGET 24.0f   /* desired launcher_abs_position in rad */
	#define LAUNCHER_REVERSE_POS_TOL   1.0f  /* rad, consider at target */
	#define LAUNCHER_REVERSE_POS_VEL_MAX 1.0f /* clamp velocity command */
#endif
#define LAUNCHER_REVERSE_KP  0.0f
#define LAUNCHER_REVERSE_KD  2.0f
#define LAUNCHER_REVERSE_TOR 0.0f
/* Torque PID: e = T_target - T_measured, output v_cmd (clamped) */
#define REVERSE_TORQUE_PID_KP  0.5f
#define REVERSE_TORQUE_PID_KI  0.05f
#define REVERSE_TORQUE_PID_KD  0.02f
#define REVERSE_V_MIN  -20.0f
#define REVERSE_V_MAX   20.0f
#define TOL 1.0f

/* HOLD */
#define LAUNCHER_HOLD_KP    4.0f
#define LAUNCHER_HOLD_KD    1.0f
#define LAUNCHER_HOLD_TOR   0.0f

/* Launcher release servo (TIM4_CH4): angle to fire, then back to rest */
#define LAUNCHER_SERVO_ANGLE_RELEASE  8.0f   /* angle to trigger dart release */
#define LAUNCHER_SERVO_ANGLE_REST     100.0f  /* angle when not firing (idle) */
#define LAUNCHER_SERVO_RELEASE_MS     200     /* hold release this long before moving back */

/* Testing (manual velocity control) */
#define LAUNCHER_TEST_KD    4.0f
#define MAX_VELOCITY        100.0f

#endif /* SRC_TASKS_LAUNCHERTASK_H_ */
