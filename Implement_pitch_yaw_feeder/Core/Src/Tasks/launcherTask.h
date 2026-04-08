/*
 * launcherTask.h
 *
 *  Created on: Jan 19, 2026
 *      Author: Barry Ang
 */

#ifndef SRC_TASKS_LAUNCHERTASK_H_
#define SRC_TASKS_LAUNCHERTASK_H_
/* Feeder--------------------------------------------------------*/
typedef enum {
	POS_1_0,
	POS_1_1,
	UNLOAD_1,
	PAUSE_1,
	RELOAD_1,
	POS_2_0,
	POS_2_1,
	UNLOAD_2,
	PAUSE_2_1,
	PAUSE_2_2,
	PAUSE_2_3,
	PAUSE_2_4,
	RELOAD_2,
	POS_3_0,
	POS_3_1,
	UNLOAD_3,
	PAUSE_3,
	RELOAD_3,
	POS_4_0
} feeder_state_t;
#define UNLOAD_MS  500

#define KP_SET_FEEDER 15.0f
#define KD_SET_FEEDER 1.5f

//#define POS_1_TUNING -1.6f
//#define POS_2_TUNING -5.0f
//#define POS_3_TUNING -10.0f
//#define POS_4_TUNING 0.0f

#define POS_1_TUNING 0.0f
#define POS_2_TUNING 0.0f
#define POS_3_TUNING 0.0f
#define POS_4_TUNING 0.0f

#define DEG_TO_RAD        (3.14159265358979323846f / 180.0f)

#define POS_DEG_INC       60.0f

#define POS_DEG_TOL	      10.0f
#define POS_DEG_1_0		  180.0f

#define POS_DEG_1_1_1       (POS_DEG_1_0 + POS_DEG_INC)
#define POS_DEG_1_1         (POS_DEG_1_1_1 + POS_1_TUNING)
#define POS_DEG_2_0_1         (POS_DEG_1_1_1 + POS_DEG_INC)
#define POS_DEG_2_0         (POS_DEG_2_0_1)
#define POS_DEG_2_1_1         (POS_DEG_2_0_1 + POS_DEG_INC)
#define POS_DEG_2_1         (POS_DEG_2_1_1 + POS_2_TUNING)
#define POS_DEG_3_0_1         (POS_DEG_2_1_1 + POS_DEG_INC)
#define POS_DEG_3_0         (POS_DEG_3_0_1)
#define POS_DEG_3_1_1         (POS_DEG_3_0_1 + POS_DEG_INC)
#define POS_DEG_3_1         (POS_DEG_3_1_1 + POS_3_TUNING)
#define POS_DEG_4_0_1         (POS_DEG_3_1_1 + POS_DEG_INC)
#define POS_DEG_4_0         (POS_DEG_4_0_1)



#define POS_RAD_TOL       	(POS_DEG_TOL * DEG_TO_RAD)
#define POS_RAD_1_0         (POS_DEG_1_0 * DEG_TO_RAD)
#define POS_RAD_1_1         (POS_DEG_1_1 * DEG_TO_RAD)
#define POS_RAD_1_1_0		4.1644f
#define POS_RAD_2_0         (POS_DEG_2_0 * DEG_TO_RAD)
#define POS_RAD_2_1_0		6.24f
#define POS_RAD_2_1_1		6.13f
#define POS_RAD_2_1         (POS_DEG_2_1 * DEG_TO_RAD)
#define POS_RAD_3_0         (POS_DEG_3_0 * DEG_TO_RAD)
#define POS_RAD_3_1         (POS_DEG_3_1 * DEG_TO_RAD)
#define POS_RAD_3_1_0		8.3282f
#define POS_RAD_4_0			(POS_DEG_4_0 * DEG_TO_RAD)

#define POS_RAD_1_KP  15.0f
#define POS_RAD_1_KD  1.5f

#define POS_RAD_2_KP  15.0f
#define POS_RAD_2_KD  1.5f

#define POS_RAD_3_KP  15.0f
#define POS_RAD_3_KD  1.5f

// might need change more
#define UNLOAD_DURATION  1000
#define LOOP_DELAY       0


//TODO configure damiao angle for UNLOAD_1 and UNLOAD_3
#define LOAD_ANGLE_1 0.0f
#define UNLOAD_ANGLE_1  30.0f

#define LOAD_ANGLE_2 0.0f
#define UNLOAD_ANGLE_2  45.0f
#define UNLOAD_ANGLE_2_0  50.0f

#define LOAD_ANGLE_3 0.0f
#define UNLOAD_ANGLE_3  30.0f



/* Launcher motor: per-state KP, KD, TOR (and VEL where used) for dm_launching_motor */

/* Launcher--------------------------------------------------------*/

//TODO figure out the distance that hit the back of the fin

#define PAUSE_1_DIST -8.76f
#define PAUSE_2_123DIST 4.0f
#define PAUSE_2_4DIST -4.0f
#define PAUSE_3_DIST -11.0f
#define DIST_TOL 0.4f

typedef enum {
LAUNCHER_WAIT,
SEEK_LOCK_CW,
LOCKED,
FEED,
REVERSE_TO_WEIGHT,
HOLD
} launcher_state_t;

enum LaunchernFeederState {
	WAIT = 0,
	FIRING = 1
};

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
	#define LAUNCHER_REVERSE_POS_KP    2.0f   /* proportional gain for position -> velocity */
	#define LAUNCHER_REVERSE_POS_KI	   0.0f   /* integral gain (used in inline PI controller) */
	#define LAUNCHER_REVERSE_POS_KD    0.3f
	#define LAUNCHER_REVERSE_POS_TARGET 10.0f   /* desired launcher_abs_position in rad */
	#define LAUNCHER_REVERSE_POS_VEL_MAX 2.0f /* clamp velocity command */
	#define TOL 3.0f
#endif
#define LAUNCHER_REVERSE_KP  0.0f
#define LAUNCHER_REVERSE_KD  5.0f
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
#define LAUNCHER_SERVO_ANGLE_RELEASE  11.0f   /* angle to trigger dart release */
#define LAUNCHER_SERVO_ANGLE_REST     100.0f  /* angle when not firing (idle) */
#define LAUNCHER_SERVO_RELEASE_MS     200     /* hold release this long before moving back */

/* Testing (manual velocity control) */
#define LAUNCHER_TEST_KD    4.0f
#define MAX_VELOCITY        100.0f

#endif /* SRC_TASKS_LAUNCHERTASK_H_ */
