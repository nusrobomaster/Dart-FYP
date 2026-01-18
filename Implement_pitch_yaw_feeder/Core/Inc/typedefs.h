/*
 * typedefs.h
 *
 *  Created on: May 23, 2021
 *      Author: wx
 */

#ifndef TASKS_INC_TYPEDEFS_H_
#define TASKS_INC_TYPEDEFS_H_

#include "FreeRTOS.h"
#include <semphr.h>
#include <event_groups.h>

typedef struct {
    float kp;        // Proportional gain
    float ki;        // Integral gain
    float kd;        // Derivative gain
    float prev_error; // Previous error
    float integral;  // Integral of the error
    float max_output; // Maximum control signal
    float min_output; // Minimum control signal
    float output;
} PID;

typedef struct
{
	PID inner;
	PID outer;
	float output;
}CascadePID;


typedef struct
{
    uint8_t mode;
    //PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�

} PidTypeDef;

//switch goes from 1-3-2 from top to down
enum left_switch{
	ge_LSW_UNSAFE = 1,
	ge_LSW_CONFIG = 3,
	ge_LSW_STANDBY = 2
};

enum right_switch
{
	ge_RSW_SHUTDOWN = 1,
	ge_RSW_GIMBAL = 3,
	ge_RSW_ALL_ON = 2,
};

#define KEYBOARD_CTRL_MODE	1
#define REMOTE_CTRL_MODE	2
#define SBC_CTRL_MODE 		3

enum rb_mode_t{
	ge_RB_KEYBOARD,
	ge_RB_REMOTE,
	ge_RB_SBC_CTRL
};

typedef struct
{
	float kp;
	float ki;
	float kd;
	float error[2];
	float integral;
	float int_max;
	float max_out;
	float output;
	float physical_max;
	uint32_t last_time[2];
}pid_data_t;


/* Struct containing cleaned data from remote */
typedef struct {
	/* Joysticks - Values range from -660 to 660 */
	int16_t right_x;
	int16_t right_y;
	int16_t left_x;
	int16_t left_y;
	/* Switches - Values range from 1 - 3 */
	int8_t left_switch;
	int8_t right_switch;
	/* Mouse movement - Values range from -32768 to 32767 */
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int32_t mouse_hori;
	int32_t mouse_vert;
	/* Mouse clicks - Values range from 0 to 1 */
	int8_t mouse_left;
	int8_t mouse_right;

	/* Keyboard keys mapping
	 * Bit0 -- W 键
	 * Bit1 -- S 键
	 *	Bit2 -- A 键
	 *	Bit3 -- D 键
	 *	Bit4 -- Q 键
	 *	Bit5 -- E 键
	 *	Bit6 -- Shift 键
	 *	Bit7 -- Ctrl 键
	 *
	 */
	uint16_t keyboard_keys;
	int16_t side_dial;
	uint32_t last_time;
} remote_cmd_t;




#endif /* TASKS_INC_TYPEDEFS_H_ */
