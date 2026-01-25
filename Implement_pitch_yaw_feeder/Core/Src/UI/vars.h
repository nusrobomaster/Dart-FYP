#ifndef EEZ_LVGL_UI_VARS_H
#define EEZ_LVGL_UI_VARS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// enum declarations

typedef enum {
    Subsystem_Launcher = 0,
    Subsystem_Pitch = 1,
    Subsystem_Yaw = 2
} Subsystem;

// Flow global variables

enum FlowGlobalVariables {
    FLOW_GLOBAL_VARIABLE_CURRENT_YAW_ANGLE = 0,
    FLOW_GLOBAL_VARIABLE_SET_YAW_ANGLE = 1,
    FLOW_GLOBAL_VARIABLE_KEYPAD_TEXT = 2,
    FLOW_GLOBAL_VARIABLE_SELECT_SUBSYSTEM = 3,
    FLOW_GLOBAL_VARIABLE_CURRENT_PITCH_ANGLE = 4,
    FLOW_GLOBAL_VARIABLE_SET_PITCH_ANGLE = 5,
    FLOW_GLOBAL_VARIABLE_CURRENT_LAUNCHER_FORCE = 6,
    FLOW_GLOBAL_VARIABLE_SET_LAUNCHER_FORCE = 7,
    FLOW_GLOBAL_VARIABLE_YAW = 8,
    FLOW_GLOBAL_VARIABLE_PITCH = 9,
    FLOW_GLOBAL_VARIABLE_LAUNCHER = 10
};

// Native global variables



#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_VARS_H*/