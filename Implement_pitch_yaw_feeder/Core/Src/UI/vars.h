#ifndef EEZ_LVGL_UI_VARS_H
#define EEZ_LVGL_UI_VARS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// enum declarations



// Flow global variables

enum FlowGlobalVariables {
    FLOW_GLOBAL_VARIABLE_CURRENT_PITCH_ANGLE = 0,
    FLOW_GLOBAL_VARIABLE_CURRENT_YAW_ANGLE = 1,
    FLOW_GLOBAL_VARIABLE_SET_LAUNCHER_FORCE = 2,
    FLOW_GLOBAL_VARIABLE_SET_PITCH_ANGLE = 3,
    FLOW_GLOBAL_VARIABLE_SET_YAW_ANGLE = 4
};

// Native global variables



#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_VARS_H*/