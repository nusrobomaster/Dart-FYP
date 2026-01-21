/*
 * ui_interface.c
 *
 * Interface between UI and subsystem tasks (pitch, yaw, launcher, feeder)
 *
 * Created on: Jan 21, 2026
 */

#include "ui_interface.h"
#include "UI/screens.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Global UI control state */
ui_control_state_t ui_state = {0};

/* ========================= Initialization ========================= */

void ui_interface_init(void)
{
    ui_state.selection_mode = SELECT_NONE;
    ui_state.set_pitch_angle = 0.0f;
    ui_state.set_yaw_angle = 0.0f;
    ui_state.set_launcher_force = 0.0f;
    ui_state.current_pitch_angle = 0.0f;
    ui_state.current_yaw_angle = 0.0f;
    ui_state.feeder_reload_request = false;
    ui_state.values_updated = false;
}

/* ========================= Selection Mode ========================= */

void ui_interface_set_selection(ui_selection_mode_t mode)
{
    ui_state.selection_mode = mode;
}

ui_selection_mode_t ui_interface_get_selection(void)
{
    return ui_state.selection_mode;
}

/* ========================= Value Application ========================= */

void ui_interface_apply_value(void)
{
    /* This function is kept for compatibility but 
       the main logic is now in ui_interface_apply_value_direct */
}

void ui_interface_apply_value_direct(ui_selection_mode_t mode, float value)
{
    switch (mode) {
        case SELECT_PITCH_ANGLE:
            ui_state.set_pitch_angle = value;
            ui_state.values_updated = true;
            break;
        case SELECT_YAW_ANGLE:
            ui_state.set_yaw_angle = value;
            ui_state.values_updated = true;
            break;
        case SELECT_LAUNCHER_FORCE:
            ui_state.set_launcher_force = value;
            ui_state.values_updated = true;
            break;
        default:
            /* No selection - don't apply */
            break;
    }
    
    /* Update display to show new set values */
    ui_interface_update_display();
}

/* ========================= Feeder Reload ========================= */

void ui_interface_trigger_reload(void)
{
    ui_state.feeder_reload_request = true;
}

bool ui_interface_check_reload_request(void)
{
    if (ui_state.feeder_reload_request) {
        ui_state.feeder_reload_request = false;
        return true;
    }
    return false;
}

/* ========================= Value Getters ========================= */

void ui_interface_update_current_values(float pitch_angle, float yaw_angle)
{
    ui_state.current_pitch_angle = pitch_angle;
    ui_state.current_yaw_angle = yaw_angle;
}

float ui_interface_get_set_pitch(void)
{
    return ui_state.set_pitch_angle;
}

float ui_interface_get_set_yaw(void)
{
    return ui_state.set_yaw_angle;
}

float ui_interface_get_set_force(void)
{
    return ui_state.set_launcher_force;
}

/* ========================= Display Update ========================= */

void ui_interface_update_display(void)
{
    char buf[64];
    
    /* Update YAW screen labels (obj6 = current angle, obj7 = set angle) */
    if (objects.obj7 != NULL) {
        snprintf(buf, sizeof(buf), "Set Yaw Angle: %.1f", ui_state.set_yaw_angle);
        lv_label_set_text(objects.obj7, buf);
    }
    
    if (objects.obj6 != NULL) {
        snprintf(buf, sizeof(buf), "Current Angle: %.1f", ui_state.current_yaw_angle);
        lv_label_set_text(objects.obj6, buf);
    }
    
    /* Update PITCH screen labels (obj8 = current angle, obj9 = set angle) */
    if (objects.obj9 != NULL) {
        snprintf(buf, sizeof(buf), "Set Angle: %.1f", ui_state.set_pitch_angle);
        lv_label_set_text(objects.obj9, buf);
    }
    
    if (objects.obj8 != NULL) {
        snprintf(buf, sizeof(buf), "Current Angle: %.1f", ui_state.current_pitch_angle);
        lv_label_set_text(objects.obj8, buf);
    }
    
    /* Update LAUNCHER screen labels (obj10 = current force, obj11 = set force) */
    if (objects.obj11 != NULL) {
        snprintf(buf, sizeof(buf), "Set Launcher Force: %.1f", ui_state.set_launcher_force);
        lv_label_set_text(objects.obj11, buf);
    }
    
    if (objects.obj10 != NULL) {
        snprintf(buf, sizeof(buf), "Current Launcher Force: %.1f", ui_state.current_launcher_force);
        lv_label_set_text(objects.obj10, buf);
    }
}

/* ========================= Deprecated Functions ========================= */
/* These are kept for backward compatibility but may not be used */

void ui_interface_numpad_press(const char *btn_text)
{
    (void)btn_text;
    /* Number pad handling is now done in actions.c action_button_mat() */
}

void ui_interface_update_button_styles(void)
{
    /* Button styles are handled by page selection now */
}

const char* ui_interface_get_input_text(void)
{
    return "";
}

void ui_interface_clear_input(void)
{
    /* Input clearing is now done directly on textareas in actions.c */
}
