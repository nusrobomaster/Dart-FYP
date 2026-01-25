/*
 * ui_interface.c
 *
 * Interface between UI and subsystem tasks (pitch, yaw, launcher, feeder)
 *
 * Created on: Jan 21, 2026
 */

#include "ui_interface.h"
#include "UI/screens.h"
#include <stdbool.h>
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
    ui_state.fire_request = false;
    ui_state.draw_request = false;

}

bool ui_interface_check_draw_request(void)
{
    if (ui_state.draw_request) {
        ui_state.draw_request = false;
        return true;
    }
    return false;
}

void ui_interface_trigger_draw(void)
{
    ui_state.draw_request = true;
}

void ui_interface_trigger_fire(void)
{
    ui_state.fire_request = true;
}

bool ui_interface_check_fire_request(void)
{
	if (ui_state.fire_request) {
	        ui_state.fire_request = false;
	        return true;
	    }
	 return false;
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
    
    /* Update YAW labels */
    if (objects.yaw_set != NULL) {
        snprintf(buf, sizeof(buf), "Set Yaw Angle: %.1f", ui_state.set_yaw_angle);
        lv_label_set_text(objects.yaw_set, buf);
    }

    if (objects.yaw_cu != NULL) {
        snprintf(buf, sizeof(buf), "Current Yaw Angle: %.1f", ui_state.current_yaw_angle);
        lv_label_set_text(objects.yaw_cu, buf);
    }

    /* Update PITCH labels */
    if (objects.pitch_set != NULL) {
        snprintf(buf, sizeof(buf), "Set Pitch Angle: %.1f", ui_state.set_pitch_angle);
        lv_label_set_text(objects.pitch_set, buf);
    }

    if (objects.pitch_cur != NULL) {
        snprintf(buf, sizeof(buf), "Current Pitch Angle: %.1f", ui_state.current_pitch_angle);
        lv_label_set_text(objects.pitch_cur, buf);
    }

    /* Update LAUNCHER labels */
    if (objects.launcher_set != NULL) {
        snprintf(buf, sizeof(buf), "Set Launcher Force: %.1f", ui_state.set_launcher_force);
        lv_label_set_text(objects.launcher_set, buf);
    }

    if (objects.launcher_cur != NULL) {
        snprintf(buf, sizeof(buf), "Current Launcher Force: %.1f", ui_state.current_launcher_force);
        lv_label_set_text(objects.launcher_cur, buf);
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
    if (objects.yaw_text_area == NULL) {
        return "";
    }
    return lv_textarea_get_text(objects.yaw_text_area);
}

void ui_interface_clear_input(void)
{
    if (objects.yaw_text_area == NULL) {
        return;
    }
    lv_textarea_set_text(objects.yaw_text_area, "");
}
