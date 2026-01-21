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

/* Local input buffer for number pad */
static char input_buffer[16] = {0};
static uint8_t input_len = 0;

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
    
    memset(input_buffer, 0, sizeof(input_buffer));
    input_len = 0;
}

/* ========================= Number Pad Handling ========================= */

void ui_interface_numpad_press(const char *btn_text)
{
    if (btn_text == NULL) {
        return;
    }
    
    if (strcmp(btn_text, "DEL") == 0) {
        /* Delete last character */
        if (input_len > 0) {
            input_len--;
            input_buffer[input_len] = '\0';
        }
        /* Update textarea */
        if (objects.input_textarea != NULL) {
            lv_textarea_set_text(objects.input_textarea, input_buffer);
        }
    }
    else if (strcmp(btn_text, "OK") == 0) {
        /* Apply the value */
        ui_interface_apply_value();
    }
    else {
        /* It's a digit - add to buffer if space available */
        if (input_len < sizeof(input_buffer) - 1) {
            /* Handle decimal point */
            if (strcmp(btn_text, ".") == 0) {
                /* Only allow one decimal point */
                if (strchr(input_buffer, '.') == NULL) {
                    input_buffer[input_len++] = '.';
                    input_buffer[input_len] = '\0';
                }
            }
            else {
                input_buffer[input_len++] = btn_text[0];
                input_buffer[input_len] = '\0';
            }
            
            /* Update textarea */
            if (objects.input_textarea != NULL) {
                lv_textarea_set_text(objects.input_textarea, input_buffer);
            }
        }
    }
}

/* ========================= Selection Mode ========================= */

void ui_interface_set_selection(ui_selection_mode_t mode)
{
    ui_state.selection_mode = mode;
    
    /* Update button styles to show selection */
    ui_interface_update_button_styles();
}

ui_selection_mode_t ui_interface_get_selection(void)
{
    return ui_state.selection_mode;
}

/* ========================= Button Style Updates ========================= */

void ui_interface_update_button_styles(void)
{
    /* Reset all buttons to normal style */
    if (objects.btn_set_pitch != NULL) {
        lv_obj_remove_state(objects.btn_set_pitch, LV_STATE_CHECKED);
        lv_obj_set_style_text_font(lv_obj_get_child(objects.btn_set_pitch, 0), 
                                    LV_FONT_DEFAULT, LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    if (objects.btn_set_yaw != NULL) {
        lv_obj_remove_state(objects.btn_set_yaw, LV_STATE_CHECKED);
        lv_obj_set_style_text_font(lv_obj_get_child(objects.btn_set_yaw, 0), 
                                    LV_FONT_DEFAULT, LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    if (objects.btn_set_force != NULL) {
        lv_obj_remove_state(objects.btn_set_force, LV_STATE_CHECKED);
        lv_obj_set_style_text_font(lv_obj_get_child(objects.btn_set_force, 0), 
                                    LV_FONT_DEFAULT, LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    
    /* Highlight selected button with bold text */
    lv_obj_t *selected_btn = NULL;
    
    switch (ui_state.selection_mode) {
        case SELECT_PITCH_ANGLE:
            selected_btn = objects.btn_set_pitch;
            break;
        case SELECT_YAW_ANGLE:
            selected_btn = objects.btn_set_yaw;
            break;
        case SELECT_LAUNCHER_FORCE:
            selected_btn = objects.btn_set_force;
            break;
        default:
            break;
    }
    
    if (selected_btn != NULL) {
        lv_obj_add_state(selected_btn, LV_STATE_CHECKED);
        /* Make text bold by using a different font or style */
        lv_obj_t *label = lv_obj_get_child(selected_btn, 0);
        if (label != NULL) {
            /* Use heavier/bold style through font weight */
            lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 
                                        LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
}

/* ========================= Value Application ========================= */

void ui_interface_apply_value(void)
{
    if (input_len == 0) {
        return;  /* No value to apply */
    }
    
    float value = (float)atof(input_buffer);
    
    switch (ui_state.selection_mode) {
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
    
    /* Clear input after applying */
    ui_interface_clear_input();
    
    /* Update display to show new set values */
    ui_interface_update_display();
}

/* ========================= Input Management ========================= */

const char* ui_interface_get_input_text(void)
{
    return input_buffer;
}

void ui_interface_clear_input(void)
{
    memset(input_buffer, 0, sizeof(input_buffer));
    input_len = 0;
    
    if (objects.input_textarea != NULL) {
        lv_textarea_set_text(objects.input_textarea, "");
    }
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
    
    /* Update pitch labels */
    if (objects.lbl_set_pitch != NULL) {
        snprintf(buf, sizeof(buf), "Set Pitch: %.1f", ui_state.set_pitch_angle);
        lv_label_set_text(objects.lbl_set_pitch, buf);
    }
    
    if (objects.lbl_current_pitch != NULL) {
        snprintf(buf, sizeof(buf), "Current Pitch: %.1f", ui_state.current_pitch_angle);
        lv_label_set_text(objects.lbl_current_pitch, buf);
    }
    
    /* Update yaw labels */
    if (objects.lbl_set_yaw != NULL) {
        snprintf(buf, sizeof(buf), "Set Yaw: %.1f", ui_state.set_yaw_angle);
        lv_label_set_text(objects.lbl_set_yaw, buf);
    }
    
    if (objects.lbl_current_yaw != NULL) {
        snprintf(buf, sizeof(buf), "Current Yaw: %.1f", ui_state.current_yaw_angle);
        lv_label_set_text(objects.lbl_current_yaw, buf);
    }
    
    /* Update launcher force label */
    if (objects.lbl_set_force != NULL) {
        snprintf(buf, sizeof(buf), "Set Force: %.1f", ui_state.set_launcher_force);
        lv_label_set_text(objects.lbl_set_force, buf);
    }
}
