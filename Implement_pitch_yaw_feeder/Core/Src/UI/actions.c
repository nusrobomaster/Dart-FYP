/*
 * actions.c
 *
 * UI action handlers for EEZ flow events
 *
 * Created on: Jan 21, 2026
 */

#include "actions.h"
#include "screens.h"
#include "../ui_interface.h"
#include <string.h>
#include <stdlib.h>

/**
 * @brief Action handler for button matrix (number pad) events
 * 
 * This function is called when a button on any of the number pads is pressed.
 * It handles digits (0-9), DEL (delete), and OK (apply value).
 * 
 * @param e LVGL event pointer
 */
void action_button_mat(lv_event_t *e)
{
    lv_obj_t *btnm = lv_event_get_target(e);
    uint16_t btn_id = lv_btnmatrix_get_selected_btn(btnm);
    const char *txt = lv_btnmatrix_get_btn_text(btnm, btn_id);
    
    if (txt == NULL) {
        return;
    }
    
    /* Determine which textarea to update based on current selection */
    lv_obj_t *textarea = NULL;
    ui_selection_mode_t mode = ui_interface_get_selection();
    
    switch (mode) {
        case SELECT_YAW_ANGLE:
            textarea = objects.yaw_text_area;
            break;
        case SELECT_PITCH_ANGLE:
            textarea = objects.pitch_text_area;
            break;
        case SELECT_LAUNCHER_FORCE:
            textarea = objects.launcher_text_area;
            break;
        default:
            return;  /* No selection, do nothing */
    }
    
    if (textarea == NULL) {
        return;
    }
    
    /* Handle button press */
    if (strcmp(txt, "DEL") == 0) {
        /* Delete last character */
        lv_textarea_delete_char(textarea);
    }
    else if (strcmp(txt, "OK") == 0) {
        /* Apply the value from textarea */
        const char *value_str = lv_textarea_get_text(textarea);
        if (value_str != NULL && strlen(value_str) > 0) {
            float value = (float)atof(value_str);
            ui_interface_apply_value_direct(mode, value);
            
            /* Clear textarea after applying */
            lv_textarea_set_text(textarea, "");
        }
    }
    else {
        /* It's a digit or decimal point - add to textarea */
        /* Check for decimal point - only allow one */
        if (strcmp(txt, ".") == 0) {
            const char *current_text = lv_textarea_get_text(textarea);
            if (strchr(current_text, '.') != NULL) {
                return;  /* Already has decimal point */
            }
        }
        lv_textarea_add_text(textarea, txt);
    }
}

/**
 * @brief Action handler for UI selection (page change)
 * 
 * This function is called when navigating to a different screen.
 * It sets the selection mode based on which screen is being displayed.
 * 
 * @param e LVGL event pointer
 */
void action_ui_selection(lv_event_t *e)
{
    lv_obj_t *target = lv_event_get_target(e);
    (void)target;  /* May be used for more specific handling */
    
    /* Determine which screen we're on by checking the button matrices */
    /* This is called from the button matrix events, so we can check which one */
    
    if (target == objects.yaw_button_matrix) {
        ui_interface_set_selection(SELECT_YAW_ANGLE);
    }
    else if (target == objects.pitch_button_matrix) {
        ui_interface_set_selection(SELECT_PITCH_ANGLE);
    }
    else if (target == objects.launcher_button_matrix) {
        ui_interface_set_selection(SELECT_LAUNCHER_FORCE);
    }
    /* If navigating from front page buttons to sub-pages */
    else if (target == objects.obj0) {  /* Yaw button on front page */
        ui_interface_set_selection(SELECT_YAW_ANGLE);
    }
    else if (target == objects.obj1) {  /* Pitch button on front page */
        ui_interface_set_selection(SELECT_PITCH_ANGLE);
    }
    else if (target == objects.obj2) {  /* Launcher button on front page */
        ui_interface_set_selection(SELECT_LAUNCHER_FORCE);
    }
    else {
        /* Back button or unknown - clear selection */
        ui_interface_set_selection(SELECT_NONE);
    }
}
