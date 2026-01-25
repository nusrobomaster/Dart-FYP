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

static ui_selection_mode_t selection_from_dropdown(void)
{
    if (objects.obj0 == NULL) {
        return ui_interface_get_selection();
    }

    /* Dropdown options: "Launcher", "Pitch", "Yaw" */
    int32_t selected = lv_dropdown_get_selected(objects.obj0);
    switch (selected) {
        case 0:
            return SELECT_LAUNCHER_FORCE;
        case 1:
            return SELECT_PITCH_ANGLE;
        case 2:
            return SELECT_YAW_ANGLE;
        default:
            return SELECT_NONE;
    }
}

/**
 * @brief Action handler for button matrix (number pad) events
 * 
 * This function is called when a button on the number pad is pressed.
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

    /* Single shared textarea in the new UI */
    lv_obj_t *textarea = objects.yaw_text_area;
    if (textarea == NULL) {
        return;
    }

    /* Determine selected subsystem from dropdown */
    ui_selection_mode_t mode = selection_from_dropdown();
    ui_interface_set_selection(mode);

    /* Handle button press */
    if (strcmp(txt, "DEL") == 0) {
        lv_textarea_delete_char(textarea);
    } else if (strcmp(txt, "OK") == 0) {
        const char *value_str = lv_textarea_get_text(textarea);
        if (value_str != NULL && strlen(value_str) > 0) {
            float value = (float)atof(value_str);
            ui_interface_apply_value_direct(mode, value);
            lv_textarea_set_text(textarea, "");
        }
    } else {
        /* It's a digit or decimal point - add to textarea */
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

    if (target == objects.obj0) {
        ui_interface_set_selection(selection_from_dropdown());
    } else {
        ui_interface_set_selection(SELECT_NONE);
    }
}

//void action_trigger_reload(lv_event_t * e){
//
// }
//void action_trigger_fire(lv_event_t * e){
//
//}
//void action_trigger_draw(lv_event_t * e){
//
// }

