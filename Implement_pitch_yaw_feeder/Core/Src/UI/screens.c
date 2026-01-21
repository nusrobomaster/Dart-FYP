#include <string.h>

#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"
#include "../ui_interface.h"

#include <string.h>

/* Event callback declarations */
static void numpad_event_cb(lv_event_t *e);
static void btn_set_pitch_cb(lv_event_t *e);
static void btn_set_yaw_cb(lv_event_t *e);
static void btn_set_force_cb(lv_event_t *e);
static void btn_reload_cb(lv_event_t *e);

objects_t objects;
lv_obj_t *tick_value_change_obj;

void create_screen_main_page() {
    void *flowState = getFlowState(0, 0);
    (void)flowState;
    lv_obj_t *obj = lv_obj_create(0);
    objects.main_page = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    {
        lv_obj_t *parent_obj = obj;
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.obj0 = obj;
            lv_obj_set_pos(obj, 14, 14);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "");
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.obj1 = obj;
            lv_obj_set_pos(obj, 14, 59);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "");
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.obj2 = obj;
            lv_obj_set_pos(obj, 14, 107);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "");
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.obj3 = obj;
            lv_obj_set_pos(obj, 14, 160);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "");
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.obj4 = obj;
            lv_obj_set_pos(obj, 16, 212);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "");
        }
        {
            // Launcher_Matrix_4
            lv_obj_t *obj = lv_buttonmatrix_create(parent_obj);
            objects.launcher_matrix_4 = obj;
            lv_obj_set_pos(obj, 212, 73);
            lv_obj_set_size(obj, 243, 206);
            static const char *map[16] = {
                "1",
                "2",
                "3",
                "\n",
                "4",
                "5",
                "6",
                "\n",
                "7",
                "8",
                "9",
                "\n",
                "DEL",
                "0",
                "OK",
                NULL,
            };
            static lv_buttonmatrix_ctrl_t ctrl_map[12] = {
                1 | LV_BUTTONMATRIX_CTRL_CLICK_TRIG,
                1 | LV_BUTTONMATRIX_CTRL_CLICK_TRIG,
                1 | LV_BUTTONMATRIX_CTRL_CLICK_TRIG,
                1 | LV_BUTTONMATRIX_CTRL_CLICK_TRIG,
                1 | LV_BUTTONMATRIX_CTRL_CLICK_TRIG,
                1 | LV_BUTTONMATRIX_CTRL_CLICK_TRIG,
                1 | LV_BUTTONMATRIX_CTRL_CLICK_TRIG,
                1 | LV_BUTTONMATRIX_CTRL_CLICK_TRIG,
                1 | LV_BUTTONMATRIX_CTRL_CLICK_TRIG,
                1 | LV_BUTTONMATRIX_CTRL_CLICK_TRIG,
                1 | LV_BUTTONMATRIX_CTRL_CLICK_TRIG,
                1 | LV_BUTTONMATRIX_CTRL_CLICK_TRIG,
            };
            lv_buttonmatrix_set_map(obj, map);
            lv_buttonmatrix_set_ctrl_map(obj, ctrl_map);
        }
        {
            /* Input textarea for number pad */
            lv_obj_t *obj = lv_textarea_create(parent_obj);
            objects.input_textarea = obj;
            lv_obj_set_pos(obj, 212, 29);
            lv_obj_set_size(obj, 243, 30);
            lv_textarea_set_max_length(obj, 16);
            lv_textarea_set_one_line(obj, true);
            lv_textarea_set_password_mode(obj, false);
            lv_textarea_set_placeholder_text(obj, "Enter value...");
        }
        {
            /* Set Pitch Angle button */
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.btn_set_pitch = obj;
            lv_obj_set_pos(obj, 14, 30);
            lv_obj_set_size(obj, 90, 35);
            lv_obj_add_flag(obj, LV_OBJ_FLAG_CHECKABLE);
            lv_obj_add_event_cb(obj, btn_set_pitch_cb, LV_EVENT_CLICKED, NULL);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Set Pitch");
                }
            }
        }
        {
            /* Set Yaw Angle button */
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.btn_set_yaw = obj;
            lv_obj_set_pos(obj, 110, 30);
            lv_obj_set_size(obj, 90, 35);
            lv_obj_add_flag(obj, LV_OBJ_FLAG_CHECKABLE);
            lv_obj_add_event_cb(obj, btn_set_yaw_cb, LV_EVENT_CLICKED, NULL);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Set Yaw");
                }
            }
        }
        {
            /* Set Force button */
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.btn_set_force = obj;
            lv_obj_set_pos(obj, 14, 180);
            lv_obj_set_size(obj, 90, 35);
            lv_obj_add_flag(obj, LV_OBJ_FLAG_CHECKABLE);
            lv_obj_add_event_cb(obj, btn_set_force_cb, LV_EVENT_CLICKED, NULL);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Set Force");
                }
            }
        }
        {
            /* Reload button */
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.btn_reload = obj;
            lv_obj_set_pos(obj, 110, 180);
            lv_obj_set_size(obj, 90, 35);
            lv_obj_add_event_cb(obj, btn_reload_cb, LV_EVENT_CLICKED, NULL);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Reload");
                }
            }
        }
        {
            /* Labels for displaying set and current values */
            /* Set Pitch value label */
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.lbl_set_pitch = obj;
            lv_obj_set_pos(obj, 14, 70);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Set Pitch: 0.0");
        }
        {
            /* Current Pitch value label */
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.lbl_current_pitch = obj;
            lv_obj_set_pos(obj, 14, 90);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Current Pitch: 0.0");
        }
        {
            /* Set Yaw value label */
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.lbl_set_yaw = obj;
            lv_obj_set_pos(obj, 14, 115);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Set Yaw: 0.0");
        }
        {
            /* Current Yaw value label */
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.lbl_current_yaw = obj;
            lv_obj_set_pos(obj, 14, 135);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Current Yaw: 0.0");
        }
        {
            /* Set Force value label */
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.lbl_set_force = obj;
            lv_obj_set_pos(obj, 14, 220);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Set Force: 0.0");
        }
    }
    
    /* Add event callback for number pad */
    lv_obj_add_event_cb(objects.launcher_matrix_4, numpad_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    /* Initialize UI interface */
    ui_interface_init();
    
    tick_screen_main_page();
}

void tick_screen_main_page() {
    void *flowState = getFlowState(0, 0);
    (void)flowState;
    {
        const char *new_val = evalTextProperty(flowState, 0, 3, "Failed to evaluate Text in Label widget");
        const char *cur_val = lv_label_get_text(objects.obj0);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.obj0;
            lv_label_set_text(objects.obj0, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = evalTextProperty(flowState, 2, 3, "Failed to evaluate Text in Label widget");
        const char *cur_val = lv_label_get_text(objects.obj1);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.obj1;
            lv_label_set_text(objects.obj1, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = evalTextProperty(flowState, 3, 3, "Failed to evaluate Text in Label widget");
        const char *cur_val = lv_label_get_text(objects.obj2);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.obj2;
            lv_label_set_text(objects.obj2, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = evalTextProperty(flowState, 4, 3, "Failed to evaluate Text in Label widget");
        const char *cur_val = lv_label_get_text(objects.obj3);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.obj3;
            lv_label_set_text(objects.obj3, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = evalTextProperty(flowState, 5, 3, "Failed to evaluate Text in Label widget");
        const char *cur_val = lv_label_get_text(objects.obj4);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.obj4;
            lv_label_set_text(objects.obj4, new_val);
            tick_value_change_obj = NULL;
        }
    }
}


static const char *screen_names[] = { "Main Page" };
static const char *object_names[] = { "main_page", "launcher_matrix_4", "obj0", "obj1", "obj2", "obj3", "obj4" };


typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_main_page,
};
void tick_screen(int screen_index) {
    tick_screen_funcs[screen_index]();
}
void tick_screen_by_id(enum ScreensEnum screenId) {
    tick_screen_funcs[screenId - 1]();
}

void create_screens() {
    eez_flow_init_screen_names(screen_names, sizeof(screen_names) / sizeof(const char *));
    eez_flow_init_object_names(object_names, sizeof(object_names) / sizeof(const char *));
    
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    
    create_screen_main_page();
}

/* ========================= Event Callbacks ========================= */

/**
 * @brief Number pad button matrix event callback
 */
static void numpad_event_cb(lv_event_t *e)
{
    lv_obj_t *btnm = lv_event_get_target(e);
    uint16_t btn_id = lv_btnmatrix_get_selected_btn(btnm);
    const char *txt = lv_btnmatrix_get_btn_text(btnm, btn_id);
    
    if (txt != NULL) {
        ui_interface_numpad_press(txt);
    }
}

/**
 * @brief Set Pitch button click callback
 */
static void btn_set_pitch_cb(lv_event_t *e)
{
    (void)e;
    ui_interface_set_selection(SELECT_PITCH_ANGLE);
}

/**
 * @brief Set Yaw button click callback
 */
static void btn_set_yaw_cb(lv_event_t *e)
{
    (void)e;
    ui_interface_set_selection(SELECT_YAW_ANGLE);
}

/**
 * @brief Set Force button click callback
 */
static void btn_set_force_cb(lv_event_t *e)
{
    (void)e;
    ui_interface_set_selection(SELECT_LAUNCHER_FORCE);
}

/**
 * @brief Reload button click callback
 */
static void btn_reload_cb(lv_event_t *e)
{
    (void)e;
    ui_interface_trigger_reload();
}
