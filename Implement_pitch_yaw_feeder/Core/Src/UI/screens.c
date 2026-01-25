#include <string.h>

#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"

#include <string.h>

objects_t objects;
lv_obj_t *tick_value_change_obj;

static void event_handler_cb_yaw_yaw_button_matrix(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_CLICKED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 0, 0, e);
    }
}

static void event_handler_cb_yaw_yaw_text_area(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target(e);
        if (tick_value_change_obj != ta) {
            const char *value = lv_textarea_get_text(ta);
            assignStringProperty(flowState, 2, 3, value, "Failed to assign Text in Textarea widget");
        }
    }
}

static void event_handler_cb_yaw_obj0(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target(e);
        if (tick_value_change_obj != ta) {
            int32_t value = lv_dropdown_get_selected(ta);
            assignIntegerProperty(flowState, 5, 3, value, "Failed to assign Selected in Dropdown widget");
        }
    }
}

void create_screen_yaw() {
    void *flowState = getFlowState(0, 0);
    (void)flowState;
    lv_obj_t *obj = lv_obj_create(0);
    objects.yaw = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 320);
    {
        lv_obj_t *parent_obj = obj;
        {
            // yaw_button_matrix
            lv_obj_t *obj = lv_buttonmatrix_create(parent_obj);
            objects.yaw_button_matrix = obj;
            lv_obj_set_pos(obj, 217, 81);
            lv_obj_set_size(obj, 243, 206);
            static const char *map[17] = {
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
                ".",
                "OK",
                NULL,
            };
            static lv_buttonmatrix_ctrl_t ctrl_map[13] = {
                1 | LV_BUTTONMATRIX_CTRL_CLICK_TRIG,
                1 | LV_BUTTONMATRIX_CTRL_CLICK_TRIG,
                1,
                1,
                1,
                1,
                1,
                1,
                1,
                1,
                1,
                1,
                1,
            };
            lv_buttonmatrix_set_map(obj, map);
            lv_buttonmatrix_set_ctrl_map(obj, ctrl_map);
            lv_obj_add_event_cb(obj, event_handler_cb_yaw_yaw_button_matrix, LV_EVENT_ALL, flowState);
        }
        {
            // yaw_text_area
            lv_obj_t *obj = lv_textarea_create(parent_obj);
            objects.yaw_text_area = obj;
            lv_obj_set_pos(obj, 240, 24);
            lv_obj_set_size(obj, 198, 35);
            lv_textarea_set_max_length(obj, 128);
            lv_textarea_set_placeholder_text(obj, "Fill in the blank");
            lv_textarea_set_one_line(obj, false);
            lv_textarea_set_password_mode(obj, false);
            lv_obj_add_event_cb(obj, event_handler_cb_yaw_yaw_text_area, LV_EVENT_ALL, flowState);
        }
        {
            // yaw_set
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.yaw_set = obj;
            lv_obj_set_pos(obj, 13, 128);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "");
        }
        {
            // yaw_cu
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.yaw_cu = obj;
            lv_obj_set_pos(obj, 13, 91);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "");
        }
        {
            lv_obj_t *obj = lv_dropdown_create(parent_obj);
            objects.obj0 = obj;
            lv_obj_set_pos(obj, 31, 24);
            lv_obj_set_size(obj, 112, LV_SIZE_CONTENT);
            lv_dropdown_set_options(obj, "Launcher\nPitch\nYaw");
            lv_obj_add_event_cb(obj, event_handler_cb_yaw_obj0, LV_EVENT_ALL, flowState);
        }
        {
            // pitch_set
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.pitch_set = obj;
            lv_obj_set_pos(obj, 15, 128);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "");
        }
        {
            // pitch_cur
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.pitch_cur = obj;
            lv_obj_set_pos(obj, 13, 92);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "");
        }
        {
            // launcher_set
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.launcher_set = obj;
            lv_obj_set_pos(obj, 9, 144);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "");
        }
        {
            // launcher_cur
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.launcher_cur = obj;
            lv_obj_set_pos(obj, 9, 101);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "");
        }
        {
            lv_obj_t *obj = lv_button_create(parent_obj);
            objects.obj1 = obj;
            lv_obj_set_pos(obj, 69, 209);
            lv_obj_set_size(obj, 74, 50);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 0, 0);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Fire");
                }
            }
        }
    }
    
    tick_screen_yaw();
}

void tick_screen_yaw() {
    void *flowState = getFlowState(0, 0);
    (void)flowState;
    {
        const char *new_val = evalTextProperty(flowState, 2, 3, "Failed to evaluate Text in Textarea widget");
        const char *cur_val = lv_textarea_get_text(objects.yaw_text_area);
        uint32_t max_length = lv_textarea_get_max_length(objects.yaw_text_area);
        if (strncmp(new_val, cur_val, max_length) != 0) {
            tick_value_change_obj = objects.yaw_text_area;
            lv_textarea_set_text(objects.yaw_text_area, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 3, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.yaw_set, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.yaw_set;
            if (new_val) lv_obj_add_flag(objects.yaw_set, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(objects.yaw_set, LV_OBJ_FLAG_HIDDEN);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = evalTextProperty(flowState, 3, 4, "Failed to evaluate Text in Label widget");
        const char *cur_val = lv_label_get_text(objects.yaw_set);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.yaw_set;
            lv_label_set_text(objects.yaw_set, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 4, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.yaw_cu, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.yaw_cu;
            if (new_val) lv_obj_add_flag(objects.yaw_cu, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(objects.yaw_cu, LV_OBJ_FLAG_HIDDEN);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = evalTextProperty(flowState, 4, 4, "Failed to evaluate Text in Label widget");
        const char *cur_val = lv_label_get_text(objects.yaw_cu);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.yaw_cu;
            lv_label_set_text(objects.yaw_cu, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        if (!(lv_obj_get_state(objects.obj0) & LV_STATE_EDITED)) {
            int32_t new_val = evalIntegerProperty(flowState, 5, 3, "Failed to evaluate Selected in Dropdown widget");
            int32_t cur_val = lv_dropdown_get_selected(objects.obj0);
            if (new_val != cur_val) {
                tick_value_change_obj = objects.obj0;
                lv_dropdown_set_selected(objects.obj0, new_val);
                tick_value_change_obj = NULL;
            }
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 6, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.pitch_set, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.pitch_set;
            if (new_val) lv_obj_add_flag(objects.pitch_set, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(objects.pitch_set, LV_OBJ_FLAG_HIDDEN);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = evalTextProperty(flowState, 6, 4, "Failed to evaluate Text in Label widget");
        const char *cur_val = lv_label_get_text(objects.pitch_set);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.pitch_set;
            lv_label_set_text(objects.pitch_set, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 7, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.pitch_cur, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.pitch_cur;
            if (new_val) lv_obj_add_flag(objects.pitch_cur, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(objects.pitch_cur, LV_OBJ_FLAG_HIDDEN);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = evalTextProperty(flowState, 7, 4, "Failed to evaluate Text in Label widget");
        const char *cur_val = lv_label_get_text(objects.pitch_cur);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.pitch_cur;
            lv_label_set_text(objects.pitch_cur, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 8, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.launcher_set, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.launcher_set;
            if (new_val) lv_obj_add_flag(objects.launcher_set, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(objects.launcher_set, LV_OBJ_FLAG_HIDDEN);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = evalTextProperty(flowState, 8, 4, "Failed to evaluate Text in Label widget");
        const char *cur_val = lv_label_get_text(objects.launcher_set);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.launcher_set;
            lv_label_set_text(objects.launcher_set, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 9, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.launcher_cur, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.launcher_cur;
            if (new_val) lv_obj_add_flag(objects.launcher_cur, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(objects.launcher_cur, LV_OBJ_FLAG_HIDDEN);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = evalTextProperty(flowState, 9, 4, "Failed to evaluate Text in Label widget");
        const char *cur_val = lv_label_get_text(objects.launcher_cur);
        if (strcmp(new_val, cur_val) != 0) {
            tick_value_change_obj = objects.launcher_cur;
            lv_label_set_text(objects.launcher_cur, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 10, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.obj1, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.obj1;
            if (new_val) lv_obj_add_flag(objects.obj1, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(objects.obj1, LV_OBJ_FLAG_HIDDEN);
            tick_value_change_obj = NULL;
        }
    }
}


static const char *screen_names[] = { "yaw" };
static const char *object_names[] = { "yaw", "yaw_button_matrix", "yaw_text_area", "yaw_set", "yaw_cu", "obj0", "pitch_set", "pitch_cur", "launcher_set", "launcher_cur", "obj1" };


typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_yaw,
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
    
    create_screen_yaw();
}
