#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _objects_t {
    lv_obj_t *main_page;
    lv_obj_t *launcher_matrix_4;
    lv_obj_t *obj0;
    lv_obj_t *obj1;
    lv_obj_t *obj2;
    lv_obj_t *obj3;
    lv_obj_t *obj4;
    
    /* Additional UI elements for subsystem control */
    lv_obj_t *input_textarea;       /* Number input textarea */
    lv_obj_t *btn_set_pitch;        /* Set pitch angle button */
    lv_obj_t *btn_set_yaw;          /* Set yaw angle button */
    lv_obj_t *btn_set_force;        /* Set launcher force button */
    lv_obj_t *btn_reload;           /* Feeder reload button */
    
    /* Labels for displaying values */
    lv_obj_t *lbl_set_pitch;        /* Display set pitch value */
    lv_obj_t *lbl_current_pitch;    /* Display current pitch value */
    lv_obj_t *lbl_set_yaw;          /* Display set yaw value */
    lv_obj_t *lbl_current_yaw;      /* Display current yaw value */
    lv_obj_t *lbl_set_force;        /* Display set force value */
} objects_t;

extern objects_t objects;

enum ScreensEnum {
    SCREEN_ID_MAIN_PAGE = 1,
};

void create_screen_main_page();
void tick_screen_main_page();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/