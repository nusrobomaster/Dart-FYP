#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _objects_t {
    lv_obj_t *yaw;
    lv_obj_t *yaw_button_matrix;
    lv_obj_t *yaw_text_area;
    lv_obj_t *yaw_set;
    lv_obj_t *yaw_cu;
    lv_obj_t *obj0;
    lv_obj_t *pitch_set;
    lv_obj_t *pitch_cur;
    lv_obj_t *launcher_set;
    lv_obj_t *launcher_cur;
    lv_obj_t *obj1;
} objects_t;

extern objects_t objects;

enum ScreensEnum {
    SCREEN_ID_YAW = 1,
};

void create_screen_yaw();
void tick_screen_yaw();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/