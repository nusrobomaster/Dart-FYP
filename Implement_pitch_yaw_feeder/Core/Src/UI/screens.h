#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _objects_t {
    lv_obj_t *front_page;
    lv_obj_t *yaw;
    lv_obj_t *pitch;
    lv_obj_t *launcher;
    lv_obj_t *obj0;
    lv_obj_t *obj1;
    lv_obj_t *obj2;
    lv_obj_t *obj3;
    lv_obj_t *yaw_button_matrix;
    lv_obj_t *obj4;
    lv_obj_t *pitch_button_matrix;
    lv_obj_t *obj5;
    lv_obj_t *launcher_button_matrix;
    lv_obj_t *yaw_text_area;
    lv_obj_t *obj6;
    lv_obj_t *obj7;
    lv_obj_t *pitch_text_area;
    lv_obj_t *obj8;
    lv_obj_t *obj9;
    lv_obj_t *launcher_text_area;
    lv_obj_t *obj10;
    lv_obj_t *obj11;
} objects_t;

extern objects_t objects;

enum ScreensEnum {
    SCREEN_ID_FRONT_PAGE = 1,
    SCREEN_ID_YAW = 2,
    SCREEN_ID_PITCH = 3,
    SCREEN_ID_LAUNCHER = 4,
};

void create_screen_front_page();
void tick_screen_front_page();

void create_screen_yaw();
void tick_screen_yaw();

void create_screen_pitch();
void tick_screen_pitch();

void create_screen_launcher();
void tick_screen_launcher();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/