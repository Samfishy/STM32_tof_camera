#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// Screens

enum ScreensEnum {
    _SCREEN_ID_FIRST = 1,
    SCREEN_ID_LIVE_VIEW_PAGE = 1,
    SCREEN_ID_HEAT_MAP_PAGE = 2,
    _SCREEN_ID_LAST = 2
};

typedef struct _objects_t {
    lv_obj_t *live_view_page;
    lv_obj_t *heat_map_page;
    lv_obj_t *obj0;
    lv_obj_t *loading_screen;
    lv_obj_t *obj1;
    lv_obj_t *heat_map;
    lv_obj_t *view_type;
    lv_obj_t *page_num;
    lv_obj_t *min;
    lv_obj_t *modes_int;
    lv_obj_t *modes_int_1;
} objects_t;

extern objects_t objects;

void create_screen_live_view_page();
void tick_screen_live_view_page();

void create_screen_heat_map_page();
void tick_screen_heat_map_page();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/