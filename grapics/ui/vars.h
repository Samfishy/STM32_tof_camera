#ifndef EEZ_LVGL_UI_VARS_H
#define EEZ_LVGL_UI_VARS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// enum declarations

// Flow global variables

enum FlowGlobalVariables {
    FLOW_GLOBAL_VARIABLE_LOADING_SCREEN_INPUT = 0,
    FLOW_GLOBAL_VARIABLE_VIEW_TYPE = 1,
    FLOW_GLOBAL_VARIABLE_PAGE = 2,
    FLOW_GLOBAL_VARIABLE_MODE_INT = 3,
    FLOW_GLOBAL_VARIABLE_MODE_ZONE = 4
};

// Native global variables

extern const char *get_var_loading_screen_input();
extern void set_var_loading_screen_input(const char *value);
extern const char *get_var_view_type();
extern void set_var_view_type(const char *value);
extern const char *get_var_page();
extern void set_var_page(const char *value);
extern const char *get_var_mode_int();
extern void set_var_mode_int(const char *value);
extern const char *get_var_mode_zone();
extern void set_var_mode_zone(const char *value);

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_VARS_H*/