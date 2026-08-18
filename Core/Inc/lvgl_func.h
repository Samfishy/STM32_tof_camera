/*
 * lvgl_func.h
 *
 *  Created on: 17 Aug 2026
 *      Author: samfishy
 */

#ifndef INC_LVGL_FUNC_H_
#define INC_LVGL_FUNC_H_

#include "main.h"
#include "stdbool.h"
#include "vl53l5cx_api.h"

void lvgl_main_init();
void text_update(int var, char *text);
void lvgl_page_loader(uint8_t page);
void lvgl_screen_redering(int interpolation, int interpolation_menu, int mode, int mode_menu, int screen_mode);

int lvgl_intToggle(int menu_mode,int interpolation);
int lvgl_zoneToggle(int menu_mode, int mode, int interpolation);
int lvgl_menuToggle(int mode,int menu_mode, int interpolation);

#endif /* INC_LVGL_FUNC_H_ */
