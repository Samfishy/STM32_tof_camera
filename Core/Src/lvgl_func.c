/*
 * lvgl_func.c
 *
 *  Created on: 17 Aug 2026
 *      Author: samfishy
 */
#include "lvgl_func.h"
#include "st7735.h"
#include "vl53l5cx_api.h"
#include "../../grapics/lvgl.h"
#include "ui.h"
#include "vars.h"
#include "display_tft.h"
#include "strings.h"
#include "interpolation_heatmap.h"
#include "arm_math.h"

#define SCALE 4
#define IN_LEN 64
#define OUT_LEN (IN_LEN * SCALE)

#define CANVAS_WIDTH  145
#define CANVAS_HEIGHT 105

char loading_screen_input[100] = "Initialising....";
char view_type_str[10] = "LIVE View";
char page_num_str[10] = "Page 1/0";
char status_str[10] = "LIVE View";
char zone_mode[10] = "8x8 zones";
char zone_int[10] = "1x int";

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[160*20];
volatile lv_disp_drv_t disp_drv;
static lv_color_t cbuf[CANVAS_HEIGHT*CANVAS_WIDTH];

lv_color_t map[1024];

extern int interpolation_menu, mode_menu, img_str, menu_mode, img, ini;

extern int IN_W;
extern int IN_H;
extern int OUT_W;
extern int OUT_H;
extern volatile bool flag_update_heatmap;
extern volatile bool flag_update_labels;
extern volatile bool flag_use_rdata;
extern VL53L5CX_Configuration  Dev;
extern VL53L5CX_ResultsData    Results;        /* Results data from VL53L5CX */
extern q15_t out_q15;
extern int16_t rData_int;
extern uint16_t stored_img;

void text_update(int var, char *text)
{
	if(var == 1)
	{
		strcpy(view_type_str, text);
	}
	else if(var == 2)
	{
		strcpy(page_num_str, text);
	}
	else if(var == 3)
	{
		strcpy(status_str, text);
	}
	else if(var == 4)
	{
		strcpy(zone_int, text);
	}
	else if(var == 5)
	{
		strcpy(zone_mode, text);
	}
	else if(var == 6)
	{
		strcpy(loading_screen_input, text);
	}
}

void lvgl_page_loader(uint8_t page)
{
	loadScreen(page);
	lv_timer_handler();
}

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    uint16_t x = (uint16_t)area->x1;
    uint16_t y = (uint16_t)area->y1;
    uint16_t w = (uint16_t)(area->x2 - area->x1 + 1);
    uint16_t h = (uint16_t)(area->y2 - area->y1 + 1);

    uint16_t * px_map = (uint16_t *)color_p;

    ST7735_DrawImage_DMA(x, y, w, h, px_map);
}

void img_print_lvgl(int interp, int md, int16_t data[])
{
    if (!objects.heat_map) return;

    int mode1 = (md == 0) ? 8 : 4;
    int total = (mode1 * interp);
    int total_sq = total * total;
    int rec_use = CANVAS_WIDTH / total;

    for(int i = 0; i < total_sq; i++)
    {
        int dis = (int)(data[i]) / 10;
        heatmap_plotting(dis, i);
    }

    // LVGL 8.4 Canvas Drawing Descriptor
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.radius = 0;
    rect_dsc.border_width = 0;

    for (int col = 0; col < total; col++)
    {
        for (int row = 0; row < total; row++)
        {
            int idx = row + col * total;

            int x = ((total - 1) - row) * rec_use;
            int y = col * rec_use;

            rect_dsc.bg_color = map[idx];

            lv_canvas_draw_rect(objects.heat_map, x, y, rec_use, rec_use, &rect_dsc);
        }
    }
}

void lvgl_main_init()
{
	lv_init();
	lv_disp_draw_buf_init(&draw_buf, buf1, NULL, 160*20);
	lv_disp_drv_init(&disp_drv);

	disp_drv.hor_res = 160;
	disp_drv.ver_res = 128;
	disp_drv.flush_cb = my_flush_cb;
	disp_drv.draw_buf = &draw_buf;

	lv_disp_drv_register(&disp_drv);
	ui_init();

	if(objects.heat_map != NULL)
	{
		lv_canvas_set_buffer(objects.heat_map, cbuf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);
	}
	if(objects.view_type) lv_label_set_text(objects.view_type, view_type_str);
	if(objects.page_num) lv_label_set_text(objects.page_num, page_num_str);
}

void lvgl_screen_redering(int interpolation, int interpolation_menu, int mode, int mode_menu, int screen_mode)
{
    lv_timer_handler();
    ui_tick();
    if (flag_update_heatmap && screen_mode == 1)
    {
    	lvgl_page_loader(2);
        flag_update_heatmap = false;
        if(flag_use_rdata)
        {
            if(interpolation != 1){img_print_lvgl(interpolation, mode, &out_q15);}
            else {img_print_lvgl(interpolation_menu, mode_menu, &stored_img);}
        }
        else
        {
            if(interpolation != 1){img_print_lvgl(interpolation, mode, &out_q15);}
            else {img_print_lvgl(interpolation, mode, Results.distance_mm);}
        }
    }
    else if(screen_mode == 0)
    {
    	lvgl_page_loader(1);
        lv_timer_handler();
        ui_tick();
    }
}

int lvgl_intToggle(int menu_mode,int interpolation)
{
    if(menu_mode == 0)
    {
        vl53l5cx_stop_ranging(&Dev);

        if (interpolation == 1)
        {
            interpolation = 2;
            text_update(4, "2x int");
        }
        else if (interpolation == 2)
        {
            interpolation = 4;
            text_update(4, "4x int");
        }
        else
        {
            interpolation = 1;
            text_update(4, "1x int");
        }

        OUT_W = IN_W * interpolation;
        OUT_H = IN_H * interpolation;
        flag_update_labels = true;

        vl53l5cx_start_ranging(&Dev);
    }
    else
    {
        if(img_str < img ) img_str ++;
        else img_str = 0;

        image(img_str);
        flag_use_rdata = true;
        flag_update_heatmap = true;
    }

    return interpolation;
}

int lvgl_zoneToggle(int menu_mode, int mode, int interpolation)
{
	if(menu_mode == 0)
	{
		vl53l5cx_stop_ranging(&Dev);

	    // Cycle Mode: 8x8 -> 4x4 -> 8x8
	    if(mode == 0)
	    {
	       mode = 1;
	       vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_4X4);
	       IN_W = 4; IN_H = 4;
	       text_update(5, "4x4 zones");
	    }
	    else
	    {
	    	mode = 0;
	        vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_8X8);
	        IN_W = 8; IN_H = 8;
	        text_update(5, "8x8 zones");
	     }

	     OUT_W = IN_W * interpolation;
	     OUT_H = IN_H * interpolation;
	     flag_update_labels = true;

	     vl53l5cx_start_ranging(&Dev);
	}
	else
	{
		if(img_str > 0 ) img_str--;
	    else img_str = img;

		image(img_str);
	    flag_use_rdata = true;
	    flag_update_heatmap = true;
	}

	return mode;
}

int lvgl_menuToggle(int mode,int menu_mode, int interpolation)
{
    if(menu_mode == 0)
    {
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
        menu_mode = 1;

    	text_update(1, "REC MODE");
    	text_update(2, "Page 1/0");

        if(interpolation == 1)      text_update(4, "1x int");
        else if(interpolation == 2) text_update(4, "2x int");
        else text_update(4, "4x int");

        vl53l5cx_stop_ranging(&Dev);
    }
    else
    {
        menu_mode = 0;

    	text_update(1, "LIVE View");
    	text_update(2, "*Page 1/0");

        if(mode == 0) text_update(5, "8x8 zones");
        else 		  text_update(5, "4x4 zones");

        if(interpolation == 1)      text_update(4, "1x int");
        else if(interpolation == 2) text_update(4, "2x int");
        else text_update(4, "4x int");

        text_update(3, "LIVE view");
        flag_update_labels = true;

        vl53l5cx_start_ranging(&Dev);

        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    }

    return menu_mode;
}

const char * get_var_loading_screen_input(void)
{
    return loading_screen_input;
}

const char * get_var_view_type(void)
{
    return view_type_str;
}

const char * get_var_page(void)
{
    return page_num_str;
}

const char * get_var_mode_zone(void)
{
    return zone_mode;
}
const char * get_var_mode_int(void)
{
    return zone_int;
}
