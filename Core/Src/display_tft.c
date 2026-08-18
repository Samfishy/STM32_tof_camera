/*
 * display_tft.c
 *
 *  Created on: 17 Aug 2026
 *      Author: samfishy
 */
#include "lvgl_func.h"
#include "display_tft.h"
#include "flash.h"
#include "st7735.h"
#include "vl53l5cx_api.h"
#include "arm_math.h"

extern q15_t out_q15[1024];

uint32_t ID;
uint8_t bytes[2048];
uint16_t stored_img[100];

int r_flag = 0, interpolation = 1,mode = 0,ini = 0,img = 0;
int interpolation_menu = 0 ,mode_menu = 0, img_str = 0,menu_mode = 0;
int16_t rData_int[1024];

volatile bool flag_update_heatmap = false;
volatile bool flag_update_labels = false;
volatile bool flag_use_rdata = false;

extern VL53L5CX_Configuration  Dev;
extern VL53L5CX_ResultsData    Results;        /* Results data from VL53L5CX */

void TFT_init()
{
	ST7735_Init();
	ST7735_FillScreen(ST7735_BLACK);
}

uint32_t flashMEM_init()
{
   W25_rst();
   uint32_t ID = W25_devID();
   return ID;
}

int image(int x)
{
    int img_page = 100 + x;
    W25_fRead(img_page, 0, 129, bytes);

    int mode_menu = (int)bytes[0];
    int num_zones = (mode_menu == 1 || mode_menu == 0) ? 16 : 64;

    if(mode_menu == 1){text_update(5, "4x4 zones");}
    else{text_update(5, "8x8 zones");}

    for (int i = 0; i < num_zones; i++)
    {
    	stored_img[i] = ((uint16_t)bytes[1 + (i * 2)] << 8) | (uint16_t)bytes[2 + (i * 2)];
    }

    return num_zones;
}

void img_store(int mode, int img_num)
{
    int img_page = 100 + img_num;
    int num_zones = (mode == 1 || mode == 0) ? 16 : 64;

    bytes[0] = (uint8_t)mode;

    for (int i = 0; i < num_zones; i++)
    {
        bytes[1 + (i * 2)] = ((uint16_t)Results.distance_mm[i] >> 8) & 0xFF;
        bytes[2 + (i * 2)] = (uint16_t)Results.distance_mm[i] & 0xFF;
    }

    W25Q_Write_Page(img_page, 0, 2 + (num_zones * 2), bytes);
}

void capture_img(int mode, int interpolation)
{
    vl53l5cx_stop_ranging(&Dev);

    img_store(mode,img);
    img_str = img;

    if (img < 10)
        img++;
    else
        img = 0;

    vl53l5cx_start_ranging(&Dev);
}
