/*
 * display_tft.c
 *
 *  Created on: 17 Aug 2026
 *      Author: samfishy
 */
#include "display_tft.h"
#include "flash.h"
#include "st7735.h"
#include "vl53l5cx_api.h"
#include "arm_math.h"

extern q15_t out_q15[1024];

uint32_t ID;
uint8_t bytes[2052];
uint8_t rec_high_bytes[1029];
uint8_t rec_low_bytes[1024];

int r_flag = 0, interpolation = 1,mode = 0,ini = 0,img = 0;
int interpolation_menu = 0 ,mode_menu = 0, img_str = 0,menu_mode = 0;
int16_t rData_int[1024];

volatile bool flag_update_heatmap = false;
volatile bool flag_update_labels = false;
volatile bool flag_use_rdata = false;

extern VL53L5CX_Configuration  Dev;

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
   ini = 1;
}

void image(int x, int mode_menu, int interpolation_menu)
{
    int x1 = x*16+100;
    W25_fRead(x1, 0,1028, rec_high_bytes);
    HAL_Delay(10);
    W25_fRead(x1+4, 4,1024, rec_low_bytes);

    mode_menu = rec_high_bytes[1026];
    interpolation_menu = rec_high_bytes[1027];

    for (int i = 0; i < 1024; i++)
    {
         rData_int[i] = ((uint16_t)rec_high_bytes[i] << 8) | (uint16_t)rec_low_bytes[i];
    }
}

void img_store(int img_num,int size, int mode, int interpolation)
{
    int img_page = 100+img_num*16;

    for (int i = 0; i < size; i++)
    {
        bytes[i] = (((uint16_t)out_q15[i]) >> 8) & 0xFF;
        bytes[1028+i]  =  ((uint16_t)out_q15[i]) & 0xFF;
    }

    bytes[1026] = (uint8_t)mode;
    bytes[1027] = (uint8_t)interpolation;

    W25Q_Write_Page(img_page, 0, 2052, bytes);
}

void capture_img(int mode, int interpolation)
{

    vl53l5cx_stop_ranging(&Dev);

    uint8_t img_wq = (uint8_t)img;
    W25Q_Write_Page(0, 0, 1, &img_wq);
    img_store(img,1024,mode,interpolation);
    img_str = img;

    if(img < 10) img++;
    else img = 0;

    vl53l5cx_start_ranging(&Dev);
}
