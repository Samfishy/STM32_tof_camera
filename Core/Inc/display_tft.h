/*
 * display_tft.h
 *
 *  Created on: 17 Aug 2026
 *      Author: samfishy
 */

#ifndef INC_DISPLAY_TFT_H_
#define INC_DISPLAY_TFT_H_

#include "main.h"

void TFT_init();
uint32_t flashMEM_init();
void image(int x, int mode_menu, int interpolation_menu);
void img_store(int img_num,int size, int mode, int interpolation);
void capture_img(int mode, int interpolation);

#endif /* INC_DISPLAY_TFT_H_ */
