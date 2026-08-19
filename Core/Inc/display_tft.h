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
int image(int x);
void img_store(int mode, int img_num);
void capture_img(int mode, int interpolation);

#endif /* INC_DISPLAY_TFT_H_ */
