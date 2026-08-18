/*
 * interpolation_heatmap.h
 *
 *  Created on: 18 Aug 2026
 *      Author: samfishy
 */

#ifndef INC_INTERPOLATION_HEATMAP_H_
#define INC_INTERPOLATION_HEATMAP_H_

void bilinear_init_q15(void);
void load_input_int16_to_q15(const int16_t raw[IN_W * IN_H]);
void bilinear_8x8_to_16x16_q15(void);
void heatmap_plotting(int dis, int i);

#endif /* INC_INTERPOLATION_HEATMAP_H_ */
