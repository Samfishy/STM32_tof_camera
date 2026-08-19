/*
 * interpolation_heatmap.c
 *
 *  Created on: 18 Aug 2026
 *      Author: samfishy
 */

#include "main.h"
#include "arm_math.h"
#include "../../grapics/lvgl.h"

int IN_W = 8;
int IN_H = 8;
int OUT_W  = 8;
int OUT_H  = 8;   // 16

arm_bilinear_interp_instance_q15 S;
q15_t grid_q15[64];
q15_t out_q15[1024];
q15_t in_q15[64];

extern lv_color_t map[1024];

void bilinear_init_q15(void)
{
    S.numRows = IN_H;
    S.numCols = IN_W;
    S.pData   = grid_q15;
}

void load_input_int16_to_q15(const int16_t raw[IN_W * IN_H])
{
    for (uint32_t i = 0; i < IN_W * IN_H; i++)
    {
        grid_q15[i] = (q15_t)raw[i];
    }
}

void bilinear_8x8_to_16x16_q15(void)
{
    // Define the absolute maximum boundaries in Q12.20 fixed-point format
    q31_t max_x = (IN_W - 1) << 20;
    q31_t max_y = (IN_H - 1) << 20;

    for (int y = 0; y < OUT_H; y++)
    {
        q31_t src_y = ((((q31_t)y << 20) + (1 << 19)) * IN_H) / OUT_H - (1 << 19);

        // Clamp Y to prevent reading past the top or bottom edges
        if (src_y < 0) src_y = 0;
        if (src_y > max_y) src_y = max_y;

        for (int x = 0; x < OUT_W; x++)
        {
            q31_t src_x = ((((q31_t)x << 20) + (1 << 19)) * IN_W) / OUT_W - (1 << 19);

            // Clamp X to prevent reading past the right edge (which prevents left-edge mirroring)
            if (src_x < 0) src_x = 0;
            if (src_x > max_x) src_x = max_x;

            out_q15[y * OUT_W + x] = arm_bilinear_interp_q15(&S, src_x, src_y);
        }
    }
}

// Array of 30 colors sampled from Birtmap_color.png, converted to RGB565
const uint8_t heatmap_colors[30][3] = {
    // {R,   G,   B}
    {49,   0,   0},   //  0: 0x3000 dark red
    {82,   0,   0},   //  1: 0x5000
    {123,  0,   0},   //  2: 0x7800
    {165,  0,   0},   //  3: 0xA000
    {206,  0,   0},   //  4: 0xC800
    {255,  0,   0},   //  5: 0xF800 red
    {255, 65,   0},   //  6: 0xFA00 red-orange
    {255, 130,  0},   //  7: 0xFC00 orange
    {255, 190,  0},   //  8: 0xFDE0
    {255, 255,  0},   //  9: 0xFFE0 yellow
    {222, 255,  0},   // 10: 0xDFE0 yellow-green
    {156, 255,  0},   // 11: 0x9FE0
    {90,  255,  0},   // 12: 0x5FE0
    {0,   255,  0},   // 13: 0x07E0 green
    {0,   255, 66},   // 14: 0x07E8 green-cyan
    {0,   255, 132},  // 15: 0x07F0
    {0,   255, 197},  // 16: 0x07F8
    {0,   255, 255},  // 17: 0x07FF cyan
    {0,   174, 255},  // 18: 0x05FF
    {0,   125, 255},  // 19: 0x03FF
    {0,   61,  255},  // 20: 0x01FF
    {0,   0,   255},  // 21: 0x001F blue
    {0,   0,   197},  // 22: 0x0018
    {0,   0,   148},  // 23: 0x0012
    {0,   0,   107},  // 24: 0x000D
    {0,   0,   66},   // 25: 0x0008 dark blue
    {0,   0,   49},   // 26: 0x0006
    {0,   0,   33},   // 27: 0x0004
    {0,   0,   16},   // 28: 0x0002
    {0,   0,   0}     // 29: 0x0000 black
};

void heatmap_plotting(int dis, int i)
{
    if (dis < 0) {
        dis = 0;
    }
    int index = dis / 10;

    if (index > 29) {
        index = 29;
    }

    uint8_t r=0,g=0,b=0;

    r = heatmap_colors[index][0];
    g = heatmap_colors[index][1];
    b = heatmap_colors[index][2];

    lv_color_t color = lv_color_make(r, g, b);
    map[i] = color;
}
