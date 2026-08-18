/*
 * tof.c
 *
 *  Created on: 18 Aug 2026
 *      Author: samfishy
 */

#include "main.h"
#include "vl53l5cx_api.h"

uint8_t                 status, loop, isAlive, isReady, i,img_data;
VL53L5CX_Configuration  Dev;            /* Sensor configuration */
VL53L5CX_ResultsData    Results;        /* Results data from VL53L5CX */

int TOF_init(int mode)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,0);

    Dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;

    VL53L5CX_Reset_Sensor(&Dev.platform);

    status = vl53l5cx_is_alive(&Dev, &isAlive);
    status = vl53l5cx_init(&Dev);
    status = vl53l5cx_set_ranging_mode(&Dev, VL53L5CX_RANGING_MODE_CONTINUOUS);

    // mode 0 = 8x8 (64 zones)
    // mode 1 = 4x4 (16 zones)
    if(mode == 0)
    {
        status = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_8X8);
        status = vl53l5cx_set_ranging_frequency_hz(&Dev, 15);
    }
    else
    {
        status = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_4X4);
        status = vl53l5cx_set_ranging_frequency_hz(&Dev, 60);
    }

    int status = 1;
    return status;
}
