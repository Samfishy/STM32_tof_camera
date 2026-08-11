/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735.h"
#include "testimg.h"
#include "fonts.h"
#include "vl53l5cx_api.h"
#include "flash.h"
#include "string.h"

#include "string.h"
#include "stdio.h"
#include "st7735.h"

#include <stddef.h>
#include <stdint.h>

#include "ui.h"
#include "vars.h"

#include "arm_math.h"
#include "../../grapics/lvgl.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void tof_init(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint32_t ID;
uint8_t bytes[2052];
uint8_t rec_high_bytes[1029];
uint8_t rec_low_bytes[1024];

uint8_t                 status, loop, isAlive, isReady, i,img_data;
VL53L5CX_Configuration  Dev;            /* Sensor configuration */
VL53L5CX_ResultsData    Results;        /* Results data from VL53L5CX */

lv_color_t map[1024];

int exti_flag = 100;
int x1= 0 , high_ready = 0, low_ready = 0;
int j = 0,i1 = 0, x10 = 0;
int r_flag = 0, interpolation = 1,mode = 0,ini = 0,img = 0;
int interpolation_menu = 0 ,mode_menu = 0, img_str = 0,menu_mode = 0;
int16_t rData_int[1024];

static lv_disp_drv_t disp_drv;

#define SCALE 4
#define IN_LEN 64
#define OUT_LEN (IN_LEN * SCALE)

#define CANVAS_WIDTH  145
#define CANVAS_HEIGHT 105

// LVGL v8 Draw Buffer Structure
static lv_disp_draw_buf_t draw_buf;

// The actual pixel array (use lv_color_t for v8, not uint8_t)
static lv_color_t buf1[160 * 20];

q15_t in_q15[64];

// LVGL Flags & Buffers
volatile bool flag_update_heatmap = false;
volatile bool flag_update_labels = false;
volatile bool flag_use_rdata = false; // To indicate rendering from stored image

char view_type_str[10] = "LIVE View";
char page_num_str[10] = "Page 1/0";
char status_str[10] = "LIVE View";
char zone_mode[10] = "8x8 zones";
char zone_int[10] = "1x int";


// LVGL Canvas Buffer
static lv_color_t cbuf[CANVAS_HEIGHT*CANVAS_WIDTH];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int flag_tim2 = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int IN_W = 8;
int IN_H = 8;
int OUT_W  = 8;
int OUT_H  = 8;   // 16

arm_bilinear_interp_instance_q15 S;
q15_t grid_q15[64];
q15_t out_q15[1024];

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

/// Draw directly to the LVGL Canvas Buffer
void img_print_lvgl(int interp, int md, int16_t data[])
{
    // Ensure the canvas and EEZ objects exist before drawing
    if (!objects.heat_map) return;

    int mode1 = (md == 0) ? 8 : 4;
    int total = (mode1 * interp);
    int total_sq = total * total;
    int rec_use = CANVAS_WIDTH / total;

    for(int i = 0; i < total_sq; i++)
    {
        int dis = (int)(data[i])/10;
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
            // Your exact original array indexing math
            int idx = row + col * total;

            int x = ((total - 1) - row) * rec_use;
            int y = col * rec_use;

            // Assign the 16-bit RGB565 color to the LVGL color union
            rect_dsc.bg_color = map[idx];

            // Draw rectangle to canvas
            lv_canvas_draw_rect(objects.heat_map, x, y, rec_use, rec_use, &rect_dsc);
        }
    }
}

void image(int x)
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

void img_store(int img_num,int size)
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

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    uint16_t x = (uint16_t)area->x1;
    uint16_t y = (uint16_t)area->y1;
    uint16_t w = (uint16_t)(area->x2 - area->x1 + 1);
    uint16_t h = (uint16_t)(area->y2 - area->y1 + 1);

    // Cast the LVGL color struct to a standard 16-bit integer pointer
    uint16_t * px_map = (uint16_t *)color_p;

    // Send the raw data to the display
    ST7735_DrawImage_DMA(x, y, w, h, px_map);
    // Tell LVGL we are done

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_SPI4_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  ST7735_Init();
  ST7735_FillScreen(ST7735_BLACK);

  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, 160 * 20);
  lv_disp_drv_init(&disp_drv);

  disp_drv.hor_res = 160;
  disp_drv.ver_res = 128;
  disp_drv.flush_cb = my_flush_cb;
  disp_drv.draw_buf = &draw_buf;

     // Register the display driver
  lv_disp_drv_register(&disp_drv);
  // Initialize EEZ Studio UI
  ui_init();

  // Boot sequence - load the live view page with spinner first as per "image_04a174.png"
  loadScreen(SCREEN_ID_LIVE_VIEW_PAGE);
  lv_timer_handler(); // Force render of spinner page



  // Initialize standard hardware
   W25_rst();
   ID = W25_devID();

   // Initialize LVGL and Display (Replace with your actual display port init)
   // Init Sensor
   tof_init();
   HAL_Delay(100);

   // Explicitly link the C buffer to the EEZ-generated Canvas
   if(objects.heat_map != NULL) {
           lv_canvas_set_buffer(objects.heat_map, cbuf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);
      }
    // Set initial labels (Make sure "objects.view_type" aligns with your EEZ studio naming convention)
    if(objects.view_type) lv_label_set_text(objects.view_type, view_type_str);
    if(objects.page_num) lv_label_set_text(objects.page_num, page_num_str);

    vl53l5cx_start_ranging(&Dev);

    W25_fRead(0, 0,1, &img_data);
    img = (int)img_data;
    ini = 1;

    HAL_TIM_Base_Start_IT(&htim3);

    // Switch to heatmap page after initialization "image_04a17d.png"
    loadScreen(SCREEN_ID_HEAT_MAP_PAGE);
    lv_timer_handler();
   /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
              // 1. Process LVGL Tasks
              lv_timer_handler();
              ui_tick();
              // 2. Handle Display Updates securely in main loop
              if (flag_update_heatmap)
              {
                  flag_update_heatmap = false;
                  if(flag_use_rdata) {
                      img_print_lvgl(interpolation_menu, mode_menu, rData_int);
                  } else {
                      if(interpolation != 1) {
                          img_print_lvgl(interpolation, mode, out_q15);
                      } else {
                          img_print_lvgl(interpolation, mode, Results.distance_mm);
                      }
                  }
              }

              // 3. Handle Label Text updates securely in main loop

              // Provide a small delay to avoid starving FreeRTOS (if used) or spinning too fast
              HAL_Delay(5);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_1LINE;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 50-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 27000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_FLASH_Pin|GPIO_PIN_1|GPIO_PIN_2|rst_tof_Pin
                          |lp_tof_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Capture_Pin|GPIO_PIN_9|LED_Update_Pin|LED_Mode_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_FLASH_Pin */
  GPIO_InitStruct.Pin = CS_FLASH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CS_FLASH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 rst_tof_Pin lp_tof_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|rst_tof_Pin|lp_tof_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Capture_Pin LED_Update_Pin */
  GPIO_InitStruct.Pin = LED_Capture_Pin|LED_Update_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Mode_Pin */
  GPIO_InitStruct.Pin = LED_Mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED_Mode_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 2);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 2);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 2);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void tof_init()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,0);

    Dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;

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
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM4)
    {
        if(exti_flag == 0)
        {
            HAL_NVIC_EnableIRQ(EXTI0_IRQn);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        }
        else if(exti_flag == 3)
        {
            HAL_NVIC_EnableIRQ(EXTI3_IRQn);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        }
        else if(exti_flag == 2)
        {
            HAL_NVIC_EnableIRQ(EXTI2_IRQn);

            if(menu_mode == 1)
            {
                strcpy(view_type_str, "**M");
                strcpy(page_num_str, "*IN");
                strcpy(status_str, "REC");
                flag_update_labels = true;
            }
        }
        else if(exti_flag == 4)
        {
            HAL_NVIC_EnableIRQ(EXTI4_IRQn);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        }

        exti_flag = 100;
        HAL_TIM_Base_Stop_IT(&htim4);
        if(ini == 1)
        {
            HAL_TIM_Base_Start_IT(&htim3);
        }
    }
    else if(htim->Instance == TIM3)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        if(interpolation != 1)
        {
            load_input_int16_to_q15(Results.distance_mm);
            bilinear_init_q15();
            bilinear_8x8_to_16x16_q15();
        }

        // Notify the main loop to execute UI rendering
        flag_use_rdata = false;
        flag_update_heatmap = true;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    HAL_TIM_Base_Stop_IT(&htim3);

    if(GPIO_Pin == GPIO_PIN_6)
    {
        vl53l5cx_get_ranging_data(&Dev, &Results);
    }
    else if(GPIO_Pin == GPIO_PIN_0) // INTERPOLATION TOGGLE (1x, 2x, 3x)
    {
        HAL_NVIC_DisableIRQ(EXTI0_IRQn);
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
        exti_flag = 0;

        if(menu_mode == 0)
        {
            vl53l5cx_stop_ranging(&Dev);

            // Cycle Interpolation: 1 -> 2 -> 3 -> 1
            if (interpolation == 1) {
                interpolation = 2;
                strcpy(page_num_str, "2x int");
            } else if (interpolation == 2) {
                interpolation = 3;
                strcpy(page_num_str, "3x int");
            } else {
                interpolation = 1;
                strcpy(page_num_str, "1x int");
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

        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    }
    else if(GPIO_Pin == GPIO_PIN_3) // ZONE TOGGLE (8x8, 4x4)
    {
        HAL_NVIC_DisableIRQ(EXTI3_IRQn);
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
        exti_flag = 3;

        if(menu_mode == 0)
        {
            vl53l5cx_stop_ranging(&Dev);

            // Cycle Mode: 8x8 -> 4x4 -> 8x8
            if(mode == 0)
            {
                mode = 1;
                vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_4X4);
                IN_W = 4; IN_H = 4;
                strcpy(view_type_str, "4x4 zones");
            }
            else
            {
                mode = 0;
                vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_8X8);
                IN_W = 8; IN_H = 8;
                strcpy(view_type_str, "8x8 zones");
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

        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    }
    else if(GPIO_Pin == GPIO_PIN_4) // CAPTURE
    {
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
        HAL_NVIC_DisableIRQ(EXTI4_IRQn);
        vl53l5cx_stop_ranging(&Dev);
        exti_flag = 4;

        uint8_t img_wq = (uint8_t)img;
        W25Q_Write_Page(0, 0, 1, &img_wq);
        img_store(img,1024);
        img_str = img;

        if(img < 10) img++;
        else img = 0;

        vl53l5cx_start_ranging(&Dev);
    }
    else if(GPIO_Pin == GPIO_PIN_2) // MENU TOGGLE
    {
        HAL_NVIC_DisableIRQ(EXTI2_IRQn);
        exti_flag = 2;

        if(menu_mode == 0)
        {
            HAL_NVIC_DisableIRQ(EXTI4_IRQn);
            HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
            ini = 0;
            menu_mode = 1;
            vl53l5cx_stop_ranging(&Dev);
        }
        else
        {
            menu_mode = 0;
            ini = 1;

            // Reset labels to match current state when returning to Live Mode
            if(mode == 0) strcpy(view_type_str, "8x8 zones");
            else strcpy(view_type_str, "4x4 zones");

            if(interpolation == 1) strcpy(page_num_str, "1x int");
            else if(interpolation == 2) strcpy(page_num_str, "1x int");
            else strcpy(page_num_str, "1x int");

            strcpy(status_str, "LIVE view");
            flag_update_labels = true;

            vl53l5cx_start_ranging(&Dev);
            HAL_NVIC_EnableIRQ(EXTI4_IRQn);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        }
    }
    HAL_TIM_Base_Start_IT(&htim4);
}

const char * get_var_loading_screen_input(void)
{
    return "Initialising...";
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


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI4 || hspi->Instance == SPI1)
    {
        lv_disp_flush_ready(&disp_drv);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
