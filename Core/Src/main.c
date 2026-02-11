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

#include "arm_math.h"

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
uint16_t rData[20480];

uint8_t bytes[2052];

uint8_t rec_high_bytes[1029];
uint8_t rec_low_bytes[1024];

uint8_t high_bytes[20480];
uint8_t low_bytes[20480];

uint8_t 				status, loop, isAlive, isReady, i,img_data;
VL53L5CX_Configuration 	Dev;			/* Sensor configuration */
VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */

uint16_t map[1024];

int exti_flag = 100;
int x1= 0 , high_ready = 0, low_ready = 0;
int j = 0,i1 = 0, x10 = 0;
int r_flag = 0, interpolation = 1,mode = 0,ini = 0,img = 0;
int interpolation_menu = 0 ,mode_menu = 0, img_str = 0,menu_mode = 0;
int16_t rData_int[1024];

#define SCALE 4
#define IN_LEN 64
#define OUT_LEN (IN_LEN * SCALE)

q15_t in_q15[64];

extern uint16_t image_data_iniwind[];
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

   // 2x -> 8x8 -> 16x16
int IN_W = 8;
int IN_H = 8;
int OUT_W  = 8;
int OUT_H  = 8;   // 16

// ---- global CMSIS bilinear instance & buffers ----
arm_bilinear_interp_instance_q15 S;
q15_t grid_q15[64];     // flattened 8x8 input (row-major)
q15_t out_q15[1024];    // flattened 16x16 output (row-major)

// Call once to set up S (after filling grid_q15)
void bilinear_init_q15(void)
{
    S.numRows = IN_H;   // 8
    S.numCols = IN_W;   // 8
    S.pData   = grid_q15;
}

// Convert int16 raw array (row-major) into q15 grid
void load_input_int16_to_q15(const int16_t raw[IN_W * IN_H])
{
    for (uint32_t i = 0; i < IN_W * IN_H; i++)
    {
        grid_q15[i] = (q15_t)raw[i];
    }
}

// Perform 8x8 -> 16x16 bilinear using arm_bilinear_interp_q15
// Uses X,Y in 12.20 fixed point mapping: src = dst_coord / SCALE
void bilinear_8x8_to_16x16_q15(void)
{
    for (int y = 0; y < OUT_H; y++)
    {
        // (y + 0.5) * (IN_H/OUT_H) - 0.5  in Q12.20
        q31_t src_y = ((((q31_t)y << 20) + (1 << 19)) * IN_H) / OUT_H - (1 << 19);

        for (int x = 0; x < OUT_W; x++)
        {
            q31_t src_x = ((((q31_t)x << 20) + (1 << 19)) * IN_W) / OUT_W - (1 << 19);

            out_q15[y * OUT_W + x] = arm_bilinear_interp_q15(&S, src_x, src_y);
        }
    }
}

const uint16_t heatmap_30[30] =
{
    0x3000, //  0 dark red
    0x5000, //  1
    0x7800, //  2
    0xA000, //  3
    0xC800, //  4
    0xF800, //  5 red
    0xFA00, //  6 red-orange
    0xFC00, //  7 orange
    0xFDE0, //  8
    0xFFE0, //  9 yellow
    0xDFE0, // 10 yellow-green
    0x9FE0, // 11
    0x5FE0, // 12
    0x07E0, // 13 green
    0x07E8, // 14 green-cyan
    0x07F0, // 15
    0x07F8, // 16
    0x07FF, // 17 cyan
    0x05FF, // 18
    0x03FF, // 19
    0x01FF, // 20
    0x001F, // 21 blue
    0x0018, // 22
    0x0012, // 23
    0x000D, // 24
    0x0008, // 25 dark blue
    0x0006, // 26
    0x0004, // 27
    0x0002, // 28
    0x0000  // 29 black
};

void heatmap_plotting(int dis,int i)
{
		if(dis < 10)
		{
			map[i] = heatmap_30[0];
		}
		if(dis < 20 && dis >= 10)
		{
			map[i] = heatmap_30[1];
		}
		if(dis < 30 && dis >= 20)
		{
			map[i] = heatmap_30[2];
		}
		if(dis < 40 && dis >= 30)
		{
			map[i] = heatmap_30[3];
		}
		if(dis < 50 && dis >= 40)
		{
			map[i] = heatmap_30[4];
		}
		if(dis < 60 && dis >= 50)
		{
			map[i] = heatmap_30[5];
		}
		if(dis < 70 && dis >= 60)
		{
			map[i] = heatmap_30[6];
		}
		if(dis < 80 && dis >= 70)
		{
							map[i] = heatmap_30[7];
						}
						if(dis < 90 && dis >= 80)
						{
							map[i] = heatmap_30[8];
						}
						if(dis < 100 && dis >= 90)
						{
							map[i] = heatmap_30[9];
						}
						if(dis < 110 && dis >= 100)
						{
							map[i] = heatmap_30[10];
						}
						if(dis < 120 && dis >= 110)
						{
							map[i] = heatmap_30[11];
						}
						if(dis < 130 && dis >= 120)
						{
							map[i] = heatmap_30[12];
						}
						if(dis < 140 && dis >= 130)
						{
							map[i] = heatmap_30[13];
						}
						if(dis < 150 && dis >= 140)
						{
							map[i] = heatmap_30[14];
						}
						if(dis < 160 && dis >= 150)
						{
							map[i] = heatmap_30[15];
						}
						if(dis < 170 && dis >= 160)
						{
							map[i] = heatmap_30[16];
						}
						if(dis < 180 && dis >= 170)
						{
							map[i] = heatmap_30[17];
						}
						if(dis < 190 && dis >= 180)
						{
							map[i] = heatmap_30[18];
						}
						if(dis < 200 && dis >= 190)
						{
							map[i] = heatmap_30[19];
						}
						if(dis < 210 && dis >= 200)
						{
							map[i] = heatmap_30[20];
						}
						if(dis < 220 && dis >= 210)
						{
							map[i] = heatmap_30[21];
						}
						if(dis < 230 && dis >= 220)
						{
							map[i] = heatmap_30[22];
						}
						if(dis < 240 && dis >= 230)
						{
							map[i] = heatmap_30[23];
						}
						if(dis < 250 && dis >= 240)
						{
							map[i] = heatmap_30[24];
						}
						if(dis < 260 && dis >= 250)
						{
							map[i] = heatmap_30[25];
						}
						if(dis < 270 && dis >= 260)
						{
							map[i] = heatmap_30[26];
						}
						if(dis < 280 && dis >= 270)
						{
							map[i] = heatmap_30[27];
						}
						if(dis < 290 && dis >= 280)
						{
							map[i] = heatmap_30[28];
						}
						if(dis >= 290)
						{
							map[i] = heatmap_30[29];
						}
}

void res_calc2()
{
	if(interpolation == 1)
	{
		if(mode == 0)
		{
			OUT_H = 8;
			OUT_W = 8;

			  ST7735_WriteString(132, 7,  "88M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			  ST7735_WriteString(132, 27, "1IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
		}
		else
		{
			OUT_H = 4;
			OUT_W = 4;

			  ST7735_WriteString(132, 7, "44M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			  ST7735_WriteString(132, 27, "1IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
		}
	}
	else if(interpolation == 2)
	{
		if(mode == 0)
		{
			OUT_H = 16;
			OUT_W = 16;

			  ST7735_WriteString(132, 7,  "88M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			  ST7735_WriteString(132, 27, "2IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
		}
		else
		{
			OUT_H = 8;
			OUT_W = 8;

			  ST7735_WriteString(132, 7,  "44M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			  ST7735_WriteString(132, 27, "2IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
		}
	}
	else if(interpolation == 4)
	{
		if(mode == 0)
		{
			OUT_H = 32;
			OUT_W = 32;

			  ST7735_WriteString(132, 7, "88M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			  ST7735_WriteString(132, 27, "4IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
		}
		else
		{
			OUT_H = 16;
			OUT_W = 16;

			  ST7735_WriteString(132, 7,  "44M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			  ST7735_WriteString(132, 27, "2IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
		}
	}
}

void res_calc()
{
	if(r_flag == 0)
	{
		r_flag = 1;
		interpolation = 1;

		if(mode == 0)
		{
			OUT_H = 8;
			OUT_W = 8;

			 ST7735_WriteString(132, 7,  "88M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			 ST7735_WriteString(132, 27, "1IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
		}
		else
		{
			OUT_H = 4;
			OUT_W = 4;

			 ST7735_WriteString(132, 7,  "44M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			 ST7735_WriteString(132, 27, "1IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
		}
	}
	else if(r_flag == 1)
	{
		r_flag = 2;
		interpolation = 2;

		if(mode == 0)
		{
			OUT_H = 16;
			OUT_W = 16;

			 ST7735_WriteString(132, 7,  "88M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			 ST7735_WriteString(132, 27, "2IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
		}
		else
		{
			OUT_H = 8;
			OUT_W = 8;

			 ST7735_WriteString(132, 7,  "44M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			 ST7735_WriteString(132, 27, "2IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
		}
	}
	else if(r_flag == 2)
	{
		r_flag = 0;
		interpolation = 4;

		if(mode == 0)
		{
			OUT_H = 32;
			OUT_W = 32;

			 ST7735_WriteString(132, 7, "88M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			 ST7735_WriteString(132, 27, "4IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
		}
		else
		{
			OUT_H = 16;
			OUT_W = 16;
			 ST7735_WriteString(132, 7, "44M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			 ST7735_WriteString(132, 27, "4IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
		}
	}
}

void img_print(int interpolation, int mode,int16_t data[])
{
	int mode1 = 0;

	if(mode == 0)
	{
		mode1 = 8;
	}
	else
	{
		mode1 = 4;
	}

	int total 	 = (mode1*interpolation);
	int total_sq = total*total;
	int rec_use  = 128/total;

	if(HAL_DMA_PollForTransfer(&hdma_spi4_tx, HAL_DMA_FULL_TRANSFER  , 10))
	{
		for(int i = 0; i < total_sq;i++)
					{
						int dis = (int)(data[i])/10;
						heatmap_plotting(dis,i);
					}

					for (int col = 0; col < total; col++)
					{
						for (int row = 0; row < total; row++)
						{
							int idx = row + col*total;
							int x   = col * rec_use;
							int y   = row * rec_use;
							ST7735_FillRectangle(x, y, rec_use , rec_use, map[idx]);
						}
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
		 rData_int[i] = ((uint16_t)rec_high_bytes[i] << 8) |
	                 (uint16_t)rec_low_bytes[i];
	}
}


void img_store(int img_num,int size)
{
	int img_page = 100+img_num*16;

	for (int i = 0; i < size; i++)
		{
			bytes[i] = (((uint16_t)out_q15[i]) >> 8) & 0xFF; // Extract high byte
		    bytes[1028+i]  =  ((uint16_t)out_q15[i]) & 0xFF;   // Extract low byte
		}

	bytes[1026] = (uint8_t)mode;
	bytes[1027] = (uint8_t)interpolation;

	W25Q_Write_Page(img_page, 0, 2052, bytes);

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
  W25_rst();
  ID = W25_devID();


  ST7735_Init();

  W25_fRead(300, 0,20480, high_bytes);
  HAL_Delay(10);
  W25_fRead(392, 0,20480, low_bytes);

  for (int i = 0; i < 20480; i++)
  {
  	rData[i]= ((uint16_t)high_bytes[i] << 8) | low_bytes[i];
  }

  ST7735_DrawImage(0, 0, 160, 128, rData);
  ST7735_WriteString(132, 7, "88M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
  ST7735_WriteString(132, 27, "1IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
  ST7735_WriteString(132, 113, "LIV", Font_7x10,ST7735_BLACK, ST7735_WHITE );

  tof_init();
  HAL_Delay(100);
/*
 for (int i = 0; i < 20480; i++)
	{
		high_bytes[i] = (image_data_frame_8_delay0[i] >> 8) & 0xFF; // Extract high byte
	    low_bytes[i]  =  image_data_frame_8_delay0[i] & 0xFF;        // Extract low byte
	}

	W25Q_Write_Page (1280, 0, 20480, high_bytes);
	W25Q_Write_Page (1360, 0, 20480, low_bytes);


 image(j);
*/
	vl53l5cx_start_ranging(&Dev);

	W25_fRead(0, 0,1, &img_data);
	img = (int)img_data;
	ini = 1;
	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


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
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
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
				  ST7735_DrawImage(0, 0, 160, 128, rData);
				  ST7735_WriteString(132, 7, "**M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
				  ST7735_WriteString(132, 27, "*IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
				  ST7735_WriteString(132, 113, "REC", Font_7x10,ST7735_BLACK, ST7735_WHITE );
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
		//if(high_ready == 1 && low_ready == 1)

			/*for (int i = 0; i < 20480; i++)
			{
				rData[i]= ((uint16_t)high_bytes[i] << 8) | low_bytes[i];
			}*/

			//interp_1d_4x_q15(Results.distance_mm, out_q15);

		if(interpolation != 1)
		{
			load_input_int16_to_q15(Results.distance_mm);
			bilinear_init_q15();
			bilinear_8x8_to_16x16_q15();

			img_print(interpolation,mode,out_q15);
		}
		else
		{
			img_print(interpolation,mode,Results.distance_mm);
		}
	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_TIM_Base_Stop_IT(&htim3);

	if(GPIO_Pin == GPIO_PIN_6)
	{
		vl53l5cx_get_ranging_data(&Dev, &Results);
	}

	else if(GPIO_Pin == GPIO_PIN_0)
	{
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		exti_flag = 0;

		if(menu_mode == 0)
		{
			vl53l5cx_stop_ranging(&Dev);
			HAL_Delay(10);
			res_calc();
			HAL_Delay(10);
			vl53l5cx_start_ranging(&Dev);
		}
		else
		{
			if(img_str < img )
			{
				img_str ++;
			}
			else
			{
				img_str = 0;
			}
			image(img_str);
			img_print(interpolation_menu,mode_menu,rData_int);
		}

		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	}

	else if(GPIO_Pin == GPIO_PIN_3)
	{
		HAL_NVIC_DisableIRQ(EXTI3_IRQn);
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

		exti_flag = 3;

		if(menu_mode == 0)
		{
			vl53l5cx_stop_ranging(&Dev);
			HAL_Delay(10);

			if(mode == 0)
			{
				mode = 1;
				vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_4X4);
				IN_W = 4;
				IN_H = 4;
			}
			else
			{
				mode = 0;
				vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_8X8);
				IN_W = 8;
				IN_H = 8;
			}

			res_calc2();
			vl53l5cx_start_ranging(&Dev);
		}

		else
		{

			if(img_str > 0 )
			{
				img_str--;
			}
			else
			{
				img_str = img;
			}

			image(img_str);
			img_print(interpolation_menu,mode_menu,rData_int);

		}


		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	}

	else if(GPIO_Pin == GPIO_PIN_4)
	{
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		HAL_NVIC_DisableIRQ(EXTI4_IRQn);
		vl53l5cx_stop_ranging(&Dev);
		exti_flag = 4;

		uint8_t img_wq = (uint8_t)img;
		W25Q_Write_Page(0, 0, 1, &img_wq);
		HAL_Delay(10);
		img_store(img,1024);
		img_str = img;

		if(img < 10)
		{
			img++;
		}
		else
		{
			img = 0;
		}

		vl53l5cx_start_ranging(&Dev);
	}

	else if(GPIO_Pin == GPIO_PIN_2)
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

			  W25_fRead(500, 0,20480, high_bytes);
			  HAL_Delay(10);
			  W25_fRead(600, 0,20480, low_bytes);

			  for (int i = 0; i < 20480; i++)
			  {
			  	rData[i]= ((uint16_t)high_bytes[i] << 8) | low_bytes[i];
			  }

		}
		else
		{
			menu_mode = 0;
			ini = 1;

			  W25_fRead(300, 0,20480, high_bytes);
			  HAL_Delay(10);
			  W25_fRead(392, 0,20480, low_bytes);

			  for (int i = 0; i < 20480; i++)
			  {
			  	rData[i]= ((uint16_t)high_bytes[i] << 8) | low_bytes[i];
			  }

			  ST7735_DrawImage(0, 0, 160, 128, rData);
			  HAL_Delay(10);
			  ST7735_WriteString(132, 7, "88M", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			  HAL_Delay(10);
			  ST7735_WriteString(132, 27, "4IN", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			  HAL_Delay(10);
			  ST7735_WriteString(132, 113, "LIV", Font_7x10,ST7735_BLACK, ST7735_WHITE );
			  HAL_Delay(10);

			vl53l5cx_start_ranging(&Dev);
			HAL_NVIC_EnableIRQ(EXTI4_IRQn);
			HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

		}
	}

	HAL_TIM_Base_Start_IT(&htim4);

}

/*void HAL_Delay(uint32_t Delay)
{
	__HAL_TIM_SET_COUNTER(&htim4,0);
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  while((HAL_GetTick() - tickstart) < wait)
  {
  }
}
uint32_t HAL_GetTick(void)
{
  return __HAL_TIM_GET_COUNTER(&htim4);
}*/

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
