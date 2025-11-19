/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MovePlayer (void);
void GetADCValue (void);
void GetADC2Value (void);
void DrawPlayer (int x, int y, uint16_t color);
void DrawFlag(int x, int y, uint16_t color1, uint16_t color2);
void Displayplayer1Score (void);
void StartScreenSetup (void);
void StartAnimation(void);
void EndScreen(void);
bool CheckIfFlagTouched(int x, int y);
bool CheckIfGameEnd (void);


#define HEIGHT 240
#define WIDTH 320
#define MAX_SCORE 4

int flag_x[8];
int flag_y[8];

int currentFlagX;
int currentFlagY;

extern int player1X;
extern int player1Y;
extern int player2X;
extern int player2Y;
extern int player1Direction;
extern int player2Direction;
extern int player1State;
extern int player2State;
extern int x_start;
extern int y_start;
extern int x_start2;
extern int y_start2;

extern  bool isFlagReady;
extern bool isGameStart;

bool checkFlag = true;
bool isScreenWrited = false;

int temp1 = 0;
int temp2 = 0;
int player1Speed;
int player2Speed = 10;
uint8_t player1Score = 0;
int player2Score = 0;
int flagCounter = 1;

float raw1;
float raw2;

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
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  StartScreenSetup();

  flag_x[0] = 250;
  flag_y[0] = 200;

  flag_x[1] = 130;
  flag_y[1] = 120;

  flag_x[2] = 20;
  flag_y[2] = 35;

  flag_x[3] = 200;
  flag_y[3] = 55;

  flag_x[4] = 63;
  flag_y[4] = 193;

  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim4);

  /*for (int var = 0; var < 8; ++var) {
	DrawFlag(flag_x[var], flag_y[var], LCD_COLOR_RED, LCD_COLOR_WHITE);
}*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (!isGameStart){
		  StartAnimation();
	  }
	  else if (isGameStart){
		  MovePlayer();
	  }

	  if (CheckIfGameEnd()){
		  EndScreen();
	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_1LINE;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 100-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void MovePlayer (void){

	GetADC2Value();
	GetADCValue();
	DrawPlayer(player1X, player1Y, LCD_COLOR_ORANGE);
	DrawPlayer(player2X, player2Y, LCD_COLOR_BLUE);


	if (checkFlag){
		switch (flagCounter){
		case 1:
			DrawFlag(flag_x[0], flag_y[0], LCD_COLOR_WHITE, LCD_COLOR_RED);
			currentFlagX = flag_x[0];
			currentFlagY = flag_y[0];
			break;
		case 2:
			DrawFlag(flag_x[1], flag_y[1], LCD_COLOR_WHITE, LCD_COLOR_RED);
			currentFlagX = flag_x[1];
			currentFlagY = flag_y[1];
			break;
		case 3:
			DrawFlag(flag_x[2], flag_y[2], LCD_COLOR_WHITE, LCD_COLOR_RED);
			currentFlagX = flag_x[2];
			currentFlagY = flag_y[2];
			break;
		case 4:
			DrawFlag(flag_x[3], flag_y[3], LCD_COLOR_WHITE, LCD_COLOR_RED);
			currentFlagX = flag_x[3];
			currentFlagY = flag_y[3];
			break;

		}
	}



	if (CheckIfFlagTouched(currentFlagX, currentFlagY) == true){
			Displayplayer1Score();
	}
}

void DrawPlayer(int x, int y, uint16_t color){
	//Draw Square

	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(x-7   , y-4, 21, 17);
	BSP_LCD_SetTextColor(color);
	BSP_LCD_DrawRect(x, y, 5, 3);
	BSP_LCD_DrawRect(x-5, y+3, 15, 5);
	BSP_LCD_DrawRect(x-3, y+8, 3, 2);
	BSP_LCD_DrawRect(x+5, y+8, 3, 2);

}

void DrawFlag(int x, int y, uint16_t color1, uint16_t color2){
	BSP_LCD_SetTextColor(color1);
	BSP_LCD_DrawLine(x, y, x+3, y);
	BSP_LCD_DrawLine(x, y-1, x+3, y-1);
	BSP_LCD_DrawLine(x+1, y-2, x+1, y-9);
	BSP_LCD_DrawLine(x+2, y-2, x+2, y-9);

	BSP_LCD_SetTextColor(color2);
	BSP_LCD_DrawLine(x+3, y-4, x+3, y-10);
	BSP_LCD_DrawLine(x+4, y-4, x+4, y-10);
	BSP_LCD_DrawLine(x+5, y-5, x+5, y-9);
	BSP_LCD_DrawLine(x+6, y-6, x+6, y-8);
	DrawPixel(x+7, y-7, color2);

	checkFlag = false;
}

void GetADCValue (void){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw1 = HAL_ADC_GetValue(&hadc1);

	if (raw1 > 0 && raw1 <= 500){
		player1Speed = 5;
		player1Direction = 0;
	}
	else if (raw1 > 500 && raw1 <= 1000){
		player1Speed = 7;
		player1Direction = 0;
	}
	else if (raw1 > 1000 && raw1 <= 1500){
		player1Speed = 9;
		player1Direction = 0;
	}
	else if (raw1 > 1500 && raw1 <= 2000){
		player1Speed = 10;
		player1Direction = 0;
	}
	else if (raw1 <= 3000 && raw1 > 2500){
		player1Speed = 9;
		player1Direction = 1;
	}
	else if (raw1 <= 3500 && raw1 > 3000){
			player1Speed = 7;
			player1Direction = 1;
	}
	else if (raw1 <= 4038 && raw1 > 3500){
				player1Speed = 5;
				player1Direction = 1;
	}
	else{
		player1Direction = 2;
	}
}

void GetADC2Value (void){
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	raw2 = HAL_ADC_GetValue(&hadc2);

	if (raw2 - temp2 <= 100 && temp2-raw2 <= 100)
		return;
	temp2 = raw2;

	if (raw2 > 0 && raw2 <= 2000){
		player2Speed = 10;
		player2Direction = 0;
	}
	else if (raw2 <= 4038 && raw2 > 2500){
		player2Speed = 10;
		player2Direction = 1;
	}
	else{
		player2Direction = 2;
	}
}

bool CheckIfFlagTouched(int x, int y){
	if (player1X >= x && player1X <= x + 6 && player1Y <= y && player1Y >= y - 10){
		DrawFlag(x, y, LCD_COLOR_BLACK, LCD_COLOR_BLACK);
		player1Score++;
		checkFlag = true;
		flagCounter++;
		return true;

	}
	if (player1X + 10 >= x && player1X + 10 <= x + 6 && player1Y +3 <= y && player1Y + 3 >= y - 10){
		DrawFlag(x, y, LCD_COLOR_BLACK, LCD_COLOR_BLACK);
		player1Score++;
		checkFlag = true;
		flagCounter++;
		return true;

	}
	if (player2X >= x && player2X <= x + 6 && player2Y <= y && player2Y >= y - 10){
		DrawFlag(x, y, LCD_COLOR_BLACK, LCD_COLOR_BLACK);
		player2Score++;
		checkFlag = true;
		flagCounter++;
		return true;
	}
	if (player2X + 10 >= x && player2X + 10 <= x + 6 && player2Y + 3 <= y && player2Y + 3 >= y - 10){
			DrawFlag(x, y, LCD_COLOR_BLACK, LCD_COLOR_BLACK);
			player2Score++;
			checkFlag = true;
			flagCounter++;
			return true;
		}
	return false;
}

void Displayplayer1Score (void){
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

	switch(player1Score){
	case 1:
		BSP_LCD_DisplayStringAt(0, 230, (uint8_t*) "Player1: 1", LEFT_MODE);
		break;
	case 2:
		BSP_LCD_DisplayStringAt(0, 230, (uint8_t*) "Player1: 2", LEFT_MODE);
		break;
	case 3:
		BSP_LCD_DisplayStringAt(0, 230, (uint8_t*) "Player1: 3", LEFT_MODE);
		break;
	case 4:
		BSP_LCD_DisplayStringAt(0, 230, (uint8_t*) "Player1: 4", LEFT_MODE);
		break;

	}

	switch(player2Score){
	case 1:
		BSP_LCD_DisplayStringAt(0, 230, (uint8_t*) "Player2: 1", RIGHT_MODE);
		break;
	case 2:
		BSP_LCD_DisplayStringAt(0, 230, (uint8_t*) "Player2: 2", RIGHT_MODE);
		break;
	case 3:
		BSP_LCD_DisplayStringAt(0, 230, (uint8_t*) "Player2: 3", RIGHT_MODE);
		break;
	case 4:
		BSP_LCD_DisplayStringAt(0, 230, (uint8_t*) "Player2: 4", RIGHT_MODE);
		break;

	}

}

void StartScreenSetup(){
	BSP_LCD_Init();
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(0, 50, (uint8_t *) "BASLAMAK ICIN 'KEY' TUSUNA BASINIZ.", CENTER_MODE);
}

void StartAnimation(void){
	//CAR1
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DrawRect(x_start, y_start, 14, 8);
	BSP_LCD_DrawRect(x_start-15, y_start+8, 45, 13);
	BSP_LCD_DrawCircle(x_start-2, y_start+21, 5);
	BSP_LCD_DrawCircle(x_start+17, y_start+21, 5);
	BSP_LCD_DrawRect(0, 142, 320, 5);
	BSP_LCD_FillRect(x_start+1, y_start+1, 13, 6);

	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(x_start+1, y_start+1, 13, 6);
	BSP_LCD_FillRect(x_start-14, y_start+9, 43, 11);
	BSP_LCD_FillCircle(x_start-2, y_start+21, 4);
	BSP_LCD_FillCircle(x_start+17, y_start+21, 4);
	BSP_LCD_DrawLine(x_start-15, y_start+8, x_start-15, y_start+21);
	BSP_LCD_DrawLine(x_start, y_start, x_start, y_start+8);
	x_start++;
}

bool CheckIfGameEnd (void){
	if (flagCounter == MAX_SCORE + 1){return true;}
	return false;
}

void EndScreen(void){
	if (!isScreenWrited){
		BSP_LCD_Clear(LCD_COLOR_BLACK);
		BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
		BSP_LCD_SetFont(&Font12);

		if (player1Score > player2Score){
			BSP_LCD_DisplayStringAt(100, 200, (uint8_t *) "OYUNCU 1  KAZANDI...", LEFT_MODE);
		}
		else if (player2Score > player1Score){
			BSP_LCD_DisplayStringAt(100, 200, (uint8_t *) "OYUNCU 2  KAZANDI...", LEFT_MODE);
		}
		else{
			BSP_LCD_DisplayStringAt(100, 200, (uint8_t *) "BERABERE...", LEFT_MODE);
		}
		isScreenWrited = true;
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

#ifdef  USE_FULL_ASSERT
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
