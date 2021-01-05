/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "oled.h"
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
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
u8 ch = 0;
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
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	// 开启定时器
	HAL_TIM_Base_Start_IT(&htim3);
	// 开启 USART1 中断模式
	HAL_UART_Receive_IT(&huart1, &ch, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// 圆心
	OLED_DrawPoint(64, 32);
	// 外圆
	OLED_DrawCircle(64, 32, 32);
	// 由 y 轴负半轴到 x 轴负半轴
	OLED_DrawLine(64, 59, 64, 64, 1);
	OLED_DrawLine(91, 32, 96, 32, 1);
	OLED_DrawLine(64, 5, 64,   0, 1);
	OLED_DrawLine(37, 32, 32, 32, 1);
	// 右下角
	OLED_DrawLine(78, 55, 80, 60, 1);
	OLED_DrawLine(87, 45, 92, 48, 1);
	// 右上角
	OLED_DrawLine(88, 19, 92, 16, 1);
	OLED_DrawLine(77, 9, 80,   5, 1);
	// 左上角
	OLED_DrawLine(51, 9, 48,   5, 1);
	OLED_DrawLine(42, 19, 36, 16, 1);
	// 左下角
	OLED_DrawLine(42, 45, 36, 48, 1);
	OLED_DrawLine(52, 55, 48, 60, 1);	
	// 显示默认日期
	OLED_DrawLine(32, 0, 32, 64);
	// 显示Date
	OLED_ShowString(0, 0, "Date:", 12);
	// 年
	OLED_ShowChar(0, 15, 'Y', 12);
	OLED_ShowNum(8, 15, 2020, 4, 12);
	// 月
	OLED_ShowChar(0, 30, 'M', 12);
	OLED_ShowNum(8, 30, 12, 2, 12);	
	// 日
	OLED_ShowChar(0, 45, 'D', 12);
	OLED_ShowNum(8, 45, 19, 2, 12);	
	// 后期可通过虚拟终端修改
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
double second = 0, minute = 45, hour = 4;
double pi = 3.1415926535897932384626;
int sx, sy, mx, my, hx, hy;
int st = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim3.Instance) { // 时钟盘 计时
			// 秒针 1s 走 6     度
			// 分针 1s 走 1/10  度
			// 时针 1s 走 1/120 度
		  // 秒针、分针、时针一起走
			double hudu1 = (2 * pi / 360) * 6 * second;
			double hudu2 = (2 * pi / 360) * 6 * (minute + second / 60.0);
			double hudu3 = (2 * pi / 360) * 30 * (hour + minute / 60.0 + second / 3600.0);
			int x1 = 64 + 27 * sin(hudu1);
			int y1 = 32 - 27 * cos(hudu1);
			int x2 = 64 + 21 * sin(hudu2);
			int y2 = 32 - 21 * cos(hudu2);
			int x3 = 64 + 13 * sin(hudu3);
			int y3 = 32 - 13 * cos(hudu3);
			if (second) {     // 清除上一条线
				OLED_DrawLine(64, 32, sx, sy, 0);
			}
			if (minute) {
				OLED_DrawLine(64, 32, mx, my, 0);
			}
			if (hour) {
				OLED_DrawLine(64, 32, hx, hy, 0);
			}
			// HAL_Delay(500); // 给一定清除上一条线的时间
			sx = x1, sy = y1;
			mx = x2, my = y2;
			hx = x3, hy = y3;
			// 标记刻度的目的是防止在清除上一条线的同时也将刻度清除
			// 刻度 12 点
			OLED_ShowNum(58, 5, 12, 2, 12);
			// 刻度 3 点
			OLED_ShowNum(85, 26, 3, 1, 12);
			// 刻度 6 点
			OLED_ShowNum(61, 47, 6, 1, 12);
			// 刻度 9 点
			OLED_ShowNum(37, 26, 9, 1, 12);
			OLED_DrawLine(64, 32, sx, sy, 1);
			OLED_DrawLine(64, 32, mx, my, 1);
			OLED_DrawLine(64, 32, hx, hy, 1);
			// OLED_ShowNum(8, 15, 4096, 4, 12);			
			OLED_Refresh_Gram();
			// HAL_Delay(1000);
			++second;
			if (second == 60) {
				++minute;
				if (minute == 60) {
					++hour;
					if (hour == 12) {
						hour = 0;
					}
					minute = 0;
				}
				second = 0;
			}
		}			
}

int month[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
int isy = 1, ism = 0, isd = 0;
int ley = 0, lem = 0, led = 0;
int yea = 0, mon = 0, day = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	// 开始记录月份
	if (ch == ' ' && !st) {
		isy = 0, ism = 1;
		st = 1;
		return ;
	}
	// 开始记录天数
	else if (ch == ' ' && st) {
		ism = 0, isd = 1;
		st = 0;
		return ;
	}
	// 判断其合法性，合法则输出
	else if (ch == '\r') {
		if (isLeapYear(yea)) month[2] = 29;
		else month[2] = 28;
		if (mon > 12 || mon < 1 || day > month[mon] || day < 1) {
			yea = mon = day = 0;
			ley = lem = led = 0;
			ism = isd = 0;
			isy = 1;
			return ;
		}
		OLED_ShowNum(8, 15, yea, ley, 12);
		OLED_ShowNum(8, 30, mon, lem, 12);
		OLED_ShowNum(8, 45, day, led, 12);
		OLED_Refresh_Gram();
		yea = mon = day = 0;
		ley = lem = led = 0;
		ism = isd = 0;
		isy = 1;
		return ;
	}
	if (isy) {
		yea = yea * 10 + ch - '0';
		++ley;
	}
	if (isy == 4) {
		OLED_ShowNum(8, 15, yea, ley, 12);
		OLED_Refresh_Gram();		
	}
	if (ism) {
		mon = mon * 10 + ch - '0';
		++lem;
	}
	if (isd) {
		day = day * 10 + ch - '0';
		++led;
	}
	HAL_UART_Receive_IT(&huart1, &ch, 1);
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  htim3.Init.Prescaler = 8000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin|RES_Pin|DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_Pin DC_Pin */
  GPIO_InitStruct.Pin = CS_Pin|DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RES_Pin */
  GPIO_InitStruct.Pin = RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RES_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
