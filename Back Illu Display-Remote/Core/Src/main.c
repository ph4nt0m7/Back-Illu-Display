/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "gc9a01a.h"
#include "math.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FRAME_SIZE (240 * 240)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
uint16_t pwm_value = 0;
static uint16_t frame_buf[FRAME_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void setPWM(int8_t value);
void TestDisplayLoop(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t RGB565(uint8_t R, uint8_t G, uint8_t B){
	R 	/= 8;
	G 	/= 4;
	B 	/= 8;

	uint8_t hi = ((G & 0x07) << 5) | ((B & 0x1F));
	uint8_t lo = ((G & 0x38) >> 3) | ((R & 0x1F) << 3);

	return lo | (hi << 8);
}



void setPWM(int8_t value) {
	  TIM16->CCR1 = value;
}

void TestFillColor(uint8_t *color) {
	// First pixel: transmit command
    DC_OFF();
    CS_OFF();
    HAL_SPI_Transmit(&hspi2, MEM_WR, 1, 1);
    CS_ON();

	// First pixel: transmit color
    DC_ON();
    CS_OFF();
    HAL_SPI_Transmit(&hspi2, color, 3, 1);
    CS_ON();

    for (uint16_t i = 0; i < (240*240 - 1); i++) {
    	// Other pixels: transmit command
        DC_OFF();
        CS_OFF();
        HAL_SPI_Transmit(&hspi2, MEM_WR_CONT, 1, 1);
        CS_ON();

    	// Other pixels: transmit color
        DC_ON();
        CS_OFF();
        HAL_SPI_Transmit(&hspi2, color, 3, 1);
        CS_ON();

    }

}

/*
void TestDisplayLoop(void){
	  uint8_t color[3];
	      // Triangle
	      color[0] = 0xFF;
	      color[1] = 0xFF;
	      for (int x = 0; x < 240; x++) {
	          for (int y = 0; y < 240; y++) {
	              if (x < y) {
	                  color[2] = 0xFF;
	              } else {
	                  color[2] = 0x00;
	              }
	              if (x == 0 && y == 0) {
//	            	  GPIOC->BSRR = GPIO_BSRR_BS8;
	                  GC9A01_write(color, sizeof(color));
//	                  GPIOC->BSRR = GPIO_BSRR_BR8;
	              } else {
//	            	  GPIOC->BSRR = GPIO_BSRR_BS8;
	                  GC9A01_write_continue(color, sizeof(color));
//	                  GPIOC->BSRR = GPIO_BSRR_BR8;

	              }
	          }
	      }
//	      setPWM(1);
	      HAL_Delay(1000);
	      // setPWM(0);
	      // Rainbow
	      float frequency = 0.026;
	      for (int x = 0; x < 240; x++) {
	          color[0] = sin(frequency*x + 0) * 127 + 128;
	          color[1] = sin(frequency*x + 2) * 127 + 128;
	          color[2] = sin(frequency*x + 4) * 127 + 128;
	          for (int y = 0; y < 240; y++) {
	              if (x == 0 && y == 0) {
	                  GC9A01_write(color, sizeof(color));
	              } else {
	                  GC9A01_write_continue(color, sizeof(color));
	              }
	          }
	      }
	      // setPWM(1);
	      HAL_Delay(1000);
	      // setPWM(0);
	      // Checkerboard
	      for (int x = 0; x < 240; x++) {
	          for (int y = 0; y < 240; y++) {
	              if ((x / 10) % 2 ==  (y / 10) % 2) {
	                  color[0] = 0xFF;
	                  color[1] = 0xFF;
	                  color[2] = 0xFF;
	              } else {
	                  color[0] = 0x00;
	                  color[1] = 0x00;
	                  color[2] = 0x00;
	              }
	              if (x == 0 && y == 0) {
	                  GC9A01_write(color, sizeof(color));
	              } else {
	                  GC9A01_write_continue(color, sizeof(color));
	              }
	          }
	      }
	      // setPWM(1);
	      HAL_Delay(1000);
	      // setPWM(0);
	      // Swiss flag
	      color[0] = 0xFF;
	      for (int x = 0; x < 240; x++) {
	          for (int y = 0; y < 240; y++) {
	              if ((x >= 1*48 && x < 4*48 && y >= 2*48 && y < 3*48) ||
	                  (x >= 2*48 && x < 3*48 && y >= 1*48 && y < 4*48)) {
	                  color[1] = 0xFF;
	                  color[2] = 0xFF;
	              } else {
	                  color[1] = 0x00;
	                  color[2] = 0x00;
	              }
	              if (x == 0 && y == 0) {
	                  GC9A01_write(color, sizeof(color));
	              } else {
	                  GC9A01_write_continue(color, sizeof(color));
	              }
	          }
	      }
	      // setPWM(1);
	      GC9A01_write_command(0x20);
}
*/

uint8_t tougle_test_pin = 0;

void TIM15_IRQHandler(void){
//	if(tougle_test_pin){
//		TEST_ON();
//	} else {
//		TEST_OFF();
//	}
//	tougle_test_pin = !tougle_test_pin;
//
//	TIM15->SR &= ~TIM_SR_UIF;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim6) {
//		if(tougle_test_pin){
//			TEST_ON();
//		} else {
//			TEST_OFF();
//		}
//		tougle_test_pin = !tougle_test_pin;





	}
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
  MX_SPI2_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */


//  HAL_TIM_Base_Start(&htim6);
//  HAL_TIM_Base_Start_IT(&htim6);


  gc9a01_t gc9a01;
  gc9a01.spi = &hspi2;
  gc9a01.frame = frame_buf;
  gc9a01.frame_size = FRAME_SIZE;

  gc9a01.res_port = RES_GPIO_Port;
  gc9a01.res_pin = RES_Pin;
  gc9a01.dc_port = DC_GPIO_Port;
  gc9a01.dc_pin = DC_Pin;
  gc9a01.cs_port = CS_GPIO_Port;
  gc9a01.cs_pin = CS_Pin;


  GC9A01_init(&gc9a01);

  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  setPWM(50);

//  uint8_t clr[3] = {0xFF, 0xFF, 0xFF};
//  TestFillColor(clr);
//
//  HAL_Delay(500);
//
//  uint8_t clr2[3] = {0xFF, 0xFF, 0xFF};
//  TestFillColor(clr2);


  int increment = 16;
  int fade_size = 0;
  uint16_t color[ (6+1) * (255 / increment)];




  for (int i = 0; i < 255; i += increment){
	  color[fade_size++] = RGB565(255, i, 0); 	// red -> yellow
  }

  for (int i = 255; i >= 0; i -= increment){
	  color[fade_size++] = RGB565(i, 255, 0); 	// yellow - > green
  }

  for (int i = 0; i < 255; i += increment){
	  color[fade_size++] = RGB565(0, 255, i); 	// green -> cyan
  }

  for (int i = 255; i >= 0; i -= increment){
	  color[fade_size++] = RGB565(0, i, 255); 	// cyan -> blue
  }

  for (int i = 0; i < 255; i += increment){
	  color[fade_size++] = RGB565(i, 0, 255); 	// blue -> purple
  }

  for (int i = 255; i >= 0; i -= increment){
	  color[fade_size++] = RGB565(255, 0, i); 	// purple -> red
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


//	  color = RGB565(255, 0, 0);
//	  for (int j = 0; j < 240 * 240; j++) {
//		  frame_buf[j] = color; // red
//	  }
//	  GC9A01_spi_frame(&gc9a01);
//	  HAL_Delay(1000);
//
//
//
//	  color = RGB565(0, 255, 0);
//	  for (int j = 0; j < 240 * 240; j++) {
//		  frame_buf[j] = color; // green
//	  }
//	  GC9A01_spi_frame(&gc9a01);
//	  HAL_Delay(1000);
//
//
//
//	  color = RGB565(0, 0, 255);
//	  for (int j = 0; j < 240 * 240; j++) {
//		  frame_buf[j] = color; // blue
//	  }
//	  GC9A01_spi_frame(&gc9a01);
//	  HAL_Delay(1000);




	  for(int i = 0; i < fade_size; i++){
		  for (int j = 0; j < 240 * 240; j++) {
			  frame_buf[j] = color[i];
		  }
		  GC9A01_spi_frame(&gc9a01);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 80;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 100;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DC_Pin|CS_Pin|TEST_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DC_Pin CS_Pin TEST_PIN_Pin */
  GPIO_InitStruct.Pin = DC_Pin|CS_Pin|TEST_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RES_Pin */
  GPIO_InitStruct.Pin = RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
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

