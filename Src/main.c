/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
I2C_HandleTypeDef hi2c1;
i2cLcd_HandleTypeDef h_lcd;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t i2c_addr = (0x20<<1);
	uint8_t strData[32] = "Simona <3\0";
	uint8_t i;
	uint8_t j;

	uint32_t freq;
	freq = HAL_RCC_GetSysClockFreq();

	freq = 0;


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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // crappy code
  //_hi2c = &hi2c1;
  //i2cLcd_Init(&hi2c1, i2c_addr);

  //i2cLcd_SendByte( (1<<5)|(0<<4)|(1<<3)|(1<<2), 0, 1);
  //i2cLcd_SendCmd(FUNC_SET | FONT_SEL | LINES | 0);
  //i2cLcd_SendByte( (1<<3)|(1<<2)|(1<<1)|(1<<0), 0, 0);
  //i2cLcd_SendByte( (1<<4), 0, 0);
  //i2cLcd_SendByte( (1<<4), 0, 0);
  //i2cLcd_SendByte( 1, 0, 0);
  HAL_Delay(100);
  i2cLcd_CreateHandle(&h_lcd, &hi2c1, i2c_addr);
  HAL_Delay(2);
  i2cLcd_Init(&h_lcd);
  HAL_Delay(2);
  i2cLcd_ClearDisplay(&h_lcd);

  for(i=0; i<8;i++)
	  i2cLcd_SendChar(&h_lcd, 'C');

  i2cLcd_SetPos(&h_lcd, 3);

  i2cLcd_SetPos(&h_lcd, 44);

  i2cLcd_SendChar(&h_lcd, 'C');
  i2cLcd_SendByte(&h_lcd, DISP_CTRL | DISP_CTRL_CURSOR_ON | DISP_CTRL_BLINK_ON | DISP_CTRL_DISPLAY_ON, I2CLCD_OPTS_NOINIT);
//  i2cLcd_SendByte(&h_lcd, FUNC_SET | FUNC_SET_DLEN_4B | FUNC_SET_FO_5X8 | FUNC_SET_LINES_2, I2CLCD_OPTS_NOINIT);
  i2cLcd_SendChar(&h_lcd, 'Z');
  i2cLcd_SendChar(&h_lcd, 'z');
  //i2cLcd_SendByte(&h_lcd, FUNC_SET | FUNC_SET_DLEN_4B | FUNC_SET_FO_5X8 | FUNC_SET_LINES_2, I2CLCD_OPTS_NOINIT);


  i=0;
  j=0;

  while (1){
	  i2cLcd_ClearDisplay(&h_lcd);
	  HAL_Delay(20);
	  sprintf(strData,"line1 text");
	  while(strData[i]) {
		  i2cLcd_SendChar(&h_lcd, strData[i]);
		  HAL_Delay(100);
		  i++;
	  }

	  i2cLcd_SetPos(&h_lcd, 0x40);
	  i=0;
	  sprintf(strData,"line2 text");
	  while(strData[i]) {
		  i2cLcd_SendChar(&h_lcd, strData[i]);
		  HAL_Delay(100);
		  i++;
	  }

	  i2cLcd_SendChar(&h_lcd, 0x7E);
	  HAL_Delay(2000);
	  //i2cLcd_RetHome();
	  //HAL_Delay(1000);
	  //i2cLcd_SendByte( 'z', 1, 0);
	  //HAL_Delay(1000);

	  //i2cLcd_ClearDisplay();
	  //HAL_Delay(1000);
	  i=0;
	  j++;
    /* USER CODE END WHILE */
  }
  /* USER CODE BEGIN 3 */
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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