/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "help_functions.h"
#include "ADBMS6948.h"
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
SPI_HandleTypeDef hspi2;
uint16_t init_PEC = 16;
uint16_t converted_PEC_15[16];
uint16_t converted_PEC_10[16];
uint16_t in[16];
uint16_t converted_Cmd[16];
uint8_t data_read[64];
uint8_t config_read_B[6];
uint8_t config_read_A[6];
SPI_HandleTypeDef hspi1;
uint8_t spi_frame[16];
uint8_t readVcell[6];
uint8_t write_config_A[6] = {0x83, 0x00, 0x04, 0xFF, 0x3F, 0x08};
uint8_t write_config_B[6] = {0x00, 0xF8, 0x7F, 0x8F, 0x00, 0x00};
uint8_t write_config_B1[6] = {0x00, 0xF8, 0x7F, 0x0F, 0x02, 0x00};
uint8_t write_config_D[6] = {0x00, 0x00, 0x00, 0x00, 0x18, 0x00};
uint8_t write_data_A[6] = {0x06, 0x00, 0x04, 0xFF, 0x3F, 0x08};
uint8_t write_data_B[6] = {0x00, 0xf8, 0x7F, 0x02, 0x00, 0x00};
uint8_t test_PWM_write[6] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t dummy_spi_frame[12] = {0x00, 0x88, 0x3D, 0x60, 0x81, 0x00, 0x04, 0xFF, 0x3F, 0x08, 0x00, 0xDC};
uint8_t dummy_standby[1] = {0xFF};
uint8_t dummy_wakeup[120];
uint8_t binary_array[48];
uint8_t expanded_array[54];
uint8_t lpm_reg[6] = {0x8A, 0x01, 0x00, 0x00, 0x00, 0x00};

float cells_voltage[16];
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
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
  /* USER CODE BEGIN 2 */
  for(int i= 0; i < 120; i++ ) {

	  dummy_wakeup[i] = 0xFF;
  }

  wakeup_dummy();
  send_dummy_byte();
  HAL_Delay(100);

  BMS_command_SPI(0x0027); //SRST
  HAL_Delay(200);

  wakeup_dummy();
  send_dummy_byte();
  HAL_Delay(100);

  BMS_write_SPI(WRCFGB, spi_frame, write_config_B1, 6); //DCTO = 15, DCC = 2
  HAL_Delay(10);
  BMS_read_SPI(RDCFGB, spi_frame, config_read_B); //Read Configuration Register Group B
  HAL_Delay(1);
  BMS_write_SPI(WRCFGA, spi_frame, write_config_A, 6); //REFON up
  HAL_Delay(10);
  BMS_command_SPI(0x03E0); //Start Cell Voltage ADC Conversion and Poll Status
  HAL_Delay(200);


//
//  wakeup_dummy();
//    send_dummy_byte();
//
//  BMS_write_SPI(WRCFGD, spi_frame, write_config_D, 6);
//  HAL_Delay(1);
//  BMS_read_SPI(RDCFGD, spi_frame, data_read);
//
//  HAL_Delay(1);
//  BMS_write_SPI(WRCFGB, spi_frame, write_data_B, 6);
//
//  HAL_Delay(1);
//  BMS_write_SPI(WRPWMA, spi_frame, test_PWM_write, 6);
//
//  HAL_Delay(1);
//
//  BMS_read_SPI(RDPWMA, spi_frame, data_read);
//
//  HAL_Delay(200);







//  HAL_Delay(500);
//  wakeup_dummy();
//  send_dummy_byte();
//   HAL_Delay(100);
//
//  BMS_write_SPI(WRCFGB, spi_frame, write_data_B, 6);
//  HAL_Delay(1);
//  BMS_read_SPI(RDCFGB, spi_frame, data_read);
//    BMS_write_SPI(0x0001, spi_frame, write_config_A, 6);
//    	    HAL_Delay(1);
//    BMS_read_SPI(0x0026, spi_frame, config_read_A);
//    	    HAL_Delay(1);
//    BMS_write_SPI(0x0024, spi_frame, write_config_B, 6);
//    		HAL_Delay(1);
//    BMS_read_SPI(0x0026, spi_frame, config_read_B);
//    		HAL_Delay(1);
//    BMS_command_SPI(0x01F8); //S-ADC conversion
//    		HAL_Delay(200);

//  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */
	  //BMS_read_SPI(RDCFGA, spi_frame, data_read);
//	  BMS_command_SPI(0x03F0);
//	  HAL_Delay(20);
//	  wakeup_dummy();
	  send_dummy_byte(); //Sending for exiting sleep mode
	  HAL_Delay(1);

	  BMS_read_all_SPI(0x000C, spi_frame, data_read); //Cell Voltage reading
	  monitor_cells();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
