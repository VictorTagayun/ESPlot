/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

// ESPlot
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"

#define USART			0
#define FT4222			1
#define COMM_INTERFACE	USART

#include "comm_prot.h"
#include "math.h"

//Signals used for plotting
uint8_t  cnt_8 = 0;
uint16_t cnt_16 = 0;
float harmonic_signal = 0.0f;

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
 UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;
DMA_HandleTypeDef hdma_lpuart1_tx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

// Structure of the USB Protocol
volatile comm_prot USB_COMM;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM2_Init(void);
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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //Initialize the USB protocol
  comm_prot_init_struct(&USB_COMM);

  //Set rx signals
//  comm_prot_set_rx_data_info(&USB_COMM, 0, "LED1");

  //Set tx signals
  comm_prot_set_tx_data_info(&USB_COMM, 0, SCALING_FACTOR_APPLIED, TYPE_UINT8,  "VTest UINT8", 		PLOT_NR_1, 1.0, 4, BLACK);
  comm_prot_set_tx_data_info(&USB_COMM, 1, SCALING_FACTOR_APPLIED, TYPE_INT8,   "VTest INT8", 		PLOT_NR_1, 2.0, 4, BLUE);
  comm_prot_set_tx_data_info(&USB_COMM, 2, SCALING_FACTOR_APPLIED, TYPE_UINT16, "VTest UINT16", 		PLOT_NR_2, 3.14, 4, RED);
  comm_prot_set_tx_data_info(&USB_COMM, 3, SCALING_FACTOR_APPLIED, TYPE_INT16,  "VTest INT16", 		PLOT_NR_2, 4.0, 4, RANDOM);
  comm_prot_set_tx_data_info(&USB_COMM, 4, SCALING_FACTOR_APPLIED, TYPE_UINT32, "VTest UINT32", 		PLOT_NR_3, 2.0, 4, BLUE);
  comm_prot_set_tx_data_info(&USB_COMM, 5, SCALING_FACTOR_APPLIED, TYPE_INT32,  "VTest INT32", 		PLOT_NR_3, 3.0, 4, RED);
  comm_prot_set_tx_data_info(&USB_COMM, 6, SCALING_FACTOR_APPLIED, TYPE_FLOAT,  "VTest FLOAT", 		PLOT_NR_4, 1/3.14, 4, RANDOM);
  comm_prot_set_tx_data_info(&USB_COMM, 7, SCALING_FACTOR_NOT_APPLIED, TYPE_FLOAT, "VHarmonic signal", PLOT_NR_5, 1.0, 4, BLACK);
  comm_prot_init_comm(&USB_COMM);

  //Initiate first transfer to set all the settings
//  HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
//                                                uint16_t Size)
//  HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) USB_COMM.tx_i_buff, (uint8_t*) USB_COMM.rx_d_buff, (uint16_t) USB_COMM.buff_dimension_info);

//  if (HAL_OK != HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*) USB_COMM.tx_i_buff, (uint16_t) USB_COMM.buff_dimension_info))
//  {
//	  Error_Handler();
//  };

  // start_data_transfer(uint32_t buffer_size, uint8_t* tx_data_buff, uint8_t* rx_data_buff);
  start_data_transfer((uint16_t) USB_COMM.buff_dimension_info, (uint8_t*) USB_COMM.tx_i_buff, (uint8_t*) USB_COMM.rx_i_buff);

  //Start Timer (1 kHz Timer)
  HAL_TIM_Base_Start_IT(&htim2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_Delay(2000);
//	  comm_prot_set_cmd(&USB_COMM, RECORD_CMD);
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//      HAL_Delay(2000);
//	  comm_prot_set_cmd(&USB_COMM, NO_CMD);
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 1000000;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 170-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/**
 * Function of the protocol used for transmitting data
 */
void start_data_transfer(uint32_t buffer_size, uint8_t* tx_data_buff, uint8_t* rx_data_buff)
{

#if (COMM_INTERFACE == FT4222)
	HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) tx_data_buff, (uint8_t*) rx_data_buff, (uint16_t) buffer_size);
	//Quick_HAL_SPI_Transfer(buffer_size, tx_data_buff, rx_data_buff);
#elif (COMM_INTERFACE == USART)
	HAL_UART_Receive_DMA(&hlpuart1, (uint8_t*) rx_data_buff, (uint16_t) buffer_size);
	HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*) tx_data_buff, (uint16_t) buffer_size);
#endif

}

/**
 * Function of the protocol used for getting the actual transmission status
 */
uint8_t  get_transfer_status()
{

#if (COMM_INTERFACE == FT4222)
	if (HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_READY)
		return  1;
	else
		return 0;
#elif (COMM_INTERFACE == USART)
	if (HAL_UART_GetState(&hlpuart1) != HAL_UART_STATE_BUSY_TX)
		return  1;
	else
		return 0;
#endif

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */

  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	cnt_8++; //increment counter variable until it overflows
	harmonic_signal = 3.0f * sinf(2.0f*3.14f*((float)cnt_16)/1000.0f) + 0.5f * sinf(2.0f*3.14f*5.0f*((float)cnt_16)/1000.0f)+ 0.05f * sinf(2.0f*3.14f*50.0f*((float)cnt_16)/1000.0f);
	cnt_16++;
	if (cnt_16 > 999)
		cnt_16 = 0;

	//Set tx signals
	comm_prot_write_tx_value(&USB_COMM, 0, (cnt_8));
	comm_prot_write_tx_value(&USB_COMM, 1, (cnt_8 - 127));
	comm_prot_write_tx_value(&USB_COMM, 2, (cnt_8*256));
	comm_prot_write_tx_value(&USB_COMM, 3, (cnt_8-127)*256);
	comm_prot_write_tx_value(&USB_COMM, 4, (cnt_8*1000));
	comm_prot_write_tx_value(&USB_COMM, 5, (cnt_8-127)*1000);
	comm_prot_write_tx_value(&USB_COMM, 6, 3.14f*(float)cnt_8);
	comm_prot_write_tx_value(&USB_COMM, 7, harmonic_signal);

	//Update Communication
	comm_prot_manager(&USB_COMM);

	//Read rx signals
//	uint8_t led = comm_prot_get_rx_value(&USB_COMM, 0);

	//Switch LED1 accordingly to the received value
//	if (led == 1)
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
//	else
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);

//	//Switch RGB LED accordingly to received terminal command
//	char red[16] = "LED RED";
//	char green[16] = "LED GREEN";
//	char blue[16] = "LED BLUE";
//	char off[16] = "LED OFF";
//	if (strcmp(USB_COMM.terminal_cmd, red) == 0)
//	{
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//	}
//	else if (strcmp(USB_COMM.terminal_cmd, green) == 0)
//	{
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//	}
//	else if (strcmp(USB_COMM.terminal_cmd, blue) == 0)
//	{
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
//	}
//	else if (strcmp(USB_COMM.terminal_cmd, off) == 0)
//	{
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//	}

	//Switch LED2 accordingly if the connection is established or not
//	if(USB_COMM.comm_state == ESTABLISHED)
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
//	else
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
//
//	if (get_transfer_status())
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
//	else
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);

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
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
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
