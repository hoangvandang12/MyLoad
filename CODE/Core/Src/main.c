/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include "MCP4822.h"
#include "INA260.h"
#include "Delay_Timer4_Tick.h"
#include "ENCODER.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define on GPIO_PIN_SET
#define off GPIO_PIN_RESET
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//volatile char uart1_indx[1];
//float temp=0;
//volatile float current=0.0, voltage=0.0, power=0.0;
//volatile uint16_t ID=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f){
	
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,10);
	return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Frame_Template(void);
void Menu_Main(uint8_t* Mn_Main);
void Function_Vset(uint8_t* Mn_Main);
void Function_Aset(uint8_t* Mn_Main);
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	uint8_t Mn_Main=1;
//	float current=0.0, voltage=0.0, power=0.0;
//	uint16_t ID=0;
	ina260_t INA260_A;
//	tick_timer_t Tick, Tick1, Tick2;
//	char Temp_Buffer_text[11];
	
	
	Tick_Timer_Init();
	Encorder_C11_Init(1);
	
//	char Temp_Buffer_text[40];
	
//	HAL_UART_Receive_IT(&huart1, (uint8_t *) uart1_indx, 1);
//	HAL_ADC_Start_IT(&hadc1);
	
	HAL_GPIO_WritePin(LCD_LED_GPIO_Port,LCD_LED_Pin,on);
	HAL_Delay(100);

	ILI9341_Init();
////	
	ILI9341_Fill_Screen(BLACK);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	
//	ILI9341_Set_Address(0,100,320,125);	
	//ILI9341_Draw_Hollow_Rectangle_Coord(0,0,316,236,YELLOW);
	//ILI9341_Draw_Hollow_Rectangle_Coord(0,0,316,236,YELLOW);
	
	//LCD_Draw_Text(0,0,"HELLO123",8,BLACK,YELLOW,font_16_26);
	
//	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);	
//	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
//	
////	for(uint32_t j = 0; j < 32; j++)
////	{
//		HAL_SPI_Transmit_DMA(&hspi1, led_buff, 16640);	
////	}

//	ILI9341_Draw_Rectangle(0,0,10,10,RED);

//	LCD_Draw_Char('A',0,0,YELLOW);
	
	
//	ILI9341_Draw_Text("HELLO WORLD !", 10, 40, RED, 1, YELLOW);
	INA260_Config(&INA260_A, AVR16, V_CONV_1_1_MS, C_CONV_1_1_MS, CURRENT_VOLTAGE_CONTINUTE, false);

	MCP4822_DAC_Write(DAC_B, GAIN_X1, SHUTDOWN_MODE, 2000);
	HAL_Delay(100);
	MCP4822_DAC_Write(DAC_A, GAIN_X1, ACTIVE_MODE, 0);
	HAL_Delay(100);
//	

	INA260_Mask_Config(&INA260_A,ALERT_OFF, ALERT_CONVERSION_READY_ON, ACTIVE_LOW, TRANSPARENT);
	
	Frame_Template();
//	LCD_Draw_Text("CV",1,285,YELLOW);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,HAL_GPIO_ReadPin(POWER_GOOD_GPIO_Port,POWER_GOOD_Pin));
		Menu_Main(&Mn_Main);
		Function_Vset(&Mn_Main);
		Function_Aset(&Mn_Main);
		// TEST BLINK LED & BUZZER //
//		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
//		HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
//		HAL_GPIO_TogglePin(BUZZER_GPIO_Port,BUZZER_Pin);
//		HAL_Delay(300);
		// END TEST BLINK LED & BUZZER //
		
		// TEST BUTTON //
//			if(!HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin))
//				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,on);
//			else
//				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,off);
//			if(!HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin))
//				HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,on);
//			else
//				HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,off);
		// END TEST BUTTON //
		
		// TEST UART //
				//printf("he\n");
//				sprintf(Temp_Buffer_text, "nhiet do: %5.2f", temp);
//				ILI9341_Draw_Text(Temp_Buffer_text, 10, 20, RED, 2, WHITE);
//				HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
//				HAL_Delay(100);
				
//				MCP4822_DAC_Write(DAC_A, GAIN_X2, ACTIVE_MODE, value_dac);
//				value_dac++;
//				if(value_dac==4096)
//					value_dac=0;
//			if(HAL_UART_Receive(&huart1,(uint8_t*)uart1_indx,1,100)== HAL_OK){
//					printf("\n%c\n",uart1_indx[0]-32);
//			}
		
		// END TEST UART //	

		// TEST ADC //
		
		
		// TEST INA260 //
		
	//	Encoder_C11((int16_t*)&INA260_DAC_Value,0,4095);

//
//		//HAL_SPI_Transmit_DMA(&hspi1, led_buff, 15000);	
//		if(Tick_Timer_Is_Over_Ms(Tick1,100)){
//			
//			if(!HAL_GPIO_ReadPin(INA260_ALERT_GPIO_Port,INA260_ALERT_Pin)){
//				
////				INA260_Read(INA260_ADDRESS,MASK_EN_REGISTER_ADD,&ID);
//				current=INA260_Current_Read();
//				voltage=INA260_Voltage_Read();
////				ID=INA260_ID();
//				power=voltage*current;
//				HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
//				
//				//INA260_DAC_Value=PID_VR2(current*10, 2.0, 4000, 0, false, 6.5, 20.5, 0.015, 0.1);	
//				INA260_DAC_Value=PID_VR2(voltage, 4.5, 4000, 0, true, 6.5, 20.5, 0.015, 0.1);

//			MCP4822_DAC_Write(DAC_A, GAIN_X2, ACTIVE_MODE, INA260_DAC_Value);	
//			}			
//		}
			
//		if(Tick_Timer_Is_Over_Ms(Tick,300)){
//			
//			INA260_DAC_Value=PID(voltage,4.0,2500,0);	
//			MCP4822_DAC_Write(DAC_A, GAIN_X2, ACTIVE_MODE, INA260_DAC_Value);
//		}


//		if(Tick_Timer_Is_Over_Ms(Tick,500)){
////			
//			
////////			printf("dong dien: %5.4f\r\n", current);
////////			printf("dien ap: %5.4f\r\n", voltage);
////////			printf("cong suat: %5.4f\r\n", power);
////////			printf("NHIET DO: %5.4f\r\n", temp);
////////			printf("ID: %5d\r\n\n\n", ID);
//			sprintf(Temp_Buffer_text, "%06.3f", current);
//			LCD_Draw_Text(35, 85, Temp_Buffer_text, strlen(Temp_Buffer_text), GREEN, BLACK, font_11_18);
//			sprintf(Temp_Buffer_text, "%06.3f", voltage);
//			LCD_Draw_Text(35, 45, Temp_Buffer_text, strlen(Temp_Buffer_text), YELLOW, BLACK, font_11_18);
//			sprintf(Temp_Buffer_text, "%06.3f", power);
//			LCD_Draw_Text(35, 125, Temp_Buffer_text, strlen(Temp_Buffer_text), GREENYELLOW, BLACK, font_11_18);
//////			sprintf(Temp_Buffer_text, "R: ");
//////			LCD_Draw_Text(Temp_Buffer_text, 150, 10, ORANGE);
//				
//////			LCD_Draw_Text(Temp_Buffer_text,10,10,YELLOW);
//		}
		
		// END TEST INA260 //	
		
		//////////////
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  htim4.Init.Prescaler = 31;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  huart1.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_LED_Pin|LCD_DC_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BT1_Pin BT2_Pin */
  GPIO_InitStruct.Pin = BT1_Pin|BT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_LED_Pin LCD_DC_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_LED_Pin|LCD_DC_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DAC_CS_Pin */
  GPIO_InitStruct.Pin = DAC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DAC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : POWER_GOOD_Pin ENCODER_PB_Pin INA260_ALERT_Pin */
  GPIO_InitStruct.Pin = POWER_GOOD_Pin|ENCODER_PB_Pin|INA260_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER_PA_Pin ENCODER_BT_Pin */
  GPIO_InitStruct.Pin = ENCODER_PA_Pin|ENCODER_BT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	
//	if(huart->Instance == USART1){
//		
//		printf("hello\n");
//	}
//}

float lnx(float x,float epsilon){

	int i = 1;
	float s = 0, gg = 1;
	x=x-1;
	
	do{
		gg = pow ( x , i ) / i;
		s = s + pow (- 1 , i - 1) * gg;
		i++;
	}while(gg>=epsilon);

	return s;
}

void Frame_Template(void){
	
	ILI9341_Draw_Horizontal_Line(0,0,320,YELLOW);
	ILI9341_Draw_Horizontal_Line(0,239,320,YELLOW);
	ILI9341_Draw_Horizontal_Line(0,30,320,YELLOW);
	ILI9341_Draw_Horizontal_Line(0,160,320,YELLOW);
	
	ILI9341_Draw_Vertical_Line(0,0,240,YELLOW);
	ILI9341_Draw_Vertical_Line(319,0,240,YELLOW);
	ILI9341_Draw_Vertical_Line(105,0,240,YELLOW);
	ILI9341_Draw_Vertical_Line(213,0,240,YELLOW);
	
	ILI9341_Draw_Text("CV",70,3,YELLOW,1,BLACK);
	ILI9341_Draw_Text("CA",178,3,YELLOW,1,BLACK);
	ILI9341_Draw_Text("CP",284,3,YELLOW,1,BLACK);
	
	ILI9341_Draw_Text("V:",3,40,YELLOW,1,BLACK);
	ILI9341_Draw_Text("A:",3,80,YELLOW,1,BLACK);
	ILI9341_Draw_Text("P:",3,120,YELLOW,1,BLACK);
	
	ILI9341_Draw_Text("V:",108,40,YELLOW,1,BLACK);
	ILI9341_Draw_Text("A:",108,80,YELLOW,1,BLACK);
	ILI9341_Draw_Text("P:",108,120,YELLOW,1,BLACK);
	
	ILI9341_Draw_Text("V:",217,40,YELLOW,1,BLACK);
	ILI9341_Draw_Text("A:",217,80,YELLOW,1,BLACK);
	ILI9341_Draw_Text("P:",217,120,YELLOW,1,BLACK);
	
	ILI9341_Draw_Text("V_Set:",3,170,YELLOW,1,BLACK);
	ILI9341_Draw_Text("A_Set:",108,170,YELLOW,1,BLACK);
	ILI9341_Draw_Text("P_Set:",217,170,YELLOW,1,BLACK);
	
	ILI9341_Draw_Text("OFF",3,3,BLACK,1,YELLOW);
	ILI9341_Draw_Text("OFF",108,3,YELLOW,1,BLACK);
	ILI9341_Draw_Text("OFF",217,3,YELLOW,1,BLACK);
}

void Menu_Main(uint8_t* Mn_Main){
	
	if(*Mn_Main>0)
		Encoder_C11_int8((int8_t*)Mn_Main,1,3);
	
	switch(*Mn_Main){
			
		case 1: 
			ILI9341_Draw_Text("OFF",3,3,BLACK,1,YELLOW);
			ILI9341_Draw_Text("OFF",108,3,YELLOW,1,BLACK);
			ILI9341_Draw_Text("OFF",217,3,YELLOW,1,BLACK);
			*Mn_Main=11;
			break;
		case 2: 
			ILI9341_Draw_Text("OFF",3,3,YELLOW,1,BLACK);
			ILI9341_Draw_Text("OFF",108,3,BLACK,1,YELLOW);
			ILI9341_Draw_Text("OFF",217,3,YELLOW,1,BLACK);
			*Mn_Main=22;
			break;
		case 3: 
			ILI9341_Draw_Text("OFF",3,3,YELLOW,1,BLACK);
			ILI9341_Draw_Text("OFF",108,3,YELLOW,1,BLACK);
			ILI9341_Draw_Text("OFF",217,3,BLACK,1,YELLOW);
			*Mn_Main=33;
			break;
	}
}

void Function_Vset(uint8_t* Mn_Main){
	
	static uint8_t V_Function=0;
	static char Temp_Buffer_text[8]={0};
	static int8_t aa=0,bb=0,cc=0,dd=0;
	static bool V_Func_Select=false, Mode_On=false;
	static float V_Set=0.0;
	static float current=0.0, voltage=0.0, power=0.0;
	static uint16_t INA260_DAC_Value=0;
	static tick_timer_t Tick, Tick1;
	
	if(HAL_GPIO_ReadPin(ENCODER_BT_GPIO_Port,ENCODER_BT_Pin)==0 && *Mn_Main==11){
		
		*Mn_Main=0;	// khoa menu chinh
		ILI9341_Draw_Text("V_Set:",3,170,BLACK,1,YELLOW);
		V_Func_Select=true;
		while(HAL_GPIO_ReadPin(ENCODER_BT_GPIO_Port,ENCODER_BT_Pin)==0);
	}
	
	if(V_Func_Select){
	
		if(HAL_GPIO_ReadPin(ENCODER_BT_GPIO_Port,ENCODER_BT_Pin)==0 && Mode_On==false){
					
			*Mn_Main=11;
			V_Func_Select=false;
			ILI9341_Draw_Text("V_Set:",3,170,YELLOW,1,BLACK);
			while(HAL_GPIO_ReadPin(ENCODER_BT_GPIO_Port,ENCODER_BT_Pin)==0);
		}
		
		if(!HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)){
			
			while(!HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin));	
			switch(V_Function){
			
				case 0:
					ILI9341_Draw_Horizontal_Line(12,227,80,BLACK);
					ILI9341_Draw_Horizontal_Line(12,227,12,YELLOW);
					V_Function=1;
					break;
				case 1:
					ILI9341_Draw_Horizontal_Line(12,227,90,BLACK);
					ILI9341_Draw_Horizontal_Line(28,227,12,YELLOW);
					V_Function=2;
					break;
				case 2:
					ILI9341_Draw_Horizontal_Line(12,227,90,BLACK);
					ILI9341_Draw_Horizontal_Line(60,227,12,YELLOW);
					V_Function=3;
					break;
				case 3:
					ILI9341_Draw_Horizontal_Line(12,227,90,BLACK);
					ILI9341_Draw_Horizontal_Line(76,227,12,YELLOW);
					V_Function=0;
					break;				
			}		
		}
		
		switch(V_Function){
			
				case 0:
					Encoder_C11_int8(&dd,0,9);
					break;				
				case 1:
					Encoder_C11_int8(&aa,0,9);
					break;
				case 2:
					Encoder_C11_int8(&bb,0,9);
					break;
				case 3:
					Encoder_C11_int8(&cc,0,9);
					break;
		}
		
//		s=aa*1000+bb*100+cc*10+dd;
		V_Set=(aa*10) + (bb) + ((float)cc/10) + ((float)dd/100);
		
		sprintf(Temp_Buffer_text, "%05.2f", V_Set);
		LCD_Draw_Text(10, 200, Temp_Buffer_text, strlen(Temp_Buffer_text), YELLOW, BLACK, font_16_26);
		
		if(!HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin)){
		
			Mode_On=!Mode_On;
			if(Mode_On)
				ILI9341_Draw_Text("ON ",3,3,BLACK,1,YELLOW);
			else
				ILI9341_Draw_Text("OFF",3,3,BLACK,1,YELLOW);
			while(!HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin));
		}
				
		if(Tick_Timer_Is_Over_Ms(Tick1,50)){
			
			if(!HAL_GPIO_ReadPin(INA260_ALERT_GPIO_Port,INA260_ALERT_Pin)){
				
				current=INA260_Current_Read();
				voltage=INA260_Voltage_Read();
				power=voltage*current;
				HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
				
				if(Mode_On){
					
					INA260_DAC_Value=PID_VR2(voltage, V_Set, 4000, 0, true, 6.5, 20.5, 0.015, 0.05,false);
					MCP4822_DAC_Write(DAC_A, GAIN_X2, ACTIVE_MODE, INA260_DAC_Value);
				}
				else
					MCP4822_DAC_Write(DAC_A, GAIN_X2, ACTIVE_MODE, 0);
//				INA260_DAC_Value=PID_VR2(current*10, 2.0, 4000, 0, false, 6.5, 20.5, 0.015, 0.1);	
//				INA260_DAC_Value=PID_VR2(voltage, 4.5, 4000, 0, true, 6.5, 20.5, 0.015, 0.1);
//				MCP4822_DAC_Write(DAC_A, GAIN_X2, ACTIVE_MODE, INA260_DAC_Value);	
			}			
		}
		
		if(Mode_On){
		
			if(voltage<0.1 || current>10.0 || INA260_DAC_Value>2500){
				
				Mode_On=false;
				INA260_DAC_Value=PID_VR2(voltage, V_Set, 4000, 0, true, 6.5, 20.5, 0.015, 0.05,true);
				ILI9341_Draw_Text("OFF",3,3,BLACK,1,YELLOW);
				for(uint8_t i=0;i<5;i++){
					HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_SET);
					HAL_Delay(200);
					HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_RESET);
					HAL_Delay(200);
				}	
			}
		}
		
		if(Tick_Timer_Is_Over_Ms(Tick,500)){

			sprintf(Temp_Buffer_text, "%06.3f", current);
			LCD_Draw_Text(35, 85, Temp_Buffer_text, strlen(Temp_Buffer_text), GREEN, BLACK, font_11_18);
			sprintf(Temp_Buffer_text, "%06.3f", voltage);
			LCD_Draw_Text(35, 45, Temp_Buffer_text, strlen(Temp_Buffer_text), YELLOW, BLACK, font_11_18);
			sprintf(Temp_Buffer_text, "%06.3f", power);
			LCD_Draw_Text(35, 125, Temp_Buffer_text, strlen(Temp_Buffer_text), GREENYELLOW, BLACK, font_11_18);
		}
			
	}
}

void Function_Aset(uint8_t* Mn_Main){
	
	static uint8_t A_Function=0;
	static char Temp_Buffer_text[8]={0};
	static int8_t aa=0,bb=0,cc=0,dd=0;
	static bool A_Func_Select=false, Mode_On=false;
	static float A_Set=0.0;
	static float current=0.0, voltage=0.0, power=0.0;
	static uint16_t INA260_DAC_Value=0;
	static tick_timer_t Tick, Tick1;
	
	if(HAL_GPIO_ReadPin(ENCODER_BT_GPIO_Port,ENCODER_BT_Pin)==0 && *Mn_Main==22){
		
		*Mn_Main=0;	// khoa menu chinh
		ILI9341_Draw_Text("A_Set:",108,170,BLACK,1,YELLOW);
		A_Func_Select=true;
		while(HAL_GPIO_ReadPin(ENCODER_BT_GPIO_Port,ENCODER_BT_Pin)==0);
	}
	
	if(A_Func_Select){
	
		if(HAL_GPIO_ReadPin(ENCODER_BT_GPIO_Port,ENCODER_BT_Pin)==0 && Mode_On==false){
					
			*Mn_Main=22;
			A_Func_Select=false;
			ILI9341_Draw_Text("A_Set:",108,170,YELLOW,1,BLACK);
			while(HAL_GPIO_ReadPin(ENCODER_BT_GPIO_Port,ENCODER_BT_Pin)==0);
		}
		
		if(!HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)){
			
			while(!HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin));	
			switch(A_Function){
			
				case 0:
					ILI9341_Draw_Horizontal_Line(117,227,80,BLACK);
					ILI9341_Draw_Horizontal_Line(117,227,12,YELLOW);
					A_Function=1;
					break;
				case 1:
					ILI9341_Draw_Horizontal_Line(117,227,80,BLACK);
					ILI9341_Draw_Horizontal_Line(133,227,12,YELLOW);
					A_Function=2;
					break;
				case 2:
					ILI9341_Draw_Horizontal_Line(117,227,80,BLACK);
					ILI9341_Draw_Horizontal_Line(165,227,12,YELLOW);
					A_Function=3;
					break;
				case 3:
					ILI9341_Draw_Horizontal_Line(117,227,80,BLACK);
					ILI9341_Draw_Horizontal_Line(181,227,12,YELLOW);
					A_Function=0;
					break;				
			}		
		}
		
		switch(A_Function){
			
				case 0:
					Encoder_C11_int8(&dd,0,9);
					break;				
				case 1:
					Encoder_C11_int8(&aa,0,9);
					break;
				case 2:
					Encoder_C11_int8(&bb,0,9);
					break;
				case 3:
					Encoder_C11_int8(&cc,0,9);
					break;
		}
		
//		s=aa*1000+bb*100+cc*10+dd;
		A_Set=(aa*10) + (bb) + ((float)cc/10) + ((float)dd/100);
		
		sprintf(Temp_Buffer_text, "%05.2f", A_Set);
		LCD_Draw_Text(115, 200, Temp_Buffer_text, strlen(Temp_Buffer_text), YELLOW, BLACK, font_16_26);
		
		if(!HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin)){
		
			Mode_On=!Mode_On;
			if(Mode_On)
				ILI9341_Draw_Text("ON ",108,3,BLACK,1,YELLOW);
			else
				ILI9341_Draw_Text("OFF",108,3,BLACK,1,YELLOW);
			while(!HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin));
		}
				
		if(Tick_Timer_Is_Over_Ms(Tick1,50)){
			
			if(!HAL_GPIO_ReadPin(INA260_ALERT_GPIO_Port,INA260_ALERT_Pin)){
				
				current=INA260_Current_Read();
				voltage=INA260_Voltage_Read();
				power=voltage*current;
				HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
				
				if(Mode_On){
					
//					if(A_Set<0.5)
//						INA260_DAC_Value=PID_VR2(current*10, A_Set*10, 4000, 0, false, 6.5, 20.5, 0.015, 0.1);
//					else
					INA260_DAC_Value=PID_VR2(current, A_Set, 4000, 0, false, 70.5, 60.5, 0.015, 0.05, false);
					MCP4822_DAC_Write(DAC_A, GAIN_X2, ACTIVE_MODE, INA260_DAC_Value);
				}
				else
					MCP4822_DAC_Write(DAC_A, GAIN_X2, ACTIVE_MODE, 0);
//				INA260_DAC_Value=PID_VR2(current*10, 2.0, 4000, 0, false, 6.5, 20.5, 0.015, 0.1);	
//				INA260_DAC_Value=PID_VR2(voltage, 4.5, 4000, 0, true, 6.5, 20.5, 0.015, 0.1);
//				MCP4822_DAC_Write(DAC_A, GAIN_X2, ACTIVE_MODE, INA260_DAC_Value);	
			}			
		}
		
		if(Mode_On){
		
			if(voltage<0.1 || current>10.0 || INA260_DAC_Value>2500){
				
				Mode_On=false;
				PID_VR2(current, 0, 4000, 0, false, 30.5, 20.5, 0.015, 0.05, true);
				ILI9341_Draw_Text("OFF",108,3,BLACK,1,YELLOW);
				for(uint8_t i=0;i<5;i++){
					HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_SET);
					HAL_Delay(200);
					HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_RESET);
					HAL_Delay(200);
				}	
			}
		}
		
		if(Tick_Timer_Is_Over_Ms(Tick,500)){

			sprintf(Temp_Buffer_text, "%06.3f", current);
			LCD_Draw_Text(140, 85, Temp_Buffer_text, strlen(Temp_Buffer_text), GREEN, BLACK, font_11_18);
			sprintf(Temp_Buffer_text, "%06.3f", voltage);
			LCD_Draw_Text(140, 45, Temp_Buffer_text, strlen(Temp_Buffer_text), YELLOW, BLACK, font_11_18);
			sprintf(Temp_Buffer_text, "%06.3f", power);
			LCD_Draw_Text(140, 125, Temp_Buffer_text, strlen(Temp_Buffer_text), GREENYELLOW, BLACK, font_11_18);
		}
			
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	
//	static float V_adc=0, R_ntc=0;
//	static uint16_t adc_value=0;
//	if(hadc -> Instance == ADC1){
//		
//		adc_value=HAL_ADC_GetValue(&hadc1);
//		V_adc=adc_value*0.80566;								// 0.80566 = 3300/4096 //
//		R_ntc = (10*(3300-V_adc))/V_adc; 				// (R_ref*(Vcc-V_adc))/V_adc  // Vcc=3.3V,R_ref=10k//
//    temp=1/(lnx(R_ntc/10,0.5)/3950.0 + 1/298.15)-273.15;
//	}

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
