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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "pid.h"
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
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
PID_TypeDef TPID;
double p = 8; //kp
double i = 1;  //ki
double d = 11; //kd
double PIDOut = 0;
double setposition = 0;
double actualDistancedouble = 0;

uint32_t IC_Val11 = 0;
uint32_t IC_Val21 = 0;
uint32_t Difference1 = 0;
uint8_t Is_First_Captured1 = 0;
uint32_t Distance1  = 0;

uint32_t IC_Val12 = 0;
uint32_t IC_Val22 = 0;
uint32_t Difference2 = 0;
uint8_t Is_First_Captured2 = 0;
uint32_t Distance2  = 0;

int32_t actualDistance = 0; //from center

uint8_t recvd_data=0x0; // receive buffer for UART
uint8_t data_buffer[100]; // data buffer for UART
uint8_t uartsendbuffer[100];
uint32_t countuart=0; // count how many bytes are received from UART

int32_t actualposition = 0; //actual position
int32_t destposition = 0; //destination position
uint8_t pulsestatus = 0; //pulse status

double actualpositiondouble = 0;
int32_t actualpositionforsend = 0;

int size_len;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

void moveto(int32_t degree){ //move motor to target position example:123.45
	if(degree>-4500){
		if(degree<4500){
			destposition = -(degree*25)/100;
		}
	}
}

void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim8, 0);
	while (__HAL_TIM_GET_COUNTER (&htim8) < time);
}

void SR05_Read1 (void){
	HAL_GPIO_WritePin(trig1_GPIO_Port, trig1_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);
	HAL_GPIO_WritePin(trig1_GPIO_Port, trig1_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_CC1);
}

void SR05_Read2 (void){
	HAL_GPIO_WritePin(trig2_GPIO_Port, trig2_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);
	HAL_GPIO_WritePin(trig2_GPIO_Port, trig2_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_CC2);
}

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
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_UART_Receive_IT(&huart2,&recvd_data,1); //receive data from data buffer interrupt mode

  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);


  PID(&TPID, &actualDistancedouble, &PIDOut, &setposition, p, i, d, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&TPID, 10);
  PID_SetOutputLimits(&TPID, -3000, 3000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //SR05_Read1();
	  SR05_Read2();
	  Distance1 = Distance1;
	  Distance2 = Distance2;
	  actualDistance = -(24-Distance2);
	  actualDistancedouble = actualDistance;
	  HAL_Delay(10);
	  PID_Compute(&TPID);
	  moveto(PIDOut);
	  actualpositionforsend = (actualposition*100)/25;
	  size_len = sprintf(uartsendbuffer, "%d,",actualDistance);
	  HAL_UART_Transmit(&huart2,uartsendbuffer,size_len ,HAL_MAX_DELAY);
	  size_len = sprintf(uartsendbuffer, "%d\r\n",actualpositionforsend);
	  HAL_UART_Transmit(&huart2,uartsendbuffer,size_len ,HAL_MAX_DELAY);
	  //sprintf(uartsendbuffer, "%d\r\n",PIDOut);
	  //HAL_UART_Transmit(&huart2,uartsendbuffer,sizeof(uartsendbuffer),HAL_MAX_DELAY);
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 42;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  htim3.Init.Prescaler = 7000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 168-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0xffff-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Pulse_Pin|Dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, trig1_Pin|trig2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Pulse_Pin Dir_Pin */
  GPIO_InitStruct.Pin = Pulse_Pin|Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : trig1_Pin trig2_Pin */
  GPIO_InitStruct.Pin = trig1_Pin|trig2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 if(recvd_data == '\r' || recvd_data == 'p'  || recvd_data == 'i' || recvd_data == 'd' || recvd_data == 'm') //when enter is pressed go to this condition
	 {
		 data_buffer[countuart++]='\r';
		 //HAL_UART_Transmit(huart,data_buffer,countuart,HAL_MAX_DELAY); //transmit the full sentence again

		 //moveto(atol(data_buffer));
		 //sprintf(uartsendbuffer, "%d\r\n",Distance1);
		 int32_t temp = atol(data_buffer);
		 if(recvd_data=='p')p = temp;
		 if(recvd_data=='i')i = temp;
		 if(recvd_data=='d')d = temp;
		 if(recvd_data=='m')setposition = temp;

		 PID(&TPID, &actualDistancedouble, &PIDOut, &setposition, p, i, d, _PID_P_ON_E, _PID_CD_DIRECT);
		 PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
		 PID_SetSampleTime(&TPID, 10);
		 PID_SetOutputLimits(&TPID, -3000, 3000);
		 //HAL_UART_Transmit(huart,data_buffer,10,HAL_MAX_DELAY);
		 //sprintf(data_buffer, "%d\r\n",destposition);
		 //HAL_UART_Transmit(huart,data_buffer,countuart,HAL_MAX_DELAY);

		 memset(data_buffer, 0, countuart); // enpty the data buffer
		 countuart=0;
	 }
	 else
	 {
		 data_buffer[countuart++] = recvd_data; // every time when interrput is happen, received 1 byte of data
	 }
	 HAL_UART_Receive_IT(&huart2,&recvd_data,1); //start next data receive interrupt
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim3)
	{
		if(pulsestatus==1)
		{
			HAL_GPIO_WritePin(Pulse_GPIO_Port, Pulse_Pin, GPIO_PIN_RESET);
			pulsestatus = 0;
		}
		else
		{
			if(destposition>=actualposition){
				HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, GPIO_PIN_RESET);
				actualposition++;
				HAL_GPIO_WritePin(Pulse_GPIO_Port, Pulse_Pin, GPIO_PIN_SET);
				pulsestatus = 1;
			}
			else if(destposition<actualposition){
				HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, GPIO_PIN_SET);
				actualposition--;
				HAL_GPIO_WritePin(Pulse_GPIO_Port, Pulse_Pin, GPIO_PIN_SET);
				pulsestatus = 1;
			}

		}
		HAL_GPIO_TogglePin(Pulse_GPIO_Port, Pulse_Pin);
	}
}

//SR05_Read
void input_capture1(void)
	{
		if (Is_First_Captured1==0)
		{
			IC_Val11 = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_1);
			Is_First_Captured1 = 1;

			__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (Is_First_Captured1==1)
		{
			IC_Val21 = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_1);
			__HAL_TIM_SET_COUNTER(&htim8, 0);

			if (IC_Val21 > IC_Val11)
			{
				Difference1 = IC_Val21-IC_Val11;
				if(((Difference1*2)/100)<60) Distance1 = (Difference1*2)/100;
			}



			Is_First_Captured1 = 0;


			__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim8, TIM_IT_CC1);
			}
}

void input_capture2(void)
	{
		if (Is_First_Captured2==0)
		{
			IC_Val12 = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_2);
			Is_First_Captured2 = 1;

			__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (Is_First_Captured2==1)
		{
			IC_Val22 = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_2);
			__HAL_TIM_SET_COUNTER(&htim8, 0);

			if (IC_Val22 > IC_Val12)
			{
				Difference2 = IC_Val22-IC_Val12;
				if(((Difference2*2)/100)<60) Distance2 = (Difference2*2)/100;
			}

			Is_First_Captured2 = 0;


			__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim8, TIM_IT_CC2);
			}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		input_capture1();
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		input_capture2();
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
