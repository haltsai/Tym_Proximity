/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t Rx_Data[1] = {0};

typedef union {
	#define FloatSize (2)
	float f4Byte[FloatSize];
	char  Byte4float[4*FloatSize];
} xu_floatChar;
xu_floatChar PostAvgOffsetValue = {0};
#define SAVE_ON_FLOAT  PostAvgOffsetValue.f4Byte[0]
#define SAVE_OFF_FLOAT PostAvgOffsetValue.f4Byte[1]

typedef union {
	char u8Byte;
	struct {
		unsigned bit0:1;
		unsigned bit1:1;
		unsigned bit2:1;
		unsigned bit3:1;
		unsigned bit4:1;
		unsigned bit5:1;
		unsigned bit6:1;
		unsigned bit7:1;
	} sbit;
} xu_sbit;
xu_sbit flag[1] = {0};
#define BTN_Blue       flag[0].sbit.bit0
#define GetUartCMD     flag[0].sbit.bit1

typedef enum {
	xePR_State_Empty,
	xePR_State_Ready,
	xePR_State_Save,
	xePR_State_Idle,
} xe_ProximityReady;
xe_ProximityReady ProximityReady = xePR_State_Empty;
unsigned int SaveCurrFreqOffset = 0;

typedef enum {
	xePOCFD_PowerOn,
	xePOCFD_FreeRun,
} xe_PowerOn_Check_Freq_Directly;
xe_PowerOn_Check_Freq_Directly xe_WearValueState = xePOCFD_PowerOn;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void fnPollingTask1ms(void);
void fnPollingTask5ms(void);
void fnPollingTask10ms(void);
void fnPollingTask50ms(void);
void fnPollingTask100ms(void);
void fnPollingTask500ms(void);
void fnPollingTask1000ms(void);
void fnInfinitePolling(void);
void fnSteup(void);
void fnButtonEvent(void);

float ffnPostAvg_0d05(float u32Input);
float ffnPostAvg_0d25(float u32Input);

void save_to_flash(uint8_t *data);
void read_flash(uint8_t* data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void fnPollingTask1ms(void) {
	static unsigned int Count1ms = 0;

	if(0)printf("Count1ms:%d\r\n", ++Count1ms);
}

void fnPollingTask5ms(void) {
	static unsigned int Count5ms = 0;

	if(0)printf("Count5ms:%d\r\n", Count5ms);

	//Get TIM2 CNT
	#if(1)
		unsigned int ReadTIM2CNT = __HAL_TIM_GET_COUNTER(&htim2);
		__HAL_TIM_SET_COUNTER(&htim2, 0);

		//Get current TIM2 CNT
		if(0)printf(",%d",  ReadTIM2CNT);
	#endif

	//Get Current Frequency
	#if(1)
		#define SaveTIM2CNT_NUM (200)
		static unsigned int SaveTIM2Cnt[SaveTIM2CNT_NUM] = {0};

		//FIFO
		for(int i=SaveTIM2CNT_NUM-1; i>0; i--) {
			SaveTIM2Cnt[i] = SaveTIM2Cnt[i-1];
		} SaveTIM2Cnt[0] = 0;

		//Push one data into queue
		SaveTIM2Cnt[0] = ReadTIM2CNT;

		//Check SaveTIM2Cnt Queue full
		switch(ProximityReady) {
			static unsigned int WaitProxiityReadyCNT = 0;
			case xePR_State_Empty:
				if(WaitProxiityReadyCNT++ >= (1000/5)*2 ) {
					ProximityReady = xePR_State_Ready;
				}
				break;

			default:
				break;
		}

		//SUM datax200
		unsigned int SUMTIM2CNT = 0;
		for(int i=0; i<SaveTIM2CNT_NUM; i++) {
			SUMTIM2CNT += SaveTIM2Cnt[i];
		}

		//Plot Current Frequency
		if(0)printf(",%d", SUMTIM2CNT);
	#endif

	//do Post average
	#if(1)
		float ftmp;

		//Plot Frequency post avg (0.25)
		ftmp = SUMTIM2CNT;
		float Get0d25PostAvg = ffnPostAvg_0d25(ftmp);
		if(0)printf(",%f", Get0d25PostAvg);

		//Plot Frequency post avg (0.05)
		ftmp = SUMTIM2CNT;
		float Get0d05PostAvg = ffnPostAvg_0d05(ftmp);
		if(0)printf(",%f", Get0d05PostAvg);
	#endif

	//Plot Offset based on post avg (0.05)
	#if(1)
		//Offset based on post avg (0.05)
		const float cfOffsetValue = 175.0;
		float fMaxBoundary = Get0d05PostAvg+cfOffsetValue,
			  fMinBoundary = Get0d05PostAvg-cfOffsetValue;
		if(1)printf(",%f,%f", fMaxBoundary-fMinBoundary, fMinBoundary-fMinBoundary);
	#endif

	//Plot Data line
	#if(1)
		//Plot Frequency post avg (0.25)
		float fPostAvgFrequency = Get0d25PostAvg-fMinBoundary;
		if(1)printf(",%f", fPostAvgFrequency);

		//Plot Current Frequency
		static const float OffsetCurrFrequency = 8787000;
		float fCurrentFrequencyOffset = SUMTIM2CNT-OffsetCurrFrequency;
		switch(ProximityReady) {
			case xePR_State_Empty:
				break;

			case xePR_State_Ready:
				ProximityReady = xePR_State_Save;
				break;

			case xePR_State_Save:
				ProximityReady = xePR_State_Idle;
				break;

			case xePR_State_Idle:
				//Plot Current Frequency Offset
                if(0)printf(",%f", fCurrentFrequencyOffset);
				break;
		}
	#endif

	//Check Wear state
	#if(1)
		typedef enum {
			xeWS_WearNull,
			xeWS_WearOn,
			xeWS_WearOff,
		} xe_WearState;
		static xe_WearState xeWearState = xeWS_WearNull;

		float TransferMax = 2*cfOffsetValue,
			  TransferMin = 0.0;
		switch(xe_WearValueState) {
			case xePOCFD_PowerOn:
				switch(ProximityReady) {
					case xePR_State_Empty:
						break;

					default:
						if(SAVE_ON_FLOAT > Get0d25PostAvg) {
							xeWearState = xeWS_WearOn;
						} else {
							xe_WearValueState = xePOCFD_FreeRun;
							xeWearState = xeWS_WearOff;
						}

						//Wear off
						if( (fPostAvgFrequency>TransferMax) ) {
							if(xeWearState != xeWS_WearOff) {
								xeWearState = xeWS_WearOff;

								xe_WearValueState = xePOCFD_FreeRun;
							}
						}
						break;
				}
				break;

			case xePOCFD_FreeRun:
				//Normal
				if( (TransferMax>fPostAvgFrequency) &&
						         (fPostAvgFrequency>TransferMin) ) {
				}

				//Wear on
				if( (TransferMin>fPostAvgFrequency) ) {
					if(xeWearState != xeWS_WearOn) {
						xeWearState = xeWS_WearOn;

						SAVE_ON_FLOAT = Get0d25PostAvg;
					}
				}

				//Wear off
				if( (fPostAvgFrequency>TransferMax) ) {
					if(xeWearState != xeWS_WearOff) {
						xeWearState = xeWS_WearOff;

						SAVE_OFF_FLOAT = Get0d25PostAvg;
					}
				}
				break;
		}

		//Plot Wear state
		switch(xeWearState) {
			case xeWS_WearNull:
				break;

			case xeWS_WearOn:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

				if(1)printf(",%f",  TransferMax+50.0);
				break;

			case xeWS_WearOff:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

				if(1)printf(",%f", -50.0);
				break;
		}

		//debug
		if(0)printf(",%d,%d",  1500, -1500);
		if(0)printf(",%f,%f",  SAVE_OFF_FLOAT, SAVE_ON_FLOAT);
	#endif

 	if(1)printf("\n");
}

void fnPollingTask10ms(void) {
	static unsigned int Count10ms = 0;

	if(0)printf("Count10ms:%d\r\n", ++Count10ms);
}

void fnPollingTask50ms(void) {
	static unsigned int Count50ms = 0;

	if(0)printf("Count50ms:%d\r\n", ++Count50ms);
}

void fnPollingTask100ms(void) {
	static unsigned int Count100ms = 0;

	if(0)printf("Count100ms:%d\r\n", ++Count100ms);

	switch(ProximityReady) {
		case xePR_State_Empty:
			break;

		default:
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			break;
	}

}

void fnPollingTask500ms(void) {
	static unsigned int Count500ms = 0;

	if(0)printf("Count500ms:%d\r\n", ++Count500ms);

	if(BTN_Blue) {
		BTN_Blue = 0;
		fnButtonEvent();
		printf("BlueButton Press\r\n");
	}
}

void fnPollingTask1000ms(void) {
	static unsigned int Count1000ms = 0;

	if(0)printf("Count1000ms:%d\r\n", ++Count1000ms);
}

void fnInfinitePolling(void) {

}

void fnSteup(void) {
    //Read Flash
    if(1) {
        memset(PostAvgOffsetValue.Byte4float, 0, sizeof(PostAvgOffsetValue));

        read_flash((uint8_t*)PostAvgOffsetValue.Byte4float);
        ProximityReady = xePR_State_Empty;
        printf("Read:<%f:%f>, done!!\r\n", PostAvgOffsetValue.f4Byte[0], PostAvgOffsetValue.f4Byte[1]);
    }
}

void fnButtonEvent(void) {
    //Write Flash
    if(1) {
        save_to_flash((uint8_t*)PostAvgOffsetValue.Byte4float);
        printf("Write:<%f:%f>, done!!\r\n", PostAvgOffsetValue.f4Byte[0], PostAvgOffsetValue.f4Byte[1]);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  fnSteup();
  printf("Initial Done\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if(1) {  //Polling Task
		static unsigned int   polling1ms    = 0;
		static unsigned char  polling5ms    = 0,
							  polling10ms   = 0,
							  polling50ms   = 0,
							  polling100ms  = 0;
		static unsigned short polling500ms  = 0,
				              polling1000ms = 0;

		//Polling per 1ms
		if( polling1ms != HAL_GetTick() ) {
			polling1ms++;

			fnPollingTask1ms();

			//Counting Polling Timer
			polling5ms++;
			polling10ms++;
			polling50ms++;
			polling100ms++;
			polling500ms++;
			polling1000ms++;
		}

		//Polling per 5ms
		if(polling5ms>=5) {
			polling5ms = 0;

			fnPollingTask5ms();
		}

		//Polling per 10ms
		if(polling10ms>=10) {
			polling10ms = 0;

			fnPollingTask10ms();
		}

		//Polling per 50ms
		if(polling50ms>=50) {
			polling50ms = 0;

			fnPollingTask50ms();
		}

		//Polling per 100ms
		if(polling100ms>=100) {
			polling100ms = 0;

			fnPollingTask100ms();
		}

		//Polling per 500ms
		if(polling500ms>=500) {
			polling500ms = 0;

			fnPollingTask500ms();
		}

		//Polling per 1000ms
		if(polling1000ms>=1000) {
			polling1000ms = 0;

			fnPollingTask1000ms();
		}

	}  //end of if(1) {  //Polling Task

	fnInfinitePolling();

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
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* PVD_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(PVD_IRQn);
  /* FLASH_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(FLASH_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FLASH_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* RCC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RCC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RCC_IRQn);
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END TIM2_Init 2 */

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
  HAL_UART_Receive_IT(&huart2, (uint8_t *)Rx_Data, 1);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);


  //Hal Modify
  if(1) {
	  /*Configure GPIO pin : D4 D5 Pin */
	  GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_5;
	  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull  = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  }

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */

  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFFFFFF);
  return ch;
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */

	#define GetData() Rx_Data[0]
    static char GetCMD[16] = {0},
    		    GetCNT = 0;
    switch(GetData()) {
		case '\r':
		case '\n':
		case '\a':
			GetCMD[(int)GetCNT] = 0;
			GetCNT = 0;

			GetUartCMD = 1;
			break;

		default:
			GetCMD[(int)GetCNT++] = GetData();
			break;
    }

    if(GetUartCMD) {
    	GetUartCMD = 0;

    	printf("GetCMD:%s\r\n", GetCMD);

    	if( strstr(GetCMD, "write") != NULL ) {
    		printf("okok, get write\r\n");

            //Read Write Flash
            if(1) {
                  char write_data[50] = {0};
                  memset(write_data, 0, sizeof(write_data));
                  strcpy(write_data, "Hello World!!!");

                  save_to_flash((uint8_t*)write_data);
                  printf("Write:<%s>, done!!\r\n", write_data);
            }
    	}

    	if( strstr(GetCMD, "read") != NULL ) {
    		printf("okok, get read\r\n");

            //Read Write Flash
            if(1) {
                  char read_data[50] = {0};
                  memset(read_data, 0, sizeof(read_data));

                  read_flash((uint8_t*)read_data);
                  printf("Read:<%s>, done!!\r\n", read_data);
            }
    	}
    }

	if(0)printf("Rx_Data:%c:0x%02X\r\n", Rx_Data[0], Rx_Data[0]);
	HAL_UART_Receive_IT(&huart2, (uint8_t *)Rx_Data, 1);
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
	switch(GPIO_Pin) {
		case GPIO_PIN_13:
			BTN_Blue = 1;
			break;
	}
}

float ffnPostAvg_0d05(float u32Input) {
	static float fsaveFreq = 0.0;
		   float fcurrFreq = 0.0;

	const float postAVGweight = (0.05);

	fcurrFreq = u32Input;
	fsaveFreq = fsaveFreq*(1-postAVGweight) + fcurrFreq*(postAVGweight);

	return fsaveFreq;
}

float ffnPostAvg_0d25(float u32Input) {
	static float fsaveFreq = 0.0;
		   float fcurrFreq = 0.0;

	const float postAVGweight = (0.25);

	fcurrFreq = u32Input;
	fsaveFreq = fsaveFreq*(1-postAVGweight) + fcurrFreq*(postAVGweight);

	return fsaveFreq;
}

#if(1)  //For Read Write Flash
	#define FLASH_STORAGE  0x08015000  //0x0800 0000 + 2048*42
	#define page_size      0x800       //2048

	void save_to_flash(uint8_t *data) {
		volatile uint32_t data_to_FLASH[(strlen((char*)data)/4) + (int)((strlen((char*)data) % 4) != 0)];
		memset( (uint8_t*)data_to_FLASH, 0, strlen((char*)data_to_FLASH) );
		strcpy(    (char*)data_to_FLASH, (char*)data );

		volatile uint32_t data_length =       (strlen((char*)data_to_FLASH) / 4)
									  + (int)((strlen((char*)data_to_FLASH) % 4) != 0);

		volatile uint16_t pages =       (strlen((char*)data)/page_size)
								+ (int)((strlen((char*)data)%page_size) != 0);

		/* Unlock the Flash to enable the flash control register access *************/
		HAL_FLASH_Unlock();

		/* Allow Access to option bytes sector */
		HAL_FLASH_OB_Unlock();

		/* Fill EraseInit structure*/
		FLASH_EraseInitTypeDef EraseInitStruct;
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.PageAddress = FLASH_STORAGE;
		EraseInitStruct.NbPages     = pages;
		uint32_t PageError;

		volatile uint32_t write_cnt = 0, index = 0;

		volatile HAL_StatusTypeDef status;
								   status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
		while(index < data_length) {
			if (status == HAL_OK) {
				status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE+write_cnt, data_to_FLASH[index]);
				if(status == HAL_OK) {
					write_cnt += 4;
					index++;
				}
			}
		}

		HAL_FLASH_OB_Lock();
		HAL_FLASH_Lock();
	}

	void read_flash(uint8_t* data) {
		volatile uint32_t read_data = 0;
		volatile uint32_t read_cnt  = 0;

		do {
			read_data = *(uint32_t*)(FLASH_STORAGE + read_cnt);

			if(read_data != 0xFFFFFFFF) {
				data[read_cnt + 0] = (uint8_t)(read_data >> 0);
				data[read_cnt + 1] = (uint8_t)(read_data >> 8);
				data[read_cnt + 2] = (uint8_t)(read_data >> 16);
				data[read_cnt + 3] = (uint8_t)(read_data >> 24);

				read_cnt += 4;
			}
		} while(read_data != 0xFFFFFFFF);
	}
#endif
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
