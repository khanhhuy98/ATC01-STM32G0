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
#include "string.h"
#include "stdio.h"
#include "thermistor.h"
#include "math.h"
#include "stdlib.h"
#include "driver_queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_TEMP    					-300
#define MAX_TEMP     					1000
#define MaxFrameIndex 				255


unsigned char MY_SLAVE_ID;
volatile unsigned char ResponseFrameSize;

volatile unsigned char data_in[MaxFrameIndex+1];
volatile unsigned char data_out[MaxFrameIndex+1];
volatile uint8_t DataPos = 0;
volatile unsigned int TotalCharsReceived;

static uint16_t Size_buff;
static int temp[MaxFrameIndex+1];
static uint8_t count = 0;
static unsigned int counter = 0;
static uint8_t checkavr = 0;

static uint32_t timout0 = 0;
static uint32_t timout1 = 0; 
static uint32_t timout2 = 0; 
static uint32_t timout3 = 0; 

uint8_t buffer;
uint8_t flagtest = 0;
uint8_t error_check = 0;

int bleng;
unsigned char buff1[128];

static unsigned char nsec[256];
char str[30];
char CS;

Queue queue;
uint8_t bufx[UART_RXSIZE_QUEUE];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static char NMEA_checkSum(char *nmea_data, int length)
{
  int crc = 0;
  int i;
  for (i = 0; i < length; i++)
  {
    crc ^= nmea_data[i];
  }
  return (char)crc;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		Queue_wirte(&queue, buffer);
		HAL_UART_Receive_IT(&huart1, &buffer, 1);
	}
}
void temp_cal(uint16_t beta_value)
{
	uint16_t ADC_VAL;
	ADC_Start(0);
	ADC_WaitForConv ();
	ADC_VAL = ADC_GetVal();
	voltage = (float)(ADC_VAL*1.0 ) * VCC / 4095.0;
    
  thermistorResistor = RESISTOR2_VALUE * voltage /(VOLTAGE_PULLUP - voltage); // Voltage divider equation
    
  temperatureK = 1 / (
                        log( thermistorResistor / RT_AT_ROOM_TEMP ) / beta_value +
                        1 / ROOM_TEMP );
     
  temperatureC = (temperatureK - 273.15)*10; // Temp in °C*10
	temp[count++] = temperatureC;
	if((temperatureC < MIN_TEMP)||(temperatureC > MAX_TEMP))
	{
		counter = 0;
	}
	else
	{
		counter++;
	}
}
void Data_process()
{
	uint8_t index = 0;
	int readbyte;
	while(queue.qout != queue.qin)
	{
		readbyte = Queue_read(&queue);
		data_in[DataPos++] = readbyte;
		if(readbyte == '#')
		{
			if((strncmp((char*)data_in, "ping",4) == 0)&&(strlen((char*)data_in) == 5))
			{
				flagtest = 1;
				Size_buff = 0;
				if(Size_buff == 0)
				{
					checkavr = 0;
				}
			}
			
			else if((strncmp((char*)data_in, "ping",4) == 0)&&(strlen((char*)data_in) > 5))
			{
				index = strlen((char*)data_in);
				memcpy(&nsec, (unsigned char*)&data_in[4], index - 5);
				for(int i = 0 ; i<index - 5; i++)
				{
					if((nsec[i] != '0')&&(nsec[i] != '1')&&(nsec[i] != '2')&&(nsec[i] != '3')&&(nsec[i] != '4')&&(nsec[i] != '5')&&(nsec[i] != '6')&&(nsec[i] != '7')&&(nsec[i] != '8')&&(nsec[i] != '9'))
					{
						error_check++;
					}
				}

				if(error_check > 0)
				{
					bleng = sprintf((char*)buff1,"Command ERROR! Please, try again!\n\r");
					HAL_UART_Transmit(&huart1, buff1, bleng, 1000);
					memset((unsigned char*)nsec, 0, index - 5);
					flagtest = 0;
					error_check = 0;
				}		
				Size_buff = atoi((const char*)nsec);
				memset((unsigned char*)nsec, 0, index - 5);
				if(Size_buff != 0)
				{
					checkavr = 1;
					count = 1;
				}
				else 
				{
					checkavr = 0;
				}
				if(checkavr == 1)
				{
					for(int j = 1; j < Size_buff; j++)
					{
						temp[Size_buff] += temp[j];
					}
					temp[0] =  (int)(temp[Size_buff] / (Size_buff));
			 		flagtest = 2;	
				}
			}
			else if ((strncmp((char*)data_in, "ping",4) != 0))
			{
				bleng = sprintf((char*)buff1,"Command ERROR! Please, try again!\n\r");
				HAL_UART_Transmit(&huart1, buff1, bleng, 1000);
				memset((unsigned char*)data_in, 0, MaxFrameIndex);
				flagtest = 0;
			}
			memset((unsigned char*)data_in, 0, MaxFrameIndex);	
			DataPos = 0;
		}
	}
	if(counter > 1000)
	{
		switch (flagtest)
		{
			case 0:
			{
				if(HAL_GetTick() - timout0 > 499)
				{
					sprintf(str, "#Temperature,%d,",(int)temperatureC);
					CS = NMEA_checkSum(str, (int) (strlen(str)));
//					printf("#Temperature,%d,%X\r\n", (int)temperatureC, CS);
					bleng =	sprintf((char*)buff1,"#Temperature,%d,%X\r\n", (int)temperatureC, CS);
					HAL_UART_Transmit(&huart1, buff1, bleng, 1000);
					timout0 = HAL_GetTick();
				}
			}
			break;
			case 1:
			{
				if(HAL_GetTick() - timout1 > 999)
				{
					sprintf(str, "#Temperature,%d,",(int)temperatureC);
					CS = NMEA_checkSum(str, (int) (strlen(str)));
//					printf("#Temperature,%d,%X\r\n", (int)temperatureC, CS);
					bleng = sprintf((char*)buff1,"#Temperature,%d,%X\r\n", (int)temperatureC, CS);
					HAL_UART_Transmit(&huart1, buff1, bleng, 1000);
					timout1 = HAL_GetTick();
				}
			}
			break;
			case 2:
			{
				if(HAL_GetTick() - timout2 > (999*Size_buff))
				{
					sprintf(str, "#Temperature,%d,",temp[0]);
					CS = NMEA_checkSum(str, (int) (strlen(str)));
//					printf("#Temperature,%d,%X\r\n", temp[0],CS);
					bleng = sprintf((char*)buff1,"#Temperature,%d,%X\r\n", temp[0], CS);
					HAL_UART_Transmit(&huart1, buff1, bleng, 1000);
					timout2 = HAL_GetTick();
				}
			}
			break;
		}
	}
	else if(counter == 0)
	{	
		if(HAL_GetTick() - timout3 > 999)
			{
				sprintf(str, "#Temperature,32767");
				CS = NMEA_checkSum(str, (int) (strlen(str)));
//				printf("#Temperature,32767,%X\n\r",CS);
				bleng = sprintf((char*)buff1,"#Temperature,32767,%X\n\r",CS);
				HAL_UART_Transmit(&huart1, buff1, bleng, 1000);
				timout3 = HAL_GetTick();
			}
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
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	Queue_Init(&queue,bufx,UART_RXSIZE_QUEUE);
	HAL_UART_Receive_IT(&huart1, &buffer, 1);
	LL_ADC_Enable(ADC1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		temp_cal(3950);
		Data_process();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0   ------> ADC1_IN0
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */

   #define ADC_CHANNEL_CONF_RDY_TIMEOUT_MS ( 1U)
   #if (USE_TIMEOUT == 1)
   uint32_t Timeout ; /* Variable used for Timeout management */
   #endif /* USE_TIMEOUT */

  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_CONFIGURABLE);

   /* Poll for ADC channel configuration ready */
   #if (USE_TIMEOUT == 1)
   Timeout = ADC_CHANNEL_CONF_RDY_TIMEOUT_MS;
   #endif /* USE_TIMEOUT */
   while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0)
     {
   #if (USE_TIMEOUT == 1)
   /* Check Systick counter flag to decrement the time-out value */
   if (LL_SYSTICK_IsActiveCounterFlag())
     {
   if(Timeout-- == 0)
         {
   Error_Handler();
         }
     }
   #endif /* USE_TIMEOUT */
     }
   /* Clear flag ADC channel configuration ready */
   LL_ADC_ClearFlag_CCRDY(ADC1);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  LL_ADC_SetTriggerFrequencyMode(ADC1, LL_ADC_CLOCK_FREQ_MODE_HIGH);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_2, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);

   /* Enable ADC internal voltage regulator */
   LL_ADC_EnableInternalRegulator(ADC1);
   /* Delay for ADC internal voltage regulator stabilization. */
   /* Compute number of CPU cycles to wait for, from delay in us. */
   /* Note: Variable divided by 2 to compensate partially */
   /* CPU processing cycles (depends on compilation optimization). */
   /* Note: If system core clock frequency is below 200kHz, wait time */
   /* is only a few CPU processing cycles. */
   uint32_t wait_loop_index;
   wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
   while(wait_loop_index != 0)
     {
   wait_loop_index--;
     }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_COMMON_1);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
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
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
