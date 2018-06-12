/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
  ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <LSM6DS3.h>
#include <string.h>
#include <stdlib.h>  // provisional atoi()
#include <stdarg.h>
#include <time.h>  // provionale time()

#include "main.h"
#include "stm32f2xx_hal.h"
#include "M95lite.h"
#include "Definitions.h"
#include "southbound_ec.h"
#include "MQTTAPI.H"
#include "NBinterface.h"
#include "BLInterface.h"


#include "circular.h"
#include "utils.h"

#include "dma.h"		// BYDMA
#include "CAN_Util.h"
#include "i2c.h"
#include "TSL2561.h"
#include "TSL2561_Internals.h"
#include "PCAL6416A.h"


// has to be moved to a utilities header when it should be
extern 	int		tprintf(int hcon, char *texto,...);

TIM_HandleTypeDef    TimHandle;
TIM_OC_InitTypeDef sConfig;
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;
TIM_OC_InitTypeDef sConfig;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

UART_HandleTypeDef huartDummy;
/// It is defined a variable but it is not going to be used.

#define GPRS_UART huart6

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
StrLSM6DS3 strLSMDev;
StrPCAL6416A strPCAL6416A;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_IWDG_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int		bydma = 1;
int		nirqs = 0;
int		hmqtt; 			// handle to the mqtt connection. Has to be a SIGNED integer
int		connections = 0;  // this counter gives us a hint for distinguish reboot/connection

// shoots to get connection times
uint32_t	tc0, //	start connection
			tc1, // after open IP
			tc2, // broker connected
			tc3;
st_CB *DataBuffer;
int elapsed10secondsAux=0;
uint16_t elapsed10seconds=0; 					// At beginning this is 0
uint8_t LOG_ACTIVATED=0;				 		/// Enable to 1 if you want to show log through logUART
uint8_t LOG_GPRS=0;  							/// For showing only GPRS information
uint8_t WDT_ENABLED=0;						 /// Enable for activate independent watch dog timer
uint8_t timeoutGPRS=0; 						/// At beginning this is 0
uint32_t timeout=1000;				 		/// Timeout between AT command sending is 1000 milliseconds.
uint8_t rebootSystem=0;						 /// At beginning this is 0
uint8_t nTimesMaximumFail_GPRS=2; 			/// For initial loop of initializing GPRS device
uint8_t retriesGPRS=1; 						/// only one retries per AT command if something goes wrong
uint8_t existDNS=1; 							/// The IP of main server to connect is not 4 number separated by dot. It is a DNS.
uint8_t offsetLocalHour=0; 					/// for getting UTC time
//  uint8_t APN[SIZE_APN]; 						/// Array where is saved the provider APN (format as example in Definitions.h)
//  uint8_t IPPORT[SIZE_MAIN_SERVER]; 			/// Array where is saved main destination server to connect (IP and PORT format as example in Definitions.h)
//  uint8_t SERVER_NTP[SIZE_NTP_SERVER]; 			/// Array where is saved server NTP to get UTC time (format as example in Definitions.h)
//  uint8_t calendar[10];               			/// Array for saving all calendar parameters get from NTC server
//  uint8_t idSIM[30];                 			 /// Array where is saved ID from SIMcard
uint8_t openFastConnection=0;      			 /// by default to 0, it is used for doing a quick connection when it is needed to call the connect function again
uint8_t setTransparentConnection=1;  			/// 1 for transparent connection, 0 for non-transparent. Then all data flow is command AT+ data
//  uint8_t GPRSbuffer[SIZE_GPRS_BUFFER];			 /// received buffer with data from GPRS
uint8_t dataByteBufferIRQ;  					/// Last received byte from GPRS
//uint16_t GPRSBufferReceivedBytes;     		/// Number of received data from GPRS after a cleanningReceptionBuffer() is called
//uint16_t indexGPRSBufferReceived;
//uint16_t indexPickingReceivedBytes=0;
uint8_t connected=0;
//unsigned char buffer2[SIZE_GPRS_BUFFER];
int32_t quantityReceived=0;
/* USER CODE END 0 */

int main(void)
{
	/* Added by Owais */
	EnumCQErrors enCQErr;
	uint8_t* ptruintMsg;
	uint8_t eventCode;
	StrI2CDeviceInfo i2cDevices[2];
	uint32_t cnt_EventsRx = 0;
	uint32_t cnt_Event_FF = 0;
	uint32_t cnt_Event_WU = 0;
	uint32_t cnt_Event_SL = 0;
	uint32_t cnt_Event_MD = 0;
	uint32_t cnt_Event_PD = 0;

	i2cDevices[0].StartAddress = 0;
	i2cDevices[0].DeviceAddress = LSM6DSM_ADDR_LOW;
	i2cDevices[0].Is8BitRegisters = true;
	i2cDevices[0].ByteCount = 0x6B + 1; /* Total registers */

	i2cDevices[0].StartAddress = 0;
	i2cDevices[0].DeviceAddress = PCAL6416A_ADDR_LOW;
	i2cDevices[0].Is8BitRegisters = true;
	i2cDevices[0].ByteCount = 8; /* read first eight registers only */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

	greenON;
	MX_ADC1_Init();
	//MX_TIM3_Init();// francis
	MX_TIM7_Init();
	//MX_USART2_UART_Init();
	MX_USART6_UART_Init();
	//MX_IWDG_Init();

	//MX_I2C2_Init();
	initializePWM();
	/* USER CODE BEGIN 2 */
	bool bI2C1Res = Config_I2C1(&hi2c1, 400000, false, i2cDevices, sizeof(i2cDevices)/sizeof(i2cDevices[0]));

	if(bI2C1Res == false)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_Delay(30);

	__enable_irq();

	/* Initialize TSL2561 sensor, added 12-May-2018 */
	/*tsl2561_t dev_tsl2561;
	if( tsl2561_init(&dev_tsl2561, &hi2c1, TSL2561_ADDR_FLOAT, TSL2561_GAIN_16X, TSL2561_INTEGRATIONTIME_13MS) != TSL2561_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}*/

	PCAL6416A_SetupRefs(&strPCAL6416A, &hi2c1, PCAL6416A_ADDR_LOW);
	/*configure port 0.0 as output */
	bool bpc1 = PCAL6416A_MakePinOutput(&strPCAL6416A, 0, 0x01);
	/*set 0.0, AG_CS as HIGH */
	bool bpc2 = PCAL6416A_WritePinValue(&strPCAL6416A, 0, 0x01, true);
	bool bpc3 = PCAL6416A_MakePinInput(&strPCAL6416A, 1, 0x10); /* Pin 1.4 should be input */
	bool bpc4 = PCAL6416A_EnableIntOnPin(&strPCAL6416A, 1, 0x10); /* Pin 1.4 rising edge (default) set as an interrupt source */

	LSM6DS3_SetupRefs(&strLSMDev, &hi2c1, LSM6DSM_ADDR_LOW);
	bool b1 = LSM6DS3_Config(&strLSMDev, En_Mode_Both, En_Pow_HIGHPERF_416Hz, 0, 2);
	bool b2 = LSM6DS3_Set_Events(&strLSMDev, true, true, true, true, true);

	while (1)
	{
		/* Process LSM6DS3 events messages from event queue */
		while(true)
		{
			ptruintMsg = LSM6DS3_Get_Event(&enCQErr);
			eventCode = *ptruintMsg;

			if(enCQErr == CQ_ERROR_OK)
			{
				cnt_EventsRx++;

				if(eventCode & MASK_EVENT_FF)
				{
					/* free fall event was detected */
					cnt_Event_FF++;
				}

				if(eventCode & MASK_EVENT_WU)
				{
					/* wake up event detected */
					cnt_Event_WU++;
				}

				if(eventCode & MASK_EVENT_SL)
				{
					/* sleep event detected */
					cnt_Event_SL++;
				}

				if(eventCode & MASK_EVENT_MD)
				{
					/* motion detect event detected */
					cnt_Event_MD++;
				}

				if(eventCode & MASK_EVENT_PD)
				{
					/* step event detected */
					cnt_Event_PD++;
				}

				/* Read all the device registers which we are interested in for all devices */
				bool readI2C = Read_I2C1_DevicesInterest(&hi2c1);

				if(readI2C)
				{
					/* We have two devices at index 0 and 1 */
					/* These structures have a byte array which will have the register values designed in initialization */
					StrI2CDeviceInfo* str1 = Get_DeviceInterestReference(0);
					StrI2CDeviceInfo* str2 = Get_DeviceInterestReference(1);
				}
			}
			else
			{
				break;
			}
		}

	} /* main while loop */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
}

/* Call back for all GPIO EXTI type interrupts */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_2) /* Line 2 i.e. PC0 */
  {
	  /* it means that GPIO expander /INT1 called */
	  ProcessISR(&strLSMDev);
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = (SystemCoreClock/1000)-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10; //10ms
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;


  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) //francis
    {
      _Error_Handler(__FILE__, __LINE__);
    }

  /*
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);
  */

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  //htim7.Init.Prescaler = 0;
  htim7.Init.Prescaler = (SystemCoreClock/1000)-1;;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 19200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}



/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, blueRGB_Pin|redRGB_Pin|greenRGB_Pin|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|txDBG_3G_Pin|GPIO_PIN_9|GPIO_PIN_10
                          |emerg_3G_Pin|pwrKey_3G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|Relay1_Pin|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : blueRGB_Pin redRGB_Pin greenRGB_Pin */
  GPIO_InitStruct.Pin = blueRGB_Pin|redRGB_Pin|greenRGB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : status_3G_Pin netlight_3G_Pin */
  GPIO_InitStruct.Pin = status_3G_Pin|netlight_3G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC4 PC5 PC8
                           PC10 PC11 PC12 */
  GPIO_InitStruct.Pin =    GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA4 PA5 PA6
                           PA7 txDBG_3G_Pin PA9 PA10 */
  GPIO_InitStruct.Pin =    GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|txDBG_3G_Pin|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins :  PB1 PB2 PB10
                           PB11 Relay1_Pin PB14 PB15
                           PB3 PB4 PB5 PB6
                           PB7*/
  GPIO_InitStruct.Pin =    GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|Relay1_Pin|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*GPIO_InitStruct.Pin =    GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);*/

  /*Configure GPIO pin : PWM_sim_Pin */
  GPIO_InitStruct.Pin = PWM_sim_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWM_sim_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : emerg_3G_Pin pwrKey_3G_Pin */
  GPIO_InitStruct.Pin = emerg_3G_Pin|pwrKey_3G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : rxDBG_3G_Pin */
  GPIO_InitStruct.Pin = rxDBG_3G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(rxDBG_3G_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  /* INT2 on LSM6DS3 cannot be used */
  /*GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);*/

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}


/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  if (huart->Instance==GPRS_UART.Instance)
 {
	  if (! bydma) { // only if not BYDMA
	  	 nirqs++;
	  		 allnew += Write(DataBuffer, dataByteBufferIRQ);
	  		 if (IsFull(DataBuffer)) {
	  			DataBuffer->overruns++;
	  		}
//	  		{
//	  			int nextw = (write_offset + 1) % bufsize;  // next position to be written
//	  			Cbuffer[write_offset++] =  dataByteBufferIRQ;
//	  			write_offset = nextw;
//	  	 	 	allnew += Write(dataByteBufferIRQ);
	  			balnew++;
//	  		}

//			 (huart,&dataByteBufferIRQ,1);
//	  	 }
//	  	 {
	  			/**
			  GPRSBufferReceivedBytes++;
			  GPRSbuffer[indexGPRSBufferReceived]=dataByteBufferIRQ;
			  indexGPRSBufferReceived=(indexGPRSBufferReceived+1)%SIZE_GPRS_BUFFER;
	  		  allold++;
	  		  balold++;
	  		  **/
//	  	 }
	  		  HAL_UART_Receive_IT(huart,&dataByteBufferIRQ,1);
	  }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	HAL_NVIC_DisableIRQ(TIM7_IRQn);
	if (htim->Instance==TIM7)
	{
	  	 AddSeconds(10);

        elapsed10seconds++;
		if (elapsed10seconds%TIMING_TIMEOUT_GPRS==0)

		{
			/// Tiempo timeoutGPRS
			timeoutGPRS=1;

		}


	}
	HAL_NVIC_EnableIRQ(TIM7_IRQn);


}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
