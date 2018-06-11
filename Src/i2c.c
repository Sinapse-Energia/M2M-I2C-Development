/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
StrI2CDeviceInfo StrDevices[MAX_DEVICES];
uint8_t          CountDevices;
uint32_t         bitrate_i2c;
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = bitrate_i2c;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/*****************************************************************************
 * Function name    : Config_I2C1
 *    returns       : bool
 *    arg1          : I2C_HandleTypeDef *hi2c, the i2c interface hardware structure handle
 *    arg2          : uint32_t baudRate, the I2C bitrate
 *    arg3          : bool bTenBitAddress, true if 10-bit slave addresses used
 *    arg4          : StrI2CDeviceInfo* strDeviceInterest
 *    arg5          : uint8_t devCount, devices of interest to read count
 * Created by       : Owais
 * Date created     : 11-JUN-2018
 * Description      : Initialize I2C with a specific rate
 *
 *                    Assumptions:
 *                    10-bit or 7-bit addressing
 *                    MCU is in master mode
 *
 * Notes            : listen only mode and address list don't make any sense when using
 *                    MCU in master mode
 *
 *                    Copies the device of interest information
 *****************************************************************************/
bool Config_I2C1(I2C_HandleTypeDef *hi2c, uint32_t baudRate, bool bTenBitAddress, StrI2CDeviceInfo strDeviceInterest[], uint8_t devCount)
{
	bool result = false;
	uint8_t loop;
	bitrate_i2c = baudRate;

	if(devCount <= MAX_DEVICES)
	{
		hi2c->Instance = I2C1;
		hi2c->Init.ClockSpeed = baudRate;
		hi2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
		hi2c->Init.OwnAddress1 = 0;

		if (bTenBitAddress == false)
			hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		else
			hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;

		hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		hi2c->Init.OwnAddress2 = 0;
		hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		hi2c->Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;

		if (HAL_I2C_Init(hi2c) != HAL_OK)
		{
			result = false;
		}
		else
		{
			result = true;
		}
	}

	/* copy the device information */
	for(loop = 0; loop < devCount; loop++)
	{
		StrDevices[loop].DeviceAddress = strDeviceInterest[loop].DeviceAddress;
		StrDevices[loop].StartAddress = strDeviceInterest[loop].StartAddress;
		StrDevices[loop].Is8BitRegisters = strDeviceInterest[loop].Is8BitRegisters;

		if(strDeviceInterest[loop].ByteCount < MAX_BYTES_TO_READ)
		{
			StrDevices[loop].ByteCount = strDeviceInterest[loop].ByteCount;
		}
		else
		{
			result = false;
			break;
		}
	}

	return result;
}
/*****************************************************************************
 * Function name    : Write_I2C1_Message
 *    returns       : HAL_StatusTypeDef
 *    arg1          : I2C_HandleTypeDef *hi2c, the i2c interface hardware structure handle
 *    arg2          : uint16_t i2c_address, the slave address, 7 or bit specified in config function
 *    arg3          : uint8_t bytes[], the byte array to send
 *    arg4          : uint32_t byteCount, the byte count to transmit
 * Created by       : Owais
 * Date created     : 11-JUN-2018
 * Description      : Writes an array of bytes to an I2C device
 * Notes            :
 *****************************************************************************/
HAL_StatusTypeDef Write_I2C1_Message(I2C_HandleTypeDef *hi2c, uint16_t i2c_address, uint8_t bytes[], uint32_t byteCount)
{
	HAL_StatusTypeDef halres;
	halres = HAL_I2C_Master_Transmit(hi2c, i2c_address, bytes, byteCount, 500);
	return halres;
}
/*****************************************************************************
 * Function name    : Write_I2C1_Register
 *    returns       : HAL_StatusTypeDef
 *    arg1          : I2C_HandleTypeDef *hi2c, the i2c interface hardware structure handle
 *    arg2          : StrI2CDeviceInfo* strDeviceInterest, device of interest details to use when transmitting
 *    arg3          : uint8_t bytes[], the byte array to send
 *    arg4          : uint32_t byteCount, the byte count to transmit
 * Created by       : Owais
 * Date created     : 11-JUN-2018
 * Description      : Writes to a starting register address on a specific i2c device
 * Notes            :
 *****************************************************************************/
HAL_StatusTypeDef Write_I2C1_Register(I2C_HandleTypeDef *hi2c, StrI2CDeviceInfo* strDeviceInterest, uint8_t bytes[], uint32_t byteCount)
{
	HAL_StatusTypeDef halres;
	uint32_t memSizeControlWord;

	if(strDeviceInterest->Is8BitRegisters == true)
		memSizeControlWord = I2C_MEMADD_SIZE_8BIT;
	else
		memSizeControlWord = I2C_MEMADD_SIZE_16BIT;

	halres = HAL_I2C_Mem_Write(hi2c,
			strDeviceInterest->DeviceAddress,
			strDeviceInterest->StartAddress,
			memSizeControlWord,
			bytes,
			byteCount,
			10);

	return halres;
}
/*****************************************************************************
 * Function name    : Read_I2C1_DevicesInterest
 *    returns       : bool, true if request was successfull for all devices
 *    arg1          : I2C_HandleTypeDef *hi2c, the i2c interface hardware structure handle
 * Created by       : Owais
 * Date created     : 11-JUN-2018
 * Description      : Uses all stored addresses to accomplish reading of starting registers upto specified count
 * Notes            :
 *****************************************************************************/
bool Read_I2C1_DevicesInterest(I2C_HandleTypeDef *hi2c)
{
	bool result = true;
	HAL_StatusTypeDef halres;
	uint32_t memSizeControlWord;

	for(uint8_t i=0; i < CountDevices; i++)
	{
		if(StrDevices[i].Is8BitRegisters == true)
			memSizeControlWord = I2C_MEMADD_SIZE_8BIT;
		else
			memSizeControlWord = I2C_MEMADD_SIZE_16BIT;

		halres = HAL_I2C_Mem_Read(hi2c,
				                  StrDevices[i].DeviceAddress,
				                  StrDevices[i].StartAddress,
								  memSizeControlWord,
				                  StrDevices[i].BytesRead,
				                  StrDevices[i].ByteCount,
				                  100);

		if(halres != HAL_OK)
		{
			result = false;
			break;
		}
	}
	return result;
}
/*****************************************************************************
 * Function name    : Get_DeviceInterestReference
 *    returns       : StrI2CDeviceInfo*, reference to device structure
 *    arg1          : uint8_t index, the requested index
 * Created by       : Owais
 * Date created     : 11-JUN-2018
 * Description      : Gives reference to device structure, used after doing mass read of bus
 * Notes            :
 *****************************************************************************/
StrI2CDeviceInfo* Get_DeviceInterestReference(uint8_t index)
{
	if(index < CountDevices)
	{
		return &StrDevices[index];
	}

	return NULL;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
