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

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

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
 * Function name    : I2C_Config
 *    returns       : bool
 *    arg1          : I2C_HandleTypeDef *hi2c, the i2c interface hardware structure handle
 *    arg2          : uint32_t baudRate, the I2C bitrate
 *    arg3          : bool bTenBitAddress, true if 10-bit slave addresses used
 *    arg4          : StrI2CDeviceInfo* strDeviceInterest
 *    arg5          : uint8_t devCount, devices of interest to read count
 * Created by       : Owais
 * Date created     : 14-JUN-2018
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
bool I2C_Config(I2C_HandleTypeDef *hi2c, uint32_t baudRate, bool bTenBitAddress)
{
	bool result = false;

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

	return result;
}
/*****************************************************************************
 * Function name    : I2C_Write_Raw_Message
 *    returns       : HAL_StatusTypeDef
 *    arg1          : I2C_HandleTypeDef *hi2c, the i2c interface hardware structure handle
 *    arg2          : uint16_t i2c_address, the slave address, 7 or bit specified in config function
 *    arg3          : uint8_t bytes[], the byte array to send
 *    arg4          : uint32_t byteCount, the byte count to transmit
 * Created by       : Owais
 * Date created     : 14-JUN-2018
 * Description      : Writes an array of bytes to an I2C device, not assuming any register addresses
 * Notes            :
 *****************************************************************************/
HAL_StatusTypeDef I2C_Write_Raw_Message(I2C_HandleTypeDef *hi2c, uint16_t i2c_address, uint8_t bytes[], uint32_t byteCount)
{
	HAL_StatusTypeDef halres;
	halres = HAL_I2C_Master_Transmit(hi2c, i2c_address, bytes, byteCount, TIMEOUT_WRITE_DEFAULT);
	return halres;
}
/*****************************************************************************
 * Function name    : I2C_Write_Reg_Device_ByteArray
 *    returns       : HAL_StatusTypeDef
 *    arg1          : I2C_HandleTypeDef *hi2c, the i2c interface hardware structure handle
 *    arg2          : StrI2CDeviceInfo* strDeviceInterest, device of interest details to use when transmitting
 *    arg3          : uint8_t bytes[], the byte array to send
 *    arg4          : uint32_t byteCount, the byte count to transmit
 * Created by       : Owais
 * Date created     : 14-JUN-2018
 * Description      : Writes to a starting register address on a specific i2c device
 * Notes            :
 *****************************************************************************/
HAL_StatusTypeDef I2C_Write_Reg_Device_ByteArray(I2C_HandleTypeDef *hi2c, StrI2CDeviceInfo* strDeviceInterest, uint8_t bytes[], uint32_t byteCount)
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
			                   TIMEOUT_WRITE_DEFAULT);

	return halres;
}
/*****************************************************************************
 * Function name    : I2C_Write_Reg_ByteArray
 *    returns       : HAL_StatusTypeDef
 *    arg1          : I2C_HandleTypeDef *hi2c, the i2c interface hardware structure handle
 *    arg2          : uint32_t memWidth, either I2C_MEMADD_SIZE_8BIT or I2C_MEMADD_SIZE_16BIT
 *    arg3          : uint16_t devAddress, the device I2C address, 7 or 10 bit depends on i2c config
 *    arg4          : uint16_t regStartAddress, the starting address where writing will being in a loop
 *    arg5          : uint8_t bytes[], the byte array to write
 *    arg6          : uint32_t byteCount, the count of bytes to write
 * Created by       : Owais
 * Date created     : 14-JUN-2018
 * Description      : Writes to a starting register address on a specific i2c device
 * Notes            :
 *****************************************************************************/
HAL_StatusTypeDef I2C_Write_Reg_ByteArray(I2C_HandleTypeDef *hi2c, uint32_t memWidth, uint16_t devAddress, uint16_t regStartAddress, uint8_t bytes[], uint32_t byteCount)
{
	HAL_StatusTypeDef halres;

	halres = HAL_I2C_Mem_Write(hi2c,
			                   devAddress,
							   regStartAddress,
							   memWidth,
			                   bytes,
			                   byteCount,
			                   TIMEOUT_WRITE_DEFAULT);

	return halres;
}
/*****************************************************************************
 * Function name    : I2C_Write_Reg_Byte
 *    returns       : HAL_StatusTypeDef
 *    arg1          : I2C_HandleTypeDef *hi2c, the i2c interface hardware structure handle
 *    arg2          : uint32_t memWidth, either I2C_MEMADD_SIZE_8BIT or I2C_MEMADD_SIZE_16BIT
 *    arg3          : uint16_t devAddress, the device I2C address, 7 or 10 bit depends on i2c config
 *    arg4          : uint16_t regStartAddress, the starting address where writing will being in a loop
 *    arg5          : uint8_t wr_byte, the byte to write
 * Created by       : Owais
 * Date created     : 14-JUN-2018
 * Description      : Writes to a single register on the i2c device with regAddress
 * Notes            :
 *****************************************************************************/
HAL_StatusTypeDef I2C_Write_Reg_Byte(I2C_HandleTypeDef *hi2c, uint32_t memWidth, uint16_t devAddress, uint16_t regAddress, uint8_t wr_byte)
{
	HAL_StatusTypeDef halres;
    uint8_t wrarr[1];
    wrarr[0] = wr_byte;

	halres = HAL_I2C_Mem_Write(hi2c,
			                   devAddress,
							   regAddress,
							   memWidth,
							   wrarr,
			                   1,
			                   TIMEOUT_WRITE_DEFAULT);

	return halres;
}
/*****************************************************************************
 * Function name    : I2C_Write_Reg_Set_Bits
 *    returns       : HAL_StatusTypeDef
 *    arg1          : I2C_HandleTypeDef *hi2c, the i2c interface hardware structure handle
 *    arg2          : uint32_t memWidth, either I2C_MEMADD_SIZE_8BIT or I2C_MEMADD_SIZE_16BIT
 *    arg3          : uint16_t devAddress, the device I2C address, 7 or 10 bit depends on i2c config
 *    arg4          : uint16_t regStartAddress, the starting address where writing will being in a loop
 *    arg5          : uint8_t uint8_t or_mask, the bits set in this byte will be set in the word read
 * Created by       : Owais
 * Date created     : 14-JUN-2018
 * Description      : Read, modify with OR mask and write to device the register regAddress
 * Notes            :
 *****************************************************************************/
HAL_StatusTypeDef I2C_Write_Reg_Set_Bits(I2C_HandleTypeDef *hi2c, uint32_t memWidth, uint16_t devAddress, uint16_t regAddress, uint8_t or_mask)
{
	HAL_StatusTypeDef res;
	uint8_t buffer[1] = { 0 };
	HAL_I2C_Mem_Read(hi2c, devAddress, regAddress, I2C_MEMADD_SIZE_8BIT, buffer, 1, TIMEOUT_WRITE_DEFAULT);
	buffer[0] |= or_mask;
	res = HAL_I2C_Mem_Write(hi2c, devAddress, regAddress, I2C_MEMADD_SIZE_8BIT, buffer, 1, TIMEOUT_READ_DEFAULT);
	return res;
}
/*****************************************************************************
 * Function name    : I2C_Write_Reg_Clear_Bits
 *    returns       : HAL_StatusTypeDef
 *    arg1          : I2C_HandleTypeDef *hi2c, the i2c interface hardware structure handle
 *    arg2          : uint32_t memWidth, either I2C_MEMADD_SIZE_8BIT or I2C_MEMADD_SIZE_16BIT
 *    arg3          : uint16_t devAddress, the device I2C address, 7 or 10 bit depends on i2c config
 *    arg4          : uint16_t regStartAddress, the starting address where writing will being in a loop
 *    arg5          : uint8_t uint8_t clr_mask, the bits set will be cleared in this byte in the target register
 * Created by       : Owais
 * Date created     : 14-JUN-2018
 * Description      : Read, modify write register, clear the bits in the clr_mask which are set
 * Notes            :
 *****************************************************************************/
HAL_StatusTypeDef I2C_Write_Reg_Clear_Bits(I2C_HandleTypeDef *hi2c, uint32_t memWidth, uint16_t devAddress, uint16_t regAddress, uint8_t clr_mask)
{
	HAL_StatusTypeDef res;
	uint8_t buffer[1] = { 0 };
	HAL_I2C_Mem_Read(hi2c, devAddress, regAddress, I2C_MEMADD_SIZE_8BIT, buffer, 1, TIMEOUT_WRITE_DEFAULT);
	buffer[0] &= ~clr_mask;
	res = HAL_I2C_Mem_Write(hi2c, devAddress, regAddress, I2C_MEMADD_SIZE_8BIT, buffer, 1, TIMEOUT_READ_DEFAULT);
	return res;
}
/*****************************************************************************
 * Function name    : I2C_Read_Reg_Byte
 *    returns       : HAL_StatusTypeDef
 *    arg1          : I2C_HandleTypeDef *hi2c, the i2c interface hardware structure handle
 *    arg2          : uint32_t memWidth, either I2C_MEMADD_SIZE_8BIT or I2C_MEMADD_SIZE_16BIT
 *    arg3          : uint16_t devAddress, the device I2C address, 7 or 10 bit depends on i2c config
 *    arg4          : uint16_t regStartAddress, the starting address where writing will being in a loop
 *    arg5          : uint8_t* rd_byte, the address of byte to read in to
 * Created by       : Owais
 * Date created     : 14-JUN-2018
 * Description      : Reads a single register on the i2c device with regAddress
 * Notes            :
 *****************************************************************************/
HAL_StatusTypeDef I2C_Read_Reg_Byte(I2C_HandleTypeDef *hi2c, uint32_t memWidth, uint16_t devAddress, uint16_t regAddress, uint8_t* rd_byte)
{
	HAL_StatusTypeDef halres;

	halres = HAL_I2C_Mem_Read(hi2c,
			                   devAddress,
							   regAddress,
							   memWidth,
							   rd_byte,
			                   1,
							   TIMEOUT_READ_DEFAULT);

	return halres;
}
/*****************************************************************************
 * Function name    : I2C_Read_Devices_Registers
 *    returns       : bool, true if request was successful for all devices
 *    arg1          : I2C_HandleTypeDef *hi2c, the i2c interface hardware structure handle
 *    arg2          : StrI2CDeviceInfo strI2cDev[], array of devices whose byte arrays need to be populated, based in initialized information
 * Created by       : Owais
 * Date created     : 14-JUN-2018
 * Description      : Uses all stored addresses to accomplish reading of starting registers up to specified count
 * Notes            :
 *****************************************************************************/
bool I2C_Read_Devices_Registers(I2C_HandleTypeDef *hi2c, StrI2CDeviceInfo strI2cDev[], uint32_t count)
{
	bool result = true;
	HAL_StatusTypeDef halres;
	uint32_t memSizeControlWord;

	for(uint32_t i=0; i < count; i++)
	{
		if(strI2cDev[i].Is8BitRegisters == true)
			memSizeControlWord = I2C_MEMADD_SIZE_8BIT;
		else
			memSizeControlWord = I2C_MEMADD_SIZE_16BIT;

		halres = HAL_I2C_Mem_Read(hi2c,
				                  strI2cDev[i].DeviceAddress,
				                  strI2cDev[i].StartAddress,
								  memSizeControlWord,
								  strI2cDev[i].BytesRead,
								  strI2cDev[i].ByteCount,
				                  TIMEOUT_READ_DEFAULT);

		if(halres != HAL_OK)
		{
			result = false;
			break;
		}
	}
	return result;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
