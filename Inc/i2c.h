/**
  ******************************************************************************
  * File Name          : I2C.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"
#include "main.h"
#include "stdbool.h"
#include "stdint.h"

/* USER CODE BEGIN Includes */
#define TIMEOUT_WRITE_DEFAULT 100
#define TIMEOUT_READ_DEFAULT  100
/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */
#define MAX_BYTES_TO_READ   256
#define MAX_DEVICES          20
typedef struct
{
	uint16_t DeviceAddress;
	uint16_t StartAddress;
	uint16_t ByteCount;
	uint8_t  BytesRead[MAX_BYTES_TO_READ];
	bool     Is8BitRegisters; /* TRUE if 8-bit registers */
}StrI2CDeviceInfo; /* a structure which helps us remember an i2c device general properties and used to populate BytesRead array with registers contents */
/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

/* USER CODE BEGIN Prototypes */
bool              I2C_Config(I2C_HandleTypeDef *hi2c, uint32_t baudRate, bool bTenBitAddress);
HAL_StatusTypeDef I2C_Write_Raw_Message(I2C_HandleTypeDef *hi2c, uint16_t i2c_address, uint8_t bytes[], uint32_t byteCount);
HAL_StatusTypeDef I2C_Write_Reg_Device_ByteArray(I2C_HandleTypeDef *hi2c, StrI2CDeviceInfo* strDeviceInterest, uint8_t bytes[], uint32_t byteCount);
HAL_StatusTypeDef I2C_Write_Reg_ByteArray(I2C_HandleTypeDef *hi2c, uint32_t memWidth, uint16_t devAddress, uint16_t regStartAddress, uint8_t bytes[], uint32_t byteCount);
HAL_StatusTypeDef I2C_Write_Reg_Byte(I2C_HandleTypeDef *hi2c, uint32_t memWidth, uint16_t devAddress, uint16_t regAddress, uint8_t wr_byte);
HAL_StatusTypeDef I2C_Write_Reg_Set_Bits(I2C_HandleTypeDef *hi2c, uint32_t memWidth, uint16_t devAddress, uint16_t regAddress, uint8_t or_mask);
HAL_StatusTypeDef I2C_Write_Reg_Clear_Bits(I2C_HandleTypeDef *hi2c, uint32_t memWidth, uint16_t devAddress, uint16_t regAddress, uint8_t clr_mask);
HAL_StatusTypeDef I2C_Read_Reg_Byte(I2C_HandleTypeDef *hi2c, uint32_t memWidth, uint16_t devAddress, uint16_t regAddress, uint8_t* rd_byte);
bool              I2C_Read_Devices_Registers(I2C_HandleTypeDef *hi2c, StrI2CDeviceInfo strI2cDev[], uint32_t count);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
