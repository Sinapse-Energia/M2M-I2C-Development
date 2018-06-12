/*****************************************************************************
 * Module name: PCAL6416A.c
 *
 * First written on May 28, 2018 by Owais.
 *
 * Module Description:
 * PCAL6416A low level brief drivers
 *
 * The INT output has an open-drain structure and requires pull-up resistor to VDD(P) or
 * VDD(I2C-bus), depending on the application. INT should be connected to the voltage source
 * of the device that requires the interrupt information
 *
 * Updates:
 *****************************************************************************/
#include "PCAL6416A.h"
/*****************************************************************************
 *  global module variables
 *****************************************************************************/

/*****************************************************************************
 *  Function Prototypes
 *****************************************************************************/

/*****************************************************************************
 * Function name    : PCAL6416A_SetupRefs
 *    returns       : void
 *    arg1          : StrPCAL6416A* strDev, the StrPCAL6416A descriptor
 *    arg2          : I2C_HandleTypeDef* i2c, The i2c interface to be used
 *    arg3          : uint8_t addr, i2c address (not including the r/w bit)
 * Created by       : Owais
 * Date created     : 28-May-2018
 * Description      : Setup context
 * Notes            :
 *****************************************************************************/
void PCAL6416A_SetupRefs(StrPCAL6416A* strDev, I2C_HandleTypeDef* i2c, uint8_t addr)
{
	strDev->i2c_dev = i2c;
	strDev->addr = addr;
}
/*****************************************************************************
 * Function name    : PCAL6416A_ReadRegister
 *    returns       : uint8_t, the register value
 *    arg1          : StrPCAL6416A* strDev, the StrPCAL6416A descriptor
 * Created by       : Owais
 * Date created     : 22-May-2018
 * Description      : Read the register value and return its 8-bit value
 * Notes            :
 *****************************************************************************/
uint8_t PCAL6416A_ReadRegister(StrPCAL6416A* dev, uint8_t reg_addr)
{
	uint8_t buffer[1] = { 0 };
	HAL_I2C_Mem_Read(dev->i2c_dev, dev->addr, reg_addr, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);
	return buffer[0];
}
/*****************************************************************************
 * Function name    : PCAL6416A_WriteRegister
 *    returns       : uint8_t, the register value
 *    arg1          : StrPCAL6416A* strDev, the PCAL6416A descriptor
 *    arg2          : uint8_t value, the register value
 * Created by       : Owais
 * Date created     : 22-May-2018
 * Description      : Read the register value and return its 8-bit value
 * Notes            :
 *****************************************************************************/
void PCAL6416A_WriteRegister(StrPCAL6416A* dev, uint8_t reg_addr, uint8_t value)
{
	uint8_t buffer[1] = { 0 };
	buffer[0] = value;
	HAL_I2C_Mem_Write(dev->i2c_dev, dev->addr, reg_addr, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);
}
/*****************************************************************************
 * Function name    : PCAL6416A_MakePinOutput
 *    returns       : bool
 *    arg1          : StrPCAL6416A* strDev, the PCAL6416A descriptor
 *    arg2          : uint8_t banknumber, the bank 0 or 1 (range 0 or 1)
 *    arg3          : uint8_t bitmask, the bitmask e,g 0x80 for bit number 7
 * Created by       : Owais
 * Date created     : 28-May-2018
 * Description      : Configure a bit as output type
 * Notes            :
 *****************************************************************************/
bool PCAL6416A_MakePinOutput(StrPCAL6416A* dev, uint8_t banknumber, uint8_t bitmask)
{
	bool result = false;
	HAL_StatusTypeDef halres;
	uint8_t buffer[10] = { 0 };

	/*test*/
	halres = HAL_I2C_Mem_Read(dev->i2c_dev, dev->addr, 0, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100);

	if(banknumber <= 1)
	{
		halres = HAL_I2C_Mem_Read(dev->i2c_dev, dev->addr, PCAL6416A_REG_ConfigurationPort0 + banknumber, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);
		/* clear bit at bit-mask */
		buffer[0] = buffer[0] & ~bitmask;
		halres = HAL_I2C_Mem_Write(dev->i2c_dev, dev->addr, PCAL6416A_REG_ConfigurationPort0 + banknumber, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);
	}

	if(halres == HAL_OK)
		result = true;
	return result;
}
/*****************************************************************************
 * Function name    : PCAL6416A_MakePinOutput
 *    returns       : bool
 *    arg1          : StrPCAL6416A* strDev, the PCAL6416A descriptor
 *    arg2          : uint8_t banknumber, the bank 0 or 1 (range 0 or 1)
 *    arg3          : uint8_t bitmask, the bitmask e,g 0x80 for bit number 7
 *    arg4          : bool pinvalue, new pin value
 * Created by       : Owais
 * Date created     : 28-May-2018
 * Description      : Set a pin value
 * Notes            : Read/modify/write the output portx register
 *****************************************************************************/
bool PCAL6416A_WritePinValue(StrPCAL6416A* dev, uint8_t banknumber, uint8_t bitmask, bool pinvalue)
{
	bool result = false;
	HAL_StatusTypeDef halres;
	uint8_t buffer[2] = { 0 };
	uint8_t outputPortAdd;

	if(banknumber == 0)
		outputPortAdd = PCAL6416A_REG_OutoutPort0;
	else
		outputPortAdd = PCAL6416A_REG_OutoutPort1;

	halres = HAL_I2C_Mem_Read(dev->i2c_dev, dev->addr, outputPortAdd, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);

	/* set pin value at bit position */
	if(pinvalue == false)
		buffer[0] = buffer[0] & ~bitmask;
	else
		buffer[0] = buffer[0] | bitmask;

	halres = HAL_I2C_Mem_Write(dev->i2c_dev, dev->addr, outputPortAdd, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);

	if(halres == HAL_OK)
		result = true;
	return result;
}
/*****************************************************************************
 * Function name    : PCAL6416A_MakePinInput
 *    returns       : bool
 *    arg1          : StrPCAL6416A* strDev, the PCAL6416A descriptor
 *    arg2          : uint8_t banknumber, the bank 0 or 1 (range 0 or 1)
 *    arg3          : uint8_t bitmask, the bitmask e,g 0x80 for bit number 7
 * Created by       : Owais
 * Date created     : 09-Jun-2018
 * Description      : Configure a bit as input
 * Notes            :
 *****************************************************************************/
bool PCAL6416A_MakePinInput(StrPCAL6416A* dev, uint8_t banknumber, uint8_t bitmask)
{
	bool result = false;
	HAL_StatusTypeDef halres;
	uint8_t buffer[10] = { 0 };

	if(banknumber <= 1)
	{
		halres = HAL_I2C_Mem_Read(dev->i2c_dev, dev->addr, PCAL6416A_REG_ConfigurationPort0 + banknumber, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);
		/* set bit(s) at bit-mask, do read modify write operation */
		buffer[0] = buffer[0] | bitmask;
		halres = HAL_I2C_Mem_Write(dev->i2c_dev, dev->addr, PCAL6416A_REG_ConfigurationPort0 + banknumber, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);
	}

	if(halres == HAL_OK)
		result = true;

	return result;
}
/*****************************************************************************
 * Function name    : PCAL6416A_EnableIntOnPin
 *    returns       : bool
 *    arg1          : StrPCAL6416A* strDev, the PCAL6416A descriptor
 *    arg2          : uint8_t banknumber, the bank 0 or 1 (range 0 or 1)
 *    arg3          : uint8_t bitmask, the bitmask e,g 0x80 for bit number 7
 * Created by       : Owais
 * Date created     : 09-Jun-2018
 * Description      : Enable interrupt on pin by clearing bit
 * Notes            : Interrupt mask registers are set to logic 1 upon power-on, disabling interrupts during
                      system start-up.
                      Interrupts may be enabled by setting corresponding mask bits to logic 0.

                      If an input changes state and the corresponding bit in the Interrupt mask register is set
                      to 1, the interrupt is masked and the interrupt pin will not be asserted. If the corresponding
                      bit in the Interrupt mask register is set to 0, the interrupt pin will be asserted.
 *****************************************************************************/
bool PCAL6416A_EnableIntOnPin(StrPCAL6416A* dev, uint8_t banknumber, uint8_t bitmask)
{
	bool result = false;
	HAL_StatusTypeDef halres;
	uint8_t buffer[10] = { 0 };

	if(banknumber <= 1)
	{
		halres = HAL_I2C_Mem_Read(dev->i2c_dev, dev->addr, PCAL6416A_REG_InterruptMask0 + banknumber, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);
		/* clear bit(s) at bit-mask, do read modify write operation */
		buffer[0] = buffer[0] & ~bitmask;
		halres = HAL_I2C_Mem_Write(dev->i2c_dev, dev->addr, PCAL6416A_REG_InterruptMask0 + banknumber, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);
	}

	if(halres == HAL_OK)
		result = true;

	return result;
}
/*****************************************************************************
 * Function name    : PCAL6416A_GetInterruptStatus
 *    returns       : bool
 *    arg1          : StrPCAL6416A* strDev, the PCAL6416A descriptor
 *    arg2          : uint8_t* bank0Status, the status bye of bank 0, if any bit is 1 the interrupt is asserted
 *    arg3          : uint8_t* bank1Status
 * Created by       : Owais
 * Date created     : 09-Jun-2018
 * Description      : Gets which pin caused interrupt on the device, read the two(2x) interrupt status registers
 * Notes            : A logic 1 indicates that the corresponding input pin was the source of the interrupt. A logic 0
                      indicates that the input pin is not the source of an interrupt.
 *****************************************************************************/
bool PCAL6416A_GetInterruptStatus(StrPCAL6416A* dev, uint8_t* bank0Status, uint8_t* bank1Status)
{
	bool result = false;
	HAL_StatusTypeDef halres;
	uint8_t buffer[10] = { 0 };

	halres = HAL_I2C_Mem_Read(dev->i2c_dev, dev->addr, PCAL6416A_REG_InterruptStatus0, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);

	if(halres == HAL_OK)
	{
		*bank0Status = buffer[0];
		*bank1Status = buffer[1];
		result = true;
	}

	return result;
}
