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
 * Modified on JUN 14, 2018 to use L1 I2C functions
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
	HAL_StatusTypeDef halres = HAL_ERROR;

	if(banknumber <= 1)
	{
		halres = I2C_Write_Reg_Clear_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, PCAL6416A_REG_ConfigurationPort0 + banknumber, bitmask);
	}

	if(halres == HAL_OK)
		result = true;
	return result;
}
/*****************************************************************************
 * Function name    : PCAL6416A_WritePinValue
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
	uint8_t buffer[1] = { 0 };
	uint8_t outputPortAdd;

	if(banknumber == 0)
		outputPortAdd = PCAL6416A_REG_OutoutPort0;
	else
		outputPortAdd = PCAL6416A_REG_OutoutPort1;

	halres = I2C_Read_Reg_Byte(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, outputPortAdd, buffer);

	/* set pin value at bit position */
	if(pinvalue == false)
		buffer[0] = buffer[0] & ~bitmask;
	else
		buffer[0] = buffer[0] | bitmask;

	halres = I2C_Write_Reg_Byte(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, outputPortAdd, buffer[0]);

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
 * Notes            : If a bit in these registers is set to 1, the corresponding port pin is enabled as a
                      high-impedance input.
                      If a bit in these registers is cleared to 0, the corresponding port pin
                      is enabled as an output.
 *****************************************************************************/
bool PCAL6416A_MakePinInput(StrPCAL6416A* dev, uint8_t banknumber, uint8_t bitmask)
{
	bool result = false;
	HAL_StatusTypeDef halres = HAL_ERROR;

	if(banknumber <= 1)
	{
		halres = I2C_Write_Reg_Set_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, PCAL6416A_REG_ConfigurationPort0 + banknumber, bitmask);
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
	HAL_StatusTypeDef halres = HAL_ERROR;

	if(banknumber <= 1)
	{
		halres = I2C_Write_Reg_Clear_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, PCAL6416A_REG_InterruptMask0 + banknumber, bitmask);
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
	HAL_StatusTypeDef halres = HAL_ERROR;
	uint8_t buffer[2] = { 0 };

	halres = I2C_Read_Reg_Byte(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, PCAL6416A_REG_InterruptStatus0, &buffer[0]);
	halres = I2C_Read_Reg_Byte(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, PCAL6416A_REG_InterruptStatus1, &buffer[1]);

	if(halres == HAL_OK)
	{
		*bank0Status = buffer[0];
		*bank1Status = buffer[1];
		result = true;
	}

	return result;
}
