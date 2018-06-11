/*******************************************************************************
 * Module name: PCAL6416A
 *
 * First written on May 28, 2018. by Owais
 *
 * Module Description:
 * Header for PCAL6416A C module
 *
 *
 *******************************************************************************/
#ifndef PCAL6416A_H_
#define PCAL6416A_H_
/*******************************************************************************
 *  Include section
 *******************************************************************************/
#include "stm32f2xx_hal.h"
#include "stdint.h"
#include "stdbool.h"
/*******************************************************************************
 *  #defines section
 *******************************************************************************/
#define PCAL6416A_ADDR_LOW                      (0x20<<1) /*0x40*/
#define PCAL6416A_ADDR_HIGH                     (0x21<<1) /*0x42*/

#define PCAL6416A_REG_InputPort0                0x00
#define PCAL6416A_REG_InputPort1                0x01
#define PCAL6416A_REG_OutoutPort0               0x02
#define PCAL6416A_REG_OutoutPort1               0x03
#define PCAL6416A_REG_PolarityInversionPort0    0x04
#define PCAL6416A_REG_PolarityInversionPort1    0x05
#define PCAL6416A_REG_ConfigurationPort0        0x06
#define PCAL6416A_REG_ConfigurationPort1        0x07
#define PCAL6416A_REG_OutputDriveStrength0_0    0x40
#define PCAL6416A_REG_OutputDriveStrength0_1    0x41
#define PCAL6416A_REG_OutputDriveStrength1_0    0x42
#define PCAL6416A_REG_OutputDriveStrength1_1    0x43
#define PCAL6416A_REG_InputLatch0               0x44
#define PCAL6416A_REG_InputLatch1               0x45
#define PCAL6416A_REG_PullUpPullDowmEnable0     0x46
#define PCAL6416A_REG_PullUpPullDowmEnable1     0x47
#define PCAL6416A_REG_PullUpPullDowmSelection0  0x48
#define PCAL6416A_REG_PullUpPullDowmSelection1  0x49
#define PCAL6416A_REG_InterruptMask0            0x4a
#define PCAL6416A_REG_InterruptMask1            0x4b
#define PCAL6416A_REG_InterruptStatus0          0x4c
#define PCAL6416A_REG_InterruptStatus1          0x4d
#define PCAL6416A_REG_OutputPortConfiguration   0x4F

typedef struct
{
	I2C_HandleTypeDef* i2c_dev;
	uint8_t addr;
} StrPCAL6416A;
/*******************************************************************************
 *  global variables
 *******************************************************************************/

/*******************************************************************************
 *  global functions (public)
 *******************************************************************************/
void PCAL6416A_SetupRefs(StrPCAL6416A* strDev, I2C_HandleTypeDef* i2c, uint8_t addr);
uint8_t PCAL6416A_ReadRegister(StrPCAL6416A* dev, uint8_t reg_addr);
void PCAL6416A_WriteRegister(StrPCAL6416A* dev, uint8_t reg_addr, uint8_t value);
bool PCAL6416A_MakePinOutput(StrPCAL6416A* dev, uint8_t banknumber, uint8_t bitmask);
bool PCAL6416A_WritePinValue(StrPCAL6416A* dev, uint8_t banknumber, uint8_t bitmask, bool pinvalue);
bool PCAL6416A_MakePinInput(StrPCAL6416A* dev, uint8_t banknumber, uint8_t bitmask);
bool PCAL6416A_EnableIntOnPin(StrPCAL6416A* dev, uint8_t banknumber, uint8_t bitmask);
bool PCAL6416A_GetInterruptStatus(StrPCAL6416A* dev, uint8_t* bank0Status, uint8_t* bank1Status);

#endif /* PCAL6416A_H_ */
