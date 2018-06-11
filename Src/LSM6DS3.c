/*****************************************************************************
 * Module name: LSM6DS3.c
 *
 * First written on May 20, 2018 by Owais.
 *
 * Module Description:
 * LSM6DS3 low level brief drivers
 *
 * SDO/SA0 is connected to GND
 * INT2 connected to MCU (pin PB0)
 * INT1 is connected in a GPIO expander (PCAL6416A, connected in pin P1.4)
 * Interrupt signal is active high by default
 *
 * Updates:
 *****************************************************************************/
#include <LSM6DS3.h>
/*****************************************************************************
 *  global module variables
 *****************************************************************************/
CircularQueue CQ_LSM6DS3_EventCodes;
uint8_t ArrEventCodes[CNT_MAX_EVENTS];
uint32_t CntEnqEventsOK = 0;
uint32_t CntEnqEventsFail = 0;
/*****************************************************************************
 *  Function Prototypes
 *****************************************************************************/
uint8_t ReadRegister(StrLSM6DS3* dev, uint8_t reg_addr);
void WriteRegister(StrLSM6DS3* dev, uint8_t reg_addr, uint8_t ORvalue);
void WriteORValueRegister(StrLSM6DS3* dev, uint8_t reg_addr, uint8_t ORvalue);
/*****************************************************************************
 * Function name    : LSM6DS3_SetupRefs
 *    returns       : void
 *    arg1          : StrLSM6DS3* strDev, the LSM6DS3 descriptor
 *    arg2          : I2C_HandleTypeDef* i2c, The i2c interface to be used
 *    arg3          : uint8_t addr, i2c address (not including the r/w bit)
 * Created by       : Owais
 * Date created     : 30-May-2018
 * Description      : Setup LSM6DS3 context
 * Notes            :
 *****************************************************************************/
void LSM6DS3_SetupRefs(StrLSM6DS3* strDev, I2C_HandleTypeDef* i2c, uint8_t addr)
{
	strDev->i2c_dev = i2c;
	strDev->addr = addr;
}
/*****************************************************************************
 * Function name    : LSM6DS3_Config
 *    returns       : bool, true if config. was successful
 *    arg1          : StrLSM6DS3* strDev, the LSM6DS3 descriptor
 *    arg2          : EnumOperatingMode enOpMode, ACC, GYRO or both
 *    arg3          : EnumPowerMode enPowMode, low power, normal or high performance mode
 *    arg4          : uint8_t scaleXL, either 0,1,2,3 corresponding to (±2/±16/±4/±8)g
 *    arg5          : uint8_t scaleG, either 0,1,2,3 corresponding to (±250/±500/±1000/±2000)dps
 * Created by       : Owais
 * Date created     : 10-JUN-2018
 * Description      : Setup LSM6DS3 Configuration
 * Notes            : We don't need to access register banks A and B for this application
 *****************************************************************************/
bool LSM6DS3_Config(StrLSM6DS3* dev, EnumOperatingMode enOp, EnumPowerMode enPow, uint8_t scaleXL, uint8_t scaleG)
{
	HAL_StatusTypeDef halres;
	bool result = false;
	dev->enOPMode = enOp;
	dev->enPowerMode = enPow;

	/* Check if device is responsive (WHO am i register) */
	uint8_t buffer[1] = { 0 };
	uint8_t wr_bytes[1];

	/* Read WHO AM I */
	halres = HAL_I2C_Mem_Read(dev->i2c_dev, dev->addr, LSM6DSM_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);

	if(buffer[0] != 0x69)
		result = false;
	else
	{
		if(enOp == En_Mode_ACC || enOp == En_Mode_Both)
		{
			/* Configure ACC */
			/* CTRL1_XL (10h) */
			/* Don't disturb BW_XL bits for now to change analog LPF bandwidth and leave it at auto selection */
			wr_bytes[0] = enPow << 4 | scaleXL << 2;
			halres = HAL_I2C_Mem_Write(dev->i2c_dev, dev->addr, LSM6DSM_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, wr_bytes, 1, 10);
		}

		if(enOp == En_Mode_Gyro || enOp == En_Mode_Both)
		{
			/* Configure GYRO */
			/* CTRL2_G (11h) */
			wr_bytes[0] = enPow << 4 | scaleG << 2;
			halres = HAL_I2C_Mem_Write(dev->i2c_dev, dev->addr, LSM6DSM_CTRL2_G, I2C_MEMADD_SIZE_8BIT, wr_bytes, 1, 10);
		}

		if(halres == HAL_OK)
			result = true;

		CQ_Init(&CQ_LSM6DS3_EventCodes, &ArrEventCodes, ( sizeof (ArrEventCodes) / sizeof (ArrEventCodes[0]) ) - 1, sizeof (uint8_t));
	}
	return result;
}
/*****************************************************************************
 * Function name    : LSM6DS3_Set_Events
 *    returns       : bool, true if event detection could be setup
 *    arg1          : StrLSM6DS3* strDev, the LSM6DS3 descriptor
 *    arg2          : bool ff, freefall detect
 *    arg3          : bool wu, wakeup detect
 *    arg4          : bool sl, sleep detect
 *    arg5          : bool md, motion detect
 *    arg6          : bool sd, step detect
 * Created by       : Owais
 * Date created     : 09-JUN-2018
 * Description      : Setup event detection
 * Notes            :
 *****************************************************************************/
bool LSM6DS3_Set_Events(StrLSM6DS3* dev, bool ff, bool wu, bool sl, bool md, bool sd)
{
	bool bResult = false;
	uint8_t regVal;
	dev->bDetectFreeFall = ff;
	dev->bDetectWakeup = wu;
	dev->bDetectSleep = sl;
	dev->bDetectMotion = md;
	dev->bDetectStep = sd;

	if(ff || wu || sl || md || sd)
	{
		if((dev->enOPMode == En_Mode_ACC || dev->enOPMode == En_Mode_Both) && (dev->enPowerMode == En_Pow_HIGHPERF) )
		{
			bResult = true;
		}
		else
		{
			bResult = false;
		}
	}

	if(bResult)
	{
		if(ff)
		{
			/*
			  1 Write 60h into CTRL1_XL // Turn on the accelerometer
                                        // ODR_XL = 416 Hz, FS_XL = ±2 g
              2 Write 00h into WAKE_UP_DUR // Set event duration (FF_DUR5 bit)
              3 Write 33h into FREE_FALL   // Set FF threshold (FF_THS[2:0] = 011b)
                                           // Set six samples event duration (FF_DUR[5:0] = 000110b)
              4 Write 10h into MD1_CFG // FF interrupt driven to INT1 pin
              5 Write 01h into TAP_CFG // Latch interrupt

              WAKE_UP_DUR registers is configured like this to ignore events that are shorter than
              6/ODR_XL = 6/412 Hz ~= 15 msec in order to avoid false detections
			*/

			WriteRegister(dev, LSM6DSM_WAKE_UP_DUR, 0x0);
			WriteRegister(dev, LSM6DSM_FREE_FALL, 0x33);

			regVal = ReadRegister(dev, LSM6DSM_MD1_CFG);
			regVal |= 0x10;
			WriteRegister(dev, LSM6DSM_MD1_CFG, regVal);

			regVal = ReadRegister(dev, LSM6DSM_TAP_CFG);
			regVal |= 0x01;
			WriteRegister(dev, LSM6DSM_TAP_CFG, regVal);
		}

		if(wu)
		{
			/*
			 1 Write 60h into CTRL1_XL  // Turn on the accelerometer
										// ODR_XL = 416 Hz, FS_XL = ±2 g
			 2 Write 10h into TAP_CFG // Apply high-pass digital filter; latch mode disabled (we use slope filter so not needed)
			 3 Write 00h into WAKE_UP_DUR // No duration
			 4 Write 02h into WAKE_UP_THS // Set wake-up threshold
			 5 Write 20h into MD1_CFG // Wake-up interrupt driven to INT1 pin
			 */
			regVal = ReadRegister(dev, LSM6DSM_WAKE_UP_THS);
			regVal |= 0x02;
			WriteRegister(dev, LSM6DSM_WAKE_UP_THS, regVal);

			regVal = ReadRegister(dev, LSM6DSM_MD1_CFG);
			regVal |= 0x20;
			WriteRegister(dev, LSM6DSM_MD1_CFG, regVal);
		}

		if(sl)
		{
			/* Max XL ODR can be 833 Hz */

			/*
			 1 Write 50h into CTRL1_XL // Turn on the accelerometer
                                       // ODR_XL = 208 Hz, FS_XL = ±2 g
             2 Write 02h into WAKE_UP_DUR // Set duration for Inactivity detection
             3 Write 42h into WAKE_UP_THS // Set Activity/Inactivity threshold
                                          // Enable Activity/Inactivity detection
             4 Write 80h into MD1_CFG // Activity/Inactivity interrupt driven to INT1 pin
			 */

			regVal = ReadRegister(dev, LSM6DSM_WAKE_UP_DUR);
			regVal |= 0x02; /* 2*512 ODR, 1024 Hz = 1 second wait */
			WriteRegister(dev, LSM6DSM_WAKE_UP_DUR, regVal);

			regVal = ReadRegister(dev, LSM6DSM_WAKE_UP_THS);
			regVal |= 0x42;
			WriteRegister(dev, LSM6DSM_WAKE_UP_THS, regVal);

			regVal = ReadRegister(dev, LSM6DSM_MD1_CFG);
			regVal |= 0x80;
			WriteRegister(dev, LSM6DSM_MD1_CFG, regVal);
		}

		if(md)
		{
			/* so the accelerometer ODR must be set at a value of 26 Hz or higher. */

			/*
			 1 Write 80h into FUNC_CFG_ADDRESS // Enable access to embedded functions registers
			 2 Write 08h into SM_THS           // Set Significant Motion threshold
		     3 Write 00h into FUNC_CFG_ADDRESS // Disable access to embedded functions registers
			 4 Write 20h into CTRL1_XL // Turn on the accelerometer
									   // ODR_XL = 26 Hz, FS_XL = ±2 g
             5 Write 00h into TAP_CFG  // Disable pedometer
             6 Write 3Dh into CTRL10_C // Enable embedded functions
                                       // Enable Significant Motion detection
             7 Write 40h into TAP_CFG   // Enable pedometer algorithm
             8 Write C0h into INT1_CTRL // Significant motion interrupt driven to INT1
			 */
			regVal = ReadRegister(dev, LSM6DSM_TAP_CFG);
			regVal &= ~0x40; /* Clear bit 6 PEDO_EN */
			WriteRegister(dev, LSM6DSM_TAP_CFG, regVal);

			regVal = ReadRegister(dev, LSM6DSM_CTRL10_C);
			regVal |= 0x3D;
			WriteRegister(dev, LSM6DSM_CTRL10_C, regVal);

			regVal = ReadRegister(dev, LSM6DSM_TAP_CFG);
			regVal |= 0x40; /* enable bit 6 PEDO_EN */
			WriteRegister(dev, LSM6DSM_TAP_CFG, regVal);

			regVal = ReadRegister(dev, LSM6DSM_INT1_CTRL);
			regVal |= 0xC0;
			WriteRegister(dev, LSM6DSM_INT1_CTRL, regVal);
		}

		if(sd)
		{
			/*
			 1 Write 20h into CTRL1_XL // Turn on the accelerometer
                                       // ODR_XL = 26 Hz, FS_XL = ±2 g
             2 Write 3Ch into CTRL10_C // Enable embedded functions
             3 Write 40h into TAP_CFG  // Enable pedometer algorithm
             4 Write 80h into INT1_CTRL // Step Detector interrupt driven to INT1 pin
			 */
			regVal = ReadRegister(dev, LSM6DSM_CTRL10_C);
			regVal |= 0x3C;
			WriteRegister(dev, LSM6DSM_CTRL10_C, regVal);

			regVal = ReadRegister(dev, LSM6DSM_TAP_CFG);
			regVal |= 0x40; /* enable bit 6 PEDO_EN */
			WriteRegister(dev, LSM6DSM_TAP_CFG, regVal);

			regVal = ReadRegister(dev, LSM6DSM_INT1_CTRL);
			regVal |= 0x80;
			WriteRegister(dev, LSM6DSM_INT1_CTRL, regVal);
		}
	}

	return bResult;
}
/*****************************************************************************
 * Function name    : ReadRegister
 *    returns       : uint8_t, the register value
 *    arg1          : StrLSM6DSM* strDev, the LSM6DSM descriptor
 * Created by       : Owais
 * Date created     : 22-May-2018
 * Description      : Read the register value and return its 8-bit value
 * Notes            :
 *****************************************************************************/
uint8_t ReadRegister(StrLSM6DS3* dev, uint8_t reg_addr)
{
	uint8_t buffer[1] = { 0 };
	HAL_I2C_Mem_Read(dev->i2c_dev, dev->addr, reg_addr, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);
	return buffer[0];
}
/*****************************************************************************
 * Function name    : WriteRegister
 *    returns       : uint8_t, the register value
 *    arg1          : StrLSM6DSM* strDev, the LSM6DSM descriptor
 *    arg2          : uint8_t value, the register value
 * Created by       : Owais
 * Date created     : 22-May-2018
 * Description      : Read the register value and return its 8-bit value
 * Notes            :
 *****************************************************************************/
void WriteRegister(StrLSM6DS3* dev, uint8_t reg_addr, uint8_t value)
{
	uint8_t buffer[1] = { 0 };
	buffer[0] = value;
	HAL_I2C_Mem_Write(dev->i2c_dev, dev->addr, reg_addr, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);
}
/*****************************************************************************
 * Function name    : WriteORValueRegister
 *    returns       : uint8_t, the register value
 *    arg1          : StrLSM6DSM* strDev, the LSM6DSM descriptor
 *    arg2          : uint8_t ORvalue, the OR mask value to write
 * Created by       : Owais
 * Date created     : 22-May-2018
 * Description      : Read modify register withy OR mask
 * Notes            :
 *****************************************************************************/
void WriteORValueRegister(StrLSM6DS3* dev, uint8_t reg_addr, uint8_t ORvalue)
{
	uint8_t buffer[1] = { 0 };
	HAL_I2C_Mem_Read(dev->i2c_dev, dev->addr, reg_addr, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);
	buffer[0] |= ORvalue;
	HAL_I2C_Mem_Write(dev->i2c_dev, dev->addr, reg_addr, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);
}
/*****************************************************************************
 * Function name    : ProcessISR
 *    returns       : void
 *    arg1          : StrLSM6DSM* strDev, the LSM6DSM descriptor
 * Created by       : Owais
 * Date created     : 25-May-2018
 * Description      : Function called from ISR (GPIO)
 * Notes            :
 *****************************************************************************/
void ProcessISR(StrLSM6DS3* dev)
{
	/* if resultCode == 0b11111, it means all five events were detected */
	/* if resultCode == 0b00001, only FF was detected */

	uint8_t resultCode = 0;
	uint8_t rdRegVal;
	EnumCQErrors cqErr;

	if(dev->bDetectFreeFall)
	{
		rdRegVal = ReadRegister(dev, LSM6DSM_WAKE_UP_SRC);

		if( rdRegVal & 0x20)
			resultCode |= MASK_EVENT_FF;
	}

	if(dev->bDetectWakeup)
	{
		rdRegVal = ReadRegister(dev, LSM6DSM_WAKE_UP_SRC);

		if( rdRegVal & 0x8)
			resultCode |= MASK_EVENT_WU;
	}

	if(dev->bDetectSleep)
	{
		rdRegVal = ReadRegister(dev, LSM6DSM_WAKE_UP_SRC);

		if( rdRegVal & 0x10)
			resultCode |= MASK_EVENT_SL;
	}

	if(dev->bDetectMotion)
	{
		rdRegVal = ReadRegister(dev, LSM6DSM_FUNC_SRC);

		if( rdRegVal & 0x40)
			resultCode |= MASK_EVENT_MD;
	}

	if(dev->bDetectStep)
	{
		rdRegVal = ReadRegister(dev, LSM6DSM_FUNC_SRC);

		if( rdRegVal & 0x10)
			resultCode |= MASK_EVENT_PD;
	}

	if(resultCode != 0)
	{
		cqErr = CQ_Enqueue(&CQ_LSM6DS3_EventCodes, (void*)&resultCode);

		if(cqErr == CQ_ERROR_OK)
			CntEnqEventsOK++;
		else
			CntEnqEventsFail++;
	}
	else
	{
		/* no important event was detected */
	}
}
/*****************************************************************************
 * Function name    : LSM6DSM_Get_Event
 *    returns       : uint8_t*, the pointer to integer based event message
 *    arg1          : EnumCQErrors* cqErrOut, the error code for function call,
 *                    return value must be checked and is valid only when
 *                    value of cqErrOut is CQ_ERROR_OK
 * Created by       : Owais
 * Date created     : 25-May-2018
 * Description      : Read a message from circular queue in FIFO fashion.
 *                    Must check error code before using result
 * Notes            :
 *****************************************************************************/
uint8_t* LSM6DS3_Get_Event(EnumCQErrors* cqErrOut)
{
	uint8_t* pi = (uint8_t*)CQ_Dequeue(&CQ_LSM6DS3_EventCodes, cqErrOut);
	return pi;
}
