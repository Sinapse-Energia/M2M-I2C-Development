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
	uint8_t rd_buffer[1] = { 0 };
	uint8_t wr_buffer[1];

	/* Read WHO AM I */
	halres = I2C_Read_Reg_Byte(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_WHO_AM_I, rd_buffer);

	if(rd_buffer[0] != 0x69)
		result = false;
	else
	{
		if(enOp == En_Mode_ACC || enOp == En_Mode_Both)
		{
			/* Configure ACC */
			/* CTRL1_XL (10h) */
			/* Don't disturb BW_XL bits for now to change analog LPF bandwidth and leave it at auto selection */
			wr_buffer[0] = enPow << 4 | scaleXL << 2;
			halres = I2C_Write_Reg_Byte(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_CTRL1_XL, wr_buffer[0]);
		}

		if(enOp == En_Mode_Gyro || enOp == En_Mode_Both)
		{
			/* Configure GYRO */
			/* CTRL2_G (11h) */
			wr_buffer[0] = enPow << 4 | scaleG << 2;
			halres = I2C_Write_Reg_Byte(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_CTRL2_G, wr_buffer[0]);
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
	dev->bDetectFreeFall = ff;
	dev->bDetectWakeup = wu;
	dev->bDetectSleep = sl;
	dev->bDetectMotion = md;
	dev->bDetectStep = sd;

	if(ff || wu || sl || md || sd)
	{
		if((dev->enOPMode == En_Mode_ACC || dev->enOPMode == En_Mode_Both) && (dev->enPowerMode >= En_Pow_HIGHPERF_416Hz) )
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

			I2C_Write_Reg_Byte(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_WAKE_UP_DUR, 0x00);
			I2C_Write_Reg_Byte(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_FREE_FALL, 0x33);

			I2C_Write_Reg_Set_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_MD1_CFG,  0x10);
			I2C_Write_Reg_Set_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_TAP_CFG, 0x01);
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

			I2C_Write_Reg_Set_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_WAKE_UP_THS, 0x02);
			I2C_Write_Reg_Set_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_MD1_CFG, 0x20);
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

			I2C_Write_Reg_Set_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_WAKE_UP_DUR, 0x02);
			I2C_Write_Reg_Set_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_WAKE_UP_THS, 0x42);
			I2C_Write_Reg_Set_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_MD1_CFG, 0x80);
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

			I2C_Write_Reg_Clear_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_TAP_CFG, 0x40);
			I2C_Write_Reg_Set_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_CTRL10_C, 0x3D);
			I2C_Write_Reg_Set_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_TAP_CFG, 0x40);
			I2C_Write_Reg_Set_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_INT1_CTRL, 0xC0);
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

			I2C_Write_Reg_Set_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_CTRL10_C, 0x3c);
			I2C_Write_Reg_Set_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_TAP_CFG, 0x40);
			I2C_Write_Reg_Set_Bits(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_INT1_CTRL, 0x80);
		}
	}

	return bResult;
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
void LSM6DS3_ProcessISR(StrLSM6DS3* dev)
{
	/* if resultCode == 0b11111, it means all five events were detected */
	/* if resultCode == 0b00001, only FF was detected */

	uint8_t resultCode = 0;
	uint8_t rdRegVal_WUS;
	uint8_t rdRegVal_FS;
	EnumCQErrors cqErr;
	uint8_t rd_buffer_WUS[1];
	uint8_t rd_buffer_FS[1];

	I2C_Read_Reg_Byte(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_WAKE_UP_SRC, rd_buffer_WUS);
	I2C_Read_Reg_Byte(dev->i2c_dev, I2C_MEMADD_SIZE_8BIT, dev->addr, LSM6DS3_FUNC_SRC, rd_buffer_FS);

	rdRegVal_WUS = rd_buffer_WUS[0];
	rdRegVal_FS = rd_buffer_FS[0];

	if(dev->bDetectFreeFall)
	{
		if( rdRegVal_WUS & 0x20)
			resultCode |= MASK_EVENT_FF;
	}

	if(dev->bDetectWakeup)
	{
		if( rdRegVal_WUS & 0x8)
			resultCode |= MASK_EVENT_WU;
	}

	if(dev->bDetectSleep)
	{
		if( rdRegVal_WUS & 0x10)
			resultCode |= MASK_EVENT_SL;
	}

	if(dev->bDetectMotion)
	{
		if( rdRegVal_FS & 0x40)
			resultCode |= MASK_EVENT_MD;
	}

	if(dev->bDetectStep)
	{
		if( rdRegVal_FS & 0x10)
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
