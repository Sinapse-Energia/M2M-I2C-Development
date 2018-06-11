/*******************************************************************************
 * Module name: LSM6DSM
 *
 * First written on May 21, 2018. by Owais
 *
 * Module Description:
 * Header for LSM6DSM C module
 *
 *
 *******************************************************************************/
#ifndef LSM6DS3_H_
#define LSM6DS3_H_
/*******************************************************************************
 *  Include section
 *******************************************************************************/
#include "stm32f2xx_hal.h"
#include "CQ_Gen.h"
#include "stdint.h"
#include "stdbool.h"
/*******************************************************************************
 *  #defines section
 *******************************************************************************/
#define LSM6DSM_ADDR_LOW                  (0x6a<<1)
#define LSM6DSM_ADDR_HIGH                 (0x6b<<1)

#define LSM6DSM_FUNC_CFG_ACCESS           0x01
#define LSM6DSM_SENSOR_SYNC_TIME_FRAME    0x04
#define LSM6DSM_FIFO_CTRL1                0x06
#define LSM6DSM_FIFO_CTRL2                0x07
#define LSM6DSM_FIFO_CTRL3                0x08
#define LSM6DSM_FIFO_CTRL4                0x09
#define LSM6DSM_FIFO_CTRL5                0x0A
#define LSM6DSM_ORIENT_CFG_G              0x0B
#define LSM6DSM_INT1_CTRL                 0x0D
#define LSM6DSM_INT2_CTRL                 0x0E
#define LSM6DSM_WHO_AM_I                  0x0F
#define LSM6DSM_CTRL1_XL                  0x10
#define LSM6DSM_CTRL2_G                   0x11
#define LSM6DSM_CTRL3_C                   0x12
#define LSM6DSM_CTRL4_C                   0x13
#define LSM6DSM_CTRL5_C                   0x14
#define LSM6DSM_CTRL6_C                   0x15
#define LSM6DSM_CTRL7_G                   0x16
#define LSM6DSM_CTRL8_XL                  0x17
#define LSM6DSM_CTRL9_XL                  0x18
#define LSM6DSM_CTRL10_C                  0x19
#define LSM6DSM_MASTER_CONFIG             0x1A
#define LSM6DSM_WAKE_UP_SRC               0x1B
#define LSM6DSM_TAP_SRC                   0x1C
#define LSM6DSM_D6D_SRC                   0x1D
#define LSM6DSM_STATUS_REG                0x1E
#define LSM6DSM_OUT_TEMP_L                0x20
#define LSM6DSM_OUT_TEMP_H                0x21
#define LSM6DSM_OUTX_L_G                  0x22
#define LSM6DSM_OUTX_H_G                  0x23
#define LSM6DSM_OUTY_L_G                  0x24
#define LSM6DSM_OUTY_H_G                  0x25
#define LSM6DSM_OUTZ_L_G                  0x26
#define LSM6DSM_OUTZ_H_G                  0x27
#define LSM6DSM_OUTX_L_XL                 0x28
#define LSM6DSM_OUTX_H_XL                 0x29
#define LSM6DSM_OUTY_L_XL                 0x2A
#define LSM6DSM_OUTY_H_XL                 0x2B
#define LSM6DSM_OUTZ_L_XL                 0x2C
#define LSM6DSM_OUTZ_H_XL                 0x2D
#define LSM6DSM_SENSORHUB1_REG            0x2E
#define LSM6DSM_SENSORHUB2_REG            0x2F
#define LSM6DSM_SENSORHUB3_REG            0x30
#define LSM6DSM_SENSORHUB4_REG            0x31
#define LSM6DSM_SENSORHUB5_REG            0x32
#define LSM6DSM_SENSORHUB6_REG            0x33
#define LSM6DSM_SENSORHUB7_REG            0x34
#define LSM6DSM_SENSORHUB8_REG            0x35
#define LSM6DSM_SENSORHUB9_REG            0x36
#define LSM6DSM_SENSORHUB10_REG           0x37
#define LSM6DSM_SENSORHUB11_REG           0x38
#define LSM6DSM_SENSORHUB12_REG           0x39
#define LSM6DSM_FIFO_STATUS1              0x3A
#define LSM6DSM_FIFO_STATUS2              0x3B
#define LSM6DSM_FIFO_STATUS3              0x3C
#define LSM6DSM_FIFO_STATUS4              0x3D
#define LSM6DSM_FIFO_DATA_OUT_L           0x3E
#define LSM6DSM_FIFO_DATA_OUT_H           0x3F
#define LSM6DSM_TIMESTAMP0_REG            0x40
#define LSM6DSM_TIMESTAMP1_REG            0x41
#define LSM6DSM_TIMESTAMP2_REG            0x42
#define LSM6DSM_STEP_TIMESTAMP_L          0x49
#define LSM6DSM_STEP_TIMESTAMP_H          0x4A
#define LSM6DSM_STEP_COUNTER_L            0x4B
#define LSM6DSM_STEP_COUNTER_H            0x4C
#define LSM6DSM_SENSORHUB13_REG           0x4D
#define LSM6DSM_SENSORHUB14_REG           0x4E
#define LSM6DSM_SENSORHUB15_REG           0x4F
#define LSM6DSM_SENSORHUB16_REG           0x50
#define LSM6DSM_SENSORHUB17_REG           0x51
#define LSM6DSM_SENSORHUB18_REG           0x52
#define LSM6DSM_FUNC_SRC                  0x53
#define LSM6DSM_TAP_CFG                   0x58
#define LSM6DSM_TAP_THS_6D                0x59
#define LSM6DSM_INT_DUR2                  0x5A
#define LSM6DSM_WAKE_UP_THS               0x5B
#define LSM6DSM_WAKE_UP_DUR               0x5C
#define LSM6DSM_FREE_FALL                 0x5D
#define LSM6DSM_MD1_CFG                   0x5E
#define LSM6DSM_MD2_CFG                   0x5F
#define LSM6DSM_OUT_MAG_RAW_X_L           0x66
#define LSM6DSM_OUT_MAG_RAW_X_H           0x67
#define LSM6DSM_OUT_MAG_RAW_Y_L           0x68
#define LSM6DSM_OUT_MAG_RAW_Y_H           0x69
#define LSM6DSM_OUT_MAG_RAW_Z_L           0x6A
#define LSM6DSM_OUT_MAG_RAW_Z_H           0x6B

#define MASK_EVENT_FF                     0x01
#define MASK_EVENT_WU                     0x02
#define MASK_EVENT_SL                     0x04
#define MASK_EVENT_MD                     0x08
#define MASK_EVENT_PD                     0x10

#define CNT_MAX_EVENTS                    100

typedef enum
{
	En_Mode_Gyro,
	En_Mode_ACC,
	En_Mode_Both
}EnumOperatingMode;

typedef enum
{
	En_Pow_POWERDOWN = 0,
	En_Pow_LOWPOWER = 2,
	En_Pow_NORMAL = 5,
	En_Pow_HIGHPERF = 10
}EnumPowerMode; /* From table 53 */

typedef struct
{
	I2C_HandleTypeDef* i2c_dev;
	uint8_t addr;
	EnumOperatingMode enOPMode;
	EnumPowerMode enPowerMode;
	bool bDetectFreeFall;
	bool bDetectWakeup;
	bool bDetectSleep;
	bool bDetectMotion;
	bool bDetectStep;
} StrLSM6DS3;
/*******************************************************************************
 *  global variables
 *******************************************************************************/

/*******************************************************************************
 *  global functions (public)
 *******************************************************************************/
void     LSM6DS3_SetupRefs(StrLSM6DS3* strDev, I2C_HandleTypeDef* i2c, uint8_t addr);
bool     LSM6DS3_Config(StrLSM6DS3* dev, EnumOperatingMode enOp, EnumPowerMode enPow, uint8_t scaleXL, uint8_t scaleG);
bool     LSM6DS3_Set_Events(StrLSM6DS3* dev, bool ff, bool wu, bool sl, bool md, bool sd);
void     ProcessISR(StrLSM6DS3* dev);
uint8_t* LSM6DS3_Get_Event(EnumCQErrors* cqErrOut);

#endif /* LSM6DS3_H_ */
