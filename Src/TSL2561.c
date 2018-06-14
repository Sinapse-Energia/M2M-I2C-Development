/*
 * Copyright (C) 2016 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 * Modified by Owais for STM32 use
 */

/**
 * @ingroup     drivers_tsl2561
 * @{
 *
 * @file
 * @brief       Device driver implementation for the TSL2561 Luminosity sensor.
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @}
 */

#include <math.h>
#include <string.h>

#include "TSL2561.h"
#include "TSL2561_Internals.h"

/* internal helpers */
static void _enable(const tsl2561_t *dev);
static void _disable(const tsl2561_t *dev);
static void _read_data(const tsl2561_t *dev, uint16_t *full, uint16_t *ir);
static void _read_data_atomic(const tsl2561_t *dev, uint16_t *full, uint16_t *ir);


/*---------------------------------------------------------------------------*
 *                          TSL2561 Core API                                 *
 *---------------------------------------------------------------------------*/

int tsl2561_init(tsl2561_t *dev, I2C_HandleTypeDef* i2c, uint8_t addr, uint8_t gain, uint8_t integration)
{
	HAL_StatusTypeDef res;
    dev->i2c_dev = i2c;
    dev->addr = addr;
    dev->gain = gain;
    dev->integration = integration;

    /* Verify sensor ID */
    uint8_t rd_id_bytes[1];
    res = HAL_I2C_Mem_Read(dev->i2c_dev, dev->addr, TSL2561_COMMAND_MODE | TSL2561_REGISTER_ID, I2C_MEMADD_SIZE_8BIT, rd_id_bytes, 1, 100);

    if (rd_id_bytes[0] != TSL2561_ID )
    {
    	return TSL2561_BADDEV;
    }

    if(res != HAL_OK)
    {
    	return TSL2561_NOI2C;
    }

    _enable(dev);

    /* configuring gain and integration time */
    uint8_t wr_bytes[1];
    wr_bytes[0] = dev->integration | dev->gain;
    res = HAL_I2C_Mem_Write(dev->i2c_dev, dev->addr, TSL2561_COMMAND_MODE | TSL2561_REGISTER_TIMING, I2C_MEMADD_SIZE_8BIT, wr_bytes, 1, 100);

    uint8_t chk_bytes[1];
    res = HAL_I2C_Mem_Read (dev->i2c_dev, dev->addr, TSL2561_COMMAND_MODE | TSL2561_REGISTER_TIMING, I2C_MEMADD_SIZE_8BIT, chk_bytes, 1, 100);
    _disable(dev);
    return TSL2561_OK;
}

uint16_t tsl2561_read_illuminance(const tsl2561_t *dev)
{
    /* Read IR and full spectrum values */
    uint16_t ir = 0;
    uint16_t full = 0;
    _read_data(dev, &full, &ir);

    /* Compute illuminance */
    uint32_t channel_scale;
    uint32_t channel_1;
    uint32_t channel_0;

    switch (dev->integration)
    {
        case TSL2561_INTEGRATIONTIME_13MS:
            channel_scale = TSL2561_CHSCALE_TINT0;
            break;

        case TSL2561_INTEGRATIONTIME_101MS:
            channel_scale = TSL2561_CHSCALE_TINT1;
            break;

        default: /* No scaling ... integration time = 402ms */
            channel_scale = (1 << TSL2561_CHSCALE);
            break;
    }

    /* Scale for gain (1x or 16x) */
    if (!dev->gain)
    {
        channel_scale = channel_scale << 4;
    }

    /* scale the channel values */
    channel_0 = (full * channel_scale) >> TSL2561_CHSCALE;
    channel_1 = (ir * channel_scale) >> TSL2561_CHSCALE;

    /* find the ratio of the channel values (Channel1/Channel0) */
    uint32_t ratio_1 = 0;
    if (channel_0 != 0) {
        ratio_1 = (channel_1 << (TSL2561_RATIOSCALE + 1)) / channel_0;
    }

    /* round the ratio value */
    uint32_t ratio = (ratio_1 + 1) >> 1;
    uint32_t b, m;

    if (ratio <= TSL2561_K1T) {
        b = TSL2561_B1T;
        m = TSL2561_M1T;
    }
    else if (ratio <= TSL2561_K2T) {
        b = TSL2561_B2T;
        m = TSL2561_M2T;
    }
    else if (ratio <= TSL2561_K3T) {
        b = TSL2561_B3T;
        m = TSL2561_M3T;
    }
    else if (ratio <= TSL2561_K4T) {
        b = TSL2561_B4T;
        m = TSL2561_M4T;
    }
    else if (ratio <= TSL2561_K5T) {
        b = TSL2561_B5T;
        m = TSL2561_M5T;
    }
    else if (ratio <= TSL2561_K6T) {
        b = TSL2561_B6T;
        m = TSL2561_M6T;
    }
    else if (ratio <= TSL2561_K7T) {
        b = TSL2561_B7T;
        m = TSL2561_M7T;
    }
    else {
        b = TSL2561_B8T;
        m = TSL2561_M8T;
    }

    uint32_t illuminance;
    illuminance = ((channel_0 * b) - (channel_1 * m));

    /* round lsb (2^(TSL2561_SCALE - 1)) */
    illuminance += (1 << (TSL2561_LUXSCALE - 1));

    /* return strip off fractional portion */
    return (uint16_t)(illuminance >> TSL2561_LUXSCALE);
}


static void _enable(const tsl2561_t *dev)
{
	HAL_StatusTypeDef res;
    /* enabling device */
    uint8_t wr_bytes[1];
    wr_bytes[0] = TSL2561_CONTROL_POWERON;
    res = HAL_I2C_Mem_Write(dev->i2c_dev, dev->addr, TSL2561_COMMAND_MODE | TSL2561_REGISTER_CONTROL, I2C_MEMADD_SIZE_8BIT, wr_bytes, 1, 10);
}


static void _disable(const tsl2561_t *dev)
{
	HAL_StatusTypeDef res;
	/* disabling device */
	uint8_t wr_bytes[1];
	wr_bytes[0] = TSL2561_CONTROL_POWEROFF;
	res = HAL_I2C_Mem_Write(dev->i2c_dev, dev->addr, TSL2561_COMMAND_MODE | TSL2561_REGISTER_CONTROL, I2C_MEMADD_SIZE_8BIT, wr_bytes, 1, 10);
}

static void _read_data(const tsl2561_t *dev, uint16_t *full, uint16_t *ir)
{
	HAL_StatusTypeDef res;
    /* Enable the device */
    _enable(dev);

    /* Wait integration time in ms for ADC to complete */
    switch (dev->integration)
    {
        case TSL2561_INTEGRATIONTIME_13MS:
        	HAL_Delay(13);
            break;

        case TSL2561_INTEGRATIONTIME_101MS:
        	HAL_Delay(101);
            break;

        default: /* TSL2561_INTEGRATIONTIME_402MS */
        	HAL_Delay(402);
            break;
    }

    uint8_t buffer[2] = { 0 };
    /* Read full spectrum channel */
    res = HAL_I2C_Mem_Read(dev->i2c_dev, dev->addr, TSL2561_COMMAND_MODE | TSL2561_COMMAND_WORD | TSL2561_REGISTER_CHAN0, I2C_MEMADD_SIZE_8BIT, buffer, 2, 10);
    *full = (buffer[1] << 8) | buffer[0];

    memset(buffer, 0, sizeof(buffer));

    /* Read infra-red spectrum channel */
    res = HAL_I2C_Mem_Read(dev->i2c_dev, dev->addr, TSL2561_COMMAND_MODE | TSL2561_COMMAND_WORD | TSL2561_REGISTER_CHAN1, I2C_MEMADD_SIZE_8BIT, buffer, 2, 10);
    *ir = (buffer[1] << 8) | buffer[0];

    /* Turn the device off to save power */
    _disable(dev);
}

static void _read_data_atomic(const tsl2561_t *dev, uint16_t *full, uint16_t *ir)
{
	HAL_StatusTypeDef res;
    /* Enable the device */
    _enable(dev);

    /* Wait integration time in ms for ADC to complete */
    switch (dev->integration)
    {
        case TSL2561_INTEGRATIONTIME_13MS:
        	HAL_Delay(13);
            break;

        case TSL2561_INTEGRATIONTIME_101MS:
        	HAL_Delay(101);
            break;

        default: /* TSL2561_INTEGRATIONTIME_402MS */
        	HAL_Delay(402);
            break;
    }

    uint8_t wr_buffer[2] = { 0 };
    uint8_t rd_buffer[2] = { 0 };
    wr_buffer[0] = TSL2561_COMMAND_MODE | TSL2561_REGISTER_CHAN0;
    res = HAL_I2C_Master_Transmit(dev->i2c_dev, dev->addr, wr_buffer, 1, 10);
    res = HAL_I2C_Master_Receive(dev->i2c_dev, dev->addr, &rd_buffer[0], 1, 10);

    wr_buffer[0] = (TSL2561_COMMAND_MODE | TSL2561_REGISTER_CHAN0) + 1;
    res = HAL_I2C_Master_Transmit(dev->i2c_dev, dev->addr, wr_buffer, 1, 10);
    res = HAL_I2C_Master_Receive(dev->i2c_dev, dev->addr, &rd_buffer[1], 1, 10);

    *full = (rd_buffer[1] << 8) | rd_buffer[0];

    /* Read infra-red spectrum channel */
    wr_buffer[0] = TSL2561_COMMAND_MODE | TSL2561_REGISTER_CHAN1;
    res = HAL_I2C_Master_Transmit(dev->i2c_dev, dev->addr, wr_buffer, 1, 10);
    res = HAL_I2C_Master_Receive(dev->i2c_dev, dev->addr, &rd_buffer[0], 1, 10);

    wr_buffer[0] = (TSL2561_COMMAND_MODE | TSL2561_REGISTER_CHAN1) + 1;
    res = HAL_I2C_Master_Transmit(dev->i2c_dev, dev->addr, wr_buffer, 1, 10);
    res = HAL_I2C_Master_Receive(dev->i2c_dev, dev->addr, &rd_buffer[1], 1, 10);
    *ir = (rd_buffer[1] << 8) | rd_buffer[0];

    /* Turn the device off to save power */
    _disable(dev);
}

