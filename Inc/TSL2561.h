/*
 * Copyright (C) 2016 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 * Modified by Owais for use in STM32 environment
 */

#ifndef TSL2561_H
#define TSL2561_H

#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TSL2561_ADDR_LOW                  (0x29)
#define TSL2561_ADDR_FLOAT                (0x39)
#define TSL2561_ADDR_HIGH                 (0x49)

#define TSL2561_INTEGRATIONTIME_13MS      (0x00)    /* 13.7ms */
#define TSL2561_INTEGRATIONTIME_101MS     (0x01)    /* 101ms  */
#define TSL2561_INTEGRATIONTIME_402MS     (0x02)    /* 402ms  */
#define TSL2561_INTEGRATIONTIME_NA        (0x03)    /* N/A    */

#define TSL2561_GAIN_1X                   (0x00)
#define TSL2561_GAIN_16X                  (0x10)

#define TSL2561_OK                        (0)
#define TSL2561_NOI2C                     (-1)
#define TSL2561_BADDEV                    (-2)

typedef struct {
	I2C_HandleTypeDef* i2c_dev;
	uint8_t addr;
	uint8_t gain;
	uint8_t integration;
} tsl2561_t;

typedef tsl2561_t tsl2561_params_t;

int tsl2561_init(tsl2561_t *dev, I2C_HandleTypeDef* i2c, uint8_t addr, uint8_t gain, uint8_t integration);

uint16_t tsl2561_read_illuminance(const tsl2561_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* TSL2561_H */
