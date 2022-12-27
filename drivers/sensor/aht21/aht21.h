/*
 * Copyright (c) 2023 Russ Webber
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>


#ifndef ZEPHYR_DRIVERS_SENSOR_AHT21_AHT21_H_
#define ZEPHYR_DRIVERS_SENSOR_AHT21_AHT21_H_

// #define AHT21_REG_INIT		0xBE
#define AHT21_REG_STATUS		0x71
#define AHT21_REG_TRIGGER_MEASUREMENT		0xAC
#define AHT21_REG_RESET		0xBA
#define AHT21_REG_DATA		0x72
/* the following are taken from the sensor's example code */
#define AHT21_REG_CALIBRATION_SET		0xB0
#define AHT21_REG_CALIBRATION_1		0x1B
#define AHT21_REG_CALIBRATION_2		0x1C
#define AHT21_REG_CALIBRATION_3		0x1E

#define AHT21_COMMAND_TRIGGER_MEASUREMENT		{0x33, 0x00}
#define AHT21_COMMAND_CALIBRATE		{0x00, 0x00}

#define AHT21_STATUS_MEASURING		BIT(7)
#define AHT21_STATUS_INIT_FLAG		BIT(5) /* unknown init flag */
#define AHT21_STATUS_INIT_FLAG_2		BIT(4) /* unknown init flag */
#define AHT21_STATUS_CALIBRATED		BIT(3)
/* datasheet says bit 3 true = calibrated, which weirdly isn't included in the below mask */
#define AHT21_SENSOR_INITIALISED		0x18

#define AHT21_RESOLUTION_DIV		0x100000
#define AHT21_CRC_POLY       0x31

#define AHT21_PAYLOAD_STATUS		0
#define AHT21_PAYLOAD_HUMIDITY		1
#define AHT21_PAYLOAD_TEMPERATURE		3
#define AHT21_PAYLOAD_CRC		6

struct aht21_data {
	uint32_t humidity;
	uint32_t temperature;
};

struct aht21_dev_config {
	struct i2c_dt_spec i2c;
};

#endif /*  ZEPHYR_DRIVERS_SENSOR_AHT21_AHT21_H_ */
