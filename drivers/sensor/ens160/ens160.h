/*
 * Copyright (c) 2023 Russ Webber
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ENS160_ENS160_H_
#define ZEPHYR_DRIVERS_SENSOR_ENS160_ENS160_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>

#define ENS160_REG_PART_ID		0x00
#define ENS160_REG_OPMODE		0x10
#define ENS160_REG_CONFIG		0x11
#define ENS160_REG_COMMAND		0x12
#define ENS160_REG_TEMPERATURE_IN		0x13
#define ENS160_REG_RELATIVE_HUMIDITY_IN		0x15
#define ENS160_REG_DEVICE_STATUS		0x20
#define ENS160_REG_DATA_AQI		0x21
#define ENS160_REG_DATA_TVOC		0x22
#define ENS160_REG_DATA_ECO2		0x24
#define ENS160_REG_DATA_TEMPERATURE		0x30
#define ENS160_REG_DATA_RELATIVE_HUMIDITY		0x32
#define ENS160_REG_DATA_MISR		0x38
#define ENS160_REG_GPR_WRITE		0x40
#define ENS160_REG_GPR_READ		0x48

#define ENS160_PART_ID 0x0160

#define ENS160_STATUS_STATAS  BIT(7)
#define ENS160_STATUS_STATER  BIT(6)
#define ENS160_STATUS_VALIDITY_FLAG  (BIT(3) | BIT(2))
#define ENS160_STATUS_NEW_DATA  BIT(1)
#define ENS160_STATUS_NEW_GPR  BIT(0)

#define ENS160_OPMODE_DEEP_SLEEP		0x00
#define ENS160_OPMODE_IDLE		0x01
#define ENS160_OPMODE_STANDARD		0x02
#define ENS160_OPMODE_RESET		0xF0

#define ENS160_VALIDITY_FLAG_STANDARD		0x00
#define ENS160_VALIDITY_FLAG_WARMUP		0x01
#define ENS160_VALIDITY_FLAG_INITIAL_STARTUP		0x02
#define ENS160_VALIDITY_FLAG_NO_VALID_OUTPUT		0x03

/* interupt pin polarity */
#define ENS160_CONFIG_INT_POL		BIT(6)
/* interupt pin drive */
#define ENS160_CONFIG_INT_CFG		BIT(5)
/* interupt pin asserted on new data in GPRs */
#define ENS160_CONFIG_INT_GPR		BIT(3)
/* interupt pin asserted on new data in DATA registers */
#define ENS160_CONFIG_INT_DAT		BIT(1)
/* interupt pin enabled */
#define ENS160_CONFIG_INT_EN		BIT(0)

#define ENS160_CONFIG_INT_POL_ACTIVE_LOW 0
#define ENS160_CONFIG_INT_POL_ACTIVE_HIGH 1

#define ENS160_CONFIG_INT_CFG_OPEN_DRAIN 0
#define ENS160_CONFIG_INT_CFG_PUSH_PULL 1

#define ENS160_CRC_POLY       0x1D

// air quality index
struct ens160_aqi_data {
	uint8_t  air_quality_index : 3;
	uint8_t  reserved  : 5;
} __packed;

struct ens160_data {
	uint8_t aqi;
	uint16_t tvoc;
	uint16_t eco2;
	uint8_t misr;
	struct k_timer calib_timer;
	struct k_work work;
	const struct device *dev;
};

struct ens160_dev_config {
	struct i2c_dt_spec i2c;
	const struct device *humidity_source;
	enum sensor_channel humidity_channel;
	const struct device *temperature_source;
	enum sensor_channel temperature_channel;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_ENS160_ENS160_H_ */
