/*
 * Copyright (c) 2023 Russ Webber
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aosong_aht21

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include "aht21.h"

LOG_MODULE_REGISTER(AHT21, CONFIG_SENSOR_LOG_LEVEL);

static inline uint32_t aht21_get_be20(const uint8_t src[3], bool left_aligned)
{
	if (left_aligned) {
		return (src[0] << 12) | (src[1] << 4) | (src[2] >> 4);
	} else {
		return ((src[0] & 0x0F) << 16) | (src[1] << 8) | src[2];
	}
}

static int aht21_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	const struct aht21_dev_config *config = dev->config;
	struct aht21_data *drv_data = dev->data;
	uint8_t dev_status;
	uint8_t buffer[7];
	int rc;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL ||
			chan == SENSOR_CHAN_AMBIENT_TEMP ||
			chan == SENSOR_CHAN_HUMIDITY);

	uint8_t command[] = AHT21_COMMAND_TRIGGER_MEASUREMENT;

	rc = i2c_burst_write_dt(&config->i2c, AHT21_REG_TRIGGER_MEASUREMENT, (uint8_t *)&command, sizeof(command));
	if (rc < 0) {
		LOG_ERR("Failed to write TRIGGER_MEASUREMENT register");
		return -EIO;
	}

	/* wait for measurement to complete */
	k_msleep(50);

	while(1) {
		/* get sensor status byte */
		rc = i2c_reg_read_byte_dt(&config->i2c, AHT21_REG_STATUS, (uint8_t *)&dev_status);
		if (rc < 0) {
			LOG_ERR("Failed to read STATUS register");
			return -EIO;
		}

		if (dev_status & AHT21_STATUS_MEASURING) {
			LOG_WRN("Waiting more");
			k_msleep(5);
		} else {
			break;
		}
	}

	rc = i2c_burst_read_dt(&config->i2c, AHT21_REG_DATA, (uint8_t *)&buffer,
						sizeof(buffer));
	if (rc < 0) {
		LOG_ERR("Failed to read DATA register");
		return -EIO;
	}

	/* calculate CRC8 on the payload minus the last byte (the checksum) */
	if (buffer[AHT21_PAYLOAD_CRC] != crc8((uint8_t *) &buffer, sizeof(buffer) - 1, AHT21_CRC_POLY, 0xFF, false)) {
		LOG_ERR("CRC error on DATA");
		return -EIO;
	}

	drv_data->humidity = aht21_get_be20(&buffer[AHT21_PAYLOAD_HUMIDITY], true);
	drv_data->temperature = aht21_get_be20(&buffer[AHT21_PAYLOAD_TEMPERATURE], false);

	return 0;
}

static int aht21_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct aht21_data *drv_data = dev->data;
	int64_t tmp_temperature;
	uint64_t tmp_humidity;

	switch(chan) {
		case SENSOR_CHAN_HUMIDITY:
			tmp_humidity = (uint64_t) (drv_data->humidity) * 100 * 1000000 / AHT21_RESOLUTION_DIV;
			val->val1 = tmp_humidity / 1000000; /* RH % */ 
			val->val2 = tmp_humidity % 1000000;
			break;
		case SENSOR_CHAN_AMBIENT_TEMP:
			tmp_temperature = ((int64_t) drv_data->temperature) * 200 * 1000000 / AHT21_RESOLUTION_DIV - 50 * 1000000;
			val->val1 = tmp_temperature / 1000000; /* Celsius */
			val->val2 = tmp_temperature % 1000000;
			break;
		default:
			return -ENOTSUP;
	}

	return 0;
}

static int aht21_calibrate_register(const struct device *dev,
			       int reg)
{
	const struct aht21_dev_config *config = dev->config;
	uint8_t dev_status;
	uint8_t buffer[3];
	int rc;

	uint8_t command[] = AHT21_COMMAND_CALIBRATE;

	rc = i2c_burst_write_dt(&config->i2c, reg, (uint8_t *)&command, sizeof(command));
	if (rc < 0) {
		LOG_ERR("Failed to write 0x%x register", reg);
		return -EIO;
	}

	k_msleep(5);

	/* read three calibration bytes */
	rc = i2c_read_dt(&config->i2c, (uint8_t *)&buffer, sizeof(buffer));
	if (rc < 0) {
		LOG_ERR("Failed to read calibration bytes");
		return -EIO;
	}

	k_msleep(10);

	/* write the latter two bytes recieved */
	rc = i2c_burst_write_dt(&config->i2c, AHT21_REG_CALIBRATION_SET | reg, (uint8_t *)&buffer[1], 2);
	if (rc < 0) {
		LOG_ERR("Failed to write calibration bytes to 0x%x register", reg);
		return -EIO;
	}

	return 0;
}

static int aht21_init(const struct device *dev)
{
	const struct aht21_dev_config *config = dev->config;
	uint8_t dev_status;
	int rc;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C device %s not ready", config->i2c.bus->name);
		return -EINVAL;
	}

	/* get sensor status byte */
	rc = i2c_reg_read_byte_dt(&config->i2c, AHT21_REG_STATUS, (uint8_t *)&dev_status);
	if (rc < 0) {
		LOG_ERR("Failed to read STATUS register");
		return -EIO;
	}

	LOG_DBG("Status is 0x%x", dev_status);

	/* check that device is calibrated and initialised */
	if ((dev_status & AHT21_SENSOR_INITIALISED) != AHT21_SENSOR_INITIALISED) {
		LOG_INF("Sensor is not calibrated/initialised. Initialising the calibration registers");
		/* this initialisation routine is only found in the sample code for AHT2X sensors */
		aht21_calibrate_register(dev, AHT21_REG_CALIBRATION_1);
		aht21_calibrate_register(dev, AHT21_REG_CALIBRATION_2);
		aht21_calibrate_register(dev, AHT21_REG_CALIBRATION_3);

		rc = i2c_reg_read_byte_dt(&config->i2c, AHT21_REG_STATUS, (uint8_t *)&dev_status);
		if (rc < 0) {
			LOG_ERR("Failed to read STATUS register");
			return -EIO;
		}

		if ((dev_status & AHT21_SENSOR_INITIALISED) != AHT21_SENSOR_INITIALISED) {
			LOG_ERR("Failed to initialise sensor");
			return -EIO;
		}
	}

	/* sleep 10ms before sending triggers */
	k_msleep(10);

	return 0;
}

static const struct sensor_driver_api aht21_driver_api = {
	.sample_fetch = aht21_sample_fetch,
	.channel_get = aht21_channel_get
};

#define DEFINE_AHT21(_num) \
	static struct aht21_data aht21_data_##_num; \
	static const struct aht21_dev_config aht21_config_##_num = { \
		.i2c = I2C_DT_SPEC_INST_GET(_num) \
	}; \
	DEVICE_DT_INST_DEFINE(_num, aht21_init, NULL,			\
		&aht21_data_##_num, &aht21_config_##_num, POST_KERNEL, \
		CONFIG_SENSOR_INIT_PRIORITY, &aht21_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_AHT21)
