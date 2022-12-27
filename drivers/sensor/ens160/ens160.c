/*
 * Copyright (c) 2023 Russ Webber.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_ens160

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include "ens160.h"

LOG_MODULE_REGISTER(ENS160, CONFIG_SENSOR_LOG_LEVEL);

#ifdef CONFIG_ENS160_CRC_CHECK
static int ens160_misr_fetch(const struct device *dev, uint8_t *crc_checksum)
{
	const struct ens160_dev_config *config = dev->config;
	int ret;
	uint8_t misr;

	ret = i2c_reg_read_byte_dt(&config->i2c, ENS160_REG_DATA_MISR, &misr);
	if (ret < 0) {
		LOG_ERR("Failed to read MISR register");
		return -EIO;
	}
	
	*crc_checksum = misr;
	return 0;
}

/* CRC-esque routine based off code from the datasheet */
static uint8_t ens160_crc(const uint8_t *src, size_t len, uint8_t initial_value)
{
	uint8_t crc = initial_value;
	size_t i;

	for (i = 0; i < len; i++) {
		uint8_t misr_xor = ((crc << 1) ^ src[i]) & 0xFF;

		if ((crc & 0x80) == 0) {
			crc = misr_xor;
		} else {
			crc = misr_xor ^ ENS160_CRC_POLY;
		}
	}

	return crc;
}
#endif /* CONFIG_ENS160_CRC_CHECK */

static int ens160_read_data_crc(const struct device *dev, uint8_t reg,
			       uint8_t *dst, size_t len)
{
	const struct ens160_dev_config *config = dev->config;
	struct ens160_data *drv_data = dev->data;
	int ret, cnt;
#ifdef CONFIG_ENS160_CRC_CHECK
	uint8_t misr_pre, misr_tmp;
#endif /* CONFIG_ENS160_CRC_CHECK */

	for (cnt = 0; cnt <= CONFIG_ENS160_MAX_READ_RETRIES; cnt++) {
#ifdef CONFIG_ENS160_CRC_CHECK
		ens160_misr_fetch(dev, &misr_pre);
#endif /* CONFIG_ENS160_CRC_CHECK */
		ret = i2c_burst_read_dt(&config->i2c, reg, dst, len);
		if (ret < 0) {
			LOG_ERR("Failed to read data from register 0x%x", reg);
			continue;
		}

#ifdef CONFIG_ENS160_CRC_CHECK
		ret = ens160_misr_fetch(dev, &drv_data->misr);
		misr_tmp = ens160_crc(dst, len, misr_pre);
		if (ret < 0 || misr_tmp != drv_data->misr) {
			LOG_WRN("CRC error reading register 0x%x", reg);
			ret = -EIO;
			continue;
		}
#endif /* CONFIG_ENS160_CRC_CHECK */
		return 0;
	}

	return ret;
}

static int ens160_reg_write_16(const struct device *dev, int reg, uint16_t value)
{
	const struct ens160_dev_config *config = dev->config;
	int ret;

	ret = i2c_burst_write_dt(&config->i2c, reg, (uint8_t *)&value, sizeof(value));
	if (ret < 0) {
		LOG_ERR("Failed to write to register 0x%x", reg);
	}
	return ret;
}

static int ens160_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct ens160_data *drv_data = dev->data;
	const struct ens160_dev_config *config = dev->config;

	struct ens160_aqi_data aqi_data;
	uint16_t eco2_data;
	uint16_t tvoc_data;
	int cnt, ret;
	uint8_t dev_status;

	for (cnt = 0; cnt <= CONFIG_ENS160_MAX_READ_RETRIES; cnt++) {
		ret = i2c_reg_read_byte_dt(&config->i2c, ENS160_REG_DEVICE_STATUS, (uint8_t *)&dev_status);
		if (ret < 0) {
			LOG_ERR("Could not read DEVICE_STATUS register");
			return ret;
		}

		if (!(dev_status & ENS160_STATUS_NEW_DATA)) {
			LOG_WRN("Sensor doesn't have a new sample to read - waiting 250ms");
			k_msleep(250); /* is it acceptable to do this in a sensor driver? or just return EBUSY instead? */
			ret = -EBUSY;
			continue;
		}

		break;
	}

	if (ret < 0) {
		LOG_ERR("No sample available - giving up.");
		return ret;
	}

	/* clear the samples*/
	drv_data->aqi = 0U;
	drv_data->eco2 = 0U;
	drv_data->tvoc = 0U;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL
			|| chan == SENSOR_CHAN_AIR_QUALITY_INDEX
			|| chan == SENSOR_CHAN_CO2
			|| chan == SENSOR_CHAN_VOC);

	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AIR_QUALITY_INDEX) {
		ret = ens160_read_data_crc(dev, ENS160_REG_DATA_AQI, (uint8_t *)&aqi_data,
						sizeof(aqi_data));
		if (ret < 0) {
			return ret;
		}

		drv_data->aqi = aqi_data.air_quality_index;
	}

	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_VOC) {
		ret = ens160_read_data_crc(dev, ENS160_REG_DATA_TVOC, (uint8_t *)&tvoc_data,
						sizeof(tvoc_data));

		if (ret < 0) {
			return ret;
		}

		drv_data->tvoc = tvoc_data;
	}

	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_CO2) {
		ret = ens160_read_data_crc(dev, ENS160_REG_DATA_ECO2, (uint8_t *)&eco2_data,
						sizeof(eco2_data));

		if (ret < 0) {
			return ret;
		}

		drv_data->eco2 = eco2_data;
	}

	return 0;
}

static int ens160_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct ens160_data *drv_data = dev->data;

	switch (chan) {
		case SENSOR_CHAN_AIR_QUALITY_INDEX:
			val->val1 = drv_data->aqi;
			val->val2 = 0;
			break;
		case SENSOR_CHAN_CO2:
			val->val1 = sys_le16_to_cpu(drv_data->eco2);
			val->val2 = 0;
			break;
		case SENSOR_CHAN_VOC:
			val->val1 = sys_le16_to_cpu(drv_data->tvoc);
			val->val2 = 0;
			break;
		default:
			return -ENOTSUP;
	}

	return 0;
}

static int ens160_attr_set(const struct device *dev,
			   enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	uint16_t value;

	switch (attr) {
	case SENSOR_ATTR_CALIBRATION:
		switch(chan) {
			case SENSOR_CHAN_AMBIENT_TEMP:
				/* sensor expects Kelvin*64 */
				value = (((uint64_t)273150000 + val->val1 * 1000000 + val->val2) * 64) / 1000000;
				LOG_DBG("Setting TEMP_IN to 0x%x", value);
				return ens160_reg_write_16(dev, ENS160_REG_TEMPERATURE_IN, sys_cpu_to_le16(value));
			case SENSOR_CHAN_HUMIDITY:
				/* sensor expects %RH*512 */
				value = (((uint64_t)1000000 * val->val1 + val->val2) * 512) / 1000000;
				LOG_DBG("Setting RH_IN to 0x%x", value);
				return ens160_reg_write_16(dev, ENS160_REG_RELATIVE_HUMIDITY_IN, sys_cpu_to_le16(value));
			default:
				return -ENOTSUP;
		}
	case SENSOR_ATTR_CONFIGURATION:
		/* TODO - SET INTERUPT BEHAVIOURS */
		return -ENOTSUP;
	default:
		return -ENOTSUP;
	}
}

static int ens160_set_opmode(const struct device *dev, uint8_t mode)
{
	const struct ens160_dev_config *config = dev->config;
	int ret;

	LOG_INF("Setting OPMODE to 0x%x", mode);

	ret = i2c_reg_write_byte_dt(&config->i2c, ENS160_REG_OPMODE, mode);
	if (ret < 0) {
		LOG_ERR("Failed to set OPMODE to 0x%x", mode);
	}
	return ret;
}

static int ens160_fetch_calibration(const struct device *dev)
{
	const struct ens160_dev_config *config = dev->config;
	struct sensor_value temperature, humidity;
	int ret;

	if (config->humidity_source != NULL) {
		ret = sensor_channel_get(config->humidity_source, config->humidity_channel, &humidity);
		if (ret < 0) {
			LOG_ERR("Could not get relative humidity calibration data from %s using channel 0x%x error code (%d)",
			config->humidity_source->name, config->humidity_channel, ret);
		} else {
			ret = ens160_attr_set(dev, SENSOR_CHAN_HUMIDITY, SENSOR_ATTR_CALIBRATION, &humidity);
			if (ret < 0) {
				LOG_ERR("Could not set the humidity SENSOR_ATTR_CALIBRATION - error %d", ret);
			}
		}
	}

	if (config->temperature_source != NULL) {
		ret = sensor_channel_get(config->temperature_source, config->temperature_channel, &temperature);
		if (ret < 0) {
			LOG_ERR("Could not get SENSOR_CHAN_AMBIENT_TEMP channel (%d)", ret);
		} else {
			ret = ens160_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_CALIBRATION, &temperature);
			if (ret < 0) {
				LOG_ERR("Could not set the humidity SENSOR_ATTR_CALIBRATION - error %d", ret);
			}
		}
	}

	return 0;
}

static void ens160_work_cb(struct k_work *work)
{
	struct ens160_data *drv_data =
		CONTAINER_OF(work, struct ens160_data, work);

	ens160_fetch_calibration(drv_data->dev);
}

static void calib_timer_expiry(struct k_timer *timer_id)
{
	struct ens160_data *drv_data = (struct ens160_data*) k_timer_user_data_get(timer_id);
	k_work_submit(&drv_data->work);
}

static int ens160_init(const struct device *dev)
{
	const struct ens160_dev_config *config = dev->config;
	struct ens160_data *drv_data = dev->data;
	uint8_t dev_status;

	int ret;
	uint16_t part_id;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C device %s not ready", config->i2c.bus->name);
		return -EINVAL;
	}

	/* Check Hardware ID. */
	ret = i2c_burst_read_dt(&config->i2c, ENS160_REG_PART_ID, (uint8_t *)&part_id,
				sizeof(part_id));
	if (ret < 0) {
		LOG_ERR("Failed to read Part ID register");
		return -EIO;
	}

	if (sys_le16_to_cpu(part_id) != ENS160_PART_ID) {
		LOG_ERR("Part ID does not match. Needed 0x%x, got 0x%x",
			    ENS160_PART_ID, part_id);
		return -EINVAL;
	}

	ret = i2c_reg_read_byte_dt(&config->i2c, ENS160_REG_DEVICE_STATUS, (uint8_t *)&dev_status);
	if (ret < 0) {
		LOG_ERR("Failed to read DEVICE_STATUS");
		return -EIO;
	}

	LOG_DBG("DEVICE_STATUS: 0x%x, STATAS: %lx, STATER: %lx, Validity Flag: %lx",
		dev_status, ENS160_STATUS_STATAS & dev_status, ENS160_STATUS_STATER & dev_status,
		(ENS160_STATUS_VALIDITY_FLAG & dev_status));

	switch((ENS160_STATUS_VALIDITY_FLAG & dev_status) >> 2) {
		case ENS160_VALIDITY_FLAG_WARMUP:
			LOG_INF("Sensor is in warm-up mode");
			break;
		case ENS160_VALIDITY_FLAG_INITIAL_STARTUP:
			LOG_INF("Sensor is in initial start up mode");
			break;
		case ENS160_VALIDITY_FLAG_NO_VALID_OUTPUT:
			LOG_WRN("Sensor is in no valid output mode");
			break;
	}

	if (ENS160_STATUS_STATER & dev_status) {
		LOG_WRN("Sensor is in an error state");
	}

	if (!(ENS160_STATUS_STATAS & dev_status)) {
		/* Set standard operation mode */
		ret = ens160_set_opmode(dev, ENS160_OPMODE_STANDARD);
		if (ret < 0) {
			return ret;
		}
	}

	drv_data->dev = dev;

	/* Set up the timer and work handler for calibration setting */
	if ((config->humidity_source != NULL) || (config->temperature_source != NULL)) {
		k_timer_init(&drv_data->calib_timer, calib_timer_expiry, NULL);
		k_timer_user_data_set(&drv_data->calib_timer, (void *)drv_data);
		drv_data->work.handler = ens160_work_cb;
		k_timer_start(&drv_data->calib_timer, K_MSEC(CONFIG_ENS160_CALIBRATION_SET_RATE), K_MSEC(CONFIG_ENS160_CALIBRATION_SET_RATE));
	}

	return 0;
}

static const struct sensor_driver_api ens160_driver_api = {
	.attr_set = ens160_attr_set,
	.sample_fetch = ens160_sample_fetch,
	.channel_get = ens160_channel_get
};

#define ENS160_DEFINE(inst)								\
	static struct ens160_data ens160_data_##inst;					\
	static const struct ens160_dev_config ens160_dev_config_##inst = {			\
		.i2c = I2C_DT_SPEC_INST_GET(inst),					\
		.humidity_source = DEVICE_DT_GET(DT_INST_PROP(inst, humidity_source)),					\
		.humidity_channel = SENSOR_CHAN_HUMIDITY,					\
		.temperature_source = DEVICE_DT_GET(DT_INST_PROP(inst, temperature_source)),					\
		.temperature_channel = SENSOR_CHAN_AMBIENT_TEMP					\
	};										\
											\
	DEVICE_DT_INST_DEFINE(inst, ens160_init, NULL,					\
			      &ens160_data_##inst, &ens160_dev_config_##inst, POST_KERNEL,	\
			      CONFIG_SENSOR_INIT_PRIORITY, &ens160_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ENS160_DEFINE)
