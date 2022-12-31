/*
 * Copyright (c) 2023 Russ Webber
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app);

static struct sensor_value temperature;
static struct sensor_value humidity;

// void calibrate(const struct device *gas_sensor) {
// 	int ret;
// 	ret = sensor_attr_set(gas_sensor, SENSOR_CHAN_HUMIDITY, SENSOR_ATTR_CALIBRATION, &humidity);
// 	if (ret < 0) {
// 		LOG_ERR("Could not set the humidity calibration channel on %s - error %d", gas_sensor->name, ret);
// 	}

// 	ret = sensor_attr_set(gas_sensor, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_CALIBRATION, &temperature);
// 	if (ret < 0) {
// 		LOG_ERR("Could not set the temperature calibration channel on %s - error %d", gas_sensor->name, ret);
// 	}
// }

void main(void)
{
	int ret;
	const struct device *gas_sensor;
	const struct device *ht_sensor;

	LOG_INF("ENS160+AHT2x board sample started");

	// gas_sensor = DEVICE_DT_GET_ANY(ams_ens160);
	gas_sensor = DEVICE_DT_GET(DT_NODELABEL(gas_sensor));
	if (!device_is_ready(gas_sensor)) {
		LOG_ERR("Air quality sensor %s not ready", gas_sensor->name);
		return;
	}

	ht_sensor = DEVICE_DT_GET(DT_NODELABEL(ht_sensor));
	if (!device_is_ready(ht_sensor)) {
		LOG_ERR("Humidity and Temp sensor %s not ready", ht_sensor->name);
		return;
	}

	while (1) {
		struct sensor_value val;

		ret = sensor_sample_fetch(gas_sensor);
		if (ret < 0) {
			LOG_ERR("%s - could not fetch sample (%d)", gas_sensor->name, ret);
			return;
		}

		ret = sensor_sample_fetch(ht_sensor);
		if (ret < 0) {
			LOG_ERR("%s - could not fetch sample (%d)", ht_sensor->name, ret);
			return;
		}

		ret = sensor_channel_get(ht_sensor, SENSOR_CHAN_HUMIDITY, &val);
		if (ret < 0) {
			LOG_ERR("Could not get SENSOR_CHAN_HUMIDITY channel (%d)", ret);
		} else {
			LOG_INF("Relative Humidity (%%): %d.%06d", val.val1, val.val2);
			humidity = val;
		}

		ret = sensor_channel_get(ht_sensor, SENSOR_CHAN_AMBIENT_TEMP, &val);
		if (ret < 0) {
			LOG_ERR("Could not get SENSOR_CHAN_AMBIENT_TEMP channel (%d)", ret);
		} else {
			LOG_INF("Temperature (C): %d.%06d", val.val1, val.val2);
			temperature = val;
		}

		// calibrate(gas_sensor);
	
		ret = sensor_channel_get(gas_sensor, SENSOR_CHAN_AIR_QUALITY_INDEX, &val);
		if (ret < 0) {
			LOG_ERR("Could not get SENSOR_CHAN_AIR_QUALITY_INDEX channel (%d)", ret);
		} else {
			LOG_INF("Air Quality Index: %d", val.val1);
		}

		ret = sensor_channel_get(gas_sensor, SENSOR_CHAN_CO2, &val);
		if (ret < 0) {
			LOG_ERR("Could not get SENSOR_CHAN_CO2 channel (%d)", ret);
		} else {
			LOG_INF("Equivalent CO2 (ppm): %d", val.val1);
		}

		ret = sensor_channel_get(gas_sensor, SENSOR_CHAN_VOC, &val);
		if (ret < 0) {
			LOG_ERR("Could not get SENSOR_CHAN_VOC channel (%d)", ret);
		} else {
			LOG_INF("TVOC (ppb): %d", val.val1);
		}

		k_msleep(2000);
	}
}

