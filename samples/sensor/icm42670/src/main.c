/*
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(ICM42670, LOG_LEVEL_INF);

static struct sensor_trigger data_trigger;
static volatile uint8_t sample_cnt;

static int process_icm42670(const struct device *dev)
{
	struct sensor_value temperature;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);
	}
	if (rc == 0) {

		if (sample_cnt++ / 50) {
			LOG_INF("temp % g Cel"
				"\t accel % f % f % f m/s/s"
				"\t gyro  % f % f % f rad/s",
				sensor_value_to_double(&temperature),
				sensor_value_to_double(&accel[0]),
				sensor_value_to_double(&accel[1]),
				sensor_value_to_double(&accel[2]), sensor_value_to_double(&gyro[0]),
				sensor_value_to_double(&gyro[1]), sensor_value_to_double(&gyro[2]));
			sample_cnt = 0;
		}
	} else {
		LOG_INF("sample fetch/get failed: %d", rc);
	}

	return rc;
}

static void handle_icm42670_drdy(const struct device *dev, const struct sensor_trigger *trig)
{
	int rc = process_icm42670(dev);

	if (rc != 0) {
		LOG_INF("cancelling trigger due to failure: %d", rc);
		(void)sensor_trigger_set(dev, trig, NULL);
		return;
	}
}

int main(void)
{
	const struct device *const icm42670 = DEVICE_DT_GET_ONE(invensense_icm42670);

	if (!device_is_ready(icm42670)) {
		LOG_INF("sensor: device not ready.\n");
		return 0;
	}

	data_trigger = (struct sensor_trigger){
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};

	if (sensor_trigger_set(icm42670, &data_trigger, handle_icm42670_drdy) < 0) {
		LOG_INF("Cannot configure data trigger!!!\n");
		return 0;
	}

	LOG_INF("Configured for triggered sampling.\n");
	return 0;
}
