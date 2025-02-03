/*
 * Copyright (c) 2022 Esco Medical ApS
 * Copyright (c) 2016 TDK Invensense
 * Copyright (c) 2025 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include "icm42670.h"
#include "icm42670_reg.h"
#include "icm42670_trigger.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(ICM42670, CONFIG_SENSOR_LOG_LEVEL);

#ifdef ICM42670_TRIGGER_IBI
static int icm42670_ibi_callback(struct i3c_device_desc *target,
								struct i3c_ibi_payload *payload)
{
	struct icm42670_data *data = (struct icm42670_data *)target->dev->data;

#if defined(CONFIG_ICM42670_TRIGGER_OWN_THREAD)
	k_sem_give(&data->trig_sem);
#elif defined(CONFIG_ICM42670_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->work);
#endif
	return 0;
}
#else
static void icm42670_gpio_callback(const struct device *dev, struct gpio_callback *cb,
					uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	struct icm42670_data *data = CONTAINER_OF(cb, struct icm42670_data, gpio_cb);

#if defined(CONFIG_ICM42670_TRIGGER_OWN_THREAD)
	k_sem_give(&data->trig_sem);
#elif defined(CONFIG_ICM42670_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->work);
#endif
}
#endif

static void icm42670_thread_cb(const struct device *dev)
{
	struct icm42670_data *data = dev->data;

	icm42670_lock(dev);

#ifndef ICM42670_TRIGGER_IBI
	const struct icm42670_config *cfg = dev->config;

	gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_DISABLE);
#endif

	if (data->data_ready_handler) {
		data->data_ready_handler(dev, data->data_ready_trigger);
	}

#ifndef ICM42670_TRIGGER_IBI
	gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_TO_ACTIVE);
#endif

	icm42670_unlock(dev);
}

#if defined(CONFIG_ICM42670_TRIGGER_OWN_THREAD)

static void icm42670_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct icm42670_data *data = p1;

	while (1) {
		k_sem_take(&data->trig_sem, K_FOREVER);
		icm42670_thread_cb(data->dev);
	}
}

#elif defined(CONFIG_ICM42670_TRIGGER_GLOBAL_THREAD)

static void icm42670_work_handler(struct k_work *work)
{
	struct icm42670_data *data = CONTAINER_OF(work, struct icm42670_data, work);

	icm42670_thread_cb(data->dev);
}

#endif

int icm42670_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	int res = 0;
	struct icm42670_data *data = dev->data;

	if (!handler) {
		return -EINVAL;
	}

	icm42670_lock(dev);

#ifndef ICM42670_TRIGGER_IBI
	const struct icm42670_config *cfg = dev->config;

	gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_DISABLE);
#endif

	switch (trig->type) {
	case SENSOR_TRIG_DATA_READY:
		data->data_ready_handler = handler;
		data->data_ready_trigger = trig;
		break;
	default:
		res = -ENOTSUP;
		break;
	}

	icm42670_unlock(dev);

#ifndef ICM42670_TRIGGER_IBI
	gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_TO_ACTIVE);
#endif

	return res;
}

int icm42670_trigger_init(const struct device *dev)
{
	struct icm42670_data *data = dev->data;
	const struct icm42670_config *cfg = dev->config;
	int res = 0;

	data->dev = dev;

#ifdef ICM42670_TRIGGER_IBI
	res = i3c_subscribe_ibi_cb_dt(&cfg->bus.i3c, icm42670_ibi_callback);
	if (res < 0) {
		LOG_ERR("Failed to set IBI callback");
		return res;
	}
#else
	if (!cfg->gpio_int.port) {
		LOG_ERR("trigger enabled but no interrupt gpio supplied");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->gpio_int)) {
		LOG_ERR("gpio_int gpio not ready");
		return -ENODEV;
	}

	gpio_pin_configure_dt(&cfg->gpio_int, GPIO_INPUT);
	gpio_init_callback(&data->gpio_cb, icm42670_gpio_callback, BIT(cfg->gpio_int.pin));
	res = gpio_add_callback(cfg->gpio_int.port, &data->gpio_cb);

	if (res < 0) {
		LOG_ERR("Failed to set gpio callback");
		return res;
	}
#endif
	k_mutex_init(&data->mutex);

#if defined(CONFIG_ICM42670_TRIGGER_OWN_THREAD)
	k_sem_init(&data->trig_sem, 0, K_SEM_MAX_LIMIT);
	k_thread_create(&data->thread, data->thread_stack, CONFIG_ICM42670_THREAD_STACK_SIZE,
			icm42670_thread, data, NULL, NULL,
			K_PRIO_COOP(CONFIG_ICM42670_THREAD_PRIORITY), 0, K_NO_WAIT);
#elif defined(CONFIG_ICM42670_TRIGGER_GLOBAL_THREAD)
	data->work.handler = icm42670_work_handler;
#endif

#ifndef ICM42670_TRIGGER_IBI
	return gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_TO_ACTIVE);
#endif

	return 0;
}

int icm42670_trigger_enable_interrupt(const struct device *dev)
{
	int res;
	const struct icm42670_config *cfg = dev->config;

#ifdef ICM42670_TRIGGER_IBI
	res = cfg->bus_io->write(&cfg->bus, REG_INT_SOURCE8, BIT_INT_DRDY_IBI_EN);
	if (res) {
		return res;
	}

	res = cfg->bus_io->write(&cfg->bus, REG_INTF_CONFIG6, BIT_IBI_EN);
	if (res) {
		return res;
	}
#else
	/* pulse-mode (auto clearing), push-pull and active-high */
	res = cfg->bus_io->write(&cfg->bus, REG_INT_CONFIG,
					BIT_INT1_DRIVE_CIRCUIT | BIT_INT1_POLARITY);

	if (res) {
		return res;
	}

	/* enable data ready interrupt on INT1 pin */
	return cfg->bus_io->write(&cfg->bus, REG_INT_SOURCE0, BIT_INT_DRDY_INT1_EN);
#endif

	return 0;
}

void icm42670_lock(const struct device *dev)
{
	struct icm42670_data *data = dev->data;

	k_mutex_lock(&data->mutex, K_FOREVER);
}

void icm42670_unlock(const struct device *dev)
{
	struct icm42670_data *data = dev->data;

	k_mutex_unlock(&data->mutex);
}
