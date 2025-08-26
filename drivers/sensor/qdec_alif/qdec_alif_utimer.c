/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_utimer_qdec

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include "utimer.h"

LOG_MODULE_REGISTER(qdec_alif_utimer, CONFIG_SENSOR_LOG_LEVEL);

/* QDEC constant configuration parameters */
struct qdec_alif_utimer_config {
	DEVICE_MMIO_NAMED_ROM(global);
	DEVICE_MMIO_NAMED_ROM(timer);
	const uint8_t timer_id;
	bool filter_enable;
	uint8_t filter_prescaler;
	uint8_t filter_taps;
	const struct pinctrl_dev_config *pcfg;
	uint32_t counts_per_revolution;
};

/* QDEC run time data */
struct qdec_alif_utimer_data {
	DEVICE_MMIO_NAMED_RAM(global);
	DEVICE_MMIO_NAMED_RAM(timer);
	int32_t position;
};

#define DEV_CFG(_dev) ((const struct qdec_alif_utimer_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct qdec_alif_utimer_data *const)(_dev)->data)

static int qdec_alif_utimer_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct qdec_alif_utimer_config *cfg = DEV_CFG(dev);
	struct qdec_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);
	uint32_t counter_value;

	if ((chan != SENSOR_CHAN_ALL) && (chan != SENSOR_CHAN_ROTATION)) {
		return -ENOTSUP;
	}

	counter_value = alif_utimer_get_counter_value(timer_base);
	data->position = (counter_value * 360) / cfg->counts_per_revolution;

	return 0;
}

static int qdec_alif_utimer_channel_get(const struct device *dev, enum sensor_channel chan,
			struct sensor_value *val)
{
	struct qdec_alif_utimer_data *data = DEV_DATA(dev);

	if (chan != SENSOR_CHAN_ROTATION) {
		return -ENOTSUP;
	}

	val->val1 = data->position;
	val->val2 = 0;
	return 0;
}

static const struct sensor_driver_api qdec_alif_utimer_driver_api = {
	.sample_fetch = qdec_alif_utimer_sample_fetch,
	.channel_get = qdec_alif_utimer_channel_get
};

static int qdec_alif_utimer_init(const struct device *dev)
{
	const struct qdec_alif_utimer_config *cfg = DEV_CFG(dev);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);
	uintptr_t global_base = DEVICE_MMIO_NAMED_GET(dev, global);
	int32_t ret;

	/* apply pin configuration */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	if (cfg->counts_per_revolution < 1) {
		LOG_ERR("Invalid number of counts per revolution should be positive");
		return -EINVAL;
	}

	alif_utimer_enable_timer_clock(global_base, cfg->timer_id);
	alif_utimer_disable_soft_counter_ctrl(timer_base);
	alif_utimer_set_up_counter(timer_base);
	alif_utimer_set_counter_value(timer_base, 0x0);
	alif_utimer_set_counter_reload_value(timer_base, cfg->counts_per_revolution - 1);
	alif_utimer_enable_counter(timer_base);

	if (cfg->filter_enable) {
		alif_utimer_enable_filter(timer_base, cfg->filter_prescaler, cfg->filter_taps);
	}

	alif_utimer_config_qdec_triggers(timer_base);

	return 0;
}

#define CHECK_FILTER_PARAM_VALUES(n)								\
	BUILD_ASSERT((DT_INST_PROP(n, filter_prescaler) < CHAN_FILTER_CTRL_FILTER_PRESCALER_Msk),\
		"UTIMER QDEC filter prescaler value exceeds maximum of "			\
		STRINGIFY(CHAN_FILTER_CTRL_FILTER_PRESCALER_Msk));				\
	BUILD_ASSERT((DT_INST_PROP(n, filter_taps) < CHAN_FILTER_CTRL_FILTER_TAPS_Msk),		\
		"UTIMER QDEC filter taps value exceeds maximum of "				\
		STRINGIFY(CHAN_FILTER_CTRL_FILTER_TAPS_Msk));					\

#define QDEC_ALIF_UTIMER_INIT(n)								\
	PINCTRL_DT_INST_DEFINE(n);								\
	COND_CODE_1(DT_INST_PROP(n, input_filter_enable), (CHECK_FILTER_PARAM_VALUES(n)), ());	\
	static struct qdec_alif_utimer_data qdec_alif_utimer_data_##n;				\
	static const struct qdec_alif_utimer_config qdec_alif_utimer_cfg_##n = {		\
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(global, DT_INST_PARENT(n)),			\
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(timer, DT_INST_PARENT(n)),			\
		.timer_id = DT_PROP(DT_INST_PARENT(n), timer_id),				\
		.counts_per_revolution = DT_INST_PROP(n, counts_per_revolution),		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),					\
		COND_CODE_1(DT_INST_PROP(n, input_filter_enable),				\
		(.filter_enable = DT_INST_PROP(n, input_filter_enable),				\
		.filter_prescaler = DT_INST_PROP(n, filter_prescaler),				\
		.filter_taps = DT_INST_PROP(n, filter_taps)), ())				\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(n,								\
				     qdec_alif_utimer_init,					\
				     NULL,							\
				     &qdec_alif_utimer_data_##n,				\
				     &qdec_alif_utimer_cfg_##n,					\
				     POST_KERNEL,						\
				     CONFIG_SENSOR_INIT_PRIORITY,				\
				     &qdec_alif_utimer_driver_api);

DT_INST_FOREACH_STATUS_OKAY(QDEC_ALIF_UTIMER_INIT)
