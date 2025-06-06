/*
 * Copyright (c) 2024 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_pwm

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>

#include "utimer.h"
#include <zephyr/dt-bindings/timer/alif_utimer.h>

LOG_MODULE_REGISTER(pwm_alif_utimer, CONFIG_PWM_LOG_LEVEL);

#define NUM_CHANNELS  2U

/** PWM configuration */
struct pwm_alif_utimer_config {
	DEVICE_MMIO_NAMED_ROM(global);
	DEVICE_MMIO_NAMED_ROM(timer);
	uint32_t counterdirection;
	const uint8_t timer_id;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clk_dev;
	clock_control_subsys_t clkid;
};

/** PWM runtime data configuration */
struct pwm_alif_utimer_data {
	DEVICE_MMIO_NAMED_RAM(global);
	DEVICE_MMIO_NAMED_RAM(timer);
	uint32_t prev_period;
	uint32_t prev_pulse;
	uint32_t frequency;
	pwm_flags_t prev_flags;
};

#define DEV_CFG(_dev) ((const struct pwm_alif_utimer_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct pwm_alif_utimer_data *const)(_dev)->data)

static void utimer_set_direction(uint32_t reg_base, uint8_t direction)
{
	switch (direction) {
	case ALIF_UTIMER_COUNTER_DIRECTION_UP:
		alif_utimer_set_up_counter(reg_base);
		break;
	case ALIF_UTIMER_COUNTER_DIRECTION_DOWN:
		alif_utimer_set_down_counter(reg_base);
		break;
	case ALIF_UTIMER_COUNTER_DIRECTION_TRIANGLE:
		alif_utimer_set_triangular_counter(reg_base);
		break;
	default:
		LOG_ERR("invalid counter-direction");
	}
}

static uint32_t utimer_get_driver_config(uint8_t direction, pwm_flags_t flags)
{
	uint32_t config_val;

	switch (direction) {
	case ALIF_UTIMER_COUNTER_DIRECTION_UP:
	case ALIF_UTIMER_COUNTER_DIRECTION_DOWN:
		config_val = (flags & PWM_POLARITY_INVERTED) ?
			(COMPARE_CTRL_DRV_HIGH_AT_COMP_MATCH | COMPARE_CTRL_DRV_LOW_AT_CYCLE_END) :
			(COMPARE_CTRL_DRV_LOW_AT_COMP_MATCH | COMPARE_CTRL_DRV_HIGH_AT_CYCLE_END);
		break;
	case ALIF_UTIMER_COUNTER_DIRECTION_TRIANGLE:
		config_val = (flags & PWM_POLARITY_INVERTED) ?
		(COMPARE_CTRL_DRV_TOGGLE_AT_COMP_MATCH | COMPARE_CTRL_DRV_RETAIN_VAL_AT_CYCLE_END |
					COMPARE_CTRL_DRV_START_VAL_HIGH) :
		(COMPARE_CTRL_DRV_TOGGLE_AT_COMP_MATCH | COMPARE_CTRL_DRV_RETAIN_VAL_AT_CYCLE_END |
					COMPARE_CTRL_DRV_START_VAL_LOW);
		break;
	default:
		config_val = 0;
		LOG_ERR("invalid counter-direction");
	}
	return config_val;
}

static int pwm_alif_utimer_set_cycles(const struct device *dev, uint32_t channel,
			 uint32_t period_cycles,
			 uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_alif_utimer_config *cfg = DEV_CFG(dev);
	struct pwm_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);
	uintptr_t global_base = DEVICE_MMIO_NAMED_GET(dev, global);
	uint32_t value;

	if (channel > NUM_CHANNELS) {
		LOG_ERR("Invalid channel number");
		return -EINVAL;
	}

	value = utimer_get_driver_config(cfg->counterdirection, flags);

	/* disable driver output if period or pulse is zero */
	if (period_cycles == 0U || pulse_cycles == 0U) {
		alif_utimer_disable_driver(timer_base, channel);
		if (flags == PWM_POLARITY_INVERTED) {
			/* make driver disable state as high */
			alif_utimer_set_driver_disable_val_high(timer_base, channel);
		} else {
			alif_utimer_set_driver_disable_val_low(timer_base, channel);
		}

		data->prev_period = 0;
		return 0;
	}

	if (flags != data->prev_flags) {
		value = utimer_get_driver_config(cfg->counterdirection, flags);
		alif_utimer_config_driver_output(timer_base, channel, value);

		data->prev_flags = flags;
	}

	/* update period value if it as been changed */
	if (period_cycles != data->prev_period) {
		/* set period value */
		alif_utimer_set_counter_reload_value(timer_base, period_cycles);

		data->prev_period = period_cycles;
	}

	/* update pulse value if it as been changed */
	if (pulse_cycles != data->prev_pulse) {
		/* set pulse value */
		alif_utimer_set_compare_value(timer_base, channel, pulse_cycles);

		data->prev_pulse = pulse_cycles;
	}

	/* enable channel if not enabled */
	if (!alif_utimer_driver_enabled(timer_base, channel)) {
		alif_utimer_config_driver_output(timer_base, channel, value);
		alif_utimer_enable_driver_output(global_base, channel, cfg->timer_id);
		alif_utimer_enable_driver(timer_base, channel);
	}

	/* enable channel compare match if not enabled */
	if (!alif_utimer_comp_match_enabled(timer_base, channel)) {
		alif_utimer_enable_compare_match(timer_base, channel);
	}

	/* start the counter not started */
	if (!alif_utimer_counter_running(global_base, cfg->timer_id)) {
		alif_utimer_start_counter(global_base, cfg->timer_id);
	}

	return 0;
}

static int pwm_alif_utimer_get_cycles_per_sec(const struct device *dev, uint32_t channel,
				 uint64_t *cycles)
{
	struct pwm_alif_utimer_data *data = DEV_DATA(dev);

	ARG_UNUSED(channel);

	*cycles = data->frequency;
	return 0;
}

static const struct pwm_driver_api pwm_alif_utimer_driver_api = {
	.set_cycles = pwm_alif_utimer_set_cycles,
	.get_cycles_per_sec = pwm_alif_utimer_get_cycles_per_sec,
};

static int pwm_alif_utimer_init(const struct device *dev)
{
	const struct pwm_alif_utimer_config *cfg = DEV_CFG(dev);
	struct pwm_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);
	uintptr_t global_base = DEVICE_MMIO_NAMED_GET(dev, global);
	int32_t ret;

	/* apply pin configuration */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* get clock rate from clock manager */
	if (!device_is_ready(cfg->clk_dev)) {
		LOG_ERR("clock controller device not ready");
		return -ENODEV;
	}
	ret = clock_control_get_rate(cfg->clk_dev,
			cfg->clkid, &data->frequency);
	if (ret != 0) {
		LOG_ERR("Unable to get clock rate: err:%d", ret);
		return ret;
	}

	/* disable for timer outputs */
	alif_utimer_disable_timer_output(global_base, cfg->timer_id);

	/* enable timer clock */
	alif_utimer_enable_timer_clock(global_base, cfg->timer_id);

	/* program enable for start, stop and clear counter */
	alif_utimer_enable_soft_counter_ctrl(timer_base);

	/* set counter direction */
	utimer_set_direction(timer_base, cfg->counterdirection);

	/* enable timer counter */
	alif_utimer_enable_counter(timer_base);

	data->prev_flags = PWM_POLARITY_NORMAL;

	return 0;
}

/* Device Instantiation */
#define PWM_ALIF_UTIMER_INIT(n)									\
	PINCTRL_DT_INST_DEFINE(n);								\
	static struct pwm_alif_utimer_data pwm_alif_utimer_data_##n;				\
	static const struct pwm_alif_utimer_config pwm_alif_utimer_cfg_##n = {			\
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(global, DT_INST_PARENT(n)),			\
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(timer, DT_INST_PARENT(n)),			\
		.counterdirection = DT_PROP(DT_INST_PARENT(n), counter_direction),		\
		.timer_id = DT_PROP(DT_INST_PARENT(n), timer_id),				\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),					\
		.clk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(n))),			\
		.clkid = (clock_control_subsys_t)DT_CLOCKS_CELL(DT_INST_PARENT(n), clkid),	\
	};											\
												\
	DEVICE_DT_INST_DEFINE(n,								\
			      &pwm_alif_utimer_init,						\
			      NULL,								\
			      &pwm_alif_utimer_data_##n,					\
			      &pwm_alif_utimer_cfg_##n,						\
			      POST_KERNEL,							\
			      CONFIG_PWM_INIT_PRIORITY,						\
			      &pwm_alif_utimer_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_ALIF_UTIMER_INIT)
