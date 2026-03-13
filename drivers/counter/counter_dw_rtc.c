/* Copyright (C) 2024 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_dw_apb_rtc

#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <stdbool.h>
#include "counter_dw_rtc.h"

LOG_MODULE_REGISTER(counter_dw_rtc, CONFIG_COUNTER_LOG_LEVEL);


struct counter_dw_config {
	struct counter_config_info info;
	void (*config_func)(void);
	uint32_t base_address;
	uint32_t load_value;
	uint16_t prescaler;
	uint8_t wrap_enable;
};

struct counter_dw_data {
	counter_alarm_callback_t alarm_cb;
	void *user_data;
	uint32_t ccr_cache;
};

static int counter_dw_start(const struct device *dev)
{
	const struct counter_dw_config *config = dev->config;
	struct counter_dw_data *data = dev->data;

	/* Use cached value - no register read needed */
	if (data->ccr_cache & BIT(DW_RTC_CCR_EN)) {
		LOG_DBG("%p Counter already started", dev);
		return -EALREADY;
	}

	/* Enable counter with cached configuration */
	data->ccr_cache |= BIT(DW_RTC_CCR_EN);
	write_ccr(data->ccr_cache, config->base_address);

	LOG_DBG("%p Counter started", dev);

	return 0;
}

static int counter_dw_stop(const struct device *dev)
{
	const struct counter_dw_config *config = dev->config;
	struct counter_dw_data *data = dev->data;

	if (!(data->ccr_cache & BIT(DW_RTC_CCR_EN))) {
		LOG_DBG("%p Counter already in stopped state", dev);
		return 0;
	}

	data->ccr_cache &= ~BIT(DW_RTC_CCR_EN);
	write_ccr(data->ccr_cache, config->base_address);

	return 0;
}

static int counter_dw_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_dw_config *config = dev->config;

	*ticks = read_ccvr(config->base_address);
	return 0;
}

static int counter_dw_set_alarm(const struct device *dev, uint8_t chan_id,
				const struct counter_alarm_cfg *alarm_cfg)
{
	const struct counter_dw_config *config = dev->config;
	struct counter_dw_data *data = dev->data;
	uint32_t ticks;

	if (chan_id != 0) {
		LOG_ERR("Invalid channel id %u", chan_id);
		return -ENOTSUP;
	}

	if (data->alarm_cb != NULL) {
		return -EBUSY;
	}

	if (alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) {
		ticks = alarm_cfg->ticks;
	} else {
		ticks = read_ccvr(config->base_address) + alarm_cfg->ticks;
	}

	write_cmr(ticks, config->base_address);

	data->alarm_cb = alarm_cfg->callback;
	data->user_data = alarm_cfg->user_data;

	data->ccr_cache |= BIT(DW_RTC_CCR_IEN);
	data->ccr_cache &= ~BIT(DW_RTC_CCR_MASK);

	write_ccr(data->ccr_cache, config->base_address);

	LOG_DBG("%p Counter alarm set to %u ticks", dev, alarm_cfg->ticks);

	return 0;
}

static int counter_dw_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	const struct counter_dw_config *config = dev->config;
	struct counter_dw_data *data = dev->data;

	if (chan_id != 0) {
		LOG_ERR("Invalid channel id %u", chan_id);
		return -ENOTSUP;
	}

	if (data->ccr_cache & BIT(DW_RTC_CCR_IEN)) {
		data->ccr_cache &= ~BIT(DW_RTC_CCR_IEN);
		write_ccr(data->ccr_cache, config->base_address);
	}

	data->alarm_cb = NULL;
	data->user_data = NULL;

	LOG_DBG("%p Counter alarm canceled", dev);

	return 0;
}

static uint32_t counter_dw_get_pending_int(const struct device *dev)
{
	const struct counter_dw_config *config = dev->config;

	return test_bit_stat(config->base_address);
}

static void counter_dw_isr(const struct device *dev)
{
	const struct counter_dw_config *config = dev->config;
	struct counter_dw_data *data = dev->data;
	counter_alarm_callback_t alarm_cb = data->alarm_cb;
	uint32_t ticks;

	/* Single alarm is supported, disable interrupt and callback */
	clear_interrupts(config->base_address);

	data->ccr_cache &= ~BIT(DW_RTC_CCR_IEN);
	write_ccr(data->ccr_cache, config->base_address);

	if (alarm_cb) {
		data->alarm_cb = NULL;
		counter_dw_get_value(dev, &ticks);
		alarm_cb(dev, 0, ticks, data->user_data);
	}
}

static const struct counter_driver_api counter_dw_api = {
		.start = counter_dw_start,
		.stop = counter_dw_stop,
		.get_value = counter_dw_get_value,
		.set_alarm = counter_dw_set_alarm,
		.cancel_alarm = counter_dw_cancel_alarm,
		.get_pending_int = counter_dw_get_pending_int,
};

/**
 * @brief Detect if RTC configuration needs updating
 *
 * Compares current hardware state with desired DTS configuration
 * and returns a bitmask indicating which fields mismatch.
 *
 * @param dev Device pointer
 * @param config Device configuration
 * @param ccr Current CCR register value
 * @param clr Current CLR register value
 * @param cpsr Current CPSR register value (only valid if prescaler != 0)
 * @return Bitmask of DW_RTC_RECONFIG_* flags indicating what needs reconfiguration
 */
static uint32_t dw_rtc_needs_reconfig(const struct device *dev,
				      const struct counter_dw_config *config,
				      uint32_t ccr, uint32_t clr, uint32_t cpsr)
{
	uint32_t reconfig_flags = 0;

	/* Check if load value matches DTS */
	if (clr != config->load_value) {
		reconfig_flags |= DW_RTC_RECONFIG_LOAD_VALUE;
		LOG_DBG("%p Load value mismatch: HW=%u, DTS=%u",
			dev, clr, config->load_value);
	}

	/* Check if prescaler configuration matches DTS */
	if (config->prescaler) {
		/* Prescaler should be enabled with specific value */
		if ((cpsr != config->prescaler) ||
		    !(ccr & BIT(DW_RTC_CCR_PSCLR_EN))) {
			reconfig_flags |= DW_RTC_RECONFIG_PRESCALER;
			LOG_DBG("%p Prescaler mismatch: HW=%u (EN=%d), DTS=%u",
				dev, cpsr,
				!!(ccr & BIT(DW_RTC_CCR_PSCLR_EN)),
				config->prescaler);
		}
	} else {
		/* Prescaler should be disabled */
		if (ccr & BIT(DW_RTC_CCR_PSCLR_EN)) {
			reconfig_flags |= DW_RTC_RECONFIG_PRESCALER;
			LOG_DBG("%p Prescaler should be disabled but is enabled", dev);
		}
	}

	/* Check if wrap_enable matches DTS */
	if (config->wrap_enable) {
		if (!(ccr & BIT(DW_RTC_CCR_WEN))) {
			reconfig_flags |= DW_RTC_RECONFIG_WRAP_ENABLE;
			LOG_DBG("%p Wrap enable mismatch: HW=disabled, DTS=enabled", dev);
		}
	} else {
		if (ccr & BIT(DW_RTC_CCR_WEN)) {
			reconfig_flags |= DW_RTC_RECONFIG_WRAP_ENABLE;
			LOG_DBG("%p Wrap enable mismatch: HW=enabled, DTS=disabled", dev);
		}
	}

	return reconfig_flags;
}

/**
 * @brief Apply RTC configuration to hardware
 *
 * Writes the desired configuration to RTC registers based on the
 * reconfiguration flags. Handles counter enable state preservation.
 *
 * @param dev Device pointer
 * @param config Device configuration
 * @param ccr Pointer to CCR value (will be modified and written to hardware)
 * @param was_enabled True if counter was enabled before reconfiguration
 * @param reconfig_flags Bitmask of DW_RTC_RECONFIG_* flags
 */
static void dw_rtc_apply_config(const struct device *dev,
				const struct counter_dw_config *config,
				uint32_t *ccr, bool was_enabled,
				uint32_t reconfig_flags)
{
	/* Disable counter if enabled */
	if (was_enabled) {
		*ccr &= ~BIT(DW_RTC_CCR_EN);
		write_ccr(*ccr, config->base_address);
	}

	/* Apply load value if needed */
	if (reconfig_flags & DW_RTC_RECONFIG_LOAD_VALUE) {
		write_clr(config->load_value, config->base_address);
	}

	/* Apply prescaler configuration if needed */
	if (reconfig_flags & DW_RTC_RECONFIG_PRESCALER) {
		if (config->prescaler) {
			write_cpsr(config->prescaler, config->base_address);
			*ccr |= BIT(DW_RTC_CCR_PSCLR_EN);
		} else {
			*ccr &= ~BIT(DW_RTC_CCR_PSCLR_EN);
		}
	}

	/* Apply wrap enable if needed */
	if (reconfig_flags & DW_RTC_RECONFIG_WRAP_ENABLE) {
		if (config->wrap_enable) {
			*ccr |= BIT(DW_RTC_CCR_WEN);
		} else {
			*ccr &= ~BIT(DW_RTC_CCR_WEN);
		}
	}

	/* Re-enable if was enabled */
	if (was_enabled) {
		*ccr |= BIT(DW_RTC_CCR_EN);
	}

	/* Write final configuration */
	write_ccr(*ccr, config->base_address);

	LOG_DBG("%p RTC reconfigured: PSCLR=%u (EN=%d), WEN=%d, EN=%d",
		dev, config->prescaler,
		!!(*ccr & BIT(DW_RTC_CCR_PSCLR_EN)),
		!!(*ccr & BIT(DW_RTC_CCR_WEN)),
		!!(*ccr & BIT(DW_RTC_CCR_EN)));
}


static int counter_dw_init(const struct device *dev)
{
	const struct counter_dw_config *config = dev->config;
	struct counter_dw_data *data = dev->data;
	uint32_t ccr, clr, cpsr = 0;
	uint32_t reconfig_flags;
	bool was_enabled;

	data->alarm_cb = NULL;

	/* Read current hardware state */
	ccr = read_ccr(config->base_address);
	clr = read_clr(config->base_address);
	was_enabled = ccr & BIT(DW_RTC_CCR_EN);

	/* Read prescaler value if configured in DTS */
	if (config->prescaler) {
		cpsr = read_cpsr(config->base_address);
	}

	/* Detect configuration mismatches */
	reconfig_flags = dw_rtc_needs_reconfig(dev, config, ccr, clr, cpsr);

	/* Apply configuration if needed */
	if (reconfig_flags) {
		LOG_DBG("%p RTC configuration mismatch detected (0x%x), reconfiguring",
			dev, reconfig_flags);
		dw_rtc_apply_config(dev, config, &ccr, was_enabled, reconfig_flags);
	} else {
		LOG_DBG("%p RTC already configured correctly", dev);
	}

	/* Cache the current CCR state for fast operations */
	data->ccr_cache = ccr;

	config->config_func();

	LOG_DBG("Designware RTC driver initialized on device: %p", dev);
	return 0;
}

#define COUNTER_DW_INIT(inst)							\
	static void counter_dw_irq_config_##inst(void);				\
										\
	static struct counter_dw_data counter_dw_dev_data_##inst;		\
										\
	static struct counter_dw_config counter_dw_dev_config_##inst = {	\
		.info = {							\
			.freq = DT_INST_PROP(inst, clock_frequency) /		\
				(DT_INST_PROP_OR(inst, prescaler, 0) ?: 1),	\
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,			\
			.channels = 1,						\
		},								\
		.config_func = counter_dw_irq_config_##inst,			\
		.base_address = DT_INST_REG_ADDR(inst),				\
		.prescaler = DT_INST_PROP_OR(inst, prescaler, 0),		\
		.load_value = DT_INST_PROP_OR(inst, load_value, 0),		\
		.wrap_enable = DT_INST_PROP_OR(inst, wrap_enable, 0),		\
	};									\
										\
	DEVICE_DT_INST_DEFINE(inst, counter_dw_init, NULL,			\
			      &counter_dw_dev_data_##inst,			\
			      &counter_dw_dev_config_##inst, POST_KERNEL,	\
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_dw_api);	\
										\
	static void counter_dw_irq_config_##inst(void)				\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),	\
			    counter_dw_isr, DEVICE_DT_INST_GET(inst), 0);	\
		irq_enable(DT_INST_IRQN(inst));					\
	}

DT_INST_FOREACH_STATUS_OKAY(COUNTER_DW_INIT)
