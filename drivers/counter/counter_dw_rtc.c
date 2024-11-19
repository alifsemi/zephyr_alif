/* Copyright (C) 2024 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_dw_apb_rtc

#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(counter_dw_rtc, CONFIG_COUNTER_LOG_LEVEL);

#include <zephyr/drivers/counter.h>
#include <errno.h>
#include <stdbool.h>
#include "counter_dw_rtc.h"


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
};


static int counter_dw_start(const struct device *dev)
{
	const struct counter_dw_config *config = dev->config;
	uint32_t reg_value;

	reg_value = read_ccr(config->base_address);

	if ((reg_value & (1 << DW_RTC_CCR_EN))) {
		return -EALREADY;
	}

	write_clr(config->load_value, config->base_address);

	if (config->prescaler) {
		write_cpsr(config->prescaler, config->base_address);
		reg_value |= (1 << DW_RTC_CCR_PSCLR_EN);
	}

	if (config->wrap_enable) {
		reg_value |= (1 << DW_RTC_CCR_WEN);
	}

	reg_value |= (1 << DW_RTC_CCR_EN);
	write_ccr(reg_value, config->base_address);
	LOG_DBG("%p Counter started", dev);

	return 0;
}

static int counter_dw_stop(const struct device *dev)
{
	const struct counter_dw_config *config = dev->config;
	uint32_t reg_value;

	reg_value = read_ccr(config->base_address);

	if (!(reg_value & (1 << DW_RTC_CCR_EN))) {
		LOG_DBG("%p Counter already in stopped state", dev);
		return 0;
	}

	reg_value &= ~(1 << DW_RTC_CCR_EN);
	write_ccr(reg_value, config->base_address);
	LOG_DBG("%p Counter stopped", dev);

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
	uint32_t reg_value;
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

	reg_value = read_ccr(config->base_address);
	reg_value |= (1 << DW_RTC_CCR_IEN);
	reg_value &= ~(1 << DW_RTC_CCR_MASK);

	write_ccr(reg_value, config->base_address);

	LOG_DBG("%p Counter alarm set to %u ticks", dev, alarm_cfg->ticks);

	return 0;
}

static int counter_dw_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	const struct counter_dw_config *config = dev->config;
	struct counter_dw_data *data = dev->data;
	uint32_t reg_value;

	if (chan_id != 0) {
		LOG_ERR("Invalid channel id %u", chan_id);
		return -ENOTSUP;
	}

	reg_value = read_ccr(config->base_address);
	reg_value &= ~(1 << DW_RTC_CCR_IEN);

	write_ccr(reg_value, config->base_address);
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
	uint32_t reg_value;

	LOG_DBG("%p Counter ISR", dev);

	/* Single alarm is supported, disable interrupt and callback */
	clear_interrupts(config->base_address);
	reg_value = read_ccr(config->base_address);
	reg_value &= ~(1 << DW_RTC_CCR_IEN);

	write_ccr(reg_value, config->base_address);

	counter_dw_get_value(dev, &ticks);

	if (alarm_cb) {
		data->alarm_cb = NULL;
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

static int counter_dw_init(const struct device *dev)
{
	const struct counter_dw_config *config = dev->config;

	counter_dw_stop(dev);
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
			(DT_INST_PROP(inst, prescaler) ?			\
			DT_INST_PROP(inst, prescaler):1),			\
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,			\
			.channels = 1,						\
		},								\
										\
		.config_func = counter_dw_irq_config_##inst,			\
		.base_address = DT_INST_REG_ADDR(inst),				\
		.prescaler = DT_INST_PROP(inst, prescaler),			\
		.load_value = DT_INST_PROP(inst, load_value),			\
		.wrap_enable = DT_INST_PROP(inst, wrap_enable),			\
	};									\
										\
	DEVICE_DT_INST_DEFINE(inst,						\
			    counter_dw_init,					\
			    NULL,						\
			    &counter_dw_dev_data_##inst,			\
			    &counter_dw_dev_config_##inst,			\
			    POST_KERNEL,					\
			    CONFIG_COUNTER_INIT_PRIORITY,			\
			    &counter_dw_api);					\
										\
	static void counter_dw_irq_config_##inst(void)				\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(inst),					\
			    DT_INST_IRQ(inst, priority),			\
			    counter_dw_isr,					\
			    DEVICE_DT_INST_GET(inst), 0);			\
		irq_enable(DT_INST_IRQN(inst));					\
	}

DT_INST_FOREACH_STATUS_OKAY(COUNTER_DW_INIT)
