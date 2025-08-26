/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_utimer_counter

#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>

#include "utimer.h"
#include <zephyr/dt-bindings/timer/alif_utimer.h>

LOG_MODULE_REGISTER(counter_alif_utimer, CONFIG_COUNTER_LOG_LEVEL);

#define NUM_CHANNELS   2U

struct counter_alif_utimer_ch_data {
	counter_alarm_callback_t alarm_cb;
	void *alarm_user_data;
};

struct counter_alif_utimer_data {
	DEVICE_MMIO_NAMED_RAM(global);
	DEVICE_MMIO_NAMED_RAM(timer);
	uint32_t guard_period;
	uint32_t frequency;
	counter_top_callback_t top_cb;
	void *top_user_data;
	atomic_t cc_int_pending;
	struct counter_alif_utimer_ch_data alarm[NUM_CHANNELS];
};

struct counter_alif_utimer_config {
	DEVICE_MMIO_NAMED_ROM(global);
	DEVICE_MMIO_NAMED_ROM(timer);
	struct counter_config_info counter_info;
	const uint8_t timer_id;
	uint32_t counterdirection;
	const struct device *clk_dev;
	clock_control_subsys_t clkid;
	void (*irq_config)(const struct device *dev);
	void (*set_irq_pending)(uint8_t interrupt);
	uint32_t (*get_irq_pending)(uint8_t interrupt);
};

#define DEV_CFG(_dev) ((const struct counter_alif_utimer_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct counter_alif_utimer_data *const)(_dev)->data)

static int utimer_set_direction(uint32_t reg_base, uint8_t direction)
{
	switch (direction) {
	case ALIF_UTIMER_COUNTER_DIRECTION_UP:
		alif_utimer_set_up_counter(reg_base);
		break;
	case ALIF_UTIMER_COUNTER_DIRECTION_DOWN:
		alif_utimer_set_down_counter(reg_base);
		break;
	case ALIF_UTIMER_COUNTER_DIRECTION_TRIANGLE:
	default:
		return -EINVAL;
	}

	return 0;
}

static int counter_alif_utimer_start(const struct device *dev)
{
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);
	uintptr_t global_base = DEVICE_MMIO_NAMED_GET(dev, global);

	/* start the timer counter*/
	alif_utimer_start_counter(global_base, cfg->timer_id);
	return 0;
}

static int counter_alif_utimer_stop(const struct device *dev)
{
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);
	uintptr_t global_base = DEVICE_MMIO_NAMED_GET(dev, global);

	/* stop the timer counter */
	alif_utimer_stop_counter(global_base, cfg->timer_id);

	return 0;
}

static int counter_alif_utimer_get_value(const struct device *dev,
					uint32_t *ticks)
{
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);

	*ticks = alif_utimer_get_counter_value(timer_base);

	return 0;
}

static uint32_t counter_alif_utimer_get_top_value(const struct device *dev)
{
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);

	return alif_utimer_get_counter_reload_value(timer_base);
}

static uint32_t counter_alif_utimer_get_pending_int(const struct device *dev)
{
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);

	return alif_utimer_get_pending_interrupt(timer_base);
}

static uint32_t ticks_add(uint32_t val1, uint32_t val2, uint32_t top)
{
	uint32_t to_top;

	if (IS_BIT_MASK(top)) {
		return (val1 + val2) & top;
	}

	to_top = top - val1;

	return (val2 <= to_top) ? val1 + val2 : val2 - to_top;
}

static uint32_t ticks_sub(uint32_t val, uint32_t old, uint32_t top)
{
	if (IS_BIT_MASK(top)) {
		return (val - old) & top;
	}

	/* if top is not 2^n-1 */
	return (val >= old) ? (val - old) : val + top + 1 - old;
}

static void set_cc_int_pending(const struct device *dev, uint8_t chan)
{
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);
	struct counter_alif_utimer_data *data = DEV_DATA(dev);

	atomic_or(&data->cc_int_pending, BIT(chan));
	cfg->set_irq_pending(chan);
}

static int set_compare(const struct device *dev, uint8_t chan, uint32_t val,
		  uint32_t flags)
{
	struct counter_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);

	__ASSERT_NO_MSG(data->guard_period < counter_alif_utimer_get_top_value(dev));
	bool absolute = flags & COUNTER_ALARM_CFG_ABSOLUTE;
	bool irq_on_late;
	uint32_t top = counter_alif_utimer_get_top_value(dev);
	uint32_t evt_bit = chan;
	uint32_t now, diff, max_rel_val;
	int err = 0;

	__ASSERT(alif_utimer_check_interrupt_enabled(dev, evt_bit) == 0,
				"Expected that CC interrupt is disabled.");

	alif_utimer_enable_compare_match(timer_base, chan);

	/* First take care of a risk of an event coming from CC being set to
	 * next tick. Reconfigure CC to future (now tick is the furthest
	 * future).
	 */
	now = alif_utimer_get_counter_value(timer_base);
	alif_utimer_set_compare_value(timer_base, chan, now);
	alif_utimer_clear_interrupt(timer_base, evt_bit);

	if (absolute) {
		max_rel_val = top - data->guard_period;
		irq_on_late = flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE;
	} else {
		/* If relative value is smaller than half of the counter range
		 * it is assumed that there is a risk of setting value too late
		 * and late detection algorithm must be applied. When late
		 * setting is detected, interrupt shall be triggered for
		 * immediate expiration of the timer. Detection is performed
		 * by limiting relative distance between CC and counter.
		 *
		 * Note that half of counter range is an arbitrary value.
		 */
		irq_on_late = val < (top / 2);
		/* limit max to detect short relative being set too late. */
		max_rel_val = irq_on_late ? top / 2 : top;
		val = ticks_add(now, val, top);
	}

	alif_utimer_set_compare_value(timer_base, chan, val);

	/* decrement value to detect also case when
	 * val == alif_utimer_get_counter_value(dev). Otherwise,
	 * condition would need to include comparing diff against 0.
	 */
	diff = ticks_sub(val - 1, alif_utimer_get_counter_value(timer_base), top);
	if (diff > max_rel_val) {
		if (absolute) {
			err = -ETIME;
		}

		/* Interrupt is triggered always for relative alarm and
		 * for absolute depending on the flag.
		 */
		if (irq_on_late) {
			set_cc_int_pending(dev, chan);
		} else {
			data->alarm[chan].alarm_cb = NULL;
		}
	} else {
		alif_utimer_enable_interrupt(timer_base, evt_bit);
	}

	return err;
}

static int counter_alif_utimer_set_alarm(const struct device *dev, uint8_t chan,
			const struct counter_alarm_cfg *alarm_cfg)
{
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);
	struct counter_alif_utimer_data *data = DEV_DATA(dev);
	struct counter_alif_utimer_ch_data *chdata = &data->alarm[chan];

	if (chan > cfg->counter_info.channels) {
		LOG_ERR("Invalid counter channel number");
		return -EINVAL;
	}

	if (alarm_cfg->ticks > counter_alif_utimer_get_top_value(dev)) {
		LOG_ERR("Invalid tick value");
		return -EINVAL;
	}

	if (chdata->alarm_cb) {
		LOG_ERR("Counter is busy");
		return -EBUSY;
	}

	chdata->alarm_cb = alarm_cfg->callback;
	chdata->alarm_user_data = alarm_cfg->user_data;

	return set_compare(dev, chan, alarm_cfg->ticks, alarm_cfg->flags);
}

static int counter_alif_utimer_cancel_alarm(const struct device *dev, uint8_t chan)
{
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);
	struct counter_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);
	struct counter_alif_utimer_ch_data *chdata = &data->alarm[chan];
	uint8_t evt_bit = chan;

	if (chan > cfg->counter_info.channels) {
		LOG_ERR("Invalid counter channel number");
		return -EINVAL;
	}

	alif_utimer_disable_compare_match(timer_base, chan);
	alif_utimer_disable_interrupt(timer_base, evt_bit);
	chdata->alarm_cb = NULL;
	return 0;
}

static int counter_alif_utimer_set_top_value(const struct device *dev,
			const struct counter_top_cfg *cfg)
{
	const struct counter_alif_utimer_config *config = DEV_CFG(dev);
	struct counter_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);
	int err = 0;

	for (int i = 0; i < config->counter_info.channels; i++) {
		/* Overflow can be changed only when all alarms are
		 * disabled.
		 */
		if (data->alarm[i].alarm_cb) {
			LOG_ERR("Counter is busy");
			return -EBUSY;
		}
	}

	if (cfg->flags & COUNTER_CONFIG_INFO_COUNT_UP) {
		alif_utimer_set_up_counter(timer_base);
	} else {
		alif_utimer_set_down_counter(timer_base);
	}

	alif_utimer_disable_interrupt(timer_base, CHAN_INTERRUPT_OVER_FLOW_BIT);
	alif_utimer_set_counter_reload_value(timer_base, cfg->ticks);
	alif_utimer_clear_interrupt(timer_base, CHAN_INTERRUPT_OVER_FLOW_BIT);

	data->top_cb = cfg->callback;
	data->top_user_data = cfg->user_data;

	if (!(cfg->flags & COUNTER_TOP_CFG_DONT_RESET)) {
		alif_utimer_set_counter_value(timer_base, 0x0);
	} else if (alif_utimer_get_counter_value(timer_base) >= cfg->ticks) {
		err = -ETIME;
		if (cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) {
			alif_utimer_set_counter_value(timer_base, 0x0);
		}
	}

	if (cfg->callback) {
		alif_utimer_enable_interrupt(timer_base, CHAN_INTERRUPT_OVER_FLOW_BIT);
	}

	return err;
}

static int counter_alif_utimer_set_guard_period(const struct device *dev, uint32_t guard,
						uint32_t flags)
{
	struct counter_alif_utimer_data *data = DEV_DATA(dev);

	ARG_UNUSED(flags);

	if (guard > counter_alif_utimer_get_top_value(dev)) {
		LOG_ERR("Invalid Ticks value");
		return -EINVAL;
	}

	data->guard_period = guard;
	return 0;
}

static uint32_t counter_alif_utimer_get_frequency(const struct device *dev)
{
	struct counter_alif_utimer_data *data = DEV_DATA(dev);

	return data->frequency;
}

static uint32_t counter_alif_utimer_get_guard_period(const struct device *dev, uint32_t flags)
{
	struct counter_alif_utimer_data *data = DEV_DATA(dev);

	ARG_UNUSED(flags);
	return data->guard_period;
}

static void top_irq_handle(const struct device *dev)
{
	struct counter_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);

	counter_top_callback_t cb = data->top_cb;

	if ((alif_utimer_get_pending_interrupt(timer_base) & CHAN_INTERRUPT_OVER_FLOW) &&
		alif_utimer_check_interrupt_enabled(timer_base,
				CHAN_INTERRUPT_OVER_FLOW_BIT)) {
		alif_utimer_clear_interrupt(timer_base, CHAN_INTERRUPT_OVER_FLOW_BIT);
		__ASSERT(cb != NULL, "top event enabled - expecting callback");
		cb(dev, data->top_user_data);
	}
}

static void alarm_irq_handle(const struct device *dev, uint32_t chan)
{
	struct counter_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);
	struct counter_alif_utimer_ch_data *alarm = &data->alarm[chan];
	counter_alarm_callback_t cb;
	uint8_t evt_bit = chan;
	bool hw_irq_pending = ((alif_utimer_get_pending_interrupt(timer_base) &
		BIT(evt_bit)) && alif_utimer_check_interrupt_enabled(timer_base, evt_bit));
	bool sw_irq_pending = (data->cc_int_pending & evt_bit);

	if (hw_irq_pending || sw_irq_pending) {
		alif_utimer_clear_interrupt(timer_base, evt_bit);
		atomic_and(&data->cc_int_pending, ~BIT(evt_bit));
		alif_utimer_disable_interrupt(timer_base, evt_bit);

		cb = alarm->alarm_cb;
		alarm->alarm_cb = NULL;

		if (cb) {
			cb(dev, chan, alif_utimer_get_counter_value(timer_base),
					alarm->alarm_user_data);
		}
	}
}

static void counter_irq_handler(const void *arg)
{
	const struct device *dev = arg;
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);

	top_irq_handle(dev);

	for (uint8_t i = 0; i < cfg->counter_info.channels; i++) {
		alarm_irq_handle(dev, i);
	}
}

static int counter_alif_utimer_init(const struct device *dev)
{
	int32_t ret;
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);
	struct counter_alif_utimer_data *data = DEV_DATA(dev);

	DEVICE_MMIO_NAMED_MAP(dev, timer, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, global, K_MEM_CACHE_NONE);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);
	uintptr_t global_base = DEVICE_MMIO_NAMED_GET(dev, global);

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

	alif_utimer_enable_timer_clock(global_base, cfg->timer_id);
	ret = utimer_set_direction(timer_base, cfg->counterdirection);
	if (ret != 0) {
		LOG_ERR("Invalid Counter Direction: err:%d", ret);
		return ret;
	}

	alif_utimer_enable_soft_counter_ctrl(timer_base);
	alif_utimer_set_counter_reload_value(timer_base, cfg->counter_info.max_top_value);
	alif_utimer_enable_counter(timer_base);

	cfg->irq_config(dev);

	return 0;
}

static DEVICE_API(counter, counter_alif_utimer_api) = {
	.start = counter_alif_utimer_start,
	.stop = counter_alif_utimer_stop,
	.get_value = counter_alif_utimer_get_value,
	.set_alarm = counter_alif_utimer_set_alarm,
	.cancel_alarm = counter_alif_utimer_cancel_alarm,
	.set_top_value = counter_alif_utimer_set_top_value,
	.get_pending_int = counter_alif_utimer_get_pending_int,
	.get_top_value = counter_alif_utimer_get_top_value,
	.get_freq = counter_alif_utimer_get_frequency,
	.get_guard_period = counter_alif_utimer_get_guard_period,
	.set_guard_period = counter_alif_utimer_set_guard_period
};

#define TIMER(x)	DT_INST_PARENT(x)

#define COUNTER_ALIF_UTIMER(n)                                                                  \
	static void counter_utimer##n##_irq_config(const struct device *dev)                    \
	{                                                                                       \
		IRQ_CONNECT(DT_IRQ_BY_NAME(TIMER(n), comp_capt_a, irq),                         \
					DT_IRQ_BY_NAME(TIMER(n), comp_capt_a, priority),        \
					counter_irq_handler,                                    \
					DEVICE_DT_INST_GET(n),                                  \
					0);                                                     \
		irq_enable(DT_IRQ_BY_NAME(TIMER(n), comp_capt_a, irq));                         \
		IRQ_CONNECT(DT_IRQ_BY_NAME(TIMER(n), comp_capt_b, irq),                         \
					DT_IRQ_BY_NAME(TIMER(n), comp_capt_b, priority),        \
					counter_irq_handler,                                    \
					DEVICE_DT_INST_GET(n),                                  \
					0);                                                     \
		irq_enable(DT_IRQ_BY_NAME(TIMER(n), comp_capt_b, irq));                         \
		IRQ_CONNECT(DT_IRQ_BY_NAME(TIMER(n), overflow, irq),                            \
					DT_IRQ_BY_NAME(TIMER(n), overflow, priority),           \
					counter_irq_handler,                                    \
					DEVICE_DT_INST_GET(n),                                  \
					0);                                                     \
		irq_enable(DT_IRQ_BY_NAME(TIMER(n), overflow, irq));                            \
	}                                                                                       \
	static void set_irq_pending_##n(uint8_t chan)                                           \
	{                                                                                       \
		if (chan) {                                                                     \
			NVIC_SetPendingIRQ(DT_IRQ_BY_NAME(TIMER(n), comp_capt_b, irq));         \
		} else {                                                                        \
			NVIC_SetPendingIRQ(DT_IRQ_BY_NAME(TIMER(n), comp_capt_a, irq));         \
		}                                                                               \
	}                                                                                       \
	static uint32_t get_irq_pending_##n(uint8_t chan)                                       \
	{                                                                                       \
		if (chan) {                                                                     \
			return NVIC_GetPendingIRQ(DT_IRQ_BY_NAME(TIMER(n), comp_capt_b, irq));  \
		} else {                                                                        \
			return NVIC_GetPendingIRQ(DT_IRQ_BY_NAME(TIMER(n), comp_capt_a, irq));  \
		}                                                                               \
	}                                                                                       \
	static struct counter_alif_utimer_data counter_alif_utimer_data_##n;                    \
	static const struct counter_alif_utimer_config counter_alif_utimer_cfg_##n = {          \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(global, DT_INST_PARENT(n)),	                \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(timer, DT_INST_PARENT(n)),		        \
		.counter_info = {                                                               \
			.max_top_value = UINT32_MAX,                                            \
			.flags = ((DT_PROP(TIMER(n), counter_direction) ==                      \
					 ALIF_UTIMER_COUNTER_DIRECTION_UP) ?                    \
					 COUNTER_CONFIG_INFO_COUNT_UP : 0),                     \
			.channels = NUM_CHANNELS,                                               \
		},                                                                              \
		.timer_id = DT_PROP(TIMER(n), timer_id),                                        \
		.counterdirection = DT_PROP(TIMER(n), counter_direction),                       \
		.clk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(TIMER(n))),                             \
		.clkid = (clock_control_subsys_t)DT_CLOCKS_CELL(TIMER(n), clkid),               \
		.irq_config = counter_utimer##n##_irq_config,                                   \
		.set_irq_pending = set_irq_pending_##n,                                         \
		.get_irq_pending = get_irq_pending_##n,                                         \
	};                                                                                      \
                                                                                                \
	DEVICE_DT_INST_DEFINE(n,                                                                \
			      &counter_alif_utimer_init,                                        \
			      NULL,                                                             \
			      &counter_alif_utimer_data_##n,                                    \
			      &counter_alif_utimer_cfg_##n,                                     \
			      PRE_KERNEL_1,                                                     \
			      CONFIG_COUNTER_INIT_PRIORITY,                                     \
			      &counter_alif_utimer_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_ALIF_UTIMER);
