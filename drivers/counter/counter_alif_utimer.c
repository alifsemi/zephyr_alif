/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_utimer_counter

#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "utimer.h"
#include <zephyr/dt-bindings/timer/alif_utimer.h>

LOG_MODULE_REGISTER(counter_alif_utimer, CONFIG_COUNTER_LOG_LEVEL);

#define NUM_CHANNELS   2U

struct counter_alif_utimer_ch_data {
	counter_alarm_callback_t alarm_cb;
	void *alarm_user_data;
#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
	/* DMA armed: compare/trig active or REQ may be high until cancel. */
	bool dma_armed;
	/* cancel_alarm re-arm in progress (lock dropped during force-low poll). */
	bool dma_rearming;
#endif
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
	/* Tracks whether the counter was running before a PM suspend, so that
	 * pm_resume can correctly restart it only if it was actually running.
	 */
	bool running;
	uint32_t cached_top;
#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
	struct k_spinlock dma_lock;
#endif
};

struct counter_alif_utimer_config {
	struct counter_config_info counter_info;
	DEVICE_MMIO_NAMED_ROM(global);
	DEVICE_MMIO_NAMED_ROM(timer);
	const uint8_t timer_id;
	uint32_t counterdirection;
	const struct device *clk_dev;
	clock_control_subsys_t clkid;
	void (*irq_config)(const struct device *dev);
	void (*set_irq_pending)(uint8_t interrupt);
	uint32_t (*get_irq_pending)(uint8_t interrupt);
#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
	/* DT dma-trig: NVIC off, no CHAN_INTERRUPT clear in ISR */
	bool dma_trig;
#endif
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
	struct counter_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t global_base = DEVICE_MMIO_NAMED_GET(dev, global);

	/* start the timer counter*/
	alif_utimer_start_counter(global_base, cfg->timer_id);
	data->running = true;
	return 0;
}

static int counter_alif_utimer_stop(const struct device *dev)
{
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);
	struct counter_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t global_base = DEVICE_MMIO_NAMED_GET(dev, global);

	/* stop the timer counter */
	alif_utimer_stop_counter(global_base, cfg->timer_id);
	data->running = false;

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

	/* Counter range [0, top]: wrap uses (top + 1) modulus, not top. */
	return (val2 <= to_top) ? val1 + val2 : (val2 - to_top - 1U);
}

static uint32_t ticks_sub(uint32_t val, uint32_t old, uint32_t top)
{
	if (IS_BIT_MASK(top)) {
		return (val - old) & top;
	}

	/* if top is not 2^n-1 */
	return (val >= old) ? (val - old) : val + top + 1 - old;
}

#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
#define UTIMER_DMA_FORCE_MIN_US			2U
/* Extra margin on force-low poll timeout (see utimer_dma_force_req_low). */
#define UTIMER_DMA_FORCE_TIMEOUT_MARGIN_US	20U

#define UTIMER_DMA_REARM_FORCE_LOW	IS_ENABLED(CONFIG_COUNTER_ALIF_UTIMER_DMA_REARM_FORCE_LOW)
#define UTIMER_DMA_REARM_IDLE_LOW	IS_ENABLED(CONFIG_COUNTER_ALIF_UTIMER_DMA_REARM_IDLE_LOW)
#define UTIMER_DMA_USE_HW_CLEAR		IS_ENABLED(CONFIG_COUNTER_ALIF_UTIMER_DMA_HW_CLEAR_ENABLE)

#if !UTIMER_DMA_REARM_FORCE_LOW && !UTIMER_DMA_REARM_IDLE_LOW
#error "Select a UTIMER DMA re-arm mode (FORCE_LOW or IDLE_LOW)"
#endif

#if UTIMER_DMA_REARM_FORCE_LOW
/* UTIMER_GLB_DRIVER_OEN is shared by all timer instances on the block. */
static struct k_spinlock utimer_glb_oen_lock;
#endif

static uintptr_t utimer_compare_ctrl(uintptr_t timer_base, uint8_t chan)
{
	return chan ? UTIMER_COMPARE_CTRL_B(timer_base)
		    : UTIMER_COMPARE_CTRL_A(timer_base);
}

static void utimer_set_dma_compare(uintptr_t timer_base, uint8_t chan, uint32_t val)
{
	if (chan) {
		sys_write32(val, UTIMER_COMPARE_B(timer_base));
		sys_write32(val, UTIMER_COMPARE_B_BUF1(timer_base));
		sys_write32(val, UTIMER_COMPARE_B_BUF2(timer_base));
	} else {
		sys_write32(val, UTIMER_COMPARE_A(timer_base));
		sys_write32(val, UTIMER_COMPARE_A_BUF1(timer_base));
		sys_write32(val, UTIMER_COMPARE_A_BUF2(timer_base));
	}
}

static uint32_t utimer_dma_lead_ticks(uint32_t frequency)
{
	uint32_t ticks = (uint32_t)((uint64_t)frequency * UTIMER_DMA_FORCE_MIN_US /
				    USEC_PER_SEC);

	return MAX(64U, ticks);
}

static uint32_t utimer_ticks_ahead(const struct device *dev, uint32_t now,
				   uint32_t lead, uint32_t top)
{
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);

	if (cfg->counterdirection == ALIF_UTIMER_COUNTER_DIRECTION_DOWN) {
		return ticks_sub(now, lead, top);
	}

	return ticks_add(now, lead, top);
}

static uint32_t utimer_ticks_until(const struct device *dev, uint32_t target,
				   uint32_t now, uint32_t top)
{
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);

	if (cfg->counterdirection == ALIF_UTIMER_COUNTER_DIRECTION_DOWN) {
		return ticks_sub(now, target, top);
	}

	return ticks_sub(target, now, top);
}

static void utimer_dma_disarm(uintptr_t timer_base, uint8_t chan)
{
	uintptr_t ctrl = utimer_compare_ctrl(timer_base, chan);

	alif_utimer_disable_compare_match(timer_base, chan);
	sys_clear_bits(ctrl, COMPARE_CTRL_DRV_COMPARE_TRIG_EN);
	alif_utimer_clear_interrupt(timer_base, chan);
}

#if UTIMER_DMA_USE_HW_CLEAR
/*
 * HRM §13.2.4.3: DMA_CLEAR_SRC_*_1 uses START_1_SRC decode (driver edges).
 */
static void utimer_dma_prog_hw_clear(uintptr_t timer_base, uint8_t chan)
{
	if (chan == 0U) {
		sys_write32(CNTR_SRC1_DRIVER_A_FALLING_B_0,
			    UTIMER_DMA_CLEAR_SRC_A_1(timer_base));
	} else {
		sys_write32(CNTR_SRC1_DRIVER_B_FALLING_A_0,
			    UTIMER_DMA_CLEAR_SRC_B_1(timer_base));
	}
}
#endif

#if UTIMER_DMA_REARM_IDLE_LOW
/* DRIVER_EN=0 + DISABLE_VAL low: software high→low without dummy compare. */
static void utimer_dma_idle_driver_low(uintptr_t timer_base, uint8_t chan)
{
	uintptr_t ctrl = utimer_compare_ctrl(timer_base, chan);

	alif_utimer_disable_compare_match(timer_base, chan);
	sys_clear_bits(ctrl, COMPARE_CTRL_DRV_COMPARE_TRIG_EN);
	alif_utimer_disable_driver(timer_base, chan);
	alif_utimer_set_driver_disable_val_low(timer_base, chan);
	alif_utimer_clear_interrupt(timer_base, chan);
}
#endif

/*
 * Hold UTn_Tx / DMA_REQ low and arm HIGH_AT_COMP_MATCH. Next compare
 * match is the rising edge that triggers DMA. Use when the line is
 * already low.
 */
static void utimer_dma_hold_req_low(uintptr_t timer_base, uint8_t chan)
{
	uintptr_t ctrl = utimer_compare_ctrl(timer_base, chan);

	alif_utimer_disable_compare_match(timer_base, chan);
	alif_utimer_set_driver_disable_val_low(timer_base, chan);
	alif_utimer_config_driver_output(timer_base, chan,
					 COMPARE_CTRL_DRV_HIGH_AT_COMP_MATCH);
	sys_clear_bits(ctrl, COMPARE_CTRL_DRV_START_VAL_HIGH |
			     COMPARE_CTRL_DRV_COMPARE_TRIG_EN);
#if UTIMER_DMA_USE_HW_CLEAR
	sys_set_bits(ctrl, COMPARE_CTRL_DRV_DMA_CLEAR_EN);
#else
	sys_clear_bits(ctrl, COMPARE_CTRL_DRV_DMA_CLEAR_EN);
#endif
	alif_utimer_enable_driver(timer_base, chan);
}

#if UTIMER_DMA_REARM_FORCE_LOW
/*
 * DMA_REQ stays high after compare match; stop/start does not drop it.
 * Force high→low with a dummy LOW_AT_COMP_MATCH, then hold_req_low
 * arms the next rising edge.
 */
static int utimer_dma_force_req_low(const struct device *dev, uint8_t chan)
{
	struct counter_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);
	uintptr_t ctrl = utimer_compare_ctrl(timer_base, chan);
	uint32_t top = alif_utimer_get_counter_reload_value(timer_base);
	uint32_t lead = utimer_dma_lead_ticks(data->frequency);
	uint32_t now;
	uint32_t tgt;
	uint32_t t0;

	if (!data->running) {
		return -EAGAIN;
	}

	alif_utimer_disable_compare_match(timer_base, chan);
	alif_utimer_config_driver_output(timer_base, chan,
					 COMPARE_CTRL_DRV_LOW_AT_COMP_MATCH);
	sys_clear_bits(ctrl, COMPARE_CTRL_DRV_COMPARE_TRIG_EN);
	alif_utimer_clear_interrupt(timer_base, chan);

	{
		k_spinlock_key_t k = k_spin_lock(&data->dma_lock);

		now = alif_utimer_get_counter_value(timer_base);
		tgt = utimer_ticks_ahead(dev, now, lead, top);
		utimer_set_dma_compare(timer_base, chan, tgt);
		sys_set_bits(ctrl, COMPARE_CTRL_DRV_COMPARE_TRIG_EN);
		alif_utimer_enable_compare_match(timer_base, chan);
		t0 = k_cycle_get_32();
		k_spin_unlock(&data->dma_lock, k);
	}

	{
		uint32_t lead_us = (uint32_t)DIV_ROUND_UP((uint64_t)lead * USEC_PER_SEC,
							  data->frequency);
		uint32_t timeout_us = (lead_us * 2U) + UTIMER_DMA_FORCE_TIMEOUT_MARGIN_US;

		while (!(alif_utimer_get_pending_interrupt(timer_base) & BIT(chan))) {
			if (k_cyc_to_us_ceil32(k_cycle_get_32() - t0) > timeout_us) {
				LOG_ERR("ch%u: DMA_REQ force-low timed out", chan);
				utimer_dma_disarm(timer_base, chan);
				return -ETIMEDOUT;
			}
		}
	}

	alif_utimer_disable_compare_match(timer_base, chan);
	sys_clear_bits(ctrl, COMPARE_CTRL_DRV_COMPARE_TRIG_EN);
	alif_utimer_clear_interrupt(timer_base, chan);

	return 0;
}
#endif /* UTIMER_DMA_REARM_FORCE_LOW */

static int utimer_dma_reset_req(const struct device *dev, uint8_t chan)
{
	struct counter_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);
	k_spinlock_key_t key;
	bool was_armed;
	int ret = 0;

	key = k_spin_lock(&data->dma_lock);

	if (data->alarm[chan].dma_rearming) {
		k_spin_unlock(&data->dma_lock, key);
		return -EBUSY;
	}

	was_armed = data->alarm[chan].dma_armed;
	data->alarm[chan].dma_rearming = true;

	if (was_armed) {
#if UTIMER_DMA_REARM_IDLE_LOW
		utimer_dma_disarm(timer_base, chan);
		utimer_dma_idle_driver_low(timer_base, chan);
#elif UTIMER_DMA_REARM_FORCE_LOW
		const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);
		uintptr_t global_base = DEVICE_MMIO_NAMED_GET(dev, global);
		uint32_t oen_bit = BIT((cfg->timer_id * 2U) + chan);
		bool pad_on = !(sys_read32(UTIMER_GLB_DRIVER_OEN(global_base)) & oen_bit);
		k_spinlock_key_t oen_key;

		oen_key = k_spin_lock(&utimer_glb_oen_lock);
		sys_set_bits(UTIMER_GLB_DRIVER_OEN(global_base), oen_bit);
		k_spin_unlock(&utimer_glb_oen_lock, oen_key);

		/* Do not hold dma_lock across HW poll (force-low busy-wait). */
		k_spin_unlock(&data->dma_lock, key);
		ret = utimer_dma_force_req_low(dev, chan);
		key = k_spin_lock(&data->dma_lock);

		oen_key = k_spin_lock(&utimer_glb_oen_lock);
		if (pad_on) {
			sys_clear_bits(UTIMER_GLB_DRIVER_OEN(global_base), oen_bit);
		}
		k_spin_unlock(&utimer_glb_oen_lock, oen_key);

		if (ret != 0) {
			data->alarm[chan].dma_rearming = false;
			k_spin_unlock(&data->dma_lock, key);
			return ret;
		}
#endif
	}

	utimer_dma_hold_req_low(timer_base, chan);

	data->alarm[chan].dma_armed = false;
	data->alarm[chan].dma_rearming = false;
	k_spin_unlock(&data->dma_lock, key);

	return ret;
}

static int utimer_dma_arm_all(const struct device *dev)
{
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);
	int ret;

	for (uint8_t chan = 0; chan < cfg->counter_info.channels; chan++) {
		ret = utimer_dma_reset_req(dev, chan);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

#if UTIMER_DMA_USE_HW_CLEAR
static void utimer_dma_setup_hw_clear_regs(const struct device *dev)
{
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);

	for (uint8_t c = 0; c < cfg->counter_info.channels; c++) {
		utimer_dma_prog_hw_clear(timer_base, c);
	}
}
#endif

static void utimer_dma_log_rearm_mode(void)
{
#if UTIMER_DMA_REARM_FORCE_LOW
	LOG_INF("dma-trig: re-arm=force-low, DMA_CLEAR=%s",
		UTIMER_DMA_USE_HW_CLEAR ? "on" : "off");
#elif UTIMER_DMA_REARM_IDLE_LOW
	LOG_INF("dma-trig: re-arm=idle-low, DMA_CLEAR=%s",
		UTIMER_DMA_USE_HW_CLEAR ? "on" : "off");
#endif
}
#endif

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

	__ASSERT(alif_utimer_check_interrupt_enabled(timer_base, evt_bit) == 0,
				"Expected that CC interrupt is disabled.");

#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
	if (DEV_CFG(dev)->dma_trig) {
		uintptr_t ctrl = utimer_compare_ctrl(timer_base, chan);
		uint32_t lead = utimer_dma_lead_ticks(data->frequency);
		uint32_t diff_abs;
		k_spinlock_key_t key;
		bool dma_armed;
		bool dma_rearming;

		key = k_spin_lock(&data->dma_lock);
		if (data->alarm[chan].dma_armed || data->alarm[chan].dma_rearming) {
			dma_armed = data->alarm[chan].dma_armed;
			dma_rearming = data->alarm[chan].dma_rearming;
			k_spin_unlock(&data->dma_lock, key);
			LOG_ERR("DMA channel %u busy (armed=%d rearming=%d)", chan,
				dma_armed, dma_rearming);
			return -EBUSY;
		}

		now = alif_utimer_get_counter_value(timer_base);
		if (!absolute) {
			uint32_t rel_ticks = val;

			/* Same late window as IRQ path: long relative alarms may wrap. */
			irq_on_late = rel_ticks < (top / 2);
			max_rel_val = irq_on_late ? (top / 2) : top;
			val = utimer_ticks_ahead(dev, now, MAX(val, lead), top);
		} else {
			diff_abs = utimer_ticks_until(dev, val, now, top);
			if (diff_abs < lead || diff_abs > top - data->guard_period) {
				k_spin_unlock(&data->dma_lock, key);
				return -ETIME;
			}
			max_rel_val = top - data->guard_period;
		}

		utimer_set_dma_compare(timer_base, chan, val);
		alif_utimer_clear_interrupt(timer_base, evt_bit);
		sys_set_bits(ctrl, COMPARE_CTRL_DRV_COMPARE_TRIG_EN);
		alif_utimer_enable_compare_match(timer_base, chan);
		data->alarm[chan].dma_armed = true;

		now = alif_utimer_get_counter_value(timer_base);
		diff = utimer_ticks_until(dev, (val == 0U) ? top : val - 1U, now, top);
		if (diff > max_rel_val &&
		    !(alif_utimer_get_pending_interrupt(timer_base) & BIT(chan))) {
			utimer_dma_disarm(timer_base, chan);
			data->alarm[chan].dma_armed = false;
			k_spin_unlock(&data->dma_lock, key);
			return -ETIME;
		}

		k_spin_unlock(&data->dma_lock, key);
		return 0;
	}
#endif

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
	struct counter_alif_utimer_ch_data *chdata;

	if (chan >= cfg->counter_info.channels) {
		LOG_ERR("Invalid counter channel number");
		return -EINVAL;
	}

	chdata = &data->alarm[chan];

	if (alarm_cfg->ticks > counter_alif_utimer_get_top_value(dev)) {
		LOG_ERR("Invalid tick value");
		return -EINVAL;
	}

#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
	if (cfg->dma_trig) {
		/* DMA mode: NULL callback; re-arm via cancel_alarm after each trigger. */
		if (alarm_cfg->callback != NULL) {
			LOG_ERR("dma-trig does not support alarm callbacks");
			return -ENOTSUP;
		}

		return set_compare(dev, chan, alarm_cfg->ticks, alarm_cfg->flags);
	} else if (chdata->alarm_cb) {
		LOG_ERR("Counter is busy");
		return -EBUSY;
	}
#else
	if (chdata->alarm_cb) {
		LOG_ERR("Counter is busy");
		return -EBUSY;
	}
#endif

	chdata->alarm_cb = alarm_cfg->callback;
	chdata->alarm_user_data = alarm_cfg->user_data;

	return set_compare(dev, chan, alarm_cfg->ticks, alarm_cfg->flags);
}

static int counter_alif_utimer_cancel_alarm(const struct device *dev, uint8_t chan)
{
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);
	struct counter_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);
	struct counter_alif_utimer_ch_data *chdata;
	uint8_t evt_bit = chan;

	if (chan >= cfg->counter_info.channels) {
		LOG_ERR("Invalid counter channel number");
		return -EINVAL;
	}

	chdata = &data->alarm[chan];

#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
	if (cfg->dma_trig) {
		int ret = utimer_dma_reset_req(dev, chan);

		if (ret != 0) {
			return ret;
		}
	} else {
		alif_utimer_disable_compare_match(timer_base, chan);
	}
#else
	alif_utimer_disable_compare_match(timer_base, chan);
#endif
	alif_utimer_disable_interrupt(timer_base, evt_bit);
	alif_utimer_clear_interrupt(timer_base, evt_bit);
	atomic_and(&data->cc_int_pending, ~BIT(evt_bit));
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

#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
	if (config->dma_trig && cfg->callback != NULL) {
		return -ENOTSUP;
	}
#endif

	for (int i = 0; i < config->counter_info.channels; i++) {
		/* Overflow can be changed only when all alarms are
		 * disabled.
		 */
#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
		if (data->alarm[i].alarm_cb ||
		    (config->dma_trig &&
		     (data->alarm[i].dma_armed || data->alarm[i].dma_rearming))) {
#else
		if (data->alarm[i].alarm_cb) {
#endif
			LOG_ERR("Counter is busy");
			return -EBUSY;
		}
	}

	alif_utimer_disable_interrupt(timer_base, CHAN_INTERRUPT_OVER_FLOW_BIT);
	alif_utimer_set_counter_reload_value(timer_base, cfg->ticks);
	alif_utimer_clear_interrupt(timer_base, CHAN_INTERRUPT_OVER_FLOW_BIT);

	data->cached_top = cfg->ticks;
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
#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
		if (!config->dma_trig)
#endif
		{
			alif_utimer_enable_interrupt(timer_base, CHAN_INTERRUPT_OVER_FLOW_BIT);
		}
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
	bool sw_irq_pending = (data->cc_int_pending & BIT(evt_bit));

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

#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
	/* dma-trig: NVIC off; DMA uses driver output, not this ISR. */
	if (cfg->dma_trig) {
		return;
	}
#endif

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

	/* check device availability */
	if (!device_is_ready(cfg->clk_dev)) {
		LOG_ERR("clock controller device not ready");
		return -ENODEV;
	}
	/* Enable clock only for lputimer instances from clock manager */
	ret = clock_control_on(cfg->clk_dev, cfg->clkid);
	if (ret != 0) {
		LOG_ERR("Unable to turn on clock: err:%d", ret);
		return ret;
	}
	/* get clock rate from clock manager */
	ret = clock_control_get_rate(cfg->clk_dev,
			cfg->clkid, &data->frequency);
	if (ret != 0) {
		LOG_ERR("Unable to get clock rate: err:%d", ret);
		return ret;
	}

	alif_utimer_enable_timer_clock(global_base, cfg->timer_id);
#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
	if (cfg->dma_trig) {
		if (cfg->counterdirection != ALIF_UTIMER_COUNTER_DIRECTION_UP) {
			LOG_ERR("dma-trig requires up counter direction");
			return -ENOTSUP;
		}
		alif_utimer_disable_timer_output(global_base, cfg->timer_id);
	}
#endif
	ret = utimer_set_direction(timer_base, cfg->counterdirection);
	if (ret != 0) {
		LOG_ERR("Invalid Counter Direction: err:%d", ret);
		return ret;
	}

	alif_utimer_enable_soft_counter_ctrl(timer_base);
	data->cached_top = cfg->counter_info.max_top_value;
	alif_utimer_set_counter_reload_value(timer_base, data->cached_top);
	alif_utimer_enable_counter(timer_base);

	cfg->irq_config(dev);

	data->running = false;

#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
	if (cfg->dma_trig) {
		utimer_dma_log_rearm_mode();
#if UTIMER_DMA_USE_HW_CLEAR
		utimer_dma_setup_hw_clear_regs(dev);
#endif
		ret = utimer_dma_arm_all(dev);
		if (ret != 0) {
			return ret;
		}
	}
#endif

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int counter_alif_utimer_pm_suspend(const struct device *dev)
{
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);
	struct counter_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t global_base = DEVICE_MMIO_NAMED_GET(dev, global);
	int ret;

	for (uint8_t i = 0; i < cfg->counter_info.channels; i++) {
		if (data->alarm[i].alarm_cb) {
			LOG_DBG("PM: alarm active, refusing suspend");
			return -EBUSY;
		}
#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
		if (cfg->dma_trig &&
		    (data->alarm[i].dma_armed || data->alarm[i].dma_rearming)) {
			LOG_DBG("PM: DMA alarm active/rearming, refusing suspend");
			return -EBUSY;
		}
#endif
	}

	if (alif_utimer_any_counter_running(global_base)) {
		LOG_DBG("PM: UTIMER IP in use, refusing suspend");
		return -EBUSY;
	}
	alif_utimer_disable_timer_clock(global_base, cfg->timer_id);

	if (alif_utimer_all_channel_clocks_disabled(global_base)) {
		ret = clock_control_off(cfg->clk_dev, cfg->clkid);
		if (ret != 0 && ret != -EALREADY &&
			ret != -ENOSYS && ret != -ENOTSUP) {
			LOG_WRN("clock off failed: %d", ret);
		}
	}

	return 0;
}

static int counter_alif_utimer_pm_resume(const struct device *dev)
{
	const struct counter_alif_utimer_config *cfg = DEV_CFG(dev);
	struct counter_alif_utimer_data *data = DEV_DATA(dev);
	uintptr_t timer_base = DEVICE_MMIO_NAMED_GET(dev, timer);
	uintptr_t global_base = DEVICE_MMIO_NAMED_GET(dev, global);
	int ret;

	ret = clock_control_on(cfg->clk_dev, cfg->clkid);
	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("Unable to turn on clock: err:%d", ret);
		return ret;
	}

	alif_utimer_enable_timer_clock(global_base, cfg->timer_id);
	ret = utimer_set_direction(timer_base, cfg->counterdirection);
	if (ret != 0) {
		LOG_ERR("Failed to restore counter direction: err:%d", ret);
		return ret;
	}

	alif_utimer_enable_soft_counter_ctrl(timer_base);
	alif_utimer_set_counter_reload_value(timer_base, data->cached_top);
#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
	if (data->top_cb && !cfg->dma_trig) {
#else
	if (data->top_cb) {
#endif
		alif_utimer_clear_interrupt(timer_base, CHAN_INTERRUPT_OVER_FLOW_BIT);
		alif_utimer_enable_interrupt(timer_base, CHAN_INTERRUPT_OVER_FLOW_BIT);
	}

	alif_utimer_enable_counter(timer_base);
	if (data->running) {
		alif_utimer_start_counter(global_base, cfg->timer_id);
	}

#ifdef CONFIG_COUNTER_ALIF_UTIMER_DMA
	if (cfg->dma_trig) {
#if UTIMER_DMA_USE_HW_CLEAR
		utimer_dma_setup_hw_clear_regs(dev);
#endif
		ret = utimer_dma_arm_all(dev);
		if (ret != 0) {
			return ret;
		}
	}
#endif

	return 0;
}

static int counter_alif_utimer_pm_action(const struct device *dev,
					 enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		return counter_alif_utimer_pm_suspend(dev);
	case PM_DEVICE_ACTION_RESUME:
		return counter_alif_utimer_pm_resume(dev);
	case PM_DEVICE_ACTION_TURN_OFF:
	case PM_DEVICE_ACTION_TURN_ON:
		return 0;
	default:
		return -ENOTSUP;
	}
}
#endif

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
		IF_ENABLED(CONFIG_COUNTER_ALIF_UTIMER_DMA, (                                    \
			if (DEV_CFG(dev)->dma_trig) {                                           \
				return;                                                         \
			}                                                                       \
		))                                                                              \
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
		.counter_info = {                                                               \
			.max_top_value = UINT32_MAX,                                            \
			.flags = ((DT_PROP(TIMER(n), counter_direction) ==                      \
					 ALIF_UTIMER_COUNTER_DIRECTION_UP) ?                    \
					 COUNTER_CONFIG_INFO_COUNT_UP : 0),                     \
			.channels = NUM_CHANNELS,                                               \
		},                                                                              \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(global, DT_INST_PARENT(n)),	                \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(timer, DT_INST_PARENT(n)),	            \
		.timer_id = DT_PROP(TIMER(n), timer_id),                                        \
		.counterdirection = DT_PROP(TIMER(n), counter_direction),                       \
		.clk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(TIMER(n))),                             \
		.clkid = (clock_control_subsys_t)DT_CLOCKS_CELL(TIMER(n), clkid),               \
		.irq_config = counter_utimer##n##_irq_config,                                   \
		.set_irq_pending = set_irq_pending_##n,                                         \
		.get_irq_pending = get_irq_pending_##n,                                         \
		IF_ENABLED(CONFIG_COUNTER_ALIF_UTIMER_DMA,                                      \
			(.dma_trig = DT_PROP_OR(TIMER(n), dma_trig, 0),))                       \
	};                                                                                      \
                                                                                                \
	PM_DEVICE_DT_INST_DEFINE(n, counter_alif_utimer_pm_action);                             \
	DEVICE_DT_INST_DEFINE(n,                                                                \
			      &counter_alif_utimer_init,                                        \
			      PM_DEVICE_DT_INST_GET(n),                                         \
			      &counter_alif_utimer_data_##n,                                    \
			      &counter_alif_utimer_cfg_##n,                                     \
			      PRE_KERNEL_1,                                                     \
			      CONFIG_COUNTER_INIT_PRIORITY,                                     \
			      &counter_alif_utimer_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_ALIF_UTIMER);
