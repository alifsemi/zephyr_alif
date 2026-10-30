/*
 * SPDX-FileCopyrightText: Copyright Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT arm_armv7_timer_mem_frame

#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/sys_io.h>
#if defined(CONFIG_GIC)
#include <zephyr/drivers/interrupt_controller/gic.h>
#endif

LOG_MODULE_REGISTER(counter_arm_timer_mem, CONFIG_COUNTER_LOG_LEVEL);

/* CNTControlBase */
#define CNTCR			0x000
#define CNTCR_EN		BIT(0)
#define CNTFID0			0x020

/* CNTCTLBase */
#define CNTACR(n)		(0x040 + (4U * (n)))
#define CNTACR_RPCT		BIT(0)
#define CNTACR_RFRQ		BIT(2)
#define CNTACR_RWPT		BIT(5)
#define CNTACR_PHYS		(CNTACR_RPCT | CNTACR_RFRQ | CNTACR_RWPT)

/* CNTBaseN physical timer */
#define CNTPCT_LO		0x000
#define CNTPCT_HI		0x004
#define CNTP_CVAL_LO		0x020
#define CNTP_CVAL_HI		0x024
#define CNTP_CTL		0x02C
#define CNTP_CTL_ENABLE		BIT(0)
#define CNTP_CTL_IMASK		BIT(1)
#define CNTP_CTL_ISTATUS	BIT(2)

#define TIMER_MAX_VALUE		UINT32_MAX

#define PARENT(n)		DT_INST_PARENT(n)

#define REG_READ(base, off)	sys_read32((mem_addr_t)((base) + (off)))
#define REG_WRITE(base, off, v)	sys_write32((v), (mem_addr_t)((base) + (off)))

struct arm_timer_mem_data {
	counter_alarm_callback_t callback;
	void *user_data;
	uint32_t guard_period;
	bool alarm_active;
};

struct arm_timer_mem_config {
	struct counter_config_info info;
	mem_addr_t base;
	mem_addr_t control;
	mem_addr_t ctl;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	unsigned int irqn;
	uint8_t frame;
	void (*irq_config)(const struct device *dev);
};

static uint64_t read_cntpct(mem_addr_t base)
{
	uint32_t hi;
	uint32_t lo;

	do {
		hi = REG_READ(base, CNTPCT_HI);
		lo = REG_READ(base, CNTPCT_LO);
	} while (hi != REG_READ(base, CNTPCT_HI));

	return ((uint64_t)hi << 32) | lo;
}

static void write_cval(mem_addr_t base, uint64_t cval)
{
	REG_WRITE(base, CNTP_CVAL_LO, (uint32_t)cval);
	REG_WRITE(base, CNTP_CVAL_HI, (uint32_t)(cval >> 32));
}

static void timer_disarm(mem_addr_t base)
{
	REG_WRITE(base, CNTP_CTL, CNTP_CTL_IMASK);
}

static int request_phys_access(mem_addr_t ctl, uint8_t frame)
{
	uint32_t cntacr;

	REG_WRITE(ctl, CNTACR(frame), CNTACR_PHYS);
	cntacr = REG_READ(ctl, CNTACR(frame));
	if ((cntacr & (CNTACR_RPCT | CNTACR_RWPT)) != (CNTACR_RPCT | CNTACR_RWPT)) {
		LOG_ERR("CNTACR%u physical access not granted (0x%x)", frame, cntacr);
		return -EACCES;
	}

	return 0;
}

static void irq_set_pending_line(unsigned int irq)
{
#if defined(CONFIG_GIC)
	arm_gic_irq_set_pending(irq);
#else
	NVIC_SetPendingIRQ(irq);
#endif
}

static int arm_timer_mem_start(const struct device *dev)
{
	const struct arm_timer_mem_config *cfg = dev->config;
	uint32_t cntcr;

	cntcr = REG_READ(cfg->control, CNTCR);
	REG_WRITE(cfg->control, CNTCR, cntcr | CNTCR_EN);

	return 0;
}

static int arm_timer_mem_stop(const struct device *dev)
{
	const struct arm_timer_mem_config *cfg = dev->config;
	struct arm_timer_mem_data *data = dev->data;
	uint32_t cntcr;

	timer_disarm(cfg->base);
	data->callback = NULL;
	data->user_data = NULL;
	data->alarm_active = false;

	cntcr = REG_READ(cfg->control, CNTCR);
	REG_WRITE(cfg->control, CNTCR, cntcr & ~CNTCR_EN);

	return 0;
}

static int arm_timer_mem_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct arm_timer_mem_config *cfg = dev->config;

	*ticks = (uint32_t)read_cntpct(cfg->base);

	return 0;
}

static int arm_timer_mem_get_value_64(const struct device *dev, uint64_t *ticks)
{
	const struct arm_timer_mem_config *cfg = dev->config;

	*ticks = read_cntpct(cfg->base);

	return 0;
}

static int arm_timer_mem_set_alarm(const struct device *dev, uint8_t chan_id,
				   const struct counter_alarm_cfg *alarm_cfg)
{
	const struct arm_timer_mem_config *cfg = dev->config;
	struct arm_timer_mem_data *data = dev->data;
	uint64_t now64;
	uint64_t target64;
	uint32_t ticks = alarm_cfg->ticks;
	uint32_t flags = alarm_cfg->flags;
	uint32_t max_rel_val = 0;
	bool irq_on_late;
	bool late;
	int err = 0;

	ARG_UNUSED(chan_id);

	if (data->alarm_active) {
		return -EBUSY;
	}

	now64 = read_cntpct(cfg->base);

	if (flags & COUNTER_ALARM_CFG_ABSOLUTE) {
		__ASSERT_NO_MSG(data->guard_period < TIMER_MAX_VALUE);
		max_rel_val = TIMER_MAX_VALUE - data->guard_period;
		irq_on_late = !!(flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE);
		target64 = (now64 & ~((uint64_t)TIMER_MAX_VALUE)) | ticks;
		if (target64 < now64) {
			target64 += ((uint64_t)TIMER_MAX_VALUE) + 1U;
		}
	} else {
		irq_on_late = true;
		target64 = now64 + ticks;
	}

	data->callback = alarm_cfg->callback;
	data->user_data = alarm_cfg->user_data;
	data->alarm_active = true;

	timer_disarm(cfg->base);
	write_cval(cfg->base, target64);
	REG_WRITE(cfg->base, CNTP_CTL, CNTP_CTL_ENABLE);

	now64 = read_cntpct(cfg->base);
	if (flags & COUNTER_ALARM_CFG_ABSOLUTE) {
		late = ((uint32_t)(target64 - 1U) - (uint32_t)now64) > max_rel_val;
	} else {
		late = now64 >= target64;
	}

	if (late) {
		if (flags & COUNTER_ALARM_CFG_ABSOLUTE) {
			err = -ETIME;
		}

		if (irq_on_late) {
			irq_set_pending_line(cfg->irqn);
		} else {
			timer_disarm(cfg->base);
			data->callback = NULL;
			data->user_data = NULL;
			data->alarm_active = false;
		}
	}

	return err;
}

static int arm_timer_mem_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	const struct arm_timer_mem_config *cfg = dev->config;
	struct arm_timer_mem_data *data = dev->data;

	ARG_UNUSED(chan_id);

	timer_disarm(cfg->base);
	data->callback = NULL;
	data->user_data = NULL;
	data->alarm_active = false;

	return 0;
}

static int arm_timer_mem_set_top_value(const struct device *dev,
				       const struct counter_top_cfg *cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cfg);

	return -ENOTSUP;
}

static uint32_t arm_timer_mem_get_pending_int(const struct device *dev)
{
	const struct arm_timer_mem_config *cfg = dev->config;

	return (REG_READ(cfg->base, CNTP_CTL) & CNTP_CTL_ISTATUS) ? 1U : 0U;
}

static uint32_t arm_timer_mem_get_top_value(const struct device *dev)
{
	const struct arm_timer_mem_config *cfg = dev->config;

	return cfg->info.max_top_value;
}

static int arm_timer_mem_set_guard_period(const struct device *dev, uint32_t ticks,
					  uint32_t flags)
{
	struct arm_timer_mem_data *data = dev->data;

	if (flags & ~COUNTER_GUARD_PERIOD_LATE_TO_SET) {
		return -ENOTSUP;
	}

	if (ticks >= TIMER_MAX_VALUE) {
		return -EINVAL;
	}

	data->guard_period = ticks;

	return 0;
}

static uint32_t arm_timer_mem_get_guard_period(const struct device *dev, uint32_t flags)
{
	struct arm_timer_mem_data *data = dev->data;

	ARG_UNUSED(flags);

	return data->guard_period;
}

static uint32_t arm_timer_mem_get_freq(const struct device *dev)
{
	const struct arm_timer_mem_config *cfg = dev->config;

	return REG_READ(cfg->control, CNTFID0);
}

static void arm_timer_mem_isr(const struct device *dev)
{
	const struct arm_timer_mem_config *cfg = dev->config;
	struct arm_timer_mem_data *data = dev->data;
	counter_alarm_callback_t cb;
	void *user_data;

	timer_disarm(cfg->base);

	cb = data->callback;
	user_data = data->user_data;
	data->callback = NULL;
	data->user_data = NULL;
	data->alarm_active = false;

	if (cb) {
		cb(dev, 0, (uint32_t)read_cntpct(cfg->base), user_data);
	}
}

static int arm_timer_mem_init(const struct device *dev)
{
	const struct arm_timer_mem_config *cfg = dev->config;
	int err;

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	err = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (err) {
		LOG_ERR("Failed to enable clock");
		return err;
	}

	if (REG_READ(cfg->control, CNTFID0) == 0U) {
		uint32_t rate;

		err = clock_control_get_rate(cfg->clock_dev, cfg->clock_subsys, &rate);
		if (err) {
			LOG_ERR("Failed to get clock frequency");
			return err;
		}
		if (rate != 0U) {
			REG_WRITE(cfg->control, CNTFID0, rate);
		}
	}

	err = request_phys_access(cfg->ctl, cfg->frame);
	if (err) {
		return err;
	}

	timer_disarm(cfg->base);
	cfg->irq_config(dev);

	return 0;
}

static DEVICE_API(counter, arm_timer_mem_api) = {
	.start = arm_timer_mem_start,
	.stop = arm_timer_mem_stop,
	.get_value = arm_timer_mem_get_value,
	.get_value_64 = arm_timer_mem_get_value_64,
	.set_alarm = arm_timer_mem_set_alarm,
	.cancel_alarm = arm_timer_mem_cancel_alarm,
	.set_top_value = arm_timer_mem_set_top_value,
	.get_pending_int = arm_timer_mem_get_pending_int,
	.get_top_value = arm_timer_mem_get_top_value,
	.get_guard_period = arm_timer_mem_get_guard_period,
	.set_guard_period = arm_timer_mem_set_guard_period,
	.get_freq = arm_timer_mem_get_freq,
};

#define ARM_TIMER_MEM_INIT(n)							\
	static void arm_timer_mem_irq_config_##n(const struct device *dev)	\
	{									\
		ARG_UNUSED(dev);						\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
			    arm_timer_mem_isr, DEVICE_DT_INST_GET(n),		\
			    COND_CODE_1(DT_INST_IRQ_HAS_CELL(n, flags),		\
					(DT_INST_IRQ(n, flags)), (0)));		\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
										\
	static struct arm_timer_mem_data arm_timer_mem_data_##n;		\
										\
	static const struct arm_timer_mem_config arm_timer_mem_config_##n = {	\
		.info = {							\
			.max_top_value = TIMER_MAX_VALUE,			\
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,			\
			.channels = 1,						\
		},								\
		.base = DT_INST_REG_ADDR(n),					\
		.control = DT_REG_ADDR_BY_NAME(PARENT(n), control),		\
		.ctl = DT_REG_ADDR_BY_NAME(PARENT(n), ctl),			\
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(PARENT(n))),		\
		.clock_subsys = (clock_control_subsys_t)			\
			DT_CLOCKS_CELL(PARENT(n), clkid),			\
		.irqn = DT_INST_IRQN(n),					\
		.frame = DT_INST_PROP(n, frame_number),				\
		.irq_config = arm_timer_mem_irq_config_##n,			\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n,						\
			      arm_timer_mem_init,				\
			      NULL,						\
			      &arm_timer_mem_data_##n,				\
			      &arm_timer_mem_config_##n,			\
			      PRE_KERNEL_1,					\
			      CONFIG_COUNTER_INIT_PRIORITY,			\
			      &arm_timer_mem_api);

DT_INST_FOREACH_STATUS_OKAY(ARM_TIMER_MEM_INIT)
