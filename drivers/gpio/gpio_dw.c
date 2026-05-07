/*
 * Copyright (c) 2015 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_designware_gpio

#include <errno.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_dw, CONFIG_GPIO_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/snps-designware-gpio.h>
#include "gpio_dw.h"
#include <zephyr/drivers/gpio/gpio_utils.h>

#include <zephyr/pm/device.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>

#ifdef CONFIG_GPIO_DW_MULTICORE
#include <zephyr/drivers/hwsem_ipm.h>
#include <zephyr/drivers/pinctrl/pinctrl_alif.h>

#if defined(CONFIG_RTSS_HE)
#define GPIO_DW_HWSEM_MASTER_ID 1
#elif defined(CONFIG_RTSS_HP)
#define GPIO_DW_HWSEM_MASTER_ID 2
#else
#error "GPIO_DW_MULTICORE requires CONFIG_RTSS_HE or CONFIG_RTSS_HP"
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(hwsem0), okay)
static const struct device *const gpio_dw_hwsem = DEVICE_DT_GET(DT_NODELABEL(hwsem0));
#else
#error "GPIO_DW_MULTICORE requires hwsem0 node to be enabled"
#endif

static inline void gpio_dw_lock(void)
{
	int ret = hwsem_lock(gpio_dw_hwsem, GPIO_DW_HWSEM_MASTER_ID);

	__ASSERT(ret == 0, "hwsem_lock failed: %d", ret);
	ARG_UNUSED(ret);
}

static inline void gpio_dw_unlock(void)
{
	hwsem_unlock(gpio_dw_hwsem, GPIO_DW_HWSEM_MASTER_ID);
}
#endif /* CONFIG_GPIO_DW_MULTICORE */

#ifdef CONFIG_IOAPIC
#include <zephyr/drivers/interrupt_controller/ioapic.h>
#endif

/*
 * ARC architecture configure IP through IO auxiliary registers.
 * Other architectures as ARM and x86 configure IP through MMIO registers
 */
#ifdef GPIO_DW_IO_ACCESS
static inline uint32_t dw_read(uint32_t base_addr, uint32_t offset)
{
	return sys_in32(base_addr + offset);
}

static inline void dw_write(uint32_t base_addr, uint32_t offset,
			    uint32_t val)
{
	sys_out32(val, base_addr + offset);
}

static void dw_set_bit(uint32_t base_addr, uint32_t offset,
		       uint32_t bit, bool value)
{
	if (!value) {
		sys_io_clear_bit(base_addr + offset, bit);
	} else {
		sys_io_set_bit(base_addr + offset, bit);
	}
}
#else
static inline uint32_t dw_read(uint32_t base_addr, uint32_t offset)
{
	return sys_read32(base_addr + offset);
}

static inline void dw_write(uint32_t base_addr, uint32_t offset,
			    uint32_t val)
{
	sys_write32(val, base_addr + offset);
}

static void dw_set_bit(uint32_t base_addr, uint32_t offset,
		       uint32_t bit, bool value)
{
	if (!value) {
		sys_clear_bit(base_addr + offset, bit);
	} else {
		sys_set_bit(base_addr + offset, bit);
	}
}
#endif

static inline int dw_base_to_block_base(uint32_t base_addr)
{
	return (base_addr & 0xFFFFFFC0);
}

static inline int dw_derive_port_from_base(uint32_t base_addr)
{
	uint32_t port = (base_addr & 0x3f) / 12U;
	return port;
}

static inline int dw_interrupt_support(const struct gpio_dw_config *config)
{
	return ((int)(config->irq_num) > 0U);
}

static inline uint32_t dw_get_ext_port(uint32_t base_addr)
{
	uint32_t ext_port;

	/* 4-port GPIO implementation translates from base address to port */
	switch (dw_derive_port_from_base(base_addr)) {
	case 1:
		ext_port = EXT_PORTB;
		break;
	case 2:
		ext_port = EXT_PORTC;
		break;
	case 3:
		ext_port = EXT_PORTD;
		break;
	case 0:
	default:
		ext_port = EXT_PORTA;
		break;
	}

	return ext_port;
}

static inline uint32_t dw_get_data_port(uint32_t base_addr)
{
	uint32_t dr_port;

	/* 4-port GPIO implementation translates from base address to port */
	switch (dw_derive_port_from_base(base_addr)) {
	case 1:
		dr_port = SWPORTB_DR;
		break;
	case 2:
		dr_port = SWPORTC_DR;
		break;
	case 3:
		dr_port = SWPORTD_DR;
		break;
	case 0:
	default:
		dr_port = SWPORTA_DR;
		break;
	}

	return dr_port;
}

static inline uint32_t dw_get_dir_port(uint32_t base_addr)
{
	uint32_t ddr_port;

	/* 4-port GPIO implementation translates from base address to port */
	switch (dw_derive_port_from_base(base_addr)) {
	case 1:
		ddr_port = SWPORTB_DDR;
		break;
	case 2:
		ddr_port = SWPORTC_DDR;
		break;
	case 3:
		ddr_port = SWPORTD_DDR;
		break;
	case 0:
	default:
		ddr_port = SWPORTA_DDR;
		break;
	}

	return ddr_port;
}

static int gpio_dw_pin_interrupt_configure(const struct device *port,
					   gpio_pin_t pin,
					   enum gpio_int_mode mode,
					   enum gpio_int_trig trig)
{
	struct gpio_dw_runtime *context = port->data;
	const struct gpio_dw_config *config = port->config;
	uint32_t base_addr = dw_base_to_block_base(context->base_addr);
	uint32_t port_base_addr = context->base_addr;
	uint32_t dir_port = dw_get_dir_port(port_base_addr);
	uint32_t data_port = dw_get_data_port(port_base_addr);
	uint32_t dir_reg;

	/* Check for invalid pin number */
	if (pin >= config->ngpios) {
		return -EINVAL;
	}

	/* Only PORT-A supports interrupts */
	if (data_port != SWPORTA_DR) {
		return -ENOTSUP;
	}

#ifdef CONFIG_GPIO_DW_MULTICORE
	gpio_dw_lock();
#endif

	if (mode != GPIO_INT_MODE_DISABLED) {
		/* Check if GPIO port supports interrupts */
		if (!dw_interrupt_support(config)) {
#ifdef CONFIG_GPIO_DW_MULTICORE
			gpio_dw_unlock();
#endif
			return -ENOTSUP;
		}

		/* Interrupt to be enabled but pin is not set to input */
		dir_reg = dw_read(base_addr, dir_port) & BIT(pin);
		if (dir_reg != 0U) {
#ifdef CONFIG_GPIO_DW_MULTICORE
			gpio_dw_unlock();
#endif
			return -EINVAL;
		}
	}

	/* Clear interrupt enable */
	dw_set_bit(base_addr, INTEN, pin, false);

	/* Mask and clear interrupt */
	dw_set_bit(base_addr, INTMASK, pin, true);
	dw_write(base_addr, PORTA_EOI, BIT(pin));

	if (mode != GPIO_INT_MODE_DISABLED) {
		if ((mode == GPIO_INT_MODE_EDGE) &&
		    (trig == GPIO_INT_TRIG_BOTH)) {
			/* enable interrupt for both edge. */
			dw_set_bit(base_addr, INT_BOTHEDGE, pin, 1);
		} else {
			/* Clear both-edge mode if previously set */
			dw_set_bit(base_addr, INT_BOTHEDGE, pin, 0);

			/* level (0) or edge (1) */
			dw_set_bit(base_addr, INTTYPE_LEVEL, pin,
				   (mode == GPIO_INT_MODE_EDGE));

			/* Active low/high */
			dw_set_bit(base_addr, INT_POLARITY, pin,
				   (trig == GPIO_INT_TRIG_HIGH));

			if (IS_ENABLED(CONFIG_ENSEMBLE_GEN2)) { /* ENSEMBLE_GEN2 SoC */
				/* default enable debounce when using interrupts. */
				dw_set_bit(base_addr, PORTA_DEBOUNCE, pin, 1);
			}
		}

		/* Finally enabling interrupt */
		dw_set_bit(base_addr, INTEN, pin, true);
		dw_set_bit(base_addr, INTMASK, pin, false);

#ifdef CONFIG_GPIO_DW_MULTICORE
		context->owned_pins |= BIT(pin);
		if (config->shared_irq) {
			irq_enable(config->irq_num);
		} else {
			irq_enable(config->irq_num + pin);
		}
#endif
	} else {
#ifdef CONFIG_GPIO_DW_MULTICORE
		context->owned_pins &= ~BIT(pin);
		if (config->shared_irq) {
			if (context->owned_pins == 0) {
				irq_disable(config->irq_num);
			}
		} else {
			irq_disable(config->irq_num + pin);
		}
#endif
	}

#ifdef CONFIG_GPIO_DW_MULTICORE
	gpio_dw_unlock();
#endif

	return 0;
}

static inline void dw_pin_config(const struct device *port,
				 uint32_t pin, int flags)
{
	struct gpio_dw_runtime *context = port->data;
	const struct gpio_dw_config *config = port->config;
	uint32_t base_addr = dw_base_to_block_base(context->base_addr);
	uint32_t port_base_addr = context->base_addr;
	uint32_t dir_port = dw_get_dir_port(port_base_addr);
	uint32_t data_port = dw_get_data_port(port_base_addr);
	bool pin_is_output, need_debounce;

	/* Set init value then direction */
	pin_is_output = (flags & GPIO_OUTPUT) != 0U;

	dw_set_bit(base_addr, dir_port, pin, pin_is_output);

	if (pin_is_output) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			dw_set_bit(base_addr, data_port, pin, true);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			dw_set_bit(base_addr, data_port, pin, false);
		}
	}

	/* Use built-in debounce.
	 * Note debounce circuit is only available if also supporting
	 * interrupts according to datasheet.
	 */
	if (dw_interrupt_support(config) && (dir_port == SWPORTA_DDR)) {
		need_debounce = (flags & DW_GPIO_DEBOUNCE);
		dw_set_bit(base_addr, PORTA_DEBOUNCE, pin, need_debounce);
	}
}

static inline int gpio_dw_config(const struct device *port,
				 gpio_pin_t pin,
				 gpio_flags_t flags)
{
	const struct gpio_dw_config *config = port->config;
	uint32_t io_flags;

	/* Check for invalid pin number */
	if (pin >= config->ngpios) {
		return -EINVAL;
	}

	/* Does not support disconnected pin, and
	 * not supporting both input/output at same time.
	 */
	io_flags = flags & (GPIO_INPUT | GPIO_OUTPUT);
	if ((io_flags == GPIO_DISCONNECTED)
	    || (io_flags == (GPIO_INPUT | GPIO_OUTPUT))) {
		return -ENOTSUP;
	}

	/* No open-drain support */
	if ((flags & GPIO_SINGLE_ENDED) != 0U) {
		return -ENOTSUP;
	}

	/* Does not support pull-up/pull-down */
	if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0U) {
		return -ENOTSUP;
	}

#if defined(CONFIG_PINCTRL) && defined(CONFIG_GPIO_DW_MULTICORE)
	/*
	 * Apply pinctrl per-pin rather than bank-wide. Search the GPIO's
	 * pinctrl state for the entry matching this pin and apply only that
	 * one, so other pins in the bank (which may be muxed for alt-functions
	 * by another core) are not disturbed.
	 */
	if (config->pcfg != NULL) {
		const struct pinctrl_state *state;

		if (pinctrl_lookup_state(config->pcfg, PINCTRL_STATE_DEFAULT, &state) == 0) {
			/* Find the lowest absolute pin in the state to use as base */
			uint32_t base_pin = UINT32_MAX;

			for (uint8_t i = 0; i < state->pin_cnt; i++) {
				uint32_t pmux_val =
					*(const uint32_t *)&state->pins[i];
				uint32_t p = ALIF_PINMUX_GET_PIN(pmux_val);

				if (p < base_pin) {
					base_pin = p;
				}
			}

			bool found = false;

			for (uint8_t i = 0; i < state->pin_cnt; i++) {
				uint32_t pmux = *(const uint32_t *)&state->pins[i];

				if ((ALIF_PINMUX_GET_PIN(pmux) - base_pin) == pin) {
					pinctrl_configure_pins(&state->pins[i], 1,
							       PINCTRL_REG_NONE);
					found = true;
					break;
				}
			}

			if (!found) {
				LOG_WRN("pin %u not in pinctrl state, mux not applied",
					pin);
			}
		}
	}
#endif

#ifdef CONFIG_GPIO_DW_MULTICORE
	gpio_dw_lock();
#endif

	dw_pin_config(port, pin, flags);

#ifdef CONFIG_GPIO_DW_MULTICORE
	gpio_dw_unlock();
#endif

	return 0;
}

static int gpio_dw_port_get_raw(const struct device *port, uint32_t *value)
{
	struct gpio_dw_runtime *context = port->data;
	uint32_t base_addr = dw_base_to_block_base(context->base_addr);
	uint32_t port_base_addr = context->base_addr;
	uint32_t ext_port = dw_get_ext_port(port_base_addr);

	*value = dw_read(base_addr, ext_port);

	return 0;
}

static int gpio_dw_port_set_masked_raw(const struct device *port,
				       uint32_t mask, uint32_t value)
{
	struct gpio_dw_runtime *context = port->data;
	uint32_t base_addr = dw_base_to_block_base(context->base_addr);
	uint32_t port_base_addr = context->base_addr;
	uint32_t data_port = dw_get_data_port(port_base_addr);
	uint32_t pins;

#ifdef CONFIG_GPIO_DW_MULTICORE
	gpio_dw_lock();
#endif
	pins = dw_read(base_addr, data_port);
	pins = (pins & ~mask) | (mask & value);
	dw_write(base_addr, data_port, pins);
#ifdef CONFIG_GPIO_DW_MULTICORE
	gpio_dw_unlock();
#endif

	return 0;
}

static int gpio_dw_port_set_bits_raw(const struct device *port, uint32_t mask)
{
	struct gpio_dw_runtime *context = port->data;
	uint32_t base_addr = dw_base_to_block_base(context->base_addr);
	uint32_t port_base_addr = context->base_addr;
	uint32_t data_port = dw_get_data_port(port_base_addr);
	uint32_t pins;

#ifdef CONFIG_GPIO_DW_MULTICORE
	gpio_dw_lock();
#endif
	pins = dw_read(base_addr, data_port);
	pins |= mask;
	dw_write(base_addr, data_port, pins);
#ifdef CONFIG_GPIO_DW_MULTICORE
	gpio_dw_unlock();
#endif

	return 0;
}

static int gpio_dw_port_clear_bits_raw(const struct device *port,
				       uint32_t mask)
{
	struct gpio_dw_runtime *context = port->data;
	uint32_t base_addr = dw_base_to_block_base(context->base_addr);
	uint32_t port_base_addr = context->base_addr;
	uint32_t data_port = dw_get_data_port(port_base_addr);
	uint32_t pins;

#ifdef CONFIG_GPIO_DW_MULTICORE
	gpio_dw_lock();
#endif
	pins = dw_read(base_addr, data_port);
	pins &= ~mask;
	dw_write(base_addr, data_port, pins);
#ifdef CONFIG_GPIO_DW_MULTICORE
	gpio_dw_unlock();
#endif

	return 0;
}

static int gpio_dw_port_toggle_bits(const struct device *port, uint32_t mask)
{
	struct gpio_dw_runtime *context = port->data;
	uint32_t base_addr = dw_base_to_block_base(context->base_addr);
	uint32_t port_base_addr = context->base_addr;
	uint32_t data_port = dw_get_data_port(port_base_addr);
	uint32_t pins;

#ifdef CONFIG_GPIO_DW_MULTICORE
	gpio_dw_lock();
#endif
	pins = dw_read(base_addr, data_port);
	pins ^= mask;
	dw_write(base_addr, data_port, pins);
#ifdef CONFIG_GPIO_DW_MULTICORE
	gpio_dw_unlock();
#endif

	return 0;
}

static inline int gpio_dw_manage_callback(const struct device *port,
					  struct gpio_callback *callback,
					  bool set)
{
	struct gpio_dw_runtime *context = port->data;

	return gpio_manage_callback(&context->callbacks, callback, set);
}

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(interrupts)
#ifdef CONFIG_GPIO_DW_MULTICORE

struct gpio_dw_isr_param {
	const struct device *dev;
	uint32_t pin;
};

/* Per-pin ISR: used when each pin has its own NVIC line */
static void gpio_dw_isr_pin(const void *arg)
{
	const struct gpio_dw_isr_param *param = arg;
	const struct device *port = param->dev;
	struct gpio_dw_runtime *context = port->data;
	uint32_t base_addr = dw_base_to_block_base(context->base_addr);
	uint32_t pin_bit = BIT(param->pin);

	if (!(pin_bit & context->owned_pins)) {
		return;
	}

	/* PORTA_EOI is write-1-to-clear; harmless if already cleared */
	dw_write(base_addr, PORTA_EOI, pin_bit);

	gpio_fire_callbacks(&context->callbacks, port, pin_bit);
}

#ifdef CONFIG_ENSEMBLE_GEN2
/* Shared ISR: used when all pins share a single NVIC line (gpio16/17 on E4/E8) */
static void gpio_dw_isr_shared(const struct device *port)
{
	struct gpio_dw_runtime *context = port->data;
	uint32_t base_addr = dw_base_to_block_base(context->base_addr);
	uint32_t int_status;

	int_status = dw_read(base_addr, INTSTATUS);
	int_status &= context->owned_pins;
	if (!int_status) {
		return;
	}

	dw_write(base_addr, PORTA_EOI, int_status);

	gpio_fire_callbacks(&context->callbacks, port, int_status);
}
#endif /* CONFIG_ENSEMBLE_GEN2 */

#else

static void gpio_dw_isr(const struct device *port)
{
	struct gpio_dw_runtime *context = port->data;
	uint32_t base_addr = dw_base_to_block_base(context->base_addr);
	uint32_t int_status;

	int_status = dw_read(base_addr, INTSTATUS);

	dw_write(base_addr, PORTA_EOI, int_status);

	gpio_fire_callbacks(&context->callbacks, port, int_status);
}

#endif
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(interrupts) */

static DEVICE_API(gpio, api_funcs) = {
	.pin_configure = gpio_dw_config,
	.port_get_raw = gpio_dw_port_get_raw,
	.port_set_masked_raw = gpio_dw_port_set_masked_raw,
	.port_set_bits_raw = gpio_dw_port_set_bits_raw,
	.port_clear_bits_raw = gpio_dw_port_clear_bits_raw,
	.port_toggle_bits = gpio_dw_port_toggle_bits,
	.pin_interrupt_configure = gpio_dw_pin_interrupt_configure,
	.manage_callback = gpio_dw_manage_callback,
};

static int gpio_dw_initialize(const struct device *port)
{
	struct gpio_dw_runtime *context = port->data;
	const struct gpio_dw_config *config = port->config;
	uint32_t base_addr;
	int err = 0;

	if (dw_interrupt_support(config)) {

		base_addr = dw_base_to_block_base(context->base_addr);

		/* interrupts in sync with system clock */
		dw_set_bit(base_addr, INT_CLOCK_SYNC, LS_SYNC_POS, 1);

#ifndef CONFIG_GPIO_DW_MULTICORE
		/* mask and disable interrupts */
		dw_write(base_addr, INTMASK, ~(0));
		dw_write(base_addr, INTEN, 0);
		dw_write(base_addr, PORTA_EOI, ~(0));
#endif

		config->config_func(port);
	}

#if defined(CONFIG_PINCTRL)
#ifndef CONFIG_GPIO_DW_MULTICORE
	/*
	 * In multi-core mode, skip bank-wide pinctrl at init.
	 * The GPIO pinctrl group muxes ALL pins in the bank to GPIO function,
	 * which would clobber pins already muxed for alt-functions (Ethernet,
	 * SPI, etc.) by the other core. Each subsystem driver applies its own
	 * per-pin pinctrl, and gpio_pin_configure() only sets direction.
	 */
	if (config->pcfg != NULL) {
		err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	}
#endif
#endif

	return err;
}

/* Bindings to the platform */
#define INST_IRQ_FLAGS(n) \
	COND_CODE_1(DT_INST_IRQ_HAS_CELL(n, flags), (DT_INST_IRQ(n, flags)), (0))

#ifdef CONFIG_GPIO_DW_MULTICORE
/*
 * Multi-core IRQ setup: per-pin ISR params when each pin has its own IRQ,
 * shared ISR when all pins share a single IRQ line.
 */
#define GPIO_CFG_IRQ_PIN(idx, n)                                                                   \
	IRQ_CONNECT(DT_INST_IRQN_BY_IDX(n, idx), DT_INST_IRQ_BY_IDX(n, idx, priority),             \
		    gpio_dw_isr_pin, &gpio_isr_params_##n[idx], INST_IRQ_FLAGS(n));

#ifdef CONFIG_ENSEMBLE_GEN2
#define GPIO_CFG_IRQ_SHARED(idx, n)                                                                \
	IRQ_CONNECT(DT_INST_IRQN_BY_IDX(n, idx), DT_INST_IRQ_BY_IDX(n, idx, priority),             \
		    gpio_dw_isr_shared, DEVICE_DT_INST_GET(n), INST_IRQ_FLAGS(n));
#endif

#define GPIO_ISR_PARAM_ENTRY(idx, n) \
	{ .dev = DEVICE_DT_INST_GET(n), .pin = idx }

/* Select per-pin or shared based on whether ngpios == num_irqs */
#ifdef CONFIG_ENSEMBLE_GEN2
#define GPIO_CFG_IRQ(idx, n)                                              \
	COND_CODE_1(UTIL_BOOL(DT_INST_IRQ_HAS_IDX(n, 1)),            \
		(GPIO_CFG_IRQ_PIN(idx, n)),                            \
		(GPIO_CFG_IRQ_SHARED(idx, n)))
#else
#define GPIO_CFG_IRQ(idx, n)  GPIO_CFG_IRQ_PIN(idx, n)
#endif

#else
#define GPIO_CFG_IRQ(idx, n)									\
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(n, idx),					\
			    DT_INST_IRQ_BY_IDX(n, idx, priority), gpio_dw_isr,			\
			    DEVICE_DT_INST_GET(n), INST_IRQ_FLAGS(n));				\
		irq_enable(DT_INST_IRQN_BY_IDX(n, idx));					\

#endif

#define GPIO_DW_INIT(n)										\
	IF_ENABLED(CONFIG_GPIO_DW_MULTICORE, (						\
		COND_CODE_1(UTIL_BOOL(DT_INST_IRQ_HAS_IDX(n, 1)),			\
			(static const struct gpio_dw_isr_param				\
				gpio_isr_params_##n[] = {				\
				LISTIFY(DT_NUM_IRQS(DT_DRV_INST(n)),			\
					GPIO_ISR_PARAM_ENTRY, (,), n)			\
			};),								\
			(/* single-IRQ: no per-pin params */))				\
	))										\
	static void gpio_config_##n##_irq(const struct device *port)				\
	{											\
		ARG_UNUSED(port);			                                        \
		LISTIFY(DT_NUM_IRQS(DT_DRV_INST(n)), GPIO_CFG_IRQ, (), n)                       \
	}											\
												\
	IF_ENABLED(DT_INST_NODE_HAS_PROP(n, pinctrl_0),						\
			(PINCTRL_DT_INST_DEFINE(n)));						\
												\
	static const struct gpio_dw_config gpio_dw_config_##n = {				\
		.common = {									\
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),			\
		},										\
		.irq_num = COND_CODE_1(DT_INST_IRQ_HAS_IDX(n, 0), (DT_INST_IRQN(n)), (0)),	\
		.ngpios = DT_INST_PROP(n, ngpios),						\
		.config_func = gpio_config_##n##_irq,						\
		IF_ENABLED(DT_INST_NODE_HAS_PROP(n, pinctrl_0),					\
		(.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_DRV_INST(n)),))				\
		IF_ENABLED(CONFIG_GPIO_DW_MULTICORE, (						\
		.shared_irq = !UTIL_BOOL(DT_INST_IRQ_HAS_IDX(n, 1)),))			\
	};											\
												\
	static struct gpio_dw_runtime gpio_##n##_runtime = {					\
		.base_addr = DT_INST_REG_ADDR(n),						\
	};											\
												\
	DEVICE_DT_INST_DEFINE(n, gpio_dw_initialize, NULL, &gpio_##n##_runtime,			\
		      &gpio_dw_config_##n, PRE_KERNEL_1,					\
		      CONFIG_GPIO_INIT_PRIORITY, &api_funcs);					\

DT_INST_FOREACH_STATUS_OKAY(GPIO_DW_INIT)
