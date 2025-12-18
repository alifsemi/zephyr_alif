/*
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/comparator.h>
#include <zephyr/drivers/clock_control.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc_common.h>
#include "analog_ctrl.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(CMP);

#define DT_DRV_COMPAT alif_cmp

struct cmp_config {
	DEVICE_MMIO_NAMED_ROM(cmp_reg);
	DEVICE_MMIO_NAMED_ROM(adc_vref);
	DEVICE_MMIO_NAMED_ROM(dac6_reg);
	DEVICE_MMIO_NAMED_ROM(config_reg);
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
	const struct gpio_dt_spec cmp_gpio;
	const struct device *clk_dev;
	clock_control_subsys_t clkid;
	uint32_t drv_inst;
	uint8_t polarity_en;
	uint32_t filter_taps;
	uint32_t prescaler;
	uint8_t positive_inp;
	uint8_t negative_inp;
	uint8_t hysteresis_level;
};

struct cmp_data {
	DEVICE_MMIO_NAMED_RAM(cmp_reg);
	DEVICE_MMIO_NAMED_RAM(adc_vref);
	DEVICE_MMIO_NAMED_RAM(dac6_reg);
	DEVICE_MMIO_NAMED_RAM(config_reg);
	uint8_t polarity;
	uint8_t interrupt_mask;
	comparator_callback_t callback;
	void *user_data;
};

enum CMP_INSTANCE {
	CMP_INSTANCE_LP,
	CMP_INSTANCE_0,
	CMP_INSTANCE_1,
	CMP_INSTANCE_2,
	CMP_INSTANCE_3
};

#define DEV_DATA(dev) ((struct cmp_data *)((dev)->data))
#define DEV_CFG(dev)  ((struct cmp_config *)((dev)->config))

/* comparator register */
#define CMP_COMP_REG1        (0x00)
#define CMP_COMP_REG2        (0x04)
#define CMP_POLARITY_CTRL    (0x08)
#define CMP_WINDOW_CTRL      (0x0C)
#define CMP_FILTER_CTRL      (0x10)
#define CMP_PRESCALER_CTRL   (0x14)
#define CMP_STATUS           (0x18)
#define CMP_INTERRUPT_STATUS (0x20)
#define CMP_INTERRUPT_MASK   (0x24)

#if defined(CONFIG_ANALOG_ALIASING)
/* Each instances are enabled
 * from the own register.
 */
#define CMP0_ENABLE  (1U << 28)
#define CMP1_ENABLE  (1U << 28)
#define CMP2_ENABLE  (1U << 28)
#define CMP3_ENABLE  (1U << 28)
#define LPCMP_ENABLE (1U << 24)
#else
#define CMP0_ENABLE  (1U << 28)
#define CMP1_ENABLE  (1U << 29)
#define CMP2_ENABLE  (1U << 30)
#define CMP3_ENABLE  (1U << 31)
#define LPCMP_ENABLE (1U << 24)
#endif

#define CMP_FILTER_CONTROL_ENABLE (1U << 0)
#define CMP_PRESCALER_MAX_VALUE   (0x3FU)
#define CMP_POLARITY_MAX_VALUE    (0x2U)
#define CMP_WINDOW_MAX_VALUE      (0x3U)
#define CMP_FILTER_MIN_VALUE      (0x2U)
#define CMP_FILTER_MAX_VALUE      (0x8U)

#define CMP_INT_STATUS_MASK  CONFIG_CMP_INT_STATUS_MASK

#define CMP_INT_MASK        (0x01UL)
#define CMP_INTERRUPT_CLEAR (0x01UL)

/* CMP interrupts*/
#define CMP_FILTER_EVENT0   (1U << 0)
#define CMP_FILTER_EVENT1   (1U << 1)

#define CMP_FILTER_EVENT0_CLEAR (1U << 0)
#define CMP_FILTER_EVENT1_CLEAR (1U << 1)
#define CMP_FILTER_EVENTS_CLEAR_ALL (0x3U)

/* Comparator reg1 macro */
#if defined(CONFIG_ANALOG_ALIASING)
#define CMP0_IN_POS_SEL_POS (0)
#define CMP0_IN_NEG_SEL_POS (2)
#define CMP0_HYST_SEL_POS   (4)

#define CMP1_IN_POS_SEL_POS (0)
#define CMP1_IN_NEG_SEL_POS (2)
#define CMP1_HYST_SEL_POS   (4)

#define CMP2_IN_POS_SEL_POS (0)
#define CMP2_IN_NEG_SEL_POS (2)
#define CMP2_HYST_SEL_POS   (4)

#define CMP3_IN_POS_SEL_POS (0)
#define CMP3_IN_NEG_SEL_POS (2)
#define CMP3_HYST_SEL_POS   (4)
#else
#define CMP0_IN_POS_SEL_POS (0)
#define CMP0_IN_NEG_SEL_POS (2)
#define CMP0_HYST_SEL_POS   (4)

#define CMP1_IN_POS_SEL_POS (7)
#define CMP1_IN_NEG_SEL_POS (9)
#define CMP1_HYST_SEL_POS   (11)

#define CMP2_IN_POS_SEL_POS (14)
#define CMP2_IN_NEG_SEL_POS (16)
#define CMP2_HYST_SEL_POS   (18)

#define CMP3_IN_POS_SEL_POS (21)
#define CMP3_IN_NEG_SEL_POS (23)
#define CMP3_HYST_SEL_POS   (25)
#endif

/* CLKCTL_PER_SLV CMP field definitions */
#if defined(CONFIG_ANALOG_ALIASING)
#define CMP_CTRL_CMP0_CLKEN (1U << 0U)
#define CMP_CTRL_CMP1_CLKEN (1U << 4U)
#define CMP_CTRL_CMP2_CLKEN (1U << 8U)
#define CMP_CTRL_CMP3_CLKEN (1U << 12U)
#define LPCMP_CTRL_CLKEN    (1U << 0U)
#else
#define CMP_CTRL_CMP0_CLKEN (1U << 0U)
#define CMP_CTRL_CMP1_CLKEN ((1U << 4U) | CMP_CTRL_CMP0_CLKEN)
#define CMP_CTRL_CMP2_CLKEN ((1U << 8U) | CMP_CTRL_CMP0_CLKEN)
#define CMP_CTRL_CMP3_CLKEN ((1U << 12U) | CMP_CTRL_CMP0_CLKEN)
#define LPCMP_CTRL_CLKEN    (1 << 14)
#endif


/* LP comparator macro */
#define LPCOMP_CLK_SEL	         (1)
#define LPCOMP_CLK32K_EN        (14)
#define COMP_LP0_EN             (24)
#define COMP_LP0_IN_POS_SEL_POS (25)
#define COMP_LP0_IN_NEG_SEL_POS (27)
#define COMP_LP0_HYST_POS       (29)

/* Comparator reg2 marcos */
#define DAC6_VREF_SCALE      (0x1U << 27)
#define DAC6_CONT            (0x20U << 21)
#define DAC6_EN              (0x1U << 20)
#define DAC12_VREF_CONT      (0x4U << 17)
#define ADC_VREF_BUF_RDIV_EN (0x0U << 16)
#define ADC_VREF_BUF_EN      (0x1U << 15)
#define ADC_VREF_CONT        (0x10U << 10)
#define ANA_PERIPH_LDO_CONT  (0xAU << 6)
#define ANA_PERIPH_BG_CONT   (0xAU << 1)

static void cmp_analog_config(const struct device *dev)
{
	const struct cmp_config *config = dev->config;
#if defined(CONFIG_ANALOG_ALIASING)
	uintptr_t dac6_base = DEVICE_MMIO_NAMED_GET(dev, dac6_reg);
	uintptr_t adc_vref_base = DEVICE_MMIO_NAMED_GET(dev, adc_vref);

	if (config->drv_inst != CMP_INSTANCE_LP) {
		enable_dac6_ref_voltage_alias_mode(adc_vref_base, dac6_base);
	}
#else
	uint32_t regs;
	uint32_t cmp_reg2_base;

	regs = DEVICE_MMIO_NAMED_GET(dev, config_reg);

	cmp_reg2_base = (regs + CMP_COMP_REG2);

	if (config->drv_inst != CMP_INSTANCE_LP) {
		enable_dac6_ref_voltage(cmp_reg2_base);
	}
#endif
	enable_analog_peripherals(ANA_VBAT_REG2);
}

static inline void cmp_enable_interrupt(uintptr_t cmp, uint8_t interrupts)
{
	sys_write32((~interrupts & 0x3), cmp + CMP_INTERRUPT_MASK);
}

static inline void cmp_set_polarity_ctrl(uintptr_t cmp, uint8_t polarity)
{
	sys_write32(polarity, cmp + CMP_POLARITY_CTRL);
}

static inline void cmp_prescaler_ctrl(uintptr_t cmp, uint8_t prescaler)
{
	sys_write32(prescaler, cmp + CMP_PRESCALER_CTRL);
}

static inline void cmp_set_filter_ctrl(uintptr_t cmp, uint32_t filter)
{
	uint32_t data;

	data = sys_read32(cmp + CMP_FILTER_CTRL);
	data |= (CMP_FILTER_CONTROL_ENABLE | filter << 8);
	sys_write32(data, cmp + CMP_FILTER_CTRL);
}

static void lpcmp_set_config(const struct device *dev)
{
	uint32_t data = 0;
	uintptr_t regs = 0;

	regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);
	const struct cmp_config *config = dev->config;

	data |= config->positive_inp << COMP_LP0_IN_POS_SEL_POS |
		config->negative_inp << COMP_LP0_IN_NEG_SEL_POS |
		config->hysteresis_level << COMP_LP0_HYST_POS;

#if defined(CONFIG_ANALOG_ALIASING)
	sys_write32(data, regs);
#else
	uint32_t value = 0;

	value = sys_read32(regs);
	value |= data;
	sys_write32(value, regs);
#endif

}

static void cmp_set_config(const struct device *dev)
{
	uintptr_t regs = 0;
	uint32_t data = 0;

	const struct cmp_config *config = dev->config;

#if defined(CONFIG_ANALOG_ALIASING)
	regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);
#else
	regs = DEVICE_MMIO_NAMED_GET(dev, config_reg);
#endif
	switch (config->drv_inst) {
	case CMP_INSTANCE_0:
		data |= config->positive_inp << CMP0_IN_POS_SEL_POS |
			config->negative_inp << CMP0_IN_NEG_SEL_POS |
			config->hysteresis_level << CMP0_HYST_SEL_POS;
		break;

	case CMP_INSTANCE_1:
		data |= config->positive_inp << CMP1_IN_POS_SEL_POS |
			config->negative_inp << CMP1_IN_NEG_SEL_POS |
			config->hysteresis_level << CMP1_HYST_SEL_POS;
		break;

	case CMP_INSTANCE_2:
		data |= config->positive_inp << CMP2_IN_POS_SEL_POS |
			config->negative_inp << CMP2_IN_NEG_SEL_POS |
			config->hysteresis_level << CMP2_HYST_SEL_POS;
		break;

	case CMP_INSTANCE_3:
		data |= config->positive_inp << CMP3_IN_POS_SEL_POS |
			config->negative_inp << CMP3_IN_NEG_SEL_POS |
			config->hysteresis_level << CMP3_HYST_SEL_POS;
		break;
	}

	sys_write32(data, (regs + CMP_COMP_REG1));
}

static inline void enable_cmp(const struct device *dev)
{
	uintptr_t regs = 0;
	uint32_t data;
	const struct cmp_config *config = dev->config;

#if defined(CONFIG_ANALOG_ALIASING)
	regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);
#else
	regs = DEVICE_MMIO_NAMED_GET(dev, config_reg);
#endif

	data = sys_read32(regs);

	switch (config->drv_inst) {
	case CMP_INSTANCE_0:
		/* Enable the CMP0 module */
		data |= CMP0_ENABLE;
		break;

	case CMP_INSTANCE_1:
		/* Enable the CMP1 module */
		data |= CMP1_ENABLE;
		break;

	case CMP_INSTANCE_2:
		/* Enable the CMP2 module */
		data |= CMP2_ENABLE;
		break;

	case CMP_INSTANCE_3:
		/* Enable the CMP3 module */
		data |= CMP3_ENABLE;
		break;
	case CMP_INSTANCE_LP:
		/* Enable the LPCMP module */
		data |= LPCMP_ENABLE;
		break;
	}

	sys_write32(data, regs);
}

static void cmp_irq_handler(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);
	const struct cmp_config *config = dev->config;
	struct cmp_data *data = dev->data;

	if (config->drv_inst != CMP_INSTANCE_LP) {
		uint8_t int_status =
			sys_read32(regs + CMP_INTERRUPT_STATUS) & CMP_INT_STATUS_MASK;

		/* clear the interrupt before re-starting */
		if (int_status) {
			sys_write32(int_status, regs + CMP_INTERRUPT_STATUS);
		}
	}

	if (data->callback) {
		*(uint8_t *)data->user_data =  gpio_pin_get_dt(&config->cmp_gpio);
		data->callback(dev, data->user_data);
	}
}

static void cmp_setup(const struct device *dev)
{
	const struct cmp_config *config = dev->config;
	uintptr_t regs;

	regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);

	/* polarity setup */
	cmp_set_polarity_ctrl(regs, config->polarity_en);

	/* filter tap setup */
	cmp_set_filter_ctrl(regs, config->filter_taps);

	/* prescaler setup */
	cmp_prescaler_ctrl(regs, config->prescaler);
}

static int alif_comp_set_trigger(const struct device *dev,
				  enum comparator_trigger trigger)
{
	uintptr_t regs;
	struct cmp_data *data = dev->data;

	regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);

	switch (trigger) {
	case COMPARATOR_TRIGGER_NONE:
		data->interrupt_mask = 0;
		break;

	case COMPARATOR_TRIGGER_RISING_EDGE:
		data->interrupt_mask = CMP_FILTER_EVENT0;
		break;

	case COMPARATOR_TRIGGER_FALLING_EDGE:
		data->interrupt_mask = CMP_FILTER_EVENT1;
		break;

	case COMPARATOR_TRIGGER_BOTH_EDGES:
		data->interrupt_mask = CMP_FILTER_EVENT0 |
				       CMP_FILTER_EVENT1;
		break;
	}

	if (data->callback != NULL) {
		cmp_enable_interrupt(regs, data->interrupt_mask);
	}

	enable_cmp(dev);

	return 0;

}

static int alif_comp_set_trigger_callback(const struct device *dev,
					   comparator_callback_t callback,
					   void *user_data)
{
	uintptr_t regs;
	struct cmp_data *data = dev->data;

	regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);

	data->callback = callback;
	data->user_data = user_data;

	return 0;
}

static int alif_comp_get_output(const struct device *dev)
{
	/* read pin status */
#if defined(CONFIG_ANALOG_ALIASING)
	uintptr_t regs;

	regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);

	return sys_read32(regs + CMP_STATUS);
#else
	const struct cmp_config *config = dev->config;

	return gpio_pin_get_dt(&config->cmp_gpio);
#endif

}

static int alif_cmp_trigger_is_pending(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);

	uint8_t int_status =
			sys_read32(regs + CMP_INTERRUPT_STATUS) & CMP_INT_STATUS_MASK;

	/* Check for the pending interrupts */
	if (int_status)	{
		sys_write32(int_status, regs + CMP_INTERRUPT_STATUS);
		return 1;
	}

	return 0;
}

static int cmp_init(const struct device *dev)
{
	int ret;
	uintptr_t regs;
	const struct cmp_config *config = dev->config;

	DEVICE_MMIO_NAMED_MAP(dev, cmp_reg, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, config_reg, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, dac6_reg, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, adc_vref, K_MEM_CACHE_NONE);

	regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		return ret;
	}

	/* comparator set configuration */
	cmp_analog_config(dev);

	if (config->drv_inst == CMP_INSTANCE_LP) {

		/*Enable LPCMP clock*/
		sys_write32(LPCMP_CTRL_CLKEN, regs);

		/* LPCMP configuration value to the Vbat reg2 */
		lpcmp_set_config(dev);
	} else {
		/* check device availability */
		if (!device_is_ready(config->clk_dev)) {
			LOG_ERR("clock controller device not ready");
			return -ENODEV;
		}

		/* Configure CMP clock sources */
		ret = clock_control_configure(config->clk_dev,
						config->clkid, NULL);
		if (ret != 0) {
			LOG_ERR("Unable to configure clock: err:%d", ret);
			return ret;
		}

		/* Enable CMP clock from clock manager */
		ret = clock_control_on(config->clk_dev, config->clkid);
		if (ret != 0) {
			LOG_ERR("Unable to turn on clock: err:%d", ret);
			return ret;
		}

		/*Configure Reg1 register*/
		cmp_set_config(dev);

		/* Setup comparator registers */
		cmp_setup(dev);

		/* configuring GPIO pins */
		if (config->cmp_gpio.port != NULL) {
			ret = gpio_pin_configure_dt(&config->cmp_gpio, GPIO_INPUT);
			if (ret < 0) {
				LOG_ERR("Could not configure reset GPIO (%d)", ret);
				return ret;
			}
		}
	}

	config->irq_config_func(dev);

	return 0;
}

static DEVICE_API(comparator, alif_comp_api) = {
	.get_output = alif_comp_get_output,
	.set_trigger = alif_comp_set_trigger,
	.set_trigger_callback = alif_comp_set_trigger_callback,
	.trigger_is_pending = alif_cmp_trigger_is_pending,
};

#define CMP_ALIF_INIT(inst)                                                                        \
                                                                                                   \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, pinctrl_0), (PINCTRL_DT_INST_DEFINE(inst)));        \
                                                                                                   \
	static void cmp_config_func_##inst(const struct device *dev);                              \
	const struct cmp_config config_##inst = {                                                  \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(cmp_reg, DT_DRV_INST(inst)),                    \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(config_reg, DT_DRV_INST(inst)),                 \
		IF_ENABLED(DT_INST_REG_HAS_NAME(inst, dac6_reg),                                   \
		(DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(dac6_reg, DT_DRV_INST(inst)),))                \
		IF_ENABLED(DT_INST_REG_HAS_NAME(inst, adc_vref),                                   \
		(DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(adc_vref, DT_DRV_INST(inst)),))                \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, clocks), (                                  \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                               \
		.clkid = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, clkid),))               \
		.irq_config_func = cmp_config_func_##inst,                                         \
		.cmp_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, cmp_gpios, {0}),                        \
		.drv_inst = DT_INST_ENUM_IDX(inst, driver_instance),                               \
		.polarity_en = DT_INST_PROP_OR(inst, polarity_en, 0),                              \
		.prescaler = DT_INST_PROP_OR(inst, prescaler, 0),                                  \
		.filter_taps = DT_INST_PROP_OR(inst, filter_taps, 0),                              \
		.positive_inp = DT_INST_ENUM_IDX(inst, positive_input),                            \
		.negative_inp = DT_INST_ENUM_IDX(inst, negative_input),                            \
		.hysteresis_level = DT_INST_ENUM_IDX(inst, hysteresis_level),                      \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, pinctrl_0),                                 \
			   (.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_DRV_INST(inst)),))};              \
                                                                                                   \
	struct cmp_data data_##inst;                                                               \
	DEVICE_DT_INST_DEFINE(inst, cmp_init, NULL, &data_##inst, &config_##inst, POST_KERNEL,     \
			      CONFIG_COMPARATOR_INIT_PRIORITY, &alif_comp_api);                    \
                                                                                                   \
	static void cmp_config_func_##inst(const struct device *dev)                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), cmp_irq_handler,      \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
                                                                                                   \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}

DT_INST_FOREACH_STATUS_OKAY(CMP_ALIF_INIT)
