/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/comparator/alif_cmp.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(CMP);

#define DT_DRV_COMPAT alif_cmp

struct cmp_config {
	DEVICE_MMIO_NAMED_ROM(cmp_reg);
	DEVICE_MMIO_NAMED_ROM(config_reg);
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
	const struct gpio_dt_spec cmp_gpio;
	uint32_t drv_inst;
	uint8_t positive_inp;
	uint8_t negative_inp;
	uint8_t hysteresis_level;
};

struct cmp_data {
	DEVICE_MMIO_NAMED_RAM(cmp_reg);
	DEVICE_MMIO_NAMED_RAM(config_reg);
	uint8_t polarity;
	void (*callback)(const struct device *dev, uint8_t event);
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

#define CMP0_ENABLE  (1U << 28)
#define CMP1_ENABLE  (1U << 29)
#define CMP2_ENABLE  (1U << 30)
#define CMP3_ENABLE  (1U << 31)
#define LPCMP_ENABLE (1U << 24)

#define CMP_FILTER_CONTROL_ENABLE (1U << 0)
#define CMP_PRESCALER_MAX_VALUE   (0x3FU)
#define CMP_POLARITY_MAX_VALUE    (0x2U)
#define CMP_WINDOW_MAX_VALUE      (0x3U)
#define CMP_FILTER_MIN_VALUE      (0x2U)
#define CMP_FILTER_MAX_VALUE      (0x8U)

#if defined(CONFIG_SOC_SERIES_ENSEMBLE_E7) || defined(CONFIG_SOC_SERIES_ENSEMBLE_E3)
#define CMP_WINDOW_CONTROL_ENABLE (3U)
#define CMP_INT_STATUS_MASK       (1U)
#elif defined(CONFIG_SOC_SERIES_ENSEMBLE_E1C) || defined(CONFIG_SOC_SERIES_BALLETTO_B1)
#define CMP_WINDOW_CONTROL_ENABLE (1U)
#define CMP_INT_STATUS_MASK       (3U)
#endif

#define CMP_INT_MASK        (0x01UL)
#define CMP_INTERRUPT_CLEAR (0x01UL)

#define CMP_FILTER_EVENT0_CLEAR (1U << 0)
#define CMP_FILTER_EVENT1_CLEAR (1U << 1)

#define LPCMP_MSK_CTRL_VAL (0xFEU << 24)

/* Comparator reg1 macro */
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

static void cmp_clk_config(void)
{
	uint32_t data;

	/* Enable LDO and BG for the ANALOG*/
	data = sys_read32(ANA_VBAT_REG2);
	data |= (BIT(22) | BIT(23));
	sys_write32(data, ANA_VBAT_REG2);

	/* Enable CMP Clock Control */
	data = sys_read32(EXPSLV_CMP_CTRL);
	data |= (BIT(0) | BIT(4) | BIT(8) | BIT(12));
	sys_write32(data, EXPSLV_CMP_CTRL);

	/* Enable LPCMP Clock Control */
	data = sys_read32(ANA_VBAT_REG1);
	data |= BIT(14);
	sys_write32(data, ANA_VBAT_REG1);
}

static void cmp_analog_config(const struct device *dev)
{
	uint32_t data;
	uint32_t regs;

	regs = DEVICE_MMIO_NAMED_GET(dev, config_reg);

	/* Enable ANALOG Configurations*/
	data = sys_read32(regs + CMP_COMP_REG2);
	data |= (DAC6_VREF_SCALE | DAC6_CONT | DAC6_EN | DAC12_VREF_CONT | ADC_VREF_BUF_RDIV_EN |
		 ADC_VREF_BUF_EN | ADC_VREF_CONT | ANA_PERIPH_LDO_CONT | ANA_PERIPH_BG_CONT);
	sys_write32(data, (regs + CMP_COMP_REG2));
}

void cmp_enable_interrupt(uintptr_t cmp)
{
	uint32_t data;

	data = sys_read32(cmp + CMP_INTERRUPT_MASK);
	data = 0U;
	sys_write32(data, cmp + CMP_INTERRUPT_MASK);
}

static inline void cmp_set_polarity_ctrl(uintptr_t cmp, uint8_t polarity)
{
	uint32_t data;

	data = sys_read32(cmp + CMP_POLARITY_CTRL);
	data |= polarity;
	sys_write32(data, cmp + CMP_POLARITY_CTRL);
}

void cmp_alif_window_ctrl(const struct device *dev, uint32_t windowing)
{
	uint32_t data;
	uint32_t regs;

	regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);

	data = sys_read32(regs + CMP_WINDOW_CTRL);
	data |= (CMP_WINDOW_CONTROL_ENABLE | windowing << 8);
	sys_write32(data, regs + CMP_WINDOW_CTRL);
}

static inline void cmp_set_filter_ctrl(uintptr_t cmp, uint32_t filter)
{
	uint32_t data;

	data = sys_read32(cmp + CMP_FILTER_CTRL);
	data |= (CMP_FILTER_CONTROL_ENABLE | filter << 8);
	sys_write32(data, cmp + CMP_FILTER_CTRL);
}

static inline void cmp_prescaler_ctrl(uintptr_t cmp, uint8_t presclar)
{
	uint32_t data;

	data = sys_read32(cmp + CMP_PRESCALER_CTRL);
	data |= presclar;
	sys_write32(data, cmp + CMP_PRESCALER_CTRL);
}

static int cmp_alif_setup(const struct device *dev, struct cmp_params *setup)
{
	uintptr_t regs = 0;

	regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);
	const struct cmp_config *config = dev->config;
	struct cmp_data *data = dev->data;

	data->callback = setup->callback;
	data->polarity = setup->polarity;

	if (config->drv_inst != CMP_INSTANCE_LP) {
		/* polarity setup */
		cmp_set_polarity_ctrl(regs, setup->polarity);

		/* filter tap setup */
		cmp_set_filter_ctrl(regs, setup->filter_taps);

		/* prescaler setup */
		cmp_prescaler_ctrl(regs, setup->prescalar);
	}

	return 0;
}

static void lpcmp_set_config(const struct device *dev)
{
	uint32_t data;
	const struct cmp_config *config = dev->config;

	data = sys_read32(ANA_VBAT_REG2);

	data |= config->positive_inp << 25 | config->negative_inp << 27 |
		config->hysteresis_level << 29;

	sys_write32(data, ANA_VBAT_REG2);
}

static void cmp_set_config(const struct device *dev)
{
	uintptr_t regs = 0;
	uint32_t data = 0;

	const struct cmp_config *config = dev->config;

	regs = DEVICE_MMIO_NAMED_GET(dev, config_reg);

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

	regs = DEVICE_MMIO_NAMED_GET(dev, config_reg);

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
		data |= LPCMP_ENABLE;
		break;
	}

	sys_write32(data, regs);
}

int cmp_alif_start(const struct device *dev)
{
	uintptr_t regs = 0;
	const struct cmp_config *config = dev->config;

	regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);

	/* Enable the Comparator module */
	enable_cmp(dev);

	if (config->drv_inst != CMP_INSTANCE_LP) {
		/* enable the interrupt(unmask the interrupt 0x0)*/
		cmp_enable_interrupt(regs);
	}

	return 0;
}

void cmp_irq_handler(const struct device *dev)
{
	uintptr_t regs = 0;
	uint8_t status = 0;

	regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);
	const struct cmp_config *config = dev->config;
	const struct cmp_data *data = dev->data;

	if (config->drv_inst != CMP_INSTANCE_LP) {
		uint8_t int_status =
			(sys_read32(regs + CMP_INTERRUPT_STATUS) & CMP_INT_STATUS_MASK);

		/* clear the interrupt before re-starting */
		if (int_status == CMP_FILTER_EVENT0_CLEAR) {
			sys_write32(CMP_FILTER_EVENT0_CLEAR, regs + CMP_INTERRUPT_STATUS);
		}

		if (int_status == CMP_FILTER_EVENT1_CLEAR) {
			sys_write32(CMP_FILTER_EVENT1_CLEAR, regs + CMP_INTERRUPT_STATUS);
		}

		/* read pin status */
		status = gpio_pin_get_dt(&config->cmp_gpio);
	}

	if (data->callback) {
		data->callback(dev, status);
	}
}

static int cmp_init(const struct device *dev)
{
	int err;
	int ret;
	uintptr_t regs;
	const struct cmp_config *config = dev->config;

	DEVICE_MMIO_NAMED_MAP(dev, cmp_reg, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, config_reg, K_MEM_CACHE_NONE);

	regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

	/* comp clock configuration */
	cmp_clk_config();

	/* compartor set configuration */
	cmp_analog_config(dev);

	if (config->drv_inst == CMP_INSTANCE_LP) {
		/* LPCMP configuration value to the Vbat reg2 */
		lpcmp_set_config(dev);
	} else {
		/* value to the reg1 of CMP0 instance
		 * which controls
		 */
		cmp_set_config(dev);

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

struct cmp_driver_api alif_cmp_api = {
	.windowing = &cmp_alif_window_ctrl,
	.setup = &cmp_alif_setup,
	.start = &cmp_alif_start,
};

#define CMP_ALIF_INIT(inst)                                                                        \
                                                                                                   \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, pinctrl_0), (PINCTRL_DT_INST_DEFINE(inst)));        \
                                                                                                   \
	static void cmp_config_func_##inst(const struct device *dev);                              \
	const struct cmp_config config_##inst = {                                                  \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(cmp_reg, DT_DRV_INST(inst)),                    \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(config_reg, DT_DRV_INST(inst)),                 \
		.irq_config_func = cmp_config_func_##inst,                                         \
		.cmp_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, cmp_gpios, {0}),                        \
		.drv_inst = DT_INST_ENUM_IDX(inst, driver_instance),                               \
		.positive_inp = DT_INST_ENUM_IDX(inst, positive_input),                            \
		.negative_inp = DT_INST_ENUM_IDX(inst, negative_input),                            \
		.hysteresis_level = DT_INST_ENUM_IDX(inst, hysteresis_level),                      \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, pinctrl_0),                                 \
			   (.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_DRV_INST(inst)),))};              \
                                                                                                   \
	struct cmp_data data_##inst;                                                               \
	DEVICE_DT_INST_DEFINE(inst, cmp_init, NULL, &data_##inst, &config_##inst, POST_KERNEL,     \
			      CONFIG_CMP_INIT_PRIORITY, &alif_cmp_api);                            \
                                                                                                   \
	static void cmp_config_func_##inst(const struct device *dev)                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), cmp_irq_handler,      \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
                                                                                                   \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}

DT_INST_FOREACH_STATUS_OKAY(CMP_ALIF_INIT)
