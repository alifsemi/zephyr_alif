/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_dac

#include <stddef.h>
#include <stdint.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>
#include <soc_common.h>
#include "analog_ctrl.h"

LOG_MODULE_REGISTER(DAC);

struct dac_config {
	DEVICE_MMIO_NAMED_ROM(cmp_reg);
	DEVICE_MMIO_NAMED_ROM(dac_reg);
	DEVICE_MMIO_NAMED_ROM(vbat_reg);
#if CONFIG_ANALOG_ALIASING
	DEVICE_MMIO_NAMED_ROM(adc_vref);
#endif
	const struct pinctrl_dev_config *pcfg;
	bool bypass_enabled;
	uint32_t bypass_val;
	bool twoscomp_enabled;
	uint8_t capacitance;
	uint8_t output_current;
};

struct dac_data {
	DEVICE_MMIO_NAMED_RAM(cmp_reg);
	DEVICE_MMIO_NAMED_RAM(dac_reg);
	DEVICE_MMIO_NAMED_RAM(vbat_reg);
#if CONFIG_ANALOG_ALIASING
	DEVICE_MMIO_NAMED_RAM(adc_vref);
#endif
};

#define DEV_DATA(dev) ((struct dac_data *)((dev)->data))
#define DEV_CFG(dev)  ((const struct dac_config *)((dev)->config))

/* DAC register offsets */
#define DAC_REG1				(0x00)
#define DAC_IN					(0x04)

/* CMP register offset */
#define CMP_COMP_REG2_OFFSET			(0x4)
#define VBAT_REG2_OFFSET			(0x3C)

/* DAC  Control register */
#define DAC_REG1_DAC_EN				(1U << 0)
#define DAC_REG1_DAC_RESET_B			(1U << 27)
#define DAC_REG1_DAC_HP_MODE_EN			(1U << 18)
#define DAC_MAX_INPUT				(0xFFFU)
#define DAC_MIN_INPUT				(0x0U)
#define DAC_REG1_DAC_IN_BYP_MUX			(1U << 1U)
#define DAC_MAX_BYP_VAL_Msk			(0x3FFCU)
#define DAC_REG1_DAC_TWOSCOMP_EN		22U
#define DAC_REG1_DAC_INPUT_BYP_MUX_EN		1U
#define DAC_REG1_DAC_BYP_VAL_Pos		2U
#define DAC_REG1_DAC_IBIAS_VAL_Pos		23U
#define DAC_REG1_DAC_CAP_CONT_Pos		14U
#define DAC_CTRL_DAC0_CKEN			(1U << 0U)
#define DAC_CTRL_DAC1_CKEN			(1U << 4U)

#define CMP_COMP_REG2_DAC6_VREF_SCALE		(0x1U << 27)
#define CMP_COMP_REG2_DAC6_CONT			(0x20U << 21)
#define CMP_COMP_REG2_DAC6_EN			(0x1U << 20)
#define CMP_COMP_REG2_DAC12_VREF_CONT		(0x4U << 17)
#define CMP_COMP_REG2_ANA_PERIPH_LDO_CONT	(0xAU << 6)
#define CMP_COMP_REG2_ANA_PERIPH_BG_CONT	(0xAU << 1)
#define CMP_COMP_REG2_ADC_VREF_CONT		(0x10U << 10)
#define CMP_COMP_REG2_ADC_VREF_BUF_EN		(0x1U << 15)
#define CMP_COMP_REG2_ADC_VREF_BUF_RDIV_EN	(0x0U << 16)

#define DAC12_MAX_RESOLUTION			12

static inline void enable_dac_periph_clk(void)
{
	uint32_t data;

	/* Enable DAC Clock Control */
	data = sys_read32(EXPSLV_DAC_CTRL);
	data |= (DAC_CTRL_DAC0_CKEN | DAC_CTRL_DAC1_CKEN);
	sys_write32(data, EXPSLV_DAC_CTRL);
}

static void analog_config(const struct device *dev)
{
	uintptr_t vbat_reg_base = DEVICE_MMIO_NAMED_GET(dev, vbat_reg);
	uintptr_t cmp_regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);

	/* Calculate specific register addresses */
	uintptr_t vbat_reg2_addr = vbat_reg_base + VBAT_REG2_OFFSET;
	uintptr_t cmp_reg2_addr = cmp_regs + CMP_COMP_REG2_OFFSET;

	unsigned int key = irq_lock();

	/* Analog configuration Vbat register2 */
	enable_analog_peripherals(vbat_reg2_addr);

	irq_unlock(key);

	uintptr_t adc_vref_base = 0;
#if CONFIG_ANALOG_ALIASING
	adc_vref_base = DEVICE_MMIO_NAMED_GET(dev, adc_vref);
#endif

	key = irq_lock();

	/* Enables DAC12 voltage reference and internal buffer for DAC operation */
	enable_dac12_ref_voltage(cmp_reg2_addr, adc_vref_base);

	irq_unlock(key);
}

static int dac_enable(const struct device *dev, const struct dac_channel_cfg *channel_cfg)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);
	uint32_t data;

	if (channel_cfg->resolution != DAC12_MAX_RESOLUTION) {
		return -ENOTSUP;
	}

	data = sys_read32(regs + DAC_REG1);
	data |= DAC_REG1_DAC_EN;
	sys_write32(data, (regs + DAC_REG1));

	return 0;
}

static void dac_disable(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);
	uint32_t data;

	data = sys_read32(regs + DAC_REG1);
	data &= ~(DAC_REG1_DAC_EN);
	sys_write32(data, (regs + DAC_REG1));
}

static inline void dac_set_config(const struct device *dev)
{
	const struct dac_config *config = DEV_CFG(dev);
	uintptr_t reg_base = DEVICE_MMIO_NAMED_GET(dev, dac_reg);
	uint32_t data = 0U;

	data = sys_read32(reg_base + DAC_REG1);

	data |= ((config->bypass_enabled << DAC_REG1_DAC_INPUT_BYP_MUX_EN) |
			(config->twoscomp_enabled << DAC_REG1_DAC_TWOSCOMP_EN) |
			(config->capacitance << DAC_REG1_DAC_CAP_CONT_Pos) |
			(config->output_current << DAC_REG1_DAC_IBIAS_VAL_Pos));

	sys_write32(data, (reg_base + DAC_REG1));
}

static inline void dac_clear_config(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);
	uint32_t data = 0U;

	sys_write32(data, (regs + DAC_REG1));
}

static inline void dac_hp_mode_enable(uintptr_t dac)
{
	uint32_t data;

	data = sys_read32(dac + DAC_REG1);
	data |= DAC_REG1_DAC_HP_MODE_EN;
	sys_write32(data, (dac + DAC_REG1));
}

static inline void dac_lp_mode_enable(uintptr_t dac)
{
	uint32_t data;

	data = sys_read32(dac + DAC_REG1);
	data &= ~DAC_REG1_DAC_HP_MODE_EN;
	sys_write32(data, (dac + DAC_REG1));
}

static void dac_reset_deassert(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);
	uint32_t data;

	data = sys_read32(regs + DAC_REG1);
	data |= DAC_REG1_DAC_RESET_B;
	sys_write32(data, (regs + DAC_REG1));
}

static void dac_set_bypass_input(const struct device *dev, uint32_t bypass_val)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);
	uint32_t data;

	data = sys_read32(regs + DAC_REG1);
	data &= ~(DAC_MAX_BYP_VAL_Msk);
	sys_write32(data, (regs + DAC_REG1));

	bypass_val = (bypass_val & DAC_MAX_INPUT);

	data |= (bypass_val << DAC_REG1_DAC_BYP_VAL_Pos);
	sys_write32(data, (regs + DAC_REG1));
}

static bool dac_input_mux_enabled(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);
	uint32_t data;

	data = sys_read32(regs + DAC_REG1);
	return (data & DAC_REG1_DAC_IN_BYP_MUX) ? true : false;
}

static int dac_write_data(const struct device *dev, uint8_t channel, uint32_t input_value)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);

	const struct dac_config *config = DEV_CFG(dev);

	if (config->twoscomp_enabled != 1 &&
	(input_value > DAC_MAX_INPUT || input_value < DAC_MIN_INPUT)) {
		return -EINVAL;
	}

	/* If bypass mode is not enabled then pass
	 * the input through the DAC_IN reg
	 */
	if (!(dac_input_mux_enabled(dev))) {
		sys_write32(input_value, (regs + DAC_IN));
	} else {
		dac_set_bypass_input(dev, config->bypass_val);
	}

	/* There are no channels in DAC */
	ARG_UNUSED(channel);

	return 0;
}

static int dac_init(const struct device *dev)
{
	int err;
	const struct dac_config *config = DEV_CFG(dev);

	uintptr_t regs;

	DEVICE_MMIO_NAMED_MAP(dev, cmp_reg, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, dac_reg, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, vbat_reg, K_MEM_CACHE_NONE);

#if CONFIG_ANALOG_ALIASING
	DEVICE_MMIO_NAMED_MAP(dev, adc_vref, K_MEM_CACHE_NONE);
#endif

	regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

	unsigned int key = irq_lock();

	enable_analog_periph_clk(EXPSLV_CMP_CTRL);

	irq_unlock(key);

	enable_dac_periph_clk();

	analog_config(dev);

	dac_disable(dev);

	dac_reset_deassert(dev);

	dac_set_config(dev);

	return 0;
}

DEVICE_API(dac, alif_dac_driver_api) = {
	.write_value = dac_write_data,
	.channel_setup = dac_enable
};

#define CHECK_INPUT_BYPASS_VALUES(n)								\
	BUILD_ASSERT(DT_INST_PROP(n, bypass_val) <= DAC_MAX_INPUT,				\
		"DAC input bypass value exceeds maximum of " STRINGIFY(DAC_MAX_INPUT))

#define DAC_ALIF_INIT(n)									\
	COND_CODE_1(DT_INST_PROP(n, bypass_enabled), (CHECK_INPUT_BYPASS_VALUES(n)), ());	\
	static struct dac_data  dac_data_##n;							\
	IF_ENABLED(DT_INST_NODE_HAS_PROP(n, pinctrl_0), (PINCTRL_DT_INST_DEFINE(n)));		\
	static const struct dac_config dac_config_##n = {					\
			DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(cmp_reg, DT_DRV_INST(n)),		\
			DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(dac_reg, DT_DRV_INST(n)),		\
			DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(vbat_reg, DT_DRV_INST(n)),		\
			IF_ENABLED(CONFIG_ANALOG_ALIASING,					\
			(DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(adc_vref, DT_DRV_INST(n)),))	\
			.twoscomp_enabled = DT_INST_PROP(n, twoscomp_enabled),			\
			.output_current = DT_INST_ENUM_IDX(n, output_current),			\
			.capacitance = DT_INST_ENUM_IDX(n, capacitance),			\
			IF_ENABLED(DT_INST_NODE_HAS_PROP(n, pinctrl_0),				\
			(.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_DRV_INST(n)),))			\
			COND_CODE_1(DT_INST_PROP(n, bypass_enabled),				\
			(.bypass_enabled = 1U,							\
			.bypass_val = DT_INST_PROP(n, bypass_val)), ())				\
		};										\
	DEVICE_DT_INST_DEFINE(n, dac_init, NULL,						\
			&dac_data_##n,								\
			&dac_config_##n,							\
			POST_KERNEL, CONFIG_DAC_INIT_PRIORITY,					\
			&alif_dac_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DAC_ALIF_INIT)
