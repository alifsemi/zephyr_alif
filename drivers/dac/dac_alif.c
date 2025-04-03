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
#include <zephyr/drivers/dac/alif_dac.h>
#include <zephyr/sys/device_mmio.h>
#include <soc.h>

LOG_MODULE_REGISTER(DAC);

struct dac_config {
	DEVICE_MMIO_NAMED_ROM(cmp_reg);
	DEVICE_MMIO_NAMED_ROM(dac_reg);
	const struct pinctrl_dev_config *pcfg;
	uint8_t input_mux_val;
	bool dac_twoscomp_in;
};

struct dac_data {
	DEVICE_MMIO_NAMED_RAM(cmp_reg);
	DEVICE_MMIO_NAMED_RAM(dac_reg);
};

#define DEV_DATA(dev) ((struct dac_data *)((dev)->data))
#define DEV_CFG(dev)  ((struct dac_config *)((dev)->config))

/* DAC register offsets */
#define DAC_REG1				(0x00)
#define DAC_IN					(0x04)

/* CMP register offset */
#define CMP_COMP_REG2				(0x4)

/* DAC  Control register */
#define DAC_REG1_DAC_EN				(1U << 0)
#define DAC_REG1_DAC_RESET_B			(1U << 27)
#define DAC_REG1_DAC_HP_MODE_EN			(1U << 18)
#define DAC_MAX_INPUT				(0xFFFU)
#define DAC_REG1_DAC_IN_BYP_MUX			(1U << 1U)
#define DAC_MAX_BYP_VAL_Msk			(0x3FFCU)
#define DAC_REG1_DAC_TWOSCOMP_EN		22U
#define DAC_REG1_DAC_INPUT_BYP_MUX_EN		1U
#define DAC_REG1_DAC_BYP_VAL_Pos		2U
#define DAC_REG1_DAC_IBIAS_VAL_Pos		23U
#define DAC_REG1_DAC_CAP_CONT_Pos		14U

#define CMP_COMP_REG2_DAC6_VREF_SCALE		(0x1U << 27)
#define CMP_COMP_REG2_DAC6_CONT			(0x20U << 21)
#define CMP_COMP_REG2_DAC6_EN			(0x1U << 20)
#define CMP_COMP_REG2_DAC12_VREF_CONT		(0x4U << 17)
#define CMP_COMP_REG2_ANA_PERIPH_LDO_CONT	(0xAU << 6)
#define CMP_COMP_REG2_ANA_PERIPH_BG_CONT	(0xAU << 1)
#define CMP_COMP_REG2_ADC_VREF_CONT		(0x10U << 10)
#define CMP_COMP_REG2_ADC_VREF_BUF_EN		(0x1U << 15)
#define CMP_COMP_REG2_ADC_VREF_BUF_RDIV_EN	(0x0U << 16)

static inline void dac_clk_config(void)
{
	uint32_t data;

	/* Enable COMP Clock Control */
	data = sys_read32(EXPSLV_CMP_CTRL);
	data |= (BIT(0));
	sys_write32(data, EXPSLV_CMP_CTRL);

	/* Enable DAC Clock Control */
	data = sys_read32(EXPSLV_DAC_CTRL);
	data |= (BIT(0) | BIT(4));
	sys_write32(data, EXPSLV_DAC_CTRL);

	/* Enable LDO and BG for the ANALOG */
	data = sys_read32(ANA_VBAT_REG2);
	data |= (BIT(22) | BIT(23));
	sys_write32(data, ANA_VBAT_REG2);
}

void dac_analog_config(const struct device *dev)
{
	uintptr_t cmp_regs = DEVICE_MMIO_NAMED_GET(dev, cmp_reg);
	uint32_t cmp_reg2_val;
	uint32_t data;

	cmp_reg2_val = (CMP_COMP_REG2_DAC6_VREF_SCALE | CMP_COMP_REG2_DAC6_CONT |
			CMP_COMP_REG2_DAC6_EN  | CMP_COMP_REG2_DAC12_VREF_CONT |
			CMP_COMP_REG2_ANA_PERIPH_LDO_CONT | CMP_COMP_REG2_ANA_PERIPH_BG_CONT |
			CMP_COMP_REG2_ADC_VREF_BUF_EN | CMP_COMP_REG2_ADC_VREF_CONT |
			CMP_COMP_REG2_ADC_VREF_BUF_RDIV_EN);

	data = sys_read32(cmp_regs + CMP_COMP_REG2);
	data |= cmp_reg2_val;
	sys_write32(data, (cmp_regs + CMP_COMP_REG2));
}

static int dac_enable(const struct device *dev, const struct dac_channel_cfg *channel_cfg)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);
	uint32_t data;

	ARG_UNUSED(channel_cfg);

	data = sys_read32(regs + DAC_REG1);
	data |= DAC_REG1_DAC_EN;
	sys_write32(data, (regs + DAC_REG1));
	return 0;
}

void dac_disable(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);
	uint32_t data;

	data = sys_read32(regs + DAC_REG1);
	data &= ~(DAC_REG1_DAC_EN);
	sys_write32(data, (regs + DAC_REG1));
}

static inline void dac_set_config(uintptr_t dac, uint8_t input_mux_val, uint8_t conv_input)
{
	uint32_t data;

	data = sys_read32(dac + DAC_REG1);

	data |= ((input_mux_val << DAC_REG1_DAC_INPUT_BYP_MUX_EN) |
		(conv_input << DAC_REG1_DAC_TWOSCOMP_EN));
	sys_write32(data, (dac + DAC_REG1));
}

void dac_clear_config(const struct device *dev)
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

void dac_reset_deassert(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);
	uint32_t data;

	data = sys_read32(regs + DAC_REG1);
	data |= DAC_REG1_DAC_RESET_B;
	sys_write32(data, (regs + DAC_REG1));
}

void dac_reset_assert(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);
	uint32_t data;

	data = sys_read32(regs + DAC_REG1);
	data &= ~(DAC_REG1_DAC_RESET_B);
	sys_write32(data, (regs + DAC_REG1));
}

void dac_reset_control(const struct device *dev, bool enable)
{
	if (enable) {
		/* DAC reset asserted */
		dac_reset_assert(dev);
	} else {
		/* DAC reset released */
		dac_reset_deassert(dev);
	}
}

void dac_set_output_current(const struct device *dev, uint8_t cur_val)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);
	uint32_t data;

	data = sys_read32(regs + DAC_REG1);
	data |= (cur_val << DAC_REG1_DAC_IBIAS_VAL_Pos);
	sys_write32(data, (regs + DAC_REG1));
}

void dac_set_bypass_input(const struct device *dev, uint32_t bypass_val)
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

void dac_set_capacitance(const struct device *dev, uint8_t cap_val)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);
	uint32_t data;

	data = sys_read32(regs + DAC_REG1);
	data |= (cap_val << DAC_REG1_DAC_CAP_CONT_Pos);
	sys_write32(data, (regs + DAC_REG1));
}

bool dac_input_mux_enabled(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);
	uint32_t data;

	data = sys_read32(regs + DAC_REG1);
	return (data & DAC_REG1_DAC_IN_BYP_MUX) ? true : false;
}

static int dac_write_data(const struct device *dev, uint8_t channel, uint32_t input_value)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);

	/* If bypass mode is not enabled then pass
	 *the input through the DAC_IN reg
	 */
	if (!(dac_input_mux_enabled(dev))) {
		sys_write32(input_value, (regs + DAC_IN));
	} else {
		dac_set_bypass_input(dev, input_value);
	}

	/* There are no channels in DAC */
	ARG_UNUSED(channel);
	return 0;
}

static int dac_init(const struct device *dev)
{
	int err;
	const struct dac_config *config = dev->config;

	uintptr_t regs;

	DEVICE_MMIO_NAMED_MAP(dev, cmp_reg, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, dac_reg, K_MEM_CACHE_NONE);

	regs = DEVICE_MMIO_NAMED_GET(dev, dac_reg);

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (err != 0) {
			return err;
		}

	dac_clk_config();

	dac_analog_config(dev);

	dac_disable(dev);

	dac_reset_deassert(dev);

	dac_set_config(regs, config->input_mux_val, config->dac_twoscomp_in);

	return 0;
}

static const struct dac_driver_api alif_dac_driver_api = {
	.write_value = dac_write_data,
	.channel_setup = dac_enable
};

#define DAC_ALIF_INIT(n) \
	static struct dac_data  dac_data_##n;							   \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(n, pinctrl_0), (PINCTRL_DT_INST_DEFINE(n)));		   \
	static const struct dac_config dac_config_##n = {					   \
	DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(cmp_reg, DT_DRV_INST(n)),				   \
			DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(dac_reg, DT_DRV_INST(n)),		   \
			.dac_twoscomp_in = DT_INST_PROP(n, dac_twoscomp_in),			   \
			.input_mux_val = DT_INST_PROP(n, input_mux_val),			   \
			 IF_ENABLED(DT_INST_NODE_HAS_PROP(n, pinctrl_0),			   \
				   (.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_DRV_INST(n)),))};	   \
	DEVICE_DT_INST_DEFINE(n, dac_init, NULL,						   \
			&dac_data_##n,								   \
			&dac_config_##n,							   \
			POST_KERNEL, CONFIG_DAC_INIT_PRIORITY,		                           \
			&alif_dac_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DAC_ALIF_INIT)
