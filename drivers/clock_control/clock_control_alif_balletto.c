/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_clk

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>

#include <zephyr/dt-bindings/clock/alif_balletto_clocks.h>
#include "alif_clock_control.h"

LOG_MODULE_REGISTER(alif_clock_control, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

struct clock_control_alif_config {
	uint32_t master_clkctrl_base;
	uint32_t slave_clkctrl_base;
	uint32_t aon_clkctrl_base;
	uint32_t vbat_clkctrl_base;
	uint32_t m55he_clkctrl_base;
	uint32_t cgu_clkctrl_base;
};

#define OSC_CLOCK_SRC_FREQ(clk)       DT_PROP(DT_PATH(clocks, clk), clock_frequency)
#define PLL_CLOCK1_SRC_FREQ           DT_PROP(DT_PATH(clocks, pll), pll_clk1_frequency)
#define PLL_CLOCK2_SRC_FREQ           DT_PROP(DT_PATH(clocks, pll), pll_clk2_frequency)
#define PLL_CLOCK3_SRC_FREQ           DT_PROP(DT_PATH(clocks, pll), pll_clk3_frequency)

#define ALIF_CLOCK_SYST_CORE_FREQ     CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
#define ALIF_CLOCK_SYST_ACLK_FREQ     PLL_CLOCK1_SRC_FREQ
#define ALIF_CLOCK_SYST_HCLK_FREQ     (PLL_CLOCK1_SRC_FREQ / 2U)
#define ALIF_CLOCK_SYST_PCLK_FREQ     (PLL_CLOCK1_SRC_FREQ / 4U)
#define ALIF_CLOCK_REFCLK_FREQ        (PLL_CLOCK1_SRC_FREQ / 2U)
#define ALIF_CLOCK_USB_CLK_FREQ       (PLL_CLOCK1_SRC_FREQ / 8U)
#define ALIF_CLOCK_160M_CLK_FREQ      PLL_CLOCK1_SRC_FREQ
#define ALIF_CLOCK_10M_CLK_FREQ       (PLL_CLOCK1_SRC_FREQ / 16U)
#define ALIF_CLOCK_USB_CLK_FREQ       (PLL_CLOCK1_SRC_FREQ / 8U)
#define ALIF_CLOCK_PLL_CLK3_FREQ      PLL_CLOCK3_SRC_FREQ
#define ALIF_CLOCK_HFOSC_CLK_FREQ     OSC_CLOCK_SRC_FREQ(hfxo)
#define ALIF_CLOCK_76M8_CLK_FREQ      (OSC_CLOCK_SRC_FREQ(hfxo) * 2U)
#define ALIF_CLOCK_128K_CLK_FREQ      (OSC_CLOCK_SRC_FREQ(lfrc) * 4U)
#define ALIF_CLOCK_S32K_CLK_FREQ      OSC_CLOCK_SRC_FREQ(lfxo)

/** register offset (from clkid cell) */
#define ALIF_CLOCK_CFG_REG(id) (((id) >> ALIF_CLOCK_REG_SHIFT) & ALIF_CLOCK_REG_MASK)
/** enable bit (from clkid cell) */
#define ALIF_CLOCK_CFG_ENABLE(id) (((id) >> ALIF_CLOCK_EN_BIT_POS_SHIFT) & \
					ALIF_CLOCK_EN_BIT_POS_MASK)
/** clock control mask (from clkid cell) */
#define ALIF_CLOCK_CFG_EN_MASK(id) ((id) >> ALIF_CLOCK_EN_MASK_SHIFT)
/** clock source name (from clkid cell) */
#define ALIF_CLOCK_CFG_CLK_SRC(id) (((id) >> ALIF_CLOCK_SRC_SHIFT) & ALIF_CLOCK_SRC_MASK)
/** clock source mask (from clkid cell) */
#define ALIF_CLOCK_CFG_CLK_SRC_MASK(id) (((id) >> ALIF_CLOCK_SRC_MASK_SHIFT) & \
					ALIF_CLOCK_SRC_MASK_MASK)
/** clock source bit position (from clkid cell) */
#define ALIF_CLOCK_CFG_CLK_BIT_POS(id) (((id) >> ALIF_CLOCK_SRC_BIT_POS_SHIFT) & \
					ALIF_CLOCK_SRC_BIT_POS_MASK)
/** clock module (from clkid cell) */
#define ALIF_CLOCK_CFG_MODULE(id) (((id) >> ALIF_CLOCK_MODULE_SHIFT) & ALIF_CLOCK_MODULE_MASK)


static uint32_t alif_get_input_clock(uint32_t clock_name)
{
	switch (clock_name) {
	case ALIF_CDC200_PIX_SYST_ACLK:
	case ALIF_OSPI_CLK:
	case ALIF_UTIMER_CLK:
		return ALIF_CLOCK_SYST_ACLK_FREQ;
	case ALIF_CDC200_PIX_PLL_CLK3:
		return ALIF_CLOCK_PLL_CLK3_FREQ;
	case ALIF_CANFD0_HFOSC_CLK:
	case ALIF_CANFD1_HFOSC_CLK:
		return ALIF_CLOCK_HFOSC_CLK_FREQ;
	case ALIF_CANFD0_160M_CLK:
	case ALIF_CANFD1_160M_CLK:
		return ALIF_CLOCK_160M_CLK_FREQ;
	case ALIF_I2S0_76M8_CLK:
	case ALIF_I2S1_76M8_CLK:
	case ALIF_LPI2S_76M8_CLK:
		return ALIF_CLOCK_76M8_CLK_FREQ;
	case ALIF_LPRTCA_CLK:
	case ALIF_LPRTCB_CLK:
	case ALIF_GPIO0_DB_CLK:
	case ALIF_GPIO1_DB_CLK:
	case ALIF_GPIO2_DB_CLK:
	case ALIF_GPIO3_DB_CLK:
	case ALIF_GPIO4_DB_CLK:
	case ALIF_GPIO5_DB_CLK:
	case ALIF_GPIO6_DB_CLK:
	case ALIF_GPIO7_DB_CLK:
	case ALIF_GPIO8_DB_CLK:
	case ALIF_GPIO9_DB_CLK:
	case ALIF_LPTIMER0_S32K_CLK:
	case ALIF_LPTIMER1_S32K_CLK:
		return ALIF_CLOCK_S32K_CLK_FREQ;
	case ALIF_LPTIMER0_128K_CLK:
	case ALIF_LPTIMER1_128K_CLK:
		return ALIF_CLOCK_128K_CLK_FREQ;
#if CONFIG_COUNTER_SNPS_DW
	case ALIF_LPTIMER0_LPTMR0_IO_PIN:
		return CONFIG_LPTIMER0_EXT_CLK_FREQ;
	case ALIF_LPTIMER1_LPTMR1_IO_PIN:
		return CONFIG_LPTIMER1_EXT_CLK_FREQ;
#endif
	case ALIF_I3C_CLK:
		return ALIF_CLOCK_SYST_ACLK_FREQ;
	default:
		return 0;
	}

	return 0;
}

void alif_get_div_reg_info(uint32_t clock_name, uint32_t *mask,
			uint32_t *pos)
{
	switch (clock_name) {
	case ALIF_CDC200_PIX_SYST_ACLK:
	case ALIF_CDC200_PIX_PLL_CLK3:
		*mask = ALIF_CDC200_PIX_CLK_DIV_MASK;
		*pos = ALIF_CDC200_PIX_CLK_DIV_POS;
		break;
		break;
	case ALIF_CANFD0_HFOSC_CLK:
	case ALIF_CANFD0_160M_CLK:
		*mask = ALIF_CANFD0_CLK_DIV_MASK;
		*pos = ALIF_CANFD0_CLK_DIV_POS;
		break;
	case ALIF_CANFD1_HFOSC_CLK:
	case ALIF_CANFD1_160M_CLK:
		*mask = ALIF_CANFD1_CLK_DIV_MASK;
		*pos = ALIF_CANFD1_CLK_DIV_POS;
		break;
	case ALIF_I2S0_76M8_CLK:
	case ALIF_I2S1_76M8_CLK:
	case ALIF_LPI2S_76M8_CLK:
		*mask = ALIF_I2S_CLK_DIV_MASK;
		*pos = ALIF_I2S_CLK_DIV_POS;
		break;
	case ALIF_GPIO0_DB_CLK:
	case ALIF_GPIO1_DB_CLK:
	case ALIF_GPIO2_DB_CLK:
	case ALIF_GPIO3_DB_CLK:
	case ALIF_GPIO4_DB_CLK:
	case ALIF_GPIO5_DB_CLK:
	case ALIF_GPIO6_DB_CLK:
	case ALIF_GPIO7_DB_CLK:
	case ALIF_GPIO8_DB_CLK:
	case ALIF_GPIO9_DB_CLK:
		*mask = ALIF_GPIO_DB_CLK_DIV_MASK;
		*pos = ALIF_GPIO_DB_CLK_DIV_POS;
		break;
	default:
		*mask = 0U;
		*pos = 0U;
		break;
	}
}

static int32_t alif_get_module_base(const struct device *dev, uint32_t module, uint32_t *base)
{
	const struct clock_control_alif_config *config = dev->config;

	switch (module) {
	case ALIF_DUMMY_CLKCTL_MODULE:
		break;
	case ALIF_PER_MST_CLKCTL_MODULE:
		*base = config->master_clkctrl_base;
		break;
	case ALIF_PER_SLV_CLKCTL_MODULE:
		*base = config->slave_clkctrl_base;
		break;
	case ALIF_AON_CLKCTL_MODULE:
		*base = config->aon_clkctrl_base;
		break;
	case ALIF_VBAT_CLKCTL_MODULE:
		*base = config->vbat_clkctrl_base;
		break;
	case ALIF_M55HE_CLKCTL_MODULE:
		*base = config->m55he_clkctrl_base;
		break;
	case ALIF_CGU_CLKCTL_MODULE:
		*base = config->cgu_clkctrl_base;
		break;
	default:
		LOG_ERR("ERROR: Un-supported clock module\n");
		return -EINVAL;
	}

	return 0;
}

static int alif_clock_control_on(const struct device *dev,
			clock_control_subsys_t sub_system)
{
	uint32_t clk_id = (uint32_t) sub_system;
	uint32_t module_base, reg_addr;
	int32_t ret;

	if (!ALIF_CLOCK_CFG_EN_MASK(clk_id)) {
		LOG_ERR("ERROR: Clock enable not supported\n");
		return -ENOTSUP;
	}

	ret = alif_get_module_base(dev, ALIF_CLOCK_CFG_MODULE(clk_id),
					&module_base);
	if (ret) {
		return ret;
	}
	reg_addr = module_base + ALIF_CLOCK_CFG_REG(clk_id);

	sys_set_bit(reg_addr, ALIF_CLOCK_CFG_ENABLE(clk_id));

	return 0;
}

static int alif_clock_control_off(const struct device *dev,
			clock_control_subsys_t sub_system)
{
	uint32_t clk_id = *(uint32_t *) sub_system;
	uint32_t module_base, reg_addr;
	int32_t ret;

	if (!ALIF_CLOCK_CFG_EN_MASK(clk_id)) {
		LOG_ERR("ERROR: Clock disable not supported\n");
		return -ENOTSUP;
	}

	ret = alif_get_module_base(dev, ALIF_CLOCK_CFG_MODULE(clk_id),
					&module_base);
	if (ret) {
		return ret;
	}
	reg_addr = module_base + ALIF_CLOCK_CFG_REG(clk_id);

	sys_clear_bit(reg_addr, ALIF_CLOCK_CFG_ENABLE(clk_id));

	return 0;
}

static int alif_clock_control_set_rate(const struct device *dev,
				clock_control_subsys_t sub_system,
				clock_control_subsys_rate_t rate)
{
	uint32_t clk_id = (uint32_t) sub_system;
	uint32_t clk_freq, reg_addr, module_base;
	uint32_t div_mask, freq_div, div_pos;
	uint32_t frequency = (uint32_t) rate;
	int32_t ret;

	clk_freq = alif_get_input_clock(clk_id);
	if (!clk_freq) {
		return -ENOTSUP;
	}

	ret = alif_get_module_base(dev, ALIF_CLOCK_CFG_MODULE(clk_id),
					&module_base);
	if (ret) {
		return ret;
	}
	reg_addr = module_base + ALIF_CLOCK_CFG_REG(clk_id);

	freq_div = (clk_freq / frequency);

	alif_get_div_reg_info(clk_id, &div_mask, &div_pos);

	if (!div_mask) {
		LOG_ERR("ERROR: Frequency setting is not possible\n");
		return -ENOTSUP;
	}
	if (freq_div > div_mask) {
		LOG_ERR("ERROR: Desired frequency setting is not possible\n");
		return -EINVAL;
	}

	alif_set_clock_divisor((uint32_t *) reg_addr, div_mask, div_pos, freq_div);

	return 0;
}

static int alif_clock_control_get_rate(const struct device *dev,
				clock_control_subsys_t sub_system,
				uint32_t *rate)
{
	uint32_t clk_id = (uint32_t) sub_system;
	uint32_t clk_freq, reg_addr, module_base;
	uint32_t div_mask, freq_div, div_pos;
	int32_t ret;

	clk_freq = alif_get_input_clock(clk_id);
	if (!clk_freq) {
		return -ENOTSUP;
	}

	ret = alif_get_module_base(dev, ALIF_CLOCK_CFG_MODULE(clk_id),
					&module_base);
	if (ret) {
		return ret;
	}
	reg_addr = module_base + ALIF_CLOCK_CFG_REG(clk_id);

	alif_get_div_reg_info(clk_id, &div_mask, &div_pos);

	if (div_mask) {
		freq_div = alif_get_clock_divisor((uint32_t *) reg_addr, div_mask, div_pos);
		*rate = (clk_freq / freq_div);
	} else {
		*rate = clk_freq;
	}

	return 0;
}


static enum clock_control_status
	alif_clock_control_get_status(const struct device *dev,
				clock_control_subsys_t sub_system)
{
	uint32_t clk_id = (uint32_t) sub_system;
	uint32_t reg_addr, module_base;

	if (!ALIF_CLOCK_CFG_EN_MASK(clk_id)) {
		LOG_ERR("ERROR: Clock status not avilable\n");
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	alif_get_module_base(dev, ALIF_CLOCK_CFG_MODULE(clk_id),
					&module_base);
	reg_addr = module_base + ALIF_CLOCK_CFG_REG(clk_id);

	if (sys_test_bit(reg_addr, ALIF_CLOCK_CFG_ENABLE(clk_id)) != 0) {
		return CLOCK_CONTROL_STATUS_ON;
	}

	return CLOCK_CONTROL_STATUS_OFF;
}

static inline int alif_clock_control_configure(const struct device *dev,
						clock_control_subsys_t sub_system,
						void *data)
{
	uint32_t clk_id = (uint32_t) sub_system;
	uint32_t reg_addr, reg_value, module_base;
	int32_t ret;

	ARG_UNUSED(data);

	if (!ALIF_CLOCK_CFG_CLK_SRC_MASK(clk_id)) {
		return -ENOTSUP;
	}

	ret = alif_get_module_base(dev, ALIF_CLOCK_CFG_MODULE(clk_id),
					&module_base);
	if (ret) {
		return ret;
	}
	reg_addr = module_base + ALIF_CLOCK_CFG_REG(clk_id);

	reg_value = sys_read32(reg_addr);
	reg_value &= ~(ALIF_CLOCK_CFG_CLK_SRC_MASK(clk_id) << ALIF_CLOCK_CFG_CLK_BIT_POS(clk_id));
	reg_value |= (ALIF_CLOCK_CFG_CLK_SRC(clk_id) << ALIF_CLOCK_CFG_CLK_BIT_POS(clk_id));
	sys_write32(reg_value, reg_addr);

	return 0;
}

static const struct clock_control_driver_api alif_clock_control_driver_api = {
	.on = alif_clock_control_on,
	.off = alif_clock_control_off,
	.set_rate = alif_clock_control_set_rate,
	.get_rate = alif_clock_control_get_rate,
	.get_status = alif_clock_control_get_status,
	.configure = alif_clock_control_configure
};

static const struct clock_control_alif_config config = {
	.master_clkctrl_base = DT_INST_REG_ADDR_BY_NAME(0, master_clkctrl),
	.slave_clkctrl_base = DT_INST_REG_ADDR_BY_NAME(0, slave_clkctrl),
	.aon_clkctrl_base = DT_INST_REG_ADDR_BY_NAME(0, aon_clkctrl),
	.vbat_clkctrl_base = DT_INST_REG_ADDR_BY_NAME(0, vbat_clkctrl),
	.m55he_clkctrl_base = DT_INST_REG_ADDR_BY_NAME(0, m55he_clkctrl),
	.cgu_clkctrl_base = DT_INST_REG_ADDR_BY_NAME(0, cgu_clkctrl),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &config, PRE_KERNEL_1,
				CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
				&alif_clock_control_driver_api);
