/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_clockctrl

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/clock/alif_balletto_clocks.h>
#include <soc_common.h>

LOG_MODULE_REGISTER(alif_clock_control, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

struct clock_control_alif_config {
	uint32_t cgu_base;
	uint32_t clkctl_per_mst_base;
	uint32_t clkctl_per_slv_base;
	uint32_t aon_base;
	uint32_t vbat_base;
	uint32_t m55he_cfg_base;
};

#define ALIF_CAMERA_PIX_CLK_DIV_MASK BIT_MASK(9U)
#define ALIF_CAMERA_PIX_CLK_DIV_POS  16U
#define ALIF_CDC200_PIX_CLK_DIV_MASK BIT_MASK(9U)
#define ALIF_CDC200_PIX_CLK_DIV_POS  16U
#define ALIF_CSI_PIX_CLK_DIV_MASK    BIT_MASK(9U)
#define ALIF_CSI_PIX_CLK_DIV_POS     16U
#define ALIF_CANFD0_CLK_DIV_MASK     BIT_MASK(8U)
#define ALIF_CANFD0_CLK_DIV_POS      0U
#define ALIF_CANFD1_CLK_DIV_MASK     BIT_MASK(8U)
#define ALIF_CANFD1_CLK_DIV_POS      16U
#define ALIF_I2S_CLK_DIV_MASK        BIT_MASK(10U)
#define ALIF_I2S_CLK_DIV_POS         0U
#define ALIF_GPIO_DB_CLK_DIV_MASK    BIT_MASK(10U)
#define ALIF_GPIO_DB_CLK_DIV_POS     0U
#define ALIF_LPI2S_CLK_DIV_MASK      BIT_MASK(10U)
#define ALIF_LPI2S_CLK_DIV_POS       0U
#define ALIF_LPCPI_PIX_CLK_DIV_MASK  BIT_MASK(9U)
#define ALIF_LPCPI_PIX_CLK_DIV_POS   16U

#define OSC_CLOCK_SRC_FREQ(clk) DT_PROP(DT_PATH(clocks, clk), clock_frequency)
#define PLL_CLOCK1_SRC_FREQ     DT_PROP(DT_PATH(clocks, pll_clk1), clock_frequency)
#define PLL_CLOCK2_SRC_FREQ     DT_PROP(DT_PATH(clocks, pll_clk2), clock_frequency)

/* CLK_ENA register config */
#define ALIF_CLK_ENA_CLK38P4M_BIT 23U
#define ALIF_CLK_ENA_CLK20M_BIT   22U
#define ALIF_CLK_ENA_CLK100M_BIT  21U
#define ALIF_CLK_ENA_CLK160M_BIT  20U

#define ALIF_CLOCK_SYST_CORE_FREQ     CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
#define ALIF_CLOCK_HFOSC_CLK_FREQ     OSC_CLOCK_SRC_FREQ(hfxo)
#define ALIF_CLOCK_HFRC_CLK_FREQ      OSC_CLOCK_SRC_FREQ(hfrc)
#define ALIF_CLOCK_76M8_CLK_FREQ      (ALIF_CLOCK_HFOSC_CLK_FREQ * 2U)
#define ALIF_CLOCK_128K_CLK_FREQ      (OSC_CLOCK_SRC_FREQ(lfrc) * 4U)
#define ALIF_CLOCK_S32K_CLK_FREQ      OSC_CLOCK_SRC_FREQ(lfxo)
#define ALIF_CLOCK_AUDIO_PLL_CLK_FREQ ALIF_CLOCK_HFOSC_CLK_FREQ

/** register offset (from clkid cell) */
#define ALIF_CLOCK_CFG_REG(id) (((id) >> ALIF_CLOCK_REG_SHIFT) & ALIF_CLOCK_REG_MASK)
/** enable bit (from clkid cell) */
#define ALIF_CLOCK_CFG_ENABLE(id)                                                                  \
	(((id) >> ALIF_CLOCK_EN_BIT_POS_SHIFT) & ALIF_CLOCK_EN_BIT_POS_MASK)
/** clock control mask (from clkid cell) */
#define ALIF_CLOCK_CFG_EN_MASK(id) ((id) >> ALIF_CLOCK_EN_MASK_SHIFT)
/** clock source name (from clkid cell) */
#define ALIF_CLOCK_CFG_CLK_SRC(id) (((id) >> ALIF_CLOCK_SRC_SHIFT) & ALIF_CLOCK_SRC_MASK)
/** clock source mask (from clkid cell) */
#define ALIF_CLOCK_CFG_CLK_SRC_MASK(id)                                                            \
	(((id) >> ALIF_CLOCK_SRC_MASK_SHIFT) & ALIF_CLOCK_SRC_MASK_MASK)
/** clock source bit position (from clkid cell) */
#define ALIF_CLOCK_CFG_CLK_BIT_POS(id)                                                             \
	(((id) >> ALIF_CLOCK_SRC_BIT_POS_SHIFT) & ALIF_CLOCK_SRC_BIT_POS_MASK)
/** clock module (from clkid cell) */
#define ALIF_CLOCK_CFG_MODULE(id) (((id) >> ALIF_CLOCK_MODULE_SHIFT) & ALIF_CLOCK_MODULE_MASK)

static uint32_t get_hfrc_clk_freq(void)
{
	/* Read HFRC clock divider from VBAT Analog Control 2 */
	const uint32_t divider = (sys_read32(ANA_VBAT_REG2) >> 11) & 0x7;

	return ALIF_CLOCK_HFRC_CLK_FREQ >> divider;
}

static uint32_t get_osc_ctrl_clk_freq(void)
{
	/* Select oscillator clock source for SYSPLL_CLK and SYST_REFCLK */
	if (sys_read32(ALIF_OSC_CTRL_REG + CGU_BASE) & BIT(0)) {
		return ALIF_CLOCK_76M8_CLK_FREQ;
	}
	return get_hfrc_clk_freq();
}

static uint32_t get_syspll_clk_freq(void)
{
	if (sys_read32(CGU_PLL_CLK_SEL) & CGU_PLL_CLK_SEL_SYS) {
		return PLL_CLOCK1_SRC_FREQ;
	}
	return get_osc_ctrl_clk_freq();
}

static uint32_t get_syst_refclk_freq(void)
{
	const uint32_t base_clk = (sys_read32(CGU_PLL_CLK_SEL) & CGU_PLL_CLK_SEL_SYSREF)
					  ? (PLL_CLOCK1_SRC_FREQ / 2)
					  : get_osc_ctrl_clk_freq();
	const uint32_t reg_val = sys_read32(CGU_ESCLK_SEL);
	const uint32_t divider = (reg_val & BIT(27)) ? ((reg_val >> 16) & 0x7ff) : 0;

	return base_clk / (!divider ? 1 : divider);
}

static uint32_t get_syst_aclk_freq(void)
{
#define SYST_ACLK_CTRL 0x820
#define SYST_ACLK_DIV0 0x824

	const uint32_t clkselect = (sys_read32(HOST_BASE_SYS_CTRL + SYST_ACLK_CTRL) >> 8) & 0xFF;

	if (!clkselect) {
		/* Clock gated */
		return 0;
	}

	if (clkselect == 1) {
		return get_syst_refclk_freq();
	}

	/* Read current divider value */
	const uint32_t divider = (sys_read32(HOST_BASE_SYS_CTRL + SYST_ACLK_DIV0) >> 16) & 0xF;

	return (get_syspll_clk_freq() >> divider);
}

uint32_t get_syst_hclk_freq(void)
{
	uint32_t divider = (sys_read32(AON_BUS_CLK_DIV) >> 8) & 0x3;

	if (divider > 1) {
		/* 2 or 3 == divide by 4 */
		divider = 2;
	}
	return (get_syspll_clk_freq() >> divider);
}

static uint32_t get_syst_pclk_freq(void)
{
	uint32_t divider = (sys_read32(AON_BUS_CLK_DIV)) & 0x3;

	if (divider > 1) {
		/* 2 or 3 == divide by 4 */
		divider = 2;
	}
	return (get_syspll_clk_freq() >> divider);
}

static uint32_t get_hfxo_divided_clk_freq(void)
{
	uint32_t divider = (sys_read32(AON_MISC_REG1) >> 13) & 0xF;

	divider = (divider > 7) ? 0 : divider;
	return (ALIF_CLOCK_HFOSC_CLK_FREQ >> divider);
}

static uint32_t get_hfosc_clk_freq(void)
{
	/* Check oscillator clock source for HFOSC_CLK */
	if (sys_read32(CGU_OSC_CTRL) & BIT(4)) {
		/* HFXO selected */
		return get_hfxo_divided_clk_freq();
	}

	return (get_hfrc_clk_freq() / 2);
}

static uint32_t get_he_clock_freq(void)
{
	if (sys_read32(CGU_PLL_CLK_SEL) & CGU_PLL_CLK_SEL_ES1) {
		switch ((sys_read32(CGU_ESCLK_SEL) >> HE_PLL_DIV_POS) & HE_PLL_DIV_MASK) {
		case 0:
			return (PLL_CLOCK2_SRC_FREQ / 2);
		case 1:
			return (PLL_CLOCK1_SRC_FREQ / 2);
		case 2:
			return PLL_CLOCK2_SRC_FREQ;
		case 3:
		default:
			return PLL_CLOCK1_SRC_FREQ;
		}
	}

	const uint32_t es1_osc = (sys_read32(CGU_ESCLK_SEL) >> HE_PLL_DIV_POS) & HE_PLL_DIV_MASK;

	switch (es1_osc) {
	case 0:
		return get_hfrc_clk_freq();
	case 1:
		return get_hfrc_clk_freq() / 2;
	case 2:
		return ALIF_CLOCK_76M8_CLK_FREQ;
	case 3:
	default:
		return get_hfxo_divided_clk_freq();
	};
}

static uint32_t alif_get_input_clock(uint32_t const clock_name)
{
	switch (clock_name) {
	case ALIF_CDC200_PIX_SYST_ACLK:
	case ALIF_OSPI_CLK:
	case ALIF_UTIMER_CLK:
	case ALIF_I3C_CLK:
		return get_syst_aclk_freq();
	case ALIF_CANFD0_HFOSC_CLK:
	case ALIF_CANFD1_HFOSC_CLK:
		return get_hfosc_clk_freq();
	case ALIF_CANFD0_160M_CLK:
	case ALIF_CANFD1_160M_CLK:
		return get_he_clock_freq();
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
	case ALIF_UART0_SYST_PCLK:
	case ALIF_UART1_SYST_PCLK:
	case ALIF_UART2_SYST_PCLK:
	case ALIF_UART3_SYST_PCLK:
	case ALIF_UART4_SYST_PCLK:
	case ALIF_UART5_SYST_PCLK:
		return get_syst_pclk_freq();
	case ALIF_UART0_38M4_CLK:
	case ALIF_UART1_38M4_CLK:
	case ALIF_UART2_38M4_CLK:
	case ALIF_UART3_38M4_CLK:
	case ALIF_UART4_38M4_CLK:
	case ALIF_UART5_38M4_CLK:
		return get_hfosc_clk_freq();
	case ALIF_LPUART_CLK:
	case ALIF_HCI_AHI_CLK:
		return get_he_clock_freq();
	default:
		break;
	}

	return 0;
}

static void alif_get_div_reg_info(uint32_t const clock_name, uint32_t *const mask,
				  uint32_t *const pos)
{
	switch (clock_name) {
	case ALIF_CDC200_PIX_SYST_ACLK:
		*mask = ALIF_CDC200_PIX_CLK_DIV_MASK;
		*pos = ALIF_CDC200_PIX_CLK_DIV_POS;
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

static int32_t alif_get_module_base(const struct device *dev, uint32_t const module,
				    uint32_t *const base)
{
	const struct clock_control_alif_config *config = dev->config;

	switch (module) {
	case ALIF_DUMMY_MODULE:
		break;
	case ALIF_CGU_MODULE:
		*base = config->cgu_base;
		break;
	case ALIF_CLKCTL_PER_MST_MODULE:
		*base = config->clkctl_per_mst_base;
		break;
	case ALIF_CLKCTL_PER_SLV_MODULE:
		*base = config->clkctl_per_slv_base;
		break;
	case ALIF_AON_MODULE:
		*base = config->aon_base;
		break;
	case ALIF_VBAT_MODULE:
		*base = config->vbat_base;
		break;
	case ALIF_M55HE_CFG_MODULE:
		*base = config->m55he_cfg_base;
		break;
	default:
		LOG_ERR("ERROR: Un-supported clock module\n");
		return -EINVAL;
	}

	return 0;
}

static void alif_set_clock_divisor(mem_addr_t const reg_addr, uint32_t const mask,
				   uint32_t const pos, uint32_t const value)
{
	uint32_t reg_value;

	reg_value = sys_read32(reg_addr);
	reg_value &= ~(mask << pos);
	reg_value |= (value << pos);
	sys_write32(reg_value, reg_addr);
}

static uint32_t alif_get_clock_divisor(mem_addr_t const reg_addr, uint32_t const mask,
				       uint32_t const pos)
{
	return ((sys_read32(reg_addr) & (mask << pos)) >> pos);
}

static void alif_set_cgu_clock_gate_on(const struct device *dev, uint32_t const clk_id)
{
	uint32_t reg_addr;
	uint32_t control_bit;

	switch (clk_id) {
	case ALIF_UART0_38M4_CLK:
	case ALIF_UART1_38M4_CLK:
	case ALIF_UART2_38M4_CLK:
	case ALIF_UART3_38M4_CLK:
	case ALIF_UART4_38M4_CLK:
	case ALIF_UART5_38M4_CLK:
		control_bit = 23;
		break;
	default:
		return;
	}

	alif_get_module_base(dev, ALIF_CGU_MODULE, &reg_addr);

	reg_addr += ALIF_CLK_ENA_REG;
	sys_set_bit(reg_addr, control_bit);
}

static int alif_clock_control_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	uint32_t clk_id = (uint32_t)sub_system;
	uint32_t module_base, reg_addr;
	int32_t ret;

	if (!ALIF_CLOCK_CFG_EN_MASK(clk_id)) {
		LOG_WRN("Clock enable not supported\n");
		return 0;
	}

	ret = alif_get_module_base(dev, ALIF_CLOCK_CFG_MODULE(clk_id), &module_base);
	if (ret) {
		return ret;
	}
	reg_addr = module_base + ALIF_CLOCK_CFG_REG(clk_id);

	sys_set_bit(reg_addr, ALIF_CLOCK_CFG_ENABLE(clk_id));

	alif_set_cgu_clock_gate_on(dev, clk_id);
	return 0;
}

static int alif_clock_control_off(const struct device *dev, clock_control_subsys_t sub_system)
{
	uint32_t clk_id = *(uint32_t *)sub_system;
	uint32_t module_base, reg_addr;
	int32_t ret;

	if (!ALIF_CLOCK_CFG_EN_MASK(clk_id)) {
		LOG_WRN("Clock disable not supported\n");
		return 0;
	}

	ret = alif_get_module_base(dev, ALIF_CLOCK_CFG_MODULE(clk_id), &module_base);
	if (ret) {
		return ret;
	}
	reg_addr = module_base + ALIF_CLOCK_CFG_REG(clk_id);

	sys_clear_bit(reg_addr, ALIF_CLOCK_CFG_ENABLE(clk_id));

	return 0;
}

static int alif_clock_control_get_rate(const struct device *dev, clock_control_subsys_t sub_system,
				       uint32_t *rate)
{
	uint32_t clk_id = (uint32_t)sub_system;
	uint32_t clk_freq, reg_addr, module_base;
	uint32_t div_mask, freq_div, div_pos;
	int32_t ret;

	clk_freq = alif_get_input_clock(clk_id);
	if (!clk_freq) {
		return -ENOTSUP;
	}

	ret = alif_get_module_base(dev, ALIF_CLOCK_CFG_MODULE(clk_id), &module_base);
	if (ret) {
		return ret;
	}
	reg_addr = module_base + ALIF_CLOCK_CFG_REG(clk_id);

	alif_get_div_reg_info(clk_id, &div_mask, &div_pos);

	if (div_mask) {
		freq_div = alif_get_clock_divisor((mem_addr_t)reg_addr, div_mask, div_pos);
		*rate = (clk_freq / freq_div);
	} else {
		*rate = clk_freq;
	}

	return 0;
}

static int alif_clock_control_set_rate(const struct device *dev, clock_control_subsys_t sub_system,
				       clock_control_subsys_rate_t rate)
{
	uint32_t clk_id = (uint32_t)sub_system;
	uint32_t clk_freq, curr_freq, reg_addr, module_base;
	uint32_t div_mask, freq_div, div_pos;
	uint32_t frequency = (uint32_t)rate;
	int32_t ret;

	ret = alif_clock_control_get_rate(dev, sub_system, &curr_freq);
	if (ret) {
		return ret;
	}

	/* check if current frequency is already same as desired frequency */
	if (curr_freq == frequency) {
		return 0;
	}

	clk_freq = alif_get_input_clock(clk_id);

	ret = alif_get_module_base(dev, ALIF_CLOCK_CFG_MODULE(clk_id), &module_base);
	if (ret) {
		return ret;
	}
	reg_addr = module_base + ALIF_CLOCK_CFG_REG(clk_id);

	freq_div = (clk_freq / frequency);

	alif_get_div_reg_info(clk_id, &div_mask, &div_pos);

	if (!div_mask) {
		LOG_ERR("ERROR: Frequency setting is not supported\n");
		return -ENOTSUP;
	}
	if (freq_div > div_mask) {
		LOG_ERR("ERROR: Desired frequency setting is not possible\n");
		return -EINVAL;
	}

	alif_set_clock_divisor((mem_addr_t)reg_addr, div_mask, div_pos, freq_div);

	return 0;
}

static enum clock_control_status alif_clock_control_get_status(const struct device *dev,
							       clock_control_subsys_t sub_system)
{
	uint32_t clk_id = (uint32_t)sub_system;
	uint32_t reg_addr, module_base;

	if (!ALIF_CLOCK_CFG_EN_MASK(clk_id)) {
		LOG_ERR("ERROR: Clock status not avilable\n");
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	alif_get_module_base(dev, ALIF_CLOCK_CFG_MODULE(clk_id), &module_base);
	reg_addr = module_base + ALIF_CLOCK_CFG_REG(clk_id);

	if (sys_test_bit(reg_addr, ALIF_CLOCK_CFG_ENABLE(clk_id)) != 0) {
		return CLOCK_CONTROL_STATUS_ON;
	}

	return CLOCK_CONTROL_STATUS_OFF;
}

static inline int alif_clock_control_configure(const struct device *dev,
					       clock_control_subsys_t sub_system, void *data)
{
	uint32_t clk_id = (uint32_t)sub_system;
	uint32_t reg_addr, reg_value, module_base;
	int32_t ret;

	ARG_UNUSED(data);

	if (!ALIF_CLOCK_CFG_CLK_SRC_MASK(clk_id)) {
		return 0;
	}

	ret = alif_get_module_base(dev, ALIF_CLOCK_CFG_MODULE(clk_id), &module_base);
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

static int clockctrl_init(const struct device *dev)
{
	uint32_t cgu_mask = 0;
	uint32_t cgu_module_base;
	int32_t ret;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_38p4m), okay)
	cgu_mask |= BIT(ALIF_CLK_ENA_CLK38P4M_BIT);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_20m), okay)
	cgu_mask |= BIT(ALIF_CLK_ENA_CLK20M_BIT);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_100m), okay)
	cgu_mask |= BIT(ALIF_CLK_ENA_CLK100M_BIT);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_160m), okay)
	cgu_mask |= BIT(ALIF_CLK_ENA_CLK160M_BIT);
#endif

	/* enable cgu clock */
	if (cgu_mask) {
		ret = alif_get_module_base(dev, ALIF_CGU_MODULE, &cgu_module_base);
		if (ret) {
			return -ENODEV;
		}

		sys_set_bits(cgu_module_base + ALIF_CLK_ENA_REG, cgu_mask);
	}

	return 0;
}

static DEVICE_API(clock_control,
		  alif_clock_control_driver_api) = {.on = alif_clock_control_on,
						    .off = alif_clock_control_off,
						    .set_rate = alif_clock_control_set_rate,
						    .get_rate = alif_clock_control_get_rate,
						    .get_status = alif_clock_control_get_status,
						    .configure = alif_clock_control_configure};

static const struct clock_control_alif_config config = {
	.cgu_base = DT_INST_REG_ADDR_BY_NAME(0, cgu),
	.clkctl_per_mst_base = DT_INST_REG_ADDR_BY_NAME(0, clkctl_per_mst),
	.clkctl_per_slv_base = DT_INST_REG_ADDR_BY_NAME(0, clkctl_per_slv),
	.aon_base = DT_INST_REG_ADDR_BY_NAME(0, aon),
	.vbat_base = DT_INST_REG_ADDR_BY_NAME(0, vbat),
	.m55he_cfg_base = DT_INST_REG_ADDR_BY_NAME(0, m55he_cfg),
};

DEVICE_DT_DEFINE(DT_NODELABEL(clockctrl), clockctrl_init, NULL, NULL, &config, PRE_KERNEL_1,
		 CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &alif_clock_control_driver_api);
