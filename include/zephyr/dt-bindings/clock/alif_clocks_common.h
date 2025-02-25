/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ZEPHYR_DT_BINDINGS_CLOCK_ALIF_CLOCKS_COMMON_H_
#define ZEPHYR_INCLUDE_ZEPHYR_DT_BINDINGS_CLOCK_ALIF_CLOCKS_COMMON_H_

#define ALIF_CLOCK_MODULE_MASK         0x7U
#define ALIF_CLOCK_MODULE_SHIFT        0U
#define ALIF_CLOCK_REG_MASK            0xFFU
#define ALIF_CLOCK_REG_SHIFT           3U
#define ALIF_CLOCK_EN_BIT_POS_MASK     0x1FU
#define ALIF_CLOCK_EN_BIT_POS_SHIFT    11U
#define ALIF_CLOCK_EN_MASK_SHIFT       16U
#define ALIF_CLOCK_SRC_MASK            0x3U
#define ALIF_CLOCK_SRC_SHIFT           17U
#define ALIF_CLOCK_SRC_MASK_MASK       0x3U
#define ALIF_CLOCK_SRC_MASK_SHIFT      19U
#define ALIF_CLOCK_SRC_BIT_POS_MASK    0x1FU
#define ALIF_CLOCK_SRC_BIT_POS_SHIFT   21U

/**
 * @brief ALIF clock configuration bit field.
 *
 * - module       [ 0 : 2 ]
 * - reg          [ 3 : 10 ]
 * - en_bit       [ 11 : 15 ]
 * - en_mask      [ 16 ]
 * - src          [ 17 : 18 ]
 * - src_mask     [ 19 : 20 ]
 * - src_bit      [ 21 : 25 ]
 * - reserved     [ 26 : 31 ]
 *
 * @param module   clock module.
 * @param reg      register offset
 * @param en_bit   clock enable bit.
 * @param en_mask  clock enable mask (used to check clock ctrl is required)
 * @param src      clock source value.
 * @param src_mask clock source value mask (check clk src select is required)
 * @param src_bit  clock source bit position
 */
#define ALIF_CLK_CFG(module, reg, en_bit, en_mask, src, src_mask, src_bit)    \
	((((ALIF_##module##_CLKCTL_MODULE) & ALIF_CLOCK_MODULE_MASK) <<       \
			ALIF_CLOCK_MODULE_SHIFT) |                            \
	(((ALIF_##reg##_REG) & ALIF_CLOCK_REG_MASK) << ALIF_CLOCK_REG_SHIFT) | \
	(((en_bit) & ALIF_CLOCK_EN_BIT_POS_MASK) <<                           \
			ALIF_CLOCK_EN_BIT_POS_SHIFT) |                        \
	((en_mask) << ALIF_CLOCK_EN_MASK_SHIFT) |                             \
	(((src) & ALIF_CLOCK_SRC_MASK) << ALIF_CLOCK_SRC_SHIFT) |             \
	(((src_mask) & ALIF_CLOCK_SRC_MASK_MASK) <<                           \
			ALIF_CLOCK_SRC_MASK_SHIFT) |                          \
	(((src_bit) & ALIF_CLOCK_SRC_BIT_POS_MASK) <<                         \
			ALIF_CLOCK_SRC_BIT_POS_SHIFT))

/**
 * Few clocks configurations are not available but used
 * to get clock frequency.
 * Module will be fixed for these macro and dummy value will be
 * passed as parameter which will differ for each macros.
 *
 */
#define ALIF_DUMMY_CLKCTL_MODULE       0x0U
#define ALIF_DUMMY_REG                 0xFFU
/**
 * @brief Macro for dummy clocks (clock configuration absent)
 * @param value  dummy value which differs for all dummy clock macros
 *               (range: 1-31)
 */
#define ALIF_CLK(value)  ALIF_CLK_CFG(DUMMY, DUMMY, value, 0U, 0U, 0U, 0U)

/* Clock modules */
#define ALIF_PER_MST_CLKCTL_MODULE     0x1U
#define ALIF_PER_SLV_CLKCTL_MODULE     0x2U
#define ALIF_AON_CLKCTL_MODULE         0x3U
#define ALIF_VBAT_CLKCTL_MODULE        0x4U
#define ALIF_M55HE_CLKCTL_MODULE       0x5U
#define ALIF_M55HP_CLKCTL_MODULE       0x6U

/* register offset for PER_MST_CLKCTL module */
#define ALIF_CAMERA_PIXCLK_CTRL_REG    0x0U
#define ALIF_CDC200_PIXCLK_CTRL_REG    0x4U
#define ALIF_CSI_PIXCLK_CTRL_REG       0x8U
#define ALIF_PERIPH_CLK_ENA_REG        0xCU
#define ALIF_DPHY_PLL_CTRL0_REG        0x10U
#define ALIF_DPHY_PLL_CTRL1_REG        0x14U
#define ALIF_DPHY_PLL_CTRL2_REG        0x18U
#define ALIF_MIPI_CKEN_REG             0x40U
#define ALIF_ETH_CTRL_REG              0x80U

/* register offset for PER_SLV_CLKCTL module */
#define ALIF_EXPMST0_CTRL_REG          0x0U
#define ALIF_UART_CTRL_REG             0x8U
#define ALIF_CANFD_CTRL_REG            0xCU
#define ALIF_I2S0_CTRL_REG             0x10U
#define ALIF_I2S1_CTRL_REG             0x14U
#define ALIF_I2S2_CTRL_REG             0x18U
#define ALIF_I2S3_CTRL_REG             0x1CU
#define ALIF_I3C_CTRL_REG              0x24U
#define ALIF_ADC_CTRL_REG              0x30U
#define ALIF_DAC_CTRL_REG              0x34U
#define ALIF_CMP_CTRL_REG              0x38U
#define ALIF_GPIO0_CTRL_REG            0x80U
#define ALIF_GPIO1_CTRL_REG            0x84U
#define ALIF_GPIO2_CTRL_REG            0x88U
#define ALIF_GPIO3_CTRL_REG            0x8CU
#define ALIF_GPIO4_CTRL_REG            0x90U
#define ALIF_GPIO5_CTRL_REG            0x94U
#define ALIF_GPIO6_CTRL_REG            0x98U
#define ALIF_GPIO7_CTRL_REG            0x9CU
#define ALIF_GPIO8_CTRL_REG            0xA0U
#define ALIF_GPIO9_CTRL_REG            0xA4U
#define ALIF_GPIO10_CTRL_REG           0xA8U
#define ALIF_GPIO11_CTRL_REG           0xACU
#define ALIF_GPIO12_CTRL_REG           0xB0U
#define ALIF_GPIO13_CTRL_REG           0xB4U
#define ALIF_GPIO14_CTRL_REG           0xB8U

/* register offset for AON_CLKCTL module */
#define ALIF_RTSS_HE_LPUART_CKEN_REG   0x1CU
#define ALIF_SYSTOP_CLK_DIV_REG        0x20U
#define ALIF_MISC_REG1_REG             0x30U

/* register offset for VBAT_CLKCTL module */
#define ALIF_TIMER_CLKSEL_REG          0x4U

/* register offset for M55HE_CFG_CLKCTL module */
#define ALIF_HE_CLK_ENA_REG            0x10U
#define ALIF_HE_I2S_CTRL_REG           0x14U
#define ALIF_HE_CAMERA_PIXCLK_REG      0x20U

/* register offset for M55HP_CFG_LKCTL module */
#define ALIF_HP_CLK_ENA_REG            0x10U

#endif /* ZEPHYR_INCLUDE_ZEPHYR_DT_BINDINGS_CLOCK_ALIF_CLOCKS_COMMON_H_ */
