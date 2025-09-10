/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_COMMON_H_
#define _SOC_COMMON_H_

/* Host Base System Control Registers */
#define HOST_BASE_SYS_CTRL		0x1A010000
#define HOST_BSYS_PWR_REQ		(HOST_BASE_SYS_CTRL + 0x400)
#define HOST_BSYS_PWR_ST		(HOST_BASE_SYS_CTRL + 0x404)

/* AON registers. */
#define AON_BASE                        0x1A604000UL
#define AON_RTSS_HE_CTRL                (AON_BASE + 0x10)
#define AON_RTSS_HE_RESET               (AON_BASE + 0x14)
#define AON_RTSS_HE_LPUART_CKEN         (AON_BASE + 0x1C)

/* VBAT Modules */
#define VBAT_BASE                       0x1A609000UL
#define VBAT_PWR_CTRL                   (VBAT_BASE + 0x8)
#define LPRTC0_CLK_EN                   (VBAT_BASE + 0x10)
#define LPRTC1_CLK_EN                   (VBAT_BASE + 0x14)

/* Peripheral CLKCTRL */
#define CLKCTRL_PER_SLV                 0X4902F000UL
#define UART_CLK_EN                     (CLKCTRL_PER_SLV + 0x8)

/* CGU Module */
#define CGU_BASE			0x1A602000
#define CGU_CLK_ENA			(CGU_BASE + 0x14)

/* ANA Register */
#define ANA_BASE			0x1A60A000
#define ANA_VBAT_REG1                   (ANA_BASE + 0x38)
#define ANA_VBAT_REG2			(ANA_BASE + 0x3C)

/* Expansion Slave registers. */
#define EXPSLV_BASE			0x4902F000
#define EXPSLV_UART_CTRL		(EXPSLV_BASE + 0x8)
#define EXPSLV_ADC_CTRL			(EXPSLV_BASE + 0x30)
#define EXPSLV_DAC_CTRL                 (EXPSLV_BASE + 0x34)
#define EXPSLV_CMP_CTRL			(EXPSLV_BASE + 0x38)
#define EXPSLV_OSPI_CTRL		(EXPSLV_BASE + 0x3C)

/* Peripheral CLKCTRL */
#define CLKCTRL_PER_SLV			0X4902F000UL
#define SSI_CTRL_EN			(CLKCTRL_PER_SLV + 0x28)

/* Expansion Master-0 registers. */
#define EXPMST_BASE			0x4903F000
#define EXPMST_CDC200_PIXCLK_CTRL	(EXPMST_BASE + 0x4)
#define EXPMST_PERIPH_CLK_EN		(EXPMST_BASE + 0xC)

/* M55-HE Config Registers */
#define M55HE_CFG_BASE			0x43007000UL
#define HE_PER_CLK_EN			(M55HE_CFG_BASE + 0x10)
#define HE_CAMERA_PIXCLK		(M55HE_CFG_BASE + 0x20)

/* VBAT Modules */
#define VBAT_REG_BASE			0x1A609000UL
#define VBAT_GPIO_CTRL_EN		(VBAT_REG_BASE + 0x0)

/* LPGPIO Base address for LPTIMER pin config */
#define LPGPIO_BASE			0x42002008UL

/* Event Router registers. */
#define EVTRTRLOCAL_BASE		0x400E2000
#define EVTRTRLOCAL_DMA_CTRL0		(EVTRTRLOCAL_BASE)
#define EVTRTRLOCAL_DMA_REQ_CTRL	(EVTRTRLOCAL_BASE + 0x80)
#define EVTRTRLOCAL_DMA_ACK_TYPE0	(EVTRTRLOCAL_BASE + 0x90)

/* M55-HE Config registers. */
#define M55HE_CFG_HE_CFG_BASE		0x43007000
#define M55HE_CFG_HE_DMA_CTRL		(M55HE_CFG_HE_CFG_BASE)
#define M55HE_CFG_HE_DMA_IRQ		(M55HE_CFG_HE_CFG_BASE + 0x4)
#define M55HE_CFG_HE_DMA_PERIPH		(M55HE_CFG_HE_CFG_BASE + 0x8)
#define M55HE_CFG_HE_DMA_SEL		(M55HE_CFG_HE_CFG_BASE + 0xC)
#define M55HE_CFG_HE_CLK_ENA		(M55HE_CFG_HE_CFG_BASE + 0x10)
#define M55HE_CFG_HE_CAMERA_PIXCLK	(M55HE_CFG_HE_CFG_BASE + 0x20)

/* lptimer helper macro */
#define LPTIMER_CONFIG(idx)						\
	/* Check if timer node is enabled in DT */			\
	IF_ENABLED(DT_NODE_HAS_STATUS(DT_NODELABEL(timer##idx), okay), (\
		if (IS_ENABLED(CONFIG_LPTIMER##idx##_OUTPUT_TOGGLE) ||	\
			(CONFIG_LPTIMER##idx##_EXT_CLK_FREQ > 0U)) {	\
			/* enable LPTIMER##idx pin via LPGPIO */	\
			sys_set_bit(LPGPIO_BASE, idx);			\
		}							\
	))
#endif /* _SOC_COMMON_H_ */
