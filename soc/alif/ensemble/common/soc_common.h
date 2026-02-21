/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_COMMON_H_
#define _SOC_COMMON_H_

/* CGU registers. */
#define CGU_BASE                                0x1A602000
#define CGU_PLL_CLK_SEL                         (CGU_BASE + 0x8)
#define CGU_CLK_ENA                             (CGU_BASE + 0x14)

/* AON registers. */
#define AON_BASE                                 0x1A604000
#define AON_RTSS_HP_CTRL                         (AON_BASE + 0x0)
#define AON_RTSS_HP_RESET                        (AON_BASE + 0x4)
#define AON_RTSS_HE_CTRL                         (AON_BASE + 0x10)
#define AON_RTSS_HE_RESET                        (AON_BASE + 0x14)
#define AON_RTSS_HE_LPPERI_CKEN                  (AON_BASE + 0x1C)

/* VBAT registers. */
#define VBAT_BASE                                0x1A609000
#define VBAT_GPIO_CTRL_EN                        (VBAT_BASE)
#define VBAT_PWR_CTRL                            (VBAT_BASE + 0x8)
#define VBAT_LPRTC0_CLK_EN                       (VBAT_BASE + 0x10)
#define VBAT_LPRTC1_CLK_EN                       (VBAT_BASE + 0x14)

/* Expansion Slave registers. */
#define CLKCTRL_PER_SLV_BASE                     0x4902F000
#define CLKCTRL_PER_SLV_UART_CTRL                (CLKCTRL_PER_SLV_BASE + 0x8)
#define CLKCTRL_PER_SLV_SSI_CTRL                 (CLKCTRL_PER_SLV_BASE + 0x28)
#define CLKCTRL_PER_SLV_ADC_CTRL                 (CLKCTRL_PER_SLV_BASE + 0x30)
#define CLKCTRL_PER_SLV_DAC_CTRL                 (CLKCTRL_PER_SLV_BASE + 0x34)
#define CLKCTRL_PER_SLV_CMP_CTRL                 (CLKCTRL_PER_SLV_BASE + 0x38)
#define CLKCTRL_PER_SLV_OSPI_CTRL                (CLKCTRL_PER_SLV_BASE + 0x3C)
#define CLKCTRL_PER_SLV_GPIO_CTRLn               (CLKCTRL_PER_SLV_BASE + 0x80)

#define EVTRTR0_BASE                             0x49035000
#define EVTRTR0_DMA_CTRL0                        (EVTRTR0_BASE)
#define EVTRTR0_DMA_REQ_CTRL                     (EVTRTR0_BASE + 0x80)
#define EVTRTR0_DMA_ACK_TYPE0                    (EVTRTR0_BASE + 0x90)

#define EVTRTRLOCAL_BASE                         0x400E2000
#define EVTRTRLOCAL_DMA_CTRL0                    (EVTRTRLOCAL_BASE)
#define EVTRTRLOCAL_DMA_REQ_CTRL                 (EVTRTRLOCAL_BASE + 0x80)
#define EVTRTRLOCAL_DMA_ACK_TYPE0                (EVTRTRLOCAL_BASE + 0x90)

/* Expansion Master-0 registers. */
#define CLKCTRL_PER_MST_BASE                     0x4903F000
#define CLKCTRL_PER_MST_CAMERA_PIXCLK_CTRL       (CLKCTRL_PER_MST_BASE)
#define CLKCTRL_PER_MST_CDC200_PIXCLK_CTRL       (CLKCTRL_PER_MST_BASE + 0x4)
#define CLKCTRL_PER_MST_CSI_PIXCLK_CTRL          (CLKCTRL_PER_MST_BASE + 0x8)
#define CLKCTRL_PER_MST_PERIPH_CLK_EN            (CLKCTRL_PER_MST_BASE + 0xC)
#define CLKCTRL_PER_MST_MIPI_CKEN                (CLKCTRL_PER_MST_BASE + 0x40)
#define CLKCTRL_PER_MST_DMA_CTRL                 (CLKCTRL_PER_MST_BASE + 0x70)
#define CLKCTRL_PER_MST_DMA_IRQ                  (CLKCTRL_PER_MST_BASE + 0x74)
#define CLKCTRL_PER_MST_DMA_PERIPH               (CLKCTRL_PER_MST_BASE + 0x78)
#define CLKCTRL_PER_MST_USB_CTRL2                (CLKCTRL_PER_MST_BASE + 0xAC)

/* M55-HE Config registers. */
#define M55HE_CFG_HE_CFG_BASE                    0x43007000
#define M55HE_CFG_HE_DMA_CTRL                    (M55HE_CFG_HE_CFG_BASE)
#define M55HE_CFG_HE_DMA_IRQ                     (M55HE_CFG_HE_CFG_BASE + 0x4)
#define M55HE_CFG_HE_DMA_PERIPH                  (M55HE_CFG_HE_CFG_BASE + 0x8)
#define M55HE_CFG_HE_DMA_SEL                     (M55HE_CFG_HE_CFG_BASE + 0xC)
#define M55HE_CFG_HE_CLK_ENA                     (M55HE_CFG_HE_CFG_BASE + 0x10)
#define M55HE_CFG_HE_CAMERA_PIXCLK               (M55HE_CFG_HE_CFG_BASE + 0x20)

/* M55-HP Config registers. */
#define M55HP_CFG_HP_CFG_BASE                    0x400F0000
#define M55HP_CFG_HP_DMA_CTRL                    (M55HP_CFG_HP_CFG_BASE)
#define M55HP_CFG_HP_DMA_IRQ                     (M55HP_CFG_HP_CFG_BASE + 0x4)
#define M55HP_CFG_HP_DMA_PERIPH                  (M55HP_CFG_HP_CFG_BASE + 0x8)
#define M55HP_CFG_HP_DMA_SEL                     (M55HP_CFG_HP_CFG_BASE + 0xC)
#define M55HP_CFG_HP_CLK_ENA                     (M55HP_CFG_HP_CFG_BASE + 0x10)

/* ANA Register */
#define ANA_BASE                                 0x1A60A000
#define ANA_VBAT_REG1                            (ANA_BASE + 0x38)
#define ANA_VBAT_REG2                            (ANA_BASE + 0x3C)

/* LPGPIO Base address for LPTIMER pin config */
#define LPGPIO_BASE                              0x42002008

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
