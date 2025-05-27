/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_COMMON_H_
#define _SOC_COMMON_H_

/* CGU registers. */
#define CGU_BASE        		0x1A602000
#define CGU_PLL_CLK_SEL 		(CGU_BASE + 0x8)
#define CGU_CLK_ENA     		(CGU_BASE + 0x14)

/* AON registers. */
#define AON_BASE                	0x1A604000
#define AON_RTSS_HP_CTRL        	(AON_BASE + 0x0)
#define AON_RTSS_HP_RESET       	(AON_BASE + 0x4)
#define AON_RTSS_HE_CTRL        	(AON_BASE + 0x10)
#define AON_RTSS_HE_RESET       	(AON_BASE + 0x14)
#define AON_RTSS_HE_LPUART_CKEN 	(AON_BASE + 0x1C)

/* VBAT registers. */
#define VBAT_BASE       		0x1A609000
#define VBAT_PWR_CTRL   		(VBAT_BASE + 0x8)
#define VBAT_RTC_CLK_EN 		(VBAT_BASE + 0x10)

/* Expansion Slave registers. */
#define EXPSLV_BASE         		0x4902F000
#define EXPSLV_EXPMST0_CTRL 		(EXPSLV_BASE)
#define EXPSLV_UART_CTRL    		(EXPSLV_BASE + 0x8)
#define EXPSLV_SSI_CTRL     		(EXPSLV_BASE + 0x28)
#define EXPSLV_ADC_CTRL     		(EXPSLV_BASE + 0x30)
#define EXPSLV_CMP_CTRL     		(EXPSLV_BASE + 0x38)

#define EVTRTR0_BASE          		0x49035000
#define EVTRTR0_DMA_CTRL0     		(EVTRTR0_BASE)
#define EVTRTR0_DMA_REQ_CTRL  		(EVTRTR0_BASE + 0x80)
#define EVTRTR0_DMA_ACK_TYPE0 		(EVTRTR0_BASE + 0x90)

#define EVTRTRLOCAL_BASE          	0x400E2000
#define EVTRTRLOCAL_DMA_CTRL0     	(EVTRTRLOCAL_BASE)
#define EVTRTRLOCAL_DMA_REQ_CTRL  	(EVTRTRLOCAL_BASE + 0x80)
#define EVTRTRLOCAL_DMA_ACK_TYPE0 	(EVTRTRLOCAL_BASE + 0x90)

/* Expansion Master-0 registers. */
#define EXPMST_BASE                  0x4903F000
#define EXPMST_CAMERA_PIXCLK_CTRL    (EXPMST_BASE + 0x00)
#define EXPMST_CDC200_PIXCLK_CTRL    (EXPMST_BASE + 0x04)
#define EXPMST_CSI_PIXCLK_CTRL       (EXPMST_BASE + 0x08)
#define EXPMST_PERIPH_CLK_EN         (EXPMST_BASE + 0x0C)
#define EXPMST_DPHY_PLL_CTRL0        (EXPMST_BASE + 0x10)
#define EXPMST_DPHY_PLL_CTRL1        (EXPMST_BASE + 0x14)
#define EXPMST_DPHY_PLL_CTRL2        (EXPMST_BASE + 0x18)
#define EXPMST_DPHY_PLL_STAT0        (EXPMST_BASE + 0x20)
#define EXPMST_DPHY_PLL_STAT1        (EXPMST_BASE + 0x24)
#define EXPMST_TX_DPHY_CTRL0         (EXPMST_BASE + 0x30)
#define EXPMST_TX_DPHY_CTRL1         (EXPMST_BASE + 0x34)
#define EXPMST_RX_DPHY_CTRL0         (EXPMST_BASE + 0x38)
#define EXPMST_RX_DPHY_CTRL1         (EXPMST_BASE + 0x3C)
#define EXPMST_MIPI_CKEN             (EXPMST_BASE + 0x40)
#define EXPMST_DSI_CTRL              (EXPMST_BASE + 0x44)
#define EXPMST_DMA_CTRL              (EXPMST_BASE + 0x70)
#define EXPMST_DMA_IRQ               (EXPMST_BASE + 0x74)
#define EXPMST_DMA_PERIPH            (EXPMST_BASE + 0x78)
#define EXPMST_DMA_GLITCH_FLT        (EXPMST_BASE + 0x7C)
#define EXPMST_ETH_CTRL0             (EXPMST_BASE + 0x80)
#define EXPMST_ETH_STAT0             (EXPMST_BASE + 0x84)
#define EXPMST_ETH_PTP_TMST0         (EXPMST_BASE + 0x88)
#define EXPMST_ETH_PTP_TMST1         (EXPMST_BASE + 0x8C)
#define EXPMST_SDC_CTRL0             (EXPMST_BASE + 0x90)
#define EXPMST_SDC_STAT0             (EXPMST_BASE + 0x94)
#define EXPMST_SDC_STAT1             (EXPMST_BASE + 0x98)
#define EXPMST_USB_GPIO0             (EXPMST_BASE + 0xA0)
#define EXPMST_USB_STAT0             (EXPMST_BASE + 0xA4)
#define EXPMST_USB_CTRL1             (EXPMST_BASE + 0xA8)
#define EXPMST_USB_CTRL2             (EXPMST_BASE + 0xAC)

/* M55-HE Config registers. */
#define M55HE_CFG_HE_CFG_BASE      	0x43007000
#define M55HE_CFG_HE_DMA_CTRL      	(M55HE_CFG_HE_CFG_BASE)
#define M55HE_CFG_HE_DMA_IRQ       	(M55HE_CFG_HE_CFG_BASE + 0x4)
#define M55HE_CFG_HE_DMA_PERIPH    	(M55HE_CFG_HE_CFG_BASE + 0x8)
#define M55HE_CFG_HE_DMA_SEL       	(M55HE_CFG_HE_CFG_BASE + 0xC)
#define M55HE_CFG_HE_CLK_ENA       	(M55HE_CFG_HE_CFG_BASE + 0x10)
#define M55HE_CFG_HE_CAMERA_PIXCLK 	(M55HE_CFG_HE_CFG_BASE + 0x20)

/* M55-HP Config registers. */
#define M55HP_CFG_HP_CFG_BASE   	0x400F0000
#define M55HP_CFG_HP_DMA_CTRL   	(M55HP_CFG_HP_CFG_BASE)
#define M55HP_CFG_HP_DMA_IRQ    	(M55HP_CFG_HP_CFG_BASE + 0x4)
#define M55HP_CFG_HP_DMA_PERIPH 	(M55HP_CFG_HP_CFG_BASE + 0x8)
#define M55HP_CFG_HP_DMA_SEL    	(M55HP_CFG_HP_CFG_BASE + 0xC)
#define M55HP_CFG_HP_CLK_ENA    	(M55HP_CFG_HP_CFG_BASE + 0x10)

/* ANA Register */
#define ANA_BASE      			0x1A60A000
#define ANA_VBAT_REG1 			(ANA_BASE + 0x38)
#define ANA_VBAT_REG2 			(ANA_BASE + 0x3C)

/* LPGPIO Base address for LPTIMER pin config */
#define LPGPIO_BASE 			0x42002008

#endif /* _SOC_COMMON_H_ */
