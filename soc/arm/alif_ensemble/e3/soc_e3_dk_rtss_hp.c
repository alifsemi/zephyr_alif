/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/gpio/gpio_mmio32.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/cache.h>

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * @return 0
 */
static int ensemble_e3_dk_rtss_hp_init(void)
{
	unsigned int data;

	/* Enable ICACHE */
	sys_cache_instr_enable();

	/* Enable DCACHE */
	sys_cache_data_enable();

	/* Might need to move later.. Just putting this here for now..*/
	/* Enable UART clock and clock selection bits in CFGMST0 */
	sys_write32(0xFFFF, EXPSLV_UART_CTRL);
	/* Set PLL clock of 160 MHz */
	sys_write32(0x10000, EXPMST_PERIPH_CLK_EN);

	/* CGU_UART_CLK source to PLL */
	data = sys_read32(CGU_PLL_CLK_SEL);
	data |= 1U << 8;
	sys_write32(data, CGU_PLL_CLK_SEL);

	/* Enable CGU_UART_CLK */
	data = sys_read32(CGU_CLK_ENA);
	data |= 1U << 17;
	sys_write32(data, CGU_CLK_ENA);

	/* Switch HOSTUARTCLK Source to CGU_UART_CLK */
	sys_write32(2, 0x1A010850);

	/* Setting expansion master0 control register value */
	/* for enabling clock */
	data |= 0xc0000000;
	sys_write32(data, EXPSLV_EXPMST0_CTRL);

	/* SPI: Enable Master Mode and SS Val */
#if  UTIL_AND(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(spi0)), \
	!DT_PROP(DT_NODELABEL(spi0), serial_target))
	sys_set_bit(EXPSLV_SSI_CTRL, 0);
	sys_set_bit(EXPSLV_SSI_CTRL, 8);
#endif

#if  UTIL_AND(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(spi1)), \
	!DT_PROP(DT_NODELABEL(spi1), serial_target))
	sys_set_bit(EXPSLV_SSI_CTRL, 1);
	sys_set_bit(EXPSLV_SSI_CTRL, 9);
#endif

#if  UTIL_AND(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(spi2)), \
	!DT_PROP(DT_NODELABEL(spi2), serial_target))
	sys_set_bit(EXPSLV_SSI_CTRL, 2);
	sys_set_bit(EXPSLV_SSI_CTRL, 10);
#endif

#if  UTIL_AND(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(spi3)), \
	!DT_PROP(DT_NODELABEL(spi3), serial_target))
	sys_set_bit(EXPSLV_SSI_CTRL, 3);
	sys_set_bit(EXPSLV_SSI_CTRL, 11);
#endif

	/* enable pdm in expansion master */
	sys_set_bits(EXPSLV_EXPMST0_CTRL, BIT(8));

	/* lptimer settings */
#if DT_HAS_COMPAT_STATUS_OKAY(snps_dw_timers)
	/* LPTIMER 0 settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timer0), okay)
	if (IS_ENABLED(CONFIG_LPTIMER0_OUTPUT_TOGGLE) ||
			(CONFIG_LPTIMER0_EXT_CLK_FREQ > 0U)) {
		/*
		 * enable of LPTIMER0 pin by config lpgpio
		 * pin 0 as Hardware control
		 */
		sys_set_bit(LPGPIO_BASE, 0);
	}
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(timer0), okay) */
	/* LPTIMER 1 settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timer1), okay)
	if (IS_ENABLED(CONFIG_LPTIMER1_OUTPUT_TOGGLE) ||
			(CONFIG_LPTIMER1_EXT_CLK_FREQ > 0U)) {
		/*
		 * enable of LPTIMER1 pin by config lpgpio
		 * pin 1 as Hardware control
		 */
		sys_set_bit(LPGPIO_BASE, 1);
	}
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(timer1), okay) */
	/* LPTIMER 2 settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timer2), okay)
	if (IS_ENABLED(CONFIG_LPTIMER2_OUTPUT_TOGGLE) ||
			(CONFIG_LPTIMER2_EXT_CLK_FREQ > 0U)) {
		/*
		 * enable of LPTIMER2 pin by config lpgpio
		 * pin 2 as Hardware control
		 */
		sys_set_bit(LPGPIO_BASE, 2);
	}
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(timer2), okay) */
	/* LPTIMER 3 settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timer3), okay)
	if (IS_ENABLED(CONFIG_LPTIMER3_OUTPUT_TOGGLE) ||
			(CONFIG_LPTIMER3_EXT_CLK_FREQ > 0U)) {
		/*
		 * enable of LPTIMER3 pin by config lpgpio
		 * pin 3 as Hardware control
		 */
		sys_set_bit(LPGPIO_BASE, 3);
	}
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(timer3), okay) */
#endif /* DT_HAS_COMPAT_STATUS_OKAY(snps_dw_timers) */

	if (IS_ENABLED(CONFIG_DISPLAY)) {
		/* Enable CDC200 peripheral clock. */
		sys_set_bits(EXPMST_PERIPH_CLK_EN, BIT(1));

		if (IS_ENABLED(CONFIG_MIPI_DSI)) {
			/*
			 * CDC200 clock enablement for serial display.
			 *  Pixclk control register:
			 *	clk_divisor[24:16] - 0x10 (16)
			 * Pixel clock observed = (400 / 16) MHz = 25 MHz
			 * Serial display has recommended FPS of 54-66 FPS,
			 * and tested at 60 FPS.
			 */
			sys_write32(0x100001, EXPMST_CDC200_PIXCLK_CTRL);
		} else {
			/*
			 * CDC200 clock Pixel clock for parallel display.
			 *  Pixclk control register:
			 *	clk_divisor[24:16] - 0x90 (144)
			 * Pixel clock observed = (400 / 144) MHz = 2.77 MHz
			 * Parallel display tested at 5 FPS.
			 */
			sys_write32(0x900001, EXPMST_CDC200_PIXCLK_CTRL);
		}
	}
	if (IS_ENABLED(CONFIG_VIDEO)) {
		/* Enable CAM controller peripheral clock. */
		sys_set_bits(EXPMST_PERIPH_CLK_EN, BIT(0));

		/* CPI Pixel clock - Generate XVCLK. Used by ARX3A0 */
		sys_write32(0x140001, EXPMST_CAMERA_PIXCLK_CTRL);
	}
	if (IS_ENABLED(CONFIG_MIPI_DSI)) {
		/* Enable DSI controller peripheral clock. */
		sys_set_bits(EXPMST_PERIPH_CLK_EN, BIT(28));

		/* Enable TX-DPHY and PLL ref clock.*/
		sys_set_bits(EXPMST_MIPI_CKEN, BIT(0) | BIT(8));

		/* Enable TX-DPHY and D-PLL Power and Disable Isolation.*/
		sys_clear_bits(VBAT_PWR_CTRL, BIT(0) | BIT(1) | BIT(8) | BIT(9) |
				BIT(12));

		/* Enable HFOSC (38.4 MHz) and CFG (100 MHz) clock.*/
		sys_set_bits(CGU_CLK_ENA, BIT(21) | BIT(23));
	}
	if (IS_ENABLED(CONFIG_VIDEO_MIPI_CSI2_DW)) {
		/* Enable CSI2 controller peripheral clock. */
		sys_set_bits(EXPMST_PERIPH_CLK_EN, BIT(24));

		/* CSI Pixel clock. */
		sys_write32(0x20001, EXPMST_CSI_PIXCLK_CTRL);

		/* Enable RX-DPHY.*/
		sys_set_bits(EXPMST_MIPI_CKEN, BIT(4));

		/* Enable RX-DPHY Power and Disable Isolation.*/
		sys_clear_bits(VBAT_PWR_CTRL, BIT(4) | BIT(5));

		/* Enable CFG clock - 100 MHz used by RX-DPHY*/
		sys_set_bits(CGU_CLK_ENA, BIT(21));
	}

	/* Enable LPRTC clock via VBAT registers */
	sys_set_bits(VBAT_RTC_CLK_EN, BIT(0));

	/* Enable DMA */
#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma1), arm_dma_pl330, okay) /* dma1 */
	sys_set_bits(M55HP_CFG_HP_CLK_ENA, BIT(4));
	sys_write32(0x1111, EVTRTRLOCAL_DMA_REQ_CTRL);
	sys_clear_bits(M55HP_CFG_HP_DMA_CTRL, BIT(0));
	sys_write32(0U, M55HP_CFG_HP_DMA_IRQ);
	sys_write32(0U, M55HP_CFG_HP_DMA_PERIPH);
	sys_set_bits(M55HP_CFG_HP_DMA_CTRL, BIT(16));
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma0), arm_dma_pl330, okay) /* dma0 */
	sys_set_bits(EXPMST_PERIPH_CLK_EN, BIT(4));
	sys_write32(0x1111, EVTRTR0_DMA_REQ_CTRL);
	sys_clear_bits(EXPMST_DMA_CTRL, BIT(0));
	sys_write32(0U, EXPMST_DMA_IRQ);
	sys_write32(0U, EXPMST_DMA_PERIPH);
	sys_set_bits(EXPMST_DMA_CTRL, BIT(16));
#endif

	/* CAN settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay)
	/* Enable HFOSC and 160MHz clock */
	data = sys_read32(CGU_CLK_ENA);
	data |= ((1 << 20) | (1 << 23));
	sys_write32(data, CGU_CLK_ENA);
#endif

	/* I3C settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c0), okay)
	/*I3C Flex GPIO */
	sys_write32(0x1, VBAT_BASE);
#endif

	return 0;
}

SYS_INIT(ensemble_e3_dk_rtss_hp_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
