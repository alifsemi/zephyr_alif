/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/arch/cpu.h>
#include <soc_common.h>

/**
 * @brief Perform common SoC initialization at boot
 *        for ensemble family.
 *        This will run after soc_early_init_hook.
 *
 * @return 0
 */
static int soc_init(void)
{
	uint32_t uart_clk_mask = 0;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay)
	uart_clk_mask |= BIT(0) | BIT(8);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart1), okay)
	uart_clk_mask |= BIT(1) | BIT(9);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart2), okay)
	uart_clk_mask |= BIT(2) | BIT(10);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart3), okay)
	uart_clk_mask |= BIT(3) | BIT(11);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart4), okay)
	uart_clk_mask |= BIT(4) | BIT(12);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart5), okay)
	uart_clk_mask |= BIT(5) | BIT(13);
#endif
	sys_write32(uart_clk_mask, 0x4902F008);

	/* LPUART settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart), okay)
	if (IS_ENABLED(CONFIG_SERIAL)) {
		/* Enable clock supply for LPUART */
		sys_write32(0x1, AON_RTSS_HE_LPUART_CKEN);
	}
#endif

	if (IS_ENABLED(CONFIG_SPI_DW)) {
		/*
		 * Setting expansion master0 SPI control register values
		 * 0xf at 8-11 bit is setting all 4 SPI instances as master
		 * bit 0-3; ss_in_sel; 0 - from io pad; 1 - from ssi_in_val
		 * bit 8-11; ss_in_val; when ss_in_sel=1, feed ss_in_val to SSI,
		 * each bit controls one SSI instance.
		 * For setting an spi instance as slave, put 0 in the corresponding
		 * bit position of both 8-11 and 0-3 bit fields.
		 * For example if we want to set SPI1 as master and
		 * remaining instances as slave, set the 1st bit for ss_in_sel, which will
		 * make ss_in_val to feed to SSI, and set the corresponding ss_in_val bit.
		 * here for SPI1 as master set the 9th bit. So the value to feed SPI1 as
		 * master and remaining as slave is 0x0202.
		 */
		sys_write32(0x0202, SSI_CTRL_EN);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpspi0), okay)
		/*Clock : LP-SPI*/
		sys_set_bits(HE_PER_CLK_EN, BIT(16));

		/* LP-SPI0 Mode Selection */
		/* To Slave Set Bit : 15  */
		/* To Master Clear Bit : 15 */
		sys_clear_bits(HE_PER_CLK_EN, BIT(15));

		/*LP-SPI0 Flex GPIO*/
		sys_write32(0x1, VBAT_GPIO_CTRL_EN);
#endif
	}

	/* Enable DMA */
#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma2), arm_dma_pl330, okay)
	sys_set_bits(M55HE_CFG_HE_CLK_ENA, BIT(4));
	sys_write32(0x1111, EVTRTRLOCAL_DMA_REQ_CTRL);
	sys_clear_bits(M55HE_CFG_HE_DMA_CTRL, BIT(0));
	sys_write32(0U, M55HE_CFG_HE_DMA_IRQ);
	sys_write32(0U, M55HE_CFG_HE_DMA_PERIPH);
	sys_set_bits(M55HE_CFG_HE_DMA_CTRL, BIT(16));
#endif

	/* RTC Clk Enable */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(rtc0), okay)
	sys_write32(0x1, LPRTC0_CLK_EN);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(rtc1), okay)
	sys_write32(0x1, LPRTC1_CLK_EN);
#endif

/* lptimer settings */
#if DT_HAS_COMPAT_STATUS_OKAY(snps_dw_timers)
	LPTIMER_CONFIG(0);
	LPTIMER_CONFIG(1);
#endif /* DT_HAS_COMPAT_STATUS_OKAY(snps_dw_timers) */

	return 0;
}

SYS_INIT(soc_init, PRE_KERNEL_1, 1);
