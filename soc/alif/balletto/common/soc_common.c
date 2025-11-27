/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <se_service.h>
#include <soc_common.h>
#include <zephyr/init.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/reboot.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);


/**
 * @brief Perform common SoC initialization at boot
 *        for ensemble family.
 *        This will run after soc_early_init_hook.
 *
 * @return 0
 */
static int soc_init(void)
{
	uint32_t uart_clk_mask = sys_read32(EXPSLV_UART_CTRL);

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
	sys_write32(uart_clk_mask, EXPSLV_UART_CTRL);

	/* LPUART settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart), okay)
	if (IS_ENABLED(CONFIG_SERIAL)) {
		/* Enable clock supply for LPUART */
		sys_write32(0x1, AON_RTSS_HE_LPUART_CKEN);
	}
#endif

	/* SPI: Enable Master Mode and SS Value */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi0), okay) && !DT_PROP(DT_NODELABEL(spi0), serial_target)
	sys_set_bits(EXPSLV_SSI_CTRL, BIT(0) | BIT(8));
#endif /* spi0 */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi1), okay) && !DT_PROP(DT_NODELABEL(spi1), serial_target)
	sys_set_bits(EXPSLV_SSI_CTRL, BIT(1) | BIT(9));
#endif /* spi1 */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi2), okay) && !DT_PROP(DT_NODELABEL(spi2), serial_target)
	sys_set_bits(EXPSLV_SSI_CTRL, BIT(2) | BIT(10));
#endif /* spi2 */

	/* LPSPI: lpspi0 or lpspi1 */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpspi0), okay) || DT_NODE_HAS_STATUS(DT_NODELABEL(lpspi1), okay)
	/* Enable LPSPI CLK */
	sys_set_bit(M55HE_CFG_HE_CLK_ENA, 16);

	/* LPSPI Flex GPIO */
	sys_write32(0x1, VBAT_GPIO_CTRL_EN);

	/* LPSPI Master/Slave Mode Selection */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpspi0), okay) /* LPSPI0 Master only. */
	sys_clear_bit(M55HE_CFG_HE_CLK_ENA, 15);
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(lpspi1), okay) /* LPSPI1 Slave only. */
	sys_set_bit(M55HE_CFG_HE_CLK_ENA, 15);
#endif /* LPSPI Master/Slave Mode Selection */

#endif /* lpspi0 or lpspi1 */

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

	/*Clock : OSPI */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(ospi0), okay)
	if (IS_ENABLED(CONFIG_SOC_SERIES_B1)) {
		sys_write32(0x1, EXPSLV_OSPI_CTRL);
	}
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(sdhc), okay)
	/* Enable CFG (100 MHz and 20MHz) clock.*/
	sys_set_bits(CGU_CLK_ENA, BIT(21) | BIT(22));

	/* Peripheral clock enable */
	sys_set_bits(EXPMST_PERIPH_CLK_EN, BIT(16));
#endif

	return 0;
}

#ifdef CONFIG_REBOOT
void sys_arch_reboot(int type)
{
	switch (type) {
	case SYS_REBOOT_WARM:
		/* Use Cold boot until NVIC reset is fully working */
		/* se_service_boot_reset_cpu(EXTSYS_1); */
		se_service_boot_reset_soc();
		break;
	case SYS_REBOOT_COLD:
		se_service_boot_reset_soc();
		break;

	default:
		/* Do nothing */
		break;
	}
}
#endif

SYS_INIT(soc_init, PRE_KERNEL_1, 1);
