/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/arch/cpu.h>
#include <soc_common.h>
#include <zephyr/pm/policy.h>

#if CONFIG_ENSEMBLE_GEN2 /* ENSEMBLE_GEN2 SoC */
/* GPIO: enable debounce clock / divisor. */
#define GPIO_DEBOUNCE_CK_DIV_MASK   0x3FF
#define GPIO_DEBOUNCE_CK_DIV2       BIT(0)
#define GPIO_DEBOUNCE_CK_ENABLE     BIT(12)

/* GPIO: enable debounce clock / divisor for gpio0..gpio14 */
#define EXPSLV_GPIO_DEBOUNCE_CK_EN(n) \
	IF_ENABLED(DT_NODE_HAS_STATUS(DT_NODELABEL(gpio##n), okay), ( \
		sys_clear_bits(EXPSLV_GPIO_CTRLn + (0x4 * n), GPIO_DEBOUNCE_CK_DIV_MASK); \
		sys_set_bits(EXPSLV_GPIO_CTRLn + (0x4 * n), \
				GPIO_DEBOUNCE_CK_ENABLE | GPIO_DEBOUNCE_CK_DIV2); \
	))

/* GPIO: enable debounce clock for gpio16 and gpio17 */
#define AON_GPIO_DEBOUNCE_CK_EN(n, bit) \
	IF_ENABLED(DT_NODE_HAS_STATUS(DT_NODELABEL(gpio##n), okay), \
		(sys_set_bits(AON_RTSS_HE_LPPERI_CKEN, BIT(bit));))

/* EXPSLV gpio0..gpio14 */
#define EXPSLV_ALL_GPIO_DEBOUNCE_CK_EN() \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(0);  \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(1);  \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(2);  \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(3);  \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(4);  \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(5);  \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(6);  \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(7);  \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(8);  \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(9);  \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(10); \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(11); \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(12); \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(13); \
	EXPSLV_GPIO_DEBOUNCE_CK_EN(14)

/* AON gpio16 and gpio17. */
#define AON_ALL_GPIO_DEBOUNCE_CK_EN() \
	AON_GPIO_DEBOUNCE_CK_EN(16, 8); \
	AON_GPIO_DEBOUNCE_CK_EN(17, 9)

static inline void enable_gpio_debounce_clock(void)
{
	/* EXPSLV gpio0..gpio14 */
	EXPSLV_ALL_GPIO_DEBOUNCE_CK_EN();

	/* LPGPIO(gpio15) debounce clock is always enabled. */

	/* AON gpio16 and gpio17. */
	AON_ALL_GPIO_DEBOUNCE_CK_EN();
}
#endif /* CONFIG_ENSEMBLE_GEN2 */

/*
 * Lock deeper power states during early boot to prevent premature sleep
 *
 * During driver initialization, some drivers may trigger idle conditions
 * that could cause the PM subsystem to enter deep sleep states before the
 * system is ready. This locks all deeper states, allowing only
 * PM_STATE_RUNTIME_IDLE during boot. Locks are released at APPLICATION phase.
 */
static int soc_pm_lock_boot_states(void)
{
	/* Lock all deeper power states, allowing only RUNTIME_IDLE during boot */
	for (enum pm_state state = PM_STATE_SUSPEND_TO_IDLE; state < PM_STATE_COUNT; state++) {
		pm_policy_state_lock_get(state, PM_ALL_SUBSTATES);
	}

	return 0;
}
SYS_INIT(soc_pm_lock_boot_states, PRE_KERNEL_2, 0);

/*
 * Release deeper power state locks after kernel initialization
 *
 * Once kernel initialization is complete (APPLICATION phase), release the
 * boot-time locks to allow normal power management operation.
 */
static int soc_pm_unlock_boot_states(void)
{
	/* Unlock all deeper power states */
	for (enum pm_state state = PM_STATE_SUSPEND_TO_IDLE; state < PM_STATE_COUNT; state++) {
		pm_policy_state_lock_put(state, PM_ALL_SUBSTATES);
	}

	return 0;
}
SYS_INIT(soc_pm_unlock_boot_states, APPLICATION, 0);

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
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart6), okay)
	uart_clk_mask |= BIT(6) | BIT(14);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart7), okay)
	uart_clk_mask |= BIT(7) | BIT(15);
#endif

	sys_write32(uart_clk_mask, EXPSLV_UART_CTRL);

	/* LPUART settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart), okay)
	if (IS_ENABLED(CONFIG_SERIAL)) {
		/* Enable clock supply for LPUART */
		sys_write32(0x1, AON_RTSS_HE_LPPERI_CKEN);
	}
#endif

#if IS_ENABLED(CONFIG_SPI_DW) /* SPI */
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

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi3), okay) && !DT_PROP(DT_NODELABEL(spi3), serial_target)
	sys_set_bits(EXPSLV_SSI_CTRL, BIT(3) | BIT(11));
#endif /* spi3 */

	/* LP-SPI */
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(lpspi0), okay) || DT_NODE_HAS_STATUS(DT_NODELABEL(spi4), okay))
		/*Clock : LP-SPI*/
		sys_set_bits(M55HE_CFG_HE_CLK_ENA, BIT(16));

		/* LP-SPI0 Mode Selection */
		/* To Slave Set Bit : 15  */
		/* To Master Clear Bit : 15 */
		sys_clear_bits(M55HE_CFG_HE_CLK_ENA, BIT(15));

		/*LP-SPI0 Flex GPIO*/
		sys_write32(0x1, VBAT_GPIO_CTRL_EN);
#endif /* (DT_NODE_HAS_STATUS(DT_NODELABEL(lpspi0), okay) || DT_NODE_HAS_STATUS(DT_NODELABEL(spi4),
	* okay))
	*/

#endif /* defined(CONFIG_SPI_DW) */

	/* Enable DMA */
#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma0), arm_dma_pl330, okay) /* dma0 */
	sys_set_bits(EXPMST_PERIPH_CLK_EN, BIT(4));
	sys_write32(0x1111, EVTRTR0_DMA_REQ_CTRL);
	sys_clear_bits(EXPMST_DMA_CTRL, BIT(0));
	sys_write32(0U, EXPMST_DMA_IRQ);
	sys_write32(0U, EXPMST_DMA_PERIPH);
	sys_set_bits(EXPMST_DMA_CTRL, BIT(16));
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma1), arm_dma_pl330, okay) /* dma1 */
	sys_set_bits(M55HP_CFG_HP_CLK_ENA, BIT(4));
	sys_write32(0x1111, EVTRTRLOCAL_DMA_REQ_CTRL);
	sys_clear_bits(M55HP_CFG_HP_DMA_CTRL, BIT(0));
	sys_write32(0U, M55HP_CFG_HP_DMA_IRQ);
	sys_write32(0U, M55HP_CFG_HP_DMA_PERIPH);
	sys_set_bits(M55HP_CFG_HP_DMA_CTRL, BIT(16));
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma2), arm_dma_pl330, okay) /* dma2 */
	sys_set_bits(M55HE_CFG_HE_CLK_ENA, BIT(4));
	sys_write32(0x1111, EVTRTRLOCAL_DMA_REQ_CTRL);
	sys_clear_bits(M55HE_CFG_HE_DMA_CTRL, BIT(0));
	sys_write32(0U, M55HE_CFG_HE_DMA_IRQ);
	sys_write32(0U, M55HE_CFG_HE_DMA_PERIPH);
	sys_set_bits(M55HE_CFG_HE_DMA_CTRL, BIT(16));
#endif

	/* Enable LPRTC Clock via VBAT registers */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(rtc0), okay)
	sys_write32(0x1, VBAT_LPRTC0_CLK_EN);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(rtc1), okay)
	sys_write32(0x1, VBAT_LPRTC1_CLK_EN);
#endif

	/* lptimer settings */
#if DT_HAS_COMPAT_STATUS_OKAY(snps_dw_timers)
	LPTIMER_CONFIG(0);
	LPTIMER_CONFIG(1);
	LPTIMER_CONFIG(2);
	LPTIMER_CONFIG(3);
#endif /* DT_HAS_COMPAT_STATUS_OKAY(snps_dw_timers) */

	/*Enable Clock : 76P8M */
#if DT_HAS_COMPAT_STATUS_OKAY(snps_designware_i2s)
	if (IS_ENABLED(CONFIG_ENSEMBLE_GEN2)) {
		sys_set_bits(CGU_CLK_ENA, BIT(24));
	}
#endif


	/*Clock : OSPI */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(ospi0), okay)
	if (IS_ENABLED(CONFIG_ENSEMBLE_GEN2) ||
		IS_ENABLED(CONFIG_SOC_SERIES_E1C)) {
		sys_write32(0x1, EXPSLV_OSPI_CTRL);
	}
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(ospi1), okay)
	if (IS_ENABLED(CONFIG_ENSEMBLE_GEN2)) {
		sys_write32(0x2, EXPSLV_OSPI_CTRL);
	}
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(sdhc), okay)
	/* Enable CFG (100 MHz and 20MHz) clock.*/
#if defined(CONFIG_ENSEMBLE_GEN2)
	sys_set_bits(CGU_CLK_ENA, BIT(7) | BIT(9) | BIT(22));
#else
	sys_set_bits(CGU_CLK_ENA, BIT(21) | BIT(22));
#endif

	/* Peripheral clock enable */
	sys_set_bits(EXPMST_PERIPH_CLK_EN, BIT(16));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usb), okay)
	/* Enable phy pwr mask and Enable the phy Isolation. */
	sys_clear_bits(VBAT_PWR_CTRL, BIT(16) | BIT(17));

	/* USB power on reset clear */
	sys_clear_bits(EXPMST_USB_CTRL2, BIT(8));
#endif

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(pdm), okay) || DT_NODE_HAS_STATUS(DT_NODELABEL(lppdm), okay))
	if (IS_ENABLED(CONFIG_ENSEMBLE_GEN2)) {
		/* enable the HFOSCx2 clock */
		sys_set_bits(CGU_CLK_ENA, BIT(24));
	}
#endif
	/* I3C settings */
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(i3c0), okay) || DT_NODE_HAS_STATUS(DT_NODELABEL(lpi3c0), okay))
	/* I3C Flex GPIO */
	sys_write32(0x1, VBAT_GPIO_CTRL_EN);
#endif

#if CONFIG_ENSEMBLE_GEN2 /* ENSEMBLE_GEN2 SoC */
	/* GPIO: enable debounce clock. */
	enable_gpio_debounce_clock();
#endif /* CONFIG_ENSEMBLE_GEN2 */

	/* CAN settings */
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay) || \
	DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay))
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay)
	/* Flex GPIO */
	sys_write32(0x1, VBAT_GPIO_CTRL_EN);
#endif
	/* Enable HFOSC and 160MHz clock */
	sys_set_bits(CGU_CLK_ENA, BIT(20) | BIT(23));
#endif
	return 0;
}

SYS_INIT(soc_init, PRE_KERNEL_1, 1);
