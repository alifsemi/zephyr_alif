/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/gpio/gpio_mmio32.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/linker/linker-defs.h>
#ifdef CONFIG_REBOOT
#include <zephyr/sys/reboot.h>
#include <se_service.h>
#endif
#include <zephyr/cache.h>

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * @return 0
 */
static int ensemble_e1c_dk_rtss_he_init(void)
{
	/* Enable ICACHE */
	sys_cache_instr_enable();

	/* Enable DCACHE */
	sys_cache_data_enable();

	/* enable all UART[5-0] modules */
	/* select UART[5-0]_SCLK as SYST_PCLK clock. */
	sys_write32(0xFFFF, UART_CLK_EN);

	/* LPUART settings */
	if (IS_ENABLED(CONFIG_SERIAL)) {
		/* Enable clock supply for LPUART */
		sys_write32(0x1, AON_RTSS_HE_LPUART_CKEN);
	}

	/* RTC Clk Enable */
	sys_write32(0x1, LPRTC0_CLK_EN);
	sys_write32(0x1, LPRTC1_CLK_EN);

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

#if  DT_NODE_HAS_STATUS(DT_NODELABEL(lpspi0), okay)
	/*Clock : LP-SPI*/
	sys_set_bit(M55HE_CFG_HE_CLK_ENA, 16);

	/*LP-SPI0 Flex GPIO */
	sys_write32(0x1, VBAT_GPIO_CTRL_EN);

	/* LP-SPI0 Mode Selection */
#if (DT_PROP(DT_NODELABEL(lpspi0), serial_target))
	/* To Slave Set Bit : 15  */
	sys_set_bit(M55HE_CFG_HE_CLK_ENA, 15);
#else
	/* To Master Clear Bit : 15 */
	sys_clear_bit(M55HE_CFG_HE_CLK_ENA, 15);
#endif
#endif

	/* Enable LPPDM clock */
	if (IS_ENABLED(CONFIG_ALIF_PDM)) {
		sys_set_bits(HE_PER_CLK_EN, BIT(8));
	}

	if (IS_ENABLED(CONFIG_VIDEO)) {
		/*
		 * TODO: Check from the DTS property if LP-CAM is enabled and
		 * set clocks only for LP-CAM controller.
		 */
		/* Enable LPCAM Controller Peripheral clock. */
		sys_set_bits(HE_PER_CLK_EN, BIT(12));

		/* Enable LPCAM controller Pixel Clock (XVCLK). */
		/*
		 * Not needed for the time being as LP-CAM supports only
		 * parallel data-mode of cature and only MT9M114 sensor is
		 * tested with parallel data capture which generates clock
		 * internally. But can be used to generate XVCLK from LP CAM
		 * controller.
		 * sys_write32(0x140001, HE_CAMERA_PIXCLK);
		 */
	}

	if (IS_ENABLED(CONFIG_MIPI_DSI)) {
		/* Enable TX-DPHY and D-PLL Power and Disable Isolation.*/
		sys_clear_bits(VBAT_PWR_CTRL, BIT(0) | BIT(1) | BIT(8) |
				BIT(9) | BIT(12));

		/* Enable HFOSC (38.4 MHz) and CFG (100 MHz) clock.*/
		sys_set_bits(CGU_CLK_ENA, BIT(21) | BIT(23));
	}

	/*Clock : OSPI */
	if (IS_ENABLED(CONFIG_OSPI)) {
		sys_write32(0x1, EXPSLV_OSPI_CTRL);
	}

	/* lptimer settings */
#if DT_HAS_COMPAT_STATUS_OKAY(snps_dw_timers)
	/* LPTIMER 0 settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timer0), okay)
	if (IS_ENABLED(CONFIG_LPTIMER0_OUTPUT_TOGGLE) || (CONFIG_LPTIMER0_EXT_CLK_FREQ > 0U)) {
		/*
		 * enable of LPTIMER0 pin by config lpgpio
		 * pin 0 as Hardware control
		 */
		sys_set_bit(LPGPIO_BASE, 0);
	}
#endif  /* DT_NODE_HAS_STATUS(DT_NODELABEL(timer0), okay) */
	/* LPTIMER 1 settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timer1), okay)
	if (IS_ENABLED(CONFIG_LPTIMER1_OUTPUT_TOGGLE) || (CONFIG_LPTIMER1_EXT_CLK_FREQ > 0U)) {
		/*
		 * enable of LPTIMER1 pin by config lpgpio
		 * pin 1 as Hardware control
		 */
		sys_set_bit(LPGPIO_BASE, 1);
	}
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(timer1), okay) */
#endif /* DT_HAS_COMPAT_STATUS_OKAY(snps_dw_timers) */

	/* Enable DMA */
#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma2), arm_dma_pl330, okay)
	sys_set_bits(M55HE_CFG_HE_CLK_ENA, BIT(4));
	sys_write32(0x1111, EVTRTRLOCAL_DMA_REQ_CTRL);
	sys_clear_bits(M55HE_CFG_HE_DMA_CTRL, BIT(0));
	sys_write32(0U, M55HE_CFG_HE_DMA_IRQ);
	sys_write32(0U, M55HE_CFG_HE_DMA_PERIPH);
	sys_set_bits(M55HE_CFG_HE_DMA_CTRL, BIT(16));
#endif

	/* CAN settings */
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay) || \
		DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay))
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay)
	/*I3C Flex GPIO */
	sys_write32(0x1, VBAT_BASE);
#endif
	/* Enable HFOSC and 160MHz clock */
	reg_val  = sys_read32(CGU_CLK_ENA);
	reg_val |= ((1 << 20) | (1 << 23));
	sys_write32(reg_val, CGU_CLK_ENA);
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

SYS_INIT(ensemble_e1c_dk_rtss_he_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
