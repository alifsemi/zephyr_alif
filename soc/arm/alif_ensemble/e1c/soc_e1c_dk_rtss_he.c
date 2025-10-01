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
#endif
#include <se_service.h>
#include <zephyr/cache.h>

#define HOST_SYSTOP_PWR_REQ_LOGIC_ON_MEM_ON 0x12

/**
 * Set the RUN profile parameters for this application.
 */
static int pm_set_run_params(void)
{
	run_profile_t runp;

	runp.power_domains =
		PD_VBAT_AON_MASK | PD_SYST_MASK | PD_SSE700_AON_MASK | PD_DBSS_MASK | PD_SESS_MASK;
	runp.dcdc_voltage = 825;
	runp.dcdc_mode = DCDC_MODE_PFM_FORCED;
	runp.aon_clk_src = CLK_SRC_LFXO;
	runp.run_clk_src = CLK_SRC_PLL;
	runp.cpu_clk_freq = CLOCK_FREQUENCY_160MHZ;
	runp.phy_pwr_gating = LDO_PHY_MASK;
	runp.ip_clock_gating = LP_PERIPH_MASK;
	runp.vdd_ioflex_3V3 = IOFLEX_LEVEL_1V8;
	runp.scaled_clk_freq = SCALED_FREQ_XO_HIGH_DIV_38_4_MHZ;

	runp.memory_blocks = MRAM_MASK;
	runp.memory_blocks |= SRAM2_MASK | SRAM3_MASK;
	runp.memory_blocks |= SERAM_1_MASK | SERAM_2_MASK | SERAM_3_MASK | SERAM_4_MASK;
	runp.memory_blocks |=
		SRAM4_1_MASK | SRAM4_2_MASK | SRAM4_3_MASK | SRAM4_4_MASK; /* M55-HE ITCM */
	runp.memory_blocks |= SRAM5_1_MASK | SRAM5_2_MASK | SRAM5_3_MASK | SRAM5_4_MASK |
			      SRAM5_5_MASK; /* M55-HE DTCM */

	return se_service_set_run_cfg(&runp);
}

/*
 * This function will be invoked in the PRE_KERNEL_2 phase of the init
 * routine to prevent sleep during startup.
 */
static int soc_run_profile(void)
{
	int ret;
	uint32_t host_bsys_pwr_req = sys_read32(HOST_BSYS_PWR_REQ);

	sys_write32(host_bsys_pwr_req | HOST_SYSTOP_PWR_REQ_LOGIC_ON_MEM_ON, HOST_BSYS_PWR_REQ);

	ret = pm_set_run_params();
	if (ret) {
		return ret;
	}

	sys_write32(host_bsys_pwr_req, HOST_BSYS_PWR_REQ);

	return 0;
}
SYS_INIT(soc_run_profile, PRE_KERNEL_1, 2); /*CONFIG_SE_SERVICE_INIT_PRIORITY + 1 */

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * @return 0
 */
static int ensemble_e1c_dk_rtss_he_init(void)
{
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
