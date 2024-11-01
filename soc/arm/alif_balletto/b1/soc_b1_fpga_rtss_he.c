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

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * @return 0
 */
static int balletto_b1_fpga_rtss_he_init(void)
{
	/* Enable ICACHE */
	sys_cache_instr_enable();

	/* Enable DCACHE */
	sys_cache_data_enable();

	/* Might need to move later.. Just putting this here for now..*/
	/* Enable the UART clocks and set bypass to 100Mhz clocks */
	/* sys_write32(0x11, 0x4902F000); */
	sys_write32(0xFFFF, 0x4902F008);
	sys_write32(0x10000, 0x4903F00C);

	/* Set the pinmux for Port 3 pins P3_1 and P3_2 to UART4 Rx/Tx */
	/* B0 ENSEMBLE FPGA
	 * sys_write32(0x00220002, 0x1A6030A0);
	 * sys_write32(0x00220002, 0x1A6030A4);
	 * B1 BLE FPGA
	 * sys_write32(0x00220002, 0x1A60307C);
	 *  sys_write32(0x00220002, 0x1A603078);
	 */

	/* Enable AHI Tracing */
	sys_write32(0x00000004, 0x1a605008);

	/* Set AHI and HCI break condition.
	 * Makes link layer deep sleep possible if one, or the other, is not used and its driver
	 * isn't loaded
	 */
	uint8_t lcr_reg1 = sys_read8(0x4300A00c);
	uint8_t lcr_reg2 = sys_read8(0x4300B00c);

	sys_write8(lcr_reg1 | 0x40, 0x4300A00c);
	sys_write8(lcr_reg2 | 0x40, 0x4300B00c);

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

SYS_INIT(balletto_b1_fpga_rtss_he_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
