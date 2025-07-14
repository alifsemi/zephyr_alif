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
	/* Enable LPRTC Clock via VBAT registers */
	sys_write32(0x1, VBAT_RTC_CLK_EN);

	return 0;
}

SYS_INIT(soc_init, PRE_KERNEL_1, 1);
