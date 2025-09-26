/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/arch/common/pm_s2ram.h>
#include <zephyr/linker/sections.h>
#include <zephyr/sys/util.h>
#include <hal/nrf_resetinfo.h>
#include "pm_s2ram.h"

#include <cmsis_core.h>

int soc_s2ram_suspend(pm_s2ram_system_off_fn_t system_off)
{
	int ret;

	pm_s2ram_save_ext_regs();
	ret = arch_pm_s2ram_suspend(system_off);
	if (ret < 0) {
		return ret;
	}

	pm_s2ram_restore_ext_regs();

	return ret;
}

void __attribute__((naked)) pm_s2ram_mark_set(void)
{
	/* empty */
	__asm__ volatile("bx	lr\n");
}

bool __attribute__((naked)) pm_s2ram_mark_check_and_clear(void)
{
	__asm__ volatile(
		/* Set return value to 0 */
		"mov	r0, #0\n"

		/* Load and check RESETREAS register */
		"ldr	r3, [%[resetinfo_addr], %[resetreas_offs]]\n"
		"cmp	r3, %[resetreas_unretained_mask]\n"

		"bne	exit\n"

		/* Clear RESETREAS register */
		"str	r0, [%[resetinfo_addr], %[resetreas_offs]]\n"

		/* Load RESTOREVALID register */
		"ldr	r3, [%[resetinfo_addr], %[restorevalid_offs]]\n"

		/* Clear RESTOREVALID */
		"str	r0, [%[resetinfo_addr], %[restorevalid_offs]]\n"

		/* Check RESTOREVALID register */
		"cmp	r3, %[restorevalid_present_mask]\n"
		"bne	exit\n"

		/* Set return value to 1 */
		"mov	r0, #1\n"

		"exit:\n"
		"bx	lr\n"
		:
		: [resetinfo_addr] "r"(NRF_RESETINFO),
		  [resetreas_offs] "r"(offsetof(NRF_RESETINFO_Type, RESETREAS.LOCAL)),
		  [resetreas_unretained_mask] "r"(NRF_RESETINFO_RESETREAS_LOCAL_UNRETAINED_MASK),
		  [restorevalid_offs] "r"(offsetof(NRF_RESETINFO_Type, RESTOREVALID)),
		  [restorevalid_present_mask] "r"(RESETINFO_RESTOREVALID_RESTOREVALID_Msk)

		: "r0", "r1", "r3", "r4", "memory");
}
