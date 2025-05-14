/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/linker/linker-defs.h>
#include <string.h>
#include <cmsis_core.h>
#include <zephyr/cache.h>
#include <zephyr/sys/sys_io.h>

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 */
void soc_early_init_hook(void)
{
	/* Enable ICACHE */
	sys_cache_instr_enable();

	/* Enable DCACHE */
	sys_cache_data_enable();
}
