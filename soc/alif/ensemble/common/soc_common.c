/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>

/**
 * @brief Perform alif ensemble common SoC configuration at boot.
 *        This will run after soc_early_init_hook.
 *
 * @return 0
 */
static int alif_ensemble_common_soc_config(void)
{
	return 0;
}

SYS_INIT(alif_ensemble_common_soc_config, PRE_KERNEL_1, 1);
