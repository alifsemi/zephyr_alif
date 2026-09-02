/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_ALIF_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_ALIF_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Drop cached Hertz so the next get_rate() re-reads the clocks.
 *
 * Call after a successful se_service_set_run_cfg() that changes
 * cpu-clk-freq, clk-src, or scaled-clk-freq.
 */
void alif_clock_sys_clk_cache_invalidate(void);

/**
 * @brief Drop the cached CPU (EXTSYS) only.
 *
 * Zeros EXTSYS0 on RTSS_HP and EXTSYS1 on RTSS_HE. AXI/AHB/APB stay
 * valid when only cpu-clk-freq changed.
 */
void alif_clock_sys_clk_cache_invalidate_cpu(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_ALIF_H_ */
