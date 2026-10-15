/*
 * Copyright (C) 2026 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_OSPI_ALIF_OSPI_DT_H_
#define ZEPHYR_INCLUDE_DRIVERS_OSPI_ALIF_OSPI_DT_H_

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <ospi_hal.h>

/* Internal helpers for optional delay properties. */
#define ALIF_OSPI_ASSERT_DELAY_ELEM(node_id, prop, idx) \
	BUILD_ASSERT(DT_PROP_BY_IDX(node_id, prop, idx) <= OSPI_SIGNAL_DELAY_MAX, \
		     #prop " delay must be in range 0..23");

#define ALIF_OSPI_VALIDATE_DELAY_ARRAY(node_id, prop, count) \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, prop), \
		(BUILD_ASSERT(DT_PROP_LEN(node_id, prop) == (count), \
			      #prop " has an invalid number of entries"); \
		 DT_FOREACH_PROP_ELEM(node_id, prop, ALIF_OSPI_ASSERT_DELAY_ELEM)), ())

#define ALIF_OSPI_VALIDATE_DELAY_VALUE(node_id, prop) \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, prop), \
		(BUILD_ASSERT(DT_PROP(node_id, prop) <= OSPI_SIGNAL_DELAY_MAX, \
			      #prop " delay must be in range 0..23");), ())

/**
 * @brief Validate an Alif OSPI controller's signal-delay properties at build time.
 *
 * Invoke at file scope. Validation is skipped unless enable-signal-delay is set.
 * Optional arrays are checked for length and tap range. When rx-ds-delays is
 * absent, the scalar rx-ds-delay fallback is checked instead.
 *
 * @param node_id Devicetree node identifier for the OSPI controller.
 */
#define ALIF_OSPI_VALIDATE_SIGNAL_DELAYS(node_id) \
	COND_CODE_1(DT_PROP_OR(node_id, enable_signal_delay, 0), \
		(BUILD_ASSERT(IS_ENABLED(CONFIG_ENSEMBLE_GEN2), \
			      "OSPI per-signal delays require Ensemble Gen2"); \
		 ALIF_OSPI_VALIDATE_DELAY_ARRAY(node_id, txd_delays, 16) \
		 ALIF_OSPI_VALIDATE_DELAY_ARRAY(node_id, rxd_delays, 16) \
		 ALIF_OSPI_VALIDATE_DELAY_ARRAY(node_id, ssi_oe_n_delays, 16) \
		 ALIF_OSPI_VALIDATE_DELAY_ARRAY(node_id, rx_ds_delays, 2) \
		 ALIF_OSPI_VALIDATE_DELAY_ARRAY(node_id, txd_dm_delays, 2) \
		 ALIF_OSPI_VALIDATE_DELAY_ARRAY(node_id, dm_oe_n_delays, 2) \
		 ALIF_OSPI_VALIDATE_DELAY_ARRAY(node_id, ss_n_delays, 2) \
		 ALIF_OSPI_VALIDATE_DELAY_VALUE(node_id, sclk_delay) \
		 ALIF_OSPI_VALIDATE_DELAY_VALUE(node_id, sclkn_delay) \
		 COND_CODE_1(DT_NODE_HAS_PROP(node_id, rx_ds_delays), (), \
			(ALIF_OSPI_VALIDATE_DELAY_VALUE(node_id, rx_ds_delay)))), ())

#endif /* ZEPHYR_INCLUDE_DRIVERS_OSPI_ALIF_OSPI_DT_H_ */
