/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_INCLUDE_DRIVERS_DPHY_DW_H__
#define __ZEPHYR_INCLUDE_DRIVERS_DPHY_DW_H__

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <stddef.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Utility functions. */
#define CEIL(x) (((x) - (uint32_t)(x)) > 0 ? ((uint32_t)(x) + 1) : (uint32_t)(x))
#define ROUND(x) (uint32_t)((double)(x) + 0.5)

struct dphy_dsi_settings {
	/* Number of lanes in D-PHY. */
	uint8_t num_lanes;
	/* Output PLL frequency. */
	uint32_t pll_fout;
	/* Clk Lane HS->LP and LP->HS timings. */
	uint16_t clk_hs2lp;
	uint16_t clk_lp2hs;
	/* Data Lane HS->LP and LP->HS timings. */
	uint16_t lane_hs2lp;
	uint16_t lane_lp2hs;

	uint16_t pll_m;
	uint16_t pll_n;
	uint8_t pll_p;
	uint8_t vco_cntrl;
};

struct dphy_csi2_settings {
	/* Number of lanes in D-PHY. */
	uint8_t num_lanes;
	/* Input PLL frequency. */
	uint32_t pll_fin;
};

/*
 * Setup the D-PHY as TX-PHY.
 */
int dphy_dw_master_setup(const struct device *dev,
		struct dphy_dsi_settings *phy);

/*
 * Setup the D-PHY as RX-PHY.
 */
int dphy_dw_slave_setup(const struct device *dev,
		struct dphy_csi2_settings *phy,
		uint8_t dphy_id);

int dphy_dw_slave_select(const struct device *dev,
		uint8_t slave_id);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ZEPHYR_INCLUDE_DRIVERS_DPHY_DW_H__ */
