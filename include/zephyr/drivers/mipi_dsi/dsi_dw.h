/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_INCLUDE_DRIVERS_DSI_DW_H__
#define __ZEPHYR_INCLUDE_DRIVERS_DSI_DW_H__

#include <zephyr/device.h>
#include <stddef.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

enum dsi_dw_mode {
	DSI_DW_COMMAND_MODE,
	DSI_DW_VIDEO_MODE,
};

/*
 * Set command/video mode for Designware DSI.
 */
int dsi_dw_set_mode(const struct device *dev,
		enum dsi_dw_mode mode);

#ifdef __cplusplus
}
#endif
#endif /* __ZEPHYR_INCLUDE_DRIVERS_DSI_DW_H__ */
