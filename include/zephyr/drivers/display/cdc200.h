/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_INCLUDE_DRIVERS_CDC200_H__
#define __ZEPHYR_INCLUDE_DRIVERS_CDC200_H__

#include <zephyr/device.h>
#include <stddef.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Data structures */
/*
 * Frame buffer descriptor. Used by CDC200 specific APIs.
 */
struct cdc200_fb_desc {
	uint8_t *fb_addr;
	size_t fb_size;
};

/*
 * Layer Capabilities. Used to provide complete CDC200 Capabilities.
 */
struct cdc200_layer_caps {
	bool layer_en;
	uint16_t x_resolution;
	uint16_t y_resolution;
	enum display_pixel_format current_pixel_format;
};

/*
 * CDC200 Capabilities. Used by CDC200 specific API to extend the Standard API
 * functionalities.
 */
struct cdc200_display_caps {
	uint16_t x_panel_resolution;
	uint16_t y_panel_resolution;
	uint32_t supported_pixel_formats;
	uint32_t screen_info;
	enum display_orientation current_orientation;
	struct cdc200_layer_caps layer[2];
};

/* APIs */
/*
 * Get Framebuffer assigned to any layer of CDC200. Extension of standard API
 * to support multiple layers.
 */
void cdc200_get_framebuffer(const struct device *dev,
		uint8_t idx,
		struct cdc200_fb_desc *fb);

/*
 * Write to Framebuffer assigned to any layer of CDC200. Extension of standard
 * API to support multiple layers.
 */
int cdc200_display_write(const struct device *dev,
		uint8_t idx,
		const uint16_t x,
		const uint16_t y,
		const struct display_buffer_descriptor *desc,
		const void *buf);

/*
 * Read from Framebuffer assigned to any layer of CDC200. Extension of standard
 * API to support multiple layers.
 */
int cdc200_display_read(const struct device *dev,
		uint8_t idx,
		const uint16_t x,
		const uint16_t y,
		const struct display_buffer_descriptor *desc,
		void *buf);

/*
 * Get CDC200 capabilities. Extension of standard API to support multiple
 * layers.
 */
void cdc200_get_capabilities(const struct device *dev,
		struct cdc200_display_caps *capabilities);

/*
 * Enable/Disable the CDC200.
 */
void cdc200_set_enable(const struct device *dev,
		bool enable);

/*
 * Swap Framebuffers for specified layer. The buffer change will happen at next vblank
 */
void cdc200_swap_fb(const struct device *dev,
		uint8_t idx,
		struct cdc200_fb_desc *fb);

/*
 * Restore default framebuffers for CDC. The buffer change will happen at next vblank
 */
void restore_fb(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* __ZEPHYR_INCLUDE_DRIVERS_CDC200_H__ */
