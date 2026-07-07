/*
 * Copyright (c) 2019 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <lvgl.h>
#include "lvgl_display.h"

void lvgl_flush_cb_24bit(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
	uint16_t w = area->x2 - area->x1 + 1;
	uint16_t h = area->y2 - area->y1 + 1;
	/* LVGL's aligned byte stride is the source of truth; pixel pitch is derived from it */
	uint32_t stride = ROUND_UP(w * 3U, LV_DRAW_BUF_STRIDE_ALIGN);
	struct lvgl_display_flush flush;

	flush.display = display;
	flush.x = area->x1;
	flush.y = area->y1;
	flush.desc.width = w;
	flush.desc.pitch = DIV_ROUND_UP(stride, 3U);
	flush.desc.buf_size = stride * h;
	flush.desc.height = h;
	flush.buf = (void *)px_map;

	if (IS_ENABLED(CONFIG_LV_Z_COLOR_24_BGR_TO_RGB)) {
		/* LVGL assumes BGR byte ordering, convert to RGB */
		for (uint16_t row = 0; row < h; row++) {
			uint8_t *line = px_map + (size_t)row * stride;

			for (uint16_t col = 0; col < w; col++) {
				uint8_t tmp = line[col * 3U];

				line[col * 3U] = line[col * 3U + 2U];
				line[col * 3U + 2U] = tmp;
			}
		}
	}

	lvgl_flush_display(&flush);
}
