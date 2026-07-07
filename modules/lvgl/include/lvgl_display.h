/*
 * Copyright (c) 2019 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULES_LVGL_DISPLAY_H_
#define ZEPHYR_MODULES_LVGL_DISPLAY_H_

#include <zephyr/drivers/display.h>
#include <lvgl.h>

/*
 * NOTE (LVGL 9.5.0 backport): TEMPORARY compatibility shim - revert this whole
 * block once the Zephyr base is updated. The upstream 9.5.0 LVGL glue targets a
 * newer Zephyr display layer than this fork provides. Supply the missing pieces
 * so the glue builds on the current base:
 *   - DT_ZEPHYR_DISPLAYS_COUNT / DT_ZEPHYR_DISPLAY(): newer Zephyr derives these
 *     from a "zephyr,displays" node; here we synthesise a single-display setup
 *     from the chosen "zephyr,display".
 *   - PIXEL_FORMAT_{RGB_565X,L_8,AL_88}: display_pixel_format values added
 *     upstream after this fork. No driver on this base reports them, so the
 *     glue's switch cases are unreachable; define unique bits so they compile.
 */
#ifndef DT_ZEPHYR_DISPLAYS_COUNT
#if DT_HAS_CHOSEN(zephyr_display)
#define DT_ZEPHYR_DISPLAYS_COUNT 1
#define DT_ZEPHYR_DISPLAY(n)     DT_CHOSEN(zephyr_display)
#else
#define DT_ZEPHYR_DISPLAYS_COUNT 0
#endif
#endif /* DT_ZEPHYR_DISPLAYS_COUNT */

#ifndef PIXEL_FORMAT_RGB_565X
#define PIXEL_FORMAT_RGB_565X BIT(6)
#endif
#ifndef PIXEL_FORMAT_L_8
#define PIXEL_FORMAT_L_8      BIT(7)
#endif
#ifndef PIXEL_FORMAT_AL_88
#define PIXEL_FORMAT_AL_88    BIT(8)
#endif
/* end LVGL 9.5.0 backport shim */

#ifdef __cplusplus
extern "C" {
#endif

struct lvgl_disp_data {
	const struct device *display_dev;
	struct display_capabilities cap;
	bool blanking_on;
};

struct lvgl_display_flush {
	lv_display_t *display;
	uint16_t x;
	uint16_t y;
	struct display_buffer_descriptor desc;
	void *buf;
};

void lvgl_flush_cb_mono(lv_display_t *display, const lv_area_t *area, uint8_t *px_map);
void lvgl_flush_cb_8bit(lv_display_t *display, const lv_area_t *area, uint8_t *px_map);
void lvgl_flush_cb_16bit(lv_display_t *display, const lv_area_t *area, uint8_t *px_map);
void lvgl_flush_cb_24bit(lv_display_t *display, const lv_area_t *area, uint8_t *px_map);
void lvgl_flush_cb_32bit(lv_display_t *display, const lv_area_t *area, uint8_t *px_map);

void lvgl_rounder_cb_mono(lv_event_t *e);
void lvgl_set_mono_conversion_buffer(uint8_t *buffer, uint32_t buffer_size);

int set_lvgl_rendering_cb(lv_display_t *display);

void lvgl_flush_display(struct lvgl_display_flush *request);

#ifdef CONFIG_LV_Z_USE_ROUNDER_CB
void lvgl_rounder_cb(lv_event_t *e);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_MODULES_LVGL_DISPLAY_H_ */
