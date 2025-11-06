/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT tes_cdc_2_1

#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include <string.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/display/cdc200.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/device_mmio.h>
#include "display_cdc200.h"
#include <soc_memory_map.h>

LOG_MODULE_REGISTER(CDC200, CONFIG_DISPLAY_LOG_LEVEL);

/* Helper functions */
/* Global configuration setup functions. */
void cdc200_shadow_reload_control(uintptr_t regs)
{
	sys_write32(CDC_SRCTRL_IMR, regs + CDC_SRCTRL);
}

void cdc200_set_irq_mask(uintptr_t regs, uint32_t irqs)
{
	sys_write32(irqs, regs + CDC_IRQ_MASK0);
}

uint32_t cdc200_get_irq_status(uintptr_t regs)
{
	return sys_read32(regs + CDC_IRQ_STATUS0);
}

void cdc200_set_irq_clear(uintptr_t regs, uint32_t irqs)
{
	sys_write32(irqs, regs + CDC_IRQ_CLEAR0);
}

void cdc200_global_enable(uintptr_t regs)
{
	uint32_t temp = sys_read32(regs + CDC_GLB_CTRL);

	temp |= CDC_GLB_CTRL_CDC_EN;
	sys_write32(temp, regs + CDC_GLB_CTRL);
}

void cdc200_global_disable(uintptr_t regs)
{
	uint32_t temp = sys_read32(regs + CDC_GLB_CTRL);

	temp &= ~CDC_GLB_CTRL_CDC_EN;
	sys_write32(temp, regs + CDC_GLB_CTRL);
}

void cdc200_irq_setup(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	const struct cdc200_config *config = dev->config;
	const struct cdc200_panel_config *panel_cfg = &config->panel_cfg;
	uint32_t linpos =
		(panel_cfg->vsync_len + panel_cfg->vbp + panel_cfg->active_height) & 0xffff;

	/* Enable just the Line IRQ. */
	sys_write32(linpos, regs + CDC_LINE_IRQ_POS);
	cdc200_set_irq_mask(regs, CDC_IRQ_LINE);
}

void cdc200_global_timings_set(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	const struct cdc200_config *config = dev->config;
	const struct cdc200_panel_config *panel_cfg = &config->panel_cfg;

	uint32_t sync_size = (((panel_cfg->hsync_len - 1) & 0xffff) << 16) +
			     ((panel_cfg->vsync_len - 1) & 0xffff);
	uint32_t back_porch =
		(((panel_cfg->hbp) & 0xffff) << 16) + ((panel_cfg->vbp) & 0xffff) + sync_size;
	uint32_t active_width = ((panel_cfg->active_width & 0xffff) << 16) +
				(panel_cfg->active_height & 0xffff) + back_porch;
	uint32_t total_width =
		((panel_cfg->hfp & 0xffff) << 16) + (panel_cfg->vfp & 0xffff) + active_width;

	/* Setup global signal polarity. */
	sys_write32(panel_cfg->polarity, regs + CDC_GLB_CTRL);

	/* Setup Global Timing registers. */
	sys_write32(sync_size, regs + CDC_SYNC_SIZE_CFG);
	sys_write32(back_porch, regs + CDC_BP_CFG);
	sys_write32(active_width, regs + CDC_ACTW_CFG);
	sys_write32(total_width, regs + CDC_TOTALW_CFG);
}

void cdc200_global_set_bg_color(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	const struct cdc200_config *config = dev->config;

	sys_write32(config->bg_color, regs + CDC_BACKGND_COLOR);
}

/* Layer setup functions. */
void cdc200_layer_enable(uintptr_t regs, enum cdc_layer idx)
{
	uint32_t temp = 0;

	switch (idx) {
	case CDC_LAYER_1:
		temp = sys_read32(regs + CDC_L1_CTRL);
		temp |= CDC_LN_CTRL_LAYER_EN;
		sys_write32(temp, regs + CDC_L1_CTRL);
		break;
	case CDC_LAYER_2:
		temp = sys_read32(regs + CDC_L2_CTRL);
		temp |= CDC_LN_CTRL_LAYER_EN;
		sys_write32(temp, regs + CDC_L2_CTRL);
		break;
	default:
		LOG_ERR("Incorrect Layer ID.");
	}
}

void cdc200_layer_disable(uintptr_t regs, enum cdc_layer idx)
{
	uint32_t temp = 0;

	switch (idx) {
	case CDC_LAYER_1:
		temp = sys_read32(regs + CDC_L1_CTRL);
		temp &= ~CDC_LN_CTRL_LAYER_EN;
		sys_write32(temp, regs + CDC_L1_CTRL);
		break;
	case CDC_LAYER_2:
		temp = sys_read32(regs + CDC_L2_CTRL);
		temp &= ~CDC_LN_CTRL_LAYER_EN;
		sys_write32(temp, regs + CDC_L2_CTRL);
		break;
	default:
		LOG_ERR("Incorrect Layer ID.");
	}
}

void cdc200_layer_blend_cfg_set(uintptr_t regs, enum cdc_layer idx, int f1, int f2)
{
	switch (idx) {
	case CDC_LAYER_1:
		sys_write32((f1 & CDC_LN_BLEND_CFG_F1_SEL_MASK) << CDC_LN_BLEND_CFG_F1_SEL_SHIFT |
				    (f2 & CDC_LN_BLEND_CFG_F2_SEL_MASK),
			    regs + CDC_L1_BLEND_CFG);
		break;
	case CDC_LAYER_2:
		sys_write32((f1 & CDC_LN_BLEND_CFG_F1_SEL_MASK) << CDC_LN_BLEND_CFG_F1_SEL_SHIFT |
				    (f2 & CDC_LN_BLEND_CFG_F2_SEL_MASK),
			    regs + CDC_L2_BLEND_CFG);
		break;
	default:
		LOG_ERR("Incorrect Layer ID.");
	}
}

void cdc200_layer_const_alpha_set(uintptr_t regs, enum cdc_layer idx, int alpha)
{
	switch (idx) {
	case CDC_LAYER_1:
		sys_write32(alpha & CDC_LN_CONST_ALPHA_CONSTA_MASK, regs + CDC_L1_CONST_ALPHA);
		break;
	case CDC_LAYER_2:
		sys_write32(alpha & CDC_LN_CONST_ALPHA_CONSTA_MASK, regs + CDC_L2_CONST_ALPHA);
		break;
	default:
		LOG_ERR("Incorrect Layer ID.");
	}
}

int cdc200_layer_fb_setup(const struct device *dev, enum cdc_layer idx)
{
	const struct cdc200_config *config = dev->config;
	const struct cdc200_layer_config *layer = config->layer + idx;
	uintptr_t regs = DEVICE_MMIO_GET(dev);

	const uint32_t fb_len = (layer->x1 - layer->x0) * layer->pixel_size;
	const uint32_t fb_llen = fb_len + BUS_WIDTH;

	switch (idx) {
	case CDC_LAYER_1:
#ifdef CONFIG_FB_USES_DTCM_REGION
		sys_write32(local_to_global(UINT_TO_POINTER(layer->fb_addr)), regs + CDC_L1_CFB_ADDR);
#else
		sys_write32((uint32_t)layer->fb_addr, regs + CDC_L1_CFB_ADDR);
#endif
		sys_write32(layer->fmt, regs + CDC_L1_PIX_FORMAT);
		sys_write32(
			((fb_len & CDC_LN_CFB_LENGTH_PITCH_MASK) << CDC_LN_CFB_LENGTH_PITCH_SHIFT) |
				(fb_llen & CDC_LN_CFB_LENGTH_LLENGTH_MASK),
			regs + CDC_L1_CFB_LENGTH);
		sys_write32(layer->y1 - layer->y0, regs + CDC_L1_CFB_LINES);
		break;
	case CDC_LAYER_2:
#ifdef CONFIG_FB_USES_DTCM_REGION
		sys_write32(local_to_global(UINT_TO_POINTER(layer->fb_addr)), regs + CDC_L2_CFB_ADDR);
#else
		sys_write32((uint32_t)layer->fb_addr, regs + CDC_L2_CFB_ADDR);
#endif
		sys_write32(layer->fmt, regs + CDC_L2_PIX_FORMAT);
		sys_write32(
			((fb_len & CDC_LN_CFB_LENGTH_PITCH_MASK) << CDC_LN_CFB_LENGTH_PITCH_SHIFT) |
				(fb_llen & CDC_LN_CFB_LENGTH_LLENGTH_MASK),
			regs + CDC_L2_CFB_LENGTH);
		sys_write32(layer->y1 - layer->y0, regs + CDC_L2_CFB_LINES);
		break;
	default:
		LOG_ERR("Incorrect Layer ID.");
	}
	return 0;
}

int cdc200_layer_dflt_color_set(const struct device *dev, enum cdc_layer idx)
{
	const struct cdc200_config *config = dev->config;
	const struct cdc200_layer_config *layer = config->layer + idx;
	const struct cdc200_panel_config *panel_cfg = &config->panel_cfg;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	uint32_t temp = 0;

	const uint32_t lw = layer->x1 - layer->x0;
	const uint32_t lh = layer->y1 - layer->y0;

	/*
	 * If the default color is not enabled or
	 * layer is enabled with no windowing - skip default layer color.
	 * Else, setup default layer color & enable blending with it.
	 */
	if (!layer->def_color_en || (layer->layer_en && (!(lw - panel_cfg->active_width) &&
							 !(lh - panel_cfg->active_height)))) {
		return 0;
	}

	switch (idx) {
	case CDC_LAYER_1:
		temp = sys_read32(regs + CDC_L1_CTRL);
		temp |= CDC_LN_CTRL_DFLT_CBLEND_EN;
		sys_write32(temp, regs + CDC_L1_CTRL);
		sys_write32(layer->def_color, regs + CDC_L1_DFLT_COLOR);
		break;
	case CDC_LAYER_2:
		temp = sys_read32(regs + CDC_L2_CTRL);
		temp |= CDC_LN_CTRL_DFLT_CBLEND_EN;
		sys_write32(temp, regs + CDC_L2_CTRL);
		sys_write32(layer->def_color, regs + CDC_L2_DFLT_COLOR);
		break;
	default:
		LOG_ERR("Incorrect Layer ID.");
	}
	return 0;
}

void cdc200_layer_blending_setup(const struct device *dev, enum cdc_layer idx)
{
	const struct cdc200_config *config = dev->config;
	const struct cdc200_layer_config *layer = config->layer + idx;
	uintptr_t regs = DEVICE_MMIO_GET(dev);

	switch (layer->fmt) {
	case CDC_PIXEL_FORMAT_RGB888:
	case CDC_PIXEL_FORMAT_RGB565:
		cdc200_layer_blend_cfg_set(regs, idx, CDC_BLEND_CONST_ALPHA,
					   CDC_BLEND_CONST_ALPHA_INV);
		break;
	default:
		cdc200_layer_blend_cfg_set(regs, idx, layer->curr_layer_bf_f1,
					   layer->subj_layer_bf_f2);
	}
	cdc200_layer_const_alpha_set(regs, idx, layer->const_alpha);
}

int cdc200_layer_window_set(const struct device *dev, enum cdc_layer idx)
{
	const struct cdc200_config *config = dev->config;
	const struct cdc200_layer_config *layer = config->layer + idx;
	const struct cdc200_panel_config *panel_cfg = &config->panel_cfg;
	uintptr_t regs = DEVICE_MMIO_GET(dev);

	uint32_t layer_hstart = panel_cfg->hsync_len + panel_cfg->hbp + layer->x0;
	uint32_t layer_hend = panel_cfg->hsync_len + panel_cfg->hbp + layer->x1 - 1;
	uint32_t layer_vstart = panel_cfg->vsync_len + panel_cfg->vbp + layer->y0;
	uint32_t layer_vend = panel_cfg->vsync_len + panel_cfg->vbp + layer->y1 - 1;

	if ((layer->x1 > panel_cfg->active_width) || (layer->y1 > panel_cfg->active_height)) {
		LOG_ERR("Incorrect window parameters for layer %d", idx + 1);
		return -EINVAL;
	}

	switch (idx) {
	case CDC_LAYER_1:
		sys_write32((layer_hstart & CDC_LN_WIN_HPOS_START_POS_MASK) |
				    ((layer_hend & CDC_LN_WIN_HPOS_STOP_POS_MASK)
				     << CDC_LN_WIN_HPOS_STOP_POS_SHIFT),
			    regs + CDC_L1_WIN_HPOS);
		sys_write32((layer_vstart & CDC_LN_WIN_VPOS_START_POS_MASK) |
				    ((layer_vend & CDC_LN_WIN_VPOS_STOP_POS_MASK)
				     << CDC_LN_WIN_VPOS_STOP_POS_SHIFT),
			    regs + CDC_L1_WIN_VPOS);
		break;
	case CDC_LAYER_2:
		sys_write32((layer_hstart & CDC_LN_WIN_HPOS_START_POS_MASK) |
				    ((layer_hend & CDC_LN_WIN_HPOS_STOP_POS_MASK)
				     << CDC_LN_WIN_HPOS_STOP_POS_SHIFT),
			    regs + CDC_L2_WIN_HPOS);
		sys_write32((layer_vstart & CDC_LN_WIN_VPOS_START_POS_MASK) |
				    ((layer_vend & CDC_LN_WIN_VPOS_STOP_POS_MASK)
				     << CDC_LN_WIN_VPOS_STOP_POS_SHIFT),
			    regs + CDC_L2_WIN_VPOS);
		break;
	default:
		LOG_ERR("Incorrect Layer ID.");
	}
	return 0;
}

int cdc200_setup_registers(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	const struct cdc200_config *config = dev->config;
	const struct cdc200_layer_config *layer = config->layer;
	int ret = 0;

	/* Layer(s) setup */
	for (int i = CDC_LAYER_1; i <= CDC_LAYER_2; i++) {
		cdc200_layer_dflt_color_set(dev, i);
		cdc200_shadow_reload_control(regs);
		cdc200_layer_blending_setup(dev, i);
		if (layer[i].layer_en) {
			ret = cdc200_layer_window_set(dev, i);
			if (ret) {
				LOG_ERR("Failed setting window on layer - %d", (i + 1));
				return ret;
			}
			cdc200_layer_fb_setup(dev, i);
			cdc200_layer_enable(regs, i);
		}
		cdc200_shadow_reload_control(regs);
	}

	/* Global registers setup */
	cdc200_global_timings_set(dev);
	cdc200_global_set_bg_color(dev);
	cdc200_irq_setup(dev);
	return 0;
}

/* API functions */
/* Generic APIs */
static int cdc200_blanking_on(const struct device *dev)
{
	/*
	 * Disable the Backlight GPIO here.
	 * Not available with parallel display.
	 */
	return -ENOTSUP;
}

static int cdc200_blanking_off(const struct device *dev)
{
	/*
	 * Enable the Backlight GPIO here.
	 * Not available with parallel display.
	 */
	return -ENOTSUP;
}

int cdc200_generic_write(const struct device *dev, const uint16_t x, const uint16_t y,
			 const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct cdc200_config *config = dev->config;
	/* Always get config for Layer 0 by default. */
	const struct cdc200_layer_config *layer = config->layer;
	struct cdc200_data *data = dev->data;
	const uint8_t *src = buf;
	uint32_t width;
	uint8_t *dst;
	uint16_t row;
	size_t pix_size;

	width = layer->x1 - layer->x0;
	pix_size = layer->pixel_size;

	dst = data->curr_fb[CDC_LAYER_1];
	dst += (y * width + x) * pix_size;

	for (row = 0; row < desc->height; row++) {
		memcpy(dst, src, desc->width * layer->pixel_size);
		dst += (width * layer->pixel_size);
		src += (desc->pitch * layer->pixel_size);
	}

	return 0;
}

int cdc200_generic_read(const struct device *dev, const uint16_t x, const uint16_t y,
			const struct display_buffer_descriptor *desc, void *buf)
{
	const struct cdc200_config *config = dev->config;
	/* Always get config for Layer 0 by default. */
	const struct cdc200_layer_config *layer = config->layer;
	struct cdc200_data *data = dev->data;
	const uint8_t *src;
	uint8_t *dst = buf;
	uint32_t width;
	uint16_t row;
	size_t pix_size;

	width = layer->x1 - layer->x0;
	pix_size = layer->pixel_size;

	src = data->curr_fb[CDC_LAYER_1];
	src += (y * width + x) * pix_size;

	for (row = 0; row < desc->height; row++) {
		memcpy(dst, src, desc->width * layer->pixel_size);
		src += (width * layer->pixel_size);
		dst += (desc->pitch * layer->pixel_size);
	}

	return 0;
}

static void *cdc200_generic_get_fb(const struct device *dev)
{
	struct cdc200_data *data = dev->data;
	void *ret = NULL;

	ret = (void *)data->curr_fb[CDC_LAYER_1];

	return ret;
}

static int cdc200_set_brightness(const struct device *dev, const uint8_t brightness)
{
	return -ENOTSUP;
}

static int cdc200_set_contrast(const struct device *dev, const uint8_t contrast)
{
	return -ENOTSUP;
}

void cdc200_generic_get_caps(const struct device *dev, struct display_capabilities *capabilities)
{
	const struct cdc200_config *config = dev->config;

	memset(capabilities, 0, sizeof(*capabilities));
	capabilities->x_resolution = config->panel_cfg.active_width;
	capabilities->y_resolution = config->panel_cfg.active_height;
	capabilities->supported_pixel_formats =
		PIXEL_FORMAT_ARGB_8888 | PIXEL_FORMAT_RGB_888 | PIXEL_FORMAT_RGB_565;
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static int cdc200_set_pixel_format(const struct device *dev, const enum display_pixel_format format)
{
	const struct cdc200_config *config = dev->config;
	const struct cdc200_layer_config *layer = config->layer;
	uint32_t pixel_fmt = 0;

	switch (layer->fmt) {
	case CDC_PIXEL_FORMAT_ARGB8888:
		pixel_fmt = PIXEL_FORMAT_ARGB_8888;
		break;
	case CDC_PIXEL_FORMAT_RGB888:
		pixel_fmt = PIXEL_FORMAT_RGB_888;
		break;
	case CDC_PIXEL_FORMAT_RGB565:
		pixel_fmt = PIXEL_FORMAT_RGB_565;
		break;
	default:
		break;
	}

	if (format == pixel_fmt) {
		return 0;
	}
	LOG_INF("Pixel Format change not supported.");
	return -ENOTSUP;
}

static int cdc200_set_orientation(const struct device *dev,
				  const enum display_orientation orientation)
{
	ARG_UNUSED(dev);
	if (orientation == DISPLAY_ORIENTATION_NORMAL) {
		return 0;
	}
	return -ENOTSUP;
}

/* Device Specific APIs */
void cdc200_get_framebuffer(const struct device *dev, uint8_t idx, struct cdc200_fb_desc *fb)
{
	struct cdc200_data *data = dev->data;
	const struct cdc200_config *config = dev->config;
	const struct cdc200_layer_config *layer = config->layer + idx;

	if (idx > CDC_LAYER_2) {
		return;
	}

	fb->fb_addr = data->curr_fb[idx];
	fb->fb_size = layer->fb_size;
}

int cdc200_display_write(const struct device *dev, uint8_t idx, const uint16_t x, const uint16_t y,
			 const struct display_buffer_descriptor *desc, const void *buf)
{
	struct cdc200_data *data = dev->data;
	const struct cdc200_config *config = dev->config;
	const struct cdc200_layer_config *layer = config->layer + idx;
	const uint8_t *src = buf;
	uint32_t width;
	uint8_t *dst;
	uint16_t row;
	size_t pix_size;

	if (idx > CDC_LAYER_2) {
		return -EINVAL;
	}

	width = layer->x1 - layer->x0;
	pix_size = layer->pixel_size;

	dst = data->curr_fb[idx];
	dst += (y * width + x) * pix_size;

	for (row = 0; row < desc->height; row++) {
		memcpy(dst, src, desc->width * layer->pixel_size);
		dst += (width * layer->pixel_size);
		src += (desc->pitch * layer->pixel_size);
	}

	return 0;
}

int cdc200_display_read(const struct device *dev, uint8_t idx, const uint16_t x, const uint16_t y,
			const struct display_buffer_descriptor *desc, void *buf)
{
	struct cdc200_data *data = dev->data;
	const struct cdc200_config *config = dev->config;
	const struct cdc200_layer_config *layer = config->layer + idx;
	const uint8_t *src;
	uint8_t *dst = buf;
	uint32_t width;
	uint16_t row;
	size_t pix_size;

	if (idx > CDC_LAYER_2) {
		return -EINVAL;
	}

	width = layer->x1 - layer->x0;
	pix_size = layer->pixel_size;

	src = data->curr_fb[idx];
	src += (y * width + x) * pix_size;

	for (row = 0; row < desc->height; row++) {
		memcpy(dst, src, desc->width * layer->pixel_size);
		src += (width * layer->pixel_size);
		dst += (desc->pitch * layer->pixel_size);
	}

	return 0;
}

void cdc200_get_capabilities(const struct device *dev, struct cdc200_display_caps *capabilities)
{
	const struct cdc200_config *config = dev->config;
	const struct cdc200_layer_config *layer = config->layer;

	memset(capabilities, 0, sizeof(struct cdc200_display_caps));
	capabilities->x_panel_resolution = config->panel_cfg.active_width;
	capabilities->y_panel_resolution = config->panel_cfg.active_height;
	capabilities->supported_pixel_formats =
		PIXEL_FORMAT_ARGB_8888 | PIXEL_FORMAT_RGB_888 | PIXEL_FORMAT_RGB_565;
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;

	for (int i = CDC_LAYER_1; i <= CDC_LAYER_2; i++) {
		capabilities->layer[i].layer_en = layer[i].layer_en;
		capabilities->layer[i].x_resolution = layer[i].x1 - layer[i].x0;
		capabilities->layer[i].y_resolution = layer[i].y1 - layer[i].y0;
		switch (layer[i].fmt) {
		case CDC_PIXEL_FORMAT_ARGB8888:
			capabilities->layer[i].current_pixel_format = PIXEL_FORMAT_ARGB_8888;
			break;
		case CDC_PIXEL_FORMAT_RGB888:
			capabilities->layer[i].current_pixel_format = PIXEL_FORMAT_RGB_888;
			break;
		case CDC_PIXEL_FORMAT_RGB565:
			capabilities->layer[i].current_pixel_format = PIXEL_FORMAT_RGB_565;
			break;
		default:
			break;
		}
	}
}

void cdc200_set_enable(const struct device *dev, bool enable)
{
	uintptr_t regs = DEVICE_MMIO_GET(dev);

	if (enable) {
		cdc200_global_enable(regs);
	} else {
		cdc200_global_disable(regs);
	}
}

void cdc200_swap_fb(const struct device *dev, uint8_t idx, struct cdc200_fb_desc *fb)
{
	struct cdc200_data *data = dev->data;
	const struct cdc200_config *config = dev->config;
	const struct cdc200_layer_config *layer = config->layer + idx;

	if (idx > CDC_LAYER_2) {
		return;
	}

	if (fb->fb_size != layer->fb_size) {
		LOG_ERR("In-compatible Framebuffers.");
		return;
	}

	data->next_fb[idx] = fb->fb_addr;
}

void restore_fb(const struct device *dev)
{
	struct cdc200_data *data = dev->data;
	const struct cdc200_config *config = dev->config;
	const struct cdc200_layer_config *layer = config->layer;

	data->next_fb[CDC_LAYER_1] = layer->fb_addr;
	data->next_fb[CDC_LAYER_2] = (layer + CDC_LAYER_2)->fb_addr;
}

/* ISR function */
static void cdc200_isr(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	uint32_t irq_st = cdc200_get_irq_status(regs);
	struct cdc200_data *data = dev->data;

	if (irq_st & CDC_IRQ_LINE) {
		cdc200_set_irq_clear(regs, CDC_IRQ_LINE);

		if (data->curr_fb[CDC_LAYER_1] != data->next_fb[CDC_LAYER_1]) {
			sys_write32((uint32_t)data->next_fb[CDC_LAYER_1], regs + CDC_L1_CFB_ADDR);
			data->curr_fb[CDC_LAYER_1] = data->next_fb[CDC_LAYER_1];
		}

		if (data->curr_fb[CDC_LAYER_2] != data->next_fb[CDC_LAYER_2]) {
			sys_write32((uint32_t)data->next_fb[CDC_LAYER_2], regs + CDC_L2_CFB_ADDR);
			data->curr_fb[CDC_LAYER_2] = data->next_fb[CDC_LAYER_2];
		}
		cdc200_shadow_reload_control(regs);
	}
}

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(clocks)
static int cdc200_setup_pixel_clock(const struct device *dev)
{
	const struct cdc200_config *config = dev->config;
	struct cdc200_data *data = dev->data;
	int ret;

	ret = clock_control_configure(config->clk_dev, config->pix_cid, NULL);
	if (ret) {
		LOG_ERR("Pixel-CLK source configuration failed! ret - %d", ret);
		return ret;
	}

	ret = clock_control_set_rate(config->clk_dev, config->pix_cid,
			(clock_control_subsys_rate_t)data->clk_freq);
	if (ret) {
		LOG_ERR("Pixel-CLK set frequency failed! ret - %d", ret);
		return ret;
	}

	ret = clock_control_on(config->clk_dev, config->pix_cid);
	if (ret) {
		LOG_ERR("Enable Pixel-CLK failed! ret - %d", ret);
		return ret;
	}

	return 0;

}

static int cdc200_setup_clocks(const struct device *dev)
{
	const struct cdc200_config *config = dev->config;
	int ret;

	/* Enable DPI clock. */
	ret = clock_control_on(config->clk_dev, config->dpi_cid);
	if (ret) {
		LOG_ERR("Enable DPI clock source for APB & AXI interface failed! ret - %d", ret);
		return ret;
	}

	/* Setup and Enable Pixel clock. */
	ret = cdc200_setup_pixel_clock(dev);
	if (ret) {
		return ret;
	}

	return 0;
}

static uint32_t cdc200_pixel_clock_valid(const struct device *dev)
{
	const struct cdc200_config *config = dev->config;
	const struct cdc200_panel_config *pcfg = &config->panel_cfg;
	struct cdc200_data *data = dev->data;
	uint32_t htotal = 0;
	uint32_t vtotal = 0;

	htotal = (pcfg->hsync_len + pcfg->hbp + pcfg->hfp + pcfg->active_width);
	vtotal = (pcfg->vsync_len + pcfg->vbp + pcfg->vfp + pcfg->active_height);

	LOG_DBG("Min pixel clock for 1 FPS required: %d", htotal * vtotal);
	LOG_DBG("Expected FPS: %f", (double)data->clk_freq / (htotal * vtotal));
	return ((htotal * vtotal) <= data->clk_freq);
}
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(clocks) */

/* Init function */
static int cdc200_init(const struct device *dev)
{
	const struct cdc200_config *config = dev->config;
	const struct cdc200_panel_config *pcfg = &config->panel_cfg;
	const struct cdc200_layer_config *layer = NULL;
	struct cdc200_data *data = dev->data;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	LOG_DBG("MMIO address - 0x%x", (uint32_t)DEVICE_MMIO_GET(dev));

	data->dev = dev;

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(clocks)
	if (!cdc200_pixel_clock_valid(dev)) {
		LOG_ERR("Incorrect Pixel Clock value supplied in DTS.");
		return -EINVAL;
	}
	ret = cdc200_setup_clocks(dev);
	if (ret) {
		LOG_ERR("CDC200 Clock enable failed! Exiting!");
		return ret;
	}
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(clocks) */

#if !defined(CONFIG_MIPI_DSI)
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	LOG_DBG("Applying Pinctrl state. ret - %d", ret);
	if (ret < 0) {
		return ret;
	}
#endif /* !defined(CONFIG_MIPI_DSI) */

	/* Setup the CDC200 controller... */
	ret = cdc200_setup_registers(dev);
	if (ret) {
		return ret;
	}

	/* Install ISR handler. */
	config->irq_config_func(dev);

	LOG_DBG("Device setup parameters.");
	LOG_DBG("\tIRQ number for Scanline interrupt - %d", config->irq);
	LOG_DBG("\tEnabled Layers - %d", data->layer_count);
	LOG_DBG("\tBackground color - 0x%06x", config->bg_color);
	LOG_DBG("\thtimings(%d, %d, %d, %d)\tvtimings(%d, %d, %d, %d)",
			pcfg->hsync_len, pcfg->hbp, pcfg->active_width, pcfg->hfp,
			pcfg->vsync_len, pcfg->vbp, pcfg->active_height, pcfg->vfp);

	LOG_DBG("Layer Parameters.");
	for (uint32_t i = CDC_LAYER_1; i < CDC_LAYER_MAX; i++) {
		layer = config->layer + i;
		LOG_DBG("\tLayer %d FB-Addr - 0x%x", (i + 1), (uint32_t)layer->fb_addr);
		LOG_DBG("\tLayer %d FB size - %d", (i + 1), layer->fb_size);
		LOG_DBG("\tLayer %d FB Format - %d", (i + 1), layer->fmt);

		LOG_DBG("\tLayer %d windowing (x0, y0) - (%d, %d)", (i + 1), layer->x0, layer->y0);
		LOG_DBG("\tLayer %d windowing (x1, y1) - (%d, %d)", (i + 1), layer->x1, layer->y1);
	}
	return 0;
}

static DEVICE_API(display, cdc200_display_api) = {
	.blanking_on = cdc200_blanking_on,
	.blanking_off = cdc200_blanking_off,
	.write = cdc200_generic_write,
	.read = cdc200_generic_read,
	.get_framebuffer = cdc200_generic_get_fb,
	.set_brightness = cdc200_set_brightness,
	.set_contrast = cdc200_set_contrast,
	.get_capabilities = cdc200_generic_get_caps,
	.set_pixel_format = cdc200_set_pixel_format,
	.set_orientation = cdc200_set_orientation,
	/*
	 * Following APIs have been extended to provide layer support:
	 *	1. write,
	 *	2. read,
	 *	3. get_framebuffer,
	 *	4. get_capabilities
	 *
	 * These APIs are provided as device specific APIs,
	 * and the generic APIs work for only layer 0.
	 */
};

/***************** Per-layer Pixel size calculation Macros. ******************/
#define CDC200_PIXEL_SIZE_IDX_0         4 /* ARGB-8888 */
#define CDC200_PIXEL_SIZE_IDX_1         3 /* RGB-888 */
#define CDC200_PIXEL_SIZE_IDX_2         2 /* RGB-565 */
#define CDC200_PIXEL_SIZE_IDX_3         4 /* RGBA-8888 */
#define CDC200_PIXEL_SIZE_IDX_4         1 /* AL-44 */
#define CDC200_PIXEL_SIZE_IDX_5         1 /* L-8 */
#define CDC200_PIXEL_SIZE_IDX_6         2 /* ARGB-1555 */
#define CDC200_PIXEL_SIZE_IDX_7         2 /* ARGB-4444 */
#define CDC200_PIXEL_SIZE_IDX_8         0 /* Undefined */
#define CDC200_PIXEL_SIZE_IDX_UNDEFINED CDC200_PIXEL_SIZE_IDX_8

#define LAYER_PROPERTY(prop, i) UTIL_CAT(prop, i)

#define CDC200_PIXEL_SIZE_LAYER(i, n)                                                              \
	UTIL_CAT(CDC200_PIXEL_SIZE_IDX_,                                                           \
		 DT_INST_ENUM_IDX_OR(i, LAYER_PROPERTY(pixel_fmt_l, n), UNDEFINED))

/********************* Frame-buffer allocation Macros. ***********************/
/*
 * FIXME: allocate the maximum framebuffer that can be allocated for 480x800 display.
 * Issue observed due to compilation issue with armClang.
 */
#if defined(CONFIG_FB_USES_DTCM_REGION)
#if defined(NO_RELOCATE_SRAM0)
#define ALLOCATE_FB0(i)
#define ALLOCATE_FB1(i)
#define FB0(i) ((uint8_t *)DT_REG_ADDR(DT_NODELABEL(ns)))
#define FB1(i) ((uint8_t *)(DT_REG_ADDR(DT_NODELABEL(ns)) + 0x80000))

#else
#define FRAME_BUFFER_SECTION __alif_ns_section

#define ALLOCATE_FB0(i)                                                                            \
	IF_ENABLED(                                                                                \
		DT_INST_PROP(i, enable_l1),                                                        \
		(static uint8_t FRAME_BUFFER_SECTION fb0_##i                                       \
			 [CDC200_PIXEL_SIZE_LAYER(i, 1) *                                          \
			  (COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_x1_l1),                        \
				       (DT_INST_PROP(i, win_x1_l1)), (DT_INST_PROP(i, width))) -   \
			   DT_INST_PROP(i, win_x0_l1)) *                                           \
			  (COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_y1_l1),                        \
				       (DT_INST_PROP(i, win_y1_l1)), (DT_INST_PROP(i, height))) -  \
			   DT_INST_PROP(i, win_y0_l1))]))

#define ALLOCATE_FB1(i)                                                                            \
	IF_ENABLED(                                                                                \
		DT_INST_PROP(i, enable_l2),                                                        \
		(static uint8_t FRAME_BUFFER_SECTION fb1_##i                                       \
			 [CDC200_PIXEL_SIZE_LAYER(i, 2) *                                          \
			  (COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_x1_l2),                        \
				       (DT_INST_PROP(i, win_x1_l2)), (DT_INST_PROP(i, width))) -   \
			   DT_INST_PROP(i, win_x0_l2)) *                                           \
			  (COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_y1_l2),                        \
				       (DT_INST_PROP(i, win_y1_l2)), (DT_INST_PROP(i, height))) -  \
			   DT_INST_PROP(i, win_y0_l2))]))

#define FB0(i) COND_CODE_1(DT_INST_PROP(i, enable_l1), (fb0_##i), (NULL))
#define FB1(i) COND_CODE_1(DT_INST_PROP(i, enable_l2), (fb1_##i), (NULL))
#endif

#else
#if defined(NO_RELOCATE_SRAM0)
#define ALLOCATE_FB0(i)
#define ALLOCATE_FB1(i)
#define FB0(i) ((uint8_t *)0x02000000)
#define FB1(i) ((uint8_t *)0x02177000)

#else
#define FRAME_BUFFER_SECTION __alif_sram0_section

#define ALLOCATE_FB0(i)                                                                            \
	IF_ENABLED(                                                                                \
		DT_INST_PROP(i, enable_l1),                                                        \
		(static uint8_t FRAME_BUFFER_SECTION fb0_##i                                       \
			 [CDC200_PIXEL_SIZE_LAYER(i, 1) *                                          \
			  (COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_x1_l1),                        \
				       (DT_INST_PROP(i, win_x1_l1)), (DT_INST_PROP(i, width))) -   \
			   DT_INST_PROP(i, win_x0_l1)) *                                           \
			  (COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_y1_l1),                        \
				       (DT_INST_PROP(i, win_y1_l1)), (DT_INST_PROP(i, height))) -  \
			   DT_INST_PROP(i, win_y0_l1))]))

#define ALLOCATE_FB1(i)                                                                            \
	IF_ENABLED(                                                                                \
		DT_INST_PROP(i, enable_l2),                                                        \
		(static uint8_t FRAME_BUFFER_SECTION fb1_##i                                       \
			 [CDC200_PIXEL_SIZE_LAYER(i, 2) *                                          \
			  (COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_x1_l2),                        \
				       (DT_INST_PROP(i, win_x1_l2)), (DT_INST_PROP(i, width))) -   \
			   DT_INST_PROP(i, win_x0_l2)) *                                           \
			  (COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_y1_l2),                        \
				       (DT_INST_PROP(i, win_y1_l2)), (DT_INST_PROP(i, height))) -  \
			   DT_INST_PROP(i, win_y0_l2))]))
#define FB0(i) COND_CODE_1(DT_INST_PROP(i, enable_l1), (fb0_##i), (NULL))

#define FB1(i) COND_CODE_1(DT_INST_PROP(i, enable_l2), (fb1_##i), (NULL))
#endif
#endif

#define FB0_SIZE(i)                                                                                \
	CDC200_PIXEL_SIZE_LAYER(i, 1) *                                                            \
		(COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_x1_l1), (DT_INST_PROP(i, win_x1_l1)),    \
			     (DT_INST_PROP(i, width))) -                                           \
		 DT_INST_PROP(i, win_x0_l1)) *                                                     \
		(COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_y1_l1), (DT_INST_PROP(i, win_y1_l1)),    \
			     (DT_INST_PROP(i, height))) -                                          \
		 DT_INST_PROP(i, win_y0_l1)) *                                                     \
		DT_INST_PROP(i, enable_l1)

#define FB1_SIZE(i)                                                                                \
	CDC200_PIXEL_SIZE_LAYER(i, 2) *                                                            \
		(COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_x1_l2), (DT_INST_PROP(i, win_x1_l2)),    \
			     (DT_INST_PROP(i, width))) -                                           \
		 DT_INST_PROP(i, win_x0_l2)) *                                                     \
		(COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_y1_l2), (DT_INST_PROP(i, win_y1_l2)),    \
			     (DT_INST_PROP(i, height))) -                                          \
		 DT_INST_PROP(i, win_y0_l2)) *                                                     \
		DT_INST_PROP(i, enable_l2)

/********************** PinCtrl define. **********************/
#ifdef CONFIG_MIPI_DSI
#define CDC200_PINCTRL_INIT(n)
#define CDC200_PINCTRL_GET(n) (NULL)
#else
#define CDC200_PINCTRL_INIT(n) PINCTRL_DT_INST_DEFINE(n)
#define CDC200_PINCTRL_GET(n)  PINCTRL_DT_INST_DEV_CONFIG_GET(n)
#endif

#define CDC200_GET_CLK(i)                                                                         \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(i, clocks),                                              \
		(.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(i)),                                \
		 .dpi_cid = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(i,                \
			 dpi_clk_en, clkid),                                                      \
		 .pix_cid = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(i,                \
			 pixel_clk, clkid),))                                                     \

/********************** Device Definition per instance Macros. ***********************/
#define CDC200_DEFINE(i)                                                                           \
	CDC200_PINCTRL_INIT(i);                                                                    \
	static void cdc200_config_func_##i(const struct device *dev);                              \
                                                                                                   \
	ALLOCATE_FB0(i);                                                                           \
	ALLOCATE_FB1(i);                                                                           \
	static const struct cdc200_config config##i = {                                            \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(i)),                                              \
                                                                                                   \
		.irq = DT_INST_IRQ_BY_NAME(i, scanline_0, irq),                                    \
		.irq_config_func = cdc200_config_func_##i,                                         \
		.pcfg = CDC200_PINCTRL_GET(i),                                                     \
		.bg_color = DT_INST_PROP(i, bg_color),                                             \
                                                                                                   \
		CDC200_GET_CLK(i)                                                                  \
                                                                                                   \
		.panel_cfg =                                                                       \
			{                                                                          \
				.polarity = COND_CODE_1(DT_INST_PROP(i, hsync_active),             \
							CDC_GLB_CTRL_HSPOL, (0)) |                 \
					    COND_CODE_1(DT_INST_PROP(i, vsync_active),             \
							CDC_GLB_CTRL_VSPOL, (0)) |                 \
					    COND_CODE_0(DT_INST_PROP(i, de_active),                \
							CDC_GLB_CTRL_BLPOL, (0)) |                 \
					    COND_CODE_1(DT_INST_PROP(i, pixelclk_active),          \
							CDC_GLB_CTRL_PCLKPOL, (0)),                \
                                                                                                   \
				.hsync_len = DT_INST_PROP(i, hsync_len),                           \
				.vsync_len = DT_INST_PROP(i, vsync_len),                           \
				                                                                   \
				.hbp = DT_INST_PROP(i, hback_porch),                               \
				.vbp = DT_INST_PROP(i, vback_porch),                               \
				                                                                   \
				.active_width = DT_INST_PROP(i, width),                            \
				.active_height = DT_INST_PROP(i, height),                          \
				                                                                   \
				.hfp = DT_INST_PROP(i, hfront_porch),                              \
				.vfp = DT_INST_PROP(i, vfront_porch),                              \
			},                                                                         \
		.layer[0] =                                                                        \
			{                                                                          \
				.layer_en = DT_INST_PROP(i, enable_l1),                            \
				.fb_addr = FB0(i),                                                 \
				.fb_size = FB0_SIZE(i),                                            \
				.fmt = DT_INST_ENUM_IDX_OR(i, pixel_fmt_l1,                        \
							   CDC_PIXEL_FORMAT_UNDEFINED),            \
				.pixel_size = CDC200_PIXEL_SIZE_LAYER(i, 1),                       \
				.def_color_en = DT_INST_NODE_HAS_PROP(i, def_back_color_l1),       \
				.def_color =                                                       \
					COND_CODE_1(DT_INST_NODE_HAS_PROP(i, def_back_color_l1),   \
						    (DT_INST_PROP(i, def_back_color_l1)), (0)),    \
				.x0 = DT_INST_PROP(i, win_x0_l1),                                  \
				.y0 = DT_INST_PROP(i, win_y0_l1),                                  \
				.x1 = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_x1_l1),             \
						  (DT_INST_PROP(i, win_x1_l1)),                    \
						  (DT_INST_PROP(i, width))),                       \
				.y1 = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_y1_l1),             \
						  (DT_INST_PROP(i, win_y1_l1)),                    \
						  (DT_INST_PROP(i, height))),                      \
				.const_alpha = (uint8_t)DT_INST_PROP(i, const_alpha_l1),           \
				.curr_layer_bf_f1 = DT_INST_PROP(i, blend_factor1_l1),             \
				.subj_layer_bf_f2 = DT_INST_PROP(i, blend_factor2_l1),             \
			},                                                                         \
		.layer[1] =                                                                        \
			{                                                                          \
				.layer_en = DT_INST_PROP(i, enable_l2),                            \
				.fb_addr = FB1(i),                                                 \
				.fb_size = FB1_SIZE(i),                                            \
				.fmt = DT_INST_ENUM_IDX_OR(i, pixel_fmt_l2,                        \
							   CDC_PIXEL_FORMAT_UNDEFINED),            \
				.pixel_size = CDC200_PIXEL_SIZE_LAYER(i, 2),                       \
				.def_color_en = DT_INST_NODE_HAS_PROP(i, def_back_color_l2),       \
				.def_color =                                                       \
					COND_CODE_1(DT_INST_NODE_HAS_PROP(i, def_back_color_l2),   \
						    (DT_INST_PROP(i, def_back_color_l2)), (0)),    \
				.x0 = DT_INST_PROP(i, win_x0_l2),                                  \
				.y0 = DT_INST_PROP(i, win_y0_l2),                                  \
				.x1 = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_x1_l2),             \
						  (DT_INST_PROP(i, win_x1_l2)),                    \
						  (DT_INST_PROP(i, width))),                       \
				.y1 = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, win_y1_l2),             \
						  (DT_INST_PROP(i, win_y1_l2)),                    \
						  (DT_INST_PROP(i, height))),                      \
				.const_alpha = (uint8_t)DT_INST_PROP(i, const_alpha_l2),           \
				.curr_layer_bf_f1 = DT_INST_PROP(i, blend_factor1_l2),             \
				.subj_layer_bf_f2 = DT_INST_PROP(i, blend_factor2_l2),             \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	static struct cdc200_data data##i = {                                                      \
		.layer_count = DT_INST_PROP(i, enable_l1) + DT_INST_PROP(i, enable_l2),            \
		.curr_fb[0] = FB0(i),                                                              \
		.curr_fb[1] = FB1(i),                                                              \
		.next_fb[0] = FB0(i),                                                              \
		.next_fb[1] = FB1(i),                                                              \
		.clk_freq = DT_INST_PROP_OR(i, clock_frequency, 0),                                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(i, cdc200_init, NULL, &data##i, &config##i, POST_KERNEL,             \
			      CONFIG_DISPLAY_INIT_PRIORITY, &cdc200_display_api);                  \
                                                                                                   \
	static void cdc200_config_func_##i(const struct device *dev)                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, scanline_0, irq),                               \
			    DT_INST_IRQ_BY_NAME(i, scanline_0, priority), cdc200_isr,              \
			    DEVICE_DT_INST_GET(i), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(i, scanline_0, irq));                               \
	}

DT_INST_FOREACH_STATUS_OKAY(CDC200_DEFINE);
