/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/video.h>

#ifndef __ZEPHYR_INCLUDE_DRIVERS_ISP_VSI_H__
#define __ZEPHYR_INCLUDE_DRIVERS_ISP_VSI_H__

#ifdef __cplusplus
extern "C" {
#endif

enum tpg_image_type {
	IMG_3X3_COLOR_BLOCK = 0,
	IMG_COLOR_BAR,
	IMG_GRAY_BAR,
	IMG_HIGHLIGHTED_GRID,
	IMG_RANDOM_GENERATOR,
	IMG_DISABLED,
};

enum input_type {
	INPUT_SENSOR,
	INPUT_TPG,
};

enum hdr_mode {
	LINEAR,
	STITCH,
	SENSOR_STITCH,
};

enum ccir_seq {
	YCBYCR = 0,
	YCRYCB,
	CBYCRY,
	CRYCBY,
};

enum isp_subsampling {
	COSITED,
	INTER,
	NON_COSITED,
};

enum field_sampling {
	BOTH,
	EVEN,
	ODD,
};

enum transmission_bus_mode {
	ONLINE,
	FLEXA,
	DMA,
};

struct rect {
	uint32_t top;
	uint32_t left;
	uint32_t width;
	uint32_t height;
};

struct port_parameters {
	enum input_type input;
	uint8_t tpg_image_idx;
	struct video_format port_fmt;
	enum hdr_mode hdr;
	enum ccir_seq seq;
	enum isp_subsampling mode;
	enum field_sampling field;
	struct rect sns_rect;
	uint32_t snsfps;
	struct rect in_form_rect;
	struct rect image_stabilization_rect;
	struct rect out_form_rect;
	uint8_t isp_idx;
	uint8_t port_id;
};

struct channel_parameters {
	enum transmission_bus_mode trans_bus;
	struct video_format output_fmt;
	uint8_t channel_idx;
};

struct isp_config_params {
	struct port_parameters port;
	struct channel_parameters channel;
};

int isp_vsi_init(struct isp_config_params *init_cfg);
int isp_vsi_update_cfg(struct isp_config_params *init_cfg);
int isp_vsi_uninit(struct isp_config_params *init_cfg);
void isp_vsi_bottom_half(struct isp_config_params *init_cfg, uint32_t mi_mis);
int isp_vsi_start(struct isp_config_params *init_cfg);
int isp_vsi_stop(struct isp_config_params *init_cfg);
int isp_vsi_enqueue(struct isp_config_params *init_cfg,
		struct video_buffer *buf);
int isp_vsi_dequeue(struct isp_config_params *init_cfg,
		struct video_buffer *buf);

#ifdef __cplusplus
}
#endif

#endif /* __ZEPHYR_INCLUDE_DRIVERS_ISP_VSI_H__ */
