/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video/isp_ctrl_params.h>

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

struct binning {
	bool enable;
	uint8_t hstep;
	uint8_t vstep;
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
	struct binning bin;
};

struct channel_parameters {
	enum transmission_bus_mode trans_bus;
	struct video_format output_fmt;
	uint8_t channel_idx;
};

/**
 * @brief Callback invoked each frame end with the current AE stability status.
 *
 * Called from interrupt bottom-half context (workqueue).
 *
 * @param dev        ISP device that generated the event.
 * @param ae_stable  1 if AE has converged, 0 otherwise.
 * @param user_data  Opaque pointer supplied at registration time.
 */
typedef void (*isp_ae_status_cb)(const struct device *dev, uint8_t ae_stable,
				  void *user_data);

struct isp_config_params {
	struct port_parameters port;
	struct channel_parameters channel;
	isp_ae_status_cb ae_status_cb;
	void *ae_status_user_data;
};

int isp_vsi_init(struct isp_config_params *init_cfg);
int isp_vsi_update_cfg(struct isp_config_params *init_cfg);
int isp_vsi_uninit(struct isp_config_params *init_cfg);
void isp_vsi_bottom_half(const struct device *dev,
		struct isp_config_params *init_cfg, uint32_t mi_mis);
int isp_vsi_start(struct isp_config_params *init_cfg);
int isp_vsi_stop(struct isp_config_params *init_cfg);
int isp_vsi_enqueue(struct isp_config_params *init_cfg,
		struct video_buffer *buf);
int isp_vsi_dequeue(struct isp_config_params *init_cfg,
		struct video_buffer *buf);

/**
 * @brief Set ISP module parameters on a live ISP port.
 *
 * Called by isp_pico.c in response to video_set_ctrl() with
 * VIDEO_CID_ALIF_ISP_SET.  Must be called after isp_vsi_init().
 * Only modules whose ISP_PARAM_MASK_* bit is set in params->valid_mask
 * are applied; all others are ignored.
 *
 * @param init_cfg  Pointer to the config used during isp_vsi_init().
 * @param params    Aggregate parameter struct with valid_mask set.
 *
 * @retval 0        Success.
 * @retval -EINVAL  init_cfg or params is NULL.
 * @retval -ENOTSUP Module not compiled in.
 * @retval <0       ISP library error mapped to errno.
 */
int isp_vsi_set_param(struct isp_config_params *init_cfg,
		      const struct isp_params *params);

/**
 * @brief Get ISP module parameters from a live ISP port.
 *
 * Called by isp_pico.c in response to video_get_ctrl() with
 * VIDEO_CID_ALIF_ISP_GET.  Must be called after isp_vsi_init().
 * Only modules whose ISP_PARAM_MASK_* bit is set in params->valid_mask
 * are read back; all others are left untouched.
 *
 * @param init_cfg  Pointer to the config used during isp_vsi_init().
 * @param params    Aggregate parameter struct with valid_mask set;
 *                  filled in on return.
 *
 * @retval 0        Success.
 * @retval -EINVAL  init_cfg or params is NULL.
 * @retval -ENOTSUP Module not compiled in.
 * @retval <0       ISP library error mapped to errno.
 */
int isp_vsi_get_param(struct isp_config_params *init_cfg,
		      struct isp_params *params);


__syscall int isp_vsi_register_ae_status_callback(const struct device *dev,
		isp_ae_status_cb ae_status_cb, void *user_data);

#include <zephyr/syscalls/isp-vsi.h>

#ifdef __cplusplus
}
#endif

#endif /* __ZEPHYR_INCLUDE_DRIVERS_ISP_VSI_H__ */
