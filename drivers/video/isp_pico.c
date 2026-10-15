/*
 * Copyright (C) 2026 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT vsi_isp_pico

#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ISP, CONFIG_VIDEO_LOG_LEVEL);

#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/drivers/pinctrl.h>

#include "isp_pico.h"
#include <zephyr/drivers/video/video_alif.h>
#include <soc_memory_map.h>
#include <zephyr/cache.h>
#include <zephyr/pm/device.h>

#define WORKQ_STACK_SIZE 4096
#define WORKQ_PRIORITY   7
K_KERNEL_STACK_DEFINE(isp_cb_workq, WORKQ_STACK_SIZE);

#define ISP_VIDEO_FORMAT_CAP(format, width, height)                                             \
	{                                                                                       \
		.pixelformat = (format), .width_min = (0), .width_max = (width),                \
		.height_min = (0), .height_max = (height), .width_step = 8, .height_step = 4,   \
	}

#define ISP_VIDEO_FIXED_FORMAT_CAP(format, width, height)                                          \
	{                                                                                          \
		.pixelformat = (format), .width_min = (width), .width_max = (width),               \
		.height_min = (height), .height_max = (height), .width_step = 0, .height_step = 0, \
	}

static const struct video_format_cap supported_input_fmts[] = {
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_BGGR8, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GBRG8, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GRBG8, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_RGGB8, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_BGGR10, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GBRG10, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GRBG10, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_RGGB10, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_BGGR12, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GBRG12, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GRBG12, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_RGGB12, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GREY, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_Y10P, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_YUYV, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_YVYU, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_UYVY, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_VYUY, 1920, 1080),
	{ 0 },
};

static const struct video_format_cap supported_tpg_fmts[] = {
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_BGGR8, 1280, 720),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GBRG8, 1280, 720),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GRBG8, 1280, 720),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_RGGB8, 1280, 720),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_BGGR10, 1280, 720),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GBRG10, 1280, 720),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GRBG10, 1280, 720),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_RGGB10, 1280, 720),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_BGGR12, 1280, 720),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GBRG12, 1280, 720),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GRBG12, 1280, 720),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_RGGB12, 1280, 720),

	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_BGGR8, 1920, 1080),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GBRG8, 1920, 1080),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GRBG8, 1920, 1080),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_RGGB8, 1920, 1080),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_BGGR10, 1920, 1080),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GBRG10, 1920, 1080),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GRBG10, 1920, 1080),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_RGGB10, 1920, 1080),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_BGGR12, 1920, 1080),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GBRG12, 1920, 1080),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GRBG12, 1920, 1080),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_RGGB12, 1920, 1080),

	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_BGGR8, 3840, 2160),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GBRG8, 3840, 2160),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GRBG8, 3840, 2160),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_RGGB8, 3840, 2160),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_BGGR10, 3840, 2160),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GBRG10, 3840, 2160),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GRBG10, 3840, 2160),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_RGGB10, 3840, 2160),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_BGGR12, 3840, 2160),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GBRG12, 3840, 2160),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_GRBG12, 3840, 2160),
	ISP_VIDEO_FIXED_FORMAT_CAP(VIDEO_PIX_FMT_RGGB12, 3840, 2160),
	{ 0 },
};

static const struct video_format_cap supported_output_fmts[] = {
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_BGGR8, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GBRG8, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GRBG8, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_RGGB8, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_BGGR10, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GBRG10, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GRBG10, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_RGGB10, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_BGGR12, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GBRG12, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GRBG12, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_RGGB12, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_NV12, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_NV16, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_YUV422P, 19200, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_YUV420, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_YUYV, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GREY, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_Y10, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_Y12, 1920, 1080),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_RGB888_PLANAR_PRIVATE, 1920, 1080),
	{ 0 },
};

static int get_format_cap(uint32_t fourcc_fmt,
		const struct video_format_cap supported_fmts[])
{
	for (int i = 0; supported_fmts[i].pixelformat; i++) {
		if (fourcc_fmt == supported_fmts[i].pixelformat) {
			return i;
		}
	}

	return -1;
}

static int find_format(struct video_format *fmt,
		const struct video_format_cap supported_fmts[])
{
	for (int i = 0; supported_fmts[i].pixelformat; i++) {
		if (fmt->pixelformat == supported_fmts[i].pixelformat &&
		    fmt->width >= supported_fmts[i].width_min &&
		    fmt->width <= supported_fmts[i].width_max &&
		    fmt->height >= supported_fmts[i].height_min &&
		    fmt->height <= supported_fmts[i].height_max) {
			/* The matching format supported by ISP is found. */
			return 0;
		}
	}

	return -ENOTSUP;
}

static void hw_disable_mi_interrupts(uintptr_t regs, uint32_t mask)
{
	sys_clear_bits(regs + ISP_MI_IMSC, mask);
}

static void isp_bottom_half(const struct device *dev)
{
	struct isp_data *data = dev->data;

	/*
	 * The finished buffer was already moved to the library done list in
	 * isp_isr_handler() via isp_vsi_mi_irq(). This work item only runs
	 * the frame-end path (AE and the port callback).
	 */
	isp_vsi_bottom_half(dev, &data->init_cfg, data->mi_mis);

#if defined(CONFIG_POLL)
	if (data->signal) {
		k_poll_signal_raise(data->signal, VIDEO_BUF_DONE);
	}
#endif /* defined(CONFIG_POLL) */
}

static void isp_cb_work(struct k_work *work)
{
	struct isp_data *data = CONTAINER_OF(work, struct isp_data, cb_work);

	/* Call a helper to process the things further. */
	isp_bottom_half(data->dev);
}

static void isp_isr_handler(const struct device *dev)
{
	struct isp_data *data = dev->data;

	uint32_t isp_intr_err_mask = INTR_SIZE_ERR | INTR_DATALOSS;
	static bool is_not_corrupted_frame = true;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	uint32_t mi_int_st;
	uint32_t int_st;

	int_st = sys_read32(regs + ISP_MIS);
	sys_write32(int_st, regs + ISP_ICR);

	mi_int_st = sys_read32(regs + ISP_MI_MIS);
	sys_write32(mi_int_st, regs + ISP_MI_ICR);

	data->mi_mis = mi_int_st;

	if (int_st & INTR_EXP_END) {
		LOG_DBG("Exposure measurement complete.");
	}

	if (int_st & INTR_H_START) {
		LOG_DBG("H-Sync detected");
	}

	if (int_st & INTR_V_START) {
		LOG_DBG("V-Sync detected");
	}

	if (int_st & INTR_FRAME_IN) {
		LOG_DBG("Sampled Input frame is complete.");
	}

	if (int_st & INTR_AWB_DONE) {
		LOG_DBG("White balancing measurement complete");
	}

	if (int_st & INTR_SIZE_ERR) {
		LOG_ERR("Picture size violation occurred; incorrect programming");
	}

	if (int_st & INTR_DATALOSS) {
		LOG_ERR("Loss of data within a line; processing failure");
	}

	if (int_st & isp_intr_err_mask) {
		LOG_ERR("Frame capture error. int_st - 0x%08x", int_st);
		is_not_corrupted_frame = false;
#if defined(CONFIG_POLL)
		if (data->signal) {
			k_poll_signal_raise(data->signal, VIDEO_BUF_ERROR);
		}
#endif /* defined(CONFIG_POLL) */
	}

	if (mi_int_st & MI_INTR_WRAP_MP_CR) {
		LOG_DBG("Main picture Cr address wrap");
	}

	if (mi_int_st & MI_INTR_WRAP_MP_CB) {
		LOG_DBG("Main picture Cb address wrap");
	}

	if (mi_int_st & MI_INTR_WRAP_MP_Y) {
		LOG_DBG("Main picture Y address wrap");
	}

	if (mi_int_st & MI_INTR_FILL_MP_Y) {
		LOG_DBG("Main picture fill level interrupt");
	}

	if (mi_int_st & MI_INTR_MBLK_LINE) {
		LOG_DBG("Main picture Macro block line interrupt");
	}

	if (mi_int_st & MI_INTR_MP_FRAME_END) {
		LOG_DBG("End of Frame at MI interface of Main picture.");
		/*
		 * Retire the buffer here, not in the work item: both ISP IRQ
		 * lines share this handler and would overwrite data->mi_mis
		 * before the work item runs.
		 */
		isp_vsi_mi_irq(&data->init_cfg, mi_int_st);
		if (is_not_corrupted_frame) {
			k_work_submit_to_queue(&data->cb_workq, &data->cb_work);
		} else {
			is_not_corrupted_frame = true;
		}
	}
}

int isp_set_fmt(const struct device *dev,
		enum video_endpoint_id ep,
		struct video_format *fmt)
{
	const struct isp_config *config = dev->config;
	struct isp_data *data = dev->data;

	struct channel_parameters *channel = &data->init_cfg.channel;
	struct port_parameters *port = &data->init_cfg.port;
	int ret = -ENODEV;

	if (!fmt) {
		LOG_ERR("Illegal format to set!");
		return -EINVAL;
	}

	switch (ep) {
	case VIDEO_EP_IN:
#ifdef CONFIG_PM_DEVICE
		if (!data->needs_reinit &&
		    !memcmp(fmt, &port->port_fmt, sizeof(*fmt))) {
#else
		if (!memcmp(fmt, &port->port_fmt, sizeof(*fmt))) {
#endif
			/* Nothing to do */
			return 0;
		}

		ret = find_format(fmt, supported_input_fmts);
		if (ret) {
			LOG_ERR("Desired format is not supported by the ISP Input EP!");
			return ret;
		}

		ret = video_set_format(config->controller, VIDEO_EP_OUT, fmt);
		if (ret) {
			LOG_ERR("Failed to set desired format on camera pipeline!");
			return ret;
		}

		/* Cache the desired input format. */
		port->port_fmt = *fmt;
#ifdef CONFIG_PM_DEVICE
		data->needs_reinit = false;
#endif
		break;
	case VIDEO_EP_OUT:
		if (!memcmp(fmt, &channel->output_fmt, sizeof(*fmt))) {
			/* Nothing to do */
			return 0;
		}

		ret = find_format(fmt, supported_output_fmts);
		if (ret) {
			LOG_ERR("Desired format is not supported by the ISP Output EP!");
			return ret;
		}

		channel->output_fmt = *fmt;
		break;
	case VIDEO_EP_ALL:
#ifdef CONFIG_PM_DEVICE
		if (data->needs_reinit || memcmp(fmt, &port->port_fmt, sizeof(*fmt))) {
#else
		if (memcmp(fmt, &port->port_fmt, sizeof(*fmt))) {
#endif
			ret = find_format(fmt, supported_input_fmts);
			if (ret) {
				LOG_ERR("Desired format is not supported by the ISP Input EP!");
				return ret;
			}

			ret = video_set_format(config->controller, VIDEO_EP_OUT, fmt);
			if (ret) {
				LOG_ERR("Failed to set desired format on camera pipeline!");
				return ret;
			}

			/* Cache the desired input format. */
			port->port_fmt = *fmt;
#ifdef CONFIG_PM_DEVICE
			data->needs_reinit = false;
#endif
		}

		if (memcmp(fmt, &channel->output_fmt, sizeof(*fmt))) {
			ret = find_format(fmt, supported_output_fmts);
			if (ret) {
				LOG_ERR("Desired format is not supported by the ISP Output EP!");
				return ret;
			}

			/* Cache the desired output format. */
			channel->output_fmt = *fmt;
		}
		break;
	default:
		LOG_ERR("Unsupported Endpoint!");
		return -EINVAL;

	}

	return 0;
}

int isp_get_fmt(const struct device *dev,
		enum video_endpoint_id ep,
		struct video_format *fmt)
{
	const struct isp_config *config = dev->config;
	struct isp_data *data = dev->data;

	struct channel_parameters *channel = &data->init_cfg.channel;
	struct port_parameters *port = &data->init_cfg.port;
	int ret;

	if (!fmt) {
		return -EINVAL;
	}

	switch (ep) {
	case VIDEO_EP_IN:
		if (!port->port_fmt.pixelformat) {
			ret = video_get_format(config->controller, VIDEO_EP_OUT, fmt);
			if (ret) {
				return ret;
			}

			ret = find_format(fmt, supported_input_fmts);
			if (ret) {
				LOG_ERR("Pipeline running on unsupported format by ISP!");
				return ret;
			}

			port->port_fmt = *fmt;
		}

		*fmt = port->port_fmt;
		break;
	case VIDEO_EP_OUT:
		if (!channel->output_fmt.pixelformat) {
			uint32_t tmp_fmt = VIDEO_PIX_FMT_RGB888_PLANAR_PRIVATE;
			int i;

			i = get_format_cap(tmp_fmt, supported_output_fmts);
			if (i == -1) {
				LOG_ERR("Failed to set output format for ISP!");
				return -EINVAL;
			}

			/*
			 * If input format is also not set, use
			 * RGB888 planar output format.
			 */
			channel->output_fmt.pixelformat =
				supported_output_fmts[i].pixelformat;
			channel->output_fmt.height =
				supported_output_fmts[i].height_max;
			channel->output_fmt.width =
				supported_output_fmts[i].width_max;
			channel->output_fmt.pitch =
				(video_bits_per_pixel(tmp_fmt) *
				 channel->output_fmt.width) >> 3;
		}

		*fmt = channel->output_fmt;
		break;
	default:
		LOG_ERR("Unsupported endpoint ID!");
		return -EINVAL;
	}
	return 0;
}

static int isp_stream_start(const struct device *dev)
{
	const struct isp_config *config = dev->config;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	struct isp_data *data = dev->data;

	struct port_parameters *port = &data->init_cfg.port;
	uint32_t tmp;
	int ret;

	/* Cancel any stale work from previous session before starting */
	struct k_work_sync sync;

	/* Ensure MI frame-end interrupt is unmasked (may have been cleared by flush) */
	sys_set_bits(regs + ISP_MI_IMSC, MI_INTR_MP_FRAME_END);

	k_work_cancel_sync(&data->cb_work, &sync);

	if (data->is_streaming) {
		LOG_DBG("Already streaming");
		return -EBUSY;
	}

	if (!isp_vsi_has_buffer()) {
		LOG_ERR("No buffer queued. Can't start streaming!");
		return -ENOBUFS;
	}

	/* Update ISP configuration to the middleware */
	switch (port->port_fmt.pixelformat) {
	case VIDEO_PIX_FMT_YUYV:
		port->seq = YCBYCR;
		break;
	case VIDEO_PIX_FMT_YVYU:
		port->seq = YCRYCB;
		break;
	case VIDEO_PIX_FMT_VYUY:
		port->seq = CRYCBY;
		break;
	case VIDEO_PIX_FMT_UYVY:
		port->seq = CBYCRY;
		break;
	}

	port->sns_rect.width = port->port_fmt.width;
	port->sns_rect.height = port->port_fmt.height;

	port->in_form_rect.width = port->port_fmt.width;
	port->in_form_rect.height = port->port_fmt.height;

	port->image_stabilization_rect.top = port->in_form_rect.top;
	port->image_stabilization_rect.left = port->in_form_rect.left;
	port->image_stabilization_rect.width = port->in_form_rect.width;
	port->image_stabilization_rect.height = port->in_form_rect.height;

	port->out_form_rect.width = port->port_fmt.width - (port->out_form_rect.left << 1);
	port->out_form_rect.height = port->port_fmt.height - (port->out_form_rect.top << 1);

	ret = isp_vsi_update_cfg(&data->init_cfg);
	if (ret) {
		LOG_ERR("Failed to update ISP config to input/output formats and ROI!");
		data->curr_vid_buf = 0;
		return ret;
	}

	tmp = sys_read32(regs + ISP_ACQ_PROP);
	tmp &= ~(ACQ_PROP_PIN_MAPPING_MASK << ACQ_PROP_PIN_MAPPING_SHIFT);

	switch (pix_fmt_bpp(port->port_fmt.pixelformat)) {
	case 10:
		tmp |= (1 << ACQ_PROP_PIN_MAPPING_SHIFT);
		break;
	case 8:
		tmp |= (2 << ACQ_PROP_PIN_MAPPING_SHIFT);
		break;
	case 12:
	default:
		tmp |= (0 << ACQ_PROP_PIN_MAPPING_SHIFT);
		break;
	}
	sys_write32(tmp, regs + ISP_ACQ_PROP);

	/* Set is_streaming BEFORE starting hardware to prevent
	 * bottom_half from stopping CPI mid-start
	 */
	data->is_streaming = true;

	ret = isp_vsi_start(&data->init_cfg);
	if (ret) {
		LOG_ERR("Failed to start stream!");
		data->is_streaming = false;
		return ret;
	}

	ret = video_stream_start(config->controller);
	if (ret) {
		int stop_ret;

		LOG_ERR("Failed to start stream for Endpoint device: %s!",
				config->controller->name);
		data->is_streaming = false;
		stop_ret = isp_vsi_stop(&data->init_cfg);
		if (stop_ret) {
			LOG_ERR("Failed to stop ISP device streaming");
		}
		return ret;
	}

	return 0;
}

static void isp_drain_done(struct isp_data *data)
{
	struct channel_parameters *channel = &data->init_cfg.channel;
	struct video_buffer *vbuf;
	uint32_t index;

	while (isp_vsi_dequeue(&data->init_cfg, &index) == 0) {
		vbuf = isp_vsi_buffer_by_index(index);
		if (vbuf == NULL) {
			continue;
		}

		vbuf->timestamp = k_uptime_get_32();
		vbuf->bytesused = channel->output_fmt.pitch *
				  channel->output_fmt.height;
		k_fifo_put(&data->fifo_out, vbuf);
	}
}

static void isp_release_held(struct isp_data *data, bool aborted)
{
	struct video_buffer *vbuf;

	while ((vbuf = isp_vsi_reclaim_held()) != NULL) {
		k_fifo_put(&data->fifo_out, vbuf);
		if (!aborted) {
			continue;
		}

		LOG_DBG("Video buffer aborted: 0x%x",
			(uint32_t)vbuf->buffer);
#if defined(CONFIG_POLL)
		if (data->signal) {
			k_poll_signal_raise(data->signal, VIDEO_BUF_ABORTED);
		}
#endif
	}
}

static int isp_stream_stop(const struct device *dev)
{
	const struct isp_config *config = dev->config;
	struct isp_data *data = dev->data;
	int ret;

	if (!data->is_streaming) {
		LOG_DBG("Already stopped streaming!");
		return 0;
	}

	ret = video_stream_stop(config->controller);
	if (ret) {
		LOG_ERR("Failed to stop streaming in pipeline!");
		return ret;
	}

	/* Pull finished frames out before StreamOff discards doneList. */
	isp_drain_done(data);

	ret = isp_vsi_stop(&data->init_cfg);
	if (ret) {
		LOG_ERR("Failed to stop ISP from streaming!");
		return ret;
	}

	isp_release_held(data, true);
	data->curr_vid_buf = 0;
	data->is_streaming = false;

	return 0;
}

static int isp_set_stream(const struct device *dev, bool enable)
{
	if (enable) {
		return isp_stream_start(dev);
	} else {
		return isp_stream_stop(dev);
	}
}

static int isp_get_caps(const struct device *dev,
		enum video_endpoint_id ep,
		struct video_caps *caps)
{
	const struct isp_config *config = dev->config;
	int err = -ENODEV;

	if (ep == VIDEO_EP_OUT) {
		caps->format_caps = supported_output_fmts;
	} else if (ep == VIDEO_EP_IN) {
		if (config->controller) {
			/*
			 * Camera controlled output EP should have same fmt as
			 * ISP input EP.
			 */
			err = video_get_caps(config->controller, VIDEO_EP_OUT, caps);
			if (err) {
				LOG_ERR("Failed to get caps from camera-controller!");
				return err;
			}
		} else if (config->tpg_img_idx != IMG_DISABLED) {
			/* When TPG is enabled! */
			caps->format_caps = supported_tpg_fmts;
		} else {
			/* Neither TPG nor Camera controller is enabled. */
			return -EINVAL;
		}
	} else {
		return -ENOTSUP;
	}

	caps->min_vbuf_count = ISP_MIN_VBUF;

	return 0;
}

static int isp_flush(const struct device *dev, enum video_endpoint_id ep, bool cancel)
{
	const struct isp_config *config = dev->config;
	struct isp_data *data = dev->data;

	uintptr_t regs = DEVICE_MMIO_GET(dev);
	int ret;

	/*
	 * Enqueue parks buffers in the ISP library, tracked by isp_vb_held[].
	 * fifo_in is no longer the incoming queue. Finished frames are pulled
	 * from the library done list first. Whatever is still held after the
	 * library drops its lists is returned through fifo_out.
	 */
	if (cancel && data->is_streaming) {
		hw_disable_mi_interrupts(regs, MI_INTR_MP_FRAME_END);

		for (int i = 0; (i < 20) &&
				(sys_read32(regs + ISP_MI_RIS) & MI_INTR_MP_FRAME_END); i++) {
			k_msleep(10);
		}

		if (sys_read32(regs + ISP_MI_RIS) & MI_INTR_MP_FRAME_END) {
			LOG_ERR("Failed to observe frame end!");
			return -EBUSY;
		}
	}

	if (cancel || !data->is_streaming) {
		isp_drain_done(data);

		if (data->is_streaming) {
			ret = isp_vsi_stop(&data->init_cfg);
			if (ret) {
				LOG_ERR("Failed to stop ISP device!");
				return ret;
			}
		} else if (isp_vsi_has_buffer()) {
			(void)isp_vsi_detach_buffers(&data->init_cfg);
		}

		isp_release_held(data, cancel);
		data->curr_vid_buf = 0;
		data->is_streaming = false;
	}

	video_flush(config->controller, ep, cancel);

	return 0;
}

static int isp_enqueue(const struct device *dev, enum video_endpoint_id ep,
		       struct video_buffer *buf)
{
	struct isp_data *data = dev->data;
	uint32_t tmp;
	int ret;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	/* Check if the buffer is 8-byte aligned or not */
	tmp = (uint32_t)buf->buffer;
	if (ROUND_UP(tmp, 8) != tmp) {
		LOG_ERR("Video Buffer is not aligned to 8-byte boundary."
			"It can result in corruption of captured image.");
		return -ENOBUFS;
	}

	buf->bytesused = 0;

	ret = isp_vsi_enqueue(&data->init_cfg, buf);
	if (ret) {
		LOG_ERR("Failed to enqueue buffer to ISP library: %d", ret);
		return ret;
	}

	LOG_DBG("Enqueued buffer: Addr - 0x%x, size - %d, bytesused - %d",
		(uint32_t)buf->buffer, buf->size, buf->bytesused);

	(void)sys_cache_data_flush_and_invd_range(buf->buffer, buf->size);

	return 0;
}

static int isp_dequeue(const struct device *dev, enum video_endpoint_id ep,
		       struct video_buffer **buf, k_timeout_t timeout)
{
	struct isp_data *data = dev->data;
	struct channel_parameters *channel = &data->init_cfg.channel;
	k_timepoint_t end = sys_timepoint_calc(timeout);
	uint32_t index;
	int ret;

	if (!buf || (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL)) {
		return -EINVAL;
	}

	/*
	 * Flush, stop, and suspend park returned buffers here. They are no
	 * longer on the library lists.
	 */
	*buf = k_fifo_get(&data->fifo_out, K_NO_WAIT);
	if (*buf != NULL) {
		return 0;
	}

	/*
	 * The library wait is compiled out, so an empty doneList returns
	 * immediately. Retry until a frame end queues one, or the timeout
	 * expires.
	 */
	for (;;) {
		ret = isp_vsi_dequeue(&data->init_cfg, &index);
		if (!ret) {
			break;
		}
		if (ret != -ENOBUFS) {
			LOG_ERR("Failed to dequeue ISP buffer: %d",
				ret);
			return ret;
		}
		if (!data->is_streaming || K_TIMEOUT_EQ(timeout, K_NO_WAIT) ||
		    sys_timepoint_expired(end)) {
			return -EAGAIN;
		}
		k_msleep(1);
	}

	*buf = isp_vsi_buffer_by_index(index);
	if (*buf == NULL) {
		LOG_ERR("Dequeued ISP index %u has no video buffer", index);
		return -EIO;
	}

	(*buf)->timestamp = k_uptime_get_32();
	(*buf)->bytesused = channel->output_fmt.pitch * channel->output_fmt.height;

	return 0;
}

static int isp_set_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	const struct isp_config *config = dev->config;
	struct isp_data *data = dev->data;

	switch (cid) {
	case VIDEO_CID_ALIF_ISP_SET:
		return isp_vsi_set_param(&data->init_cfg,
					 (const struct isp_params *)value);
	default:
		return video_set_ctrl(config->controller, cid, value);
	}
}

static int isp_get_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	const struct isp_config *config = dev->config;
	struct isp_data *data = dev->data;

	switch (cid) {
	case VIDEO_CID_ALIF_ISP_GET:
		return isp_vsi_get_param(&data->init_cfg,
					 (struct isp_params *)value);
	default:
		return video_get_ctrl(config->controller, cid, value);
	}
}

#ifdef CONFIG_POLL
static int isp_set_signal(const struct device *dev, enum video_endpoint_id ep,
		struct k_poll_signal *signal)
{
	struct isp_data *data = dev->data;

	if (signal && data->signal) {
		return -EALREADY;
	}
	data->signal = signal;

	return 0;
}
#endif /* CONFIG_POLL */

static DEVICE_API(video, isp_driver_api) = {
	.set_format = isp_set_fmt,
	.get_format = isp_get_fmt,
	.set_stream = isp_set_stream,
	.get_caps = isp_get_caps,
	.flush = isp_flush,
	.enqueue = isp_enqueue,
	.dequeue = isp_dequeue,
	.set_ctrl = isp_set_ctrl,
	.get_ctrl = isp_get_ctrl,
#ifdef CONFIG_POLL
	.set_signal = isp_set_signal,
#endif /* CONFIG_POLL */
};

int z_impl_isp_vsi_register_ae_status_callback(const struct device *dev,
		isp_ae_status_cb ae_status_cb, void *user_data)
{
	struct isp_data *data = dev->data;

	data->init_cfg.ae_status_cb = ae_status_cb;
	data->init_cfg.ae_status_user_data = user_data;

	return 0;
}

#ifdef CONFIG_USERSPACE
#include <zephyr/internal/syscall_handler.h>
static int z_vrfy_isp_vsi_register_ae_status_callback(const struct device *dev,
		isp_ae_status_cb ae_status_cb, void *user_data)
{
	K_OOPS(K_SYSCALL_SPECIFIC_DRIVER(dev, K_OBJ_DRIVER_VIDEO, &isp_driver_api));
	return z_impl_isp_vsi_register_ae_status_callback(dev, ae_status_cb, user_data);
}
#include <zephyr/syscalls/register_ae_status_callback_mrsh.c>
#endif /* CONFIG_USERSPACE */

static int isp_configure(const struct device *dev)
{
	const struct isp_config *config = dev->config;
	struct isp_data *data = dev->data;

	struct port_parameters *port = &data->init_cfg.port;
	int ret;

	ret = isp_vsi_init(&data->init_cfg);
	if (ret) {
		LOG_ERR("Failed to Init ISP device!");
		return ret;
	}

	if (config->tpg_img_idx == IMG_DISABLED) {
		port->input = INPUT_SENSOR;
	} else {
		port->input = INPUT_TPG;
		switch (config->tpg_pix_width) {
		case TPG_BIT_WIDTH_8:
			if (config->tpg_bayer_pattern == RGGB) {
				port->port_fmt.pixelformat = VIDEO_PIX_FMT_RGGB8;
			} else if (config->tpg_bayer_pattern == GRBG) {
				port->port_fmt.pixelformat = VIDEO_PIX_FMT_GRBG8;
			} else if (config->tpg_bayer_pattern == GBRG) {
				port->port_fmt.pixelformat = VIDEO_PIX_FMT_GBRG8;
			} else if (config->tpg_bayer_pattern == BGGR) {
				port->port_fmt.pixelformat = VIDEO_PIX_FMT_BGGR8;
			}
			break;
		case TPG_BIT_WIDTH_10:
			if (config->tpg_bayer_pattern == RGGB) {
				port->port_fmt.pixelformat = VIDEO_PIX_FMT_RGGB10;
			} else if (config->tpg_bayer_pattern == GRBG) {
				port->port_fmt.pixelformat = VIDEO_PIX_FMT_GRBG10;
			} else if (config->tpg_bayer_pattern == GBRG) {
				port->port_fmt.pixelformat = VIDEO_PIX_FMT_GBRG10;
			} else if (config->tpg_bayer_pattern == BGGR) {
				port->port_fmt.pixelformat = VIDEO_PIX_FMT_BGGR10;
			}
			break;
		case TPG_BIT_WIDTH_12:
			if (config->tpg_bayer_pattern == RGGB) {
				port->port_fmt.pixelformat = VIDEO_PIX_FMT_RGGB12;
			} else if (config->tpg_bayer_pattern == GRBG) {
				port->port_fmt.pixelformat = VIDEO_PIX_FMT_GRBG12;
			} else if (config->tpg_bayer_pattern == GBRG) {
				port->port_fmt.pixelformat = VIDEO_PIX_FMT_GBRG12;
			} else if (config->tpg_bayer_pattern == BGGR) {
				port->port_fmt.pixelformat = VIDEO_PIX_FMT_BGGR12;
			}
			break;
		default:
			LOG_ERR("Unknown bit width!");
			return -EINVAL;
		}
		port->tpg_image_idx = config->tpg_img_idx;
	}

	port->hdr = LINEAR;

	return 0;
}

int video_isp_init(const struct device *dev)
{
	const struct isp_config *config = dev->config;
	struct isp_data *data = dev->data;
	int ret;

	if (!config->controller && config->tpg_img_idx == IMG_DISABLED) {
		LOG_ERR("Both Camera controller and TPG are not enabled!");
		return -ENODEV;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	LOG_DBG("MMIO Address: 0x%x", (uint32_t) DEVICE_MMIO_GET(dev));
	/*
	 * Setup the ISR callback work.
	 */
	k_work_init(&data->cb_work, isp_cb_work);
	k_work_queue_init(&data->cb_workq);
	k_work_queue_start(&data->cb_workq, isp_cb_workq, K_KERNEL_STACK_SIZEOF(isp_cb_workq),
			   K_PRIO_COOP(WORKQ_PRIORITY), NULL);
	k_thread_name_set(&data->cb_workq.thread, "isp_work_helper");

	/*
	 * Setup interrupts.
	 */
	config->irq_config_func(dev);

	/*
	 * Setup FIFO for ISP driver.
	 */
	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);
	data->dev = dev;

	/*
	 * Do ISP configuration.
	 */
	ret = isp_configure(dev);
	if (ret) {
		LOG_ERR("Failed to configure the ISP!");
		return ret;
	}

	LOG_DBG("ISP IRQn: %d MI-ISP IRQn: %d", config->irqn, config->mi_irqn);

	switch (config->tpg_img_idx) {
	case IMG_3X3_COLOR_BLOCK:
		LOG_DBG("TPG Status: 3x3 Color Bar");
		break;
	case IMG_COLOR_BAR:
		LOG_DBG("TPG Status: Color Bar");
		break;
	case IMG_GRAY_BAR:
		LOG_DBG("TPG Status: Gray Bar");
		break;
	case IMG_HIGHLIGHTED_GRID:
		LOG_DBG("TPG Status: Highlighted Grid");
		break;
	case IMG_RANDOM_GENERATOR:
		LOG_DBG("TPG Status: Random Generator");
		break;
	case IMG_DISABLED:
		LOG_DBG("TPG Status: Disabled");
		break;
	default:
		LOG_DBG("Unknown TPG Image format!");
	};

	return 0;
}

#if defined(CONFIG_PM_DEVICE)

static int isp_pico_suspend(const struct device *dev)
{
	struct isp_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	struct k_work_sync sync;
	int ret;

	/* 1. Stop streaming first (needs IRQs for clean stop) */
	if (data->is_streaming) {
		ret = isp_stream_stop(dev);
		if (ret) {
			LOG_ERR("Failed to stop ISP stream during suspend: %d", ret);
			return ret;
		}
	}

	/* Mask controller IRQs. NVIC enable/disable is handled by Zephyr PM. */
	sys_write32(0, regs + ISP_IMSC);
	sys_write32(0, regs + ISP_MI_IMSC);

	/* 3. Cancel any pending bottom-half work */
	k_work_cancel_sync(&data->cb_work, &sync);

	/* 4. Properly uninit the ISP library */
	ret = isp_vsi_uninit(&data->init_cfg);
		if (ret) {
			LOG_ERR("Failed to uninitialize ISP during suspend: %d", ret);
			return ret;
		}

	/*
	 * Buffers queued before streaming never reached isp_stream_stop().
	 * Hand those back. Buffers already returned by stop stay on fifo_out.
	 */
	isp_release_held(data, true);

	/* 6. Reset driver state */
	data->is_streaming = false;
	data->curr_vid_buf = 0;

	LOG_DBG("PM: Suspended %s", dev->name);
	return 0;
}

static int isp_pico_resume(const struct device *dev)
{
	struct isp_data *data = dev->data;
	int ret;

	/* Keep cached formats so set_stream() after resume still works.
	 * Force the next set_fmt() to push them down the pipeline so
	 * CSI/D-PHY/sensor re-init after S2RAM.
	 */
	ret = isp_configure(dev);
	if (ret) {
		LOG_ERR("Failed to reconfigure ISP on resume: %d", ret);
		return ret;
	}

	data->needs_reinit = true;

	LOG_INF("PM: Resumed %s", dev->name);
	return 0;
}

static int isp_pico_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		return isp_pico_resume(dev);
	case PM_DEVICE_ACTION_SUSPEND:
		return isp_pico_suspend(dev);
	case PM_DEVICE_ACTION_TURN_OFF:
	case PM_DEVICE_ACTION_TURN_ON:
		return 0;
	default:
		return -ENOTSUP;
	}
}
#endif /* CONFIG_PM_DEVICE */

#define REMOTE_DEVICE(i, idx)	                                           \
	DT_NODE_REMOTE_DEVICE(DT_INST_ENDPOINT_BY_ID(i, idx, 0))

#define REMOTE_EP(n, pid, epid)                                            \
	DT_NODELABEL(DT_STRING_TOKEN(DT_INST_ENDPOINT_BY_ID(n, pid, epid), \
				remote_endpoint_label))

/*
 * binning-hstep and binning-vstep are DT ints, but struct isp_vsi_binning
 * stores them as uint8_t. Reject values that would truncate.
 *
 * Binning steps are only meaningful when the binning module is built into the
 * ISP library and binning is enabled for the instance, where a step of zero is
 * not a valid ratio. When binning is disabled both steps are forced to zero, so
 * any value set in the devicetree is ignored.
 */
#define ISP_BINNING_ASSERT(i)                                                                 \
	BUILD_ASSERT(!DT_INST_PROP(i, binning_en) ||                                          \
		     IS_ENABLED(CONFIG_ISP_LIB_BINNING_MODULE),                               \
		     "CONFIG_ISP_LIB_BINNING_MODULE required by binning-en on "               \
		     DT_NODE_FULL_NAME(DT_DRV_INST(i)));                                      \
	COND_CODE_1(DT_INST_PROP(i, binning_en),                                              \
		    (BUILD_ASSERT(!DT_INST_PROP(i, binning_en) ||                             \
				  IN_RANGE(DT_INST_PROP(i, binning_hstep), 1, UINT8_MAX),     \
				  "binning-hstep must fit in uint8_t (1-255) on "             \
				  DT_NODE_FULL_NAME(DT_DRV_INST(i)));                         \
		     BUILD_ASSERT(!DT_INST_PROP(i, binning_en) ||                             \
				  IN_RANGE(DT_INST_PROP(i, binning_vstep), 1, UINT8_MAX),     \
				  "binning-vstep must fit in uint8_t (1-255) on "             \
				  DT_NODE_FULL_NAME(DT_DRV_INST(i)));),                       \
		    ())

#define ISP_DEFINE(i)                                                                         \
	ISP_BINNING_ASSERT(i)                                                                 \
	static void isp_config_func_##i(const struct device *dev);                            \
	const struct isp_config isp_config_##i = {                                            \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(i)),                                         \
		.irq_config_func = isp_config_func_##i,                                       \
		.controller = DEVICE_DT_GET_OR_NULL(REMOTE_DEVICE(i, 0)),                     \
		.tpg_bayer_pattern = DT_INST_ENUM_IDX(i, tpg_bayer_pattern),                  \
		.tpg_img_idx = DT_INST_ENUM_IDX(i, tpg_image_idx),                            \
		.tpg_pix_width = DT_INST_ENUM_IDX_OR(i, tpg_pix_width, 2),                    \
		.irqn = DT_INST_IRQ_BY_NAME(i, isp, irq),                                     \
		.mi_irqn = DT_INST_IRQ_BY_NAME(i, mi_isp, irq),                               \
	};                                                                                    \
                                                                                              \
	struct isp_data isp_data_##i = {                                                      \
		.is_streaming = false,                                                        \
		.init_cfg = {                                                                 \
			.port = {                                                             \
				.mode = DT_INST_ENUM_IDX(i, isp_subsampling),                 \
				.field = DT_INST_ENUM_IDX(i, fieldsel),                       \
                                                                                              \
				.out_form_rect = {                                            \
					.top = DT_INST_PROP(i, crop_y0),                      \
					.left = DT_INST_PROP(i, crop_x0),                     \
					.width = 0,                                           \
					.height = 0,                                          \
				},                                                            \
				.isp_idx = i,                                                 \
				.port_id = 0,                                                 \
				.bin = {                                                      \
					.enable = DT_INST_PROP(i, binning_en),                \
					.hstep = COND_CODE_1(DT_INST_PROP(i, binning_en),     \
							((uint8_t)DT_INST_PROP(i,             \
								binning_hstep)),              \
							(0)),                                 \
					.vstep = COND_CODE_1(DT_INST_PROP(i, binning_en),     \
							((uint8_t)DT_INST_PROP(i,             \
								binning_vstep)),              \
							(0)),                                 \
				},                                                            \
			},                                                                    \
			.channel = {                                                          \
				.trans_bus = ONLINE,                                          \
				.output_fmt = {},                                             \
				.channel_idx = 0                                              \
			},                                                                    \
		},                                                                            \
	};                                                                                    \
	PM_DEVICE_DT_INST_DEFINE(i, isp_pico_pm_action);                                      \
                                                                                              \
	DEVICE_DT_INST_DEFINE(i,                                                              \
		video_isp_init,                                                               \
		PM_DEVICE_DT_INST_GET(i),                                                     \
		&isp_data_##i,                                                                \
		&isp_config_##i,                                                              \
		POST_KERNEL,                                                                  \
		CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                                           \
		&isp_driver_api);                                                             \
                                                                                              \
		                                                                              \
	static void isp_config_func_##i(const struct device *dev)                             \
	{                                                                                     \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, isp, irq),                                 \
			    DT_INST_IRQ_BY_NAME(i, isp, priority),                            \
			    isp_isr_handler, DEVICE_DT_INST_GET(i), 0);                       \
		irq_enable(DT_INST_IRQ_BY_NAME(i, isp, irq));                                 \
		                                                                              \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(i, mi_isp, irq),                              \
			    DT_INST_IRQ_BY_NAME(i, mi_isp, priority),                         \
			    isp_isr_handler, DEVICE_DT_INST_GET(i), 0);                       \
		irq_enable(DT_INST_IRQ_BY_NAME(i, mi_isp, irq));                              \
	}

DT_INST_FOREACH_STATUS_OKAY(ISP_DEFINE)
