/*
 * Copyright (C) 2025 Alif Semiconductor.
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

#define WORKQ_STACK_SIZE 1024
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
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_BGGR8, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GBRG8, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GRBG8, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_RGGB8, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_BGGR10, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GBRG10, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GRBG10, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_RGGB10, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_BGGR12, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GBRG12, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GRBG12, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_RGGB12, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_NV12, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_NV16, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_YUV422P, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_YUV420, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_YUYV, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_GREY, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_Y10, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_Y12, 640, 480),
	ISP_VIDEO_FORMAT_CAP(VIDEO_PIX_FMT_RGB888_PLANAR_PRIVATE, 640, 480),
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

static int isp_attach_buffer_to_hw(const struct device *dev, struct video_buffer *vbuf)
{
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	struct isp_data *data = dev->data;
	uint32_t planes[3] = {};
	size_t size_plane;
	int num_planes;
	int i;

	struct channel_parameters *channel = &data->init_cfg.channel;

	num_planes = fourcc_to_numplanes(channel->output_fmt.pixelformat);
	if (num_planes == 0) {
		LOG_ERR("Unsupported format!");
		return -ENOTSUP;
	}

	for (i = 0; i < num_planes; i++) {
		size_plane = fourcc_to_plane_size(channel->output_fmt.pixelformat,
				i, vbuf->size);
		if (size_plane == 0 || size_plane > vbuf->size) {
			LOG_ERR("Unsupported format!");
			return -ENOTSUP;
		}

		planes[i] = (i) ? (planes[i-1] + size_plane) :
			POINTER_TO_UINT(local_to_global(vbuf->buffer));
	}

	LOG_DBG("planes: 0x%08x 0x%08x 0x%08x", planes[0], planes[1], planes[2]);
	sys_write32(planes[0], regs + ISP_MI_MP_Y_BASE_AD_INIT);
	sys_write32(planes[1], regs + ISP_MI_MP_CB_BASE_AD_INIT);
	sys_write32(planes[2], regs + ISP_MI_MP_CR_BASE_AD_INIT);

	sys_set_bits(regs + ISP_MI_INIT, MI_INIT_CFG_UPD);

	return 0;
}

static void hw_disable_mi_interrupts(uintptr_t regs, uint32_t mask)
{
	sys_clear_bits(regs + ISP_MI_IMSC, mask);
}

static void isp_bottom_half(const struct device *dev)
{
	enum video_signal_result signal_status = VIDEO_BUF_DONE;
	const struct isp_config *config = dev->config;
	struct isp_data *data = dev->data;
	struct video_buffer *vbuf = NULL;

	int ret;

	/* Do bottom half processing of all the modules at the end of frame. */
	isp_vsi_bottom_half(&data->init_cfg, data->mi_mis);


	vbuf = k_fifo_peek_head(&data->fifo_in);
	if (vbuf == NULL) {
		LOG_ERR("Unexpected condition! Empty IN-FIFO");
		data->is_streaming = false;
		signal_status = VIDEO_BUF_ERROR;
		goto isp_bottom_done;
	}

	if (data->curr_vid_buf != (uint32_t)vbuf->buffer) {
		signal_status = VIDEO_BUF_ERROR;
		data->is_streaming = false;
		LOG_ERR("Unknown Video Buffer assigned to ISP.");
		goto isp_bottom_done;
	}

	vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);
	if (!vbuf) {
		LOG_ERR("Failed to get video buffer from IN-FIFO, "
			"despite IN-FIFO having data");
		data->is_streaming = false;
		signal_status = VIDEO_BUF_ERROR;
		goto isp_bottom_done;
	}

	vbuf->timestamp = k_uptime_get_32();

	k_fifo_put(&data->fifo_out, vbuf);

	vbuf = k_fifo_peek_head(&data->fifo_in);
	if (vbuf == NULL) {
		LOG_DBG("No more empty buffers in the IN-FIFO. "
			"Stopping video capture. If re-queued, restart stream.");
		data->is_streaming = false;
		signal_status = VIDEO_BUF_DONE;
		goto isp_bottom_done;
	}
	data->curr_vid_buf = (uint32_t) vbuf->buffer;

	ret = isp_attach_buffer_to_hw(dev, vbuf);
	if (ret) {
		LOG_ERR("Failed to attach buffer to hardware!");
		data->is_streaming = false;
		signal_status = VIDEO_BUF_DONE;
		goto isp_bottom_done;
	}

isp_bottom_done:
	if (!data->is_streaming) {
		video_stream_stop(config->controller);
		data->curr_vid_buf = 0;
	}

	LOG_DBG("current video buffer - 0x%08x", data->curr_vid_buf);
#if defined(CONFIG_POLL)
	if (data->signal) {
		k_poll_signal_raise(data->signal, signal_status);
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
		if (!memcmp(fmt, &port->port_fmt, sizeof(*fmt))) {
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
		if (memcmp(fmt, &port->port_fmt, sizeof(*fmt))) {
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
	struct video_buffer *vbuf;
	struct video_buffer vbuf2;

	struct port_parameters *port = &data->init_cfg.port;
	uint32_t tmp;
	int ret;

	if (data->is_streaming) {
		LOG_DBG("Already streaming");
		return -EBUSY;
	}

	vbuf = k_fifo_peek_head(&data->fifo_in);
	if (vbuf == NULL) {
		LOG_ERR("Unexpected condition! Empty IN-FIFO. Can't start streaming!");
		data->is_streaming = false;
		return -ENODEV;
	}

	data->curr_vid_buf = POINTER_TO_UINT(vbuf->buffer);

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

	ret = isp_vsi_enqueue(&data->init_cfg, vbuf);
	if (ret) {
		LOG_ERR("Failed to assign buffer to hardware!");
		data->curr_vid_buf = 0;
		return ret;
	}

	ret = isp_vsi_start(&data->init_cfg);
	if (ret) {
		LOG_ERR("Failed to start stream!");
		goto dequeue_buf;
	}

	ret = video_stream_start(config->controller);
	if (ret) {
		LOG_ERR("Failed to start stream for Endpoint device: %s!",
				config->controller->name);
		goto stop_isp_stream;
	}

	data->is_streaming = true;
	return 0;

stop_isp_stream:
	ret = isp_vsi_stop(&data->init_cfg);
	if (ret) {
		LOG_ERR("Failed to stop ISP device streaming");
		return ret;
	}
dequeue_buf:
	ret = isp_vsi_dequeue(&data->init_cfg, &vbuf2);
	if (ret) {
		LOG_ERR("Failed to dequeue buffer back!");
		return ret;
	}

	return 0;
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

	ret = isp_vsi_stop(&data->init_cfg);
	if (ret) {
		LOG_ERR("Failed to stop ISP from streaming!");
		return ret;
	}

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
	struct video_buffer *vbuf = NULL;

	int ret;

	if (cancel) {
		/* Case when video stream processing needs to be stopped. */
		hw_disable_mi_interrupts(regs, MI_INTR_MP_FRAME_END);

		for (int i = 0; (i < 20) &&
				(sys_read32(regs + ISP_MI_RIS) & MI_INTR_MP_FRAME_END); i++) {
			k_msleep(10);
		}

		if (sys_read32(regs + ISP_MI_RIS) & MI_INTR_MP_FRAME_END) {
			LOG_ERR("Failed to observe frame end!");
			return -EBUSY;
		}

		ret = isp_vsi_stop(&data->init_cfg);
		if (ret) {
			LOG_ERR("Failed to stop ISP device!");
			return ret;
		}

		while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT))) {
			k_fifo_put(&data->fifo_out, vbuf);
			LOG_DBG("Video Buffer Aborted!!! - 0x%x", (uint32_t)vbuf->buffer);
#if defined(CONFIG_POLL)
			if (data->signal) {
				k_poll_signal_raise(data->signal, VIDEO_BUF_ABORTED);
			}
#endif /* defined(CONFIG_POLL) */
		}
	} else {
		/* Case when video stream processing need not be stopped. */
		if (!data->curr_vid_buf) {
			while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT))) {
				k_fifo_put(&data->fifo_out, vbuf);
			}
		}

		while (!k_fifo_is_empty(&data->fifo_in)) {
			k_msleep(1);
		}
	}

	data->curr_vid_buf = 0;
	data->is_streaming = false;

	video_flush(config->controller, ep, cancel);

	return 0;
}

static int isp_enqueue(const struct device *dev, enum video_endpoint_id ep,
		       struct video_buffer *buf)
{
	struct isp_data *data = dev->data;
	uint32_t to_read;
	uint32_t tmp;

	struct channel_parameters *channel = &data->init_cfg.channel;

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

	to_read = channel->output_fmt.pitch * channel->output_fmt.height;
	buf->bytesused = 0;

	k_fifo_put(&data->fifo_in, buf);

	LOG_DBG("Enqueued buffer: Addr - 0x%x, size - %d, bytesused - %d",
		(uint32_t)buf->buffer, buf->size, buf->bytesused);

	return 0;
}

static int isp_dequeue(const struct device *dev, enum video_endpoint_id ep,
		       struct video_buffer **buf, k_timeout_t timeout)
{
	struct isp_data *data = dev->data;

	struct channel_parameters *channel = &data->init_cfg.channel;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	*buf = k_fifo_get(&data->fifo_out, timeout);
	if (!(*buf)) {
		return -EAGAIN;
	}

	(*buf)->bytesused = channel->output_fmt.pitch * channel->output_fmt.height;
	LOG_DBG("Dequeued buffer: Addr - 0x%08x, size - %d, bytesused - %d",
		(uint32_t)(*buf)->buffer, (*buf)->size, (*buf)->bytesused);
	return 0;
}

static int isp_set_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	const struct isp_config *config = dev->config;
	int ret = -ENOTSUP;

	ret = video_set_ctrl(config->controller, cid, value);
	if (ret) {
		return ret;
	}

	return 0;
}

static int isp_get_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	const struct isp_config *config = dev->config;

	return video_get_ctrl(config->controller, cid, value);
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

#define REMOTE_DEVICE(i, idx)	                                           \
	DT_NODE_REMOTE_DEVICE(DT_INST_ENDPOINT_BY_ID(i, idx, 0))

#define REMOTE_EP(n, pid, epid)                                            \
	DT_NODELABEL(DT_STRING_TOKEN(DT_INST_ENDPOINT_BY_ID(n, pid, epid), \
				remote_endpoint_label))

#define ISP_DEFINE(i)                                                                         \
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
			},                                                                    \
			.channel = {                                                          \
				.trans_bus = ONLINE,                                          \
				.output_fmt = {},                                             \
				.channel_idx = 0                                              \
			},                                                                    \
		},                                                                            \
	};                                                                                    \
                                                                                              \
	DEVICE_DT_INST_DEFINE(i,                                                              \
		video_isp_init,                                                               \
		NULL,                                                                         \
		&isp_data_##i,                                                                \
		&isp_config_##i,                                                              \
		POST_KERNEL,                                                                  \
		CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                                           \
		&isp_driver_api);                                                             \
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
