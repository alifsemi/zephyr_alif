/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT alif_cam

#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/drivers/pinctrl.h>

#include "video_alif.h"
#include <zephyr/drivers/video/video_alif.h>
#include <soc_memory_map.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(CPI, CONFIG_VIDEO_LOG_LEVEL);

#define WORKQ_STACK_SIZE 512
#define WORKQ_PRIORITY   7
K_KERNEL_STACK_DEFINE(alif_isr_cb_workq, WORKQ_STACK_SIZE);

extern size_t fourcc_to_plane_size(uint32_t fourcc, uint8_t plane_id, size_t buffer_size);
extern int fourcc_to_numplanes(uint32_t fourcc);
extern unsigned int pix_fmt_bpp(uint32_t fmt);

size_t fourcc_to_plane_size(uint32_t fourcc, uint8_t plane_id, size_t buffer_size)
{
	switch (fourcc)	{
	case VIDEO_PIX_FMT_BGGR8:
	case VIDEO_PIX_FMT_GBRG8:
	case VIDEO_PIX_FMT_GRBG8:
	case VIDEO_PIX_FMT_RGGB8:
	case VIDEO_PIX_FMT_BGGR10:
	case VIDEO_PIX_FMT_GBRG10:
	case VIDEO_PIX_FMT_GRBG10:
	case VIDEO_PIX_FMT_RGGB10:
	case VIDEO_PIX_FMT_BGGR12:
	case VIDEO_PIX_FMT_GBRG12:
	case VIDEO_PIX_FMT_GRBG12:
	case VIDEO_PIX_FMT_RGGB12:
	case VIDEO_PIX_FMT_BGGR14:
	case VIDEO_PIX_FMT_GBRG14:
	case VIDEO_PIX_FMT_GRBG14:
	case VIDEO_PIX_FMT_RGGB14:
	case VIDEO_PIX_FMT_BGGR16:
	case VIDEO_PIX_FMT_GBRG16:
	case VIDEO_PIX_FMT_GRBG16:
	case VIDEO_PIX_FMT_RGGB16:
	case VIDEO_PIX_FMT_GREY:
	case VIDEO_PIX_FMT_Y10P:
	case VIDEO_PIX_FMT_Y12P:
	case VIDEO_PIX_FMT_YVYU:
	case VIDEO_PIX_FMT_YUYV:
	case VIDEO_PIX_FMT_VYUY:
	case VIDEO_PIX_FMT_UYVY:
		if (plane_id == 0) {
			return buffer_size;
		} else {
			return 0;
		}
	case VIDEO_PIX_FMT_NV12:
	case VIDEO_PIX_FMT_NV21:
		if (plane_id == 0) {
			return (buffer_size << 1) / 3;
		} else if (plane_id == 1) {
			return (buffer_size / 3);
		} else {
			return 0;
		}
		break;
	case VIDEO_PIX_FMT_NV16:
	case VIDEO_PIX_FMT_NV61:
		if (plane_id == 0 || plane_id == 1) {
			return (buffer_size >> 1);
		} else {
			return 0;
		}
	case VIDEO_PIX_FMT_NV24:
	case VIDEO_PIX_FMT_NV42:
		if (plane_id == 0) {
			return (buffer_size / 3);
		} else if (plane_id == 1) {
			return ((buffer_size << 1) / 3);
		} else {
			return 0;
		}
	case VIDEO_PIX_FMT_YUV422P:
		if (plane_id == 0) {
			return (buffer_size >> 1);
		} else if (plane_id == 1 || plane_id == 2) {
			return (buffer_size >> 2);
		} else {
			return 0;
		}
	case VIDEO_PIX_FMT_YUV420:
		if (plane_id == 0) {
			return (buffer_size << 1) / 3;
		} else if (plane_id == 1 || plane_id == 2) {
			return (buffer_size / 6);
		} else {
			return 0;
		}
	case VIDEO_PIX_FMT_RGB888_PLANAR_PRIVATE:
		if (plane_id == 0 || plane_id == 1 || plane_id == 2) {
			return (buffer_size / 3);
		} else {
			return 0;
		}
	default:
		return UINT_MAX;
	}
}

int fourcc_to_numplanes(uint32_t fourcc)
{
	switch (fourcc) {
	case VIDEO_PIX_FMT_BGGR8:
	case VIDEO_PIX_FMT_GBRG8:
	case VIDEO_PIX_FMT_GRBG8:
	case VIDEO_PIX_FMT_RGGB8:
	case VIDEO_PIX_FMT_BGGR10:
	case VIDEO_PIX_FMT_GBRG10:
	case VIDEO_PIX_FMT_GRBG10:
	case VIDEO_PIX_FMT_RGGB10:
	case VIDEO_PIX_FMT_BGGR12:
	case VIDEO_PIX_FMT_GBRG12:
	case VIDEO_PIX_FMT_GRBG12:
	case VIDEO_PIX_FMT_RGGB12:
	case VIDEO_PIX_FMT_BGGR14:
	case VIDEO_PIX_FMT_GBRG14:
	case VIDEO_PIX_FMT_GRBG14:
	case VIDEO_PIX_FMT_RGGB14:
	case VIDEO_PIX_FMT_BGGR16:
	case VIDEO_PIX_FMT_GBRG16:
	case VIDEO_PIX_FMT_GRBG16:
	case VIDEO_PIX_FMT_RGGB16:
	case VIDEO_PIX_FMT_GREY:
	case VIDEO_PIX_FMT_Y10P:
	case VIDEO_PIX_FMT_Y12P:
	case VIDEO_PIX_FMT_YUYV:
	case VIDEO_PIX_FMT_YVYU:
	case VIDEO_PIX_FMT_VYUY:
	case VIDEO_PIX_FMT_UYVY:
		return 1;
	case VIDEO_PIX_FMT_NV12:
	case VIDEO_PIX_FMT_NV21:
	case VIDEO_PIX_FMT_NV16:
	case VIDEO_PIX_FMT_NV61:
	case VIDEO_PIX_FMT_NV24:
	case VIDEO_PIX_FMT_NV42:
		return 2;
	case VIDEO_PIX_FMT_YUV422P:
	case VIDEO_PIX_FMT_YUV420:
	case VIDEO_PIX_FMT_RGB888_PLANAR_PRIVATE:
		return 3;
	default:
		return 0;
	}
}

static void reg_write_part(uintptr_t reg, uint32_t data, uint32_t mask, uint8_t shift)
{
	uint32_t tmp = 0;

	tmp = sys_read32(reg);
	tmp &= ~(mask << shift);
	tmp |= (data & mask) << shift;
	sys_write32(tmp, reg);
}

unsigned int pix_fmt_bpp(uint32_t fmt)
{
	uint32_t ret;

	ret = video_bits_per_pixel(fmt);
	if (ret) {
		return ret;
	}

	switch (fmt) {
	case VIDEO_PIX_FMT_RGB565:
	case VIDEO_PIX_FMT_YUYV:
		return 16;
	case VIDEO_PIX_FMT_Y6P:
	case VIDEO_PIX_FMT_Y7P:
	case VIDEO_PIX_FMT_GREY:
		return 8;
	case VIDEO_PIX_FMT_Y10P:
		return 10;
	case VIDEO_PIX_FMT_Y12P:
		return 12;
	case VIDEO_PIX_FMT_Y14P:
		return 14;
	case VIDEO_PIX_FMT_Y10:
	case VIDEO_PIX_FMT_Y12:
	case VIDEO_PIX_FMT_Y14:
	case VIDEO_PIX_FMT_Y16:
		return 16;
	default:
		return 0;
	}
}

static inline void hw_enable_interrupts(uintptr_t regs, uint32_t intr_mask)
{
	sys_set_bits(regs + CAM_INTR_ENA, intr_mask);
}

static inline void hw_disable_interrupts(uintptr_t regs, uint32_t intr_mask)
{
	sys_clear_bits(regs + CAM_INTR_ENA, intr_mask);
}

static inline void hw_cam_start_video_capture(const struct device *dev)
{
	const struct video_cam_config *config = dev->config;
	uintptr_t regs = DEVICE_MMIO_GET(dev);

	/* Reset the CPI-Controller IP. */
	sys_write32(CAM_CTRL_SW_RESET, regs + CAM_CTRL);
	sys_write32(0, regs + CAM_CTRL);

	/* Start video capture. */
	if (config->capture_mode == CPI_CAPTURE_MODE_SNAPSHOT) {
		sys_write32(CAM_CTRL_FIFO_CLK_SEL | CAM_CTRL_SNAPSHOT | CAM_CTRL_START,
			    regs + CAM_CTRL);
	} else {
		sys_write32(CAM_CTRL_FIFO_CLK_SEL | CAM_CTRL_START, regs + CAM_CTRL);
	}
}

static int32_t fourcc_to_csi_data_type(uint32_t fourcc)
{
	/* TODO: Add support for RGB formats. */
	switch (fourcc) {
	case VIDEO_PIX_FMT_Y6P:
		return CSI2_DT_RAW6;
	case VIDEO_PIX_FMT_Y7P:
		return CSI2_DT_RAW7;
	case VIDEO_PIX_FMT_GREY:
	case VIDEO_PIX_FMT_BGGR8:
	case VIDEO_PIX_FMT_GBRG8:
	case VIDEO_PIX_FMT_GRBG8:
	case VIDEO_PIX_FMT_RGGB8:
		return CSI2_DT_RAW8;
	case VIDEO_PIX_FMT_Y10P:
		return CSI2_DT_RAW10;
	case VIDEO_PIX_FMT_Y12P:
		return CSI2_DT_RAW12;
	case VIDEO_PIX_FMT_Y14P:
		return CSI2_DT_RAW14;
	case VIDEO_PIX_FMT_Y16:
		return CSI2_DT_RAW16;
	}
	return -ENOTSUP;
}

static int alif_cam_set_csi(const struct device *dev, uint32_t fourcc)
{
	const struct video_cam_config *config = dev->config;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	int32_t tmp;
	int i;

	if (config->is_lpcam) {
		LOG_ERR("LP-CPI controller does not support MIPI-CSI2");
		return -EINVAL;
	}

	/* Set MIPI-CSI as the data source and enable halt function. */
	sys_set_bits(regs + CAM_CFG, CAM_CFG_MIPI_CSI);

	/* Set MIPI-CSI halt-enable function. */
	if (config->csi_halt_en) {
		sys_set_bits(regs + CAM_CFG, CAM_CFG_CSI_HALT_EN);
	}

	tmp = fourcc_to_csi_data_type(fourcc);
	if (tmp < 0) {
		LOG_ERR("Unsupported CSI pixel format.");
		return tmp;
	}

	for (i = 0; i < ARRAY_SIZE(data_mode_settings); i++) {
		if (data_mode_settings[i].dt == tmp) {
			break;
		}
	}

	sys_write32(data_mode_settings[i].col_mode, regs + CAM_CSI_CMCFG);

	switch (tmp) {
	case CSI2_DT_RAW10:
		reg_write_part(regs + CAM_CFG, CPI_DATA_MASK_10_BIT, CAM_CFG_DATA_MASK,
			       CAM_CFG_DATA_SHIFT);
		break;
	case CSI2_DT_RAW12:
		reg_write_part(regs + CAM_CFG, CPI_DATA_MASK_12_BIT, CAM_CFG_DATA_MASK,
			       CAM_CFG_DATA_SHIFT);
		break;
	case CSI2_DT_RAW14:
		reg_write_part(regs + CAM_CFG, CPI_DATA_MASK_14_BIT, CAM_CFG_DATA_MASK,
			       CAM_CFG_DATA_SHIFT);
		break;
	case CSI2_DT_RAW16:
	default:
		reg_write_part(regs + CAM_CFG, CPI_DATA_MASK_16_BIT, CAM_CFG_DATA_MASK,
			       CAM_CFG_DATA_SHIFT);
		break;
	}

	reg_write_part(regs + CAM_CFG, data_mode_settings[i].data_mode, CAM_CFG_DATA_MODE_MASK,
		       CAM_CFG_DATA_MODE_SHIFT);

	return 0;
}

/*
 * Moves the completed video buffer from IN-FIFO to the OUT-FIFO after a
 * STOP Interrupt is signalled by the CPI IP.
 */
static void alif_cam_work_helper(const struct device *dev)
{
	enum video_signal_result signal_status = VIDEO_BUF_DONE;
	const struct video_cam_config *config = dev->config;
	struct video_cam_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	struct video_buffer *vbuf = NULL;

	if (config->axi_bus_ep) {
		vbuf = k_fifo_peek_head(&data->fifo_in);
		if (vbuf == NULL) {
			LOG_ERR("Unexpected condition! The IN-FIFO should have "
				"at leastone buffer when we are handling bottom "
				"half of STOP interrupt!");
			data->curr_vid_buf = 0;
			data->is_streaming = false;
			signal_status = VIDEO_BUF_ERROR;
			goto done;
		}

		if (data->curr_vid_buf != (uint32_t)vbuf->buffer) {
			signal_status = VIDEO_BUF_ERROR;
			LOG_ERR("Unknown Video Buffer assigned to CPI Controller.");
			goto done;
		}

		/* Move completed buffer from IN-FIFO to OUT-FIFO. */
		vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);
		if (!vbuf) {
			LOG_ERR("Failed to get a video buffer from IN-FIFO, "
					"despite IN-FIFO having data");
			data->curr_vid_buf = 0;
			data->is_streaming = false;
			signal_status = VIDEO_BUF_ERROR;
			goto done;
		}

		/* Update the last-update timestamp of buffer. */
		vbuf->timestamp = k_uptime_get_32();

		/* Move finished buffer to OUT-FIFO. */
		k_fifo_put(&data->fifo_out, vbuf);

		/* Now assign a new framebuffer to the CPI Controller. */
		vbuf = k_fifo_peek_head(&data->fifo_in);
		if (vbuf == NULL) {
			LOG_DBG("No more Empty buffers in the IN-FIFO."
					"Stopping Video Capture. If Re-queued, restart stream.");
			data->curr_vid_buf = 0;
			data->is_streaming = false;
			video_stream_stop(config->endpoint_dev);
			signal_status = VIDEO_BUF_DONE;
			goto done;
		}

		/*
		 * Set the curr_vid_buf to the device here and restart data capture.
		 */
		data->curr_vid_buf = (uint32_t)vbuf->buffer;
		sys_write32(local_to_global(UINT_TO_POINTER(data->curr_vid_buf)),
				regs + CAM_FRAME_ADDR);

		/* Restart video capture. */
		hw_cam_start_video_capture(dev);

done:
		LOG_DBG("cur_vid_buf - 0x%08x", data->curr_vid_buf);
#if defined(CONFIG_POLL)
		if (data->signal) {
			k_poll_signal_raise(data->signal, signal_status);
		}
#endif /* defined(CONFIG_POLL) */
	} else {
		/* Restart video capture. */
		hw_cam_start_video_capture(dev);
	}
}

static void alif_isr_cb_work(struct k_work *work)
{
	struct video_cam_data *data = CONTAINER_OF(work, struct video_cam_data, cb_work);

	alif_cam_work_helper(data->dev);
}

static int alif_cam_set_fmt(const struct device *dev, enum video_endpoint_id ep,
		       struct video_format *fmt)
{
	const struct video_cam_config *config = dev->config;
	int bits_pp = pix_fmt_bpp(fmt->pixelformat);
	struct video_cam_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	int ret;

	/*
	 * Bail out when either the bits per pixel is zero (un-supported format)
	 * or endpoint is neither VIDEO_EP_OUT nor VIDEO_EP_ALL.
	 */
	if (!bits_pp || !(ep == VIDEO_EP_OUT || ep == VIDEO_EP_ALL)) {
		LOG_ERR("Bits-per-pixel - %d, endpoint- %d", bits_pp, ep);
		return -EINVAL;
	}

	data->current_format.pixelformat = fmt->pixelformat;
	data->current_format.pitch = fmt->pitch;
	data->current_format.width = fmt->width;
	data->current_format.height = fmt->height;

	sys_write32((((fmt->height - 1) & CAM_VIDEO_FCFG_ROW_MASK) << CAM_VIDEO_FCFG_ROW_SHIFT) |
			    ((fmt->width & CAM_VIDEO_FCFG_DATA_MASK) << CAM_VIDEO_FCFG_DATA_SHIFT),
		    regs + CAM_VIDEO_FCFG);

	ret = video_set_format(config->endpoint_dev, ep, fmt);
	if (ret) {
		LOG_ERR("Failed to set sensor Format.");
		return ret;
	}

	if (config->interface == CAM_INTERFACE_SERIAL) {
		ret = alif_cam_set_csi(dev, fmt->pixelformat);
		if (ret) {
			LOG_ERR("Failed to configure CAM as per the CSI.");
			return ret;
		}
	}

	data->is_streaming = 0;
	return 0;
}

static int alif_cam_get_fmt(const struct device *dev, enum video_endpoint_id ep,
		       struct video_format *fmt)
{
	const struct video_cam_config *config = dev->config;
	struct video_cam_data *data = dev->data;
	int ret;

	if (!fmt || (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL)) {
		return -EINVAL;
	}

	ret = video_get_format(config->endpoint_dev, ep, fmt);
	if (ret) {
		LOG_ERR("Failed to get format from Video pipeline!");
		return ret;
	}

	ret = alif_cam_set_fmt(dev, ep, fmt);
	if (ret) {
		return ret;
	}

	if (config->interface == CAM_INTERFACE_SERIAL) {
		ret = alif_cam_set_csi(dev, fmt->pixelformat);
		if (ret) {
			return ret;
		}
	}

	fmt->pixelformat = data->current_format.pixelformat;
	fmt->width = data->current_format.width;
	fmt->height = data->current_format.height;
	fmt->pitch = data->current_format.pitch;

	return 0;
}

static int alif_cam_stream_start(const struct device *dev)
{
	const struct video_cam_config *config = dev->config;
	struct video_cam_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	struct video_buffer *vbuf;
	int ret;

	if (data->is_streaming) {
		LOG_DBG("Already streaming.");
		return -EBUSY;
	}

	if (sys_read32(regs + CAM_CTRL) & CAM_CTRL_BUSY) {
		LOG_ERR("Can't start stream. Already Capturing!");
		return -EBUSY;
	}

	if (((IS_ENABLED(CONFIG_VIDEO_ALIF_CAM_EXTENDED)) && config->axi_bus_ep) ||
	    (!IS_ENABLED(CONFIG_VIDEO_ALIF_CAM_EXTENDED))) {
		/* Update the Video-buffer address to CPI-Controller. */
		vbuf = k_fifo_peek_head(&data->fifo_in);
		if (!vbuf) {
			LOG_ERR("No empty video-buffer. Aborting!!!");
			return -ENOBUFS;
		}

		data->curr_vid_buf = (uint32_t)vbuf->buffer;
		sys_write32(local_to_global(UINT_TO_POINTER(data->curr_vid_buf)),
				regs + CAM_FRAME_ADDR);
	}

	/* Setup the interrupts. */
	hw_enable_interrupts(regs, INTR_VSYNC | INTR_BRESP_ERR | INTR_OUTFIFO_OVERRUN |
					   INTR_INFIFO_OVERRUN | INTR_STOP);

	/* Start the MIPI CSI-2 IP in case the MIPI CSI is available. */
	ret = video_stream_start(config->endpoint_dev);
	if (ret) {
		LOG_ERR("Failed to start streaming of Video pipeline!");
		return -EIO;
	}

	hw_cam_start_video_capture(dev);
	LOG_DBG("Stream started");

	data->is_streaming = true;

	return 0;
}

static int alif_cam_stream_stop(const struct device *dev)
{
	const struct video_cam_config *config = dev->config;
	struct video_cam_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	uint32_t mask;
	int ret;

	if (!data->is_streaming) {
		LOG_DBG("Already stopped streaming.");
		return 0;
	}

	ret = video_stream_stop(config->endpoint_dev);
	if (ret) {
		LOG_ERR("Failed to stop streaming in Pipeline!");
		return ret;
	}

	/* Disable Interrupts. */
	hw_disable_interrupts(regs, INTR_VSYNC | INTR_BRESP_ERR | INTR_OUTFIFO_OVERRUN |
					    INTR_INFIFO_OVERRUN | INTR_STOP);

	/* Stop the Camera sensor to dump image. */
	sys_write32(0, regs + CAM_CTRL);

	/* Set the Current buffer state to NULL */
	data->curr_vid_buf = 0;

	/*
	 * Poll on Busy flag of CPI to find out when the video buffer
	 * is no longer accessed.
	 */
	mask = CAM_CTRL_BUSY;
	for (int i = 0; (i < 20) && (sys_read32(regs + CAM_CTRL) & mask) == mask; i++) {
		k_msleep(1);
	}
	LOG_DBG("Stream stopped");

	data->is_streaming = false;

	return 0;
}

static int alif_cam_set_stream(const struct device *dev, bool enable)
{
	if (enable) {
		return alif_cam_stream_start(dev);
	} else {
		return alif_cam_stream_stop(dev);
	}
}

static int alif_cam_flush(const struct device *dev, enum video_endpoint_id ep, bool cancel)
{
	struct video_cam_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	struct video_buffer *vbuf = NULL;
	uint32_t mask;

	if (!cancel) {
		if (!data->curr_vid_buf) {
			while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT))) {
				k_fifo_put(&data->fifo_out, vbuf);
			}
		}

		/*
		 * In case the cancel option is not provided, put the thread to
		 * sleep for 1 ms repeatedly. On every interrupt, the Video
		 * buffers will move from IN-FIFO to OUT-FIFO.
		 */
		while (!k_fifo_is_empty(&data->fifo_in)) {
			k_msleep(1);
		}
	} else {
		/*
		 * Disable interrupts to ensures that the current buffer is
		 * not moved from IN-FIFO to OUT-FIFO on STOP interrupt.
		 */
		/* Disable the Stop interrupt. */
		hw_disable_interrupts(regs, INTR_STOP);
		/* Stop Video capture. */
		sys_write32(0, regs + CAM_CTRL);

		/*
		 * Poll on Busy flag of CPI to find out when the video buffer
		 * is no longer accessed.
		 */
		mask = CAM_CTRL_BUSY;
		for (int i = 0; (i < 20) && (sys_read32(regs + CAM_CTRL) & mask) == mask; i++) {
			k_msleep(1);
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
	}

	/* Set current Video buffer to null address. */
	data->curr_vid_buf = 0;
	return 0;
}

static int alif_cam_enqueue(const struct device *dev, enum video_endpoint_id ep,
		       struct video_buffer *buf)
{
	const struct video_cam_config *config = dev->config;
	struct video_cam_data *data = dev->data;
	struct video_buffer *tmp_buf = NULL;
	uint32_t to_read;
	uint32_t tmp;

	if (IS_ENABLED(CONFIG_VIDEO_ALIF_CAM_EXTENDED)) {
		if (!config->axi_bus_ep) {
			LOG_DBG("AXI Master interface of the IP is not enabled!");
			return 0;
		}
	}

	if (ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}

	/* Check if the video buffer is 8-byte aligned or not.*/
	tmp = (uint32_t)buf->buffer;
	if (ROUND_UP(tmp, 8) != tmp) {
		LOG_ERR("Video Buffer is not aligned to 8-byte boundary."
			"It can result in corruption of captured image.");
		return -ENOBUFS;
	}

	to_read = data->current_format.pitch * data->current_format.height;
	buf->bytesused = to_read;

	k_fifo_put(&data->fifo_in, buf);

	tmp_buf = k_fifo_peek_tail(&data->fifo_in);
	LOG_DBG("Enqueued buffer: Addr - 0x%x, size - %d, bytesused - %d",
		(uint32_t)tmp_buf->buffer, tmp_buf->size, tmp_buf->bytesused);

	return 0;
}

static int alif_cam_dequeue(const struct device *dev, enum video_endpoint_id ep,
		       struct video_buffer **buf, k_timeout_t timeout)
{
	const struct video_cam_config *config = dev->config;
	struct video_cam_data *data = dev->data;

	if (IS_ENABLED(CONFIG_VIDEO_ALIF_CAM_EXTENDED)) {
		if (!config->axi_bus_ep) {
			LOG_DBG("AXI Master interface of the IP is not enabled!");
			return 0;
		}
	}

	if (ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}

	*buf = k_fifo_get(&data->fifo_out, timeout);
	if (!(*buf)) {
		return -EAGAIN;
	}

	LOG_DBG("Dequeued buffer: Addr - 0x%08x, size - %d, bytesused - %d",
		(uint32_t)(*buf)->buffer, (*buf)->size, (*buf)->bytesused);
	return 0;
}

static int alif_cam_set_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	const struct video_cam_config *config = dev->config;
	int ret = -ENOTSUP;

	ret = video_set_ctrl(config->endpoint_dev, cid, value);
	if (ret) {
		return -ENOTSUP;
	}

	return 0;
}

static int alif_cam_get_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	const struct video_cam_config *config = dev->config;

	return video_get_ctrl(config->endpoint_dev, cid, value);
}

static int alif_cam_get_caps(const struct device *dev, enum video_endpoint_id ep,
			struct video_caps *caps)
{
	const struct video_cam_config *config = dev->config;
	int err = -ENODEV;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		LOG_ERR("Invalid end-point, ep = %d", ep);
		return -EINVAL;
	}

	err = video_get_caps(config->endpoint_dev, ep, caps);
	caps->min_vbuf_count = CPI_MIN_VBUF;

	return err;
}

#ifdef CONFIG_POLL
static int alif_cam_set_signal(const struct device *dev, enum video_endpoint_id ep,
			  struct k_poll_signal *signal)
{
	struct video_cam_data *data = dev->data;

	if (signal != NULL && data->signal) {
		return -EALREADY;
	}

	data->signal = signal;
	return 0;
}
#endif

static DEVICE_API(video, cam_driver_api) = {
	.set_format = alif_cam_set_fmt,
	.get_format = alif_cam_get_fmt,
	.set_stream = alif_cam_set_stream,
	.flush = alif_cam_flush,
	.enqueue = alif_cam_enqueue,
	.dequeue = alif_cam_dequeue,
	.set_ctrl = alif_cam_set_ctrl,
	.get_ctrl = alif_cam_get_ctrl,
	.get_caps = alif_cam_get_caps,
#ifdef CONFIG_POLL
	.set_signal = alif_cam_set_signal,
#endif
};

static void alif_video_cam_isr(const struct device *dev)
{
	static bool is_not_corrupted_frame = true;
	struct video_cam_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	uint32_t err_mask = INTR_OUTFIFO_OVERRUN | INTR_INFIFO_OVERRUN | INTR_BRESP_ERR;
	uint32_t int_st = 0;
	uint32_t tmp = 0;

	/* Clear interrupts. */
	int_st = sys_read32(regs + CAM_INTR) & sys_read32(regs + CAM_INTR_ENA);
	sys_write32(int_st, regs + CAM_INTR);

	if (int_st & INTR_HSYNC) {
		LOG_DBG("H-SYNC detected.");
	}

	if (int_st & INTR_VSYNC) {
		LOG_DBG("V-SYNC detected.");
	}

	if (int_st & INTR_BRESP_ERR) {
		tmp = sys_read32(regs + CAM_AXI_ERR_STAT);
		LOG_ERR("AXI Error count - %ld, AXI BRESP error code - %ld",
			((tmp >> CAM_AXI_ERR_STAT_CNT_SHIFT) & CAM_AXI_ERR_STAT_CNT_MASK),
			((tmp >> CAM_AXI_ERR_STAT_BRESP_SHIFT) & CAM_AXI_ERR_STAT_BRESP_MASK));
	}

	if (int_st & err_mask) {
		LOG_ERR("Frame Capture Error. int_st - 0x%08x", int_st);
		is_not_corrupted_frame = false;
#if defined(CONFIG_POLL)
		if (data->signal) {
			k_poll_signal_raise(data->signal, VIDEO_BUF_ERROR);
		}
#endif /* defined(CONFIG_POLL) */
	}

	if (int_st & INTR_STOP) {
		sys_write32(0, regs + CAM_CTRL);
		/* No corruption observed during dumping this frame. */
		if (is_not_corrupted_frame) {
			LOG_DBG("Video Capture stopped.");
			k_work_submit_to_queue(&data->cb_workq, &data->cb_work);
		} else {
			/* Wait for user to handle corrupted frame capture. */
			data->curr_vid_buf = 0;
			is_not_corrupted_frame = true;
		}
	}
}

static int alif_video_cam_set_config(const struct device *dev)
{
	const struct video_cam_config *config = dev->config;
	uintptr_t regs = DEVICE_MMIO_GET(dev);

	if ((config->data_mode >= CPI_DATA_MODE_16_BIT) && config->is_lpcam) {
		LOG_ERR("LP-CPI controller does not support "
			"any data format >= 16-bit.");
		return -EINVAL;
	}

	if ((config->data_mode >= CPI_DATA_MODE_32_BIT) &&
	    (config->interface == CAM_INTERFACE_PARALLEL)) {
		LOG_ERR("data modes 32-bit and 64-bit are reserved "
			"for CSI-2 only.");
		return -EINVAL;
	}

	/* Setup Camera signals (V-Sync, H-Sync, PCLK) polarity inversion. */
	sys_write32(config->polarity, regs + CAM_CFG);

	/* Capture video-data when both V-Sync and H-Sync are high. */
	if (config->vsync_en) {
		sys_set_bits(regs + CAM_CFG, CAM_CFG_VSYNC_EN);
	} else {
		sys_clear_bits(regs + CAM_CFG, CAM_CFG_VSYNC_EN);
	}

	/* Capture video-data when both V-Sync and H-Sync are aligned. */
	if (config->wait_vsync) {
		sys_set_bits(regs + CAM_CFG, CAM_CFG_WAIT_VSYNC);
	} else {
		sys_clear_bits(regs + CAM_CFG, CAM_CFG_WAIT_VSYNC);
	}

	/* Capture video-data with 10-bits of data on 8-bit channel encoding. */
	if (config->code10on8 && (config->data_mode == CPI_DATA_MODE_8_BIT)) {
		sys_set_bits(regs + CAM_CFG, CAM_CFG_CODE10ON8);
	} else {
		sys_clear_bits(regs + CAM_CFG, CAM_CFG_CODE10ON8);
	}

	/* MSB selection of video-data significant only when data-mode <= 8. */
	if (config->msb && (config->data_mode <= CPI_DATA_MODE_8_BIT)) {
		sys_set_bits(regs + CAM_CFG, CAM_CFG_MSB);
	} else {
		sys_clear_bits(regs + CAM_CFG, CAM_CFG_MSB);
	}

	/* Data-Mask to be set only when Data-Mode(pin-connects) is 16-bit. */
	if (config->data_mode == CPI_DATA_MODE_16_BIT) {
		reg_write_part(regs + CAM_CFG, config->data_mask, CAM_CFG_DATA_MASK,
			       CAM_CFG_DATA_SHIFT);
	}

	/* If write to memory via AXI-bus EP is enabled. */
	if (IS_ENABLED(CONFIG_VIDEO_ALIF_CAM_EXTENDED)) {
		if (config->axi_bus_ep) {
			sys_set_bits(regs + CAM_CFG, CAM_CFG_AXI_PORT_EN);
		}
	}

	/* If forwarding the video signals to ISP via ISP interface is enabled. */
	if (IS_ENABLED(CONFIG_VIDEO_ALIF_CAM_EXTENDED)) {
		if (config->isp_ep) {
			sys_set_bits(regs + CAM_CFG, CAM_CFG_ISP_PORT_EN);
		}
	}

	reg_write_part(regs + CAM_CFG, config->data_mode, CAM_CFG_DATA_MODE_MASK,
		       CAM_CFG_DATA_MODE_SHIFT);

	return 0;
}

int alif_video_cam_setup(const struct device *dev)
{
	const struct video_cam_config *config = dev->config;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	uint32_t tmp = 0;
	int ret;

	/* Setup the Watermarks. */
	tmp = ((config->write_wmark & CAM_FIFO_CTRL_WR_WMARK_MASK)
	       << CAM_FIFO_CTRL_WR_WMARK_SHIFT) |
	      ((config->read_wmark & CAM_FIFO_CTRL_RD_WMARK_MASK) << CAM_FIFO_CTRL_RD_WMARK_SHIFT);
	sys_write32(tmp, regs + CAM_FIFO_CTRL);

	ret = alif_video_cam_set_config(dev);
	if (ret) {
		LOG_ERR("Setup of CPI controller failed. ret - %d", ret);
		return ret;
	}

	return 0;
}

static int alif_cam_enable_clocks(const struct device *dev)
{
	const struct video_cam_config *config = dev->config;

	/* Enable CAM clock. */
	return clock_control_on(config->clk_dev, config->cid);
}

static int alif_video_cam_init(const struct device *dev)
{
	const struct video_cam_config *config = dev->config;
	struct video_cam_data *data = dev->data;
	int ret = 0;

	/*
	 * In-case there is no sensor device or CSI controller attached to the
	 * CPI controller, we need to abort.
	 */
	if (!config->endpoint_dev) {
		return -ENODEV;
	}

	if ((config->interface == CAM_INTERFACE_SERIAL) && config->is_lpcam) {
		LOG_ERR("LP-CAM does not support Serial interface!");
		return -EINVAL;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	LOG_DBG("MMIO Address: 0x%x", (uint32_t)DEVICE_MMIO_GET(dev));

	ret = alif_cam_enable_clocks(dev);
	if (ret) {
		LOG_ERR("CAM clock enable failed! Exiting! ret - %d", ret);
		return ret;
	}

	if (config->pcfg) {
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret) {
			LOG_ERR("Failed to apply Pinctrl.");
			return ret;
		}
	}

	/* Setup ISR callback work. */
	k_work_init(&data->cb_work, alif_isr_cb_work);
	k_work_queue_init(&data->cb_workq);
	k_work_queue_start(&data->cb_workq, alif_isr_cb_workq,
			   K_KERNEL_STACK_SIZEOF(alif_isr_cb_workq),
			   K_PRIO_COOP(WORKQ_PRIORITY), NULL);
	k_thread_name_set(&data->cb_workq.thread, "alif_cam_work_helper");

	/* Setup interrupts. */
	config->irq_config_func(dev);

	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);
	data->dev = dev;

	/* Setup the CPI-Controller hardware config. */
	ret = alif_video_cam_setup(dev);
	if (ret) {
		LOG_ERR("Error setting up the CPI controller.");
		return ret;
	}

	LOG_DBG("irq: %d", config->irq);
	LOG_DBG("Is LP-CPI controller - %d", config->is_lpcam);
	LOG_DBG("Is Parallel interface - %d",
			(config->interface == CAM_INTERFACE_PARALLEL));
	switch (config->data_mode) {
	case CPI_DATA_MODE_1_BIT:
		LOG_DBG("Data-mode: 1-bit");
		break;
	case CPI_DATA_MODE_2_BIT:
		LOG_DBG("Data-mode: 2-bit");
		break;
	case CPI_DATA_MODE_4_BIT:
		LOG_DBG("Data-mode: 4-bit");
		break;
	case CPI_DATA_MODE_8_BIT:
		LOG_DBG("Data-mode: 8-bit");
		break;
	case CPI_DATA_MODE_16_BIT:
		LOG_DBG("Data-mode: 16-bit");
		break;
	case CPI_DATA_MODE_32_BIT:
		LOG_DBG("Data-mode: 32-bit");
		break;
	case CPI_DATA_MODE_64_BIT:
		LOG_DBG("Data-mode: 64-bit");
		break;
	default:
		LOG_DBG("Unknown Data-mode!");
		return -ENODEV;
	}

	if (config->data_mode == CPI_DATA_MODE_16_BIT) {
		switch (config->data_mask) {
		case CPI_DATA_MASK_16_BIT:
			LOG_DBG("Data Mask: 16-bit");
			break;
		case CPI_DATA_MASK_10_BIT:
			LOG_DBG("Data Mask: 10-bit");
			break;
		case CPI_DATA_MASK_12_BIT:
			LOG_DBG("Data Mask: 12-bit");
			break;
		case CPI_DATA_MASK_14_BIT:
			LOG_DBG("Data Mask: 14-bit");
			break;
		default:
			LOG_DBG("Unknown Data mask");
			return -ENODEV;
		}
	}

	if (IS_ENABLED(CONFIG_VIDEO_ALIF_CAM_EXTENDED)) {
		LOG_DBG("AXI EP: %d, ISP EP: %d", config->axi_bus_ep, config->isp_ep);
	}

	data->is_streaming = false;

	return 0;
}

#define CAM_GET_CLK(i)                                                         \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(i, clocks),                           \
		(.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(i)),             \
		 .cid = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(i, \
			 cam_clk, clkid),))

#define REMOTE_DEVICE(i, id) \
	DT_NODE_REMOTE_DEVICE(DT_INST_ENDPOINT_BY_ID(i, id, 0))

#define CPI_DEFINE(i)                                                                              \
	IF_ENABLED(CONFIG_PINCTRL,                                                                 \
			(COND_CODE_1(DT_INST_PINCTRL_HAS_IDX(i, 0),                                \
				     (PINCTRL_DT_INST_DEFINE(i);), ())))                           \
	static void cam_config_func_##i(const struct device *dev);                                 \
                                                                                                   \
	static const struct video_cam_config config_##i = {                                        \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(i)),                                              \
                                                                                                   \
		CAM_GET_CLK(i)                                                                     \
		.endpoint_dev = DEVICE_DT_GET(REMOTE_DEVICE(i, 0)),                                \
                                                                                                   \
		.irq = DT_INST_IRQN(i),                                                            \
		.irq_config_func = cam_config_func_##i,                                            \
		IF_ENABLED(CONFIG_PINCTRL,                                                         \
			(COND_CODE_1(DT_INST_PINCTRL_HAS_IDX(i, 0),                                \
				     (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),),                 \
				     (.pcfg = NULL,))))                                            \
                                                                                                   \
		.polarity = COND_CODE_1(DT_INST_PROP(i, inv_vsync_pol), CAM_CFG_VSYNC_POL, (0)) |  \
			    COND_CODE_1(DT_INST_PROP(i, inv_hsync_pol), CAM_CFG_HSYNC_POL, (0)) |  \
			    COND_CODE_1(DT_INST_PROP(i, inv_pclk_pol), CAM_CFG_PCLK_POL, (0)),     \
		.read_wmark = DT_INST_PROP(i, fifo_rd_wmark),                                      \
		.write_wmark = DT_INST_PROP(i, fifo_wr_wmark),                                     \
		.is_lpcam = DT_INST_PROP(i, lp_cam),                                               \
		.msb = DT_INST_PROP(i, msb),                                                       \
		.vsync_en = DT_INST_PROP(i, vsync_en),                                             \
		.wait_vsync = DT_INST_PROP(i, wait_vsync),                                         \
		.capture_mode = DT_INST_ENUM_IDX(i, capture_mode),                                 \
		.data_mode = DT_INST_ENUM_IDX(i, data_mode),                                       \
		.data_mask = DT_INST_ENUM_IDX(i, data_mask),                                       \
		.code10on8 = DT_INST_PROP(i, code_10_on_8),                                        \
		.axi_bus_ep = DT_NODE_EXISTS(DT_INST_PORT_BY_ID(i, 1)),                            \
		.isp_ep = DT_NODE_HAS_STATUS_OKAY(                                                 \
				DT_NODE_REMOTE_DEVICE(                                             \
					DT_INST_ENDPOINT_BY_ID(i, 2, 0))),                         \
		.csi_halt_en = DT_INST_PROP(i, csi_halt_en),                                       \
		.interface = DT_INST_ENUM_IDX(i, cpi_interface),                                   \
	};                                                                                         \
                                                                                                   \
	static struct video_cam_data data_##i;                                                     \
	DEVICE_DT_INST_DEFINE(i, &alif_video_cam_init, NULL, &data_##i, &config_##i,               \
			      POST_KERNEL, CONFIG_VIDEO_ALIF_CAM_INIT_PRIORITY, &cam_driver_api);  \
                                                                                                   \
	static void cam_config_func_##i(const struct device *dev)                                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(i), DT_INST_IRQ(i, priority), alif_video_cam_isr,         \
			    DEVICE_DT_INST_GET(i), 0);                                             \
		irq_enable(DT_INST_IRQN(i));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(CPI_DEFINE)
