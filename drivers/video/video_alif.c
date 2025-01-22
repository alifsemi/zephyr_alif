/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT alif_cam

#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(CPI, CONFIG_VIDEO_LOG_LEVEL);

#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/drivers/pinctrl.h>

#include "video_alif.h"
#include <zephyr/drivers/video/video_alif.h>

#define WORKQ_STACK_SIZE 512
#define WORKQ_PRIORITY   7
K_KERNEL_STACK_DEFINE(isr_cb_workq, WORKQ_STACK_SIZE);

#ifdef CONFIG_FB_USES_DTCM_REGION
#define DTCM_GLOBAL_BASE   DT_PROP(DT_NODELABEL(dtcm), dtcm_global_base)
#define DTCM_LOCAL_BASE    DT_REG_ADDR(DT_NODELABEL(dtcm))
#define LOCAL_TO_GLOBAL(x) (x - DTCM_LOCAL_BASE + DTCM_GLOBAL_BASE)
#else
#define LOCAL_TO_GLOBAL(x) (x)
#endif /* CONFIG_FB_USES_DTCM_REGION */

static void reg_write_part(uintptr_t reg, uint32_t data, uint32_t mask, uint8_t shift)
{
	uint32_t tmp = 0;

	tmp = sys_read32(reg);
	tmp &= ~(mask << shift);
	tmp |= (data & mask) << shift;
	sys_write32(tmp, reg);
}

static inline unsigned int pix_fmt_bpp(uint32_t fmt)
{
	switch (fmt) {
	case VIDEO_PIX_FMT_BGGR8:
	case VIDEO_PIX_FMT_GBRG8:
	case VIDEO_PIX_FMT_GRBG8:
	case VIDEO_PIX_FMT_RGGB8:
		return 8;
	case VIDEO_PIX_FMT_RGB565:
	case VIDEO_PIX_FMT_YUYV:
		return 16;
	case VIDEO_PIX_FMT_Y6P:
	case VIDEO_PIX_FMT_Y7P:
	case VIDEO_PIX_FMT_GREY:
		return 8;
	case VIDEO_PIX_FMT_Y10P:
	case VIDEO_PIX_FMT_Y12P:
	case VIDEO_PIX_FMT_Y14P:
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

#ifdef CONFIG_VIDEO_MIPI_CSI2_DW
static int32_t fourcc_to_csi_data_type(uint32_t fourcc)
{
	/* TODO: Add support for RGB formats. */
	switch (fourcc) {
	case VIDEO_PIX_FMT_Y6P:
		return CSI2_DT_RAW6;
	case VIDEO_PIX_FMT_Y7P:
		return CSI2_DT_RAW7;
	case VIDEO_PIX_FMT_GREY:
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

static int cam_set_csi(const struct device *dev, uint32_t fourcc)
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
#endif /* CONFIG_VIDEO_MIPI_CSI2_DW */

/*
 * Moves the completed video buffer from IN-FIFO to the OUT-FIFO after a
 * STOP Interrupt is signalled by the CPI IP.
 */
static void cam_work_helper(const struct device *dev)
{
	enum video_signal_result signal_status = VIDEO_BUF_DONE;
	struct video_cam_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	struct video_buffer *vbuf = NULL;

	vbuf = k_fifo_peek_head(&data->fifo_in);
	if (data->curr_vid_buf != (uint32_t)vbuf->buffer) {
		signal_status = VIDEO_BUF_ERROR;
		LOG_ERR("Unknown Video Buffer assigned to CPI Controller.");
		goto done;
	}

	/* Move completed buffer from IN-FIFO to OUT-FIFO. */
	vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);

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
		data->state = STATE_CONTROLLER_STOPPED;
		signal_status = VIDEO_BUF_DONE;
		goto done;
	}

	/*
	 * Set the curr_vid_buf to the device here and restart data capture.
	 */
	data->curr_vid_buf = (uint32_t)vbuf->buffer;
	sys_write32(LOCAL_TO_GLOBAL(data->curr_vid_buf), regs + CAM_FRAME_ADDR);

	/* Restart video capture. */
	hw_cam_start_video_capture(dev);

done:
	LOG_DBG("cur_vid_buf - 0x%08x", data->curr_vid_buf);
#if defined(CONFIG_POLL)
	if (data->signal) {
		k_poll_signal_raise(data->signal, signal_status);
	}
#endif /* defined(CONFIG_POLL) */
}

static void isr_cb_work(struct k_work *work)
{
	struct video_cam_data *data = CONTAINER_OF(work, struct video_cam_data, cb_work);

	cam_work_helper(data->dev);
}

static int cam_set_fmt(const struct device *dev, enum video_endpoint_id ep,
		       struct video_format *fmt)
{
	const struct video_cam_config *config = dev->config;
	int bits_pp = pix_fmt_bpp(fmt->pixelformat);
	struct video_cam_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	int ret;

	if (!bits_pp || ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}

	data->current_format.pixelformat = fmt->pixelformat;
	data->current_format.pitch = fmt->pitch;
	data->current_format.width = fmt->width;
	data->current_format.height = fmt->height;
	data->bits_pp = bits_pp;

	sys_write32((((fmt->height - 1) & CAM_VIDEO_FCFG_ROW_MASK) << CAM_VIDEO_FCFG_ROW_SHIFT) |
			    ((fmt->width & CAM_VIDEO_FCFG_DATA_MASK) << CAM_VIDEO_FCFG_DATA_SHIFT),
		    regs + CAM_VIDEO_FCFG);

#ifdef CONFIG_VIDEO_MIPI_CSI2_DW
	if (config->csi_bus) {
		ret = video_set_format(config->csi_bus, ep, fmt);
		if (ret) {
			LOG_ERR("Failed to set CSI Format.");
			return ret;
		}

		ret = cam_set_csi(dev, fmt->pixelformat);
		if (ret) {
			LOG_ERR("Failed to configure CAM as per the CSI.");
			return ret;
		}
	}
#endif /* CONFIG_VIDEO_MIPI_CSI2_DW */

	if (config->sensor) {
		ret = video_set_format(config->sensor, ep, fmt);
		if (ret) {
			LOG_ERR("Failed to set sensor Format.");
			return ret;
		}
	}

	data->state = STATE_CONFIGURED;
	return 0;
}

static int cam_get_fmt(const struct device *dev, enum video_endpoint_id ep,
		       struct video_format *fmt)
{
	const struct video_cam_config *config = dev->config;
	struct video_cam_data *data = dev->data;
	int ret;

	if (!fmt || ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}

	if (config->sensor && !video_get_format(config->sensor, ep, fmt)) {
		ret = cam_set_fmt(dev, ep, fmt);
		if (ret) {
			return ret;
		}
	}

#ifdef CONFIG_VIDEO_MIPI_CSI2_DW
	if (config->csi_bus) {
		ret = video_set_format(config->csi_bus, ep, fmt);
		if (ret) {
			return ret;
		}
		ret = cam_set_csi(dev, fmt->pixelformat);
		if (ret) {
			return ret;
		}
	}
#endif /* CONFIG_VIDEO_MIPI_CSI2_DW */

	fmt->pixelformat = data->current_format.pixelformat;
	fmt->width = data->current_format.width;
	fmt->height = data->current_format.height;
	fmt->pitch = data->current_format.pitch;

	return 0;
}

static int cam_stream_start(const struct device *dev)
{
	const struct video_cam_config *config = dev->config;
	struct video_cam_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	struct video_buffer *vbuf;

	if (data->state == STATE_STREAMING) {
		LOG_DBG("Already streaming.");
		return 0;
	}

	if (sys_read32(regs + CAM_CTRL) & CAM_CTRL_BUSY) {
		LOG_ERR("Can\'t start stream. Already Capturing!");
		return -EBUSY;
	}

	/* Update the Video-buffer address to CPI-Controller. */
	vbuf = k_fifo_peek_head(&data->fifo_in);
	if (!vbuf) {
		LOG_ERR("No empty video-buffer. Aborting!!!");
		return -ENOBUFS;
	}

	data->curr_vid_buf = (uint32_t)vbuf->buffer;
	sys_write32(LOCAL_TO_GLOBAL(data->curr_vid_buf), regs + CAM_FRAME_ADDR);

	/* Setup the interrupts. */
	hw_enable_interrupts(regs, INTR_VSYNC | INTR_BRESP_ERR | INTR_OUTFIFO_OVERRUN |
					   INTR_INFIFO_OVERRUN | INTR_STOP);

#ifdef CONFIG_VIDEO_MIPI_CSI2_DW
	/* Start the MIPI CSI-2 IP in case the MIPI CSI is available. */
	if (config->csi_bus && video_stream_start(config->csi_bus)) {
		LOG_ERR("Failed to start CSI bus!");
		return -EIO;
	}
#endif /* CONFIG_VIDEO_MIPI_CSI2_DW */

	if (config->sensor && video_stream_start(config->sensor)) {
		LOG_ERR("Failed to start camera sensor!");
		return -EIO;
	}

	hw_cam_start_video_capture(dev);
	LOG_DBG("Stream started");

	data->state = STATE_STREAMING;

	return 0;
}

static int cam_stream_stop(const struct device *dev)
{
	const struct video_cam_config *config = dev->config;
	struct video_cam_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	uint32_t mask;
	int ret;

	if (data->state == STATE_STANDBY) {
		LOG_DBG("Already stopped streaming.");
		return 0;
	}

	if (config->sensor) {
		ret = video_stream_stop(config->sensor);
		if (ret) {
			LOG_ERR("Failed to stop camera sensor!");
			return ret;
		}
	}

#ifdef CONFIG_VIDEO_MIPI_CSI2_DW
	if (config->csi_bus && video_stream_stop(config->csi_bus)) {
		LOG_ERR("Failed to stop CSI Bus!");
		return -EIO;
	}
#endif /* CONFIG_VIDEO_MIPI_CSI2_DW */

	/* Disable Interrupts. */
	hw_disable_interrupts(regs, INTR_VSYNC | INTR_BRESP_ERR | INTR_OUTFIFO_OVERRUN |
					    INTR_INFIFO_OVERRUN | INTR_STOP);

	/* Stop the Camera sensor to dump image. */
	sys_write32(0, regs + CAM_CTRL);

	/* Set the Current buffer state to NULL*/
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

	data->state = STATE_STANDBY;

	return 0;
}

static int cam_flush(const struct device *dev, enum video_endpoint_id ep, bool cancel)
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

static int cam_enqueue(const struct device *dev, enum video_endpoint_id ep,
		       struct video_buffer *buf)
{
	struct video_cam_data *data = dev->data;
	struct video_buffer *tmp_buf = NULL;
	uint32_t to_read;
	uint32_t tmp;

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

static int cam_dequeue(const struct device *dev, enum video_endpoint_id ep,
		       struct video_buffer **buf, k_timeout_t timeout)
{
	struct video_cam_data *data = dev->data;

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

static int cam_set_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	const struct video_cam_config *config = dev->config;
	int ret = -ENOTSUP;

	if (IS_ENABLED(CONFIG_VIDEO_MIPI_CSI2_DW) && config->csi_bus) {
		ret = video_set_ctrl(config->csi_bus, cid, value);
	}

	if (config->sensor) {
		/* If some Control ID is supported and works for CSI bus, then
		 * carry forward that return status, else, check if the CID is
		 * still not supported on sensor.
		 */
		if (ret) {
			ret = video_set_ctrl(config->sensor, cid, value);
		} else {
			video_set_ctrl(config->sensor, cid, value);
		}
	}

	return ret;
}

static int cam_get_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	const struct video_cam_config *config = dev->config;
	int ret = -ENOTSUP;

	if (config->sensor) {
		ret = video_get_ctrl(config->sensor, cid, value);
	}

	return ret;
}

static int cam_get_caps(const struct device *dev, enum video_endpoint_id ep,
			struct video_caps *caps)
{
	const struct video_cam_config *config = dev->config;
	int err = -ENODEV;

	if (ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}

	if (config->sensor) {
		err = video_get_caps(config->sensor, ep, caps);
	}

	caps->min_vbuf_count = CPI_MIN_VBUF;
	return err;
}

#ifdef CONFIG_POLL
static int cam_set_signal(const struct device *dev, enum video_endpoint_id ep,
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

static const struct video_driver_api cam_driver_api = {
	.set_format = cam_set_fmt,
	.get_format = cam_get_fmt,
	.stream_start = cam_stream_start,
	.stream_stop = cam_stream_stop,
	.flush = cam_flush,
	.enqueue = cam_enqueue,
	.dequeue = cam_dequeue,
	.set_ctrl = cam_set_ctrl,
	.get_ctrl = cam_get_ctrl,
	.get_caps = cam_get_caps,
#ifdef CONFIG_POLL
	.set_signal = cam_set_signal,
#endif
};

static void video_cam_isr(const struct device *dev)
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

static int video_cam_set_config(const struct device *dev)
{
	const struct video_cam_config *config = dev->config;
	struct video_cam_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);

	if ((config->data_mode >= CPI_DATA_MODE_16_BIT) && config->is_lpcam) {
		LOG_ERR("LP-CPI controller does not support "
			"any data format >= 16-bit.");
		return -EINVAL;
	}

	if ((config->data_mode >= CPI_DATA_MODE_32_BIT) &&
	    (data->data_source == CPI_DATA_SOURCE_PARALLEL)) {
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

	reg_write_part(regs + CAM_CFG, config->data_mode, CAM_CFG_DATA_MODE_MASK,
		       CAM_CFG_DATA_MODE_SHIFT);

	return 0;
}

int video_cam_setup(const struct device *dev)
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

	ret = video_cam_set_config(dev);
	if (ret) {
		LOG_ERR("Setup of CPI controller failed. ret - %d", ret);
		return ret;
	}

	return 0;
}

static int video_cam_init(const struct device *dev)
{
	const struct video_cam_config *config = dev->config;
	struct video_cam_data *data = dev->data;
	int ret = 0;

	/*
	 * In-case there is no sensor device attached to the CPI controller,
	 * we need to abort.
	 */
	if (!config->sensor) {
		return -ENODEV;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	LOG_DBG("MMIO Address: 0x%x", (uint32_t)DEVICE_MMIO_GET(dev));

	if (config->pcfg) {
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret) {
			LOG_ERR("Failed to apply Pinctrl.");
			return ret;
		}
	}

	if ((!IS_ENABLED(CONFIG_SOC_E7_DK_RTSS_HE) && !IS_ENABLED(CONFIG_SOC_E1C_DK_RTSS_HE) &&
	     !IS_ENABLED(CONFIG_SOC_B1_DK_RTSS_HE)) &&
	    config->is_lpcam) {
		LOG_ERR("LP-CAM only accessible from HE-core. Aborting");
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_VIDEO_MIPI_CSI2_DW)) {
		data->data_source = CPI_DATA_SOURCE_CSI;
	} else {
		data->data_source = CPI_DATA_SOURCE_PARALLEL;
	}

	/* Setup ISR callback work. */
	k_work_init(&data->cb_work, isr_cb_work);
	k_work_queue_init(&data->cb_workq);
	k_work_queue_start(&data->cb_workq, isr_cb_workq, K_KERNEL_STACK_SIZEOF(isr_cb_workq),
			   K_PRIO_COOP(WORKQ_PRIORITY), NULL);
	k_thread_name_set(&data->cb_workq.thread, "cam_work_helper");

	/* Setup interrupts. */
	config->irq_config_func(dev);

	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);
	data->dev = dev;

	/* Setup the CPI-Controller hardware config. */
	ret = video_cam_setup(dev);
	if (ret) {
		LOG_ERR("Error setting up the CPI controller.");
		return ret;
	}

	LOG_DBG("irq: %d", config->irq);
	LOG_DBG("Is LP-CPI controller - %d", config->is_lpcam);
	LOG_DBG("Is Parallel interface - %d", (data->data_source == CPI_DATA_SOURCE_PARALLEL));
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

	data->state = STATE_INIT;

	return 0;
}

#if defined(CONFIG_VIDEO_MIPI_CSI2_DW)
#define CAM_PINCTRL_INIT(n)
#define CAM_PINCTRL_GET(n) (NULL)
#else
#define CAM_PINCTRL_INIT(n) PINCTRL_DT_INST_DEFINE(n)
#define CAM_PINCTRL_GET(n)  PINCTRL_DT_INST_DEV_CONFIG_GET(n)
#endif

#define CPI_DEFINE(i)                                                                              \
	CAM_PINCTRL_INIT(i);                                                                       \
	static void cam_config_func_##i(const struct device *dev);                                 \
                                                                                                   \
	static const struct video_cam_config config_##i = {                                        \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(i)),                                              \
                                                                                                   \
		.sensor = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(i, sensor)),                       \
		.csi_bus = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(i, csi_bus_if)),                  \
                                                                                                   \
		.irq = DT_INST_IRQN(i),                                                            \
		.irq_config_func = cam_config_func_##i,                                            \
		.pcfg = CAM_PINCTRL_GET(i),                                                        \
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
		.csi_halt_en = DT_INST_PROP(i, csi_halt_en),                                       \
	};                                                                                         \
                                                                                                   \
	static struct video_cam_data data_##i;                                                     \
	DEVICE_DT_INST_DEFINE(i, &video_cam_init, NULL, &data_##i, &config_##i, POST_KERNEL,       \
			      CONFIG_VIDEO_ALIF_CAM_INIT_PRIORITY, &cam_driver_api);               \
                                                                                                   \
	static void cam_config_func_##i(const struct device *dev)                                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(i), DT_INST_IRQ(i, priority), video_cam_isr,              \
			    DEVICE_DT_INST_GET(i), 0);                                             \
		irq_enable(DT_INST_IRQN(i));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(CPI_DEFINE)
