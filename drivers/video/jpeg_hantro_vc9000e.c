/*
 * Copyright (c) 2026 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 *
 * VeriSilicon Hantro VC9000E JPEG Hardware Encoder Driver for Zephyr RTOS
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/video/video_alif.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/cache.h>
#include <string.h>

#include <jpeg_hantro_vc9000e_sw.h>
#include "jpeg_hantro_vc9000e_regs.h"

LOG_MODULE_REGISTER(jpeg_hantro_vc9000e, CONFIG_VIDEO_LOG_LEVEL);

#define DT_DRV_COMPAT verisilicon_hantro_vc9000e_jpeg

/* JPEG encoder alignment = 16 pixels */
#define JPEG_ENC_ALIGNMENT      16

/* Device configuration structure */
struct jpeg_hantro_vc9000e_config {
	DEVICE_MMIO_ROM;
	void (*irq_config_func)(const struct device *dev);
	const struct device   *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint8_t                max_burst_length;
	uint8_t                axi_wr_outstanding;
	uint8_t                axi_rd_outstanding;
	uint16_t               default_quality;
};

/* Device runtime data */
struct jpeg_hantro_vc9000e_data {
	DEVICE_MMIO_RAM;
	struct   k_mutex lock;
	struct   k_sem encode_sem;
	struct   video_format fmt;
	struct   video_buffer *current_buf;
	void     *input_buffer;
	bool     streaming;
	uint16_t quality;
	uint32_t encoding_width;
	uint32_t encoding_height;
	uint32_t encoding_size;
	int      encoding_error;
	uint32_t header_size;
	struct jpeg_header_info header_info;
};

/**
 * @brief Write a value to a JPEG encoder register.
 *
 * @param dev Pointer to the device structure.
 * @param offset Register offset from the base address.
 * @param value Value to write.
 */
static inline void jpeg_write_reg(const struct device *dev, uint32_t offset,
				   uint32_t value)
{
	uintptr_t base = DEVICE_MMIO_GET(dev);

	sys_write32(value, base + offset);
}

/**
 * @brief Read a value from a JPEG encoder register.
 *
 * @param dev Pointer to the device structure.
 * @param offset Register offset from the base address.
 *
 * @return Register value.
 */
static inline uint32_t jpeg_read_reg(const struct device *dev, uint32_t offset)
{
	uintptr_t base = DEVICE_MMIO_GET(dev);

	return sys_read32(base + offset);
}

/**
 * @brief Modify specific bits in a JPEG encoder register.
 *
 * @param dev Pointer to the device structure.
 * @param offset Register offset from the base address.
 * @param clear_mask Bits to clear before setting.
 * @param set_mask Bits to set after clearing.
 */
static inline void jpeg_modify_reg(const struct device *dev, uint32_t offset,
				    uint32_t clear_mask, uint32_t set_mask)
{
	uint32_t val = jpeg_read_reg(dev, offset);

	val = (val & ~clear_mask) | set_mask;
	jpeg_write_reg(dev, offset, val);
}

/**
 * @brief Configure JPEG compression quality factor.
 *
 * Computes quantization tables from the quality value and programs them
 * into the hardware registers.
 *
 * @param dev Pointer to the device structure.
 * @param quality Quality factor (1-100).
 *
 * @return 0 on success.
 */
static int jpeg_quality_config(const struct device *dev, uint16_t quality)
{
	struct jpeg_hantro_vc9000e_data *data = dev->data;
	int scale_factor = jpeg_qf_scaling(quality);
	uintptr_t base = DEVICE_MMIO_GET(dev);

	jpeg_calc_q_table(scale_factor);
	jpeg_set_q_table(base);

	data->quality = quality;
	return 0;
}

/**
 * @brief Initialize the JPEG encoder hardware.
 *
 * Verifies the hardware ID, configures AXI bus parameters, burst length,
 * and outstanding transaction limits.
 *
 * @param dev Pointer to the device structure.
 *
 * @return 0 on success, negative errno on failure.
 */
static int jpeg_hw_init(const struct device *dev)
{
	const struct jpeg_hantro_vc9000e_config *config = dev->config;
	uint32_t hw_id, hw_ver;

	hw_id = jpeg_read_reg(dev, JPEG_SWREG0_OFFSET);
	if (hw_id != JPEG_HW_ID) {
		LOG_ERR("JPEG hardware not found (ID: 0x%08x)", hw_id);
		return -ENODEV;
	}

	hw_ver = jpeg_read_reg(dev, JPEG_SWREG80_OFFSET);
	if (hw_ver != JPEG_HW_VERSION) {
		LOG_WRN("JPEG hardware version mismatch (Ver: 0x%08x)", hw_ver);
	}

	/* Assert the software reset bit */
	jpeg_write_reg(dev, JPEG_SWREG1_OFFSET, JPEG_IRQ_TYPE_SW_RESET);
	k_sleep(K_MSEC(1));

	jpeg_modify_reg(dev, JPEG_SWREG4_OFFSET, JPEG_SW_ENC_MODE_MASK,
			JPEG_SW_ENC_MODE_JPEG << JPEG_SW_ENC_MODE_POS);

	jpeg_modify_reg(dev, JPEG_SWREG81_OFFSET, JPEG_MAX_BURST_MASK,
			config->max_burst_length << JPEG_MAX_BURST_POS);

	jpeg_modify_reg(dev, JPEG_SWREG246_OFFSET, JPEG_AXI_WR_OUTSTANDING_MASK,
			config->axi_wr_outstanding << JPEG_AXI_WR_OUTSTANDING_POS);

	jpeg_modify_reg(dev, JPEG_SWREG261_OFFSET, JPEG_AXI_RD_OUTSTANDING_MASK,
			config->axi_rd_outstanding << JPEG_AXI_RD_OUTSTANDING_POS);

	jpeg_modify_reg(dev, JPEG_SWREG349_OFFSET, JPEG_SBI_WAIT_FRAME_START,
			JPEG_SBI_WAIT_FRAME_START);

	LOG_DBG("JPEG encoder initialized (ID: 0x%08x, Ver: 0x%08x)", hw_id, hw_ver);
	return 0;
}

/**
 * @brief Set the video format for encoding.
 *
 * Validates the requested pixel format and dimensions, configures the
 * hardware picture size, fill values, and YUV420 mode registers.
 *
 * @param dev Pointer to the device structure.
 * @param ep Video endpoint identifier.
 * @param fmt Pointer to the video format structure.
 *
 * @return 0 on success, negative errno on failure.
 */
static int jpeg_hantro_vc9000e_set_format(const struct device *dev,
					   enum video_endpoint_id ep,
					   struct video_format *fmt)
{
	struct jpeg_hantro_vc9000e_data *data = dev->data;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	if (!fmt) {
		LOG_ERR("Invalid frame format");
		return -EINVAL;

	}

	if (fmt->width < CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MIN_SIZE ||
	    fmt->height < CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MIN_SIZE) {
		LOG_ERR("Image too small: %ux%u (min: %u)",
			fmt->width, fmt->height, CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MIN_SIZE);
		return -EINVAL;
	}

	if (fmt->width > CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MAX_WIDTH ||
	    fmt->height > CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MAX_HEIGHT) {
		LOG_ERR("Image too large: %ux%u (max: %ux%u)",
			fmt->width, fmt->height,
			CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MAX_WIDTH,
			CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MAX_HEIGHT);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	switch (fmt->pixelformat) {
	case VIDEO_PIX_FMT_NV12:
		 /* Disable chroma swap (CbCr) in semiplanar input format*/
		jpeg_modify_reg(dev, JPEG_SWREG45_OFFSET,
				JPEG_CHROMA_SWAP_MASK, ~JPEG_CHROMA_SWAP);
		break;
	case VIDEO_PIX_FMT_NV21:
		/* Enable chroma swap (CrCb) in semiplanar input format*/
		jpeg_modify_reg(dev, JPEG_SWREG45_OFFSET,
				JPEG_CHROMA_SWAP_MASK, JPEG_CHROMA_SWAP);
		break;
	default:
		LOG_ERR("Unsupported pixel format: 0x%x", fmt->pixelformat);
		k_mutex_unlock(&data->lock);
		return -ENOTSUP;
	}

	memcpy(&data->fmt, fmt, sizeof(struct video_format));
	/* The encoding width and height are 16-pixels aligned */
	data->encoding_width  = ROUND_UP(fmt->width, JPEG_ENC_ALIGNMENT);
	data->encoding_height = ROUND_UP(fmt->height, JPEG_ENC_ALIGNMENT);

	uint16_t width  = data->encoding_width >> JPEG_PIC_WH_PIXEL_SHIFT;
	uint16_t height = data->encoding_height >> JPEG_PIC_WH_PIXEL_SHIFT;

	jpeg_modify_reg(dev, JPEG_SWREG5_OFFSET, JPEG_PIC_WIDTH_MASK,
			(width & JPEG_PIC_WH_MASK) << JPEG_PIC_WIDTH_POS);
	jpeg_modify_reg(dev, JPEG_SWREG249_OFFSET, JPEG_PIC_WIDTH_MSB_MASK,
			(width >> JPEG_PIC_WH_FIELD_WIDTH) << JPEG_PIC_WIDTH_MSB_POS);
	jpeg_modify_reg(dev, JPEG_SWREG5_OFFSET, JPEG_PIC_HEIGHT_MASK,
			(height & JPEG_PIC_WH_MASK) << JPEG_PIC_HEIGHT_POS);
	jpeg_modify_reg(dev, JPEG_SWREG249_OFFSET, JPEG_PIC_HEIGHT_MSB_MASK,
			(height >> JPEG_PIC_WH_FIELD_WIDTH) << JPEG_PIC_HEIGHT_MSB_POS);

	uint8_t xfill = (fmt->width % JPEG_ENC_ALIGNMENT) ?
			(JPEG_ENC_ALIGNMENT - fmt->width % JPEG_ENC_ALIGNMENT) /
			JPEG_YUV420_CHROMA_DIV : 0;
	uint8_t yfill = (fmt->height % JPEG_ENC_ALIGNMENT) ?
			(JPEG_ENC_ALIGNMENT - fmt->height % JPEG_ENC_ALIGNMENT) : 0;

	jpeg_modify_reg(dev, JPEG_SWREG38_OFFSET, JPEG_XFILL_MASK,
			(xfill & JPEG_XFILL_FIELD_MASK) << JPEG_XFILL_POS);
	jpeg_modify_reg(dev, JPEG_SWREG38_OFFSET, JPEG_YFILL_MASK,
			(yfill & JPEG_YFILL_FIELD_MASK) << JPEG_YFILL_POS);
	jpeg_modify_reg(dev, JPEG_SWREG193_OFFSET, JPEG_XFILL_MSB_MASK,
			(xfill >> JPEG_XFILL_FIELD_WIDTH) << JPEG_XFILL_MSB_POS);
	jpeg_modify_reg(dev, JPEG_SWREG193_OFFSET, JPEG_YFILL_MSB_MASK,
			(yfill >> JPEG_YFILL_FIELD_WIDTH) << JPEG_YFILL_MSB_POS);

	/* Set mode to 4:2:0 */
	jpeg_modify_reg(dev, JPEG_SWREG18_OFFSET, JPEG_MODE_MASK, JPEG_MODE_420);
	jpeg_modify_reg(dev, JPEG_SWREG20_OFFSET, JPEG_CODING_MODE_MASK,
			JPEG_CODING_MODE_420);
	jpeg_modify_reg(dev, JPEG_SWREG38_OFFSET, JPEG_INPUT_FORMAT_MASK,
			JPEG_INPUT_FORMAT_YUV420SP << JPEG_INPUT_FORMAT_POS);

	k_mutex_unlock(&data->lock);

	return 0;
}

/**
 * @brief Get the current video format.
 *
 * @param dev Pointer to the device structure.
 * @param ep Video endpoint identifier.
 * @param fmt Pointer to the video format structure to fill.
 *
 * @return 0 on success, negative errno on failure.
 */
static int jpeg_hantro_vc9000e_get_format(const struct device *dev,
					   enum video_endpoint_id ep,
					   struct video_format *fmt)
{
	struct jpeg_hantro_vc9000e_data *data = dev->data;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	if (!fmt) {
		LOG_ERR("Invalid frame format");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	memcpy(fmt, &data->fmt, sizeof(struct video_format));
	k_mutex_unlock(&data->lock);

	return 0;
}

/**
 * @brief Prepare hardware registers and trigger JPEG encoding.
 *
 * Computes the header size, sets input/output DMA addresses, configures
 * stride registers, generates the JPEG header, flushes caches, and
 * starts the hardware encoder.
 *
 * @param dev Pointer to the device structure.
 */
static void jpeg_start_encode(const struct device *dev)
{
	struct jpeg_hantro_vc9000e_data *data = dev->data;
	struct video_buffer *buf = data->current_buf;
	void *input_ptr = data->input_buffer;
	uint32_t stride;
	uint32_t chroma_offset;
	uint32_t input_size;

	/* Output buffer: JPEG header + compressed data */
	uint8_t *output_ptr = (uint8_t *)buf->buffer + data->header_size;

	/* Set output stream address (compressed data) */
	jpeg_write_reg(dev, JPEG_SWREG8_OFFSET, (uint32_t)output_ptr);

	/* Set input luma address */
	jpeg_write_reg(dev, JPEG_SWREG12_OFFSET, (uint32_t)input_ptr);

	stride = data->fmt.pitch;
	/* Set input chroma address for YUV420 formats */
	chroma_offset = stride * data->fmt.height;

	/* Cb offset for YUV420 */
	jpeg_write_reg(dev, JPEG_SWREG13_OFFSET,
			(uint32_t)input_ptr + chroma_offset);
	/* Cr offset is zero for YUV420 */
	jpeg_write_reg(dev, JPEG_SWREG14_OFFSET, 0);

	/* Flush input buffer (YUV420 format) cache */
	input_size = JPEG_YUV420_FRAME_SIZE(stride, data->fmt.height);
	(void)sys_cache_data_flush_and_invd_range(input_ptr, input_size);

	/* Set stride configuration */
	jpeg_modify_reg(dev, JPEG_SWREG20_OFFSET, JPEG_ROWLENGTH_MASK,
			(stride & JPEG_ROWLENGTH_FIELD_MASK) << JPEG_ROWLENGTH_POS);
	jpeg_modify_reg(dev, JPEG_SWREG249_OFFSET, JPEG_ROWLENGTH_MSB_MASK,
			(stride >> JPEG_ROWLENGTH_FIELD_WIDTH) << JPEG_ROWLENGTH_MSB_POS);

	jpeg_modify_reg(dev, JPEG_SWREG210_OFFSET, JPEG_LUMA_STRIDE_MASK,
			stride << JPEG_LUMA_STRIDE_POS);
	jpeg_modify_reg(dev, JPEG_SWREG211_OFFSET, JPEG_CHROMA_STRIDE_MASK,
			stride << JPEG_CHROMA_STRIDE_POS);

	/* Generate JPEG header in output buffer */
	data->header_info.buffer = buf->buffer;
	data->header_info.width  = data->fmt.width;
	data->header_info.height = data->fmt.height;
	data->header_info.num_components = JPEG_YUV420_NUM_COMPONENTS;

	jpeg_header_generation(data->header_info);

	/* Flush output buffer header cache */
	(void)sys_cache_data_flush_and_invd_range(buf->buffer, data->header_size);

	/* Set output buffer size */
	jpeg_write_reg(dev, JPEG_SWREG9_OFFSET, buf->bytesused);

	/* Reset error state and trigger encoding */
	data->encoding_error = 0;
	jpeg_modify_reg(dev, JPEG_SWREG5_OFFSET, JPEG_ENC_ENABLE, JPEG_ENC_ENABLE);
}

/**
 * @brief Enqueue a video buffer for JPEG encoding.
 *
 * If streaming is already active, encoding is triggered immediately.
 *
 * @param dev Pointer to the device structure.
 * @param ep Video endpoint identifier.
 * @param buf Pointer to the video buffer to enqueue.
 *
 * @return 0 on success, negative errno on failure.
 */
static int jpeg_hantro_vc9000e_enqueue(const struct device *dev,
					enum video_endpoint_id ep,
					struct video_buffer *buf)
{
	struct jpeg_hantro_vc9000e_data *data = dev->data;

	if (!buf) {
		LOG_ERR("Invalid video buffer");
		return -EINVAL;
	}

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	if (data->current_buf != NULL) {
		LOG_ERR("Encoder busy");
		return -EBUSY;
	}

	if (data->input_buffer == NULL) {
		LOG_ERR("Input buffer not set");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	data->current_buf = buf;

	/* If already streaming, trigger encoding immediately */
	if (data->streaming) {
		jpeg_start_encode(dev);
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

/**
 * @brief Dequeue an encoded JPEG buffer.
 *
 * Waits for the encoder to signal completion, checks for encoding errors,
 * invalidates the CPU cache for the DMA-written compressed data region,
 * and returns the buffer with the final byte count.
 *
 * @param dev Pointer to the device structure.
 * @param ep Video endpoint identifier.
 * @param buf Double pointer to receive the dequeued video buffer.
 * @param timeout Maximum time to wait for encoding completion.
 *
 * @return 0 on success, -EAGAIN on timeout, or negative errno on encode error.
 */
static int jpeg_hantro_vc9000e_dequeue(const struct device *dev,
					enum video_endpoint_id ep,
					struct video_buffer **buf,
					k_timeout_t timeout)
{
	struct jpeg_hantro_vc9000e_data *data = dev->data;
	int ret;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	ret = k_sem_take(&data->encode_sem, timeout);
	if (ret != 0) {
		LOG_ERR("Dequeue timeout");
		return -EAGAIN;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	*buf = data->current_buf;

	if (data->encoding_error != 0) {
		int err = data->encoding_error;

		data->current_buf = NULL;
		k_mutex_unlock(&data->lock);
		return err;
	}

	/* Invalidate CPU cache for DMA-written compressed data */
	(void)sys_cache_data_invd_range(
		(uint8_t *)(*buf)->buffer + data->header_size,
		data->encoding_size - data->header_size);

	(*buf)->bytesused = data->encoding_size;
	(*buf)->timestamp = k_uptime_get_32();

	data->current_buf = NULL;

	k_mutex_unlock(&data->lock);

	return 0;
}

/**
 * @brief Enable or disable the JPEG encoding stream.
 *
 * When enabling, configures interrupt masks and triggers encoding if a
 * buffer is already enqueued. When disabling, clears interrupt masks
 * and stops the encoder.
 *
 * @param dev Pointer to the device structure.
 * @param enable true to start streaming, false to stop.
 *
 * @return 0 on success, -EALREADY if already in the requested state.
 */
static int jpeg_hantro_vc9000e_set_stream(const struct device *dev, bool enable)
{
	struct jpeg_hantro_vc9000e_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (enable) {
		if (data->streaming) {
			k_mutex_unlock(&data->lock);
			return -EALREADY;
		}
		jpeg_modify_reg(dev, JPEG_SWREG1_OFFSET,
		JPEG_IRQ_STATUS_MASK, JPEG_IRQ_EN_MASK);
		data->streaming = true;

		/* If a buffer was already enqueued, trigger encoding now */
		if (data->current_buf != NULL && data->input_buffer != NULL) {
			jpeg_start_encode(dev);
		}

	} else {
		if (!data->streaming) {
			k_mutex_unlock(&data->lock);
			return -EALREADY;
		}
		jpeg_modify_reg(dev, JPEG_SWREG1_OFFSET, JPEG_IRQ_EN_MASK, 0);
		jpeg_write_reg(dev, JPEG_SWREG5_OFFSET, 0);
		data->streaming = false;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

/**
 * @brief Set a driver-specific control parameter.
 *
 * Supported controls:
 * - VIDEO_CID_JPEG_COMPRESSION_QUALITY: Set compression quality (1-100).
 * - VIDEO_CID_JPEG_INPUT_BUFFER: Set the input buffer address for encoding.
 *
 * @param dev Pointer to the device structure.
 * @param cid Control identifier.
 * @param value Pointer to the control value.
 *
 * @return 0 on success, -ENOTSUP for unsupported controls.
 */
static int jpeg_hantro_vc9000e_set_ctrl(const struct device *dev,
					 unsigned int cid, void *value)
{
	struct jpeg_hantro_vc9000e_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	switch (cid) {
	case VIDEO_CID_JPEG_COMPRESSION_QUALITY: {
		uint16_t quality = *(uint16_t *)value;
		int ret = jpeg_quality_config(dev, quality);

		k_mutex_unlock(&data->lock);
		return ret;
	}
	case VIDEO_CID_JPEG_INPUT_BUFFER:
		/* Set input buffer address for encoding */
		data->input_buffer = value;
		break;
	default:
		k_mutex_unlock(&data->lock);
		return -ENOTSUP;
	}

	k_mutex_unlock(&data->lock);
	return 0;
}

/**
 * @brief Get a driver-specific control parameter.
 *
 * Supported controls:
 * - VIDEO_CID_JPEG_COMPRESSION_QUALITY: Get the current compression quality.
 * - VIDEO_CID_JPEG_INPUT_BUFFER: Get the input buffer address.
 *
 * @param dev Pointer to the device structure.
 * @param cid Control identifier.
 * @param value Pointer to store the control value.
 *
 * @return 0 on success, -ENOTSUP for unsupported controls.
 */
static int jpeg_hantro_vc9000e_get_ctrl(const struct device *dev,
					 unsigned int cid, void *value)
{
	struct jpeg_hantro_vc9000e_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	switch (cid) {
	case VIDEO_CID_JPEG_COMPRESSION_QUALITY:
		*(uint16_t *)value = data->quality;
		break;
	case VIDEO_CID_JPEG_INPUT_BUFFER:
		/* Get input buffer address */
		value = data->input_buffer;
		break;
	default:
		k_mutex_unlock(&data->lock);
		return -ENOTSUP;
	}

	k_mutex_unlock(&data->lock);
	return 0;
}

static const struct video_format_cap jpeg_hantro_vc9000e_format_caps[] = {
	{
		.pixelformat = VIDEO_PIX_FMT_NV12,
		.width_min   = CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MIN_SIZE,
		.width_max   = CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MAX_WIDTH,
		.height_min  = CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MIN_SIZE,
		.height_max  = CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MAX_HEIGHT,
		.width_step  = JPEG_ENC_ALIGNMENT,
		.height_step = JPEG_ENC_ALIGNMENT,
	},
	{
		.pixelformat = VIDEO_PIX_FMT_NV21,
		.width_min   = CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MIN_SIZE,
		.width_max   = CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MAX_WIDTH,
		.height_min  = CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MIN_SIZE,
		.height_max  = CONFIG_VIDEO_JPEG_HANTRO_VC9000E_MAX_HEIGHT,
		.width_step  = JPEG_ENC_ALIGNMENT,
		.height_step = JPEG_ENC_ALIGNMENT,
	},
	{ 0 }
};

/**
 * @brief Get the encoder capabilities.
 *
 * Returns the list of supported pixel formats and resolution ranges.
 *
 * @param dev Pointer to the device structure.
 * @param ep Video endpoint identifier.
 * @param caps Pointer to the capabilities structure to fill.
 *
 * @return 0 on success, -EINVAL for invalid endpoint.
 */
static int jpeg_hantro_vc9000e_get_caps(const struct device *dev,
					 enum video_endpoint_id ep,
					 struct video_caps *caps)
{
	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	caps->format_caps = jpeg_hantro_vc9000e_format_caps;
	return 0;
}

/**
 * @brief JPEG encoder interrupt service routine.
 *
 * Reads the interrupt status register and handles frame-ready, bus error,
 * buffer-full, and timeout conditions. Sets the encoding error code and
 * signals the completion semaphore.
 *
 * @param dev Pointer to the device structure.
 */
static void jpeg_hantro_vc9000e_isr(const struct device *dev)
{
	struct jpeg_hantro_vc9000e_data *data = dev->data;
	uint32_t status;

	status = jpeg_read_reg(dev, JPEG_SWREG1_OFFSET);

	jpeg_write_reg(dev, JPEG_SWREG1_OFFSET, status);

	/* Just return if the expected interrupt did not occur */
	if ((JPEG_IRQ_STATUS_MASK & status) == 0) {
		return;
	}

	if (status & JPEG_FRAME_RDY_STATUS) {
		data->encoding_size = jpeg_read_reg(dev, JPEG_SWREG9_OFFSET) +
				      data->header_size;
		data->encoding_error = 0;
		k_sem_give(&data->encode_sem);
	}

	if (status & JPEG_BUS_ERROR_STATUS) {
		LOG_ERR("JPEG bus error");
		data->encoding_error = -EIO;
		k_sem_give(&data->encode_sem);
	}

	if (status & JPEG_BUFFER_FULL) {
		LOG_ERR("JPEG buffer full");
		data->encoding_error = -ENOSPC;
		k_sem_give(&data->encode_sem);
	}

	if (status & JPEG_TIMEOUT) {
		LOG_ERR("JPEG timeout");
		data->encoding_error = -ETIMEDOUT;
		k_sem_give(&data->encode_sem);
	}
}

/* Video driver API */
static const struct video_driver_api jpeg_hantro_vc9000e_driver_api = {
	.set_format = jpeg_hantro_vc9000e_set_format,
	.get_format = jpeg_hantro_vc9000e_get_format,
	.enqueue = jpeg_hantro_vc9000e_enqueue,
	.dequeue = jpeg_hantro_vc9000e_dequeue,
	.set_stream = jpeg_hantro_vc9000e_set_stream,
	.set_ctrl = jpeg_hantro_vc9000e_set_ctrl,
	.get_ctrl = jpeg_hantro_vc9000e_get_ctrl,
	.get_caps = jpeg_hantro_vc9000e_get_caps,
};

/**
 * @brief Initialize the JPEG encoder device.
 *
 * Maps MMIO registers, initializes synchronization primitives,
 * performs hardware initialization, sets default quality,
 * and configures the interrupt.
 *
 * @param dev Pointer to the device structure.
 *
 * @return 0 on success, negative errno on failure.
 */
static int jpeg_hantro_vc9000e_init(const struct device *dev)
{
	const struct jpeg_hantro_vc9000e_config *config = dev->config;
	struct jpeg_hantro_vc9000e_data *data = dev->data;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	if (config->clock_dev != NULL) {
		if (!device_is_ready(config->clock_dev)) {
			LOG_ERR("Clock controller not ready");
			return -ENODEV;
		}
		ret = clock_control_on(config->clock_dev,
				       config->clock_subsys);
		if (ret < 0) {
			LOG_ERR("Failed to enable JPEG clock: %d", ret);
			return ret;
		}
	}

	k_sem_init(&data->encode_sem, 0, 1);
	k_mutex_init(&data->lock);

	data->streaming = false;
	data->current_buf = NULL;
	data->input_buffer = NULL;
	data->header_size = CONFIG_VIDEO_JPEG_HANTRO_VC9000E_HEADER_SIZE;

	ret = jpeg_hw_init(dev);
	if (ret != 0) {
		LOG_ERR("Hardware initialization failed: %d", ret);
		return ret;
	}

	ret = jpeg_quality_config(dev, config->default_quality);
	if (ret != 0) {
		LOG_ERR("Quality configuration failed: %d", ret);
		return ret;
	}

	config->irq_config_func(dev);

	LOG_INF("VeriSilicon Hantro VC9000E JPEG encoder initialized");
	return 0;
}

/* Device instantiation macro */
#define JPEG_HANTRO_VC9000E_INIT(inst)							\
	static void jpeg_hantro_vc9000e_irq_config_##inst(const struct device *dev)	\
	{										\
		IRQ_CONNECT(DT_INST_IRQN(inst),						\
			    DT_INST_IRQ(inst, priority),				\
			    jpeg_hantro_vc9000e_isr,					\
			    DEVICE_DT_INST_GET(inst),					\
			    0);								\
		irq_enable(DT_INST_IRQN(inst));						\
	}										\
											\
	static struct jpeg_hantro_vc9000e_data jpeg_hantro_vc9000e_data_##inst;	\
											\
	static const struct jpeg_hantro_vc9000e_config					\
		jpeg_hantro_vc9000e_config_##inst = {					\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),				\
		.irq_config_func = jpeg_hantro_vc9000e_irq_config_##inst,		\
		.clock_dev = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, clocks),		\
			(DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst))), (NULL)),		\
		.clock_subsys = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, clocks),	\
			((clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, clkid)),	\
			((clock_control_subsys_t)0)),					\
		.max_burst_length = DT_INST_PROP(inst, max_burst_length),		\
		.axi_wr_outstanding = DT_INST_PROP(inst, axi_wr_outstanding),		\
		.axi_rd_outstanding = DT_INST_PROP(inst, axi_rd_outstanding),		\
		.default_quality = DT_INST_PROP(inst, quality_factor),			\
	};										\
											\
	DEVICE_DT_INST_DEFINE(inst,							\
				jpeg_hantro_vc9000e_init,				\
				NULL,							\
				&jpeg_hantro_vc9000e_data_##inst,			\
				&jpeg_hantro_vc9000e_config_##inst,			\
				POST_KERNEL,						\
				CONFIG_VIDEO_INIT_PRIORITY,				\
				&jpeg_hantro_vc9000e_driver_api);

DT_INST_FOREACH_STATUS_OKAY(JPEG_HANTRO_VC9000E_INIT)
