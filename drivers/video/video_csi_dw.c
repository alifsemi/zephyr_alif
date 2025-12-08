/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT snps_designware_csi

#include <zephyr/devicetree.h>

#include <zephyr/sys/device_mmio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/video.h>
#include "video_csi_dw.h"
#include <zephyr/drivers/mipi_dphy/dphy_dw.h>
#include <zephyr/drivers/video/video_alif.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(csi2_dw, CONFIG_VIDEO_LOG_LEVEL);

static int csi2_is_format_supported(uint32_t fourcc)
{
	/* TODO: Add support for RGB formats. */
	switch (fourcc) {
	case VIDEO_PIX_FMT_Y6P:
	case VIDEO_PIX_FMT_Y7P:
	case VIDEO_PIX_FMT_GREY:
	case VIDEO_PIX_FMT_Y10P:
	case VIDEO_PIX_FMT_Y12P:
	case VIDEO_PIX_FMT_Y14P:
	case VIDEO_PIX_FMT_Y16:
	case VIDEO_PIX_FMT_BGGR8:
	case VIDEO_PIX_FMT_GBRG8:
	case VIDEO_PIX_FMT_GRBG8:
	case VIDEO_PIX_FMT_RGGB8:
		return true;
	default:
		return false;
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

static void reg_write_part(uintptr_t reg, uint32_t data, uint32_t mask, uint8_t shift)
{
	uint32_t tmp = 0;

	tmp = sys_read32(reg);
	tmp &= ~(mask << shift);
	tmp |= (data & mask) << shift;
	sys_write32(tmp, reg);
}

static void csi2_dw_irq_on(uintptr_t regs)
{
	sys_write32(INT_PHY_FATAL_MASK, regs + CSI_INT_MSK_PHY_FATAL);
	sys_write32(INT_PKT_FATAL_MASK, regs + CSI_INT_MSK_PKT_FATAL);
	sys_write32(INT_PHY_MASK, regs + CSI_INT_MSK_PHY);
	sys_write32(INT_LINE_ERR_MASK, regs + CSI_INT_MSK_LINE);
	sys_write32(INT_IPI_MASK, regs + CSI_INT_MSK_IPI_FATAL);
	sys_write32(INT_BNDRY_FRAME_FATAL_MASK, regs + CSI_INT_MSK_BNDRY_FRAME_FATAL);
	sys_write32(INT_SEQ_FRAME_FATAL_MASK, regs + CSI_INT_MSK_SEQ_FRAME_FATAL);
	sys_write32(INT_CRC_FRAME_FATAL_MASK, regs + CSI_INT_MSK_CRC_FRAME_FATAL);
	sys_write32(INT_PLD_CRC_FATAL_MASK, regs + CSI_INT_MSK_PLD_CRC_FATAL);
	sys_write32(INT_DATA_ID_MASK, regs + CSI_INT_MSK_DATA_ID);
	sys_write32(INT_ECC_CORRECT_MASK, regs + CSI_INT_MSK_ECC_CORRECT);
}

static void csi2_dw_irq(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	uint32_t global_st = 0;
	uint32_t event_st = 0;
	bool reset_ipi = false;

	global_st = sys_read32(regs + CSI_INT_ST_MAIN);
	if (global_st & CSI_INT_ST_MAIN_IPI_FATAL) {
		event_st = sys_read32(regs + CSI_INT_ST_IPI_FATAL);
		LOG_ERR("Fatal Interrupt at IPI interface. status - 0x%x", event_st);
		reset_ipi = true;
	}
	if (global_st & CSI_INT_ST_MAIN_LINE) {
		event_st = sys_read32(regs + CSI_INT_ST_LINE);
		LOG_ERR("Interrupt due to error in Line construction. "
			"status - 0x%x",
			event_st);
	}
	if (global_st & CSI_INT_ST_MAIN_PHY) {
		event_st = sys_read32(regs + CSI_INT_ST_PHY);
		LOG_ERR("Fatal Interrupt caused by PHY due to TX errors. "
			"status - 0x%x",
			event_st);
	}
	if (global_st & CSI_INT_ST_MAIN_ECC_CORRECTED) {
		event_st = sys_read32(regs + CSI_INT_ST_ECC_CORRECT);
		LOG_ERR("Interrupt for header error detection and correction "
			"for specific VC-ID. status - 0x%x",
			event_st);
	}
	if (global_st & CSI_INT_ST_MAIN_DATA_ID) {
		event_st = sys_read32(regs + CSI_INT_ST_DATA_ID);
		LOG_ERR("Interrupt due to unknown data type detected in a "
			"specific VC. Packet discarded. status - 0x%x",
			event_st);
	}
	if (global_st & CSI_INT_ST_MAIN_PLD_CRC) {
		event_st = sys_read32(regs + CSI_INT_ST_PLD_CRC_FATAL);
		LOG_ERR("Fatal Interrupt due to payload checksum error."
			"status - 0x%x",
			event_st);
		reset_ipi = true;
	}
	if (global_st & CSI_INT_ST_MAIN_FRAME_CRC) {
		event_st = sys_read32(regs + CSI_INT_ST_CRC_FRAME_FATAL);
		LOG_ERR("Fatal Interrupt due to frames with at least one CRC "
			"error. status - 0x%x",
			event_st);
		reset_ipi = true;
	}
	if (global_st & CSI_INT_ST_MAIN_FRAME_SEQ) {
		event_st = sys_read32(regs + CSI_INT_ST_SEQ_FRAME_FATAL);
		LOG_ERR("Fatal Interrupt due to incorrect frame sequence for "
			"a specific VC. status - 0x%x",
			event_st);
		reset_ipi = true;
	}
	if (global_st & CSI_INT_ST_MAIN_FRAME_BNDRY) {
		event_st = sys_read32(regs + CSI_INT_ST_BNDRY_FRAME_FATAL);
		LOG_ERR("Fatal Interrupt due to mismatch of Frame Start and "
			"Frame End for a specific VC. status - 0x%x",
			event_st);
		reset_ipi = true;
	}
	if (global_st & CSI_INT_ST_MAIN_PKT) {
		event_st = sys_read32(regs + CSI_INT_ST_PKT_FATAL);
		LOG_ERR("Fatal Interrupt related to Packet construction. "
			"Packet discarded. status - 0x%x",
			event_st);
		reset_ipi = true;
	}
	if (global_st & CSI_INT_ST_MAIN_PHY_FATAL) {
		event_st = sys_read32(regs + CSI_INT_ST_PHY_FATAL);
		LOG_ERR("Fatal Interrupt due to PHY Packet discard. "
			"status - 0x%x",
			event_st);
		reset_ipi = true;
	}

	if (reset_ipi) {
		LOG_ERR("Review the Timings programmed to IPI. "
			"Resetting the IPI for now.");
		sys_clear_bits(regs + CSI_IPI_SOFTRSTN, CSI_IPI_SOFTRSTN_RSTN);
		sys_set_bits(regs + CSI_IPI_SOFTRSTN, CSI_IPI_SOFTRSTN_RSTN);
	}
}

static int csi2_dw_ipi_advanced_features(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_GET(dev);

	/*
	 * 1. Disable Frame start to trigger any sync event.
	 * 2. Enable Manual selection of packets for Line Delimiters.
	 * 3. Disable use of embedded packets for IPI sync events.
	 * 4. Disable use of blanking packets for IPI sync events.
	 * 5. Disable use of NULL packets for IPI sync events.
	 * 6. Disable use of line start packets for IPI sync events.
	 * 7. Enable video packets for IPI sync events
	 * 8. Disable IPI Data Type overwrite.
	 */
	sys_clear_bits(regs + CSI_IPI_ADV_FEATURES,
		       CSI_IPI_ADV_FEATURES_SYNC_EVENT | CSI_IPI_ADV_FEATURES_EN_EMBEDDED |
			       CSI_IPI_ADV_FEATURES_EN_BLANKING | CSI_IPI_ADV_FEATURES_EN_NULL |
			       CSI_IPI_ADV_FEATURES_EN_LINE_START |
			       CSI_IPI_ADV_FEATURES_DT_OVERWRITE);

	sys_set_bits(regs + CSI_IPI_ADV_FEATURES,
		     CSI_IPI_ADV_FEATURES_SEL_LINE_EVENT | CSI_IPI_ADV_FEATURES_EN_VIDEO);

	return 0;
}

static int csi2_dw_ipi_set_timings(const struct device *dev)
{
	struct csi2_dw_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	struct timings *timing =
		&data->time[data->current_sensor];
	uint32_t tmp;

	tmp = timing->hsa + timing->hbp + timing->hsd + timing->hact;

	/* Horizontal timing. */
	sys_write32(timing->hsa & CSI_IPI_HSA_TIME_MASK, regs + CSI_IPI_HSA_TIME);
	sys_write32(timing->hbp & CSI_IPI_HBP_TIME_MASK, regs + CSI_IPI_HBP_TIME);
	sys_write32(timing->hsd & CSI_IPI_HSD_TIME_MASK, regs + CSI_IPI_HSD_TIME);
	sys_write32(tmp & CSI_IPI_HLINE_TIME_MASK, regs + CSI_IPI_HLINE_TIME);

	/* Vertical timing. */
	sys_write32(timing->vsa & CSI_IPI_VSA_LINES_MASK, regs + CSI_IPI_VSA_LINES);
	sys_write32(timing->vbp & CSI_IPI_VBP_LINES_MASK, regs + CSI_IPI_VBP_LINES);
	sys_write32(timing->vfp & CSI_IPI_VFP_LINES_MASK, regs + CSI_IPI_VFP_LINES);
	sys_write32(timing->vact & CSI_IPI_VACTIVE_LINES_MASK, regs + CSI_IPI_VACTIVE_LINES);

	return 0;
}

static int csi2_dw_ipi_mode_config(const struct device *dev)
{
	const struct csi2_dw_config *config = dev->config;
	struct csi2_dw_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);

	/* Setup IPI mode timings. */
	if (config->ipi_mode == CSI2_IPI_MODE_TIMINGS_CTRL) {
		sys_set_bits(regs + CSI_IPI_MODE, CSI_IPI_MODE_MODE);
	} else {
		sys_clear_bits(regs + CSI_IPI_MODE, CSI_IPI_MODE_MODE);
	}

	/* Setup IPI interface type. */
	if (data->csi_cpi_settings[data->current_sensor]->ipi_ifx ==
			CSI2_IPI_MODE_16_BIT_IFX) {
		sys_set_bits(regs + CSI_IPI_MODE, CSI_IPI_MODE_COLOR_COM);
	} else {
		sys_clear_bits(regs + CSI_IPI_MODE, CSI_IPI_MODE_COLOR_COM);
	}

	sys_set_bits(regs + CSI_IPI_MODE, CSI_IPI_MODE_CUT_THROUGH);
	return 0;
}

static int csi2_dw_config_host(const struct device *dev)
{
	struct csi2_dw_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	struct dphy_csi2_settings *phy =
		&data->phy[data->current_sensor];
	/*
	 * Configuring the MIPI CSI-2 Host.
	 */
	/* Setup the number of data-lanes. */
	sys_write32(phy->num_lanes - 1, regs + CSI_N_LANES);

	/* Enable Interrupts. */
	csi2_dw_irq_on(regs);

	/*
	 * Configuring IPI.
	 */
	/* IPI Mode Configuration. */
	csi2_dw_ipi_mode_config(dev);

	/* Enable Auto memory flush of CSI by default. */
	sys_set_bits(regs + CSI_IPI_MEM_FLUSH, CSI_IPI_MEM_FLUSH_AUTO_FLUSH);

	/* Setup IPI VC-ID. */
	reg_write_part(regs + CSI_IPI_VCID, 0, CSI_IPI_VCID_VCID_MASK, CSI_IPI_VCID_VCID_SHIFT);

	/* Setup IPI Data Type. */
	reg_write_part(regs + CSI_IPI_DATA_TYPE,
		       data->csi_cpi_settings[data->current_sensor]->dt,
		       CSI_IPI_DATA_TYPE_TYPE_MASK, CSI_IPI_DATA_TYPE_TYPE_SHIFT);

	/* Setup IPI Advanced Features. */
	csi2_dw_ipi_advanced_features(dev);

	/* Setup IPI timings. */
	csi2_dw_ipi_set_timings(dev);

	return 0;
}

static int csi2_dw_phy_config(const struct device *dev)
{
	const struct csi2_dw_config *config = dev->config;
	struct csi2_dw_data *data = dev->data;
	int ret;

	ret = dphy_dw_slave_setup(config->rx_dphy, &data->phy[data->current_sensor],
			data->current_sensor);
	if (ret) {
		LOG_ERR("Failed to set-up D-PHY %s", (data->current_sensor ? "RX" : "TX as RX"));
		return ret;
	}
	return 0;
}

static int csi2_dw_validate_data(const struct device *dev)
{
	const struct csi2_dw_config *config = dev->config;
	struct csi2_dw_data *data = dev->data;
	struct timings *timing =
		&data->time[data->current_sensor];
	struct dphy_csi2_settings *phy =
		&data->phy[data->current_sensor];
	float pixclock;
	uint32_t bpp;
	uint32_t tmp;
	int ret;

	bpp = data->csi_cpi_settings[data->current_sensor]->bits_per_pixel;
	/*
	 * When camera through-put is slower than IPI, all data is transferred
	 * before new Horizontal Line is received. RAM only needs to store one
	 * line.
	 */
	if (!((timing->hact * bpp / CSI2_HOST_IPI_DWIDTH) <= CSI2_IPI_FIFO_DEPTH)) {
		LOG_ERR("Camera through-put is higher than IPI. "
			"New H-Line causes corruption to stored data.");
		return -EINVAL;
	}

	/*
	 * Balancing bandwidth by making output bandwidth 20% more than input
	 * bandwidth.
	 * pix_clk = ((rx_lane_clock_"ddr" * 2) * num_lanes)/
	 *				bits_per_pixel
	 * Balanced pixel clock for 20% more input bandwidth:
	 * balanced pixel clock = pix_clk * 1.2
	 */
	pixclock = ((phy->pll_fin << 1) * phy->num_lanes * CSI2_BANDWIDTH_SCALER) / bpp;

	LOG_DBG("pll_fin - %d, Check pixclock = %d (CSI_PIXCLK_CTRL)", phy->pll_fin,
		(uint32_t)pixclock);

	tmp = (uint32_t)pixclock;
	ret = clock_control_set_rate(config->clk_dev, config->pixclk,
			(clock_control_subsys_rate_t)tmp);
	if (ret) {
		LOG_ERR("Failed to set pixel clock rate to CPI and CSI!");
		return ret;
	}

	if (config->ipi_mode == CSI2_IPI_MODE_TIMINGS_CAM) {
		/*
		 * FV(VSYNC) comes at least 3 pixel clocks before
		 * LV(HSYNC/DATA_EN). Hence, setting HSA as 3.
		 */
		timing->hsa = 3;
		timing->hbp = 0;

		/*
		 * HSD should be such that when the PPI interface should
		 * collect last pixel and send to memory before IPI interface
		 * is collecting the last pixel of Horizontal Active Area.
		 */
		timing->hsd = ((pixclock * bpp * timing->hact) / (phy->pll_fin << 1)) -
			    (timing->hact + timing->hsa) + 1;
		timing->vsa = 0;
		timing->vbp = 0;
		timing->vfp = 0;
		timing->vact = 0;
	}
	return 0;
}

static int csi2_dw_configure(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	int ret;

	/* Enter the CSI-2 reset state. */
	sys_write32(0, regs + CSI_CSI2_RESETN);

	ret = csi2_dw_validate_data(dev);
	if (ret) {
		LOG_ERR("Invalid parameters set for CSI-2");
		return ret;
	}

	/* Setup D-PHY */
	ret = csi2_dw_phy_config(dev);
	if (ret) {
		LOG_ERR("Failed to configure PHY.");
		return ret;
	}

	ret = csi2_dw_config_host(dev);
	if (ret) {
		LOG_ERR("Failed to configure CSI Host.");
		return ret;
	}

	/* Exit CSI-2 reset state*/
	sys_write32(1, regs + CSI_CSI2_RESETN);

	return 0;
}

static int csi2_dw_stream_start(const struct device *dev)
{
	const struct csi2_dw_config *config = dev->config;
	struct csi2_dw_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	int ret = -ENODEV;

	if (data->streaming_map & BIT(data->current_sensor)) {
		LOG_DBG("Already Streaming.");
		return 0;
	}

	/* Enable CSI streaming */
	sys_set_bits(regs + CSI_IPI_MODE, CSI_IPI_MODE_ENABLE);
	LOG_DBG("Stream started");

	/* Enable CMOS sensor streaming */
	if (config->sensor[data->current_sensor]) {
		ret = video_stream_start(config->sensor[data->current_sensor]);
		if (ret) {
			LOG_ERR("Failed to start sensor stream!");
			return ret;
		}
	} else {
		LOG_ERR("Incorrect sensor selected!");
		return -ENODEV;
	}

	data->streaming_map |= BIT(data->current_sensor);

	return 0;
}

static int csi2_dw_stream_stop(const struct device *dev)
{
	const struct csi2_dw_config *config = dev->config;
	struct csi2_dw_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	int ret = -ENODEV;

	if (!(data->streaming_map & BIT(data->current_sensor))) {
		LOG_DBG("Already Stopped current sensor.");
		if (!data->streaming_map) {
			LOG_DBG("CSI has already stopped streaming.");
		}
		return 0;
	}

	if (!(data->streaming_map & ~BIT(data->current_sensor))) {
		LOG_DBG("Stream stopped from IPI");
		/* Disable CSI streaming */
		sys_clear_bits(regs + CSI_IPI_MODE, CSI_IPI_MODE_ENABLE);
	}

	/* Disable CMOS sensor streaming */
	if (config->sensor[data->current_sensor]) {
		ret = video_stream_stop(config->sensor[data->current_sensor]);
		if (ret) {
			LOG_ERR("Failed to stop sensor stream!");
			return ret;
		}
	} else {
		LOG_ERR("Incorrect sensor selected!");
		return -ENODEV;
	}

	data->streaming_map &= ~BIT(data->current_sensor);

	return 0;
}

static int csi2_dw_set_stream(const struct device *dev, bool enable)
{
	if (enable) {
		return csi2_dw_stream_start(dev);
	} else {
		return csi2_dw_stream_stop(dev);
	}
}

static int csi2_dw_set_format(const struct device *dev, enum video_endpoint_id ep,
			      struct video_format *fmt)
{
	const struct csi2_dw_config *config = dev->config;
	struct csi2_dw_data *data = dev->data;
	int32_t tmp;
	int ret;
	int i;

	if (config->sensor[data->current_sensor]) {
		ret = video_set_format(config->sensor[data->current_sensor], ep, fmt);
		if (ret) {
			LOG_ERR("Failed to set Sensor pixel format!");
			return ret;
		}
	} else {
		LOG_ERR("Invalid sesnor selected!");
		return -ENODEV;
	}

	if (!csi2_is_format_supported(fmt->pixelformat)) {
		LOG_ERR("FourCC format not supported.");
		return -ENOTSUP;
	}

	/*
	 * Check if the current set data type is the same as the requested data
	 * type.
	 */
	tmp = fourcc_to_csi_data_type(fmt->pixelformat);
	if (tmp < 0) {
		LOG_ERR("Unsupported CSI pixel format.");
		return tmp;
	}

	if (data->csi_cpi_settings[data->current_sensor] != NULL) {
		if (tmp == data->csi_cpi_settings[data->current_sensor]->dt) {
			LOG_INF("FourCC format already set.");
			return 0;
		}
	}

	for (i = 0; i < ARRAY_SIZE(data_mode_settings); i++) {
		if (data_mode_settings[i].dt == tmp) {
			break;
		}
	}

	data->csi_cpi_settings[data->current_sensor] = &data_mode_settings[i];

	data->time[data->current_sensor].hact = fmt->width;
	data->time[data->current_sensor].vact = fmt->height;

	return csi2_dw_configure(dev);
}

static int csi2_dw_get_format(const struct device *dev, enum video_endpoint_id ep,
		       struct video_format *fmt)
{
	const struct csi2_dw_config *config = dev->config;
	struct csi2_dw_data *data = dev->data;
	int ret = -ENODEV;

	if (!fmt || (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL)) {
		return -EINVAL;
	}

	if (config->sensor[data->current_sensor]) {
		ret = video_get_format(config->sensor[data->current_sensor], ep, fmt);
		if (ret) {
			LOG_ERR("Failed to get sensor format!");
		}
	} else {
		LOG_ERR("Invalid sensor selected!");
	}
	return ret;
}

static int csi2_dw_get_caps(const struct device *dev, enum video_endpoint_id ep,
			    struct video_caps *caps)
{
	const struct csi2_dw_config *config = dev->config;
	struct csi2_dw_data *data = dev->data;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		LOG_ERR("Invalid endpoint, ep - %d", ep);
		return -EINVAL;
	}

	/*
	 * Get the pipeline capabilities from sensor and
	 * send the same data to user.
	 */
	return video_get_caps(config->sensor[data->current_sensor], ep, caps);
}

static int csi2_dw_set_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	const struct csi2_dw_config *config = dev->config;
	struct csi2_dw_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	int ret = 0;

	switch (cid) {
	case VIDEO_CID_ALIF_CSI_DPHY_FREQ:
		data->phy[data->current_sensor].pll_fin = *((uint32_t *)value);
		LOG_DBG("DPHY New PLL Freq. %d", data->phy[data->current_sensor].pll_fin);
		break;
	case VIDEO_CID_ALIF_CSI_CURR_CAM:
		{
			uint8_t tmp = 0;

			tmp = *((uint8_t *)value);

			if (!(data->sensors_map & BIT(tmp))) {
				LOG_ERR("Invalid sensor provided to switch to!");
				return -EINVAL;
			}

			/* Flush IPI FIFO */
			sys_set_bits(regs + CSI_IPI_MEM_FLUSH, CSI_IPI_MEM_FLUSH_FLUSH);

			ret = dphy_dw_slave_select(config->rx_dphy, tmp);
			if (ret) {
				LOG_ERR("Failed to switch selected camera!");
				return ret;
			}
			data->current_sensor = tmp;

			if (data->current_sensor) {
				LOG_DBG("Using Standard Camera");
			} else {
				LOG_DBG("Using Selfie Camera");
			}
			break;
		}
	default:
		/* This CID is not supported with the CSI driver. */
		ret = -ENOTSUP;
	}

	/* Set control call to Sensor device. */
	if (config->sensor[data->current_sensor]) {
		/*
		 * If ret is non-zero, the CID was not supported by CSI-2 bus controller.
		 * In such a case, we depend on sensor device to get support status of CID.
		 *
		 * else ret is zero, which means that the CID was supported in CSI-2
		 * controller. Thus, we can mark the set_ctrl as success even if camera
		 * sensor does not support the CID. Hence, ignore the return status from
		 * set_ctrl call to sensor device.
		 */
		if (ret) {
			ret = video_set_ctrl(config->sensor[data->current_sensor], cid, value);
			if (ret) {
				LOG_ERR("CID Is neither supported by pipeline, nor by CSI IP.");
				return ret;
			}
		} else {
			video_set_ctrl(config->sensor[data->current_sensor], cid, value);
		}
	} else {
		LOG_ERR("Invalid sensor selected!");
		return -ENODEV;
	}

	return 0;
}

static int csi2_dw_get_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	const struct csi2_dw_config *config = dev->config;
	struct csi2_dw_data *data = dev->data;
	int ret;

	switch (cid) {
	case VIDEO_CID_ALIF_CSI_CURR_CAM:
		*((uint8_t *)value) = data->current_sensor;
		ret = 0;
		break;
	default:
		ret = -ENOTSUP;
	}

	if (!ret) {
		video_get_ctrl(config->sensor[data->current_sensor], cid, value);
		return 0;
	} else {
		return video_get_ctrl(config->sensor[data->current_sensor], cid, value);
	}
}

static DEVICE_API(video, csi2_dw_driver_api) = {
	.set_format = csi2_dw_set_format,
	.get_format = csi2_dw_get_format,
	.set_stream = csi2_dw_set_stream,
	.get_caps = csi2_dw_get_caps,
	.set_ctrl = csi2_dw_set_ctrl,
	.get_ctrl = csi2_dw_get_ctrl,
};

static int csi_enable_clocks(const struct device *dev)
{
	const struct csi2_dw_config *config = dev->config;
	int ret;

	/* Enable CSI pixel clock */
	ret = clock_control_on(config->clk_dev, config->pixclk);
	if (ret) {
		LOG_ERR("Failed to enable CSI IP!");
		return ret;
	}

	/* Enable CSI peripheral clock */
	return clock_control_on(config->clk_dev, config->csiclk);

}

static uint32_t valid_sensor_map(const struct device *const *sensors, int num_sensors)
{
	uint32_t map = 0;

	for (int i = 0; i < num_sensors; i++) {
		if (sensors[i]) {
			map |= BIT(i);
		}
	}

	return map;
}

static uint32_t valid_phy_map(const uint8_t rx_dphy_ids[], int num_dphys)
{
	uint32_t map = 0;

	for (int i = 0; i < num_dphys; i++) {
		map |= BIT(rx_dphy_ids[i]);
	}

	return map;
}

static int csi2_dw_init(const struct device *dev)
{
	const struct csi2_dw_config *config = dev->config;
	struct csi2_dw_data *data = dev->data;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	if (!config->num_dphys) {
		LOG_ERR("Provide at least one dphy");
		return -EINVAL;
	}

	ret = csi_enable_clocks(dev);
	if (ret) {
		LOG_ERR("CAM clock enable failed! Exiting! ret - %d", ret);
		return ret;
	}

	config->irq_config_func(dev);

	data->current_sensor = config->rx_dphy_ids[0];
	LOG_INF("#rx_dphy_ids: %d", config->num_dphys);

	/*
	 * There should be a one-to-one correspondence between the sensors
	 * available and the D-PHYs available.
	 */
	data->sensors_map = valid_sensor_map(config->sensor, CSI2_NUM_SENSORS) &
		valid_phy_map(config->rx_dphy_ids, config->num_dphys);

	if (!data->sensors_map) {
		LOG_ERR("Incorrect Sensor and DPHY are enabled from DTS");
		return -ENODEV;
	}

	LOG_DBG("MMIO Address: 0x%08x", (uint32_t)DEVICE_MMIO_GET(dev));

	return 0;
}

#define CSI_GET_CLK(i)                                                            \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(i, clocks),                              \
		(.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(i)),                \
		 .pixclk = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(i, \
			 pix_clk, clkid),                                         \
		 .csiclk = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(i, \
			 csi_clk, clkid),))

#define REMOTE_DEVICE(n, id) \
	DT_NODE_REMOTE_DEVICE(DT_INST_ENDPOINT_BY_ID(n, id, 0))

#define REMOTE_EP(n, pid, epid) \
	DT_NODELABEL(DT_STRING_TOKEN(DT_INST_ENDPOINT_BY_ID(n, pid, epid), remote_endpoint_label))

#define CSI2_GET_SENSOR(n, idx) \
	COND_CODE_1(DT_NODE_EXISTS(REMOTE_DEVICE(n, idx)),\
		    (DEVICE_DT_GET_OR_NULL(REMOTE_DEVICE(n, idx))),\
		    (NULL))

#define HANDLE_DPHY_ID_EACH(node_id, prop, idx) \
	DT_PHA_BY_IDX(node_id, prop, idx, id)

#define ALIF_MIPI_CSI_DEVICE(i)                                                                    \
	static void csi2_dw_config_func_##i(const struct device *dev);                             \
	static const struct csi2_dw_config config_##i = {                                          \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(i)),                                              \
		CSI_GET_CLK(i)                                                                     \
		.rx_dphy = DEVICE_DT_GET(DT_INST_PHANDLE(i, phy_if)),                              \
		.rx_dphy_ids = { DT_INST_FOREACH_PROP_ELEM_SEP(i, phy_if,                          \
				HANDLE_DPHY_ID_EACH, (,)) },                                       \
		.num_dphys = DT_INST_PROP_LEN(i, phy_if),                                          \
		.sensor[0] = CSI2_GET_SENSOR(i, 0),                                                \
		.sensor[1] = CSI2_GET_SENSOR(i, 1),                                                \
                                                                                                   \
		.irq = DT_INST_IRQN(i),                                                            \
		.irq_config_func = csi2_dw_config_func_##i,                                        \
                                                                                                   \
		.ipi_mode = DT_INST_ENUM_IDX(i, ipi_mode),                                         \
	};                                                                                         \
                                                                                                   \
	static struct csi2_dw_data data_##i = {                                                    \
                                                                                                   \
		.streaming_map = 0,                                                                \
		.sensors_map = 0,                                                                  \
		.phy[0] =  {                                                                       \
			.num_lanes = COND_CODE_1(DT_NODE_HAS_PROP(REMOTE_EP(i, 0, 0),              \
						data_lanes),                                       \
					(DT_PROP_LEN(REMOTE_EP(i, 0, 0), data_lanes)),             \
					(DT_INST_PROP(i, data_lanes1))),                           \
			.pll_fin = COND_CODE_1(DT_NODE_HAS_PROP(REMOTE_EP(i, 0, 0),                \
						link_frequencies),                                 \
					(DT_PROP_LAST(REMOTE_EP(i, 0, 0), link_frequencies)),      \
					(DT_INST_PROP(i, rx_ddr_clk1))),                           \
		},                                                                                 \
                                                                                                   \
		.phy[1] =  {                                                                       \
			.num_lanes = COND_CODE_1(DT_NODE_HAS_PROP(REMOTE_EP(i, 1, 0),              \
						data_lanes),                                       \
					(DT_PROP_LEN(REMOTE_EP(i, 1, 0), data_lanes)),             \
					(DT_INST_PROP(i, data_lanes1))),                           \
			.pll_fin = COND_CODE_1(DT_NODE_HAS_PROP(REMOTE_EP(i, 1, 0),                \
						link_frequencies),                                 \
					(DT_PROP_LAST(REMOTE_EP(i, 1, 0), link_frequencies)),      \
					(DT_INST_PROP(i, rx_ddr_clk2))),                           \
		},                                                                                 \
                                                                                                   \
		.time[0] = {                                                                       \
			.hsa = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_hsa),                      \
					   (DT_INST_PROP(i, csi_hsa)),                             \
					   (3)),                                                   \
			.hbp = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_hbp),                      \
					   (DT_INST_PROP(i, csi_hbp)),                             \
					   (0)),                                                   \
			.hsd = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_hsd),                      \
					   (DT_INST_PROP(i, csi_hsd)),                             \
					   (0)),                                                   \
			.hact = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_hact),                    \
					    (DT_INST_PROP(i, csi_hact)),                           \
					    (0)),                                                  \
			.vsa = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_vsa),                      \
					   (DT_INST_PROP(i, csi_vsa)),                             \
					   (0)),                                                   \
			.vbp = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_vbp),                      \
					   (DT_INST_PROP(i, csi_vbp)),                             \
					   (0)),                                                   \
			.vfp = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_vfp),                      \
					   (DT_INST_PROP(i, csi_vfp)),                             \
					   (0)),                                                   \
			.vact = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_vact),                    \
					    (DT_INST_PROP(i, csi_vact)),                           \
					    (0)),                                                  \
		},                                                                                 \
		.time[1] = {                                                                       \
			.hsa = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_hsa),                      \
					   (DT_INST_PROP(i, csi_hsa)),                             \
					   (3)),                                                   \
			.hbp = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_hbp),                      \
					   (DT_INST_PROP(i, csi_hbp)),                             \
					   (0)),                                                   \
			.hsd = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_hsd),                      \
					   (DT_INST_PROP(i, csi_hsd)),                             \
					   (0)),                                                   \
			.hact = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_hact),                    \
					    (DT_INST_PROP(i, csi_hact)),                           \
					    (0)),                                                  \
			.vsa = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_vsa),                      \
					   (DT_INST_PROP(i, csi_vsa)),                             \
					   (0)),                                                   \
			.vbp = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_vbp),                      \
					   (DT_INST_PROP(i, csi_vbp)),                             \
					   (0)),                                                   \
			.vfp = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_vfp),                      \
					   (DT_INST_PROP(i, csi_vfp)),                             \
					   (0)),                                                   \
			.vact = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_vact),                    \
					    (DT_INST_PROP(i, csi_vact)),                           \
					    (0)),                                                  \
		},                                                                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(i, &csi2_dw_init, NULL, &data_##i, &config_##i, POST_KERNEL,         \
			      CONFIG_VIDEO_MIPI_CSI2_DW_INIT_PRIORITY, &csi2_dw_driver_api);       \
                                                                                                   \
	static void csi2_dw_config_func_##i(const struct device *dev)                              \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(i), DT_INST_IRQ(i, priority), csi2_dw_irq,                \
			    DEVICE_DT_INST_GET(i), 0);                                             \
		irq_enable(DT_INST_IRQN(i));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(ALIF_MIPI_CSI_DEVICE)
