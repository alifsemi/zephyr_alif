/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT snps_designware_csi

#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(csi2_dw, CONFIG_VIDEO_LOG_LEVEL);

#include <zephyr/sys/device_mmio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/video.h>
#include "video_csi_dw.h"
#include <zephyr/drivers/mipi_dphy/dphy_dw.h>
#include <zephyr/drivers/video/video_alif.h>

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
	uint32_t tmp;

	tmp = data->hsa + data->hbp + data->hsd + data->hact;

	/* Horizontal timing. */
	sys_write32(data->hsa & CSI_IPI_HSA_TIME_MASK, regs + CSI_IPI_HSA_TIME);
	sys_write32(data->hbp & CSI_IPI_HBP_TIME_MASK, regs + CSI_IPI_HBP_TIME);
	sys_write32(data->hsd & CSI_IPI_HSD_TIME_MASK, regs + CSI_IPI_HSD_TIME);
	sys_write32(tmp & CSI_IPI_HLINE_TIME_MASK, regs + CSI_IPI_HLINE_TIME);

	/* Vertical timing. */
	sys_write32(data->vsa & CSI_IPI_VSA_LINES_MASK, regs + CSI_IPI_VSA_LINES);
	sys_write32(data->vbp & CSI_IPI_VBP_LINES_MASK, regs + CSI_IPI_VBP_LINES);
	sys_write32(data->vfp & CSI_IPI_VFP_LINES_MASK, regs + CSI_IPI_VFP_LINES);
	sys_write32(data->vact & CSI_IPI_VACTIVE_LINES_MASK, regs + CSI_IPI_VACTIVE_LINES);

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
	if (data->csi_cpi_settings->ipi_ifx == CSI2_IPI_MODE_16_BIT_IFX) {
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

	/*
	 * Configuring the MIPI CSI-2 Host.
	 */
	/* Setup the number of data-lanes. */
	sys_write32(data->phy.num_lanes - 1, regs + CSI_N_LANES);

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
	reg_write_part(regs + CSI_IPI_DATA_TYPE, data->csi_cpi_settings->dt,
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
	struct dphy_csi2_settings *phy = &data->phy;
	struct dphy_dsi_settings dsi_phy = {0};
	int ret;

	dsi_phy.num_lanes = data->phy.num_lanes;
	dsi_phy.pll_fout = DSI_MINIMUM_PLL_FOUT;
	ret = dphy_dw_master_setup(config->rx_dphy, &dsi_phy);
	if (ret) {
		LOG_ERR("Failed to setup D-PHY TX.");
		return ret;
	}

	ret = dphy_dw_slave_setup(config->rx_dphy, phy);
	if (ret) {
		LOG_ERR("Failed to set-up D-PHY RX.");
		return ret;
	}

	return 0;
}

static int csi2_dw_validate_data(const struct device *dev)
{
	const struct csi2_dw_config *config = dev->config;
	struct csi2_dw_data *data = dev->data;
	float pixclock;
	uint32_t bpp;

	bpp = data->csi_cpi_settings->bits_per_pixel;
	/*
	 * When camera through-put is slower than IPI, all data is transferred
	 * before new Horizontal Line is received. RAM only needs to store one
	 * line.
	 */
	if (!((data->hact * bpp / CSI2_HOST_IPI_DWIDTH) <= CSI2_IPI_FIFO_DEPTH)) {
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
	pixclock = ((data->phy.pll_fin << 1) * data->phy.num_lanes * CSI2_BANDWIDTH_SCALER) / bpp;

	LOG_DBG("pll_fin - %d, Check pixclock = %d (CSI_PIXCLK_CTRL)", data->phy.pll_fin,
		(uint32_t)pixclock);

	if (config->ipi_mode == CSI2_IPI_MODE_TIMINGS_CAM) {
		/*
		 * FV(VSYNC) comes at least 3 pixel clocks before
		 * LV(HSYNC/DATA_EN). Hence, setting HSA as 3.
		 */
		data->hsa = 3;
		data->hbp = 0;

		/*
		 * HSD should be such that when the PPI interface should
		 * collect last pixel and send to memory before IPI interface
		 * is collecting the last pixel of Horizontal Active Area.
		 */
		data->hsd = ((pixclock * bpp * data->hact) / (data->phy.pll_fin << 1)) -
			    (data->hact + data->hsa) + 1;
		data->vsa = 0;
		data->vbp = 0;
		data->vfp = 0;
		data->vact = 0;
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
	struct csi2_dw_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);

	if (data->state == CSI_STATE_STREAMING) {
		LOG_DBG("Already Streaming.");
		return 0;
	}

	sys_set_bits(regs + CSI_IPI_MODE, CSI_IPI_MODE_ENABLE);
	LOG_DBG("Stream started");

	return 0;
}

static int csi2_dw_stream_stop(const struct device *dev)
{
	struct csi2_dw_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);

	if (data->state == CSI_STATE_STANDBY) {
		LOG_DBG("Already Stopped.");
		return 0;
	}

	sys_clear_bits(regs + CSI_IPI_MODE, CSI_IPI_MODE_ENABLE);
	LOG_DBG("Stream stopped");

	return 0;
}

static int csi2_dw_set_format(const struct device *dev, enum video_endpoint_id ep,
			      struct video_format *fmt)
{
	struct csi2_dw_data *data = dev->data;
	int32_t tmp;
	int i;

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

	if (data->csi_cpi_settings != NULL) {
		if (tmp == data->csi_cpi_settings->dt) {
			LOG_INF("FourCC format already set.");
			return 0;
		}
	}

	for (i = 0; i < ARRAY_SIZE(data_mode_settings); i++) {
		if (data_mode_settings[i].dt == tmp) {
			break;
		}
	}

	data->csi_cpi_settings = &data_mode_settings[i];
	data->state = CSI_STATE_CONFIGURED;

	data->hact = fmt->width;
	data->vact = fmt->height;

	return csi2_dw_configure(dev);
}

static int csi2_dw_get_caps(const struct device *dev, enum video_endpoint_id ep,
			    struct video_caps *caps)
{
	return -ENOTSUP;
}

static int csi2_dw_set_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	struct csi2_dw_data *data = dev->data;

	switch (cid) {
	case VIDEO_CID_ALIF_CSI_DPHY_FREQ:
		data->phy.pll_fin = *((uint32_t *)value);
		LOG_DBG("DPHY New PLL Freq. %d", data->phy.pll_fin);
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static const struct video_driver_api csi2_dw_driver_api = {
	.set_format = csi2_dw_set_format,
	.stream_start = csi2_dw_stream_start,
	.stream_stop = csi2_dw_stream_stop,
	.get_caps = csi2_dw_get_caps,
	.set_ctrl = csi2_dw_set_ctrl,
};

static int csi2_dw_init(const struct device *dev)
{
	const struct csi2_dw_config *config = dev->config;
	struct csi2_dw_data *data = dev->data;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	config->irq_config_func(dev);

	data->state = CSI_STATE_INIT;
	LOG_DBG("MMIO Address: 0x%08x", (uint32_t)DEVICE_MMIO_GET(dev));

	return 0;
}

#define ALIF_MIPI_CSI_DEVICE(i)                                                                    \
	static void csi2_dw_config_func_##i(const struct device *dev);                             \
	static const struct csi2_dw_config config_##i = {                                          \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(i)),                                              \
		.rx_dphy = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(i, phy_if)),                      \
                                                                                                   \
		.irq = DT_INST_IRQN(i),                                                            \
		.irq_config_func = csi2_dw_config_func_##i,                                        \
                                                                                                   \
		.ipi_mode = DT_INST_ENUM_IDX(i, ipi_mode),                                         \
	};                                                                                         \
                                                                                                   \
	static struct csi2_dw_data data_##i = {                                                    \
		.phy =                                                                             \
			{                                                                          \
				.num_lanes = DT_INST_PROP(i, data_lanes),                          \
				.pll_fin = DT_INST_PROP(i, rx_ddr_clk),                            \
			},                                                                         \
                                                                                                   \
		.hsa = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_hsa), (DT_INST_PROP(i, csi_hsa)),  \
				   (3)),                                                           \
		.hbp = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_hbp), (DT_INST_PROP(i, csi_hbp)),  \
				   (0)),                                                           \
		.hsd = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_hsd), (DT_INST_PROP(i, csi_hsd)),  \
				   (0)),                                                           \
		.hact = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_hact),                            \
				    (DT_INST_PROP(i, csi_hact)), (0)),                             \
		.vsa = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_vsa), (DT_INST_PROP(i, csi_vsa)),  \
				   (0)),                                                           \
		.vbp = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_vbp), (DT_INST_PROP(i, csi_vbp)),  \
				   (0)),                                                           \
		.vfp = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_vfp), (DT_INST_PROP(i, csi_vfp)),  \
				   (0)),                                                           \
		.vact = COND_CODE_1(DT_INST_NODE_HAS_PROP(i, csi_vact),                            \
				    (DT_INST_PROP(i, csi_vact)), (0)),                             \
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
