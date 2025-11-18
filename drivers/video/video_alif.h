/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _VIDEO_ALIF_H_
#define _VIDEO_ALIF_H_

#include <zephyr/device.h>

/* CPI Registers */
#define CAM_CTRL         0x00
#define CAM_INTR         0x04
#define CAM_INTR_ENA     0x08
#define CAM_CFG          0x10
#define CAM_FIFO_CTRL    0x14
#define CAM_AXI_ERR_STAT 0x18
#define CAM_VIDEO_FCFG   0x28
#define CAM_CSI_CMCFG    0x2C
#define CAM_FRAME_ADDR   0x30

/* CPI Register bit-field */
#define CAM_CTRL_FIFO_CLK_SEL BIT(12)
#define CAM_CTRL_SW_RESET     BIT(8)
#define CAM_CTRL_SNAPSHOT     BIT(4)
#define CAM_CTRL_BUSY         BIT(2)
#define CAM_CTRL_START        BIT(0)

#define INTR_HSYNC           BIT(20)
#define INTR_VSYNC           BIT(16)
#define INTR_BRESP_ERR       BIT(6)
#define INTR_OUTFIFO_OVERRUN BIT(5)
#define INTR_INFIFO_OVERRUN  BIT(4)
#define INTR_STOP            BIT(0)

#define CAM_CFG_DATA_MASK       GENMASK(1, 0)
#define CAM_CFG_DATA_SHIFT      28
#define CAM_CFG_CODE10ON8       BIT(24)
#define CAM_CFG_MSB             BIT(20)
#define CAM_CFG_DATA_MODE_MASK  GENMASK(2, 0)
#define CAM_CFG_DATA_MODE_SHIFT 16
#define CAM_CFG_VSYNC_POL       BIT(14)
#define CAM_CFG_HSYNC_POL       BIT(13)
#define CAM_CFG_PCLK_POL        BIT(12)
#define CAM_CFG_ROW_ROUNDUP     BIT(8)
#define CAM_CFG_VSYNC_EN        BIT(5)
#define CAM_CFG_WAIT_VSYNC      BIT(4)
#define CAM_CFG_ISP_PORT_EN     BIT(3)
#define CAM_CFG_AXI_PORT_EN     BIT(2)
#define CAM_CFG_CSI_HALT_EN     BIT(1)
#define CAM_CFG_MIPI_CSI        BIT(0)

#define CAM_FIFO_CTRL_WR_WMARK_MASK  GENMASK(4, 0)
#define CAM_FIFO_CTRL_WR_WMARK_SHIFT 8
#define CAM_FIFO_CTRL_RD_WMARK_MASK  GENMASK(4, 0)
#define CAM_FIFO_CTRL_RD_WMARK_SHIFT 0

#define CAM_AXI_ERR_STAT_CNT_MASK    GENMASK(7, 0)
#define CAM_AXI_ERR_STAT_CNT_SHIFT   8
#define CAM_AXI_ERR_STAT_BRESP_MASK  GENMASK(1, 0)
#define CAM_AXI_ERR_STAT_BRESP_SHIFT 0

#define CAM_VIDEO_FCFG_ROW_MASK   GENMASK(11, 0)
#define CAM_VIDEO_FCFG_ROW_SHIFT  16
#define CAM_VIDEO_FCFG_DATA_MASK  GENMASK(13, 0)
#define CAM_VIDEO_FCFG_DATA_SHIFT 0

#define CAM_CSI_CMCFG_MODE_MASK  GENMASK(3, 0)
#define CAM_CSI_CMCFG_MODE_SHIFT 0

#define CAM_FRAME_ADDR_MASK  GENMASK(28, 0)
#define CAM_FRAME_ADDR_SHIFT 3

/* CPI constants. */
#define CPI_MIN_VBUF 1

enum cpi_capture_mode {
	CPI_CAPTURE_MODE_CONTINUOUS = 0,
	CPI_CAPTURE_MODE_SNAPSHOT,
};

enum cpi_input_fifo_clk_sel {
	CPI_INPUT_FIFO_CLK_INTERNAL = 0,
	CPI_INPUT_FIFO_CLK_EXTERNAL,
};

enum cpi_data_mask {
	CPI_DATA_MASK_16_BIT = 0,
	CPI_DATA_MASK_10_BIT,
	CPI_DATA_MASK_12_BIT,
	CPI_DATA_MASK_14_BIT,
};

enum video_cam_device_state {
	STATE_INIT,
	STATE_CONFIGURED,
	STATE_CONTROLLER_STOPPED,
	STATE_STREAMING,
	STATE_STANDBY,
};

struct video_cam_config {
	DEVICE_MMIO_ROM;
	void (*irq_config_func)(const struct device *dev);

	uint32_t irq;
	const struct pinctrl_dev_config *pcfg;

	uint32_t polarity;

	uint32_t msb: 1;
	uint32_t is_lpcam: 1;
	uint32_t vsync_en: 1;
	uint32_t data_mode: 3;
	uint32_t code10on8: 1;
	uint32_t data_mask: 2;
	uint32_t read_wmark: 5;
	uint32_t wait_vsync: 1;
	uint32_t csi_halt_en: 1;
	uint32_t write_wmark: 5;
	uint32_t capture_mode: 1;
	uint32_t axi_bus_ep: 1;
	uint32_t isp_ep: 1;

	const struct device *clk_dev;
	clock_control_subsys_t cid;

	const struct device *sensor;
	const struct device *csi_bus;
};

struct video_cam_data {
	DEVICE_MMIO_RAM;

	const struct device *dev;

	uint32_t curr_vid_buf;

	struct k_fifo fifo_in;
	struct k_fifo fifo_out;

	struct k_work cb_work;
	struct k_work_q cb_workq;

	struct k_poll_signal *signal;
	struct video_format current_format;
	bool is_streaming;
};

#endif /* _VIDEO_ALIF_H_ */
