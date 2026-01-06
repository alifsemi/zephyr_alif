/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _CSI_DW_H_
#define _CSI_DW_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/drivers/mipi_dphy/dphy_dw.h>

/* CSI-2 Registers */
#define CSI_VERSION                     0x00  /* HW Version */
#define CSI_N_LANES                     0x04  /* Lane Config */
#define CSI_CSI2_RESETN                 0x08  /* CSI Controller Reset */
#define CSI_INT_ST_MAIN                 0x0C  /* Main INT status */
#define CSI_DATA_IDS_1                  0x10  /* DT Data ID monitor */
#define CSI_DATA_IDS_VC_1               0x30  /* VC Data ID monitor */
#define CSI_PHY_SHUTDOWNZ               0x40  /* PHY Shutdown control */
#define CSI_DPHY_RSTZ                   0x44  /* PHY Reset control */
#define CSI_PHY_RX                      0x48  /* PHY RX Signal status */
#define CSI_PHY_STOPSTATE               0x4C  /* PHY STOPSTATE status */
#define CSI_PHY_TEST_CTRL0              0x50  /* PHY Test Control 0 */
#define CSI_PHY_TEST_CTRL1              0x54  /* PHY Test Control 1 */
#define CSI_IPI_MODE                    0x80  /* IPI Mode Select */
#define CSI_IPI_VCID                    0x84  /* IPI VC Select */
#define CSI_IPI_DATA_TYPE               0x88  /* IPI DT Select */
#define CSI_IPI_MEM_FLUSH               0x8C  /* IPI Mem flush control */
#define CSI_IPI_HSA_TIME                0x90  /* IPI HSA Config */
#define CSI_IPI_HBP_TIME                0x94  /* IPI HBP Config */
#define CSI_IPI_HSD_TIME                0x98  /* IPI HSD Config */
#define CSI_IPI_HLINE_TIME              0x9C  /* IPI HLINE Config */
#define CSI_IPI_SOFTRSTN                0xA0  /* IPI Reset Control */
#define CSI_IPI_ADV_FEATURES            0xAC  /* IPI Advance features */
#define CSI_IPI_VSA_LINES               0xB0  /* IPI VSA Config */
#define CSI_IPI_VBP_LINES               0xB4  /* IPI VBP Config */
#define CSI_IPI_VFP_LINES               0xB8  /* IPI VFP Config */
#define CSI_IPI_VACTIVE_LINES           0xBC  /* IPI VACTIVE Config */
#define CSI_VC_EXTENSION                0xC8  /* VC Extension Config */
#define CSI_PHY_CAL                     0xCC  /* PHY CALIB Signal Status */
#define CSI_INT_ST_PHY_FATAL            0xE0  /* PHY PKT Discard INT-Status */
#define CSI_INT_MSK_PHY_FATAL           0xE4  /* PHY PKT Discard INT-Mask */
#define CSI_INT_FORCE_PHY_FATAL         0xE8  /* PHY PKT Discard INT-Force */
#define CSI_INT_ST_PKT_FATAL            0xF0  /* PHY PKT Construct INT-Status */
#define CSI_INT_MSK_PKT_FATAL           0xF4  /* PHY PKT Construct INT-Mask */
#define CSI_INT_FORCE_PKT_FATAL         0xF8  /* PHY PKT Construct INT-Force */
#define CSI_INT_ST_PHY                  0x110 /* PHY INT-Status */
#define CSI_INT_MSK_PHY                 0x114 /* PHY INT-Mask */
#define CSI_INT_FORCE_PHY               0x118 /* PHY INT-Force */
#define CSI_INT_ST_LINE                 0x130 /* PHY Line construct INT-Status */
#define CSI_INT_MSK_LINE                0x134 /* PHY Line construct INT-Mask */
#define CSI_INT_FORCE_LINE              0x138 /* PHY Line construct INT-Force */
#define CSI_INT_ST_IPI_FATAL            0x140 /* IPI ifx INT-Status */
#define CSI_INT_MSK_IPI_FATAL           0x144 /* IPI ifx INT-Mask */
#define CSI_INT_FORCE_IPI_FATAL         0x148 /* IPI ifx INT-Force */
#define CSI_INT_ST_BNDRY_FRAME_FATAL    0x280 /* Frame Boundary ERR INT-Status */
#define CSI_INT_MSK_BNDRY_FRAME_FATAL   0x284 /* Frame Boundary ERR INT-Mask */
#define CSI_INT_FORCE_BNDRY_FRAME_FATAL 0x288 /* Frame Boundary ERR INT-Force */
#define CSI_INT_ST_SEQ_FRAME_FATAL      0x290 /* Frame Seq ERR INT-Status */
#define CSI_INT_MSK_SEQ_FRAME_FATAL     0x294 /* Frame Seq ERR INT-Mask */
#define CSI_INT_FORCE_SEQ_FRAME_FATAL   0x298 /* Frame Seq ERR INT-Force */
#define CSI_INT_ST_CRC_FRAME_FATAL      0x2A0 /* Frame CRC ERR INT-Status */
#define CSI_INT_MSK_CRC_FRAME_FATAL     0x2A4 /* Frame CRC ERR INT-Mask */
#define CSI_INT_FORCE_CRC_FRAME_FATAL   0x2A8 /* Frame CRC ERR INT-Force */
#define CSI_INT_ST_PLD_CRC_FATAL        0x2B0 /* Frame Payload ERR INT-Status */
#define CSI_INT_MSK_PLD_CRC_FATAL       0x2B4 /* Frame Payload ERR INT-Mask */
#define CSI_INT_FORCE_PLD_CRC_FATAL     0x2B8 /* Frame Payload ERR INT-Force */
#define CSI_INT_ST_DATA_ID              0x2C0 /* DT ERR INT-Status */
#define CSI_INT_MSK_DATA_ID             0x2C4 /* DT ERR INT-Mask */
#define CSI_INT_FORCE_DATA_ID           0x2C8 /* DT ERR INT-Force */
#define CSI_INT_ST_ECC_CORRECT          0x2D0 /* ECC ERR INT-Status */
#define CSI_INT_MSK_ECC_CORRECT         0x2D4 /* ECC ERR INT-Mask */
#define CSI_INT_FORCE_ECC_CORRECT       0x2D8 /* ECC ERR INT-Force */
#define CSI_SCRAMBLING                  0x300 /* Descrambling control */
#define CSI_SCRAMBLING_SEED1            0x304 /* Descrambling Seed config lane-0*/
#define CSI_SCRAMBLING_SEED2            0x308 /* Descrambling Seed config lane-1*/

/* Registers Bit-fields */

/* Lane Configuration Register */
#define CSI_N_LANES_MASK  GENMASK(2, 0)
#define CSI_N_LANES_SHIFT 0

/* Reset Control Register */
#define CSI2_RESETN BIT(0)

/* Main Interrupt status Register */
#define CSI_INT_ST_MAIN_IPI_FATAL     BIT(18)
#define CSI_INT_ST_MAIN_LINE          BIT(17)
#define CSI_INT_ST_MAIN_PHY           BIT(16)
#define CSI_INT_ST_MAIN_ECC_CORRECTED BIT(7)
#define CSI_INT_ST_MAIN_DATA_ID       BIT(6)
#define CSI_INT_ST_MAIN_PLD_CRC       BIT(5)
#define CSI_INT_ST_MAIN_FRAME_CRC     BIT(4)
#define CSI_INT_ST_MAIN_FRAME_SEQ     BIT(3)
#define CSI_INT_ST_MAIN_FRAME_BNDRY   BIT(2)
#define CSI_INT_ST_MAIN_PKT           BIT(1)
#define CSI_INT_ST_MAIN_PHY_FATAL     BIT(0)

/* DT Data ID Monitors Configuration Register */
#define CSI_DATA_IDS_1_DI3_DT_MASK  GENMASK(6, 0)
#define CSI_DATA_IDS_1_DI3_DT_SHIFT 24
#define CSI_DATA_IDS_1_DI2_DT_MASK  GENMASK(6, 0)
#define CSI_DATA_IDS_1_DI2_DT_SHIFT 16
#define CSI_DATA_IDS_1_DI1_DT_MASK  GENMASK(6, 0)
#define CSI_DATA_IDS_1_DI1_DT_SHIFT 8
#define CSI_DATA_IDS_1_DI0_DT_MASK  GENMASK(6, 0)
#define CSI_DATA_IDS_1_DI0_DT_SHIFT 0

/* VC Data ID Monitors Configuration Register */
#define CSI_DATA_IDS_VC_1_DI3_VCX_MASK  GENMASK(1, 0)
#define CSI_DATA_IDS_VC_1_DI3_VCX_SHIFT 26
#define CSI_DATA_IDS_VC_1_DI3_MASK      GENMASK(1, 0)
#define CSI_DATA_IDS_VC_1_DI3_SHIFT     24
#define CSI_DATA_IDS_VC_1_DI2_VCX_MASK  GENMASK(1, 0)
#define CSI_DATA_IDS_VC_1_DI2_VCX_SHIFT 18
#define CSI_DATA_IDS_VC_1_DI2_MASK      GENMASK(1, 0)
#define CSI_DATA_IDS_VC_1_DI2_SHIFT     16
#define CSI_DATA_IDS_VC_1_DI1_VCX_MASK  GENMASK(1, 0)
#define CSI_DATA_IDS_VC_1_DI1_VCX_SHIFT 10
#define CSI_DATA_IDS_VC_1_DI1_MASK      GENMASK(1, 0)
#define CSI_DATA_IDS_VC_1_DI1_SHIFT     8
#define CSI_DATA_IDS_VC_1_DI0_VCX_MASK  GENMASK(3, 2)
#define CSI_DATA_IDS_VC_1_DI0_VCX_SHIFT 2
#define CSI_DATA_IDS_VC_1_DI0_MASK      GENMASK(1, 0)
#define CSI_DATA_IDS_VC_1_DI0_SHIFT     0

/* PHY Shutdown Control Register */
#define CSI_PHY_SHUTDOWNZ_SHUTDOWNZ BIT(0)

/* PHY Reset Control Register */
#define CSI_DPHY_RSTZ_RSTZ BIT(0)

/* PHY RX Signals Status Register */
#define CSI_PHY_RX_RXCLKACTIVEHS BIT(17)
#define CSI_PHY_RX_RXULPSCLKNOT  BIT(16)
#define CSI_PHY_RX_RXULPSESC_1   BIT(1)
#define CSI_PHY_RX_RXULPSESC_0   BIT(0)

/* PHY STOPSTATE Signal Status Register */
#define CSI_PHY_STOPSTATE_CLK    BIT(16)
#define CSI_PHY_STOPSTATE_DATA_1 BIT(1)
#define CSI_PHY_STOPSTATE_DATA_0 BIT(0)

/* PHY Test Interface Control Reg 0 */
#define CSI_PHY_TST_CTRL0_CLK BIT(1)
#define CSI_PHY_TST_CTRL0_CLR BIT(0)

/* PHY Test Interface Control Reg 1 */
#define CSI_PHY_TST_CTRL1_TESTEN         BIT(16)
#define CSI_PHY_TST_CTRL1_TESTDOUT_MASK  GENMASK(7, 0)
#define CSI_PHY_TST_CTRL1_TESTDOUT_SHIFT 8
#define CSI_PHY_TST_CTRL1_TESTDIN_MASK   GENMASK(7, 0)
#define CSI_PHY_TST_CTRL1_TESTDIN_SHIFT  0

/* IPI Mode Selection Register */
#define CSI_IPI_MODE_ENABLE      BIT(24)
#define CSI_IPI_MODE_CUT_THROUGH BIT(16)
#define CSI_IPI_MODE_COLOR_COM   BIT(8)
#define CSI_IPI_MODE_MODE        BIT(0)

/* IPI VCID Register */
#define CSI_IPI_VCID_VCX_MASK   GENMASK(1, 0)
#define CSI_IPI_VCID_VCX_SHIFT  2
#define CSI_IPI_VCID_VCID_MASK  GENMASK(1, 0)
#define CSI_IPI_VCID_VCID_SHIFT 0

/* IPI DT Selection Register */
#define CSI_IPI_DATA_TYPE_EMBEDDED_DATA BIT(8)
#define CSI_IPI_DATA_TYPE_TYPE_MASK     GENMASK(5, 0)
#define CSI_IPI_DATA_TYPE_TYPE_SHIFT    0

/* IPI Memory Flush Control Register */
#define CSI_IPI_MEM_FLUSH_AUTO_FLUSH BIT(8)
#define CSI_IPI_MEM_FLUSH_FLUSH      BIT(0)

/* IPI HSA Configuration Register */
#define CSI_IPI_HSA_TIME_MASK  GENMASK(11, 0)
#define CSI_IPI_HSA_TIME_SHIFT 0

/* IPI HBP Configuration Register */
#define CSI_IPI_HBP_TIME_MASK  GENMASK(11, 0)
#define CSI_IPI_HBP_TIME_SHIFT 0

/* IPI HSD Configuration Register */
#define CSI_IPI_HSD_TIME_MASK  GENMASK(11, 0)
#define CSI_IPI_HSD_TIME_SHIFT 0

/* IPI HLINE Configuration Register */
#define CSI_IPI_HLINE_TIME_MASK  GENMASK(14, 0)
#define CSI_IPI_HLINE_TIME_SHIFT 0

/* IPI Soft-Reset Register */
#define CSI_IPI_SOFTRSTN_RSTN BIT(0)

/* IPI Advanced Features Register */
#define CSI_IPI_ADV_FEATURES_SYNC_EVENT     BIT(24)
#define CSI_IPI_ADV_FEATURES_EN_EMBEDDED    BIT(21)
#define CSI_IPI_ADV_FEATURES_EN_BLANKING    BIT(20)
#define CSI_IPI_ADV_FEATURES_EN_NULL        BIT(19)
#define CSI_IPI_ADV_FEATURES_EN_LINE_START  BIT(18)
#define CSI_IPI_ADV_FEATURES_EN_VIDEO       BIT(17)
#define CSI_IPI_ADV_FEATURES_SEL_LINE_EVENT BIT(16)
#define CSI_IPI_ADV_FEATURES_DT_MASK        GENMASK(5, 0)
#define CSI_IPI_ADV_FEATURES_DT_SHIFT       8
#define CSI_IPI_ADV_FEATURES_DT_OVERWRITE   BIT(0)

/* IPI VSA Configuration Register */
#define CSI_IPI_VSA_LINES_MASK  GENMASK(9, 0)
#define CSI_IPI_VSA_LINES_SHIFT 0

/* IPI VBP Configuration Register */
#define CSI_IPI_VBP_LINES_MASK  GENMASK(9, 0)
#define CSI_IPI_VBP_LINES_SHIFT 0

/* IPI VFP Configuration Register */
#define CSI_IPI_VFP_LINES_MASK  GENMASK(9, 0)
#define CSI_IPI_VFP_LINES_SHIFT 0

/* IPI VACTIVE Configuration Register */
#define CSI_IPI_VACTIVE_LINES_MASK  GENMASK(13, 0)
#define CSI_IPI_VACTIVE_LINES_SHIFT 0

/* VC Extension Configuration Register */
#define CSI_VC_EXTENSION_VCX BIT(0)

/* PHY CALIBRATION Signal Status Register */
#define CSI_PHY_CAL_RXSKEWCALHS BIT(0)

/* Fatal PHY Packet Discarded Interrupt Mask, Status and Force */
#define INT_PHY_FATAL_ERRSOTSYNCHS_1 BIT(1)
#define INT_PHY_FATAL_ERRSOTSYNCHS_0 BIT(0)
#define INT_PHY_FATAL_MASK           (INT_PHY_FATAL_ERRSOTSYNCHS_1 | INT_PHY_FATAL_ERRSOTSYNCHS_0)

/* Fatal PHY Packet Construction Interrupt Mask, Status and Force */
#define INT_PKT_FATAL_ERR_ECC_DOUBLE BIT(0)
#define INT_PKT_FATAL_MASK           (INT_PKT_FATAL_ERR_ECC_DOUBLE)

/* PHY Interrupt Mask, Status and Force */
#define INT_PHY_ERRESC_1   BIT(17)
#define INT_PHY_ERRESC_0   BIT(16)
#define INT_PHY_ERRSOTHS_1 BIT(1)
#define INT_PHY_ERRSOTHS_0 BIT(0)
#define INT_PHY_MASK       (INT_PHY_ERRESC_1 | INT_PHY_ERRESC_0 | INT_PHY_ERRSOTHS_1 |             \
				INT_PHY_ERRSOTHS_0)

/* PHY LINE Construction Interrupt Mask, Status and Force */
#define INT_LINE_ERR_SEQ_DI3         BIT(19)
#define INT_LINE_ERR_SEQ_DI2         BIT(18)
#define INT_LINE_ERR_SEQ_DI1         BIT(17)
#define INT_LINE_ERR_SEQ_DI0         BIT(16)
#define INT_LINE_ERR_BNDRY_MATCH_DI3 BIT(3)
#define INT_LINE_ERR_BNDRY_MATCH_DI2 BIT(2)
#define INT_LINE_ERR_BNDRY_MATCH_DI1 BIT(1)
#define INT_LINE_ERR_BNDRY_MATCH_DI0 BIT(0)
#define INT_LINE_ERR_MASK                                                                          \
	(INT_LINE_ERR_SEQ_DI3 | INT_LINE_ERR_SEQ_DI2 | INT_LINE_ERR_SEQ_DI1 |                      \
	 INT_LINE_ERR_SEQ_DI0 | INT_LINE_ERR_BNDRY_MATCH_DI3 | INT_LINE_ERR_BNDRY_MATCH_DI2 |      \
	 INT_LINE_ERR_BNDRY_MATCH_DI1 | INT_LINE_ERR_BNDRY_MATCH_DI0)

/* Fatal IPI Interrupt Mask, Status and Force */
#define INT_IPI_EVENT_FIFO_OVERFLOW     BIT(5)
#define INT_IPI_PIXEL_IF_HLINE_ERR      BIT(4)
#define INT_IPI_PIXEL_IF_FIFO_NEMPTY_FS BIT(3)
#define INT_IPI_PIXEL_IF_FRAME_SYNC_ERR BIT(2)
#define INT_IPI_PIXEL_IF_FIFO_OVERFLOW  BIT(1)
#define INT_IPI_PIXEL_IF_FIFO_UNDERFLOW BIT(0)
#define INT_IPI_MASK                                                                               \
	(INT_IPI_EVENT_FIFO_OVERFLOW | INT_IPI_PIXEL_IF_HLINE_ERR |                                \
	 INT_IPI_PIXEL_IF_FIFO_NEMPTY_FS | INT_IPI_PIXEL_IF_FRAME_SYNC_ERR |                       \
	 INT_IPI_PIXEL_IF_FIFO_OVERFLOW | INT_IPI_PIXEL_IF_FIFO_UNDERFLOW)

/*
 * Per VC Interrupt Mask, Status and Force for
 * BNDRY_FRAME_FATAL, SEQ_FRAME_FATAL, CRC_FRAME_FATAL, PLD_CRC_FATAL,
 * DATA_ID, ECC_CORRECT.
 */
#define INT_VC15 BIT(15)
#define INT_VC14 BIT(14)
#define INT_VC13 BIT(13)
#define INT_VC12 BIT(12)
#define INT_VC11 BIT(11)
#define INT_VC10 BIT(10)
#define INT_VC9  BIT(9)
#define INT_VC8  BIT(8)
#define INT_VC7  BIT(7)
#define INT_VC6  BIT(6)
#define INT_VC5  BIT(5)
#define INT_VC4  BIT(4)
#define INT_VC3  BIT(3)
#define INT_VC2  BIT(2)
#define INT_VC1  BIT(1)
#define INT_VC0  BIT(0)
#define INT_VC_MASK                                                                                \
	(INT_VC15 | INT_VC14 | INT_VC13 | INT_VC12 | INT_VC11 | INT_VC10 | INT_VC9 | INT_VC8 |     \
	 INT_VC7 | INT_VC6 | INT_VC5 | INT_VC4 | INT_VC3 | INT_VC2 | INT_VC1 | INT_VC0)

#define INT_BNDRY_FRAME_FATAL_MASK INT_VC_MASK
#define INT_SEQ_FRAME_FATAL_MASK   INT_VC_MASK
#define INT_CRC_FRAME_FATAL_MASK   INT_VC_MASK
#define INT_PLD_CRC_FATAL_MASK     INT_VC_MASK
#define INT_DATA_ID_MASK           INT_VC_MASK
#define INT_ECC_CORRECT_MASK       INT_VC_MASK

/* Descrambling Control Register */
#define CSI_SCRAMBLING_ENABLE BIT(0)

/* Descrambling Seed Lane N Register */
#define CSI_SCRAMBLING_SEED_LANE_N_MASK  GENMASK(15, 0)
#define CSI_SCRAMBLING_SEED_LANE_N_SHIFT 0

/* CSI constants */
#define CSI2_HOST_IPI_DWIDTH  64
#define CSI2_IPI_FIFO_DEPTH   1024
#define DSI_MINIMUM_PLL_FOUT  MHZ(40)
#define CSI2_BANDWIDTH_SCALER (1.2)

enum csi2_ipi_mode_timings {
	CSI2_IPI_MODE_TIMINGS_CAM,
	CSI2_IPI_MODE_TIMINGS_CTRL,
};

enum csi_state {
	CSI_STATE_INIT,
	CSI_STATE_CONFIGURED,
	CSI_STATE_STREAMING,
	CSI_STATE_STANDBY,
};

struct csi2_dw_config {
	DEVICE_MMIO_ROM;

	const struct device *rx_dphy;
	uint32_t ipi_mode: 1;

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(clocks)
	const struct device *clk_dev;
	clock_control_subsys_t csi_cid;
	clock_control_subsys_t pix_cid;
	clock_control_subsys_t rxdphy_cid;
	clock_control_subsys_t cfg_cid;
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(clocks) */

	uint32_t irq;
	void (*irq_config_func)(const struct device *dev);
};

struct csi2_dw_data {
	DEVICE_MMIO_RAM;
	struct dphy_csi2_settings phy;

	uint16_t vsa;
	uint16_t vbp;
	uint16_t vfp;
	uint16_t vact;
	uint16_t hsa;
	uint16_t hbp;
	uint16_t hsd;
	uint16_t hact;

	const struct cpi_csi2_mode_settings *csi_cpi_settings;
	enum csi_state state;
};

#endif /* _CSI_DW_H_ */
