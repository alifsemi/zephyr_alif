/*
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _VIDEO_ALIF_ISP_H_
#define _VIDEO_ALIF_ISP_H_

#include <zephyr/device.h>
#include <zephyr/drivers/video/isp-vsi.h>

/* Registers */
#define ISP_VI_CCL			0x000
#define ISP_ID_CUSTOM_ID		0x004
#define ISP_ID_PRODUCT_ID		0x008
#define ISP_ID_CHIP_ID			0x00C
#define ISP_VI_ICCL			0x010
#define ISP_VI_IRCL			0x014
#define ISP_ID_ECO_ID			0x020
#define ISP_ID_CHIP_REVISION		0x024
#define ISP_ID_PATCH_REVISION		0x028
#define ISP_ID_CHIP_DATE		0x02C
#define ISP_ID_CHIP_TIME		0x030
#define ISP_VI_ID_RSV0			0x034
#define ISP_VI_ID_RSV1			0x038
#define ISP_VI_ID_RSV2			0x03C
#define ISP_VI_ID_RSV3			0x040
#define ISP_VI_ID_RSV4			0x044
#define ISP_VI_ID_RSV5			0x048
#define ISP_VI_ID_RSV6			0x04C
#define ISP_VI_ID_RSV7			0x050
#define ISP_CTRL			0x0400
#define ISP_ACQ_PROP			0x0404
#define ISP_ACQ_H_OFFS			0x0408
#define ISP_ACQ_V_OFFS			0x040C
#define ISP_ACQ_H_SIZE			0x0410
#define ISP_ACQ_V_SIZE			0x0414
#define ISP_TPG_CTRL			0x0500
#define ISP_TPG_TOTAL_IN		0x0504
#define ISP_TPG_ACT_IN			0x0508
#define ISP_TPG_FP_IN			0x050C
#define ISP_TPG_BP_IN			0x0510
#define ISP_TPG_W_IN			0x0514
#define ISP_TPG_GAP_IN			0x0518
#define ISP_TPG_GAP_STD_IN		0x051C
#define ISP_TPG_RANDOM_SEED		0x0520
#define ISP_TPG_FRAME_NUM		0x0524
#define ISP_FRAME_RATE			0x0600
#define ISP_OUT_H_OFFS			0x0604
#define ISP_OUT_V_OFFS			0x0608
#define ISP_OUT_H_SIZE			0x060C
#define ISP_OUT_V_SIZE			0x0610
#define ISP_OUT_H_OFFS_SHD		0x0614
#define ISP_OUT_V_OFFS_SHD		0x0618
#define ISP_OUT_H_SIZE_SHD		0x061C
#define ISP_OUT_V_SIZE_SHD		0x0620
#define ISP_BINNING_SIZE		0x0624
#define ISP_BINNING_NUM			0x0628
#define ISP_BINNING_SIZE_SHD		0x062C
#define ISP_BINNING_NUM_SHD		0x0630
#define ISP_BLS_CTRL			0x0700
#define ISP_BLS_A_FIXED			0x0704
#define ISP_BLS_B_FIXED			0x0708
#define ISP_BLS_C_FIXED			0x070C
#define ISP_BLS_D_FIXED			0x0710
#define ISP_EXP_CONF			0x0720
#define ISP_EXP_H_OFFSET		0x0724
#define ISP_EXP_V_OFFSET		0x0728
#define ISP_EXP_H_SIZE			0x072C
#define ISP_EXP_V_SIZE			0x0730
#define ISP_EXP_MEAN_00			0x0734
#define ISP_EXP_MEAN_10			0x0738
#define ISP_EXP_MEAN_20			0x073C
#define ISP_EXP_MEAN_30			0x0740
#define ISP_EXP_MEAN_40			0x0744
#define ISP_EXP_MEAN_01			0x0748
#define ISP_EXP_MEAN_11			0x074C
#define ISP_EXP_MEAN_21			0x0750
#define ISP_EXP_MEAN_31			0x0754
#define ISP_EXP_MEAN_41			0x0758
#define ISP_EXP_MEAN_02			0x075C
#define ISP_EXP_MEAN_12			0x0760
#define ISP_EXP_MEAN_22			0x0764
#define ISP_EXP_MEAN_32			0x0768
#define ISP_EXP_MEAN_42			0x076C
#define ISP_EXP_MEAN_03			0x0770
#define ISP_EXP_MEAN_13			0x0774
#define ISP_EXP_MEAN_23			0x0778
#define ISP_EXP_MEAN_33			0x077C
#define ISP_EXP_MEAN_43			0x0780
#define ISP_EXP_MEAN_04			0x0784
#define ISP_EXP_MEAN_14			0x0788
#define ISP_EXP_MEAN_24			0x078C
#define ISP_EXP_MEAN_34			0x0790
#define ISP_EXP_MEAN_44			0x0794
#define ISP_DGAIN_RB			0x0800
#define ISP_DGAIN_G			0x0804
#define ISP_DGAIN_RB_SHD		0x0808
#define ISP_DGAIN_G_SHD			0x080C
#define ISP_DEMOSAIC			0x0810
#define ISP_FILT_MODE			0x0814
#define ISP_FILT_THRES_BL0		0x0818
#define ISP_FILT_THRES_BL1		0x081C
#define ISP_FILT_THRES_SH0		0x0820
#define ISP_FILT_THRES_SH1		0x0824
#define ISP_FILT_LUM_WEIGHT		0x0828
#define ISP_FILT_FAC_SH1		0x082C
#define ISP_FILT_FAC_SH0		0x0830
#define ISP_FILT_FAC_MID		0x0834
#define ISP_FILT_FAC_BL0		0x0838
#define ISP_FILT_FAC_BL1		0x083C
#define ISP_CAC_CTRL			0x0870
#define ISP_CAC_COUNT_START		0x0874
#define ISP_CAC_A			0x0878
#define ISP_CAC_B			0x087C
#define ISP_CAC_C			0x0880
#define ISP_CAC_X_NORM			0x0884
#define ISP_CAC_Y_NORM			0x0888
#define ISP_GAMMA_OUT_MODE		0x0900
#define ISP_GAMMA_OUT_Y_0		0x0904
#define ISP_GAMMA_OUT_Y_1		0x0908
#define ISP_GAMMA_OUT_Y_2		0x090C
#define ISP_GAMMA_OUT_Y_3		0x0910
#define ISP_GAMMA_OUT_Y_4		0x0914
#define ISP_GAMMA_OUT_Y_5		0x0918
#define ISP_GAMMA_OUT_Y_6		0x091C
#define ISP_GAMMA_OUT_Y_7		0x0920
#define ISP_GAMMA_OUT_Y_8		0x0924
#define ISP_GAMMA_OUT_Y_9		0x0928
#define ISP_GAMMA_OUT_Y_10		0x092C
#define ISP_GAMMA_OUT_Y_11		0x0930
#define ISP_GAMMA_OUT_Y_12		0x0934
#define ISP_GAMMA_OUT_Y_13		0x0938
#define ISP_GAMMA_OUT_Y_14		0x093C
#define ISP_GAMMA_OUT_Y_15		0x0940
#define ISP_GAMMA_OUT_Y_16		0x0944
#define ISP_AWB_PROP			0x0950
#define ISP_AWB_H_OFFS			0x0954
#define ISP_AWB_V_OFFS			0x0958
#define ISP_AWB_H_SIZE			0x095C
#define ISP_AWB_V_SIZE			0x0960
#define ISP_AWB_FRAMES			0x0964
#define ISP_AWB_REF			0x0968
#define ISP_AWB_THRESH			0x096C
#define ISP_AWB_WHITE_CNT		0x0980
#define ISP_AWB_MEAN			0x0984
#define ISP_CC_COEFF_0			0x0A00
#define ISP_CC_COEFF_1			0x0A04
#define ISP_CC_COEFF_2			0x0A08
#define ISP_CC_COEFF_3			0x0A0C
#define ISP_CC_COEFF_4			0x0A10
#define ISP_CC_COEFF_5			0x0A14
#define ISP_CC_COEFF_6			0x0A18
#define ISP_CC_COEFF_7			0x0A1C
#define ISP_CC_COEFF_8			0x0A20
#define ISP_FORMAT_CONV_CTRL		0x0A24
#define ISP_CT_COEFF_0			0x0A30
#define ISP_CT_COEFF_1			0x0A34
#define ISP_CT_COEFF_2			0x0A38
#define ISP_CT_COEFF_3			0x0A3C
#define ISP_CT_COEFF_4			0x0A40
#define ISP_CT_COEFF_5			0x0A44
#define ISP_CT_COEFF_6			0x0A48
#define ISP_CT_COEFF_7			0x0A4C
#define ISP_CT_COEFF_8			0x0A50
#define ISP_CT_OFFSET_R			0x0A54
#define ISP_CT_OFFSET_G			0x0A58
#define ISP_CT_OFFSET_B			0x0A5C
#define ISP_IMSC			0x0B00
#define ISP_RIS				0x0B04
#define ISP_MIS				0x0B08
#define ISP_ICR				0x0B0C
#define ISP_ISR				0x0B10
#define ISP_ERR				0x0B14
#define ISP_ERR_CLR			0x0B18
#define ISP_MRSZ_CTRL			0x0C00
#define ISP_MRSZ_SCALE_HY		0x0C04
#define ISP_MRSZ_SCALE_HCB		0x0C08
#define ISP_MRSZ_SCALE_HCR		0x0C0C
#define ISP_MRSZ_SCALE_VY		0x0C10
#define ISP_MRSZ_SCALE_VC		0x0C14
#define ISP_MRSZ_PHASE_HY		0x0C18
#define ISP_MRSZ_PHASE_HC		0x0C1C
#define ISP_MRSZ_PHASE_VY		0x0C20
#define ISP_MRSZ_PHASE_VC		0x0C24
#define ISP_MRSZ_SCALE_LUT_ADDR		0x0C28
#define ISP_MRSZ_SCALE_LUT		0x0C2C
#define ISP_MRSZ_CTRL_SHD		0x0C30
#define ISP_MRSZ_SCALE_HY_SHD		0x0C34
#define ISP_MRSZ_SCALE_HCB_SHD		0x0C38
#define ISP_MRSZ_SCALE_HCR_SHD		0x0C3C
#define ISP_MRSZ_SCALE_VY_SHD		0x0C40
#define ISP_MRSZ_SCALE_VC_SHD		0x0C44
#define ISP_MRSZ_PHASE_HY_SHD		0x0C48
#define ISP_MRSZ_PHASE_HC_SHD		0x0C4C
#define ISP_MRSZ_PHASE_VY_SHD		0x0C50
#define ISP_MRSZ_PHASE_VC_SHD		0x0C54
#define ISP_MRSZ_FORMAT_CONV_CTRL	0x0C6C
#define ISP_MI_CTRL			0x0E00
#define ISP_MI_INIT			0x0E04
#define ISP_MI_MP_Y_BASE_AD_INIT	0x0E08
#define ISP_MI_MP_Y_SIZE_INIT		0x0E0C
#define ISP_MI_MP_Y_OFFS_CNT_INIT	0x0E10
#define ISP_MI_MP_Y_OFFS_CNT_START	0x0E14
#define ISP_MI_MP_Y_IRQ_OFFS_INIT	0x0E18
#define ISP_MI_MP_CB_BASE_AD_INIT	0x0E1C
#define ISP_MI_MP_CB_SIZE_INIT		0x0E20
#define ISP_MI_MP_CB_OFFS_CNT_INIT	0x0E24
#define ISP_MI_MP_CB_OFFS_CNT_START	0x0E28
#define ISP_MI_MP_CR_BASE_AD_INIT	0x0E2C
#define ISP_MI_MP_CR_SIZE_INIT		0x0E30
#define ISP_MI_MP_CR_OFFS_CNT_INIT	0x0E34
#define ISP_MI_MP_CR_OFFS_CNT_START	0x0E38
#define ISP_MI_BYTE_CNT			0x0E70
#define ISP_MI_CTRL_SHD			0x0E74
#define ISP_MI_MP_Y_BASE_AD_SHD		0x0E78
#define ISP_MI_MP_Y_SIZE_SHD		0x0E7C
#define ISP_MI_MP_Y_OFFS_CNT_SHD	0x0E80
#define ISP_MI_MP_Y_IRQ_OFFS_SHD	0x0E84
#define ISP_MI_MP_CB_BASE_AD_SHD	0x0E88
#define ISP_MI_MP_CB_SIZE_SHD		0x0E8C
#define ISP_MI_MP_CB_OFFS_CNT_SHD	0x0E90
#define ISP_MI_MP_CR_BASE_AD_SHD	0x0E94
#define ISP_MI_MP_CR_SIZE_SHD		0x0E98
#define ISP_MI_MP_CR_OFFS_CNT_SHD	0x0E9C
#define ISP_MI_IMSC			0x0EF8
#define ISP_MI_RIS			0x0EFC
#define ISP_MI_MIS			0x0F00
#define ISP_MI_ICR			0x0F04
#define ISP_MI_ISR			0x0F08
#define ISP_MI_STATUS			0x0F0C
#define ISP_MI_STATUS_CLR		0x0F10
#define ISP_MI_MP_Y_BASE_AD_INIT2	0x0F30
#define ISP_MI_MP_CB_BASE_AD_INIT2	0x0F34
#define ISP_MI_MP_CR_BASE_AD_INIT2	0x0F38
#define ISP_MI_MP_Y_LLENGTH		0x0F50
#define ISP_MI_OUTPUT_ALIGN_FORMAT	0x0F5C
#define ISP_MI_MP_OUTPUT_FIFO_SIZE	0x0F60
#define ISP_MI_MP_Y_PIC_WIDTH		0x0F64
#define ISP_MI_MP_Y_PIC_HEIGHT		0x0F68
#define ISP_MI_MP_Y_PIC_SIZE		0x0F6C

/* Important bit-fields */
#define MI_INIT_CFG_UPD			BIT(4)
#define ACQ_PROP_PIN_MAPPING_MASK	GENMASK(2, 0)
#define ACQ_PROP_PIN_MAPPING_SHIFT	17

/* ISP Interrupts */
#define INTR_EXP_END	BIT(18)
#define INTR_H_START	BIT(7)
#define INTR_V_START	BIT(6)
#define INTR_FRAME_IN	BIT(5)
#define INTR_AWB_DONE	BIT(4)
#define INTR_SIZE_ERR	BIT(3)
#define INTR_DATALOSS	BIT(2)


/* ISP MI Interrupts */
#define MI_INTR_WRAP_MP_CR	BIT(6)
#define MI_INTR_WRAP_MP_CB	BIT(5)
#define MI_INTR_WRAP_MP_Y	BIT(4)
#define MI_INTR_FILL_MP_Y	BIT(3)
#define MI_INTR_MBLK_LINE	BIT(2)
#define MI_INTR_MP_FRAME_END	BIT(0)

/* ISP constants. */
#define ISP_MIN_VBUF 1

#define TPG_BIT_WIDTH_8		0
#define TPG_BIT_WIDTH_10	1
#define TPG_BIT_WIDTH_12	2

enum bayer_pattern {
	RGGB = 0,
	GRBG,
	GBRG,
	BGGR,
};

struct isp_config {
	DEVICE_MMIO_ROM;
	void (*irq_config_func)(const struct device *dev);

	uint32_t irqn;
	uint32_t mi_irqn;
	const struct device *controller;

	uint32_t tpg_bayer_pattern: 2;
	uint32_t tpg_img_idx: 3;
	uint32_t tpg_pix_width: 2;
};

struct isp_data {
	DEVICE_MMIO_RAM;
	bool is_streaming;
	const struct device *dev;
	uint32_t curr_vid_buf;
	uint32_t mi_mis;

	struct k_fifo fifo_in;
	struct k_fifo fifo_out;

	struct k_work cb_work;
	struct k_work_q cb_workq;

	struct k_poll_signal *signal;

	struct isp_config_params init_cfg;
};

#endif /* _VIDEO_ALIF_ISP_H_ */
