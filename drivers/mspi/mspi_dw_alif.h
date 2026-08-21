/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Alif-specific helpers for the DesignWare MSPI/OSPI driver.
 *
 * This header is included only by mspi_dw.c after the DW register accessors
 * and driver private data structures are defined.
 */

#ifndef _MSPI_DW_ALIF_SPECIFIC_H_
#define _MSPI_DW_ALIF_SPECIFIC_H_

struct alif_ospi_aes_regs {
	uint32_t aes_ctrl;
	uint32_t aes_intr;
	uint32_t aes_intr_mask;
	uint32_t aes_clk_dis;
	uint32_t aes_addr_control;
	uint32_t aes_res_0;
	uint32_t aes_res_1;
	uint32_t aes_res_2;
	uint32_t aes_rxds_dly;
};

struct alif_mspi_vendor_data {
	volatile struct alif_ospi_aes_regs *aes_regs;
	uint8_t ddr_drive_edge;
	uint8_t rx_ds_delay;
	uint8_t baud2_delay;
};

#define ALIF_SPECIFIC_DATA_DEFINE(inst)						\
	static const struct alif_mspi_vendor_data mspi_dw_alif_##inst##_vendor_data = {	\
		.aes_regs = (void *)DT_INST_REG_ADDR_BY_NAME(inst, aes),		\
		.ddr_drive_edge = DT_INST_PROP_OR(inst, ddr_drive_edge, 0),		\
		.rx_ds_delay = DT_INST_PROP_OR(inst, rx_ds_delay, 0),			\
		.baud2_delay = DT_INST_PROP_OR(inst, baud2_delay, 0),			\
	}

#define ALIF_SPECIFIC_DATA_GET(inst) ((void *)&mspi_dw_alif_##inst##_vendor_data)

static inline const struct alif_mspi_vendor_data *
alif_vendor_data_get(const struct device *dev)
{
	const struct mspi_dw_config *config = dev->config;

	return config->vendor_specific_data;
}

#define ALIF_AES_CTRL_XIP_EN		BIT(4U)
#define ALIF_AUX_BAUD2_DELAY_MASK	BIT(30U)

static inline void alif_aes_set_rxds_delay(volatile struct alif_ospi_aes_regs *aes,
					   uint8_t delay)
{
#if defined(CONFIG_ENSEMBLE_GEN2)
	aes->aes_rxds_dly = (uint32_t)delay | ((uint32_t)delay << 8U);
#else
	aes->aes_rxds_dly = delay;
#endif
}

static inline void alif_aes_set_baud2_delay(volatile struct alif_ospi_aes_regs *aes,
					    uint8_t baud2_delay, uint32_t baudr)
{
#if defined(CONFIG_SOC_SERIES_E1C) || defined(CONFIG_SOC_SERIES_B1)
	bool enable;

	switch (baud2_delay) {
	case 1U:
		enable = true;
		break;
	case 2U:
		enable = (baudr == 2U);
		break;
	default:
		enable = false;
		break;
	}

	if (enable) {
		aes->aes_intr_mask |= ALIF_AUX_BAUD2_DELAY_MASK;
	} else {
		aes->aes_intr_mask &= ~ALIF_AUX_BAUD2_DELAY_MASK;
	}
#else
	ARG_UNUSED(aes);
	ARG_UNUSED(baud2_delay);
	ARG_UNUSED(baudr);
#endif
}

static inline void alif_apply_timing_config(const struct device *dev)
{
	const struct alif_mspi_vendor_data *data = alif_vendor_data_get(dev);
	const struct mspi_dw_data *dev_data = dev->data;

	if (data == NULL || data->aes_regs == NULL) {
		return;
	}

	alif_aes_set_rxds_delay(data->aes_regs, data->rx_ds_delay);
	alif_aes_set_baud2_delay(data->aes_regs, data->baud2_delay, dev_data->baudr);
}

static inline uint8_t alif_ddr_drive_edge(const struct device *dev)
{
	const struct alif_mspi_vendor_data *data = alif_vendor_data_get(dev);

	if (data != NULL) {
		return data->ddr_drive_edge;
	}

	return 0U;
}

#endif /* _MSPI_DW_ALIF_SPECIFIC_H_ */
