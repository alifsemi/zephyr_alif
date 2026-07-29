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
	uint8_t hyperbus_dfs;
	bool hyperbus_mode;
};

#define ALIF_SPECIFIC_DATA_DEFINE(inst)						\
	static const struct alif_mspi_vendor_data mspi_dw_alif_##inst##_vendor_data = {	\
		.aes_regs = (void *)DT_INST_REG_ADDR_BY_NAME(inst, aes),		\
		.ddr_drive_edge = DT_INST_PROP_OR(inst, ddr_drive_edge, 0),		\
		.rx_ds_delay = DT_INST_PROP_OR(inst, rx_ds_delay, 0),			\
		.baud2_delay = DT_INST_PROP_OR(inst, baud2_delay, 0),			\
		.hyperbus_dfs = DT_INST_PROP_OR(inst, hyperbus_dfs, 16),			\
		.hyperbus_mode = DT_INST_PROP(inst, hyperbus_mode),			\
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

static inline int alif_validate_dev_config(const struct device *dev,
					   enum mspi_dev_cfg_mask param_mask,
					   const struct mspi_dev_cfg *cfg)
{
	const struct alif_mspi_vendor_data *data = alif_vendor_data_get(dev);

	if (data == NULL || !data->hyperbus_mode) {
		return 0;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_IO_MODE) &&
	    cfg->io_mode != MSPI_IO_MODE_OCTAL) {
		LOG_ERR("Alif HyperBus supports only Octal I/O mode");
		return -ENOTSUP;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_DATA_RATE) &&
	    cfg->data_rate != MSPI_DATA_RATE_DUAL) {
		LOG_ERR("Alif HyperBus supports only dual data rate");
		return -ENOTSUP;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_DQS) && !cfg->dqs_enable) {
		LOG_ERR("Alif HyperBus requires RWDS");
		return -ENOTSUP;
	}

	return 0;
}

#if defined(CONFIG_MSPI_XIP)
static inline void alif_xip_update_ctrl(const struct device *dev, struct xip_ctrl *ctrl,
					const struct mspi_xip_cfg *cfg)
{
	struct mspi_dw_data *dev_data = dev->data;
	const struct alif_mspi_vendor_data *data = alif_vendor_data_get(dev);

	if (data->hyperbus_mode) {
		uint32_t dfs = data->hyperbus_dfs - 1U;

		/* HyperBus generates its command/address phase from the mapped address. */
		ctrl->read = (ctrl->read & XIP_CTRL_WAIT_CYCLES_MASK) |
			     XIP_CTRL_XIP_HYBERBUS_EN_BIT |
			     XIP_CTRL_RXDS_SIG_EN_BIT |
			     XIP_CTRL_DFS_HC_BIT |
			     FIELD_PREP(XIP_CTRL_TRANS_TYPE_MASK,
					XIP_CTRL_TRANS_TYPE_TT2);
		ctrl->write = (ctrl->write & XIP_WRITE_CTRL_WAIT_CYCLES_MASK) |
			      XIP_WRITE_CTRL_DFS_HC_BIT |
			      XIP_WRITE_CTRL_DM_EN_BIT |
			      XIP_WRITE_CTRL_RXDS_SIG_EN_BIT |
			      XIP_WRITE_CTRL_HYBERBUS_EN_BIT |
			      FIELD_PREP(XIP_WRITE_CTRL_TRANS_TYPE_MASK,
					 XIP_WRITE_CTRL_TRANS_TYPE_TT2);

		dev_data->ctrlr0 &= ~(CTRLR0_TMOD_MASK |
				      CTRLR0_DFS_MASK |
				      CTRLR0_DFS32_MASK);
		dev_data->ctrlr0 |= FIELD_PREP(CTRLR0_DFS_MASK, dfs) |
				   FIELD_PREP(CTRLR0_DFS32_MASK, dfs) |
				   FIELD_PREP(CTRLR0_TMOD_MASK, CTRLR0_TMOD_RX);
		dev_data->spi_ctrlr0 = SPI_CTRLR0_SPI_DM_EN_BIT;
		return;
	}

	ctrl->read |= XIP_CTRL_DFS_HC_BIT;

	if (dev_data->spi_ctrlr0 & SPI_CTRLR0_SPI_DDR_EN_BIT) {
		ctrl->read |= XIP_CTRL_DDR_EN_BIT;
	}

	if (dev_data->spi_ctrlr0 & SPI_CTRLR0_INST_DDR_EN_BIT) {
		ctrl->read |= XIP_CTRL_INST_DDR_EN_BIT;
	}

	if (dev_data->spi_ctrlr0 & SPI_CTRLR0_SPI_RXDS_EN_BIT) {
		ctrl->read |= XIP_CTRL_RXDS_EN_BIT;
	}

	/* TODO: XiP write support ? */
	if (cfg->permission == MSPI_XIP_READ_WRITE) {
		ctrl->write |= XIP_WRITE_CTRL_DFS_HC_BIT;

		if (dev_data->spi_ctrlr0 & SPI_CTRLR0_SPI_DDR_EN_BIT) {
			ctrl->write |= XIP_WRITE_CTRL_SPI_DDR_EN_BIT;
		}

		if (dev_data->spi_ctrlr0 & SPI_CTRLR0_INST_DDR_EN_BIT) {
			ctrl->write |= XIP_WRITE_CTRL_INST_DDR_EN_BIT;
		}
	}
}

static inline int alif_xip_enable(const struct device *dev,
				  const struct mspi_dev_id *dev_id,
				  const struct mspi_xip_cfg *cfg)
{
	const struct alif_mspi_vendor_data *data = alif_vendor_data_get(dev);

	ARG_UNUSED(dev_id);
	ARG_UNUSED(cfg);

	if (data == NULL || data->aes_regs == NULL) {
		return -ENODEV;
	}

	data->aes_regs->aes_ctrl |= ALIF_AES_CTRL_XIP_EN;
	return 0;
}

static inline int alif_xip_disable(const struct device *dev,
				   const struct mspi_dev_id *dev_id,
				   const struct mspi_xip_cfg *cfg)
{
	struct mspi_dw_data *dev_data = dev->data;
	const struct alif_mspi_vendor_data *data = alif_vendor_data_get(dev);

	ARG_UNUSED(cfg);

	if (data == NULL || data->aes_regs == NULL) {
		return -ENODEV;
	}

	if ((dev_data->xip_enabled & ~BIT(dev_id->dev_idx)) == 0U) {
		data->aes_regs->aes_ctrl &= ~ALIF_AES_CTRL_XIP_EN;
	}

	return 0;
}
#endif /* CONFIG_MSPI_XIP */

#endif /* _MSPI_DW_ALIF_SPECIFIC_H_ */
