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

#ifndef _MSPI_DW_ALIF_H_
#define _MSPI_DW_ALIF_H_

struct alif_ospi_aes_regs {
	uint32_t AES_CTRL;
	uint32_t AES_INTR;
	uint32_t AES_INTR_MASK;
	uint32_t AES_CLK_DIS;
	uint32_t AES_ADDR_CONTROL;
	uint32_t AES_RES_0;
	uint32_t AES_RES_1;
	uint32_t AES_RES_2;
	uint32_t AES_RXDS_DLY;
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

#define ALIF_SPECIFIC_DATA_GET(inst) (void *)&mspi_dw_alif_##inst##_vendor_data

static inline const struct alif_mspi_vendor_data *
alif_vendor_data_get(const struct device *dev)
{
	const struct mspi_dw_config *dev_config = dev->config;

	return dev_config->vendor_specific_data;
}

#define ALIF_AES_CTRL_XIP_EN BIT(4U)
#define ALIF_AUX_BAUD2_DELAY_MASK BIT(30U)

#if defined(CONFIG_ENSEMBLE_GEN2)
#define ALIF_AUX_SIGNAL_0_DELAY_POS 0U
#define ALIF_AUX_SIGNAL_1_DELAY_POS 8U
static inline void alif_aes_set_rxds_delay(volatile struct alif_ospi_aes_regs *aes,
					   uint8_t delay)
{
	aes->AES_RXDS_DLY = ((uint32_t)delay << ALIF_AUX_SIGNAL_0_DELAY_POS) |
			    ((uint32_t)delay << ALIF_AUX_SIGNAL_1_DELAY_POS);
}
#else
static inline void alif_aes_set_rxds_delay(volatile struct alif_ospi_aes_regs *aes,
					   uint8_t delay)
{
	aes->AES_RXDS_DLY = delay;
}
#endif

static inline void alif_aes_set_baud2_delay(volatile struct alif_ospi_aes_regs *aes,
					    uint8_t baud2_delay, uint32_t baudr)
{
#if defined(CONFIG_SOC_SERIES_E1C) || defined(CONFIG_SOC_SERIES_B1)
	bool enable = false;

	switch (baud2_delay) {
	case 0U:
		enable = false;
		break;
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
		aes->AES_INTR_MASK |= ALIF_AUX_BAUD2_DELAY_MASK;
	} else {
		aes->AES_INTR_MASK &= ~ALIF_AUX_BAUD2_DELAY_MASK;
	}
#else
	ARG_UNUSED(aes);
	ARG_UNUSED(baud2_delay);
	ARG_UNUSED(baudr);
#endif
}

static inline void alif_apply_timing_config(const struct device *dev)
{
	const struct alif_mspi_vendor_data *alif_data = alif_vendor_data_get(dev);
	const struct mspi_dw_data *dev_data = dev->data;

	if (alif_data == NULL || alif_data->aes_regs == NULL) {
		return;
	}

	alif_aes_set_rxds_delay(alif_data->aes_regs, alif_data->rx_ds_delay);
	alif_aes_set_baud2_delay(alif_data->aes_regs, alif_data->baud2_delay,
				 dev_data->baudr);
}

static inline uint8_t alif_ddr_drive_edge(const struct device *dev)
{
	const struct alif_mspi_vendor_data *alif_data = alif_vendor_data_get(dev);

	return (alif_data != NULL) ? alif_data->ddr_drive_edge : 0U;
}

#if defined(CONFIG_MSPI_XIP)
static inline void alif_xip_update_ctrl(const struct device *dev, struct xip_ctrl *ctrl)
{
	const struct alif_mspi_vendor_data *alif_data = alif_vendor_data_get(dev);
	const struct mspi_dw_data *dev_data = dev->data;

	if (alif_data == NULL) {
		return;
	}

	/*
	 * Mirror the controller-side settings already selected through
	 * mspi_dev_config(). On Alif OSPI, the XIP path uses separate XIP
	 * control registers, so DDR and RXDS/DQS need to be reflected here
	 * explicitly.
	 */
	ctrl->read |= XIP_CTRL_DFS_HC_BIT;
	ctrl->write |= XIP_WRITE_CTRL_DFS_HC_BIT;

	if (dev_data->spi_ctrlr0 & SPI_CTRLR0_SPI_DDR_EN_BIT) {
		ctrl->read |= XIP_CTRL_DDR_EN_BIT;
		ctrl->write |= XIP_WRITE_CTRL_SPI_DDR_EN_BIT;
	}

	if (dev_data->spi_ctrlr0 & SPI_CTRLR0_INST_DDR_EN_BIT) {
		ctrl->read |= XIP_CTRL_INST_DDR_EN_BIT;
		ctrl->write |= XIP_WRITE_CTRL_INST_DDR_EN_BIT;
	}

	if (dev_data->spi_ctrlr0 & SPI_CTRLR0_SPI_RXDS_EN_BIT) {
		ctrl->read |= XIP_CTRL_RXDS_EN_BIT;
	}
}

static inline void alif_xip_prepare_registers(const struct device *dev)
{
	write_xip_mode_bits(dev, 0);
	write_xip_cnt_time_out(dev, 100);
}

static inline int alif_xip_select(const struct device *dev,
				  const struct mspi_dev_id *dev_id,
				  bool enable)
{
	uint32_t mask;

	if (dev_id->dev_idx >= 2U) {
		return -EINVAL;
	}

	mask = BIT(dev_id->dev_idx);

	if (enable) {
		write_xip_ser(dev, read_xip_ser(dev) | mask);
	} else {
		write_xip_ser(dev, read_xip_ser(dev) & ~mask);
	}

	return 0;
}

static inline int alif_xip_enable(const struct device *dev,
				  const struct mspi_dev_id *dev_id,
				  const struct mspi_xip_cfg *cfg)
{
	const struct alif_mspi_vendor_data *alif_data = alif_vendor_data_get(dev);
	volatile struct alif_ospi_aes_regs *aes;
	int ret;

	ARG_UNUSED(cfg);

	if (alif_data == NULL || alif_data->aes_regs == NULL) {
		return -ENODEV;
	}

	aes = alif_data->aes_regs;

	ret = alif_xip_select(dev, dev_id, true);
	if (ret < 0) {
		return ret;
	}

	aes->AES_CTRL |= ALIF_AES_CTRL_XIP_EN;

	return 0;
}

static inline int alif_xip_disable(const struct device *dev,
				   const struct mspi_dev_id *dev_id,
				   const struct mspi_xip_cfg *cfg)
{
	struct mspi_dw_data *dev_data = dev->data;
	const struct alif_mspi_vendor_data *alif_data = alif_vendor_data_get(dev);
	volatile struct alif_ospi_aes_regs *aes;
	int ret;

	ARG_UNUSED(cfg);

	if (alif_data == NULL || alif_data->aes_regs == NULL) {
		return -ENODEV;
	}

	aes = alif_data->aes_regs;

	ret = alif_xip_select(dev, dev_id, false);
	if (ret < 0) {
		return ret;
	}

	if ((dev_data->xip_enabled & ~BIT(dev_id->dev_idx)) == 0U) {
		aes->AES_CTRL &= ~ALIF_AES_CTRL_XIP_EN;
	}

	return 0;
}

#endif /* CONFIG_MSPI_XIP */

#endif /* _MSPI_DW_ALIF_H_ */
