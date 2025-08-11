/*
 * Copyright (c) 2015 Intel Corporation.
 * Copyright (c) 2023 Synopsys, Inc. All rights reserved.
 * Copyright (c) 2023 Meta Platforms
 * Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_designware_spi

/* spi_dw.c - Designware SPI driver implementation */

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_dw);

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/device.h>

#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#ifdef CONFIG_IOAPIC
#include <zephyr/drivers/interrupt_controller/ioapic.h>
#endif

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/irq.h>

#include "spi_dw.h"
#include "spi_context.h"

#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

#ifdef CONFIG_SPI_DW_USE_DMA
#include <zephyr/drivers/dma.h>
#endif

static inline bool spi_dw_is_slave(struct spi_dw_data *spi)
{
	return (IS_ENABLED(CONFIG_SPI_SLAVE) &&
		spi_context_is_slave(&spi->ctx));
}

static void completed(const struct device *dev, int error)
{
	struct spi_dw_data *spi = dev->data;
	struct spi_context *ctx = &spi->ctx;

	if (error) {
		goto out;
	}

	if (spi_context_tx_on(&spi->ctx) ||
	    spi_context_rx_on(&spi->ctx)) {
		return;
	}

out:
#ifdef CONFIG_SPI_DW_USE_DMA
	const struct spi_dw_config *info = dev->config;

	if (info->dma_tx.enabled || info->dma_rx.enabled) {
		/* Disabling interrupts */
		write_imr(dev, DW_SPI_IMR_MASK);

		/* Disabling the controller */
		clear_bit_ssienr(dev);

		spi->dma_cb_status = error;
		k_sem_give(&spi->dma_sem);
		LOG_DBG("SPI:%p DMA transaction finished %s error",
			    dev, error ? "with" : "without");
		return;
	}
#endif /* CONFIG_SPI_DW_USE_DMA */

	/* need to give time for FIFOs to drain before issuing more commands */
	while (test_bit_sr_busy(dev)) {
	}

	/* Disabling interrupts */
	write_imr(dev, DW_SPI_IMR_MASK);
	/* Disabling the controller */
	clear_bit_ssienr(dev);

	if (!spi_dw_is_slave(spi)) {
		if (spi_cs_is_gpio(ctx->config)) {
			spi_context_cs_control(ctx, false);
		} else {
			write_ser(dev, 0);
		}
	}

	LOG_DBG("SPI transaction completed %s error",
		    error ? "with" : "without");

	spi_context_complete(&spi->ctx, dev, error);
}

static void push_data(const struct device *dev)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
	uint32_t data = 0U;
	uint32_t f_tx;

	if (spi_context_rx_on(&spi->ctx)) {
		f_tx = info->fifo_depth - read_txflr(dev) -
			read_rxflr(dev);
		if ((int)f_tx < 0) {
			f_tx = 0U; /* if rx-fifo is full, hold off tx */
		}
	} else {
		f_tx = info->fifo_depth - read_txflr(dev);
	}

	while (f_tx) {
		if (spi_context_tx_buf_on(&spi->ctx)) {
			switch (spi->dfs) {
			case 1:
				data = UNALIGNED_GET((uint8_t *)
						     (spi->ctx.tx_buf));
				break;
			case 2:
				data = UNALIGNED_GET((uint16_t *)
						     (spi->ctx.tx_buf));
				break;
			case 4:
				data = UNALIGNED_GET((uint32_t *)
						     (spi->ctx.tx_buf));
				break;
			}
		} else if (spi_context_rx_on(&spi->ctx)) {
			/* No need to push more than necessary */
			if ((int)(spi->ctx.rx_len - spi->fifo_diff) <= 0) {
				break;
			}

			data = 0U;
		} else if (spi_context_tx_on(&spi->ctx)) {
			data = 0U;
		} else {
			/* Nothing to push anymore */
			break;
		}

		write_dr(dev, data);

		spi_context_update_tx(&spi->ctx, spi->dfs, 1);
		spi->fifo_diff++;

		f_tx--;
	}

	if (!spi_context_tx_on(&spi->ctx)) {
		/* prevents any further interrupts demanding TX fifo fill */
		write_txftlr(dev, 0);
	}
}

static void pull_data(const struct device *dev)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;

	while (read_rxflr(dev)) {
		uint32_t data = read_dr(dev);

		if (spi_context_rx_buf_on(&spi->ctx)) {
			switch (spi->dfs) {
			case 1:
				UNALIGNED_PUT(data, (uint8_t *)spi->ctx.rx_buf);
				break;
			case 2:
				UNALIGNED_PUT(data, (uint16_t *)spi->ctx.rx_buf);
				break;
			case 4:
				UNALIGNED_PUT(data, (uint32_t *)spi->ctx.rx_buf);
				break;
			}
		}

		spi_context_update_rx(&spi->ctx, spi->dfs, 1);
		spi->fifo_diff--;
	}

	if (!spi->ctx.rx_len && spi->ctx.tx_len < info->fifo_depth) {
		write_rxftlr(dev, spi->ctx.tx_len - 1);
	} else if (read_rxftlr(dev) >= spi->ctx.rx_len) {
		write_rxftlr(dev, spi->ctx.rx_len - 1);
	}
}

#ifdef CONFIG_SPI_DW_USE_DMA
static uint32_t spi_dw_dma_calc_dmardlr(const struct spi_dw_config *info, struct spi_dw_data *spi)
{
	uint32_t dw_spi_rxftlr_dflt = (info->fifo_depth * 1) / 2;
	uint32_t burst_length =  dw_spi_rxftlr_dflt ? dw_spi_rxftlr_dflt : 1;
	uint32_t total_burst_length = spi_context_max_continuous_chunk(&spi->ctx);

	while (total_burst_length % burst_length) {
		burst_length--;
	}

	return burst_length;
}

static void spi_dw_dma_callback(const struct device *dma_dev, void *user_data,
	uint32_t channel, int status)
{
	const struct device *dev = (const struct device *)user_data;
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;

	spi->dma_cb_status = status;

	if (channel == info->dma_tx.ch && !spi_context_rx_buf_on(&spi->ctx)) {
		k_sem_give(&spi->dma_sem);
	}

	if (channel == info->dma_rx.ch && spi_context_rx_buf_on(&spi->ctx)) {
		k_sem_give(&spi->dma_sem);
	}

	if (status < 0) {
		LOG_ERR("SPI:%p dma:%p ch:%d callback gets error: %d", dev, dma_dev, channel,
			status);
	} else {
		if (channel == info->dma_tx.ch) {
			spi_context_update_tx(&spi->ctx,
				spi->dfs,
				spi_context_max_continuous_chunk(&spi->ctx));
		}
		if (channel == info->dma_rx.ch) {
			spi_context_update_rx(&spi->ctx,
				spi->dfs,
				spi_context_max_continuous_chunk(&spi->ctx));
		}
		LOG_DBG("SPI:%p dma:%p ch:%d transaction completed: %d", dev, dma_dev, channel,
			status);
	}
}

static int spi_dw_start_dma_ch(const struct device *dev)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
	struct dma_config dma_cfg = { 0 };
	struct dma_block_config dma_block_cfg = { 0 };
	int ret = 0;
	uint32_t dw_spi_txftlr_dflt = (info->fifo_depth * 1) / 2;

	dma_cfg.block_count = 1U;
	dma_cfg.dma_callback = spi_dw_dma_callback;
	dma_cfg.error_callback_dis = 1U;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.head_block = &dma_block_cfg;
	dma_cfg.source_data_size = dma_cfg.dest_data_size = spi->dfs;
	dma_block_cfg.block_size = spi->dfs *
			spi_context_max_continuous_chunk(&spi->ctx);

	if (spi_context_rx_buf_on(&spi->ctx)) {
		uint32_t dma_rdlr = spi_dw_dma_calc_dmardlr(info, spi);

		write_dmardlr(dev, dma_rdlr - 1);
		dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;

		dma_cfg.dma_slot = info->dma_rx.periph;
		dma_cfg.dest_burst_length = dma_rdlr;
		dma_cfg.source_burst_length = dma_cfg.dest_burst_length;

		dma_block_cfg.source_address = (mm_reg_t)DEVICE_MMIO_GET(dev) + DW_SPI_REG_DR;

		dma_block_cfg.dest_address = POINTER_TO_UINT(spi->ctx.rx_buf);
		dma_block_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_block_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

		ret = dma_config(info->dma_dev, info->dma_rx.ch, &dma_cfg);
		if (ret < 0) {
			LOG_ERR("SPI:%p dma_config %p failed %d\n", dev, info->dma_dev, ret);
			return ret;
		}

		ret = dma_start(info->dma_dev, info->dma_rx.ch);
		if (ret < 0) {
			LOG_ERR("SPI:%p dma_start %p failed %d\n", dev, info->dma_dev, ret);
			return ret;
		}
	}
	if (spi_context_tx_buf_on(&spi->ctx)) {
		dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		dma_cfg.dest_burst_length = info->fifo_depth - dw_spi_txftlr_dflt;
		write_dmatdlr(dev, dw_spi_txftlr_dflt);
		LOG_DBG("SPI:%p DMATDLR: %u", dev, dma_cfg.dest_burst_length);

		dma_cfg.dma_slot = info->dma_tx.periph;
		dma_cfg.source_burst_length = dma_cfg.dest_burst_length;

		dma_block_cfg.source_address = POINTER_TO_UINT(spi->ctx.tx_buf);
		dma_block_cfg.dest_address = (mm_reg_t)DEVICE_MMIO_GET(dev) + DW_SPI_REG_DR;
		dma_block_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_block_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;

		ret = dma_config(info->dma_dev, info->dma_tx.ch, &dma_cfg);
		if (ret < 0) {
			LOG_ERR("SPI:%p dma_config %p failed %d\n", dev, info->dma_dev, ret);
			return ret;
		}

		ret = dma_start(info->dma_dev, info->dma_tx.ch);
		if (ret < 0) {
			LOG_ERR("SPI:%p dma_start %p failed %d\n", dev, info->dma_dev, ret);
			return ret;
		}
	}
	return ret;
}

static int spi_dw_dma_transceive(const struct device *dev,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
	uint32_t reg_data;
	int ret;

	reg_data = read_dmacr(dev);
	if (info->dma_tx.enabled && tx_bufs && tx_bufs->buffers) {
		reg_data |= DW_SPI_DMACR_TDMAE;
	}
	if (info->dma_rx.enabled && rx_bufs && rx_bufs->buffers) {
		reg_data |= DW_SPI_DMACR_RDMAE;
	}

	/* Disabling the controller */
	clear_bit_ssienr(dev);

	write_dmacr(dev, reg_data);

	/* Enabling the controller */
	set_bit_ssienr(dev);

	ret = spi_dw_start_dma_ch(dev);
	if (ret) {
		goto dma_out;
	}

	k_sem_take(&spi->dma_sem, K_FOREVER);

	if (spi->dma_cb_status < 0) {

		if (spi_context_rx_buf_on(&spi->ctx)) {
			ret = dma_stop(info->dma_dev, info->dma_rx.ch);
			if (ret < 0) {
				LOG_ERR("SPI:%p dma stop %p failed for rx %d\n",
					    dev, info->dma_dev, ret);
			}
		}
		if (spi_context_tx_buf_on(&spi->ctx)) {
			ret = dma_stop(info->dma_dev, info->dma_tx.ch);
			if (ret < 0) {
				LOG_ERR("SPI:%p dma stop %p failed for tx %d\n",
					    dev, info->dma_dev, ret);
			}
		}

		ret = spi->dma_cb_status;
	} else {
		/*
		 *  Need to give time for FIFOs to drain before
		 *  issuing more commands
		 */
		while (test_bit_sr_busy(dev)) {
		}
#ifdef CONFIG_SPI_SLAVE
		if (spi_context_is_slave(&spi->ctx)) {
			return spi->ctx.recv_frames;
		}
#endif /* CONFIG_SPI_SLAVE */
	}

dma_out:
	/* Disabling interrupts */
	write_imr(dev, DW_SPI_IMR_MASK);

	/* Disabling the controller */
	clear_bit_ssienr(dev);

	/* Disabling the DMA */
	write_dmacr(dev, 0);

	if (!spi_dw_is_slave(spi)) {
		spi_context_cs_control(&spi->ctx, false);
	}

	spi_context_complete(&spi->ctx, dev, spi->dma_cb_status);
	spi->dma_cb_status = 0;

	return ret;
}
#endif /* CONFIG_SPI_DW_USE_DMA */

static int spi_dw_configure(const struct device *dev,
			    struct spi_dw_data *spi,
			    const struct spi_config *config)
{
	const struct spi_dw_config *info = dev->config;
	uint32_t ctrlr0 = 0U;

	LOG_DBG("%p (prev %p)", config, spi->ctx.config);

	if (spi_context_configured(&spi->ctx, config)) {
		/* Nothing to do */
		return 0;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	/* Verify if requested op mode is relevant to this controller */
	if (config->operation & SPI_OP_MODE_SLAVE) {
		if (!(info->serial_target)) {
			LOG_ERR("Slave mode not supported");
			return -ENOTSUP;
		}
	} else {
		if (info->serial_target) {
			LOG_ERR("Master mode not supported");
			return -ENOTSUP;
		}
	}

	if ((config->operation & SPI_TRANSFER_LSB) ||
	    (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	     (config->operation & (SPI_LINES_DUAL |
				   SPI_LINES_QUAD | SPI_LINES_OCTAL)))) {
		LOG_ERR("Unsupported configuration");
		return -EINVAL;
	}

	if (info->max_xfer_size < SPI_WORD_SIZE_GET(config->operation)) {
		LOG_ERR("Max xfer size is %u, word size of %u not allowed",
			info->max_xfer_size, SPI_WORD_SIZE_GET(config->operation));
		return -ENOTSUP;
	}

	/* Word size */
	if (info->max_xfer_size == 32) {
		if (spi->dwc_ssi) {
			ctrlr0 |= DW_SPI_CTRLR0_DFS_16(SPI_WORD_SIZE_GET(config->operation));
		} else {
			ctrlr0 |= DW_SPI_CTRLR0_DFS_32(SPI_WORD_SIZE_GET(config->operation));
		}
	} else {
		ctrlr0 |= DW_SPI_CTRLR0_DFS_16(SPI_WORD_SIZE_GET(config->operation));
	}

	/* Setting SSI_IS_MST in CTRLR0 if it is dwc_ssi on AHB */
	if (!(config->operation & SPI_OP_MODE_SLAVE) && spi->dwc_ssi) {
		ctrlr0 |= DWC_SSI_SPI_IS_MST_BIT;
	}

	/* Determine how many bytes are required per-frame */
	spi->dfs = SPI_WS_TO_DFS(SPI_WORD_SIZE_GET(config->operation));

	/* SPI mode */
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		ctrlr0 |= (spi->dwc_ssi) ? DWC_SSI_SPI_CTRLR0_SCPOL : DW_SPI_CTRLR0_SCPOL;
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		ctrlr0 |= (spi->dwc_ssi) ? DWC_SSI_SPI_CTRLR0_SCPH : DW_SPI_CTRLR0_SCPH;
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_LOOP) {
		ctrlr0 |= (spi->dwc_ssi) ? DWC_SSI_SPI_CTRLR0_SRL : DW_SPI_CTRLR0_SRL;
	}

	/* Installing the configuration */
	write_ctrlr0(dev, ctrlr0);

	/* At this point, it's mandatory to set this on the context! */
	spi->ctx.config = config;

	if (!spi_dw_is_slave(spi)) {
		/* Baud rate and Slave select, for master only */
		write_baudr(dev, SPI_DW_CLK_DIVIDER(info->clock_frequency,
						    config->frequency));
		write_ser(dev, 1 << config->slave);
	}

	if (spi_dw_is_slave(spi)) {
		LOG_DBG("Installed slave config %p:"
			    " ws/dfs %u/%u, mode %u/%u/%u",
			    config,
			    SPI_WORD_SIZE_GET(config->operation), spi->dfs,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPOL) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPHA) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_LOOP) ? 1 : 0);
	} else {
		LOG_DBG("Installed master config %p: freq %uHz (div = %u),"
			    " ws/dfs %u/%u, mode %u/%u/%u, slave %u",
			    config, config->frequency,
			    SPI_DW_CLK_DIVIDER(info->clock_frequency,
					       config->frequency),
			    SPI_WORD_SIZE_GET(config->operation), spi->dfs,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPOL) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPHA) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_LOOP) ? 1 : 0,
			    config->slave);
	}

	return 0;
}

static uint32_t spi_dw_compute_ndf(const struct spi_buf *rx_bufs,
				   size_t rx_count, uint8_t dfs)
{
	uint32_t len = 0U;

	for (; rx_count; rx_bufs++, rx_count--) {
		if (len > (UINT16_MAX - rx_bufs->len)) {
			goto error;
		}

		len += rx_bufs->len;
	}

	if (len) {
		return (len / dfs) - 1;
	}
error:
	return UINT32_MAX;
}

static void spi_dw_update_txftlr(const struct device *dev,
				 struct spi_dw_data *spi)
{
	const struct spi_dw_config *info = dev->config;
	uint32_t dw_spi_txftlr_dflt = (info->fifo_depth * 1) / 2;
	uint32_t reg_data = dw_spi_txftlr_dflt;

	if (spi_dw_is_slave(spi)) {
		if (!spi->ctx.tx_len) {
			reg_data = 0U;
		} else if (spi->ctx.tx_len < dw_spi_txftlr_dflt) {
			reg_data = spi->ctx.tx_len - 1;
		}
	}

	LOG_DBG("TxFTLR: %u", reg_data);

	write_txftlr(dev, reg_data);
}

static int transceive(const struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
		      spi_callback_t cb,
		      void *userdata)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
	uint32_t tmod = DW_SPI_CTRLR0_TMOD_TX_RX;
	uint32_t dw_spi_rxftlr_dflt = (info->fifo_depth * 1) / 2;
	uint32_t reg_data;
	int ret;

	/* Disabling the controller */
	clear_bit_ssienr(dev);

	spi_context_lock(&spi->ctx, asynchronous, cb, userdata, config);

#ifdef CONFIG_PM_DEVICE
	if (!pm_device_is_busy(dev)) {
		pm_device_busy_set(dev);
	}
#endif /* CONFIG_PM_DEVICE */

	/* Configure */
	ret = spi_dw_configure(dev, spi, config);
	if (ret) {
		goto out;
	}

	if (!rx_bufs || !rx_bufs->buffers) {
		tmod = (spi->dwc_ssi) ? DWC_SSI_SPI_CTRLR0_TMOD_TX : DW_SPI_CTRLR0_TMOD_TX;
	} else if (!tx_bufs || !tx_bufs->buffers) {
		tmod = (spi->dwc_ssi) ? DWC_SSI_SPI_CTRLR0_TMOD_RX : DW_SPI_CTRLR0_TMOD_RX;
	}

	/* ToDo: add a way to determine EEPROM mode */

	if (tmod >= ((spi->dwc_ssi) ? DWC_SSI_SPI_CTRLR0_TMOD_RX : DW_SPI_CTRLR0_TMOD_RX) &&
	    !spi_dw_is_slave(spi)) {
		reg_data = spi_dw_compute_ndf(rx_bufs->buffers,
					      rx_bufs->count,
					      spi->dfs);
		if (reg_data == UINT32_MAX) {
			ret = -EINVAL;
			goto out;
		}

		write_ctrlr1(dev, reg_data);
	} else {
		write_ctrlr1(dev, 0);
	}

	if (spi_dw_is_slave(spi)) {
		/* Enabling MISO line relevantly */
		if ((tmod == DW_SPI_CTRLR0_TMOD_RX) || (tmod == DWC_SSI_SPI_CTRLR0_TMOD_RX)) {
			tmod |= (spi->dwc_ssi) ? DWC_SSI_SPI_CTRLR0_SLV_OE : DW_SPI_CTRLR0_SLV_OE;
		} else {
			tmod &= (spi->dwc_ssi) ? ~DWC_SSI_SPI_CTRLR0_SLV_OE : ~DW_SPI_CTRLR0_SLV_OE;
		}
	}

	/* Updating TMOD in CTRLR0 register */
	reg_data = read_ctrlr0(dev);
	reg_data &= (spi->dwc_ssi) ? ~DWC_SSI_SPI_CTRLR0_TMOD_RESET : ~DW_SPI_CTRLR0_TMOD_RESET;
	reg_data |= tmod;

	/* Updating SSTE */
	if (spi->dwc_ssi) {
		if (info->slv_slct_toggle) {
			reg_data |= DWC_SSI_CTRLR0_SSTE;
		} else {
			reg_data &= ~DWC_SSI_CTRLR0_SSTE;
		}
	} else {
		if (info->slv_slct_toggle) {
			reg_data |= DW_SPI_CTRLR0_SSTE;
		} else {
			reg_data &= ~DW_SPI_CTRLR0_SSTE;
		}
	}

	write_ctrlr0(dev, reg_data);

	/* Set buffers info */
	spi_context_buffers_setup(&spi->ctx, tx_bufs, rx_bufs, spi->dfs);

	spi->fifo_diff = 0U;

	/* Tx Threshold */
	spi_dw_update_txftlr(dev, spi);

	/* Does Rx thresholds needs to be lower? */
	reg_data = dw_spi_rxftlr_dflt;

	if (spi_dw_is_slave(spi)) {
		if (spi->ctx.rx_len &&
		    spi->ctx.rx_len < dw_spi_rxftlr_dflt) {
			reg_data = spi->ctx.rx_len - 1;
		}
	} else {
		if (spi->ctx.rx_len && spi->ctx.rx_len < info->fifo_depth) {
			reg_data = spi->ctx.rx_len - 1;
		}
	}

	/* Rx Threshold */
	write_rxftlr(dev, reg_data);

	/* Enable interrupts */
	reg_data = DW_SPI_IMR_UNMASK;
	if ((!tx_bufs || !tx_bufs->buffers)) {
		reg_data &= DW_SPI_IMR_MASK_TX;
	}
	if ((!rx_bufs || !rx_bufs->buffers)) {
		reg_data &= DW_SPI_IMR_MASK_RX;
	}

#ifdef CONFIG_SPI_DW_USE_DMA
	if (info->dma_tx.enabled && tx_bufs && tx_bufs->buffers) {
		reg_data &= ~(DW_SPI_IMR_TXEIM);
	}
	if (info->dma_rx.enabled && rx_bufs && rx_bufs->buffers) {
		reg_data &= ~(DW_SPI_IMR_RXFIM);
	}
#endif

	write_imr(dev, reg_data);

	if (!spi_dw_is_slave(spi)) {
		/* if cs is not defined as gpio, use hw cs */
		if (spi_cs_is_gpio(config)) {
			spi_context_cs_control(&spi->ctx, true);
		} else {
			write_ser(dev, BIT(config->slave));
		}
	}

	LOG_DBG("Enabling controller");
	set_bit_ssienr(dev);

	/* Do a dummy write in case of rx only */
	if (!spi_dw_is_slave(spi) && (!tx_bufs || !tx_bufs->buffers)) {
		write_dr(dev, 0x0);
	}

#ifdef CONFIG_SPI_SLAVE
	if (spi_context_is_slave(&spi->ctx) && !ret) {
		ret = spi->ctx.recv_frames;
	}
#endif /* CONFIG_SPI_SLAVE */

#ifdef CONFIG_SPI_DW_USE_DMA
	if (info->dma_tx.enabled || info->dma_rx.enabled) {
		ret = spi_dw_dma_transceive(dev, tx_bufs, rx_bufs);
	} else {
#endif
		ret = spi_context_wait_for_completion(&spi->ctx);
#ifdef CONFIG_SPI_DW_USE_DMA
	}
#endif

out:
	spi_context_release(&spi->ctx, ret);

	pm_device_busy_clear(dev);

	return ret;
}

static int spi_dw_transceive(const struct device *dev,
			     const struct spi_config *config,
			     const struct spi_buf_set *tx_bufs,
			     const struct spi_buf_set *rx_bufs)
{
	LOG_DBG("%p, %p, %p", dev, tx_bufs, rx_bufs);

	return transceive(dev, config, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_dw_transceive_async(const struct device *dev,
				   const struct spi_config *config,
				   const struct spi_buf_set *tx_bufs,
				   const struct spi_buf_set *rx_bufs,
				   spi_callback_t cb,
				   void *userdata)
{
	LOG_DBG("%p, %p, %p, %p, %p", dev, tx_bufs, rx_bufs, cb, userdata);

	return transceive(dev, config, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_dw_release(const struct device *dev,
			  const struct spi_config *config)
{
	struct spi_dw_data *spi = dev->data;

	if (!spi_context_configured(&spi->ctx, config)) {
		return -EINVAL;
	}

	spi_context_unlock_unconditionally(&spi->ctx);

	return 0;
}

void spi_dw_isr(const struct device *dev)
{
	uint32_t int_status;
	int error;

	int_status = read_isr(dev);

	LOG_DBG("SPI %p int_status 0x%x - (tx: %d, rx: %d)", dev, int_status,
		read_txflr(dev), read_rxflr(dev));

	if (int_status & DW_SPI_ISR_ERRORS_MASK) {
		error = -EIO;
		goto out;
	}

	error = 0;

	if (int_status & DW_SPI_ISR_RXFIS) {
		pull_data(dev);
	}

	if (int_status & DW_SPI_ISR_TXEIS) {
		push_data(dev);
	}

out:
	clear_interrupts(dev);
	completed(dev, error);
}

static DEVICE_API(spi, dw_spi_api) = {
	.transceive = spi_dw_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_dw_transceive_async,
#endif /* CONFIG_SPI_ASYNC */
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_dw_release,
};

int spi_dw_init(const struct device *dev)
{
	int err;
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;

#ifdef CONFIG_PINCTRL
	pinctrl_apply_state(info->pcfg, PINCTRL_STATE_DEFAULT);
#endif

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	info->config_func();

	/* update rx sample delay */
	write_rxdly(dev, info->rx_delay);

	/* Masking interrupt and making sure controller is disabled */
	write_imr(dev, DW_SPI_IMR_MASK);
	clear_bit_ssienr(dev);

	LOG_DBG("Designware SPI driver initialized on device: %p", dev);

#ifdef CONFIG_SPI_DW_USE_DMA
	if (info->dma_rx.enabled || info->dma_tx.enabled) {
		if (!device_is_ready(info->dma_dev)) {
			LOG_ERR("SPI DMA %s not ready", info->dma_dev->name);
			return -ENODEV;
		}
	}

	k_sem_init(&spi->dma_sem, 0, 1);
#endif

	err = spi_context_cs_configure_all(&spi->ctx);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&spi->ctx);

	return 0;
}

#define SPI_DW_INST_DMA_IS_ENABLED(inst)                                       \
			UTIL_OR(DT_INST_DMAS_HAS_NAME(inst, txdma),                        \
				DT_INST_DMAS_HAS_NAME(inst, rxdma))

#define SPI_DW_DMA_INIT(inst)                                                  \
	IF_ENABLED(DT_INST_DMAS_HAS_NAME(inst, txdma),                             \
		(.dma_tx.enabled = 1,                                                  \
		 .dma_tx.ch = DT_INST_DMAS_CELL_BY_NAME(inst, txdma, channel),         \
		 .dma_tx.periph = DT_INST_DMAS_CELL_BY_NAME(inst, txdma, periph),))    \
	IF_ENABLED(DT_INST_DMAS_HAS_NAME(inst, rxdma),                             \
		(.dma_rx.enabled = 1,                                                  \
		 .dma_rx.ch = DT_INST_DMAS_CELL_BY_NAME(inst, rxdma, channel),         \
		 .dma_rx.periph = DT_INST_DMAS_CELL_BY_NAME(inst, rxdma, periph),))    \
	COND_CODE_1(DT_INST_DMAS_HAS_NAME(inst, txdma),                            \
		(.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, txdma)),),   \
		(.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, rxdma)),))


#define SPI_CFG_IRQS_SINGLE_ERR_LINE(inst)					\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rx_avail, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, rx_avail, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, tx_req, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, tx_req, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, err_int, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, err_int, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, rx_avail, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, tx_req, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, err_int, irq));

#define SPI_CFG_IRQS_MULTIPLE_ERR_LINES(inst)					\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rx_avail, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, rx_avail, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, tx_req, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, tx_req, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, txo_err, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, txo_err, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rxo_err, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, rxo_err, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rxu_err, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, rxu_err, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, mst_err, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, mst_err, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, rx_avail, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, tx_req, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, txo_err, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, rxo_err, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, rxu_err, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, mst_err, irq));

#define SPI_DW_IRQ_HANDLER(inst)                                   \
void spi_dw_irq_config_##inst(void)                                \
{                                                                  \
COND_CODE_1(IS_EQ(DT_NUM_IRQS(DT_DRV_INST(inst)), 1),              \
	(IRQ_CONNECT(DT_INST_IRQN(inst),                           \
		DT_INST_IRQ(inst, priority),                       \
		spi_dw_isr, DEVICE_DT_INST_GET(inst),              \
		0);                                                \
	irq_enable(DT_INST_IRQN(inst));),                          \
	(COND_CODE_1(IS_EQ(DT_NUM_IRQS(DT_DRV_INST(inst)), 3),     \
		(SPI_CFG_IRQS_SINGLE_ERR_LINE(inst)),		   \
		(SPI_CFG_IRQS_MULTIPLE_ERR_LINES(inst)))))	   \
}

#define SPI_DW_INIT(inst)                                                                   \
	IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(inst);))                         \
	SPI_DW_IRQ_HANDLER(inst);                                                           \
	static struct spi_dw_data spi_dw_data_##inst = {                                    \
		SPI_CONTEXT_INIT_LOCK(spi_dw_data_##inst, ctx),                             \
		SPI_CONTEXT_INIT_SYNC(spi_dw_data_##inst, ctx),                             \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), ctx)                     \
		SPI_UPDATE_DWC_SSI_FLAG(DT_DRV_INST(inst), dwc_ssi)			    \
	};                                                                                  \
	static const struct spi_dw_config spi_dw_config_##inst = {                          \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                    \
		.clock_frequency = COND_CODE_1(                                             \
			DT_NODE_HAS_PROP(DT_INST_PHANDLE(inst, clocks), clock_frequency),   \
			(DT_INST_PROP_BY_PHANDLE(inst, clocks, clock_frequency)),           \
			(DT_INST_PROP(inst, clock_frequency))),                             \
		.config_func = spi_dw_irq_config_##inst,                                    \
		.serial_target = DT_INST_PROP(inst, serial_target),                         \
		.fifo_depth = DT_INST_PROP(inst, fifo_depth),                               \
		.max_xfer_size = DT_INST_PROP(inst, max_xfer_size),                         \
		.rx_delay = DT_INST_PROP(inst, rx_delay),                                   \
		.slv_slct_toggle = DT_INST_PROP(inst, slv_slct_toggle),                     \
		IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),)) \
		COND_CODE_1(DT_INST_PROP(inst, aux_reg),                                    \
			(.read_func = aux_reg_read,                                         \
			.write_func = aux_reg_write,                                        \
			.set_bit_func = aux_reg_set_bit,                                    \
			.clear_bit_func = aux_reg_clear_bit,                                \
			.test_bit_func = aux_reg_test_bit,),                                \
			(.read_func = reg_read,                                             \
			.write_func = reg_write,                                            \
			.set_bit_func = reg_set_bit,                                        \
			.clear_bit_func = reg_clear_bit,                                    \
			.test_bit_func = reg_test_bit,))                                    \
		IF_ENABLED(CONFIG_SPI_DW_USE_DMA,					\
		    (COND_CODE_1(SPI_DW_INST_DMA_IS_ENABLED(inst),			\
		    (SPI_DW_DMA_INIT(inst)), ())))					\
	};                                                                                  \
	SPI_DEVICE_DT_INST_DEFINE(inst,                                                     \
		spi_dw_init,                                                                \
		NULL,                                                                       \
		&spi_dw_data_##inst,                                                        \
		&spi_dw_config_##inst,                                                      \
		POST_KERNEL,                                                                \
		CONFIG_SPI_INIT_PRIORITY,                                                   \
		&dw_spi_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_DW_INIT)
