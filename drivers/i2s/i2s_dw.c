/*
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_designware_i2s

#include <string.h>

#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/util.h>

#include "i2s_dw.h"

LOG_MODULE_REGISTER(i2s_dw, CONFIG_I2S_LOG_LEVEL);

/* WSS cycles lookup table indexed by enum i2s_dw_wss */
static const uint32_t wss_cycles[] = {16, 24, 32};

/* Register access helpers */
static inline uint32_t i2s_reg_read(const struct device *dev, uint32_t reg)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + reg);
}

static inline void i2s_reg_write(const struct device *dev, uint32_t reg,
				 uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_GET(dev) + reg);
}

static inline void i2s_reg_update(const struct device *dev, uint32_t reg,
				  uint32_t mask, uint32_t val)
{
	uint32_t tmp = i2s_reg_read(dev, reg);

	tmp = (tmp & ~mask) | (val & mask);
	i2s_reg_write(dev, reg, tmp);
}

/* Convert word_size (bits) to WLEN register value */
static inline enum i2s_dw_wlen i2s_dw_word_size_to_wlen(uint32_t word_size)
{
	switch (word_size) {
	case 12: return I2S_DW_WLEN_12BIT;
	case 16: return I2S_DW_WLEN_16BIT;
	case 20: return I2S_DW_WLEN_20BIT;
	case 24: return I2S_DW_WLEN_24BIT;
	case 32: return I2S_DW_WLEN_32BIT;
	default: return I2S_DW_WLEN_IGNORE;
	}
}

/* Message queue helpers (replaces custom ring buffer) */
static int queue_get(struct k_msgq *msgq, void **mem_block, size_t *size)
{
	struct i2s_dw_queue_item item;
	int ret;

	ret = k_msgq_get(msgq, &item, K_NO_WAIT);
	if (ret == 0) {
		*mem_block = item.mem_block;
		*size = item.size;
	}
	return ret;
}

static int queue_put(struct k_msgq *msgq, void *mem_block, size_t size)
{
	struct i2s_dw_queue_item item = {
		.mem_block = mem_block,
		.size = size,
	};

	return k_msgq_put(msgq, &item, K_NO_WAIT);
}

/* Clock configuration */
static int i2s_dw_set_clock_rate(const struct i2s_dw_cfg *cfg,
				 uint32_t sample_rate)
{
	uint32_t sclk;

	if (!sample_rate) {
		return -EINVAL;
	}

	/* sclk = 2 * WSS_cycles * sample_rate */
	sclk = 2 * wss_cycles[cfg->wss_len] * sample_rate;

	return clock_control_set_rate(cfg->clk_dev, cfg->clkid,
				      (clock_control_subsys_rate_t)sclk);
}

static void i2s_dw_enable_controller(const struct device *dev)
{
	const struct i2s_dw_cfg *cfg = dev->config;

	/* Global enable */
	i2s_reg_write(dev, I2S_DW_IER, I2S_DW_IER_IEN);

	/* Configure WSS and SCLKG */
	i2s_reg_write(dev, I2S_DW_CCR,
		      (cfg->sclkg & I2S_DW_CCR_SCLKG_MASK) |
		      ((cfg->wss_len << I2S_DW_CCR_WSS_SHIFT) &
		       I2S_DW_CCR_WSS_MASK));

	/* Enable clock */
	i2s_reg_write(dev, I2S_DW_CER, I2S_DW_CER_CLKEN);
}

/* Interrupt mask helpers */
static void i2s_dw_disable_tx_irq(const struct device *dev)
{
	i2s_reg_update(dev, I2S_DW_IMR,
		       I2S_DW_IMR_TXFEM | I2S_DW_IMR_TXFOM,
		       I2S_DW_IMR_TXFEM | I2S_DW_IMR_TXFOM);
}

static void i2s_dw_enable_tx_irq(const struct device *dev)
{
	i2s_reg_update(dev, I2S_DW_IMR,
		       I2S_DW_IMR_TXFEM | I2S_DW_IMR_TXFOM, 0);
}

static void i2s_dw_disable_rx_irq(const struct device *dev)
{
	i2s_reg_update(dev, I2S_DW_IMR,
		       I2S_DW_IMR_RXDAM | I2S_DW_IMR_RXFOM,
		       I2S_DW_IMR_RXDAM | I2S_DW_IMR_RXFOM);
}

static void i2s_dw_enable_rx_irq(const struct device *dev)
{
	i2s_reg_update(dev, I2S_DW_IMR,
		       I2S_DW_IMR_RXDAM | I2S_DW_IMR_RXFOM, 0);
}

static void i2s_dw_disable_rx_fo_irq(const struct device *dev)
{
	i2s_reg_update(dev, I2S_DW_IMR, I2S_DW_IMR_RXFOM, I2S_DW_IMR_RXFOM);
}

/* Stream start / disable / queue drop */
static void rx_stream_disable(const struct device *dev,
			      struct i2s_dw_stream *stream)
{
	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}

	i2s_reg_write(dev, I2S_DW_RER, 0);
	i2s_reg_write(dev, I2S_DW_IRER, 0);
	i2s_dw_disable_rx_irq(dev);
	i2s_reg_write(dev, I2S_DW_CER, 0);
}

static void tx_stream_disable(const struct device *dev,
			      struct i2s_dw_stream *stream)
{
	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}

	i2s_reg_write(dev, I2S_DW_TER, 0);
	i2s_reg_write(dev, I2S_DW_ITER, 0);
	i2s_dw_disable_tx_irq(dev);
	i2s_reg_write(dev, I2S_DW_CER, 0);
}

static void rx_queue_drop(const struct device *dev,
			  struct i2s_dw_stream *stream)
{
	struct i2s_dw_queue_item item;

	while (k_msgq_get(&stream->msgq, &item, K_NO_WAIT) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, item.mem_block);
	}
	k_sem_reset(&stream->sem);
}

static void tx_queue_drop(const struct device *dev,
			  struct i2s_dw_stream *stream)
{
	struct i2s_dw_queue_item item;
	unsigned int n = 0U;

	while (k_msgq_get(&stream->msgq, &item, K_NO_WAIT) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, item.mem_block);
		n++;
	}
	for (; n > 0; n--) {
		k_sem_give(&stream->sem);
	}
}

static int rx_stream_start(const struct device *dev,
			   struct i2s_dw_stream *stream)
{
	const struct i2s_dw_cfg *cfg = dev->config;
	int ret;

	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
			       K_NO_WAIT);
	if (ret < 0) {
		return ret;
	}
	stream->mem_block_offset = 0;

	/* Set clock rate */
	ret = i2s_dw_set_clock_rate(cfg, stream->cfg.frame_clk_freq);
	if (ret) {
		LOG_ERR("Failed to set clock rate: %d", ret);
		return ret;
	}

	/* Reset Rx FIFO */
	i2s_reg_write(dev, I2S_DW_RFF, I2S_DW_FIFO_RST);

	/* Set word length */
	i2s_reg_write(dev, I2S_DW_RCR,
		      i2s_dw_word_size_to_wlen(stream->cfg.word_size));

	/* Configure clock */
	i2s_dw_enable_controller(dev);

	/* Disable Tx channel */
	i2s_reg_write(dev, I2S_DW_TER, 0);

	/* Clear overrun */
	(void)i2s_reg_read(dev, I2S_DW_ROR);

	/* Enable Rx channel, interrupts, and block */
	i2s_reg_write(dev, I2S_DW_RER, I2S_DW_RER_RXCHEN);
	i2s_dw_enable_rx_irq(dev);
	i2s_reg_write(dev, I2S_DW_IRER, I2S_DW_IRER_RXEN);

	return 0;
}

static int tx_stream_start(const struct device *dev,
			   struct i2s_dw_stream *stream)
{
	const struct i2s_dw_cfg *cfg = dev->config;
	int ret;

	ret = queue_get(&stream->msgq, &stream->mem_block,
			&stream->mem_block_size);
	if (ret < 0) {
		return ret;
	}
	k_sem_give(&stream->sem);
	stream->mem_block_offset = 0;

	/* Set clock rate */
	ret = i2s_dw_set_clock_rate(cfg, stream->cfg.frame_clk_freq);
	if (ret) {
		LOG_ERR("Failed to set clock rate: %d", ret);
		return ret;
	}

	/* Reset Tx FIFO */
	i2s_reg_write(dev, I2S_DW_TFF, I2S_DW_FIFO_RST);

	/* Set word length */
	i2s_reg_write(dev, I2S_DW_TCR,
		      i2s_dw_word_size_to_wlen(stream->cfg.word_size));

	/* Configure clock */
	i2s_dw_enable_controller(dev);

	/* Disable Rx channel */
	i2s_reg_write(dev, I2S_DW_RER, 0);

	/* Clear overrun */
	(void)i2s_reg_read(dev, I2S_DW_TOR);

	/* Enable Tx channel, interrupts, and block */
	i2s_reg_write(dev, I2S_DW_TER, I2S_DW_TER_TXCHEN);
	i2s_dw_enable_tx_irq(dev);
	i2s_reg_write(dev, I2S_DW_ITER, I2S_DW_ITER_TXEN);

	return 0;
}

/* IRQ handlers */
static void i2s_dw_tx_isr(const struct device *dev)
{
	const struct i2s_dw_cfg *cfg = dev->config;
	struct i2s_dw_data *data = dev->data;
	struct i2s_dw_stream *stream = &data->tx;
	const uint32_t num_ch = (stream->cfg.format & I2S_FMT_DATA_FORMAT_MASK)
				? 2U : stream->cfg.channels;
	uint32_t tx_avail = I2S_DW_FIFO_DEPTH - cfg->tx_fifo_trg_lvl;
	const uint8_t *buf = stream->mem_block;
	uint32_t offset = stream->mem_block_offset;
	uint32_t size = stream->mem_block_size;
	uint8_t bytes, frames, cnt;
	bool last_lap = false;
	int ret;

	if (stream->state == I2S_STATE_ERROR) {
		goto tx_disable;
	}

	if (stream->last_block) {
		stream->state = I2S_STATE_READY;
		pm_device_busy_clear(dev);
		goto tx_disable;
	}

	bytes = (stream->cfg.word_size <= 16) ? I2S_DW_16BIT_SAMPLE_BYTES
					      : I2S_DW_32BIT_SAMPLE_BYTES;

	if ((offset + (2 * tx_avail * bytes)) > size) {
		frames = (size - offset) / (2 * bytes);
		last_lap = true;
	} else {
		frames = tx_avail;
	}

	for (cnt = 0; cnt < frames; cnt++) {
		if (bytes == I2S_DW_16BIT_SAMPLE_BYTES) {
			i2s_reg_write(dev, I2S_DW_LRBR,
				      (uint32_t)(*(uint16_t *)(buf + offset)));
			if (num_ch == 1) {
				i2s_reg_write(dev, I2S_DW_RRBR, 0);
				offset += I2S_DW_16BIT_SAMPLE_BYTES;
			} else {
				i2s_reg_write(dev, I2S_DW_RRBR,
					(uint32_t)(*(uint16_t *)(buf + offset +
					I2S_DW_16BIT_SAMPLE_BYTES)));
				offset += 2 * I2S_DW_16BIT_SAMPLE_BYTES;
			}
		} else {
			i2s_reg_write(dev, I2S_DW_LRBR,
				      *(uint32_t *)(buf + offset));
			if (num_ch == 1) {
				i2s_reg_write(dev, I2S_DW_RRBR, 0);
				offset += I2S_DW_32BIT_SAMPLE_BYTES;
			} else {
				i2s_reg_write(dev, I2S_DW_RRBR,
					*(uint32_t *)(buf + offset +
					I2S_DW_32BIT_SAMPLE_BYTES));
				offset += 2 * I2S_DW_32BIT_SAMPLE_BYTES;
			}
		}
	}

	if (last_lap && (offset < size)) {
		if (bytes == I2S_DW_16BIT_SAMPLE_BYTES) {
			i2s_reg_write(dev, I2S_DW_LRBR,
				      (uint32_t)(*(uint16_t *)(buf + offset)));
			offset += I2S_DW_16BIT_SAMPLE_BYTES;
		} else {
			i2s_reg_write(dev, I2S_DW_LRBR,
				      *(uint32_t *)(buf + offset));
			offset += I2S_DW_32BIT_SAMPLE_BYTES;
		}
		i2s_reg_write(dev, I2S_DW_RRBR, 0);
	}

	stream->mem_block_offset = offset;

	if (offset >= size) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
		stream->mem_block_offset = 0;

		ret = queue_get(&stream->msgq, &stream->mem_block,
				&stream->mem_block_size);
		if (ret < 0) {
			if (stream->state == I2S_STATE_STOPPING) {
				stream->state = I2S_STATE_READY;
				pm_device_busy_clear(dev);
			} else {
				stream->state = I2S_STATE_ERROR;
			}
			goto tx_disable;
		}
		k_sem_give(&stream->sem);
	}
	return;

tx_disable:
	tx_stream_disable(dev, stream);
}

static void i2s_dw_rx_isr(const struct device *dev)
{
	const struct i2s_dw_cfg *cfg = dev->config;
	struct i2s_dw_data *data = dev->data;
	struct i2s_dw_stream *stream = &data->rx;
	const uint32_t num_ch = (stream->cfg.format & I2S_FMT_DATA_FORMAT_MASK)
				? 2U : stream->cfg.channels;
	uint32_t rx_avail = cfg->rx_fifo_trg_lvl;
	uint8_t *buf = stream->mem_block;
	uint32_t offset = stream->mem_block_offset;
	uint32_t size = stream->cfg.block_size;
	uint8_t bytes, frames, cnt;
	bool last_lap = false;
	void *mblk_tmp;
	int ret;

	if (stream->state == I2S_STATE_ERROR) {
		goto rx_disable;
	}

	if (stream->state == I2S_STATE_STOPPING) {
		stream->state = I2S_STATE_READY;
		pm_device_busy_clear(dev);
		goto rx_disable;
	}

	bytes = (stream->cfg.word_size <= 16) ? I2S_DW_16BIT_SAMPLE_BYTES
					      : I2S_DW_32BIT_SAMPLE_BYTES;

	if ((offset + (2 * rx_avail * bytes)) > size) {
		frames = (size - offset) / (2 * bytes);
		last_lap = true;
	} else {
		frames = rx_avail;
	}

	for (cnt = 0; cnt < frames; cnt++) {
		if (bytes == I2S_DW_16BIT_SAMPLE_BYTES) {
			*(uint16_t *)(buf + offset) =
				(uint16_t)i2s_reg_read(dev, I2S_DW_LRBR);
			if (num_ch == 1) {
				(void)i2s_reg_read(dev, I2S_DW_RRBR);
				offset += I2S_DW_16BIT_SAMPLE_BYTES;
			} else {
				*(uint16_t *)(buf + offset +
					I2S_DW_16BIT_SAMPLE_BYTES) =
					(uint16_t)i2s_reg_read(dev,
							       I2S_DW_RRBR);
				offset += 2 * I2S_DW_16BIT_SAMPLE_BYTES;
			}
		} else {
			*(uint32_t *)(buf + offset) =
				i2s_reg_read(dev, I2S_DW_LRBR);
			if (num_ch == 1) {
				(void)i2s_reg_read(dev, I2S_DW_RRBR);
				offset += I2S_DW_32BIT_SAMPLE_BYTES;
			} else {
				*(uint32_t *)(buf + offset +
					I2S_DW_32BIT_SAMPLE_BYTES) =
					i2s_reg_read(dev, I2S_DW_RRBR);
				offset += 2 * I2S_DW_32BIT_SAMPLE_BYTES;
			}
		}
	}

	if (last_lap && (offset < size)) {
		if (bytes == I2S_DW_16BIT_SAMPLE_BYTES) {
			*(uint16_t *)(buf + offset) =
				(uint16_t)i2s_reg_read(dev, I2S_DW_LRBR);
			offset += I2S_DW_16BIT_SAMPLE_BYTES;
		} else {
			*(uint32_t *)(buf + offset) =
				i2s_reg_read(dev, I2S_DW_LRBR);
			offset += I2S_DW_32BIT_SAMPLE_BYTES;
		}
		(void)i2s_reg_read(dev, I2S_DW_RRBR);
	}

	stream->mem_block_offset = offset;

	if (offset >= size) {
		mblk_tmp = stream->mem_block;

		ret = k_mem_slab_alloc(stream->cfg.mem_slab,
				       &stream->mem_block, K_NO_WAIT);
		if (ret < 0) {
			stream->state = I2S_STATE_ERROR;
			goto rx_disable;
		}
		stream->mem_block_offset = 0;

		ret = queue_put(&stream->msgq, mblk_tmp,
				stream->cfg.block_size);
		if (ret < 0) {
			stream->state = I2S_STATE_ERROR;
			goto rx_disable;
		}
		k_sem_give(&stream->sem);
	}
	return;

rx_disable:
	rx_stream_disable(dev, stream);
}

static void i2s_dw_isr(const struct device *dev)
{
	struct i2s_dw_data *data = dev->data;
	uint32_t isr = i2s_reg_read(dev, I2S_DW_ISR);

	if ((data->dir == I2S_DIR_TX) && (isr & I2S_DW_ISR_TXFE)) {
		i2s_dw_tx_isr(dev);
	}

	if ((data->dir == I2S_DIR_RX) && (isr & I2S_DW_ISR_RXDA)) {
		i2s_dw_rx_isr(dev);
	}

	if (isr & I2S_DW_ISR_TXFO) {
		(void)i2s_reg_read(dev, I2S_DW_TOR);
	}

	if (isr & I2S_DW_ISR_RXFO) {
		(void)i2s_reg_read(dev, I2S_DW_ROR);
		i2s_dw_disable_rx_fo_irq(dev);
	}
}

/* Zephyr I2S API */
static int i2s_dw_configure(const struct device *dev, enum i2s_dir dir,
			    const struct i2s_config *i2s_cfg)
{
	const struct i2s_dw_cfg *cfg = dev->config;
	struct i2s_dw_data *data = dev->data;
	struct i2s_dw_stream *stream;

	data->dir = dir;

	switch (dir) {
	case I2S_DIR_RX:
		stream = &data->rx;
		break;
	case I2S_DIR_TX:
		stream = &data->tx;
		break;
	case I2S_DIR_BOTH:
		return -ENOSYS;
	default:
		LOG_ERR("Either RX or TX direction must be selected");
		return -EINVAL;
	}

	if (stream->state != I2S_STATE_NOT_READY &&
	    stream->state != I2S_STATE_READY) {
		LOG_ERR("invalid state");
		return -EINVAL;
	}

	stream->master = true;
	if (i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE ||
	    i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) {
		stream->master = false;
	}

	if (i2s_cfg->frame_clk_freq == 0U) {
		if (dir == I2S_DIR_TX) {
			tx_queue_drop(dev, stream);
		} else {
			rx_queue_drop(dev, stream);
		}
		memset(&stream->cfg, 0, sizeof(struct i2s_config));
		stream->state = I2S_STATE_NOT_READY;
		return 0;
	}

	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));

	/* Set FIFO trigger level */
	if (dir == I2S_DIR_TX) {
		i2s_reg_write(dev, I2S_DW_TFCR,
			      cfg->tx_fifo_trg_lvl & I2S_DW_FIFO_TRG_MASK);
	} else {
		i2s_reg_write(dev, I2S_DW_RFCR,
			      cfg->rx_fifo_trg_lvl & I2S_DW_FIFO_TRG_MASK);
	}

	stream->state = I2S_STATE_READY;
	return 0;
}

static int i2s_dw_trigger(const struct device *dev, enum i2s_dir dir,
			  enum i2s_trigger_cmd cmd)
{
	struct i2s_dw_data *data = dev->data;
	struct i2s_dw_stream *stream;
	unsigned int key;
	int ret;

	switch (dir) {
	case I2S_DIR_RX:
		stream = &data->rx;
		break;
	case I2S_DIR_TX:
		stream = &data->tx;
		break;
	case I2S_DIR_BOTH:
		return -ENOSYS;
	default:
		LOG_ERR("Either RX or TX direction must be selected");
		return -EINVAL;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %d",
				stream->state);
			return -EIO;
		}
		__ASSERT_NO_MSG(stream->mem_block == NULL);

		if (dir == I2S_DIR_TX) {
			ret = tx_stream_start(dev, stream);
		} else {
			ret = rx_stream_start(dev, stream);
		}
		if (ret < 0) {
			LOG_ERR("START trigger failed %d", ret);
			return ret;
		}
		pm_device_busy_set(dev);
		stream->state = I2S_STATE_RUNNING;
		stream->last_block = false;
		break;

	case I2S_TRIGGER_STOP:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("STOP trigger: invalid state");
			return -EIO;
		}
		stream->state = I2S_STATE_STOPPING;
		stream->last_block = true;
		irq_unlock(key);
		break;

	case I2S_TRIGGER_DRAIN:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("DRAIN trigger: invalid state");
			return -EIO;
		}
		stream->state = I2S_STATE_STOPPING;
		if (dir == I2S_DIR_RX) {
			stream->last_block = true;
		}
		irq_unlock(key);
		break;

	case I2S_TRIGGER_DROP:
		key = irq_lock();
		if (stream->state == I2S_STATE_NOT_READY) {
			irq_unlock(key);
			LOG_ERR("DROP trigger: invalid state");
			return -EIO;
		}
		if (dir == I2S_DIR_TX) {
			tx_stream_disable(dev, stream);
			tx_queue_drop(dev, stream);
		} else {
			rx_stream_disable(dev, stream);
			rx_queue_drop(dev, stream);
		}
		stream->state = I2S_STATE_READY;
		irq_unlock(key);
		pm_device_busy_clear(dev);
		break;

	case I2S_TRIGGER_PREPARE:
		if (stream->state != I2S_STATE_ERROR) {
			LOG_ERR("PREPARE trigger: invalid state");
			return -EIO;
		}
		stream->state = I2S_STATE_READY;
		if (dir == I2S_DIR_TX) {
			tx_queue_drop(dev, stream);
		} else {
			rx_queue_drop(dev, stream);
		}
		break;

	default:
		LOG_ERR("Unsupported trigger command");
		return -EINVAL;
	}

	return 0;
}

static int i2s_dw_read(const struct device *dev, void **mem_block,
		       size_t *size)
{
	struct i2s_dw_data *data = dev->data;
	struct i2s_dw_stream *stream = &data->rx;
	int ret;

	if (stream->state == I2S_STATE_NOT_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	if (stream->state != I2S_STATE_ERROR) {
		ret = k_sem_take(&stream->sem,
				 SYS_TIMEOUT_MS(stream->cfg.timeout));
		if (ret < 0) {
			return ret;
		}
	}

	return queue_get(&stream->msgq, mem_block, size);
}

static int i2s_dw_write(const struct device *dev, void *mem_block,
			size_t size)
{
	struct i2s_dw_data *data = dev->data;
	struct i2s_dw_stream *stream = &data->tx;
	int ret;

	if (stream->state != I2S_STATE_RUNNING &&
	    stream->state != I2S_STATE_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	ret = k_sem_take(&stream->sem,
			 SYS_TIMEOUT_MS(stream->cfg.timeout));
	if (ret < 0) {
		return ret;
	}

	return queue_put(&stream->msgq, mem_block, size);
}

static DEVICE_API(i2s, i2s_dw_driver_api) = {
	.configure = i2s_dw_configure,
	.read = i2s_dw_read,
	.write = i2s_dw_write,
	.trigger = i2s_dw_trigger,
};

/* Initialization */
static int i2s_dw_init(const struct device *dev)
{
	const struct i2s_dw_cfg *cfg = dev->config;
	struct i2s_dw_data *data = dev->data;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

#if defined(CONFIG_PINCTRL)
	if (cfg->pincfg != NULL) {
		ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("I2S pinctrl setup failed (%d)", ret);
			return ret;
		}
	}
#endif

	if (!device_is_ready(cfg->clk_dev)) {
		LOG_ERR("clock controller device not ready");
		return -ENODEV;
	}

	ret = clock_control_configure(cfg->clk_dev, cfg->clkid, NULL);
	if (ret != 0) {
		LOG_ERR("Unable to configure clock: err:%d", ret);
		return ret;
	}

	ret = clock_control_on(cfg->clk_dev, cfg->clkid);
	if (ret != 0) {
		LOG_ERR("Unable to turn on clock: err:%d", ret);
		return ret;
	}

	i2s_dw_enable_controller(dev);

	cfg->irq_config(dev);

	k_sem_init(&data->rx.sem, 0, CONFIG_I2S_DW_RX_BLOCK_COUNT);
	k_sem_init(&data->tx.sem, CONFIG_I2S_DW_TX_BLOCK_COUNT,
		   CONFIG_I2S_DW_TX_BLOCK_COUNT);

	/* Mask all interrupts */
	i2s_dw_disable_tx_irq(dev);
	i2s_dw_disable_rx_irq(dev);

	LOG_DBG("%s initialized", dev->name);

	return 0;
}

/* Power Management */
#if defined(CONFIG_PM_DEVICE)
static void i2s_dw_disable_controller(const struct device *dev)
{
	i2s_reg_write(dev, I2S_DW_CER, 0);
	i2s_reg_write(dev, I2S_DW_IER, 0);
}

static int i2s_dw_pm_action(const struct device *dev,
			    enum pm_device_action action)
{
	const struct i2s_dw_cfg *cfg = dev->config;
	struct i2s_dw_data *data = dev->data;
	int ret;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		data->imr_cache = i2s_reg_read(dev, I2S_DW_IMR);
		i2s_dw_disable_tx_irq(dev);
		i2s_dw_disable_rx_irq(dev);
		i2s_dw_disable_controller(dev);

		ret = clock_control_off(cfg->clk_dev, cfg->clkid);
		if (ret != 0 && ret != -EALREADY) {
			LOG_ERR("Unable to turn off clock: err:%d", ret);
			return ret;
		}

#if defined(CONFIG_PINCTRL)
		if (cfg->pincfg != NULL) {
			ret = pinctrl_apply_state(cfg->pincfg,
						  PINCTRL_STATE_SLEEP);
			if (ret < 0 && ret != -ENOENT) {
				return ret;
			}
		}
#endif
		return 0;

	case PM_DEVICE_ACTION_RESUME:
#if defined(CONFIG_PINCTRL)
		if (cfg->pincfg != NULL) {
			ret = pinctrl_apply_state(cfg->pincfg,
						  PINCTRL_STATE_DEFAULT);
			if (ret < 0) {
				LOG_ERR("pinctrl setup failed (%d)", ret);
				return ret;
			}
		}
#endif
		ret = clock_control_configure(cfg->clk_dev, cfg->clkid, NULL);
		if (ret != 0 && ret != -ENOSYS && ret != -ENOTSUP) {
			LOG_ERR("Unable to configure clock: err:%d", ret);
			return ret;
		}

		ret = clock_control_on(cfg->clk_dev, cfg->clkid);
		if (ret != 0 && ret != -EALREADY) {
			LOG_ERR("Unable to turn on clock: err:%d", ret);
			return ret;
		}

		i2s_dw_enable_controller(dev);
		i2s_reg_write(dev, I2S_DW_IMR, data->imr_cache);
		return 0;

	case PM_DEVICE_ACTION_TURN_OFF:
	case PM_DEVICE_ACTION_TURN_ON:
		return 0;

	default:
		return -ENOTSUP;
	}
}
#endif /* CONFIG_PM_DEVICE */

/* Device instantiation */
#define I2S_DW_INIT(index)						\
									\
static void i2s_dw_irq_config_##index(const struct device *dev);	\
									\
IF_ENABLED(DT_INST_NODE_HAS_PROP(index, pinctrl_0),			\
	(PINCTRL_DT_INST_DEFINE(index)));				\
									\
static struct i2s_dw_queue_item						\
	i2s_dw_rx_queue_##index[CONFIG_I2S_DW_RX_BLOCK_COUNT];		\
static struct i2s_dw_queue_item						\
	i2s_dw_tx_queue_##index[CONFIG_I2S_DW_TX_BLOCK_COUNT];		\
									\
static const struct i2s_dw_cfg i2s_dw_cfg_##index = {			\
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(index)),			\
	.wss_len = I2S_DW_WSS_32_CYCLES,				\
	.sclkg = I2S_DW_SCLKG_NONE,					\
	.tx_fifo_trg_lvl = DT_INST_PROP(index, tx_fifo_watermark),	\
	.rx_fifo_trg_lvl = DT_INST_PROP(index, rx_fifo_watermark),	\
	.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(index)),		\
	.clkid = (clock_control_subsys_t)				\
		DT_INST_CLOCKS_CELL_BY_IDX(index, 0, clkid),		\
	.irq_config = i2s_dw_irq_config_##index,			\
	IF_ENABLED(DT_INST_NODE_HAS_PROP(index, pinctrl_0),		\
		(.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),))	\
};									\
									\
static struct i2s_dw_data i2s_dw_data_##index;				\
									\
PM_DEVICE_DT_INST_DEFINE(index, i2s_dw_pm_action);			\
									\
static int i2s_dw_inst_init_##index(const struct device *dev)		\
{									\
	struct i2s_dw_data *data = dev->data;				\
									\
	k_msgq_init(&data->rx.msgq,					\
		    (char *)i2s_dw_rx_queue_##index,			\
		    sizeof(struct i2s_dw_queue_item),			\
		    CONFIG_I2S_DW_RX_BLOCK_COUNT);			\
	k_msgq_init(&data->tx.msgq,					\
		    (char *)i2s_dw_tx_queue_##index,			\
		    sizeof(struct i2s_dw_queue_item),			\
		    CONFIG_I2S_DW_TX_BLOCK_COUNT);			\
									\
	return i2s_dw_init(dev);					\
}									\
									\
DEVICE_DT_INST_DEFINE(index,						\
		      i2s_dw_inst_init_##index,				\
		      PM_DEVICE_DT_INST_GET(index),			\
		      &i2s_dw_data_##index,				\
		      &i2s_dw_cfg_##index,				\
		      POST_KERNEL,					\
		      CONFIG_I2S_INIT_PRIORITY,				\
		      &i2s_dw_driver_api);				\
									\
static void i2s_dw_irq_config_##index(const struct device *dev)		\
{									\
	IRQ_CONNECT(DT_INST_IRQN(index),				\
		    DT_INST_IRQ(index, priority),			\
		    i2s_dw_isr,						\
		    DEVICE_DT_INST_GET(index), 0);			\
	irq_enable(DT_INST_IRQN(index));				\
}

DT_INST_FOREACH_STATUS_OKAY(I2S_DW_INIT)
