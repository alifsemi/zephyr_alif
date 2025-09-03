/*
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT snps_designware_i2s

#include <string.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
#include <soc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include "alif_i2s_clk_config.h"
#include "i2s_dw.h"

#define WSS_LEN			2
#define EXT_CLK_SRC_ENABLE	0
#define TX_FIFO_TRG_LVL		8
#define RX_FIFO_TRG_LVL		8

LOG_MODULE_REGISTER(i2s_dw);

#define DMA_NUM_CHANNELS	8

struct queue_item {
	void *mem_block;
	size_t size;
};

/* Minimal ring buffer implementation */
struct dw_ring_buf {
	struct queue_item *buf;
	uint16_t len;
	uint16_t head;
	uint16_t tail;
};

struct stream {
	int32_t   state;
	struct    k_sem sem;
	uint32_t  dma_channel;
	struct dma_config dma_cfg;
	uint8_t   priority;
	bool      src_addr_increment;
	bool      dst_addr_increment;
	uint8_t   fifo_threshold;
	struct    i2s_config cfg;
	struct dw_ring_buf mem_block_queue;
	void      *mem_block;
	uint32_t  mem_block_size;
	uint32_t  mem_block_offset;
	bool      last_block;
	bool      master;
	int (*stream_start)(struct stream *strm, const struct device *dev);
	void (*stream_disable)(struct stream *strm, const struct device *dev);
	void (*queue_drop)(struct stream *strm);
};

/* Device run time data */
struct i2s_dw_data {
	enum i2s_dir dir;
	struct stream rx;
	struct stream tx;
};

#define MODULO_INC(val, max) { val = (++val < max) ? val : 0; }

/*!< I2S Input Clock Source */
#define I2S_CLK_SOURCE_76P8M_IN_HZ	76800000.0

/*!< Clock divisor max/min value  */
#define I2S_CLK_DIVISOR_MAX		0x3FF
#define I2S_CLK_DIVISOR_MIN		2

static int32_t i2s_configure_clocksource(bool enable,
					const struct i2s_dw_cfg *i2s,
					uint32_t sample_rate)
{
	uint32_t div = 0;
	uint32_t sclk = 0;
	const uint32_t clock_cycles[WSS_CLOCK_CYCLES_MAX] = {16, 24, 32};

	if (enable) {
		if (!sample_rate) {
			return -1;
		}

		/* Calculate sclk = 2* WSS * Sample Rate*/
		/* WSS = 32 */
		sclk = 2 * clock_cycles[i2s->cfg.wss_len] * (sample_rate);

		div = i2s->cfg.clk_source/sclk;
		if ((div > I2S_CLK_DIVISOR_MAX) || (div < I2S_CLK_DIVISOR_MIN)) {
			return -1;
		}

		set_i2s_clock_divisor(i2s->instance, div);
	}

	return 0;
}


/*
 * Get data from the queue
 */
static int queue_get(struct dw_ring_buf *rb, void **mem_block, size_t *size)
{
	unsigned int key;

	key = irq_lock();

	if (rb->tail == rb->head) {
		/* Ring buffer is empty */
		irq_unlock(key);
		return -ENOMEM;
	}

	*mem_block = rb->buf[rb->tail].mem_block;
	*size = rb->buf[rb->tail].size;
	MODULO_INC(rb->tail, rb->len);

	irq_unlock(key);

	return 0;
}

/*
 * Put data in the queue
 */
static int queue_put(struct dw_ring_buf *rb, void *mem_block, size_t size)
{
	uint16_t head_next;
	unsigned int key;

	key = irq_lock();

	head_next = rb->head;
	MODULO_INC(head_next, rb->len);

	if (head_next == rb->tail) {
		/* Ring buffer is full */
		irq_unlock(key);
		return -ENOMEM;
	}

	rb->buf[rb->head].mem_block = mem_block;
	rb->buf[rb->head].size = size;
	rb->head = head_next;

	irq_unlock(key);

	return 0;
}

static void i2s_enable_clock(const struct device *dev)
{
	const struct i2s_dw_cfg *i2s = dev->config;

	select_i2s_clock_source(i2s->instance, i2s->cfg.ext_clk_src_enable);

	/* Enable the I2S module clock */
	enable_i2s_sclk_aon(i2s->instance);
	enable_i2s_clock(i2s->instance);

	/* Enable I2S */
	i2s_global_enable(i2s);

	/* Enable Master Clock */
	i2s_configure_clock(i2s);
	i2s_clock_enable(i2s);
}

static int i2s_dw_configure(const struct device *dev, enum i2s_dir dir,
			       const struct i2s_config *i2s_cfg)
{
	const struct i2s_dw_cfg *i2s = dev->config;
	struct i2s_dw_data *const dev_data = dev->data;

	struct stream *stream;

	dev_data->dir = dir;

	switch (dir) {
	case I2S_DIR_RX:
		stream = &dev_data->rx;
		break;
	case I2S_DIR_TX:
		stream = &dev_data->tx;
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
		stream->queue_drop(stream);
		memset(&stream->cfg, 0, sizeof(struct i2s_config));
		stream->state = I2S_STATE_NOT_READY;
		return 0;
	}

	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));

	/* Set FIFO Trigger Level */
	if (dir == I2S_DIR_TX) {
		i2s_set_tx_trigger_level(i2s);
	} else if (dir == I2S_DIR_RX) {
		i2s_set_rx_trigger_level(i2s);
	}

	stream->state = I2S_STATE_READY;
	return 0;
}

static int i2s_dw_trigger(const struct device *dev, enum i2s_dir dir,
			     enum i2s_trigger_cmd cmd)
{
	struct i2s_dw_data *const dev_data = dev->data;
	struct stream *stream;
	unsigned int key;
	int ret;

	switch (dir) {
	case I2S_DIR_RX:
		stream = &dev_data->rx;
		break;
	case I2S_DIR_TX:
		stream = &dev_data->tx;
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

		ret = stream->stream_start(stream, dev);
		if (ret < 0) {
			LOG_ERR("START trigger failed %d", ret);
			return ret;
		}

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
		irq_unlock(key);
		stream->stream_disable(stream, dev);
		stream->queue_drop(stream);
		stream->state = I2S_STATE_READY;
		stream->last_block = true;
		break;

	case I2S_TRIGGER_DRAIN:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("DRAIN trigger: invalid state");
			return -EIO;
		}
		stream->stream_disable(stream, dev);
		stream->queue_drop(stream);
		stream->state = I2S_STATE_READY;
		irq_unlock(key);
		break;

	case I2S_TRIGGER_DROP:
		if (stream->state == I2S_STATE_NOT_READY) {
			LOG_ERR("DROP trigger: invalid state");
			return -EIO;
		}
		stream->stream_disable(stream, dev);
		stream->queue_drop(stream);
		stream->state = I2S_STATE_READY;
		break;

	case I2S_TRIGGER_PREPARE:
		if (stream->state != I2S_STATE_ERROR) {
			LOG_ERR("PREPARE trigger: invalid state");
			return -EIO;
		}
		stream->state = I2S_STATE_READY;
		stream->queue_drop(stream);
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
	struct i2s_dw_data *const dev_data = dev->data;
	int ret;

	if (dev_data->rx.state == I2S_STATE_NOT_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}
	if (dev_data->rx.state != I2S_STATE_ERROR) {
		ret = k_sem_take(&dev_data->rx.sem,
				 SYS_TIMEOUT_MS(dev_data->rx.cfg.timeout));
		if (ret < 0) {
			return ret;
		}
	}

	/* Get data from the beginning of RX queue */
	ret = queue_get(&dev_data->rx.mem_block_queue, mem_block, size);
	if (ret < 0) {
		return -EIO;
	}

	return 0;
}

static int i2s_dw_write(const struct device *dev, void *mem_block,
			   size_t size)
{
	struct i2s_dw_data *const dev_data = dev->data;
	int ret;

	if (dev_data->tx.state != I2S_STATE_RUNNING &&
	    dev_data->tx.state != I2S_STATE_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	ret = k_sem_take(&dev_data->tx.sem,
			 SYS_TIMEOUT_MS(dev_data->tx.cfg.timeout));
	if (ret < 0) {
		return ret;
	}

	/* Add data to the end of the TX queue */
	queue_put(&dev_data->tx.mem_block_queue, mem_block, size);

	return 0;
}

static const struct i2s_driver_api i2s_dw_driver_api = {
	.configure = i2s_dw_configure,
	.read = i2s_dw_read,
	.write = i2s_dw_write,
	.trigger = i2s_dw_trigger,
};

static void tx_stream_disable(struct stream *stream, const struct device *dev);
static void rx_stream_disable(struct stream *stream, const struct device *dev);

static void i2s_tx_irq_handler(const struct device *dev)
{
	const struct i2s_dw_cfg *i2s = dev->config;
	struct i2s_dw_data *const dev_data = dev->data;
	struct stream *stream = &dev_data->tx;
	const uint32_t num_channels = stream->cfg.format & I2S_FMT_DATA_FORMAT_MASK
				      ? 2U : stream->cfg.channels;
	/* Number of data that can copy to TX FIFO */
	uint32_t tx_avail = I2S_FIFO_DEPTH - i2s->cfg.tx_fifo_trg_lvl;
	const uint8_t *buff = stream->mem_block; /* Assign the buffer base address */
	uint8_t last_lap = 0, bytes = 0, cnt = 0, frames = 0;
	uint32_t offset = stream->mem_block_offset;
	uint32_t size = stream->mem_block_size;
	int ret;

	/* Stop transmission if there was an error */
	if (stream->state == I2S_STATE_ERROR) {
		LOG_ERR("TX error detected");
		goto tx_disable;
	}

	/* Stop transmission if we were requested */
	if (stream->last_block) {
		stream->state = I2S_STATE_READY;
		goto tx_disable;
	}

	if (stream->cfg.word_size <= 16) {
		bytes = I2S_16BIT_BUF_TYPE;
	} else {
		bytes = I2S_32BIT_BUF_TYPE;
	}

	/* Check if it is the last lap */
	if ((offset + (2 * tx_avail * bytes)) >  size) {
		/* Assign the number of iterations required */
		frames = (size - offset)/(2*bytes);
		last_lap = 1;
	} else {
		frames = tx_avail;
	}

	for (cnt = 0; cnt < frames; cnt++) {
		/* Assuming that application uses 16bit buffer for 16bit data resolution */
		if (bytes == I2S_16BIT_BUF_TYPE) {
			if (num_channels == 1) {
				i2s_write_left_tx((uint32_t)
				(*(uint16_t *)(buff + offset)), i2s);
				i2s_write_right_tx(0, i2s);
				offset = offset + I2S_16BIT_BUF_TYPE;
			} else {
				i2s_write_left_tx((uint32_t)
				(*(uint16_t *)(buff + offset)), i2s);
				i2s_write_right_tx((uint32_t) (*(uint16_t *)
				(buff + offset + I2S_16BIT_BUF_TYPE)), i2s);
				offset = offset + 2 * I2S_16BIT_BUF_TYPE;
			}
		} else {
			/* For > 16bit data resolution */
			/* consider as 32bit buffer */
			if (num_channels == 1) {
				i2s_write_left_tx(*(uint32_t *)(buff + offset), i2s);
				i2s_write_right_tx(0, i2s);
				offset = offset + I2S_32BIT_BUF_TYPE;
			} else {
				i2s_write_left_tx(*(uint32_t *)(buff + offset), i2s);
				i2s_write_right_tx(*(uint32_t *)
					(buff + offset + I2S_32BIT_BUF_TYPE), i2s);
				offset = offset + (2 * I2S_32BIT_BUF_TYPE);
			}
		}
	}

	if (last_lap && (offset < size)) {
		if (num_channels == 1) {
			/* Write the Left sample and fill right with 0 */
			i2s_write_left_tx((uint32_t)
					(*(uint16_t *)(buff + offset)), i2s);
			i2s_write_right_tx(0, i2s);
			offset = offset + I2S_16BIT_BUF_TYPE;
		} else {
			/* Write the Left sample and fill right with 0 */
			i2s_write_left_tx(*(uint32_t *)(buff + offset), i2s);
			i2s_write_right_tx(0, i2s);
			offset = offset + I2S_32BIT_BUF_TYPE;
		}
	}

	stream->mem_block_offset = offset;

	/* Send complete event once all the data is copied to FIFO */
	if (offset >= size) {
		/* All block data sent */
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
		stream->mem_block_offset = 0;

		/* Prepare to send the next data block */
		ret = queue_get(&stream->mem_block_queue, &stream->mem_block,
				&stream->mem_block_size);
		if (ret < 0) {
			if (stream->state == I2S_STATE_STOPPING) {
				stream->state = I2S_STATE_READY;
			} else {
				stream->state = I2S_STATE_ERROR;
			}
			goto tx_disable;
		}
		k_sem_give(&stream->sem);
	}
	return;

tx_disable:
	tx_stream_disable(stream, dev);
}

static void i2s_rx_irq_handler(const struct device *dev)
{
	const struct i2s_dw_cfg *i2s = dev->config;
	struct i2s_dw_data *const dev_data = dev->data;
	struct stream *stream = &dev_data->rx;
	void *mblk_tmp;
	int ret;

	 /* Data available in RX FIFO */
	uint32_t rx_avail = i2s->cfg.rx_fifo_trg_lvl;
	uint8_t last_lap = 0, bytes = 0, cnt = 0, frames = 0;
	const uint32_t num_channels =
			stream->cfg.format & I2S_FMT_DATA_FORMAT_MASK
			? 2U : stream->cfg.channels;
	/* Assign the buffer base address */
	uint8_t *const buff  = stream->mem_block;
	uint32_t offset = stream->mem_block_offset;
	uint32_t size = stream->cfg.block_size;

	/* Stop reception if there was an error */
	if (stream->state == I2S_STATE_ERROR) {
		goto rx_disable;
	}
	/* Stop reception if we were requested */
	if (stream->state == I2S_STATE_STOPPING) {
		stream->state = I2S_STATE_READY;
		goto rx_disable;
	}

	if ((stream->cfg.word_size <= 16)) {
		bytes = I2S_16BIT_BUF_TYPE;
	} else {
		bytes = I2S_32BIT_BUF_TYPE;
	}
	/* Check if it is the last lap */
	if ((offset + (2 * rx_avail * bytes)) >  size) {
		/* Assign the number of iterations required */
		frames = (size - offset)/(2*bytes);
		last_lap = 1;
	} else {
		frames = rx_avail;
	}

	for (cnt = 0; cnt < frames; cnt++) {
		/* Assuming that application uses 16bit */
		/* buffer for 16bit data resolution */
		if (bytes == I2S_16BIT_BUF_TYPE) {
			if (num_channels == 1) {
				(*(uint16_t *)(buff + offset)) =
				(uint16_t)i2s_read_left_rx(i2s);
				i2s_read_right_rx(i2s);
				offset = offset + I2S_16BIT_BUF_TYPE;
			} else {
				(*(uint16_t *)(buff + offset)) =
				(uint16_t)i2s_read_left_rx(i2s);
				(*(uint16_t *)
				(buff + offset + I2S_16BIT_BUF_TYPE)) =
				(uint16_t)i2s_read_right_rx(i2s);
				offset = offset + 2 * I2S_16BIT_BUF_TYPE;
			}
		} else {
			/* For > 16bit data resolution consider */
			/* as 32bit buffer */
			if (num_channels == 1) {
				*(uint32_t *)(buff + offset) = i2s_read_left_rx(i2s);
				i2s_read_right_rx(i2s);
				offset = offset + I2S_32BIT_BUF_TYPE;
			} else {
				*(uint32_t *)(buff + offset) = i2s_read_left_rx(i2s);
				*(uint32_t *)(buff + offset + I2S_32BIT_BUF_TYPE) =
				i2s_read_right_rx(i2s);
				offset = offset + 2 * I2S_32BIT_BUF_TYPE;
			}
		}
	}

	if (last_lap && (offset < size)) {
		if (bytes == I2S_16BIT_BUF_TYPE) {
			/* Read the last sample from left */
			(*(uint16_t *)(buff + offset)) =
			(uint16_t)i2s_read_left_rx(i2s);
			i2s_read_right_rx(i2s);
			offset = offset + I2S_16BIT_BUF_TYPE;
		} else {
			/* Read the last sample from left */
			*(uint32_t *)(buff + offset) = i2s_read_left_rx(i2s);
			i2s_read_right_rx(i2s);
			offset = offset + I2S_32BIT_BUF_TYPE;
		}
	}

	stream->mem_block_offset = offset;

	/* Once the buffer is full, */
	/* send complete event with interrupt disabled */
	if (offset >= size) {
		mblk_tmp = stream->mem_block;

		/* Prepare to receive the next data block */
		ret = k_mem_slab_alloc(stream->cfg.mem_slab,
				       &stream->mem_block,
				       K_NO_WAIT);
		if (ret < 0) {
			stream->state = I2S_STATE_ERROR;
			goto rx_disable;
		}
		stream->mem_block_offset = 0;

		/* All block data received */
		ret = queue_put(&stream->mem_block_queue, mblk_tmp,
				stream->cfg.block_size);
		if (ret < 0) {
			stream->state = I2S_STATE_ERROR;
			goto rx_disable;
		}
		k_sem_give(&stream->sem);
	}
	return;

rx_disable:
	rx_stream_disable(stream, dev);
}

static void i2s_dw_isr(const struct device *dev)
{
	const struct i2s_dw_cfg *i2s = dev->config;
	struct i2s_dw_data *const dev_data = dev->data;
	uint32_t int_status = 0;

	/* Get the Current Interrupt Status*/
	int_status = i2s->paddr->ISR;

	if ((dev_data->dir == I2S_DIR_TX) &&
	    (_FLD2VAL(I2S_ISR_TXFE, int_status))) {
		/* Handle Tx Interrupt */
		i2s_tx_irq_handler(dev);
	}
	if ((dev_data->dir == I2S_DIR_RX) &&
	    (_FLD2VAL(I2S_ISR_RXDA, int_status))) {
		/* Handle Rx Interrupt */
		i2s_rx_irq_handler(dev);
	}

	/* This should not happen */
	if (_FLD2VAL(I2S_ISR_TXFO, int_status)) {
		i2s_clear_tx_overrun(i2s);
	}

	if (_FLD2VAL(I2S_ISR_RXFO, int_status)) {
		/* Clear overrun interrupt */
		i2s_clear_rx_overrun(i2s);

		/* Disable the Rx Overflow interrupt for now. This will */
		/* be enabled again when Receive function is called */
		i2s_disable_rx_fo_interrupt(i2s);
	}

}

static int i2s_dw_initialize(const struct device *dev)
{
	const struct i2s_dw_cfg *i2s = dev->config;
	struct i2s_dw_data *const dev_data = dev->data;
	int ret;

	/* Enable I2S clock propagation */
	i2s_enable_clock(dev);

#if defined(CONFIG_PINCTRL)
	if (i2s->pincfg != NULL) {
		ret = pinctrl_apply_state(i2s->pincfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("I2S pinctrl setup failed (%d)", ret);
			return ret;
		}
	}
#endif
	i2s->irq_config(dev);

	k_sem_init(&dev_data->rx.sem, 0, CONFIG_I2S_DW_RX_BLOCK_COUNT);
	k_sem_init(&dev_data->tx.sem, CONFIG_I2S_DW_TX_BLOCK_COUNT,
		   CONFIG_I2S_DW_TX_BLOCK_COUNT);

	/* Mask all the interrupts */
	i2s_disable_tx_interrupt(i2s);
	i2s_disable_rx_interrupt(i2s);
	LOG_INF("%s inited", dev->name);

	return 0;
}

static int rx_stream_start(struct stream *stream, const struct device *dev)
{
	const struct i2s_dw_cfg *i2s = dev->config;
	int ret;

	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
			       K_NO_WAIT);
	if (ret < 0) {
		return ret;
	}

	/* Configure the I2S Peripheral Clock */
	i2s_configure_clocksource(true, i2s, stream->cfg.frame_clk_freq);

	/* Reset the Rx FIFO */
	i2s_rx_fifo_reset(i2s);
	/* Set WLEN */
	i2s_rx_config_wlen(i2s, stream->cfg.word_size);
	/* Enable Master Clock */
	i2s_configure_clock(i2s);
	i2s_clock_enable(i2s);
	/* Disable Tx Channel */
	i2s_tx_channel_disable(i2s);

	/* Clear Overrun interrupt if any */
	i2s_clear_rx_overrun(i2s);

	/* Enable Rx Channel */
	i2s_rx_channel_enable(i2s);

	/* Enable RX Interrupts */
	i2s_enable_rx_interrupt(i2s);
	/* Enable Rx Block */
	i2s_rx_block_enable(i2s);

	return 0;
}

static int tx_stream_start(struct stream *stream, const struct device *dev)
{
	const struct i2s_dw_cfg *i2s = dev->config;
	int ret;

	ret = queue_get(&stream->mem_block_queue, &stream->mem_block,
			&stream->mem_block_size);
	if (ret < 0) {
		return ret;
	}
	k_sem_give(&stream->sem);

	stream->mem_block_offset = 0;

	/* Configure the I2S Peripheral Clock */
	i2s_configure_clocksource(true, i2s, stream->cfg.frame_clk_freq);

	/* Reset the Tx FIFO */
	i2s_tx_fifo_reset(i2s);

	/* Set WLEN */
	i2s_tx_config_wlen(i2s, stream->cfg.word_size);

	/* Enable Master Clock */
	i2s_configure_clock(i2s);
	i2s_clock_enable(i2s);

	/* Disable Rx Channel */
	i2s_rx_channel_disable(i2s);

	/* Clear Overrun interrupt if any */
	i2s_clear_tx_overrun(i2s);

	/* Enable Tx Channel */
	i2s_tx_channel_enable(i2s);

	/* Enable Tx Interrupt */
	i2s_enable_tx_interrupt(i2s);

	/* Enable Tx Block */
	i2s_tx_block_enable(i2s);

	return 0;
}

static void rx_stream_disable(struct stream *stream, const struct device *dev)
{
	const struct i2s_dw_cfg *i2s = dev->config;

	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}

	/* Disable Rx Channel */
	i2s_rx_channel_disable(i2s);
	/* Disable Rx Block */
	i2s_rx_block_disable(i2s);

	/* Disable Rx Interrupt */
	i2s_disable_rx_interrupt(i2s);
	/* Disable Master Clock */
	i2s_clock_disable(i2s);
}

static void tx_stream_disable(struct stream *stream, const struct device *dev)
{
	const struct i2s_dw_cfg *i2s = dev->config;

	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}
	/* Disable Tx Channel */
	i2s_tx_channel_disable(i2s);
	/* Disable Tx Block */
	i2s_tx_block_disable(i2s);

	/* Disable Tx Interrupt */
	i2s_disable_tx_interrupt(i2s);

	/* Disable Master Clock */
	i2s_clock_disable(i2s);
}

static void rx_queue_drop(struct stream *stream)
{
	size_t size;
	void *mem_block;

	while (queue_get(&stream->mem_block_queue, &mem_block, &size) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, mem_block);
	}

	k_sem_reset(&stream->sem);
}

static void tx_queue_drop(struct stream *stream)
{
	size_t size;
	void *mem_block;
	unsigned int n = 0U;

	while (queue_get(&stream->mem_block_queue, &mem_block, &size) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, mem_block);
		n++;
	}

	for (; n > 0; n--) {
		k_sem_give(&stream->sem);
	}
}

#define I2S_DW_INIT(index)						\
									\
static void i2s_dw_irq_config_func_##index(const struct device *dev);	\
									\
IF_ENABLED(DT_INST_NODE_HAS_PROP(index, pinctrl_0),			\
	(PINCTRL_DT_INST_DEFINE(index)));				\
									\
static const struct i2s_dw_cfg i2s_dw_config_##index = {		\
	.cfg.clk_source = DT_PROP(DT_DRV_INST(index), clock_frequency),	\
	.cfg.wss_len = WSS_LEN,						\
	.cfg.ext_clk_src_enable =					\
		DT_PROP(DT_DRV_INST(index), ext_clk_src_enable) ? 1:0,	\
	.cfg.tx_fifo_trg_lvl = TX_FIFO_TRG_LVL,		\
	.cfg.rx_fifo_trg_lvl = RX_FIFO_TRG_LVL,		\
	.instance = DT_INST_ENUM_IDX(index, driver_instance),		\
	.paddr = (struct I2S_Type *)DT_INST_REG_ADDR(index),		\
	.irq_config = i2s_dw_irq_config_func_##index,			\
	IF_ENABLED(DT_INST_NODE_HAS_PROP(index, pinctrl_0),		\
		(.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),))	\
};									\
									\
struct queue_item \
rx_##index##_dw_ring_buf[CONFIG_I2S_DW_RX_BLOCK_COUNT + 1];	\
struct queue_item \
tx_##index##_dw_ring_buf[CONFIG_I2S_DW_TX_BLOCK_COUNT + 1];	\
static struct i2s_dw_data i2s_dw_data_##index = {			\
	.tx = {.stream_start = tx_stream_start,				\
		.stream_disable = tx_stream_disable,			\
		.queue_drop = tx_queue_drop,				\
		.mem_block_queue.buf = tx_##index##_dw_ring_buf,		\
		.mem_block_queue.len = ARRAY_SIZE(tx##_##index##_dw_ring_buf)  },\
	.rx = {.stream_start = rx_stream_start, \
		.stream_disable = rx_stream_disable, \
		.queue_drop = rx_queue_drop,				\
		.mem_block_queue.buf = rx_##index##_dw_ring_buf,		\
		.mem_block_queue.len = ARRAY_SIZE(rx##_##index##_dw_ring_buf)  },\
};									\
DEVICE_DT_INST_DEFINE(index,						\
		      &i2s_dw_initialize, NULL,			\
		      &i2s_dw_data_##index,			\
		      &i2s_dw_config_##index, POST_KERNEL,	\
		      CONFIG_I2S_INIT_PRIORITY, &i2s_dw_driver_api); \
									\
static void i2s_dw_irq_config_func_##index(const struct device *dev) \
{									\
	IRQ_CONNECT(DT_INST_IRQN(index),				\
		    DT_INST_IRQ(index, priority),			\
		    i2s_dw_isr, DEVICE_DT_INST_GET(index), 0);		\
	irq_enable(DT_INST_IRQN(index));				\
}

DT_INST_FOREACH_STATUS_OKAY(I2S_DW_INIT)
