/*
 * Copyright 2020 Broadcom
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <errno.h>
#include <zephyr/init.h>
#include <string.h>
#include <soc.h>
#include <zephyr/sys/__assert.h>
#include "dma_pl330.h"
#include <soc_memory_map.h>

#define LOG_LEVEL CONFIG_DMA_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_pl330);

#define BYTE_WIDTH(burst_size) (1 << (burst_size))

#define DMA_CHANNEL_IS_FREE   0
#define DMA_CHANNEL_IS_IN_USE 1

static int dma_pl330_submit(const struct device *dev, uint64_t dst,
			    uint64_t src, uint32_t channel, uint32_t size);

static void dma_pl330_get_counter(struct dma_pl330_ch_internal *ch_handle,
				  uint32_t *psrc_byte_width,
				  uint32_t *pdst_byte_width,
				  uint32_t *ploop_counter,
				  uint32_t *presidue)
{
	uint32_t srcbytewidth, dstbytewidth;
	uint32_t loop_counter, residue;

	srcbytewidth = BYTE_WIDTH(ch_handle->src_burst_sz);
	dstbytewidth = BYTE_WIDTH(ch_handle->dst_burst_sz);

	loop_counter = ch_handle->trans_size /
		       (srcbytewidth * (ch_handle->src_burst_len + 1));

	residue = ch_handle->trans_size - loop_counter *
		  (srcbytewidth * (ch_handle->src_burst_len + 1));

	*psrc_byte_width = srcbytewidth;
	*pdst_byte_width = dstbytewidth;
	*ploop_counter = loop_counter;
	*presidue = residue;
}

static uint32_t dma_pl330_ch_ccr(struct dma_pl330_ch_internal *ch_handle)
{
	uint32_t ccr;
	int secure = ch_handle->nonsec_mode ? SRC_PRI_NONSEC_VALUE :
		     SRC_PRI_SEC_VALUE;

	ccr = ((ch_handle->dst_cache_ctrl & CC_SRCCCTRL_MASK) <<
		CC_DSTCCTRL_SHIFT) +
	       ((ch_handle->nonsec_mode) << CC_DSTNS_SHIFT) +
	       (ch_handle->dst_burst_len << CC_DSTBRSTLEN_SHIFT) +
	       (ch_handle->dst_burst_sz << CC_DSTBRSTSIZE_SHIFT) +
	       (ch_handle->dst_inc << CC_DSTINC_SHIFT) +
	       ((ch_handle->src_cache_ctrl & CC_SRCCCTRL_MASK) <<
		CC_SRCCCTRL_SHIFT) +
	       (secure << CC_SRCPRI_SHIFT) +
	       (ch_handle->src_burst_len << CC_SRCBRSTLEN_SHIFT)  +
	       (ch_handle->src_burst_sz << CC_SRCBRSTSIZE_SHIFT)  +
	       (ch_handle->src_inc << CC_SRCINC_SHIFT);

	return ccr;
}

static int dma_pl330_calc_burstsz_len(struct dma_pl330_ch_config *channel_cfg,
				       uint64_t dst, uint64_t src, uint32_t size,
					   uint8_t max_burst_size_log2)
{
	struct dma_pl330_ch_internal *ch_handle = &channel_cfg->internal;
	uint32_t burst_sz;

	if (channel_cfg->direction == PERIPHERAL_TO_MEMORY ||
		channel_cfg->direction == MEMORY_TO_PERIPHERAL) {

		if (channel_cfg->direction == PERIPHERAL_TO_MEMORY) {
			ch_handle->src_burst_sz = channel_cfg->src_burst_sz;
			ch_handle->dst_burst_sz = channel_cfg->src_burst_sz;
			ch_handle->src_burst_len = channel_cfg->src_blen;
			ch_handle->dst_burst_len = channel_cfg->src_blen;
		} else {
			ch_handle->src_burst_sz = channel_cfg->dst_burst_sz;
			ch_handle->dst_burst_sz = channel_cfg->dst_burst_sz;
			ch_handle->src_burst_len = channel_cfg->dst_blen;
			ch_handle->dst_burst_len = channel_cfg->dst_blen;
		}

		if ((src | dst | size) & ((BYTE_WIDTH(ch_handle->src_burst_sz)) - 1)) {
			return -EINVAL;
		}
	} else {

		burst_sz = max_burst_size_log2;
		/* src, dst and size should be aligned to burst size in bytes */
		while ((src | dst | size) & ((BYTE_WIDTH(burst_sz)) - 1)) {
			burst_sz--;
		}

		ch_handle->src_burst_len = channel_cfg->dst_blen;
		ch_handle->src_burst_sz = burst_sz;
		ch_handle->dst_burst_len = channel_cfg->dst_blen;
		ch_handle->dst_burst_sz = burst_sz;
	}

	return 0;
}

#ifdef CONFIG_DMA_64BIT
static void dma_pl330_cfg_dmac_add_control(uint32_t control_reg_base,
					   uint64_t dst, uint64_t src, int ch)
{
	uint32_t src_h = src >> 32;
	uint32_t dst_h = dst >> 32;
	uint32_t dmac_higher_addr;

	dmac_higher_addr = ((dst_h & HIGHER_32_ADDR_MASK) << DST_ADDR_SHIFT) |
			   (src_h & HIGHER_32_ADDR_MASK);

	sys_write32(dmac_higher_addr,
		    control_reg_base +
		    (ch * CONTROL_OFFSET)
		   );
}
#endif

static void dma_pl330_config_channel(struct dma_pl330_ch_config *ch_cfg,
				     uint64_t dst, uint64_t src, uint32_t size)
{
	struct dma_pl330_ch_internal *ch_handle = &ch_cfg->internal;

	ch_handle->src_addr = src;
	ch_handle->dst_addr = dst;
	ch_handle->trans_size = size;

	if (ch_cfg->direction == PERIPHERAL_TO_MEMORY) {
		ch_handle->src_id = ch_cfg->periph_slot;
		ch_handle->breq_only = 1;
		ch_handle->dst_cache_ctrl = CC_CCTRL_MODIFIABLE_VALUE;
	} else if (ch_cfg->direction == MEMORY_TO_PERIPHERAL) {
		ch_handle->dst_id = ch_cfg->periph_slot;
		ch_handle->breq_only = 1;
		ch_handle->src_cache_ctrl = CC_CCTRL_MODIFIABLE_VALUE;
	} else if (ch_cfg->direction == MEMORY_TO_MEMORY) {
		ch_handle->src_cache_ctrl = CC_CCTRL_MODIFIABLE_VALUE;
		ch_handle->dst_cache_ctrl = CC_CCTRL_MODIFIABLE_VALUE;
	}


	if (ch_cfg->src_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
		ch_handle->src_inc = 1;
	}

	if (ch_cfg->dst_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
		ch_handle->dst_inc = 1;
	}
}

static inline uint32_t dma_pl330_gen_mov(mem_addr_t buf,
					 enum dmamov_type type,
					 uint32_t val)
{
	sys_write8(OP_DMA_MOV, buf);
	sys_write8(type, buf + 1);
	sys_write8(val, buf + 2);
	sys_write8(val >> 8, buf + 3);
	sys_write8(val >> 16, buf + 4);
	sys_write8(val >> 24, buf + 5);

	return SZ_CMD_DMAMOV;
}

static inline void dma_pl330_gen_op(uint8_t opcode, uint32_t addr, uint32_t val)
{
	sys_write8(opcode, addr);
	sys_write8(val, addr + 1);
}

static uint32_t dma_pl330_gen_copy_op(struct dma_pl330_ch_internal *ch_dat,
					mem_addr_t dma_exec_addr, uint32_t offset,
					enum dma_channel_direction direction, uint32_t burst_len)
{
	uint8_t req_type;

	if (ch_dat->breq_only) {
		req_type = DMA_PERIPH_REQ_TYPE_BURST;
	} else {
		req_type = burst_len ?
					DMA_PERIPH_REQ_TYPE_BURST : DMA_PERIPH_REQ_TYPE_SINGLE;
	}

	if (direction == PERIPHERAL_TO_MEMORY) {
		dma_pl330_gen_op(OP_DMA_FLUSHP, dma_exec_addr + offset,
				((ch_dat->src_id) << 3));
		offset = offset + 2;
		dma_pl330_gen_op(OP_DMA_WFP(req_type), dma_exec_addr + offset,
				((ch_dat->src_id) << 3));
		offset = offset + 2;
		dma_pl330_gen_op(OP_DMA_LDP(req_type), dma_exec_addr + offset,
				((ch_dat->src_id) << 3));
		offset = offset + 2;
		if (req_type) {
			sys_write8(OP_DMA_ST(DMA_LDST_REQ_TYPE_BURST),
				dma_exec_addr + offset);
		} else {
			sys_write8(OP_DMA_ST(DMA_LDST_REQ_TYPE_SINGLE),
				dma_exec_addr + offset);
		}
		offset = offset + 1;
	} else if (direction == MEMORY_TO_PERIPHERAL) {
		dma_pl330_gen_op(OP_DMA_FLUSHP, dma_exec_addr + offset,
				((ch_dat->dst_id) << 3));
		offset = offset + 2;
		dma_pl330_gen_op(OP_DMA_WFP(req_type), dma_exec_addr + offset,
				((ch_dat->dst_id) << 3));
		offset = offset + 2;
		if (req_type) {
			sys_write8(OP_DMA_LD(DMA_LDST_REQ_TYPE_BURST),
				dma_exec_addr + offset);
		} else {
			sys_write8(OP_DMA_LD(DMA_LDST_REQ_TYPE_SINGLE),
				dma_exec_addr + offset);
		}
		offset = offset + 1;
		dma_pl330_gen_op(OP_DMA_STP(req_type), dma_exec_addr + offset,
				((ch_dat->src_id) << 3));
		offset = offset + 2;
	} else {
		sys_write8(OP_DMA_LD(DMA_LDST_REQ_TYPE_FORCE),
			dma_exec_addr + offset);
		sys_write8(OP_DMA_ST(DMA_LDST_REQ_TYPE_FORCE),
			dma_exec_addr + offset + 1);
		offset = offset + 2;
	}

	return offset;
}

static int dma_pl330_setup_ch(const struct device *dev,
			      struct dma_pl330_ch_internal *ch_dat,
			      int ch)
{
	mem_addr_t dma_exec_addr;
	uint32_t offset = 0, ccr;
	uint32_t lp0_start, lp1_start;
	uint32_t loop_counter0 = 0, loop_counter1 = 0;
	uint32_t srcbytewidth, dstbytewidth;
	uint32_t loop_counter, residue;
	struct dma_pl330_dev_data *const dev_data = dev->data;
	struct dma_pl330_ch_config *channel_cfg;
	unsigned int irq = dev_data->event_irq[ch];
	uint32_t residue_blen;

	channel_cfg = &dev_data->channels[ch];
	dma_exec_addr = channel_cfg->dma_exec_addr;

	offset  += dma_pl330_gen_mov(dma_exec_addr,
				     SAR, ch_dat->src_addr);

	offset  += dma_pl330_gen_mov(dma_exec_addr + offset,
				     DAR, ch_dat->dst_addr);

	ccr = dma_pl330_ch_ccr(ch_dat);

	offset  += dma_pl330_gen_mov(dma_exec_addr + offset,
				     CCR, ccr);

	dma_pl330_get_counter(ch_dat, &srcbytewidth, &dstbytewidth,
			      &loop_counter, &residue);

	if (loop_counter >= PL330_LOOP_COUNTER0_MAX) {
		loop_counter0 = PL330_LOOP_COUNTER0_MAX - 1;
		loop_counter1 = loop_counter / PL330_LOOP_COUNTER0_MAX - 1;
		dma_pl330_gen_op(OP_DMA_LOOP_COUNT1, dma_exec_addr + offset,
				 loop_counter1 & 0xff);
		offset = offset + 2;
		dma_pl330_gen_op(OP_DMA_LOOP, dma_exec_addr + offset,
				 loop_counter0 & 0xff);
		offset = offset + 2;
		lp1_start = offset;
		lp0_start = offset;

		offset = dma_pl330_gen_copy_op(ch_dat, dma_exec_addr,
					offset, channel_cfg->direction, ch_dat->dst_burst_len);
		dma_pl330_gen_op(OP_DMA_LP_BK_JMP1, dma_exec_addr + offset,
				 ((offset - lp0_start) & 0xff));
		offset = offset + 2;
		dma_pl330_gen_op(OP_DMA_LOOP, dma_exec_addr + offset,
				 (loop_counter0 & 0xff));
		offset = offset + 2;
		loop_counter1--;
		dma_pl330_gen_op(OP_DMA_LP_BK_JMP2, dma_exec_addr + offset,
				 ((offset - lp1_start) & 0xff));
		offset = offset + 2;
	}

	if ((loop_counter % PL330_LOOP_COUNTER0_MAX) != 0) {
		loop_counter0 = (loop_counter % PL330_LOOP_COUNTER0_MAX) - 1;
		dma_pl330_gen_op(OP_DMA_LOOP, dma_exec_addr + offset,
				 (loop_counter0 & 0xff));
		offset = offset + 2;
		loop_counter1--;
		lp0_start = offset;
		offset = dma_pl330_gen_copy_op(ch_dat, dma_exec_addr,
					offset, channel_cfg->direction, ch_dat->dst_burst_len);
		dma_pl330_gen_op(OP_DMA_LP_BK_JMP1, dma_exec_addr + offset,
				 ((offset - lp0_start) & 0xff));
		offset = offset + 2;
	}

	if (residue != 0) {
		residue_blen = (residue / srcbytewidth) - 1;
		ccr = ccr & ~((CC_BRSTLEN_MASK << CC_SRCBRSTLEN_SHIFT) |
					  (CC_BRSTLEN_MASK << CC_DSTBRSTLEN_SHIFT));
		ccr = ccr | ((residue_blen << CC_SRCBRSTLEN_SHIFT) |
					 (residue_blen << CC_DSTBRSTLEN_SHIFT));

		offset += dma_pl330_gen_mov(dma_exec_addr + offset,
					    CCR, ccr);
		offset = dma_pl330_gen_copy_op(ch_dat, dma_exec_addr,
					offset, channel_cfg->direction, residue_blen);
	}

	sys_write8(OP_DMA_WMB, dma_exec_addr + offset);
	offset = offset + 1;

	if (channel_cfg->dma_callback) {
		dma_pl330_gen_op(OP_DMA_SEV, dma_exec_addr + offset,
				((irq & 0x1f) << 3));
		offset = offset + 2;
	}

	sys_write8(OP_DMA_END, dma_exec_addr + offset);
	sys_write8(OP_DMA_END, dma_exec_addr + offset + 1);
	sys_write8(OP_DMA_END, dma_exec_addr + offset + 2);
	sys_write8(OP_DMA_END, dma_exec_addr + offset + 3);

	channel_cfg->loop_counter0 = loop_counter0;

	return 0;
}

static int dma_pl330_start_dma_ch(const struct device *dev,
				  uint32_t reg_base, int ch, int secure)
{
	struct dma_pl330_dev_data *const dev_data = dev->data;
	struct dma_pl330_ch_config *channel_cfg;
	uint32_t count = 0U;
	uint32_t data;
	uint32_t inten;
	unsigned int irq_key;
	uint32_t irq = dev_data->event_irq[ch];

	channel_cfg = &dev_data->channels[ch];
	do {
		data = sys_read32(reg_base + DMAC_PL330_DBGSTATUS);
		if (++count > DMA_TIMEOUT_US) {
			return -ETIMEDOUT;
		}
		k_busy_wait(1);
	} while ((data & DATA_MASK) != 0);

	irq_key = irq_lock();

	sys_write32(((ch << DMA_INTSR1_SHIFT) +
		    (DMA_INTSR0 << DMA_INTSR0_SHIFT) +
		    (secure << DMA_SECURE_SHIFT) + (ch << DMA_CH_SHIFT)),
		    reg_base + DMAC_PL330_DBGINST0);

	sys_write32(local_to_global(UINT_TO_POINTER(channel_cfg->dma_exec_addr)),
		    reg_base + DMAC_PL330_DBGINST1);

	if (channel_cfg->dma_callback) {
		inten = sys_read32(reg_base + DMAC_PL330_INTEN) | (1 << irq);
		sys_write32(inten, reg_base + DMAC_PL330_INTEN);
	}

	sys_write32(0x0, reg_base + DMAC_PL330_DBGCMD);

	irq_unlock(irq_key);

	count = 0U;
	do {
		data = sys_read32(reg_base + DMAC_PL330_DBGCMD);
		if (++count > DMA_TIMEOUT_US) {
			return -ETIMEDOUT;
		}
		k_busy_wait(1);
	} while ((data & DATA_MASK) != 0);

	return 0;
}

static int dma_pl330_wait(uint32_t reg_base, int ch)
{
	int count = 0U;
	uint32_t cs0_reg = reg_base + DMAC_PL330_CS0;

	do {
		if (++count > DMA_TIMEOUT_US) {
			return -ETIMEDOUT;
		}
		k_busy_wait(1);
	} while (((sys_read32(cs0_reg + ch * 8)) & CH_STATUS_MASK) != 0);

	return 0;
}

static int dma_pl330_stop_dma_ch(const struct device *dev,
				  uint32_t reg_base, int ch)
{
	struct dma_pl330_dev_data *const dev_data = dev->data;
	int ret;
	uint32_t data, inten, count = 0U;
	uint32_t irq = dev_data->event_irq[ch];

	do {
		data = sys_read32(reg_base + DMAC_PL330_DBGSTATUS);
		if ((data & DATA_MASK) == 0)
			break;

		if (++count > DMA_TIMEOUT_US) {
			return -ETIMEDOUT;
		}
		k_busy_wait(1);
	} while (1);

	sys_write32(((OP_DMA_KILL << DMA_INTSR0_SHIFT) +
		(ch << DMA_CH_SHIFT) + DMA_DBG_CHN),
		reg_base + DMAC_PL330_DBGINST0);
	sys_write32(0, reg_base + DMAC_PL330_DBGINST1);
	sys_write32(0x0, reg_base + DMAC_PL330_DBGCMD);

	ret = dma_pl330_wait(reg_base, ch);
	inten = sys_read32(reg_base + DMAC_PL330_INTEN);
	inten = inten & ~(1U << irq);
	sys_write32(inten, reg_base + DMAC_PL330_INTEN);
	sys_write32((1 << irq), reg_base + DMAC_PL330_INTCLR);

	return ret;
}

static int dma_pl330_xfer(const struct device *dev, uint64_t dst,
			  uint64_t src, uint32_t size, uint32_t channel,
			  uint32_t *xfer_size)
{
	struct dma_pl330_dev_data *const dev_data = dev->data;
	const struct dma_pl330_config *const dev_cfg = dev->config;
	struct dma_pl330_ch_config *channel_cfg;
	struct dma_pl330_ch_internal *ch_handle;
	int ret;
	uint32_t max_size;

	channel_cfg = &dev_data->channels[channel];
	ch_handle = &channel_cfg->internal;

	ret = dma_pl330_calc_burstsz_len(channel_cfg, dst, src,
					size, dev_data->axi_data_width);
	if (ret) {
		LOG_ERR("Error in Burst size/len for DMA PL330");
		goto err;
	}

	max_size = GET_MAX_DMA_SIZE((1 << ch_handle->src_burst_sz),
				    ch_handle->src_burst_len);

	if (size > max_size) {
		size = max_size;
	}

	dma_pl330_config_channel(channel_cfg, dst, src, size);
#ifdef CONFIG_DMA_64BIT
	/*
	 * Pl330 supports only 4GB boundary, but boundary region can be
	 * configured.
	 * Support added for 36bit address, lower 32bit address are configured
	 * in pl330 registers and higher 4bit address are configured in
	 * LS_ICFG_DMAC_AXI_ADD_CONTROL registers.
	 * Each channel has 1 control register to configure higher 4bit address.
	 */

	dma_pl330_cfg_dmac_add_control(dev_cfg->control_reg_base,
				       dst, src, channel);
#endif
	ret = dma_pl330_setup_ch(dev, ch_handle, channel);
	if (ret) {
		LOG_ERR("Failed to setup channel for DMA PL330");
		goto err;
	}

	ret = dma_pl330_start_dma_ch(dev, dev_cfg->reg_base, channel,
				     ch_handle->nonsec_mode);
	if (ret) {
		LOG_ERR("Failed to start DMA PL330");
		goto err;
	}

	if (!channel_cfg->dma_callback) {
		ret = dma_pl330_wait(dev_cfg->reg_base, channel);
		if (ret) {
			LOG_ERR("Failed waiting to finish DMA PL330");
			goto err;
		}
	}

	*xfer_size = size;
err:
	return ret;
}

#if CONFIG_DMA_64BIT
static int dma_pl330_handle_boundary(const struct device *dev, uint64_t dst,
				     uint64_t src, uint32_t channel,
				     uint32_t size)
{
	uint32_t dst_low = (uint32_t)dst;
	uint32_t src_low = (uint32_t)src;
	uint32_t transfer_size;
	int ret;

	/*
	 * Pl330 has only 32bit registers and supports 4GB memory.
	 * 4GB memory window can be configured using DMAC_AXI_ADD_CONTROL
	 * registers.
	 * Divide the DMA operation in 2 parts, 1st DMA from given address
	 * to boundary (0xffffffff) and 2nd DMA on remaining size.
	 */

	if (size > (PL330_MAX_OFFSET - dst_low)) {
		transfer_size = PL330_MAX_OFFSET - dst_low;
		ret = dma_pl330_submit(dev, dst, src, channel,
				       transfer_size);
		if (ret < 0) {
			return ret;
		}

		dst += transfer_size;
		src += transfer_size;
		size -= transfer_size;
		return dma_pl330_submit(dev, dst, src, channel, size);
	}

	if (size > (PL330_MAX_OFFSET - src_low)) {
		transfer_size = PL330_MAX_OFFSET - src_low;
		ret = dma_pl330_submit(dev, dst, src, channel, transfer_size);
		if (ret < 0) {
			return ret;
		}

		src += transfer_size;
		dst += transfer_size;
		size -= transfer_size;
		return dma_pl330_submit(dev, dst, src, channel, size);
	}

	return 0;
}
#endif

static int dma_pl330_submit(const struct device *dev, uint64_t dst,
			    uint64_t src,
			    uint32_t channel, uint32_t size)
{
	int ret;
	uint32_t xfer_size;

#if CONFIG_DMA_64BIT
	/*
	 * Pl330 has only 32bit registers and supports 4GB memory.
	 * 4GB memory window can be configured using DMAC_AXI_ADD_CONTROL
	 * registers. 32bit boundary (0xffffffff) should be check.
	 * DMA on boundary condition is taken care in below function.
	 */

	if ((size > (PL330_MAX_OFFSET - (uint32_t)dst)) ||
	    (size > (PL330_MAX_OFFSET - (uint32_t)src))) {
		return dma_pl330_handle_boundary(dev, dst, src,
						 channel, size);
	}
#endif
	while (size) {
		xfer_size = 0;
		ret = dma_pl330_xfer(dev, dst, src, size,
				     channel, &xfer_size);
		if (ret) {
			return ret;
		}
		if (xfer_size > size) {
			return -EFAULT;
		}
		size -= xfer_size;
		dst += xfer_size;
		src += xfer_size;
	}

	return 0;
}

static int dma_pl330_configure(const struct device *dev, uint32_t channel,
			       struct dma_config *cfg)
{
	const struct dma_pl330_config *const dev_cfg = dev->config;
	struct dma_pl330_dev_data *const dev_data = dev->data;
	struct dma_pl330_ch_config *channel_cfg;
	struct dma_pl330_ch_internal *ch_handle;

	if (channel >= dev_cfg->max_dma_channels) {
		return -EINVAL;
	}

	if (cfg->source_burst_length > MAX_BURST_LEN ||
		cfg->dest_burst_length > MAX_BURST_LEN) {
		return -EINVAL;
	}

	channel_cfg = &dev_data->channels[channel];

	if (atomic_get(&channel_cfg->channel_is_active) != DMA_CHANNEL_IS_FREE) {
		return -EBUSY;
	}

	ch_handle = &channel_cfg->internal;
	memset(ch_handle, 0, sizeof(*ch_handle));

	if ((cfg->channel_direction == PERIPHERAL_TO_MEMORY
		|| cfg->channel_direction == MEMORY_TO_PERIPHERAL)
		&& (cfg->dma_slot > dev_data->num_periph_req)) {
		return -EINVAL;
	}

	channel_cfg->periph_slot = cfg->dma_slot;

	channel_cfg->direction = cfg->channel_direction;
	channel_cfg->dst_addr_adj = cfg->head_block->dest_addr_adj;

	channel_cfg->src_addr =
		local_to_global(UINT_TO_POINTER(cfg->head_block->source_address));
	channel_cfg->dst_addr =
		local_to_global(UINT_TO_POINTER(cfg->head_block->dest_address));
	channel_cfg->trans_size = cfg->head_block->block_size;

	channel_cfg->src_burst_sz = cfg->source_data_size;
	channel_cfg->dst_burst_sz = cfg->dest_data_size;
	channel_cfg->src_blen = cfg->source_burst_length;
	channel_cfg->dst_blen = cfg->dest_burst_length;

	channel_cfg->dma_callback = cfg->dma_callback;
	channel_cfg->user_data = cfg->user_data;

	if (cfg->head_block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT ||
	    cfg->head_block->source_addr_adj == DMA_ADDR_ADJ_NO_CHANGE) {
		channel_cfg->src_addr_adj = cfg->head_block->source_addr_adj;
	} else {
		return -ENOTSUP;
	}

	if (cfg->head_block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT ||
	    cfg->head_block->dest_addr_adj == DMA_ADDR_ADJ_NO_CHANGE) {
		channel_cfg->dst_addr_adj = cfg->head_block->dest_addr_adj;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int dma_pl330_transfer_start(const struct device *dev,
				    uint32_t channel)
{
	const struct dma_pl330_config *const dev_cfg = dev->config;
	struct dma_pl330_dev_data *const dev_data = dev->data;
	struct dma_pl330_ch_config *channel_cfg;
	int ret;

	if (channel >= dev_cfg->max_dma_channels) {
		return -EINVAL;
	}

	channel_cfg = &dev_data->channels[channel];

	if (!atomic_cas(&channel_cfg->channel_is_active, DMA_CHANNEL_IS_FREE,
			DMA_CHANNEL_IS_IN_USE)) {
		return -EBUSY;
	}

	ret = dma_pl330_submit(dev, channel_cfg->dst_addr,
			       channel_cfg->src_addr, channel,
			       channel_cfg->trans_size);

	if (!channel_cfg->dma_callback || ret) {
		/* Free the channel if polling was used or en error has happen */
		atomic_set(&channel_cfg->channel_is_active, DMA_CHANNEL_IS_FREE);
	}

	return ret;
}

static int dma_pl330_transfer_stop(const struct device *dev, uint32_t channel)
{
	const struct dma_pl330_config *const dev_cfg = dev->config;
	struct dma_pl330_dev_data *const dev_data = dev->data;
	uint32_t reg_base = dev_cfg->reg_base;
	uint32_t cs0_reg = reg_base + DMAC_PL330_CS0;
	unsigned int irq_key;
	int ret = 0;

	if (channel >= dev_cfg->max_dma_channels) {
		return -EINVAL;
	}

	if ((sys_read32(cs0_reg + channel * 8) & CH_STATUS_MASK) == 0) {
		return 0;
	}

	irq_key = irq_lock();

	ret = dma_pl330_stop_dma_ch(dev, reg_base, channel);
	atomic_set(&dev_data->channels[channel].channel_is_active, DMA_CHANNEL_IS_FREE);

	irq_unlock(irq_key);
	return ret;
}

static void dma_pl330_isr(const struct device *dev)
{
	const struct dma_pl330_config *const dev_cfg = dev->config;
	struct dma_pl330_dev_data *const dev_data = dev->data;
	struct dma_pl330_ch_config *channel_cfg;
	int err = 0;
	uint32_t intmis, fsrc, ch;
	uint32_t reg_base = dev_cfg->reg_base;

	/* First make sure there is no Abort in Manager */
	if (sys_read32(reg_base + DMAC_PL330_FSRD) & 0x1) {
		LOG_ERR("DMA Manager thread is faulting = %x",
			sys_read32(reg_base + DMAC_PL330_FTRD));
	}

	/*
	 *	Read Channel Fault status
	 */
	fsrc = sys_read32(reg_base + DMAC_PL330_FSRC);
	if (fsrc) {
		for (ch = 0; ch < dev_cfg->max_dma_channels; ch++) {
			if (fsrc & (1 << ch)) {
				channel_cfg = &dev_data->channels[ch];

				LOG_ERR("DMA Channel %d is faulting = %x",
					ch, sys_read32(reg_base + DMAC_PL330_FTR0 + (ch * 4)));

				(void)dma_pl330_stop_dma_ch(dev, reg_base, ch);

				channel_cfg = &dev_data->channels[ch];

				atomic_set(&channel_cfg->channel_is_active, DMA_CHANNEL_IS_FREE);

				if (channel_cfg->dma_callback) {
					channel_cfg->dma_callback(
						dev, channel_cfg->user_data, ch, -EIO);
				}
			}
		}
	} else {
		/*
		 *	Issue channel callback
		 *	Here the assumption is that channel number and irq number
		 *	are mapped one-to-one
		 */
		intmis = sys_read32(reg_base + DMAC_PL330_INTMIS);

		for (ch = 0; ch < dev_cfg->max_dma_channels; ch++) {
			if (intmis & (1 << ch)) {
				sys_write32((1 << ch), reg_base + DMAC_PL330_INTCLR);

				channel_cfg = &dev_data->channels[ch];

				atomic_set(&channel_cfg->channel_is_active, DMA_CHANNEL_IS_FREE);

				if (channel_cfg->dma_callback) {
					channel_cfg->dma_callback(
						dev, channel_cfg->user_data, ch, err);
				}
			}
		}
	}
}

static int dma_pl330_initialize(const struct device *dev)
{
	const struct dma_pl330_config *const dev_cfg = dev->config;
	struct dma_pl330_dev_data *const dev_data = dev->data;
	struct dma_pl330_ch_config *channel_cfg;
	uint8_t event_index;

	for (int channel = 0; channel < dev_cfg->max_dma_channels; channel++) {
		channel_cfg = &dev_data->channels[channel];
		channel_cfg->dma_exec_addr = dev_cfg->mcode_base +
					(channel * MICROCODE_SIZE_MAX);
		atomic_set(&channel_cfg->channel_is_active, DMA_CHANNEL_IS_FREE);
	}

	for (event_index = 0; event_index < DMA_MAX_EVENTS; event_index++) {
		dev_data->event_irq[event_index] = -1;
	}

	dev_cfg->irq_configure(dev);

	dev_data->num_periph_req = ((sys_read32(dev_cfg->reg_base + DMAC_PL330_CR0)
								 >> DMA_NUM_PERIPH_REQ_SHIFT)
								& DMA_NUM_PERIPH_REQ_MASK);

	dev_data->axi_data_width = sys_read32(dev_cfg->reg_base + DMAC_PL330_CRD)
								& DMA_AXI_DATA_WIDTH_MASK;

	LOG_INF("Device %s initialized", dev->name);

	return 0;
}

static int dma_pl330_get_status(const struct device *dev, uint32_t channel,
				struct dma_status *stat)
{
	const struct dma_pl330_config *const dev_cfg = dev->config;
	struct dma_pl330_dev_data *const dev_data = dev->data;
	struct dma_pl330_ch_config *channel_cfg = &dev_data->channels[channel];
	const uint32_t reg_base = dev_cfg->reg_base;

	if (!stat || channel >= dev_cfg->max_dma_channels) {
		return -EINVAL;
	}

	stat->busy = atomic_get(&channel_cfg->channel_is_active) != DMA_CHANNEL_IS_FREE;
	stat->dir = channel_cfg->direction;
	/* Pending length is calculated based on the loop counters.
	 * Two possible loops LC0 and LC1 are used.
	 */
	stat->pending_length =
		sys_read32(reg_base + DMA_PL330_LC1_n(channel)) * (channel_cfg->loop_counter0 + 1) +
		sys_read32(reg_base + DMA_PL330_LC0_n(channel)) + 1;
	/* TODO: add rest when needed... */
	stat->free = 0;
	stat->write_position = 0;
	stat->read_position = 0;
	stat->total_copied = 0;

	return 0;
}

static int dma_pl330_dma_reload(const struct device *dev, uint32_t const channel,
				uint32_t const src, uint32_t const dst,
				size_t const size)
{
	const struct dma_pl330_config *const dev_cfg = dev->config;
	struct dma_pl330_dev_data *const dev_data = dev->data;
	struct dma_pl330_ch_config *channel_cfg = &dev_data->channels[channel];

	if (channel >= dev_cfg->max_dma_channels) {
		return -EINVAL;
	}

	if (atomic_get(&channel_cfg->channel_is_active) != DMA_CHANNEL_IS_FREE) {
		return -EBUSY;
	}

	channel_cfg->src_addr = local_to_global(UINT_TO_POINTER(src));
	channel_cfg->dst_addr = local_to_global(UINT_TO_POINTER(dst));
	channel_cfg->trans_size = size;

	return 0;
}

static const struct dma_driver_api pl330_driver_api = {
	.config = dma_pl330_configure,
	.reload = dma_pl330_dma_reload,
	.start = dma_pl330_transfer_start,
	.stop = dma_pl330_transfer_stop,
	.get_status = dma_pl330_get_status,
};

#define IRQ_CONFIGURE(n, inst)                                                                 \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, n, irq),                                          \
		    DT_INST_IRQ_BY_IDX(inst, n, priority), dma_pl330_isr,                      \
		    DEVICE_DT_INST_GET(inst), 0);                                              \
	irq_enable(DT_INST_IRQ_BY_IDX(inst, n, irq));

#define CHANNEL_TO_EVENTIRQ(n, inst)                                                           \
	IF_ENABLED(DT_IRQ_HAS_NAME(DT_DRV_INST(inst), channel##n),                             \
			(pl330_data##inst.event_irq[n] = n;))

#define CONFIGURE_ALL_IRQS(inst, n) LISTIFY(n, IRQ_CONFIGURE, (), inst)

#define ASSIGN_CHANNELS_TO_EVENTIRQ(inst, n)                                                   \
	LISTIFY(n, CHANNEL_TO_EVENTIRQ, (), inst)

#define MCODE_BASE_ALLOC(inst)                                                                 \
	IF_DISABLED(DT_INST_NODE_HAS_PROP(inst, microcode),                                    \
			(static uint8_t __aligned(4) dma##inst##_pl330_mcode_buf               \
			 [DT_INST_PROP(inst, dma_channels) * MICROCODE_SIZE_MAX];))

#define CONTROL_REG_BASE(inst)                                                                 \
	IF_ENABLED(CONFIG_DMA_64BIT, (.control_reg_base =                                      \
			 COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, control_regs),                \
				 DT_INST_REG_ADDR_BY_NAME(inst, control_regs), (0)),))         \

#define CHANNELS(inst)                                                                         \
	static struct dma_pl330_ch_config dma##inst##_pl330_channels                           \
	[DT_INST_PROP(inst, dma_channels)];

/********************** Device Definition per instance Macros. ***********************/
#define DMAC_PL330_INIT(inst)                                                                  \
	static void dma_pl330_irq_configure_##inst(const struct device *dev);                  \
	MCODE_BASE_ALLOC(inst);                                                                \
	CHANNELS(inst);                                                                        \
                                                                                               \
	static const struct dma_pl330_config pl330_config##inst = {                            \
		.reg_base = DT_INST_REG_ADDR(inst),                                            \
		CONTROL_REG_BASE(inst)                                                         \
		.mcode_base = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, microcode),              \
				DT_INST_PROP_BY_IDX(inst, microcode, 0),                       \
				POINTER_TO_UINT(dma##inst##_pl330_mcode_buf)),                 \
		.max_dma_channels = DT_INST_PROP(inst, dma_channels),                          \
		.irq_configure = dma_pl330_irq_configure_##inst,                               \
		.num_irqs = DT_NUM_IRQS(DT_DRV_INST(inst)),                                    \
	};                                                                                     \
                                                                                               \
	static struct dma_pl330_dev_data pl330_data##inst = {                                  \
		.channels = dma##inst##_pl330_channels,                                        \
	};                                                                                     \
                                                                                               \
	DEVICE_DT_INST_DEFINE(inst, &dma_pl330_initialize, NULL,                               \
				&pl330_data##inst, &pl330_config##inst,                        \
				POST_KERNEL, CONFIG_DMA_INIT_PRIORITY,                         \
				&pl330_driver_api);                                            \
                                                                                               \
	static void dma_pl330_irq_configure_##inst(const struct device *dev)                   \
	{                                                                                      \
		CONFIGURE_ALL_IRQS(inst, DT_NUM_IRQS(DT_DRV_INST(inst)));                      \
		ASSIGN_CHANNELS_TO_EVENTIRQ(inst, DT_INST_PROP(inst, dma_channels))            \
	}

DT_INST_FOREACH_STATUS_OKAY(DMAC_PL330_INIT)
