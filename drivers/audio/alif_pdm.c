/*
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_alif_pdm

#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/pdm/pdm_alif.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/init.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/drivers/clock_control.h>
#include "alif_pdm_reg.h"

#ifdef CONFIG_ALIF_PDM_USE_DMA
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_pl330_opcode.h>
#include <zephyr/drivers/dma/dma_pl330.h>
#include <zephyr/dt-bindings/dma/alif_dma_event_router.h>
#include <soc_memory_map.h>
#endif

LOG_MODULE_REGISTER(alif_pdm, LOG_LEVEL_INF);

#define DEV_DATA(dev) ((struct pdm_data *)((dev)->data))
#define DEV_CFG(dev)  ((const struct pdm_config *)((dev)->config))

#ifdef CONFIG_ALIF_PDM_USE_DMA
#define PDM_DMA_MCODE_SIZE 256
#define PDM_USE_DMA_HANDSHAKE  BIT(24)
#endif

struct pdm_data {
	DEVICE_MMIO_RAM;
	struct k_mem_slab *mem_slab;
	uint32_t block_size;
	struct k_msgq buf_queue;
	uint8_t channel_map;
	uint32_t num_channels;
	uint8_t *data_buffer;
	uint32_t buf_index;
	uint32_t slab_missed;
	uint32_t record_data;
	uint32_t bytes_got;
	uint8_t bypass_iir_filter;
	void *queue_data[MAX_QUEUE_LEN];
	uint16_t data[MAX_NUM_CHANNELS * MAX_DATA_ITEMS];

	#ifdef CONFIG_ALIF_PDM_USE_DMA
	uint8_t __aligned(4) dma_mcode[PDM_DMA_MCODE_SIZE];
	#endif

};

struct pdm_config {
	DEVICE_MMIO_ROM;
	void (*irq_config)(void);
	uint32_t fifo_watermark;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clk_dev;
	clock_control_subsys_t clkid;

	#ifdef CONFIG_ALIF_PDM_USE_DMA
	const struct device *dma_dev;
	const struct device *pl330_dev;
	uint8_t periph_id;
	uint32_t dma_channel;
	#endif
	bool dma_enabled;

};

/**
 * @fn		void *get_slab(struct pdata *pdm_data)
 * @brief	Allocates a memory block from the slab for PCM data.
 * @param[in]	pdata Pointer to the PDM data structure
 *			containing the memory slab.
 * @return		Pointer to the allocated memory block on
 *			Zero on success, and a negative value on failure.
 */
static void *get_slab(struct pdm_data *pdata)
{
	int rc;
	void *buffer;

	rc = k_mem_slab_alloc(pdata->mem_slab, &buffer, K_NO_WAIT);

	if (rc == 0) {
		LOG_DBG("Memory block allocated : %p\n", buffer);
	} else {
		pdata->slab_missed++;
		return NULL;
	}

	return buffer;
}

#ifdef CONFIG_ALIF_PDM_USE_DMA
/**
 * @fn		static bool factorize_loop_counts(uint32_t n_bursts,
 *						  uint16_t *lc1, uint16_t *lc0)
 * @brief	Factorizes n_bursts into two loop counter values (lc1 x lc0)
 *		each fitting within the PL330 DMA loop counter range (1-256).
 * @param[in]	n_bursts : Total number of DMA bursts to factorize.
 * @param[out]	lc1      : Outer loop counter value.
 * @param[out]	lc0      : Inner loop counter value.
 * @return	true if factorization succeeded, false if not possible.
 */
static bool factorize_loop_counts(uint32_t n_bursts,
		uint16_t *lc1, uint16_t *lc0)
{
	if (n_bursts == 0) {
		return false;
	}
	if (n_bursts <= 256) {
		*lc1 = 1;
		*lc0 = n_bursts;
		return true;
	}
	for (uint16_t outer = 2; outer <= 256; outer++) {
		if (n_bursts % outer == 0) {
			uint32_t inner = n_bursts / outer;

			if (inner <= 256) {
				*lc1 = outer;
				*lc0 = (uint16_t)inner;
				return true;
			}
		}
	}
	return false;
}

/**
 * @fn		static bool pdm_build_mcode(const struct device *dev,
 *					    struct pdm_data *pdata,
 *					    uintptr_t reg_base,
 *					    uint8_t *dst,
 *					    uint32_t n_samples,
 *					    uint8_t periph_id,
 *					    uint8_t dma_channel,
 *					    uint8_t channel_map)
 * @brief	Builds the PL330 DMA microcode sequence to transfer audio
 *		samples from PDM channel pair registers to a destination
 *		buffer. Waits for FIFO DRQ on the first active pair and
 *		reads all active pairs per sample iteration.
 * @param[in]	dev         : Pointer to the runtime device structure.
 * @param[in]	pdata       : Pointer to the PDM driver data structure.
 * @param[in]	reg_base    : MMIO base address of the PDM peripheral.
 * @param[in]	dst         : Pointer to the destination PCM data buffer.
 * @param[in]	n_samples   : Number of sample iterations (bursts) to transfer.
 * @param[in]	periph_id   : DMA peripheral request ID for PDM FIFO.
 * @param[in]	dma_channel : DMA channel number used to send completion event.
 * @param[in]	channel_map : Bitmask of active PDM channels.
 * @return	true on success, false if mcode buffer overflows or
 *		no active channel pairs found.
 */
static bool pdm_build_mcode(const struct device *dev,
		struct pdm_data *pdata,
		uintptr_t reg_base,
		uint8_t  *dst,
		uint32_t  n_samples,
		uint8_t   periph_id,
		uint8_t   dma_channel,
		uint8_t   channel_map)
{
	struct dma_pl330_opcode_buf ob = {
		.buf      = pdata->dma_mcode,
		.off      = 0,
		.buf_size = PDM_DMA_MCODE_SIZE,
	};
	const uint32_t ch_pair_offsets[] = {
		PDM_CH0_CH1_AUDIO_OUT,
		PDM_CH2_CH3_AUDIO_OUT,
		PDM_CH4_CH5_AUDIO_OUT,
		PDM_CH6_CH7_AUDIO_OUT,
	};
	const uint8_t ch_pair_masks[] = {
		PDM_CHANNEL_0 | PDM_CHANNEL_1,
		PDM_CHANNEL_2 | PDM_CHANNEL_3,
		PDM_CHANNEL_4 | PDM_CHANNEL_5,
		PDM_CHANNEL_6 | PDM_CHANNEL_7,
	};

	/* find first active pair — waits for FIFO request */
	int first_active_pair = -1;

	for (int i = 0; i < 4; i++) {
		if (channel_map & ch_pair_masks[i]) {
			first_active_pair = i;
			break;
		}
	}

	if (first_active_pair == -1) {
		LOG_ERR("PDM: no active channel pairs in channel_map");
		return false;
	}

	union dma_pl330_ccr ccr = { .value = 0 };

	ccr.b.src_inc        = 0;
	ccr.b.dst_inc        = 1;
	ccr.b.src_burst_size = 2;
	ccr.b.dst_burst_size = 2;
	ccr.b.src_burst_len  = 0;
	ccr.b.dst_burst_len  = 0;
	ccr.b.src_cache_ctrl = 0;
	ccr.b.dst_cache_ctrl = 2;
	ccr.b.dst_prot_ctrl  = 0x0;
	ccr.b.src_prot_ctrl  = 0x0;
	ccr.b.endian_swap    = 0x0;

	bool ret = true;

	ret &= dma_pl330_gen_move(ccr.value, DMA_PL330_REG_CCR, &ob);
	ret &= dma_pl330_gen_move(
			(uint32_t)local_to_global(dst),
			DMA_PL330_REG_DAR, &ob);

	uint16_t lc0, lc1;

	if (!factorize_loop_counts(n_samples, &lc1, &lc0)) {
		LOG_ERR("PDM: n_samples %u cannot be factored", n_samples);
		return false;
	}

	ret &= dma_pl330_gen_loop(DMA_PL330_LC_1, lc1, &ob);
	uint32_t outer_start = ob.off;

	ret &= dma_pl330_gen_loop(DMA_PL330_LC_0, lc0, &ob);
	uint32_t inner_start = ob.off;

	for (int pair = 0; pair < 4; pair++) {
		if (!(channel_map & ch_pair_masks[pair])) {
			continue;
		}

		/* point SAR at this pair's register */
		ret &= dma_pl330_gen_move(
				(uint32_t)(reg_base + ch_pair_offsets[pair]),
				DMA_PL330_REG_SAR, &ob);

		if (pair == first_active_pair) {
			/* wait for FIFO request ONCE per sample iteration */
			ret &= dma_pl330_gen_flushperiph(periph_id, &ob);
			ret &= dma_pl330_gen_wfp(DMA_PL330_XFER_SINGLE, periph_id, &ob);
			ret &= dma_pl330_gen_loadperiph(DMA_PL330_XFER_SINGLE, periph_id, &ob);
		} else {
			/* subsequent pairs — data already available, no wait */
			ret &= dma_pl330_gen_loadperiph(DMA_PL330_XFER_SINGLE, periph_id, &ob);
		}
		ret &= dma_pl330_gen_store(DMA_PL330_XFER_FORCE, &ob);
	}

	/* close inner loop — PL330 jump field is 8-bit; */
	uint32_t inner_jump = ob.off - inner_start;

	if (inner_jump > UINT8_MAX) {
		LOG_ERR("PDM mcode: inner loop body too large (%u bytes, max 255)",
			inner_jump);
		return false;
	}

	struct dma_pl330_loop lp0 = {
		.xfer_type = DMA_PL330_XFER_SINGLE,
		.lc        = DMA_PL330_LC_0,
		.jump      = (uint8_t)inner_jump,
		.nf        = true,
	};
	ret &= dma_pl330_gen_loopend(&lp0, &ob);

	/* close outer loop — re-measure after lp0 was written */
	uint32_t outer_jump = ob.off - outer_start;

	if (outer_jump > UINT8_MAX) {
		LOG_ERR("PDM mcode: outer loop body too large (%u bytes, max 255)",
			outer_jump);
		return false;
	}

	struct dma_pl330_loop lp1 = {
		.xfer_type = DMA_PL330_XFER_SINGLE,
		.lc        = DMA_PL330_LC_1,
		.jump      = (uint8_t)outer_jump,
		.nf        = true,
	};
	ret &= dma_pl330_gen_loopend(&lp1, &ob);

	ret &= dma_pl330_gen_wmb(&ob);
	ret &= dma_pl330_gen_send_event(dma_channel, &ob);
	ret &= dma_pl330_gen_end(&ob);

	if (!ret) {
		LOG_ERR("PDM mcode overflow: increase PDM_DMA_MCODE_SIZE");
		return false;
	}
	const struct pdm_config *cfg = DEV_CFG(dev);

	if (!cfg->pl330_dev) {
		LOG_ERR("PDM: PL330 device not bound");
	return false;
	}

		return dma_pl330_start_with_mcode(cfg->pl330_dev,
				dma_channel,
				pdata->dma_mcode,
				PDM_DMA_MCODE_SIZE) == 0;
}
struct pdm_pair_info {
	uint8_t has_low;
	uint8_t has_high;
};

/**
 * @fn		static void build_pair_info(uint8_t channel_map,
 *					    struct pdm_pair_info *pairs,
 *					    uint32_t *num_active_pairs)
 * @brief	Scans the channel_map and fills the pairs array with
 *		has_low and has_high flags for each active PDM pair.
 *		Also sets num_active_pairs to the count of active pairs.
 * @param[in]	channel_map      : Bitmask of active PDM channels.
 * @param[out]	pairs            : Array to store active pair info.
 * @param[out]	num_active_pairs : Number of active pairs found.
 * @return	None.
 */
static void build_pair_info(uint8_t channel_map,
				struct pdm_pair_info *pairs,
				uint32_t *num_active_pairs)
{
	const uint8_t low_masks[] = {
		PDM_CHANNEL_0, PDM_CHANNEL_2,
		PDM_CHANNEL_4, PDM_CHANNEL_6
	};
	const uint8_t high_masks[] = {
		PDM_CHANNEL_1, PDM_CHANNEL_3,
		PDM_CHANNEL_5, PDM_CHANNEL_7
	};

	*num_active_pairs = 0;

	for (int p = 0; p < 4; p++) {
		uint8_t has_low  = !!(channel_map & low_masks[p]);
		uint8_t has_high = !!(channel_map & high_masks[p]);

		if (has_low || has_high) {
			pairs[*num_active_pairs].has_low  = has_low;
			pairs[*num_active_pairs].has_high = has_high;
			(*num_active_pairs)++;
		}
	}
}

/**
 * @fn		static void compact_to_actual_channels(uint8_t *buf,
 *						       uint8_t channel_map,
 *						       uint32_t num_channels,
 *						       uint32_t n_bursts)
 * @brief	For odd channel counts, compacts the DMA output buffer
 *		in-place. DMA always reads full 32-bit pair registers,
 *		so for odd channel counts the last pair contains one
 *		valid 16-bit sample and one garbage 16-bit sample.
 *		This function extracts only the valid samples.
 *		For even channel counts, all pairs are fully populated
 *		so no compaction is needed.
 * @param[in]	buf          : Pointer to the DMA output buffer.
 * @param[in]	channel_map  : Bitmask of active PDM channels.
 * @param[in]	num_channels : Total number of active channels.
 * @param[in]	n_bursts     : Number of DMA bursts completed.
 * @return	None.
 */
static void compact_to_actual_channels(uint8_t *buf,
					uint8_t channel_map,
					uint32_t num_channels,
					uint32_t n_bursts)
{
	struct pdm_pair_info pairs[4];
	uint32_t num_active_pairs = 0;

	build_pair_info(channel_map, pairs, &num_active_pairs);
	bool all_full = true;

	for (uint32_t p = 0; p < num_active_pairs; p++) {
		if (!pairs[p].has_low || !pairs[p].has_high) {
			all_full = false;
			break;
		}
	}

	if (all_full) {
		return;
	}

	uint32_t *src = (uint32_t *)buf;
	uint16_t *dst = (uint16_t *)buf;
	uint32_t out_idx = 0;

	for (uint32_t s = 0; s < n_bursts; s++) {
		for (uint32_t p = 0; p < num_active_pairs; p++) {
			uint32_t word = src[s * num_active_pairs + p];

			if (pairs[p].has_low) {
				dst[out_idx++] = (uint16_t)(word & 0xFFFF);
			}
			if (pairs[p].has_high) {
				dst[out_idx++] = (uint16_t)(word >> 16);
			}
		}
	}
}

/**
 * @fn		static void pdm_dma_callback(const struct device *dma_dev,
 *					     void *user_data,
 *					     uint32_t channel,
 *					     int status)
 * @brief	DMA completion callback for PDM audio capture. Compacts the
 *		buffer if channel count is odd, enqueues the completed PCM
 *		buffer, allocates the next buffer from the slab, rebuilds
 *		the DMA microcode, and restarts the DMA transfer.
 * @param[in]	dma_dev   : Pointer to the DMA controller device.
 * @param[in]	user_data : Pointer to the PDM device passed at DMA config time.
 * @param[in]	channel   : DMA channel number that completed.
 * @param[in]	status    : Completion status. Non-zero indicates an error.
 * @return	None.
 */
static void pdm_dma_callback(const struct device *dma_dev, void *user_data,
		uint32_t channel, int status)
{
	const struct device     *dev      = user_data;
	struct pdm_data         *pdata    = DEV_DATA(dev);
	const struct pdm_config *cfg      = DEV_CFG(dev);
	uintptr_t                reg_base = DEVICE_MMIO_GET(dev);

	if (status != 0 || !pdata->record_data) {
		return;
	}

	if (pdata->num_channels == 0) {
		LOG_ERR("PDM callback: no active channels");
		return;
	}

	uint32_t n_samples = pdata->block_size / (pdata->num_channels * sizeof(uint16_t));
	uint32_t n_bursts  = n_samples;

	if (pdata->data_buffer) {
		/*
		 * compact in-place before enqueue:
		 * for odd channel counts, last pair has 1 garbage
		 * 16-bit sample that must be stripped out.
		 * for even channel counts this returns immediately.
		 */
		compact_to_actual_channels(pdata->data_buffer,
					   pdata->channel_map,
					   pdata->num_channels,
					   n_bursts);
		int q_ret = k_msgq_put(&pdata->buf_queue,
				       &pdata->data_buffer, K_NO_WAIT);
		if (q_ret != 0) {
			/* Queue full — free buffer to avoid slab leak */
			LOG_WRN("PDM: queue full, dropping buffer");
			k_mem_slab_free(pdata->mem_slab, pdata->data_buffer);
			pdata->data_buffer = NULL;
		}
	}

	/* allocate next buffer */
	pdata->data_buffer = get_slab(pdata);
	if (!pdata->data_buffer) {
		return;
	}

	if (!pdm_build_mcode(dev, pdata, reg_base,
			pdata->data_buffer, n_bursts,
			cfg->periph_id, ALIF_DMA_DECODE_CHANNEL(cfg->dma_channel),
			pdata->channel_map)) {
		LOG_ERR("PDM mcode build failed in callback");
		k_mem_slab_free(pdata->mem_slab, pdata->data_buffer);
		pdata->data_buffer = NULL;
		return;
	}

}
#endif

/**
 * @fn		int dmic_alif_pdm_configure(const struct device *dev,
 *						struct dmic_cfg *config)
 * @brief	Configures requested number of channels, block size and
 *			enable the PDM  channels etc.
 * @param[in]   dev	: pointer to Runtime device structure
 * @param[in]   config  : Pointer to the dmic_cfg structure which contains
 *						  the input configuration.
 * @return	  Zero on success, and a negative value on failure.
 */
static int dmic_alif_pdm_configure(const struct device *dev, struct dmic_cfg *config)
{
	struct pdm_data *pdata = DEV_DATA(dev);
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t reg_val = sys_read32(reg_base + PDM_CONFIG_REGISTER);

	if (config->channel.req_num_chan == 0 || config->channel.req_num_chan > MAX_NUM_CHANNELS) {
		LOG_DBG("config invalid: number of channels not valid\n");
		return -EINVAL;
	}

	if (pdata) {
		pdata->mem_slab = config->streams[0].mem_slab;
		pdata->block_size = config->streams[0].block_size;
		pdata->channel_map = config->channel.req_chan_map_lo & 0xFF;

		reg_val |= pdata->channel_map;

		pdata->num_channels = config->channel.req_num_chan;

		LOG_DBG("block size: %d\n", pdata->block_size);
	}

	LOG_DBG("DMIC configure okay\n");

	return 0;
}

/**
 * @fn		void pdm_channel_config(const struct device *dev,
 *					struct pdm_ch_config *cnfg)
 * @brief	Sets FIR coefficient and IIR coefficient values.
 * @param[in]	dev  : Pointer to the runtime device structure.
 * @param[in]	cnfg : Pointer to the pdm_ch_config structure.
 * @return	    None
 */
void pdm_channel_config(const struct device *dev, struct pdm_ch_config *cnfg)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint8_t i;
	uint32_t *ch_n_fir_coef_0 =
		(uint32_t *)(reg_base + PDM_CH_FIR_COEF + (cnfg->ch_num * PDM_CH_OFFSET));

	/* Store the FIR coefficient values */
	for (i = 0; i < PDM_MAX_FIR_COEFFICIENT; i++) {
		*(ch_n_fir_coef_0) = cnfg->ch_fir_coef[i];
		ch_n_fir_coef_0++;
	}

	uintptr_t ch_n_iir_coef = (reg_base + PDM_CH_IIR_COEF_SEL + (cnfg->ch_num * PDM_CH_OFFSET));

	/* Store the IIR coefficient values */
	sys_write32(cnfg->ch_iir_coef, ch_n_iir_coef);
}

/**
 * @fn		void pdm_set_ch_phase(const struct device *dev,
 *					uint8_t ch_num,
 *					uint32_t ch_phase)
 * @brief	Sets the PDM channel phase control value
 * @param[in]	dev  : Pointer to the runtime device structure.
 * @param[in]	ch_num : PDM channel number.
 * @param[in]	ch_phase : PDM channel phase control value.
 * @return	    None
 */
void pdm_set_ch_phase(const struct device *dev, uint8_t ch_num, uint32_t ch_phase)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uintptr_t ch_n_phase = (reg_base + PDM_CH_PHASE + (ch_num * PDM_CH_OFFSET));

	sys_write32(ch_phase, ch_n_phase);
}

/**
 * @fn		void pdm_set_ch_gain(const struct device *dev,
 *					uint8_t ch_num,
 *					uint32_t ch_gain)
 * @brief	Sets the PDM channel gain control value
 * @param[in]	dev	: Pointer to the runtime device structure.
 * @param[in]	ch_num	: PDM channel number.
 * @param[in]	ch_gain	: PDM channel gain control value.
 * @return	    None
 */
void pdm_set_ch_gain(const struct device *dev, uint8_t ch_num, uint32_t ch_gain)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uintptr_t ch_n_gain = (reg_base + PDM_CH_GAIN + (ch_num * PDM_CH_OFFSET));

	sys_write32(ch_gain, ch_n_gain);
}

/**
 * @fn		void pdm_set_peak_detect_th(const struct device *dev,
 *						uint8_t ch_num,
 *						uint32_t ch_peak_detect_th)
 * @brief	Sets the PDM channel  Peak detector threshold value
 * @param[in]	dev	: Pointer to the runtime device structure.
 * @param[in]	ch_num	: PDM channel number.
 * @param[in]	ch_peak_detect_th : PDM channel  Peak detector
 *				threshold value.
 * @return		None
 */
void pdm_set_peak_detect_th(const struct device *dev, uint8_t ch_num, uint32_t ch_peak_detect_th)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uintptr_t ch_n_pkdet_th = (reg_base + PDM_CH_PKDET_TH + (ch_num * PDM_CH_OFFSET));

	sys_write32(ch_peak_detect_th, ch_n_pkdet_th);
}

/**
 * @fn		void pdm_set_peak_detect_itv(const struct device *dev,
 *						uint8_t ch_num,
 *						uint32_t ch_peak_detect_itv)
 * @brief	Sets the PDM channel  Peak detector interval value
 * @param[in]	dev	: Pointer to the runtime device structure.
 * @param[in]	ch_num	: PDM channel number.
 * @param[in]	ch_peak_detect_itv : PDM channel  Peak detector
 *				interval value.
 * @return		None
 */
void pdm_set_peak_detect_itv(const struct device *dev, uint8_t ch_num, uint32_t ch_peak_detect_itv)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uintptr_t ch_n_pkdet_itv = (reg_base + PDM_CH_PKDET_ITV + (ch_num * PDM_CH_OFFSET));

	sys_write32(ch_peak_detect_itv, ch_n_pkdet_itv);
}

/**
 * @fn		void pdm_mode(const struct device *dev, uint8_t mode)
 * @brief	Sets the PDM modes
 * @param[in]	dev	: Pointer to the runtime device structure.
 * @param[in]	mode	: pdm frequency modes
 * @return		None
 */
void pdm_mode(const struct device *dev, uint8_t mode)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	uint32_t reg_val = sys_read32(reg_base + PDM_CONFIG_REGISTER);

	reg_val |= (mode << PDM_CLK_MODE);

	sys_write32(reg_val, reg_base + PDM_CONFIG_REGISTER);
}

/**
 * @fn		void enable_interrupt(const struct device *dev)
 * @brief		Enable the IRQ
 * @param[in]	dev : Pointer to the runtime device structure.
 * @return	None
 */
static void enable_interrupt(const struct device *dev)
{
	const struct pdm_config *cfg = DEV_CFG(dev);
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t irq_value;

	if (cfg->dma_enabled) {
		/* DMA mode: only need overflow for error reporting. */
		irq_value = PDM_FIFO_OVERFLOW_IRQ;
	} else {
		/* ISR mode: almost-full drives FIFO drain; plus per-channel
		 * audio-detect bits and overflow.
		 */
		uint32_t audio_ch =
			sys_read32(reg_base + PDM_CONFIG_REGISTER) & PDM_CHANNEL_ENABLE;

		irq_value = (audio_ch << 8) | PDM_FIFO_ALMOST_FULL_IRQ |
			    PDM_FIFO_OVERFLOW_IRQ;
	}

	sys_write32(irq_value, reg_base + PDM_INTERRUPT_REGISTER);
}

/**
 * @fn		void disable_interrupt(const struct device *dev)
 * @brief		Disable the IRQ
 * @param[in]	dev : Pointer to the runtime device structure.
 * @return	None
 */
static void disable_interrupt(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	/* Disable the Interrupt */
	sys_write32(0, reg_base + PDM_INTERRUPT_REGISTER);
}

/**
 * @fn		static int dmic_alif_pdm_trigger(const struct device *dev,
 *					enum dmic_trigger cmd)
 * @brief	Starts or stops PDM audio capture.
 * @param[in]   dev	: pointer to Runtime device structure
 * @param[in]   cmd	: DMIC start or stop command
 * @return	  Zero on success, and a negative value on failure.
 */
static int dmic_alif_pdm_trigger(const struct device *dev, enum dmic_trigger cmd)
{
	struct pdm_data *pdata = DEV_DATA(dev);
	const struct pdm_config *cfg = DEV_CFG(dev);
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t reg_val;

	switch (cmd) {
	case DMIC_TRIGGER_STOP:
		if (cfg->dma_enabled) {
#ifdef CONFIG_ALIF_PDM_USE_DMA
			dma_stop(cfg->dma_dev, ALIF_DMA_DECODE_CHANNEL(cfg->dma_channel));
#endif
		}
		disable_interrupt(dev);
		pdata->record_data = 0;

		reg_val = sys_read32(reg_base + PDM_CONFIG_REGISTER);
		reg_val &= ~PDM_CHANNEL_ENABLE;
		sys_write32(reg_val, reg_base + PDM_CONFIG_REGISTER);

		/* Free in-progress buffer to prevent slab leak */
		if (pdata->data_buffer != NULL) {
			k_mem_slab_free(pdata->mem_slab, pdata->data_buffer);
			pdata->data_buffer = NULL;
		}

		/* Drain queued buffers that the app hasn't read */
		void *buf;

		while (k_msgq_get(&pdata->buf_queue, &buf, K_NO_WAIT) == 0) {
			k_mem_slab_free(pdata->mem_slab, buf);
		}

		pm_device_busy_clear(dev);
		break;

	case DMIC_TRIGGER_START:

		pdata->record_data = 1;
		pdata->bytes_got = 0;

		if (cfg->dma_enabled) {
#ifdef CONFIG_ALIF_PDM_USE_DMA
			pdata->data_buffer = get_slab(pdata);
			if (!pdata->data_buffer) {
				return -ENOMEM;
			}

			if (pdata->num_channels == 0) {
				LOG_ERR("PDM: no active channels");
				return -EINVAL;
			}

			uint32_t n_samples = pdata->block_size /
					(pdata->num_channels * sizeof(uint16_t));
			uint32_t n_bursts  = n_samples;


			/* configure DMA */
			struct dma_block_config dummy_block = {0};
			struct dma_config dma_cfg = {0};

			dma_cfg.channel_direction   = PERIPHERAL_TO_MEMORY;
			dma_cfg.dma_callback        = pdm_dma_callback;
			dma_cfg.user_data           = (void *)dev;
			dma_cfg.source_data_size    = 4;
			dma_cfg.dest_data_size      = 4;
			dma_cfg.source_burst_length = 1;
			dma_cfg.dest_burst_length   = 1;
			dma_cfg.block_count         = 1;
			dma_cfg.head_block          = &dummy_block;
			dma_cfg.dma_slot            = cfg->periph_id;

			int ret = dma_config(cfg->dma_dev, cfg->dma_channel, &dma_cfg);

			if (ret) {
				LOG_ERR("dma_config failed: %d", ret);
				return ret;
			}

			/* build microcode */
			if (!pdm_build_mcode(dev, pdata, reg_base,
					     pdata->data_buffer, n_bursts,
					     cfg->periph_id,
					     ALIF_DMA_DECODE_CHANNEL(cfg->dma_channel),
					     pdata->channel_map)) {
				return -ENOBUFS;
			}

			/* enable PDM channels AFTER DMA is ready */
			reg_val = sys_read32(reg_base + PDM_CONFIG_REGISTER);
			reg_val |= pdata->channel_map;
			sys_write32(reg_val, reg_base + PDM_CONFIG_REGISTER);

			enable_interrupt(dev);
			pm_device_busy_set(dev);
#endif
		} else {
			/* ---- ISR path  ---- */
			pdata->buf_index   = 0;
			pdata->data_buffer = NULL;
			pdata->bytes_got   = 0;

			reg_val = sys_read32(reg_base + PDM_CONFIG_REGISTER);
			reg_val |= pdata->channel_map;
			sys_write32(reg_val, reg_base + PDM_CONFIG_REGISTER);

			enable_interrupt(dev);
			pm_device_busy_set(dev);
		}
		break;

	default:
		LOG_ERR("Invalid command: %d", cmd);
		return -EINVAL;
	}
	return 0;
}

/**
 * @fn		int dmic_alif_pdm_read(const struct device *dev,
 *					uint8_t stream,
 *					void **buffer, size_t *size,
 *					int32_t timeout)
 * @brief	Read the stored allocated block address in msg queue
 *			get the pcm samples.
 * @param[in]	dev	: pointer to Runtime device structure
 * @param[in]	stream	: stream configuration
 * @param[in]	buffer	: A pointer to the buffer where the
 *			  retrieved message will be copied.
 * @param[in]	size	: Size of the allocated block
 * @param[in]	timeout	: Maximum time to wait for a message
 * @return		Zero on success, and a negative value on failure.
 */
static int dmic_alif_pdm_read(const struct device *dev, uint8_t stream, void **buffer, size_t *size,
			      int32_t timeout)
{
	struct pdm_data *pdata = DEV_DATA(dev);
	int rc;

	rc = k_msgq_get(&pdata->buf_queue, buffer, SYS_TIMEOUT_MS(timeout));

	if (rc != 0) {
		LOG_DBG("No audio data to be read\n");
	} else {
		*size = pdata->block_size;
	}
	return rc;
}

static inline void pdm_error_handler(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_clear_bits(reg_base + PDM_INTERRUPT_REGISTER, PDM_FIFO_OVERFLOW_IRQ);
	(void)sys_read32(reg_base + PDM_ERROR_IRQ);
}

static inline void pdm_audio_det_handler(const struct device *dev)
{
	struct pdm_data *pdata = DEV_DATA(dev);
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	if (pdata->slab_missed != 0) {
		sys_clear_bits(reg_base + PDM_INTERRUPT_REGISTER, PDM_AUDIO_DETECT_IRQ_STAT);
	}
	(void)sys_read32(reg_base + PDM_AUDIO_DETECT_IRQ);
}
/**
 * @fn		static void pdm_error_detect_irq_handler()
 * @brief	ISR to handle the error interrupt
 * @param[in]	None
 * @return	None.
 */
static __maybe_unused void pdm_error_detect_irq_handler(const struct device *dev)
{
	pdm_error_handler(dev);
}

/**
 * @fn		static void pdm_audio_detect_irq_handler(const struct device *dev)
 * @brief	ISR to handle PDM audio detect interrupts.
 * @param[in]	dev	: pointer to Runtime device structure
 * @return	None.
 */
static __maybe_unused void pdm_audio_detect_irq_handler(const struct device *dev)
{
	pdm_audio_det_handler(dev);
}

/**
 * @fn		static void alif_pdm_warning_isr()
 * @brief	ISR to handle PDM warning interrupts.
 *			Collects audio data from the PDM channels, stores it
 *			in the buffer, and handles memory allocation and queue
 *			management.
 * @param[in]	dev	: pointer to Runtime device structure
 * @return	None.
 */
static void alif_pdm_warning_isr(const struct device *dev)
{
	struct pdm_data *pdmdata = DEV_DATA(dev);
	uint8_t k = 0;
	uint8_t audio_ch;
	uint8_t intstatus;
	uintptr_t reg_base;
	uint32_t num_items;
	uint32_t data_bytes;
	uint32_t block_size;
	uint32_t bytes_available;
	uint32_t i;
	uint32_t whole;
	uint32_t audio_ch_0_1;
	uint32_t audio_ch_2_3;
	uint32_t audio_ch_4_5;
	uint32_t audio_ch_6_7;

	block_size = pdmdata->block_size;

	reg_base = DEVICE_MMIO_GET(dev);

	uint32_t reg_val = sys_read32(reg_base + PDM_CONFIG_REGISTER);

	/* User enabled channel */
	audio_ch = reg_val & PDM_CHANNEL_ENABLE;

	intstatus = sys_read32(reg_base + PDM_WARN_IRQ);
	num_items = sys_read32(reg_base + PDM_FIFO_STATUS_REGISTER);

	/* LPPDM doesn't have separate error and audio detect isr handlers */
	if (!DT_NODE_HAS_PROP(DT_NODELABEL(dev), error_intr)) {
		pdm_error_handler(dev);
	}

	if (!DT_NODE_HAS_PROP(DT_NODELABEL(dev), audio_det_intr)) {
		pdm_audio_det_handler(dev);
	}

	for (i = 0; i < num_items; i++) {
		audio_ch_0_1 = sys_read32(reg_base + PDM_CH0_CH1_AUDIO_OUT);
		audio_ch_2_3 = sys_read32(reg_base + PDM_CH2_CH3_AUDIO_OUT);
		audio_ch_4_5 = sys_read32(reg_base + PDM_CH4_CH5_AUDIO_OUT);
		audio_ch_6_7 = sys_read32(reg_base + PDM_CH6_CH7_AUDIO_OUT);

		if ((audio_ch & PDM_CHANNEL_0) == PDM_CHANNEL_0) {
			pdmdata->data[k++] = (uint16_t)(audio_ch_0_1);
		}
		if ((audio_ch & PDM_CHANNEL_1) == PDM_CHANNEL_1) {
			pdmdata->data[k++] = (uint16_t)(audio_ch_0_1 >> 16);
		}
		if ((audio_ch & PDM_CHANNEL_2) == PDM_CHANNEL_2) {
			pdmdata->data[k++] = (uint16_t)(audio_ch_2_3);
		}
		if ((audio_ch & PDM_CHANNEL_3) == PDM_CHANNEL_3) {
			pdmdata->data[k++] = (uint16_t)(audio_ch_2_3 >> 16);
		}
		if ((audio_ch & PDM_CHANNEL_4) == PDM_CHANNEL_4) {
			pdmdata->data[k++] = (uint16_t)(audio_ch_4_5);
		}
		if ((audio_ch & PDM_CHANNEL_5) == PDM_CHANNEL_5) {
			pdmdata->data[k++] = (uint16_t)(audio_ch_4_5 >> 16);
		}
		if ((audio_ch & PDM_CHANNEL_6) == PDM_CHANNEL_6) {
			pdmdata->data[k++] = (uint16_t)(audio_ch_6_7);
		}
		if ((audio_ch & PDM_CHANNEL_7) == PDM_CHANNEL_7) {
			pdmdata->data[k++] = (uint16_t)(audio_ch_6_7 >> 16);
		}
	}

	if (pdmdata->record_data == 0) {
		return;
	}

	data_bytes = num_items * pdmdata->num_channels * sizeof(unsigned short);

	pdmdata->bytes_got += data_bytes;

	if (pdmdata->data_buffer == NULL) {

		pdmdata->data_buffer = get_slab(pdmdata);
		if (pdmdata->data_buffer == NULL) {
			sys_write32(0, reg_base + PDM_INTERRUPT_REGISTER);
			return;
		}
		pdmdata->buf_index = 0;
	}

	bytes_available = block_size - pdmdata->buf_index;

	if (bytes_available >= data_bytes) {
		memcpy((pdmdata->data_buffer + pdmdata->buf_index), pdmdata->data, data_bytes);
		pdmdata->buf_index += data_bytes;
	} else {
		if (bytes_available > 0) {
			memcpy((pdmdata->data_buffer + pdmdata->buf_index), pdmdata->data,
			       bytes_available);
		}
		whole = data_bytes - bytes_available;

		if (k_msgq_put(&pdmdata->buf_queue, &pdmdata->data_buffer, K_NO_WAIT) != 0) {
			/* Queue full: drop oldest block to make room */
			void *oldest = NULL;

			if (k_msgq_get(&pdmdata->buf_queue, &oldest, K_NO_WAIT) == 0) {
				k_mem_slab_free(pdmdata->mem_slab, oldest);
				k_msgq_put(&pdmdata->buf_queue, &pdmdata->data_buffer, K_NO_WAIT);
			}
		}

		pdmdata->data_buffer = get_slab(pdmdata);

		if (pdmdata->data_buffer) {
			memcpy(pdmdata->data_buffer, pdmdata->data + (bytes_available / 2), whole);
			pdmdata->buf_index = whole;
		} else {
			pdmdata->buf_index = 0;
		}
	}
}

/* Init function */
static int pdm_initialize(const struct device *dev)
{
	const struct pdm_config *cfg = DEV_CFG(dev);
	struct pdm_data *pdata = DEV_DATA(dev);
	int32_t ret = 0;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	if (cfg->pcfg != NULL) {
		pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	}

	/* check device availability */
	if (!device_is_ready(cfg->clk_dev)) {
		LOG_ERR("clock controller device not ready");
		return -ENODEV;
	}

	/* Configure PDM clock sources */
	ret = clock_control_configure(cfg->clk_dev, cfg->clkid, NULL);
	if (ret != 0) {
		LOG_ERR("Unable to configure clock: err:%d", ret);
		return ret;
	}

	/* Enable PDM clock from clock manager */
	ret = clock_control_on(cfg->clk_dev, cfg->clkid);
	if (ret != 0) {
		LOG_ERR("Unable to turn on clock: err:%d", ret);
		return ret;
	}
#ifdef CONFIG_ALIF_PDM_USE_DMA
	if (cfg->dma_enabled) {
		if (!device_is_ready(cfg->dma_dev)) {
			LOG_ERR("Event router not ready");
			return -ENODEV;
		}
	}
#endif

	cfg->irq_config();

	k_msgq_init(&pdata->buf_queue, (char *)pdata->queue_data, sizeof(void *), MAX_QUEUE_LEN);

	/* Enable the Bypass IIR Filter */
	uint32_t regdata = pdata->bypass_iir_filter << PDM_BYPASS_IIR;


#ifdef CONFIG_ALIF_PDM_USE_DMA
	if (cfg->dma_enabled) {
		regdata |= PDM_USE_DMA_HANDSHAKE;   /* USE_DMA_HANDSHAKE */
	}
#endif
	sys_write32(regdata, reg_base + PDM_CTL_REGISTER);
	uint32_t watermark = cfg->dma_enabled ? 1U : cfg->fifo_watermark;

	sys_write32(watermark, reg_base + PDM_THRESHOLD_REGISTER);

	LOG_DBG("alif pdm driver init okay");

	return 0;
}

static const struct _dmic_ops dmic_alif_pdm_api = {
	.configure = dmic_alif_pdm_configure,
	.trigger = dmic_alif_pdm_trigger,
	.read = dmic_alif_pdm_read,
};

#if defined(CONFIG_PM_DEVICE)
static int pdm_suspend(const struct device *dev)
{
	const struct pdm_config *cfg = DEV_CFG(dev);
	int ret;

	disable_interrupt(dev);
	/* Disable PDM clock from clock manager */
	if (cfg->clk_dev != NULL) {
		ret = clock_control_off(cfg->clk_dev, cfg->clkid);
		if (ret != 0 && ret != -EALREADY) {
			LOG_ERR("Unable to turn off clock: err:%d", ret);
			return ret;
		}
	}

#if defined(CONFIG_PINCTRL)
	/* Apply sleep pin configuration if available */
	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_SLEEP);
		if (ret < 0 && ret != -ENOENT) {
			/* Ignore -ENOENT (sleep state not defined) */
			return ret;
		}
	}
#endif

	return 0;
}

static int pdm_resume(const struct device *dev)
{
	const struct pdm_config *cfg = DEV_CFG(dev);
	struct pdm_data *pdata = DEV_DATA(dev);
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	int ret = 0;

#if defined(CONFIG_PINCTRL)
	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("PDM pinctrl setup failed [%d]", ret);
			return ret;
		}
	}
#endif
	if (cfg->clk_dev) {
		/* Reconfigure PDM clock sources */
		ret = clock_control_configure(cfg->clk_dev, cfg->clkid, NULL);
		if (ret != 0 && ret != -ENOSYS && ret != -ENOTSUP) {
			LOG_ERR("Unable to configure clock: err:%d", ret);
			return ret;
		}

		/* Enable PDM clock from clock manager */
		ret = clock_control_on(cfg->clk_dev, cfg->clkid);
		if (ret != 0 && ret != -EALREADY) {
			LOG_ERR("Unable to turn on clock: err:%d", ret);
			return ret;
		}
	}
	/* Enable the Bypass IIR Filter */
	sys_write32(pdata->bypass_iir_filter << PDM_BYPASS_IIR, reg_base + PDM_CTL_REGISTER);

	sys_write32(cfg->fifo_watermark, reg_base + PDM_THRESHOLD_REGISTER);

	LOG_DBG("alif pdm driver resume okay");

	return 0;
}
/**
 * @brief PDM PM device action handler
 *
 * Handles power management state transitions for the PDM device.
 * Coordinates with power domain via PM framework.
 *
 * @param dev device struct
 * @param action PM device action
 *
 * @return 0 if successful, negative errno otherwise
 */
static int pdm_pm_action(const struct device *dev, enum pm_device_action action)
{
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		/* Save state and prepare for power down */
		ret = pdm_suspend(dev);
		break;
	case PM_DEVICE_ACTION_RESUME:
		/* Device is powered - restore state */
		ret = pdm_resume(dev);
		break;

	case PM_DEVICE_ACTION_TURN_OFF:
	case PM_DEVICE_ACTION_TURN_ON:
		/* Power domain handling is automatic via PM framework */
		return 0;

	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

/********** Device Definition per instance Macros **********/

#define PDM_INIT(n)                                                                                \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void pdm_irq_config_##n(void);                                                      \
	static struct pdm_data dmic_alif_pdm_data = {                                              \
		.bypass_iir_filter = DT_INST_PROP(n, bypass_iir_filter),                           \
	};                                                                                         \
	static const struct pdm_config dmic_alif_pdm_cfg_##n = {                                   \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.fifo_watermark = DT_INST_PROP(n, fifo_watermark),                                 \
		.irq_config = pdm_irq_config_##n,                                                  \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                  \
		.clkid = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, clkid),                    \
		IF_ENABLED(CONFIG_ALIF_PDM_USE_DMA, (                                              \
			.dma_dev = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas),                     \
				(DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, rxdma))),              \
				(NULL)),                                                           \
			.pl330_dev = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas),                   \
				(DEVICE_DT_GET(DT_PHANDLE(                                         \
					DT_INST_DMAS_CTLR_BY_NAME(n, rxdma),                       \
					dma_controller))),                                         \
				(NULL)),                                                           \
			.dma_channel = COND_CODE_1(                                                \
				DT_INST_NODE_HAS_PROP(n, dmas),                                    \
				(DT_INST_DMAS_CELL_BY_NAME(n, rxdma, channel)),                    \
				(0)),                                                              \
			.periph_id = COND_CODE_1(                                                  \
				DT_INST_NODE_HAS_PROP(n, dmas),                                    \
				(DT_INST_DMAS_CELL_BY_NAME(n, rxdma, periph)),                     \
				(0)),                                                              \
				))                                                                 \
		.dma_enabled = IS_ENABLED(CONFIG_ALIF_PDM_USE_DMA) &&                              \
			DT_INST_NODE_HAS_PROP(n, dmas),                                            \
};                                                                                                 \
	static void pdm_irq_config_##n(void)                                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, warning_intr, irq),                             \
			    DT_INST_IRQ_BY_NAME(n, warning_intr, priority), alif_pdm_warning_isr,  \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(n, warning_intr, irq));                             \
		IF_ENABLED(DT_INST_IRQ_HAS_NAME(n, error_intr),                                    \
			   (IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, error_intr, irq),                   \
					DT_INST_IRQ_BY_NAME(n, error_intr, priority),              \
					pdm_error_detect_irq_handler, DEVICE_DT_INST_GET(n), 0);   \
			    irq_enable(DT_INST_IRQ_BY_NAME(n, error_intr, irq));))                 \
		IF_ENABLED(DT_INST_IRQ_HAS_NAME(n, audio_det_intr),                                \
			   (IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, audio_det_intr, irq),               \
					DT_INST_IRQ_BY_NAME(n, audio_det_intr, priority),          \
					pdm_audio_detect_irq_handler, DEVICE_DT_INST_GET(n), 0);   \
			    irq_enable(DT_INST_IRQ_BY_NAME(n, audio_det_intr, irq));))             \
	}                                                                                          \
	PM_DEVICE_DT_INST_DEFINE(n, pdm_pm_action);                                                \
	DEVICE_DT_INST_DEFINE(n, pdm_initialize, PM_DEVICE_DT_INST_GET(n), &dmic_alif_pdm_data,    \
			      &dmic_alif_pdm_cfg_##n, POST_KERNEL,                                 \
			      CONFIG_AUDIO_DMIC_INIT_PRIORITY, &dmic_alif_pdm_api);

DT_INST_FOREACH_STATUS_OKAY(PDM_INIT)
