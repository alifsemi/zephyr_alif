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

LOG_MODULE_REGISTER(alif_pdm, LOG_LEVEL_INF);

#define DEV_DATA(dev) ((struct pdm_data *)((dev)->data))
#define DEV_CFG(dev)  ((const struct pdm_config *)((dev)->config))

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
	uint32_t overrun_count;
	uint8_t bypass_iir_filter;
	void *queue_data[MAX_QUEUE_LEN];
	uint16_t data[MAX_NUM_CHANNELS * MAX_DATA_ITEMS];
};

struct pdm_config {
	DEVICE_MMIO_ROM;
	void (*irq_config)(void);
	uint32_t fifo_watermark;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clk_dev;
	clock_control_subsys_t clkid;
	bool has_error_intr;
	bool has_audio_det_intr;
};

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

		/* Enable the PDM multiple channels */
		sys_write32(reg_val, reg_base + PDM_CONFIG_REGISTER);

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
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t irq_value = 0;
	uint32_t audio_ch = 0;

	uint32_t reg_val = sys_read32(reg_base + PDM_CONFIG_REGISTER);

	/* Store user enabled channel */
	audio_ch = reg_val & PDM_CHANNEL_ENABLE;

	irq_value |= (audio_ch << 8 | PDM_FIFO_ALMOST_FULL_IRQ | PDM_FIFO_OVERFLOW_IRQ);

	/* Enable the Interrupt */
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
 * @fn		int dmic_alif_pdm_trigger(const struct device *dev,
 *					enum dmic_trigger cmd)
 * @brief	Send DMIC_TRIGGER_STOP or DMIC_TRIGGER_START to
 *			perform the specific operation.
 * @param[in]   dev	: pointer to Runtime device structure
 * @param[in]   cmd	: DMIC start or stop command
 * @return	  Zero on success, and a negative value on failure.
 */
static int dmic_alif_pdm_trigger(const struct device *dev, enum dmic_trigger cmd)
{
	struct pdm_data *pdata = DEV_DATA(dev);

	switch (cmd) {
	case DMIC_TRIGGER_STOP:
		disable_interrupt(dev);
		pdata->record_data = 0;

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
		LOG_DBG("trigger start\n");
		pdata->record_data = 1;
		pdata->bytes_got = 0;
		pdata->buf_index = 0;
		pdata->data_buffer = NULL;
		pdata->slab_missed = 0;

		enable_interrupt(dev);

		pm_device_busy_set(dev);
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
	struct pdm_data *pdata = DEV_DATA(dev);
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t err = sys_read32(reg_base + PDM_ERROR_IRQ);

	if ((err & PDM_FIFO_OVERFLOW_STAT) != 0U) {
		/*
		 * Recover from a FIFO overflow instead of just masking it. If the
		 * warning IRQ is not serviced within the FIFO's (microsecond)
		 * budget the FIFO overflows; previously this handler permanently
		 * masked the overflow IRQ enable and performed no recovery, so
		 * overflow handling was a one-shot and samples arriving after a
		 * full FIFO were silently lost (HWRM 14.7.5.3: pre-overrun FIFO
		 * contents remain intact, new samples are dropped). Pulse FIFO_CLR
		 * (PDM_CTL0 bit 31) to restart from a clean FIFO and drop the
		 * in-flight partial block (its tail is discontinuous across the
		 * flush). The overflow IRQ enable is intentionally left set so
		 * later overflows recover too. Reading PDM_ERROR_IRQ above already
		 * acked the sticky status bit (read-to-clear).
		 *
		 * PDM_CTL0 writes need >= 4 APB transactions between them to
		 * synchronize into the audio clock domain (HWRM 14.7.5.3.1
		 * CAUTION); the dummy status reads provide that spacing so the
		 * clear pulse actually lands before it is deasserted.
		 */
		uint32_t cfg = sys_read32(reg_base + PDM_CONFIG_REGISTER);
		uint8_t spacer;

		sys_write32(cfg | PDM_FIFO_CLEAR, reg_base + PDM_CONFIG_REGISTER);
		for (spacer = 0U; spacer < 4U; spacer++) {
			(void)sys_read32(reg_base + PDM_FIFO_STATUS_REGISTER);
		}
		sys_write32(cfg & ~PDM_FIFO_CLEAR, reg_base + PDM_CONFIG_REGISTER);

		pdata->buf_index = 0;
		pdata->overrun_count++;
		LOG_WRN("fifo overflow — flushed and re-armed (count %u)",
			pdata->overrun_count);
	}
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
 * @fn		void *get_slab(struct pdm_data *pdm_data)
 * @brief	Allocates a memory block from the slab for PCM data.
 * @param[in]	pdm_data Pointer to the PDM data structure
 *			containing the memory slab.
 * @return		Pointer to the allocated memory block on
 *			Zero on success, and a negative value on failure.
 */
static void *get_slab(struct pdm_data *pdm_data)
{
	int rc;
	void *buffer;

	rc = k_mem_slab_alloc(pdm_data->mem_slab, &buffer, K_NO_WAIT);

	if (rc == 0) {
		LOG_DBG("Memory block allocated : %p\n", buffer);
	} else {
		pdm_data->slab_missed++;
		return NULL;
	}

	return buffer;
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

	/* Instances without dedicated error / audio-detect IRQ lines (e.g. the
	 * LPPDM) fold that handling into this warning ISR. has_error_intr /
	 * has_audio_det_intr are set per instance from DT_INST_IRQ_HAS_NAME so
	 * an instance that does wire dedicated lines (its own ISRs run) is not
	 * double-serviced here.
	 *
	 * Caveat: num_items is latched above BEFORE pdm_error_handler may flush
	 * the FIFO, so the drain below can read a just-emptied FIFO after an
	 * overflow recovery (one glitched block, recovered on the next IRQ).
	 */
	const struct pdm_config *cfg = DEV_CFG(dev);

	if (!cfg->has_error_intr) {
		pdm_error_handler(dev);
	}

	if (!cfg->has_audio_det_intr) {
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
			/*
			 * Slab exhausted (consumer momentarily starved). Previously
			 * this wrote 0 to PDM_INTERRUPT_REGISTER, permanently
			 * disabling every PDM interrupt: capture silently stopped
			 * forever after a single missed allocation and the stream
			 * appeared frozen until a full re-init. Instead, reclaim the
			 * oldest queued block (newest-audio-wins, matching the
			 * queue-full policy below); if none is queued, drop this
			 * window's samples — the FIFO was already drained above, so
			 * capture simply retries on the next warning IRQ.
			 */
			void *oldest = NULL;

			if (k_msgq_get(&pdmdata->buf_queue, &oldest, K_NO_WAIT) == 0) {
				k_mem_slab_free(pdmdata->mem_slab, oldest);
				pdmdata->data_buffer = get_slab(pdmdata);
			}
		}
		if (pdmdata->data_buffer == NULL) {
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

	/*
	 * Quiesce and soft-reset the block before configuring it. On a warm
	 * reset that does not reset this peripheral (e.g. the HE watchdog
	 * reset, which is CPU-subsystem-scoped), the PDM arrives here still
	 * running with its pre-reset CTL0 intact; observed on the E8 DK
	 * (2026-07-18) after a board-level partial reset landed mid-capture:
	 * the decimators kept free-running desynced and delivered a
	 * constant-DC sample stream that survived driver re-init, channel
	 * toggles, and FIFO clears. Clear the interrupt enables, pulse
	 * FIFO_CLR, drop every CHn_EN (the HWRM documents the 1->0 edge as
	 * "disable and soft-clear") together with the PDM_MODE field, ack
	 * the sticky status registers, then cycle the module clock gate
	 * (EXPMST0_CTRL[PDM_CKEN]) as belt-and-braces — gating freezes the
	 * audio-domain logic while the cleared CTL0 takes effect; the HWRM's
	 * PDM chapter does not document a per-block reset, so the CHn_EN
	 * soft-clear is the only architected state clear available here.
	 * CTL0 writes need >= 4 APB transactions of spacing to synchronize
	 * into the audio clock domain (HWRM 14.7.5.3.1 CAUTION).
	 */
	sys_write32(0U, reg_base + PDM_INTERRUPT_REGISTER);
	sys_write32(PDM_FIFO_CLEAR, reg_base + PDM_CONFIG_REGISTER);
	for (uint8_t spacer = 0U; spacer < 4U; spacer++) {
		(void)sys_read32(reg_base + PDM_FIFO_STATUS_REGISTER);
	}
	sys_write32(0U, reg_base + PDM_CONFIG_REGISTER);
	(void)sys_read32(reg_base + PDM_ERROR_IRQ);
	(void)sys_read32(reg_base + PDM_WARN_IRQ);
	(void)sys_read32(reg_base + PDM_AUDIO_DETECT_IRQ);

	ret = clock_control_off(cfg->clk_dev, cfg->clkid);
	if (ret == 0) {
		k_busy_wait(100);
		ret = clock_control_on(cfg->clk_dev, cfg->clkid);
	}
	if (ret != 0) {
		LOG_ERR("Unable to cycle clock: err:%d", ret);
		return ret;
	}

	cfg->irq_config();

	k_msgq_init(&pdata->buf_queue, (char *)pdata->queue_data, sizeof(void *), MAX_QUEUE_LEN);

	/* Enable the Bypass IIR Filter */
	sys_write32(pdata->bypass_iir_filter << PDM_BYPASS_IIR, reg_base + PDM_CTL_REGISTER);

	sys_write32(cfg->fifo_watermark, reg_base + PDM_THRESHOLD_REGISTER);

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
		.has_error_intr = DT_INST_IRQ_HAS_NAME(n, error_intr),                             \
		.has_audio_det_intr = DT_INST_IRQ_HAS_NAME(n, audio_det_intr),                     \
	};                                                                                         \
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
