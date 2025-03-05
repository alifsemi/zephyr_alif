/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_alif_pdm

#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/pdm/pdm_alif.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/init.h>
#include <zephyr/drivers/pinctrl.h>
#include "alif_pdm_reg.h"

#define CONF_PDM_INIT_PRIORITY 40

LOG_MODULE_REGISTER(alif_pdm, 3);

uint32_t ctrl_value;
uint32_t irq_value;

uint16_t data[MAX_NUM_CHANNELS * MAX_DATA_ITEMS];

void *queue_data[MAX_QUEUE_LEN];
const struct device *dt_device;

struct pdm_data {
	DEVICE_MMIO_RAM;
	struct k_mem_slab *mem_slab;
	unsigned int block_size;
	struct k_msgq buf_queue;
	uint8_t channel_map;
	uint32_t num_channels;
	unsigned char *data_buffer;
	unsigned int buf_index;
	unsigned int slab_missed;
	unsigned int record_data;
	unsigned int bytes_got;
	uint8_t bypass_iir_filter;
};

struct pdm_config {
	DEVICE_MMIO_ROM;
	void (*irq)(void);
	const unsigned long meminit;
	uint32_t fifo_watermark;
	const struct pinctrl_dev_config *pcfg;
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
	struct pdm_data *pdata;
	uintptr_t meminit = DEVICE_MMIO_GET(dev);

	if (config->channel.req_num_chan == 0 || config->channel.req_num_chan > MAX_NUM_CHANNELS) {
		LOG_DBG("config invalid: number of channels not valid\n");
		return -EINVAL;
	}

	pdata = dev->data;
	if (pdata) {
		pdata->mem_slab = config->streams[0].mem_slab;
		pdata->block_size = config->streams[0].block_size;
		pdata->channel_map = config->channel.req_chan_map_lo & 0xFF;

		ctrl_value |= pdata->channel_map;

		/* Enable the PDM multiple channels */
		sys_write32(ctrl_value, meminit + PDM_CONFIG_REGISTER);

		pdata->num_channels = config->channel.req_num_chan;

		LOG_DBG("block size: %d\n", pdata->block_size);
	}

	LOG_DBG("DMIC configure okay\n");

	return 0;
}

/**
 * @fn		void pdm_channel_config(const struct device *dev,
 *					struct PDM_CH_CONFIG *cnfg)
 * @brief	Sets FIR coefficient and IIR coefficient values.
 * @param[in]	dev  : Pointer to the runtime device structure.
 * @param[in]	cnfg : Pointer to the PDM_CH_CONFIG structure.
 * @return	    None
 */
void pdm_channel_config(const struct device *dev, struct PDM_CH_CONFIG *cnfg)
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
	uint32_t ch_n_phase = (reg_base + PDM_CH_PHASE + (ch_num * PDM_CH_OFFSET));

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
	uint32_t ch_n_gain = (reg_base + PDM_CH_GAIN + (ch_num * PDM_CH_OFFSET));

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
	uint32_t ch_n_pkdet_th = (reg_base + PDM_CH_PKDET_TH + (ch_num * PDM_CH_OFFSET));

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
	uint32_t ch_n_pkdet_itv = (reg_base + PDM_CH_PKDET_ITV + (ch_num * PDM_CH_OFFSET));

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
	const struct pdm_config *cfg;
	uint32_t meminit;

	cfg = dev->config;
	meminit = cfg->meminit;

	ctrl_value |= (mode << PDM_CLK_MODE);

	sys_write32(ctrl_value, meminit + PDM_CONFIG_REGISTER);
}

/**
 * @fn		void enable_interrupt(const struct device *dev)
 * @brief		Enable the IRQ
 * @param[in]	dev : Pointer to the runtime device structure.
 * @return	None
 */
static void enable_interrupt(const struct device *dev)
{
	const struct pdm_config *cfg;

	cfg = dev->config;

	uint32_t audio_ch;

	/* Store user enabled channel */
	audio_ch = ctrl_value & PDM_CHANNEL_ENABLE;

	irq_value |= (audio_ch << 8 | PDM_FIFO_ALMOST_FULL_IRQ | PDM_FIFO_OVERFLOW_IRQ);

	/* Enable the Interrupt */
	sys_write32(irq_value, cfg->meminit + PDM_INTERRUPT_REGISTER);
}

/**
 * @fn		void enable_interrupt(const struct device *dev)
 * @brief		Disable the IRQ
 * @param[in]	dev : Pointer to the runtime device structure.
 * @return	None
 */
static void disable_interrupt(const struct device *dev)
{
	const struct pdm_config *cfg;
	cfg = dev->config;

	/* Disable the Interrupt */
	sys_write32(0, cfg->meminit + PDM_INTERRUPT_REGISTER);
}

/**
 * @fn		int dmic_alif_pdm_trigger(const struct device *dev,
 *					enum dmic_trigger cmd)
 * @brief	Send DMIC_TRIGGER_STOP or DMIC_TRIGGER_START to
 *			perform the specific opration.
 * @param[in]   dev	: pointer to Runtime device structure
 * @param[in]   cmd	: DMIC start or stop command
 * @return	  Zero on success, and a negative value on failure.
 */
static int dmic_alif_pdm_trigger(const struct device *dev, enum dmic_trigger cmd)
{
	struct pdm_data *pdata = dev->data;

	switch (cmd) {
	case DMIC_TRIGGER_STOP:
		disable_interrupt(dev);
		pdata->record_data = 0;
		break;

	case DMIC_TRIGGER_START:
		LOG_DBG("trigger start\n");
		pdata->record_data = 1;
		pdata->bytes_got = 0;
		pdata->buf_index = 0;
		pdata->data_buffer = NULL;
		pdata->slab_missed = 0;

		enable_interrupt(dev);
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
	struct pdm_data *pdata = dev->data;
	int rc;

	rc = k_msgq_get(&pdata->buf_queue, buffer, SYS_TIMEOUT_MS(timeout));

	if (rc != 0) {
		LOG_DBG("No audio data to be read\n");
		LOG_DBG("bytes_got: %d\n", pdata->bytes_got);
	} else {
		*size = pdata->block_size;
	}
	return rc;
}

/**
 * @fn		void *get_slab(struct pdm_data *pdm_data)
 * @brief	Allocates a memory block from the slab for PCM data.
 * @param[in]	pdm_data Pointer to the PDM data structure
 *			containing the memory slab.
 * @return		Pointer to the allocated memory block on
 *			Zero on success, and a negative value on failure.
 */
void *get_slab(struct pdm_data *pdm_data)
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
 * @param[in]	None
 * @return	None.
 */
static void alif_pdm_warning_isr(void)
{
	const struct pdm_config *cfg;
	struct pdm_data *pdmdata;
	uint8_t k = 0;
	uint8_t audio_ch;
	uint8_t intstatus;
	uint32_t meminit;
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

	pdmdata = dt_device->data;
	block_size = pdmdata->block_size;

	cfg = dt_device->config;
	meminit = cfg->meminit;

	/* User enabled channel */
	audio_ch = ctrl_value & PDM_CHANNEL_ENABLE;

	intstatus = sys_read32(meminit + PDM_WARN_IRQ);
	num_items = sys_read32(meminit + PDM_FIFO_STATUS_REGISTER);

	for (i = 0; i < num_items; i++) {
		audio_ch_0_1 = sys_read32(meminit + PDM_CH0_CH1_AUDIO_OUT);
		audio_ch_2_3 = sys_read32(meminit + PDM_CH2_CH3_AUDIO_OUT);
		audio_ch_4_5 = sys_read32(meminit + PDM_CH4_CH5_AUDIO_OUT);
		audio_ch_6_7 = sys_read32(meminit + PDM_CH6_CH7_AUDIO_OUT);

		if ((audio_ch & PDM_CHANNEL_0) == PDM_CHANNEL_0) {
			data[k++] = (uint16_t)(audio_ch_0_1);
		}
		if ((audio_ch & PDM_CHANNEL_1) == PDM_CHANNEL_1) {
			data[k++] = (uint16_t)(audio_ch_0_1 >> 16);
		}
		if ((audio_ch & PDM_CHANNEL_2) == PDM_CHANNEL_2) {
			data[k++] = (uint16_t)(audio_ch_2_3);
		}
		if ((audio_ch & PDM_CHANNEL_3) == PDM_CHANNEL_3) {
			data[k++] = (uint16_t)(audio_ch_2_3 >> 16);
		}
		if ((audio_ch & PDM_CHANNEL_4) == PDM_CHANNEL_4) {
			data[k++] = (uint16_t)(audio_ch_4_5);
		}
		if ((audio_ch & PDM_CHANNEL_5) == PDM_CHANNEL_5) {
			data[k++] = (uint16_t)(audio_ch_4_5 >> 16);
		}
		if ((audio_ch & PDM_CHANNEL_6) == PDM_CHANNEL_6) {
			data[k++] = (uint16_t)(audio_ch_6_7);
		}
		if ((audio_ch & PDM_CHANNEL_7) == PDM_CHANNEL_7) {
			data[k++] = (uint16_t)(audio_ch_6_7 >> 16);
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
			sys_write32(0, cfg->meminit + PDM_INTERRUPT_REGISTER);
			return;
		}
		pdmdata->buf_index = 0;
	}

	bytes_available = block_size - pdmdata->buf_index;

	if (bytes_available >= data_bytes) {
		memcpy((pdmdata->data_buffer + pdmdata->buf_index), data, data_bytes);
		pdmdata->buf_index += data_bytes;
	} else {
		if (bytes_available > 0) {
			memcpy((pdmdata->data_buffer + pdmdata->buf_index), data, bytes_available);
		}
		whole = data_bytes - bytes_available;

		k_msgq_put(&pdmdata->buf_queue, &pdmdata->data_buffer, K_NO_WAIT);

		pdmdata->data_buffer = get_slab(pdmdata);

		if (pdmdata->data_buffer) {
			memcpy(pdmdata->data_buffer, data + bytes_available, whole);
			pdmdata->buf_index = whole;
		} else {
			pdmdata->buf_index = 0;
		}
	}
}

/* Init function */
static int pdm_initialize(const struct device *dev)
{
	const struct pdm_config *cfg;
	struct pdm_data *pdata = dev->data;
	uint32_t meminit;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	dt_device = dev;

	cfg = dev->config;
	meminit = cfg->meminit;
	pdata = dev->data;

	if (cfg->pcfg != NULL) {
		pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	}

	cfg->irq();

	k_msgq_init(&pdata->buf_queue, (char *)queue_data, sizeof(void *), MAX_QUEUE_LEN);

	/* Enable the Bypass IIR Filter */
	sys_write32(pdata->bypass_iir_filter << PDM_BYPASS_IIR, meminit + PDM_CTL_REGISTER);

	sys_write32(cfg->fifo_watermark, meminit + PDM_THRESHOLD_REGISTER);

	LOG_INF("alif pdm driver init okay\n");

	return 0;
}

static const struct _dmic_ops dmic_alif_pdm_api = {
	.configure = dmic_alif_pdm_configure,
	.trigger = dmic_alif_pdm_trigger,
	.read = dmic_alif_pdm_read,
};

/********** Device Definition per instance Macros **********/

#define PDM_INIT(i)                                                                                \
	PINCTRL_DT_INST_DEFINE(i);                                                                 \
	static void pdm_irq_config_##i(void);                                                      \
                                                                                                   \
	static struct pdm_data dmic_alif_pdm_data = {                                              \
		.bypass_iir_filter = DT_INST_PROP(i, bypass_iir_filter),                           \
	};                                                                                         \
	static const struct pdm_config dmic_alif_pdm_cfg_##i = {                                   \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(i)),                                              \
		.meminit = DT_INST_REG_ADDR(i),                                                    \
		.fifo_watermark = DT_INST_PROP(i, fifo_watermark),                                 \
		.irq = pdm_irq_config_##i,                                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),                                         \
	};                                                                                         \
	static void pdm_irq_config_##i(void)                                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(i), DT_INST_IRQ(i, priority), alif_pdm_warning_isr,       \
			    DEVICE_DT_INST_GET(i), 0);                                             \
		irq_enable(DT_INST_IRQN(i));                                                       \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(i, pdm_initialize, NULL, &dmic_alif_pdm_data,                        \
			      &dmic_alif_pdm_cfg_##i, POST_KERNEL, CONF_PDM_INIT_PRIORITY,       \
			      &dmic_alif_pdm_api);

DT_INST_FOREACH_STATUS_OKAY(PDM_INIT)
