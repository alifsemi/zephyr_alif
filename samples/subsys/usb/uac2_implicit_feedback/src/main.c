/*
 * Copyright (c) 2023-2024 Nordic Semiconductor ASA
 * Copyright (C) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <sample_usbd.h>
#include "feedback.h"

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/usb/udc_buf.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_uac2.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_SOC_FAMILY_BALLETTO) || defined(CONFIG_SOC_FAMILY_ENSEMBLE)
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/pdm/pdm_alif.h>
#define UAC2_ALIF 1
#endif
#if IS_ENABLED(CONFIG_AUDIO_CODEC)
#include <zephyr/audio/codec.h>
#endif

LOG_MODULE_REGISTER(uac2_sample, LOG_LEVEL_INF);

#define HEADPHONES_OUT_TERMINAL_ID UAC2_ENTITY_ID(DT_NODELABEL(out_terminal))
#define MICROPHONE_IN_TERMINAL_ID UAC2_ENTITY_ID(DT_NODELABEL(in_terminal))

#define SAMPLES_PER_SOF     48
#define HS_SAMPLES_PER_SOF  6
#define SAMPLE_FREQUENCY    (SAMPLES_PER_SOF * 1000)
#define SAMPLE_BIT_WIDTH    16
#define NUMBER_OF_CHANNELS  2
#define BYTES_PER_SAMPLE    DIV_ROUND_UP(SAMPLE_BIT_WIDTH, 8)
#define BYTES_PER_SLOT      (BYTES_PER_SAMPLE * NUMBER_OF_CHANNELS)
#define MIN_BLOCK_SIZE      ((SAMPLES_PER_SOF - 1) * BYTES_PER_SLOT)
#define BLOCK_SIZE          (SAMPLES_PER_SOF * BYTES_PER_SLOT)
#define MAX_BLOCK_SIZE      ((SAMPLES_PER_SOF + 1) * BYTES_PER_SLOT)

/* Absolute minimum is 5 TX buffers (1 actively consumed by I2S, 2nd queued as
 * next buffer, 3rd acquired by USB stack to receive data to, and 2 to handle
 * SOF/I2S offset errors), but add 2 additional buffers to prevent out of memory
 * errors when USB host decides to perform rapid terminal enable/disable cycles.
 *
 * Alif High-Speed keeps four USB OUT requests queued. The I2S driver queue is
 * CONFIG_I2S_DW_TX_BLOCK_COUNT (16), so the slab has to be deeper than that
 * queue or OUT reads fail with "No data buffer" once I2S is holding every
 * block.
 */
#if defined(UAC2_ALIF)
#define I2S_BLOCKS          24
/* Leave the four OUT requests, plus two spares, free for the USB stack. */
#define I2S_TX_SLAB_RESERVE 6U
K_MEM_SLAB_DEFINE_STATIC(i2s_tx_slab, ROUND_UP(MAX_BLOCK_SIZE, UDC_BUF_GRANULARITY),
			 I2S_BLOCKS, UDC_BUF_ALIGN);
K_MEM_SLAB_DEFINE_STATIC(usb_mic_slab, ROUND_UP(MAX_BLOCK_SIZE, UDC_BUF_GRANULARITY),
			 I2S_BLOCKS, UDC_BUF_ALIGN);
#else
#define I2S_BLOCKS          7
K_MEM_SLAB_DEFINE_STATIC(i2s_tx_slab, MAX_BLOCK_SIZE, I2S_BLOCKS, 4);
K_MEM_SLAB_DEFINE_STATIC(i2s_rx_slab, MAX_BLOCK_SIZE, I2S_BLOCKS, 4);
#endif

#if defined(UAC2_ALIF) && DT_HAS_ALIAS(pdm_audio)
#define PDM_NODE DT_ALIAS(pdm_audio)
#define PDM_BLOCK_SIZE 384U /* 12 ms, 8 kHz, 16-bit, 2 channels */
#define PDM_BLOCK_COUNT 8
/* USB microphone is 48 kHz. The E1C PDM voice mode is 8 kHz, so each
 * PCM sample is held for 48000/8000 frames.
 */
#define PDM_USB_REPEAT 6U
K_MEM_SLAB_DEFINE_STATIC(pdm_slab, PDM_BLOCK_SIZE, PDM_BLOCK_COUNT, 4);
static const struct device *pdm_dev = DEVICE_DT_GET(PDM_NODE);
static bool pdm_started;
static uint8_t *pdm_pending_block;
static size_t pdm_pending_bytes;
static size_t pdm_pending_offset;
static uint8_t pdm_sample_off;
static bool pdm_block_logged;
static uint32_t pdm_empty_reads;
static int16_t pdm_repeat_sample;
static uint8_t pdm_repeat_left;

/* Per-edge FIR from the Alif E1C LPPDM demo (demo_lppdm.c). */
static const uint32_t ch4_fir[18] = {
	0x00000001, 0x00000003, 0x00000003, 0x000007F4, 0x00000004, 0x000007ED,
	0x000007F5, 0x000007F4, 0x000007D3, 0x000007FE, 0x000007BC, 0x000007E5,
	0x000007D9, 0x00000793, 0x00000029, 0x0000072C, 0x00000072, 0x000002FD
};
static const uint32_t ch5_fir[18] = {
	0x00000000, 0x000007FF, 0x00000000, 0x00000004, 0x00000004, 0x000007FC,
	0x00000000, 0x000007FB, 0x000007E4, 0x00000000, 0x0000002B, 0x00000009,
	0x00000016, 0x00000049, 0x00000793, 0x000006F8, 0x00000045, 0x00000178
};

static void pdm_release_pending(void)
{
	if (pdm_pending_block != NULL) {
		k_mem_slab_free(&pdm_slab, pdm_pending_block);
		pdm_pending_block = NULL;
	}

	pdm_pending_bytes = 0U;
	pdm_pending_offset = 0U;
	pdm_sample_off = 0U;
	pdm_repeat_left = 0U;
}

/* CH4 is the first int16 of each pair, CH5 the second. A mono PDM mic drives
 * one clock edge. The other edge sits on one or two codes (the FF 80 / FF 00
 * pattern). Absolute level prefers that stuck edge, so pick the edge whose
 * samples actually move, and fall back to the one centered near zero.
 */
static uint8_t pdm_pick_channel(const uint8_t *block, size_t size)
{
	int16_t min0 = INT16_MAX;
	int16_t max0 = INT16_MIN;
	int16_t min1 = INT16_MAX;
	int16_t max1 = INT16_MIN;
	int32_t sum0 = 0;
	int32_t sum1 = 0;
	int32_t frames = 0;

	for (size_t i = 0U; i + 4U <= size; i += 4U) {
		int16_t left;
		int16_t right;

		memcpy(&left, block + i, sizeof(left));
		memcpy(&right, block + i + 2U, sizeof(right));
		min0 = MIN(min0, left);
		max0 = MAX(max0, left);
		min1 = MIN(min1, right);
		max1 = MAX(max1, right);
		sum0 += left;
		sum1 += right;
		frames++;
	}

	if (frames == 0) {
		return 0U;
	}

	if (((int32_t)max1 - min1) > ((int32_t)max0 - min0) + 64) {
		return 2U;
	}
	if (((int32_t)max0 - min0) > ((int32_t)max1 - min1) + 64) {
		return 0U;
	}

	sum0 = sum0 < 0 ? -sum0 : sum0;
	sum1 = sum1 < 0 ? -sum1 : sum1;
	return sum1 < sum0 ? 2U : 0U;
}

static void pdm_channel_setup(void)
{
	struct pdm_ch_config cfg = {0};

	/* Channel 4 is the demo's channel-1 edge. Channel 5 is the
	 * channel-0 edge. Gain 0x800 with one shared FIR is what produced
	 * the repeating FF 80 / FF 00 packets.
	 */
	pdm_set_ch_phase(pdm_dev, 4, 0x0000001F);
	pdm_set_ch_gain(pdm_dev, 4, 0x0000000D);
	pdm_set_peak_detect_th(pdm_dev, 4, 0x00060002);
	pdm_set_peak_detect_itv(pdm_dev, 4, 0x0004002D);
	cfg.ch_num = 4;
	memcpy(cfg.ch_fir_coef, ch4_fir, sizeof(ch4_fir));
	cfg.ch_iir_coef = 0x00000004;
	pdm_channel_config(pdm_dev, &cfg);

	pdm_set_ch_phase(pdm_dev, 5, 0x00000003);
	pdm_set_ch_gain(pdm_dev, 5, 0x00000013);
	pdm_set_peak_detect_th(pdm_dev, 5, 0x00060002);
	pdm_set_peak_detect_itv(pdm_dev, 5, 0x00020027);
	cfg.ch_num = 5;
	memcpy(cfg.ch_fir_coef, ch5_fir, sizeof(ch5_fir));
	cfg.ch_iir_coef = 0x00000004;
	pdm_channel_config(pdm_dev, &cfg);
	/* 512 kHz / 64 = 8 kHz. This is the mode the FIR tables above are
	 * built for. Mode 0x07 is 48 kHz and locks the output to two codes.
	 */
	pdm_mode(pdm_dev, PDM_MODE_STANDARD_VOICE_512_CLK_FRQ);
}

static int pdm_init(void)
{
	struct pcm_stream_cfg stream = {0};
	struct dmic_cfg cfg = {0};
	int ret;

	if (!device_is_ready(pdm_dev)) {
		LOG_ERR("PDM device not ready");
		return -ENODEV;
	}

	stream.pcm_rate = SAMPLE_FREQUENCY;
	stream.pcm_width = SAMPLE_BIT_WIDTH;
	stream.block_size = PDM_BLOCK_SIZE;
	stream.mem_slab = &pdm_slab;
	cfg.streams = &stream;
	cfg.channel.req_num_streams = 1;
	cfg.channel.req_num_chan = 2;
	cfg.channel.req_chan_map_lo = PDM_MASK_CHANNEL_4 | PDM_MASK_CHANNEL_5;

	ret = dmic_configure(pdm_dev, &cfg);
	if (ret < 0) {
		LOG_ERR("dmic_configure failed: %d", ret);
		return ret;
	}

	pdm_channel_setup();
	return 0;
}

static void pdm_set_enabled(bool enable)
{
	int ret;

	if (enable == pdm_started) {
		return;
	}

	if (!enable) {
		ret = dmic_trigger(pdm_dev, DMIC_TRIGGER_STOP);
		if (ret < 0) {
			LOG_ERR("PDM STOP failed: %d", ret);
		}
		pdm_started = false;
		pdm_release_pending();
		return;
	}

	pdm_release_pending();
	ret = dmic_trigger(pdm_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("PDM START failed: %d", ret);
		return;
	}

	pdm_started = true;
}

/* Copy one mono sample from each interleaved stereo PDM frame. */
static size_t pdm_copy_mono(uint8_t *dst, size_t requested_bytes)
{
	size_t copied = 0U;

	if (!pdm_started) {
		return 0U;
	}

	while (copied + sizeof(int16_t) <= requested_bytes) {
		if (pdm_repeat_left > 0U) {
			memcpy(dst + copied, &pdm_repeat_sample, sizeof(pdm_repeat_sample));
			copied += sizeof(int16_t);
			pdm_repeat_left--;
			continue;
		}

		if (pdm_pending_block == NULL ||
		    pdm_pending_offset + 4U > pdm_pending_bytes) {
			void *block = NULL;
			size_t size = 0U;
			int ret;

			pdm_release_pending();
			ret = dmic_read(pdm_dev, 0, &block, &size, 0);
			if (ret < 0 || block == NULL) {
				if (pdm_empty_reads++ == 8000U) {
					LOG_WRN("PDM queue empty");
				}
				break;
			}
			if (size > PDM_BLOCK_SIZE || (size % 4U) != 0U) {
				LOG_ERR("Unexpected PDM block size: %u", (unsigned int)size);
				k_mem_slab_free(&pdm_slab, block);
				break;
			}

			pdm_pending_block = block;
			pdm_pending_bytes = size;
			pdm_pending_offset = 0U;
			pdm_sample_off = pdm_pick_channel(block, size);
			pdm_empty_reads = 0U;
			if (!pdm_block_logged) {
				int16_t peak0 = 0;
				int16_t peak1 = 0;

				pdm_block_logged = true;
				for (size_t i = 0U; i + 4U <= size; i += 4U) {
					int16_t left;
					int16_t right;

					memcpy(&left, (uint8_t *)block + i, sizeof(left));
					memcpy(&right, (uint8_t *)block + i + 2U, sizeof(right));
					if (abs(left) > abs(peak0)) {
						peak0 = left;
					}
					if (abs(right) > abs(peak1)) {
						peak1 = right;
					}
				}
				LOG_INF("PDM block %u bytes, ch4 peak %d, ch5 peak %d, offset %u",
					(unsigned int)size, peak0, peak1, pdm_sample_off);
			}
		}

		memcpy(&pdm_repeat_sample,
		       pdm_pending_block + pdm_pending_offset + pdm_sample_off,
		       sizeof(pdm_repeat_sample));
		pdm_pending_offset += 4U;
		pdm_repeat_left = PDM_USB_REPEAT;
	}

	return copied;
}
#endif

struct usb_i2s_ctx {
	const struct device *i2s_dev;
	bool headphones_enabled;
	bool microphone_enabled;
	bool i2s_started;
	bool rx_started;
	bool usb_data_received;
	bool microframes;
	/* Counter used to determine when to start I2S and then when to start
	 * sending RX packets to host. Overflows are not a problem because this
	 * variable is not necessary after both I2S and RX is started.
	 */
	uint8_t i2s_counter;
	struct feedback_ctx *fb;

	/* Leftover samples from I2S receive buffer, already compacted to mono,
	 * that were not sent to host. The buffer, if not NULL, is allocated
	 * from I2S RX slab.
	 */
	uint8_t *pending_mic_buf;
	uint8_t pending_mic_samples;

	/* Rolling bit buffers for tracking nominal + 1 and nominal - 1 samples
	 * sent. Bits are mutually exclusive, i.e.:
	 *   plus_ones | minus_ones = plus_ones ^ minus_ones
	 *
	 * Used to avoid overcompensation in feedback regulator. LSBs indicate
	 * latest write size. Full-Speed keeps the last 8 intervals.
	 */
	uint32_t plus_ones;
	uint32_t minus_ones;
};

static void uac2_terminal_update_cb(const struct device *dev, uint8_t terminal,
				    bool enabled, bool microframes,
				    void *user_data)
{
	struct usb_i2s_ctx *ctx = user_data;

#if !defined(UAC2_ALIF)
	/* The nRF build of this sample is Full-Speed only. */
	__ASSERT_NO_MSG(microframes == false);
#endif
	ctx->microframes = microframes;

#if defined(UAC2_ALIF)
	if (!ctx->i2s_started) {
		feedback_set_speed(ctx->fb, microframes);
	}
#endif

	if (terminal == HEADPHONES_OUT_TERMINAL_ID) {
		ctx->headphones_enabled = enabled;
	} else if (terminal == MICROPHONE_IN_TERMINAL_ID) {
		ctx->microphone_enabled = enabled;
#if defined(UAC2_ALIF) && DT_HAS_ALIAS(pdm_audio)
		pdm_set_enabled(enabled);
#endif
	}

	/* Capture is PDM, so I2S TX is only the headphone path. Drop it as
	 * soon as playback stops, even if the host keeps the mic interface
	 * open for implicit feedback. Blocks queued before START must be
	 * returned too, or the next OUT stream has an empty slab.
	 */
#if defined(UAC2_ALIF)
	if (!ctx->headphones_enabled &&
	    (ctx->i2s_started || ctx->i2s_counter > 0U)) {
		i2s_trigger(ctx->i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
		ctx->i2s_started = false;
		ctx->i2s_counter = 0;
		ctx->plus_ones = ctx->minus_ones = 0;
		feedback_reset_ctx(ctx->fb);
	}
#else
	if (ctx->i2s_started && !ctx->headphones_enabled &&
	    !ctx->microphone_enabled) {
		i2s_trigger(ctx->i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_DROP);
		ctx->i2s_started = false;
		ctx->i2s_counter = 0;
		ctx->plus_ones = ctx->minus_ones = 0;
#if !defined(UAC2_ALIF)
		if (ctx->pending_mic_samples) {
			k_mem_slab_free(&i2s_rx_slab, ctx->pending_mic_buf);
			ctx->pending_mic_buf = NULL;
			ctx->pending_mic_samples = 0;
		}
#endif
		feedback_reset_ctx(ctx->fb);
	}
#endif
}

static int nominal_samples_per_sof(const struct usb_i2s_ctx *ctx)
{
#if defined(UAC2_ALIF)
	return ctx->microframes ? HS_SAMPLES_PER_SOF : SAMPLES_PER_SOF;
#else
	ARG_UNUSED(ctx);
	return SAMPLES_PER_SOF;
#endif
}

static void *uac2_get_recv_buf(const struct device *dev, uint8_t terminal,
			       uint16_t size, void *user_data)
{
	ARG_UNUSED(dev);
	struct usb_i2s_ctx *ctx = user_data;
	void *buf = NULL;
	int ret;

	if (terminal == HEADPHONES_OUT_TERMINAL_ID) {
		__ASSERT_NO_MSG(size <= MAX_BLOCK_SIZE);

		if (!ctx->headphones_enabled) {
			LOG_ERR("Buffer request on disabled terminal");
			return NULL;
		}

		ret = k_mem_slab_alloc(&i2s_tx_slab, &buf, K_NO_WAIT);
		if (ret != 0) {
			buf = NULL;
		}
	}

	return buf;
}

static void uac2_data_recv_cb(const struct device *dev, uint8_t terminal,
			      void *buf, uint16_t size, void *user_data)
{
	struct usb_i2s_ctx *ctx = user_data;
	int nominal = nominal_samples_per_sof(ctx);
	int ret;

	ctx->usb_data_received = true;

#if defined(UAC2_ALIF)
	/* Mic-only capture must not consume TX slab blocks. */
	if (!ctx->headphones_enabled) {
		k_mem_slab_free(&i2s_tx_slab, buf);
		return;
	}
#else
	if (!ctx->headphones_enabled && !ctx->microphone_enabled) {
		k_mem_slab_free(&i2s_tx_slab, buf);
		return;
	}
#endif

	if (!size) {
		/* This code path is expected when host only records microphone
		 * data and is not streaming any audio to the headphones. Simply
		 * transmit as many zero-filled samples were last sent to allow
		 * the feedback regulator to work.
		 *
		 * When host is streaming audio, this can be a transient error.
		 * While the "feedback regulator delay" is likely to differ,
		 * it is still probably best to just zero-fill last sent number
		 * of samples. If we overcompensate as a result, the situation
		 * will stabilize after a while anyway.
		 *
		 * In either case, we have to keep I2S going and the only way
		 * we can control the SOF to I2S offset is by varying the number
		 * of samples sent.
		 */
		if (ctx->plus_ones & 1) {
			size = (nominal + 1) * BYTES_PER_SLOT;
		} else if (ctx->minus_ones & 1) {
			size = (nominal - 1) * BYTES_PER_SLOT;
		} else {
			size = nominal * BYTES_PER_SLOT;
		}
		memset(buf, 0, size);
		sys_cache_data_flush_range(buf, size);
	}

	LOG_DBG("Received %d data to input terminal %d", size, terminal);

	ret = i2s_write(ctx->i2s_dev, buf, size);
	if (ret < 0) {
#if defined(UAC2_ALIF)
		/* Queue full while I2S is still running. Drop this packet.
		 * PREPARE only succeeds from the ERROR state, so treating
		 * -EAGAIN as an underrun clears i2s_started and then START
		 * fails on every SOF with the hardware still RUNNING.
		 */
		if (ret == -EAGAIN || ret == -ENOMEM || ret == -ETIMEDOUT) {
			LOG_WRN_ONCE("I2S TX queue full (%d)", ret);
			k_mem_slab_free(&i2s_tx_slab, buf);
			return;
		}
#endif
		ctx->i2s_started = false;
		ctx->i2s_counter = 0;
		ctx->plus_ones = ctx->minus_ones = 0;
#if !defined(UAC2_ALIF)
		if (ctx->pending_mic_samples) {
			k_mem_slab_free(&i2s_rx_slab, ctx->pending_mic_buf);
			ctx->pending_mic_buf = NULL;
			ctx->pending_mic_samples = 0;
		}
#endif
		feedback_reset_ctx(ctx->fb);

		/* Most likely underrun occurred, prepare I2S restart */
#if defined(UAC2_ALIF)
		/* DROP returns every block still queued. PREPARE does that
		 * only from the ERROR state and leaves a RUNNING queue held.
		 */
		i2s_trigger(ctx->i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
#else
		i2s_trigger(ctx->i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_PREPARE);
#endif

		ret = i2s_write(ctx->i2s_dev, buf, size);
		if (ret < 0) {
			/* Drop data block, will try again on next frame */
			k_mem_slab_free(&i2s_tx_slab, buf);
		}
	}

	if (ret == 0) {
		ctx->i2s_counter++;
	}
}

static void uac2_buf_release_cb(const struct device *dev, uint8_t terminal,
				void *buf, void *user_data)
{
	if (terminal == MICROPHONE_IN_TERMINAL_ID) {
#if defined(UAC2_ALIF)
		k_mem_slab_free(&usb_mic_slab, buf);
#else
		k_mem_slab_free(&i2s_rx_slab, buf);
#endif
	}
}

/* Determine next number of samples to send, called at most once every SOF */
static int next_mic_num_samples(struct usb_i2s_ctx *ctx)
{
	int nominal = nominal_samples_per_sof(ctx);
	int offset = feedback_samples_offset(ctx->fb);

	/* The rolling buffers essentially handle controller dead time, i.e.
	 * the buffers are used to prevent overcompensating on feedback offset.
	 * Remove the oldest entry by shifting the values by one bit.
	 */
	ctx->plus_ones <<= 1;
	ctx->minus_ones <<= 1;
	if (!ctx->microframes) {
		ctx->plus_ones &= 0xFFU;
		ctx->minus_ones &= 0xFFU;
	}

	if ((offset < 0) && (POPCOUNT(ctx->plus_ones) < -offset)) {
		/* Device clock is ahead. Send one extra sample so the host
		 * increases the playback rate.
		 */
		ctx->plus_ones |= 1;
		return nominal + 1;
	}

	if ((offset > 0) && (POPCOUNT(ctx->minus_ones) < offset)) {
		/* Device clock is behind. Send one fewer sample so the host
		 * decreases the playback rate.
		 */
		ctx->minus_ones |= 1;
		return nominal - 1;
	}

	/* I2S is either spot on, or the offset is expected to correct soon */
	return nominal;
}

#if defined(UAC2_ALIF)
static void process_mic_data(const struct device *dev, struct usb_i2s_ctx *ctx)
{
	uint8_t *buf;
	int samples;
	size_t bytes;
	size_t copied = 0U;
	int ret;

	if (!ctx->microphone_enabled) {
		return;
	}

	/* Arm every free IN slot. High-Speed needs one packet per 125 us
	 * microframe; a single SOF can be late, so the controller must already
	 * be holding the next millisecond of packets.
	 */
	while (usbd_uac2_can_send(dev, MICROPHONE_IN_TERMINAL_ID)) {
		samples = next_mic_num_samples(ctx);
		bytes = (size_t)samples * BYTES_PER_SAMPLE;

		ret = k_mem_slab_alloc(&usb_mic_slab, (void **)&buf, K_NO_WAIT);
		if (ret != 0) {
			return;
		}

#if DT_HAS_ALIAS(pdm_audio)
		copied = pdm_copy_mono(buf, bytes);
#endif
		if (copied < bytes) {
			memset(buf + copied, 0, bytes - copied);
		}

		sys_cache_data_flush_range(buf, bytes);
		if (usbd_uac2_send(dev, MICROPHONE_IN_TERMINAL_ID, buf, bytes) < 0) {
			k_mem_slab_free(&usb_mic_slab, buf);
			return;
		}
	}
}
#else
static void process_mic_data(const struct device *dev, struct usb_i2s_ctx *ctx)
{
	size_t num_bytes;
	uint8_t *dst, *src;
	uint8_t *mic_buf;
	void *rx_block;
	int ret;
	int samples_to_send, mic_samples, rx_samples, leftover_samples;

	samples_to_send = next_mic_num_samples(ctx);

	if (ctx->pending_mic_samples >= samples_to_send) {
		/* No need to fetch new I2S samples, this happens shortly after
		 * we have "borrowed" samples from next buffer. This is expected
		 * and means that the streams have synchronized.
		 */
		rx_block = NULL;
		rx_samples = 0;
	} else {
		ret = i2s_read(ctx->i2s_dev, &rx_block, &num_bytes);
		if (ret) {
			/* No data available, I2S will restart soon */
			return;
		}
		sys_cache_data_invd_range(rx_block, num_bytes);

		/* I2S operates on 2 channels (stereo) */
		rx_samples = num_bytes / (BYTES_PER_SAMPLE * 2);
	}

	/* Prepare microphone data to send, use pending samples if any */
	src = rx_block;
	if (ctx->pending_mic_buf) {
		mic_buf = ctx->pending_mic_buf;
		mic_samples = ctx->pending_mic_samples;
		dst = &ctx->pending_mic_buf[mic_samples * BYTES_PER_SAMPLE];
	} else if (rx_samples >= 1) {
		/* First sample is already in place */
		mic_buf = rx_block;
		dst = &mic_buf[BYTES_PER_SAMPLE];
		src += 2 * BYTES_PER_SAMPLE;
		mic_samples = 1;
		rx_samples--;
	} else {
		/* Something went horribly wrong, free the buffer and leave */
		k_mem_slab_free(&i2s_rx_slab, rx_block);
		return;
	}

	/* Copy as many samples as possible, stop if mic buffer is ready */
	while ((mic_samples < samples_to_send) && (rx_samples > 0)) {
		memcpy(dst, src, BYTES_PER_SAMPLE);

		dst += BYTES_PER_SAMPLE;
		src += 2 * BYTES_PER_SAMPLE;

		mic_samples++;
		rx_samples--;
	}

	/* Is mic buffer ready to go? */
	if (mic_samples < samples_to_send) {
		/* No, we have to borrow sample from next buffer. This can only
		 * happen if we fully drained current receive buffer.
		 */
		__ASSERT_NO_MSG(rx_samples == 0);

		if (rx_block != mic_buf) {
			/* RX buffer no longer needed, samples are in mic_buf */
			k_mem_slab_free(&i2s_rx_slab, rx_block);
		}

		ret = i2s_read(ctx->i2s_dev, &rx_block, &num_bytes);
		if (ret) {
			/* No data, I2S will likely restart due to error soon */
			ctx->pending_mic_buf = mic_buf;
			ctx->pending_mic_samples = mic_samples;
			return;
		}
		sys_cache_data_invd_range(rx_block, num_bytes);

		src = rx_block;
		rx_samples = num_bytes / (BYTES_PER_SAMPLE * 2);
	}

	/* Copy remaining sample, under normal conditions (i.e. connected to
	 * non-malicious host) this is guaranteed to fully fill mic_buf.
	 */
	while ((mic_samples < samples_to_send) && (rx_samples > 0)) {
		memcpy(dst, src, BYTES_PER_SAMPLE);

		dst += BYTES_PER_SAMPLE;
		src += 2 * BYTES_PER_SAMPLE;

		mic_samples++;
		rx_samples--;
	}

	/* Are we still short on samples? */
	if (mic_samples < samples_to_send) {
		/* The only possibility for this code to execute is that we were
		 * short on samples and the next block (pointed to by rx_block)
		 * did not contain enough samples to fill the gap.
		 */
		__ASSERT_NO_MSG(rx_block != mic_buf);

		/* Bailing out at this point likely leads to faster recovery.
		 * Note that this should never happen during normal operation.
		 */
		ctx->pending_mic_buf = mic_buf;
		ctx->pending_mic_samples = mic_samples;

		/* RX buffer is no longer needed */
		k_mem_slab_free(&i2s_rx_slab, rx_block);
		return;
	}

	/* Handle any potential leftover, start by sanitizing length */
	leftover_samples = mic_samples - samples_to_send + rx_samples;
	if (leftover_samples > (MAX_BLOCK_SIZE / BYTES_PER_SAMPLE)) {
		size_t dropped_samples =
			leftover_samples - (MAX_BLOCK_SIZE / BYTES_PER_SAMPLE);

		LOG_WRN("Too many leftover samples, dropping %d samples",
			dropped_samples);
		if (rx_samples >= dropped_samples) {
			rx_samples -= dropped_samples;
		} else {
			mic_samples -= (dropped_samples - rx_samples);
			rx_samples = 0;
		}

		leftover_samples = (MAX_BLOCK_SIZE / BYTES_PER_SAMPLE);
	}

	if (leftover_samples == 0) {
		/* No leftover samples */
		if ((rx_block != NULL) && (rx_block != mic_buf)) {
			/* All samples were copied, free source buffer */
			k_mem_slab_free(&i2s_rx_slab, rx_block);
		}
		rx_block = NULL;
	} else if ((mic_samples > samples_to_send) ||
		   ((rx_samples > 0) && (rx_block == mic_buf))) {
		/* Leftover samples have to be copied to new buffer */
		ret = k_mem_slab_alloc(&i2s_rx_slab, &rx_block, K_NO_WAIT);
		if (ret != 0) {
			LOG_WRN("Out of memory dropping %d samples",
				leftover_samples);
			mic_samples = samples_to_send;
			rx_samples = 0;
			rx_block = NULL;
		}
	}

	/* At this point rx_block is either
	 *   * NULL if there are no leftover samples, OR
	 *   * src buffer if leftover data can be copied from back to front, OR
	 *   * brand new buffer if there is leftover data in mic buffer.
	 */
	ctx->pending_mic_buf = rx_block;
	ctx->pending_mic_samples = 0;

	/* Copy excess samples from pending mic buf, if any */
	if (mic_samples > samples_to_send) {
		size_t bytes;

		/* Samples in mic buffer are already compacted */
		bytes = (mic_samples - samples_to_send) * BYTES_PER_SAMPLE;
		memcpy(ctx->pending_mic_buf, &mic_buf[mic_samples], bytes);

		ctx->pending_mic_samples = mic_samples - samples_to_send;
		dst = &ctx->pending_mic_buf[bytes];
	} else {
		dst = ctx->pending_mic_buf;
	}

	/* Copy excess samples from src buffer, so we don't lose any */
	while (rx_samples > 0) {
		memcpy(dst, src, BYTES_PER_SAMPLE);

		dst += BYTES_PER_SAMPLE;
		src += 2 * BYTES_PER_SAMPLE;

		ctx->pending_mic_samples++;
		rx_samples--;
	}

	/* Finally send the microphone samples to host */
	sys_cache_data_flush_range(mic_buf, mic_samples * BYTES_PER_SAMPLE);
	if (usbd_uac2_send(dev, MICROPHONE_IN_TERMINAL_ID,
			   mic_buf, mic_samples * BYTES_PER_SAMPLE) < 0) {
		k_mem_slab_free(&i2s_rx_slab, mic_buf);
	}
}
#endif

static void uac2_sof(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	struct usb_i2s_ctx *ctx = user_data;

	if (ctx->i2s_started) {
		feedback_process(ctx->fb);
	}

	/* If we didn't receive data since last SOF but either terminal is
	 * enabled, then we have to come up with the buffer ourself to keep
	 * I2S going.
	 */
	if (!ctx->usb_data_received &&
#if defined(UAC2_ALIF)
	    ctx->headphones_enabled && ctx->i2s_started &&
	    k_mem_slab_num_free_get(&i2s_tx_slab) > I2S_TX_SLAB_RESERVE) {
#else
	    (ctx->microphone_enabled || ctx->headphones_enabled)) {
#endif
		/* No data received since last SOF but we have to keep going */
		void *buf;
		int ret;

		ret = k_mem_slab_alloc(&i2s_tx_slab, &buf, K_NO_WAIT);
		if (ret != 0) {
			buf = NULL;
		}

		if (buf) {
			/* Use size 0 to utilize zero-fill functionality */
			uac2_data_recv_cb(dev, HEADPHONES_OUT_TERMINAL_ID,
					  buf, 0, user_data);
		}
	}
	ctx->usb_data_received = false;

	/* We want to maintain 3 SOFs delay, i.e. samples received from host
	 * during SOF n should be transmitted on I2S during SOF n+3. This
	 * provides enough wiggle room for software scheduling that effectively
	 * eliminates "buffers not provided in time" problem.
	 *
	 * ">= 2" translates into 3 SOFs delay because the timeline is:
	 * USB SOF n
	 *   OUT DATA0 n received from host
	 * USB SOF n+1
	 *   DATA0 n is available to UDC driver (See Universal Serial Bus
	 *   Specification Revision 2.0 5.12.5 Data Prebuffering) and copied
	 *   to I2S buffer before SOF n+2; i2s_counter = 1
	 *   OUT DATA0 n+1 received from host
	 * USB SOF n+2
	 *   DATA0 n+1 is copied; i2s_counter = 2
	 *   OUT DATA0 n+2 received from host
	 * USB SOF n+3
	 *   This function triggers I2S start
	 *   DATA0 n+2 is copied; i2s_counter is no longer relevant
	 *   OUT DATA0 n+3 received from host
	 */
	if (!ctx->i2s_started &&
#if defined(UAC2_ALIF)
	    ctx->headphones_enabled &&
	    ctx->i2s_counter >= 4) {
		int start_ret = i2s_trigger(ctx->i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);

		if (start_ret == 0) {
			ctx->i2s_started = true;
			feedback_start(ctx->fb, ctx->i2s_counter);
			LOG_INF("I2S TX started: queued=%u HS=%u",
				ctx->i2s_counter, ctx->microframes);
		} else {
			LOG_ERR("I2S START failed: %d", start_ret);
		}
	}
#else
	    (ctx->headphones_enabled || ctx->microphone_enabled) &&
	    ctx->i2s_counter >= 2) {
		i2s_trigger(ctx->i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_START);
		ctx->i2s_started = true;
		feedback_start(ctx->fb, ctx->i2s_counter);
		ctx->i2s_counter = 0;
	}
#endif

	/* Start sending I2S RX data only when there are at least 3 buffers
	 * ready with data. This guarantees that there'll always be a buffer
	 * available from which sample can be borrowed.
	 */
	if (!ctx->rx_started && ctx->i2s_started && ctx->i2s_counter >= 3) {
		ctx->rx_started = true;
	}

#if defined(UAC2_ALIF)
	if (ctx->microphone_enabled) {
		process_mic_data(dev, ctx);
	}
#else
	if (ctx->rx_started) {
		process_mic_data(dev, ctx);
	}
#endif
}

static struct uac2_ops usb_audio_ops = {
	.sof_cb = uac2_sof,
	.terminal_update_cb = uac2_terminal_update_cb,
	.get_recv_buf = uac2_get_recv_buf,
	.data_recv_cb = uac2_data_recv_cb,
	.buf_release_cb = uac2_buf_release_cb,
};

static struct usb_i2s_ctx main_ctx;

#if IS_ENABLED(CONFIG_AUDIO_CODEC)
static int configure_codec(const struct device *codec_dev)
{
	struct audio_codec_cfg audio_cfg = {0};
	int ret;

	audio_cfg.dai_route = AUDIO_ROUTE_PLAYBACK;
	audio_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
	audio_cfg.dai_cfg.i2s.word_size = SAMPLE_BIT_WIDTH;
	audio_cfg.dai_cfg.i2s.channels = NUMBER_OF_CHANNELS;
	audio_cfg.dai_cfg.i2s.format = I2S_FMT_DATA_FORMAT_I2S;
	/*
	 * The codec receives BCLK and LRCLK. The I2S controller is configured
	 * as master and drives both clocks.
	 */
	audio_cfg.dai_cfg.i2s.options = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE;
	audio_cfg.dai_cfg.i2s.frame_clk_freq = SAMPLE_FREQUENCY;
	audio_cfg.dai_cfg.i2s.mem_slab = &i2s_tx_slab;
	audio_cfg.dai_cfg.i2s.block_size = MAX_BLOCK_SIZE;
	audio_cfg.dai_cfg.i2s.timeout = 0;

	ret = audio_codec_configure(codec_dev, &audio_cfg);
	if (ret < 0) {
		printk("Failed to configure codec: %d\n", ret);
		return ret;
	}

	audio_codec_start_output(codec_dev);
	printk("Codec configured for playback\n");

	return 0;
}
#endif

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(uac2_headset));
	struct usbd_context *sample_usbd;
	struct i2s_config config;
	int ret;

	main_ctx.i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s_rxtx));

	if (!device_is_ready(main_ctx.i2s_dev)) {
		printk("%s is not ready\n", main_ctx.i2s_dev->name);
		return 0;
	}

#if IS_ENABLED(CONFIG_AUDIO_CODEC)
	const struct device *codec_dev = DEVICE_DT_GET(DT_ALIAS(audio_codec));

	if (!device_is_ready(codec_dev)) {
		printk("%s is not ready\n", codec_dev->name);
		return -ENODEV;
	}

	ret = configure_codec(codec_dev);
	if (ret < 0) {
		return ret;
	}
#endif

	config.word_size = SAMPLE_BIT_WIDTH;
	config.channels = NUMBER_OF_CHANNELS;
	config.format = I2S_FMT_DATA_FORMAT_I2S;
	config.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	config.frame_clk_freq = SAMPLE_FREQUENCY;
	config.mem_slab = &i2s_tx_slab;
	config.block_size = MAX_BLOCK_SIZE;
	config.timeout = 0;

	ret = i2s_configure(main_ctx.i2s_dev, I2S_DIR_TX, &config);
	if (ret < 0) {
		printk("Failed to configure TX stream: %d\n", ret);
		return 0;
	}

#if !defined(UAC2_ALIF)
	config.mem_slab = &i2s_rx_slab;
	ret = i2s_configure(main_ctx.i2s_dev, I2S_DIR_RX, &config);
	if (ret < 0) {
		printk("Failed to configure RX stream: %d\n", ret);
		return 0;
	}
#endif

#if defined(UAC2_ALIF) && DT_HAS_ALIAS(pdm_audio)
	ret = pdm_init();
	if (ret < 0) {
		return ret;
	}
#endif

	main_ctx.fb = feedback_init();
#if defined(UAC2_ALIF)
	feedback_bind_slab(main_ctx.fb, &i2s_tx_slab);
#endif

	usbd_uac2_set_ops(dev, &usb_audio_ops, &main_ctx);

	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		return -ENODEV;
	}

	ret = usbd_enable(sample_usbd);
	if (ret) {
		return ret;
	}

	return 0;
}
