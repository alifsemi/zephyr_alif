/*
 * Copyright (c) 2023-2024 Nordic Semiconductor ASA
 * Copyright (C) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdlib.h>

#include <sample_usbd.h>
#include "feedback.h"

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_uac2.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#if IS_ENABLED(CONFIG_AUDIO_CODEC)
#include <zephyr/audio/codec.h>
#endif

LOG_MODULE_REGISTER(uac2_sample, LOG_LEVEL_INF);

#define HEADPHONES_OUT_TERMINAL_ID UAC2_ENTITY_ID(DT_NODELABEL(out_terminal))

#define SAMPLE_FREQUENCY    (SAMPLES_PER_SOF * 1000)
#define SAMPLE_BIT_WIDTH    16
#define NUMBER_OF_CHANNELS  2
#define BYTES_PER_SAMPLE    DIV_ROUND_UP(SAMPLE_BIT_WIDTH, 8)
#define BYTES_PER_SLOT      (BYTES_PER_SAMPLE * NUMBER_OF_CHANNELS)
#define MIN_BLOCK_SIZE      ((SAMPLES_PER_SOF - 1) * BYTES_PER_SLOT)
#define BLOCK_SIZE          (SAMPLES_PER_SOF * BYTES_PER_SLOT)
#define MAX_BLOCK_SIZE      ((SAMPLES_PER_SOF + 1) * BYTES_PER_SLOT)

/* Absolute minimum is 5 buffers (1 actively consumed by I2S, 2nd queued as next
 * buffer, 3rd acquired by USB stack to receive data to, and 2 to handle SOF/I2S
 * offset errors), but add 2 additional buffers to prevent out of memory errors
 * when USB host decides to perform rapid terminal enable/disable cycles.
 *
 * Four USB OUT requests can own four slab blocks at once.
 */
#define I2S_BUFFERS_COUNT   16
K_MEM_SLAB_DEFINE_STATIC(i2s_tx_slab, ROUND_UP(MAX_BLOCK_SIZE, UDC_BUF_GRANULARITY),
			 I2S_BUFFERS_COUNT, UDC_BUF_ALIGN);

struct usb_i2s_ctx {
	const struct device *i2s_dev;
	bool terminal_enabled;
	bool i2s_started;
	bool microframes;
	/* Number of blocks written, used to determine when to start I2S.
	 * Overflows are not a problem becuse this variable is not necessary
	 * after I2S is started.
	 */
	uint8_t i2s_blocks_written;
	struct feedback_ctx *fb;
};

static void uac2_terminal_update_cb(const struct device *dev, uint8_t terminal,
				    bool enabled, bool microframes,
				    void *user_data)
{
	struct usb_i2s_ctx *ctx = user_data;

	/* This sample has only one terminal therefore the callback can simply
	 * ignore the terminal variable.
	 */
	__ASSERT_NO_MSG(terminal == HEADPHONES_OUT_TERMINAL_ID);

	/* False at Full-Speed. True when the host enumerates this device
	 * at High-Speed.
	 */
	ctx->microframes = microframes;
	ctx->terminal_enabled = enabled;

	if (!enabled) {
		if (ctx->i2s_started) {
			i2s_trigger(ctx->i2s_dev,
				    I2S_DIR_TX,
				    I2S_TRIGGER_DROP);
		}

		ctx->i2s_started = false;
		ctx->i2s_blocks_written = 0;

		feedback_reset_ctx(ctx->fb);
		return;
	}

	#if defined(CONFIG_SOC_FAMILY_BALLETTO) || \
	defined(CONFIG_SOC_FAMILY_ENSEMBLE)
	if (!ctx->i2s_started) {
		feedback_set_speed(ctx->fb, microframes);
	}
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

		if (!ctx->terminal_enabled) {
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
	int ret;

	if (!ctx->terminal_enabled) {
		k_mem_slab_free(&i2s_tx_slab, buf);
		return;
	}

	if (!size) {
		/* Zero fill to keep I2S going. If this is transient error, then
		 * this is probably best we can do. Otherwise, host will likely
		 * either disable terminal (or the cable will be disconnected)
		 * which will stop I2S.
		 *
		 * One lost HS microframe is six stereo samples at 48 kHz.
		 * One lost FS frame is 48 samples.
		 */
		size = ctx->microframes ? BLOCK_SIZE / 8U : BLOCK_SIZE;
		memset(buf, 0, size);
		sys_cache_data_flush_range(buf, size);
	}

	LOG_DBG("Received %d data to input terminal %d", size, terminal);

	ret = i2s_write(ctx->i2s_dev, buf, size);
	if (ret < 0) {
		ctx->i2s_started = false;
		ctx->i2s_blocks_written = 0;
		feedback_reset_ctx(ctx->fb);

		/* Most likely underrun occurred, prepare I2S restart */
		i2s_trigger(ctx->i2s_dev, I2S_DIR_TX, I2S_TRIGGER_PREPARE);

		ret = i2s_write(ctx->i2s_dev, buf, size);
		if (ret < 0) {
			/* Drop data block, will try again on next frame */
			k_mem_slab_free(&i2s_tx_slab, buf);
		}
	}

	if (ret == 0) {
		ctx->i2s_blocks_written++;
	}
}

static void uac2_buf_release_cb(const struct device *dev, uint8_t terminal,
				void *buf, void *user_data)
{
	/* This sample does not send audio data so this won't be called */
}

/* Variables for debug use to facilitate simple how feedback value affects
 * audio data rate experiments. These debug variables can also be used to
 * determine how well the feedback regulator deals with errors. The values
 * are supposed to be modified by debugger.
 *
 * Setting use_hardcoded_feedback to true, essentially bypasses the feedback
 * regulator and makes host send hardcoded_feedback samples every 16384 SOFs
 * (when operating at Full-Speed).
 *
 * The feedback at Full-Speed is Q10.14 value. For 48 kHz audio sample rate,
 * there are nominally 48 samples every SOF. The corresponding value is thus
 * 48 << 14. Such feedback value would result in host sending always 48 samples.
 * Now, if we want to receive more samples (because 1 ms according to audio
 * sink is shorter than 1 ms according to USB Host 500 ppm SOF timer), then
 * the feedback value has to be increased. The fractional part is 14-bit wide
 * and therefore increment by 1 means 1 additional sample every 2**14 SOFs.
 * (48 << 14) + 1 therefore results in host sending 48 samples 16383 times and
 * 49 samples 1 time during every 16384 SOFs.
 *
 * Similarly, if we want to receive less samples (because 1 ms according to
 * audio signk is longer than 1 ms according to USB Host), then the feedback
 * value has to be decreased. (48 << 14) - 1 therefore results in host sending
 * 48 samples 16383 times and 47 samples 1 time during every 16384 SOFs.
 *
 * If the feedback value differs by more than 1 (i.e. LSB), then the +1/-1
 * samples packets are generally evenly distributed. For example feedback value
 * (48 << 14) + (1 << 5) results in 48 samples 511 times and 49 samples 1 time
 * during every 512 SOFs.
 *
 * For High-Speed above changes slightly, because the feedback format is Q16.16
 * and microframes are used. The 48 kHz audio sample rate is achieved by sending
 * 6 samples every SOF (microframe). The nominal value is the average number of
 * samples to send every microframe and therefore for 48 kHz the nominal value
 * is (6 << 16).
 */
static volatile bool use_hardcoded_feedback;
static volatile uint32_t hardcoded_feedback = (48 << 14) + 1;

static uint32_t uac2_feedback_cb(const struct device *dev, uint8_t terminal,
				 void *user_data)
{
	/* Sample has only one UAC2 instance with one terminal so both can be
	 * ignored here.
	 */
	ARG_UNUSED(dev);
	ARG_UNUSED(terminal);
	struct usb_i2s_ctx *ctx = user_data;

	if (use_hardcoded_feedback) {
		/* High-Speed feedback is Q16.16 samples per microframe.
		 * Full-Speed stays on the Q10.14 value above.
		 */
		if (ctx->microframes) {
			return 6U << 16;
		}
		return hardcoded_feedback;
	}

	return feedback_value(ctx->fb);
}

static void uac2_sof(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	struct usb_i2s_ctx *ctx = user_data;

	if (ctx->i2s_started) {
		feedback_process(ctx->fb);
	}

	/*
	 * Start I2S after 4 packets are queued. On High-Speed each packet is
	 * one 125 us microframe, so this is 0.5 ms of audio. The USB stack
	 * holds 4 more slab blocks, and the slab has 16, so the threshold
	 * has to stay well below 12 or I2S never starts.
	 */
	if (!ctx->i2s_started &&
	    ctx->terminal_enabled &&
	    ctx->i2s_blocks_written >= 4) {

		int ret = i2s_trigger(ctx->i2s_dev,
				      I2S_DIR_TX,
				      I2S_TRIGGER_START);

		if (ret == 0) {
			ctx->i2s_started = true;

			feedback_start(ctx->fb,
				       ctx->i2s_blocks_written);
		} else {
			LOG_ERR("I2S START failed: %d", ret);
		}
	}
}

static struct uac2_ops usb_audio_ops = {
	.sof_cb = uac2_sof,
	.terminal_update_cb = uac2_terminal_update_cb,
	.get_recv_buf = uac2_get_recv_buf,
	.data_recv_cb = uac2_data_recv_cb,
	.buf_release_cb = uac2_buf_release_cb,
	.feedback_cb = uac2_feedback_cb,
};

static struct usb_i2s_ctx main_ctx;

#if IS_ENABLED(CONFIG_AUDIO_CODEC)
static int configure_codec(const struct device *codec_dev)
{
	struct audio_codec_cfg audio_cfg = {0};
	int ret;

	audio_cfg.dai_route = AUDIO_ROUTE_PLAYBACK;
	audio_cfg.dai_type = AUDIO_DAI_TYPE_I2S;

	/* Must match the UAC2/I2S configuration */
	audio_cfg.dai_cfg.i2s.word_size = SAMPLE_BIT_WIDTH;      /* 16 */
	audio_cfg.dai_cfg.i2s.channels = NUMBER_OF_CHANNELS;     /* 2 */
	audio_cfg.dai_cfg.i2s.format = I2S_FMT_DATA_FORMAT_I2S;

	/*
	 * I2S_OPT_FRAME_CLK_MASTER is defined as 0, so this assignment does
	 * not select a clock direction by itself. The WM8904 driver still
	 * calls wm8904_set_master_clock() and drives BCLK and LRCLK.
	 * I2S_OPT_FRAME_CLK_SLAVE would make the codec a clock input.
	 */
	audio_cfg.dai_cfg.i2s.options =
	I2S_OPT_BIT_CLK_SLAVE |
	I2S_OPT_FRAME_CLK_SLAVE;

	audio_cfg.dai_cfg.i2s.frame_clk_freq = SAMPLE_FREQUENCY; /* 48000 */

	audio_cfg.dai_cfg.i2s.mem_slab = &i2s_tx_slab;
	audio_cfg.dai_cfg.i2s.block_size = MAX_BLOCK_SIZE;
	audio_cfg.dai_cfg.i2s.timeout = 0;

	ret = audio_codec_configure(codec_dev, &audio_cfg);
	if (ret < 0) {
		printk("Failed to configure codec: %d\n", ret);
		return ret;
	}

	audio_codec_start_output(codec_dev);

	printk("Codec configured and output started\n");

	return 0;
}
#endif

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(uac2_headphones));
	struct usbd_context *sample_usbd;
	struct i2s_config config;
	int ret;

	main_ctx.i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s_tx));

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

	main_ctx.fb = feedback_init();
	#if defined(CONFIG_SOC_FAMILY_BALLETTO) || \
	defined(CONFIG_SOC_FAMILY_ENSEMBLE)
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
