/*
 * Copyright (c) 2026 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT himax_hm0360

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hm0360);

#include <zephyr/drivers/video/hm0360-video-controls.h>

#define REG_ENTRY(addr, ...)  { (addr), (uint8_t[]){__VA_ARGS__}, sizeof((uint8_t[]){__VA_ARGS__}) }

/* Register definitions */
#define HM0360_CHIP_ID_VAL             0x0360
#define HM0360_CHIP_ID_H_REGISTER      0x0000
#define HM0360_CHIP_ID_L_REGISTER      0x0001
#define HM0360_MODE_SELECT_REGISTER    0x0100
#define HM0360_SOFTWARE_RESET_REGISTER 0x0103
#define HM0360_PMU_CFG_7               0x3028 /* Output frame count */

/* Mode select bitfields */
enum hm0360_mode {
	HM0360_MODE_I2C_TRIGGER_SLEEP             = 0x00,
	HM0360_MODE_I2C_TRIGGER_CONT_STREAMING    = 0x01,
	HM0360_MODE_I2C_TRIGGER_AUTO_WAKEUP       = 0x02,
	HM0360_MODE_I2C_TRIGGER_SNAPSHOT_N_FRAMES = 0x03,
	HM0360_MODE_HW_TRIGGER_SLEEP              = 0x04,
	HM0360_MODE_HW_TRIGGER_CONT_STREAMING     = 0x05,
	HM0360_MODE_HW_TRIGGER_AUTO_WAKEUP        = 0x06,
	HM0360_MODE_HW_TRIGGER_SNAPSHOT_N_FRAMES  = 0x07
};

/* Data structures */
struct hm0360_burst_reg {
	uint16_t addr;
	uint8_t *data;
	uint32_t count;
};

struct hm0360_config {
	struct i2c_dt_spec i2c;
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	const struct gpio_dt_spec resetn_gpio;	/* Active low Reset (XSHUTDOWN) */
#endif
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
	const struct gpio_dt_spec power_gpio;
#endif
#if DT_INST_NODE_HAS_PROP(0, xsleep_gpios)
	const struct gpio_dt_spec xsleepn_gpio; /* Active low LP Sleep mode (XSLEEP) */
#endif
	uint8_t fps;
};

enum capture_type {
	CONTINUOUS_CAPTURE,
	SNAPSHOT_CAPTURE,
};

struct hm0360_data {
	struct video_format fmt;
	enum capture_type capture_mode;
	uint8_t numframes;
};

struct hm0360_resolution_config {
	uint16_t width;
	uint16_t height;
	uint8_t fps;
	uint16_t size_params;
	const struct hm0360_burst_reg *params;
};

/* Helper macro for video format capabilities */
#define HM0360_VIDEO_FORMAT_CAP(w, h, f) { \
	.pixelformat = (f), .width_min = (w), .width_max = (w), \
	.height_min = (h), .height_max = (h), .width_step = 0, .height_step = 0 \
}

/* Resolution configurations */

static const struct hm0360_burst_reg hm0360_320x240_60fps[] = {
	REG_ENTRY(0x3024, 0x01), /* Context-A, I2C trigger */
	REG_ENTRY(0x3588, 0x02, /* Frame_rate1_H */
		  0x12, /* frame_rate1_L */
		  0x04, /* frame_rate2_H */
		  0x24, /* frame_rate2_L */
		  0x06, /* frame_rate3_H */
		  0x36), /* frame_rate3_L */
};

static const struct hm0360_burst_reg hm0360_640x480_60fps[] = {
	REG_ENTRY(0x3024, 0x00), /* Context-A, I2C trigger */
	REG_ENTRY(0x3030, 0x01),
	REG_ENTRY(0x352E, 0x02, /* Frame_rate1_H */
		  0x14, /* frame_rate1_L */
		  0x04, /* frame_rate2_H */
		  0x10, /* frame_rate2_L */
		  0x06, /* frame_rate3_H */
		  0x1A), /* frame_rate3_L */
};

static const struct hm0360_resolution_config resolution_configs[] = {
	{
		.width = 320, .height = 240, .fps = 60,
		.params = hm0360_320x240_60fps,
		.size_params = ARRAY_SIZE(hm0360_320x240_60fps)
	},
	{
		.width = 640, .height = 480, .fps = 60,
		.params = hm0360_640x480_60fps,
		.size_params = ARRAY_SIZE(hm0360_640x480_60fps)
	},
};

static const struct video_format_cap fmts[] = {
	HM0360_VIDEO_FORMAT_CAP(320, 240, VIDEO_PIX_FMT_BGGR8),
	HM0360_VIDEO_FORMAT_CAP(640, 480, VIDEO_PIX_FMT_BGGR8),
	{0},
};

/* Initialization configuration */
static const struct hm0360_burst_reg hm0360_init_config[] = {
	REG_ENTRY(0x0102, 0x00),
	REG_ENTRY(0x0350, 0xE0),
	REG_ENTRY(0x0370, 0x00,
		  0x00, 0x01),
	REG_ENTRY(0x1000, 0x43,
		  0x80),
	REG_ENTRY(0x1003, 0x20,
		  0x20),
	REG_ENTRY(0x1007, 0x01,
		  0x20, 0x20,
		  0x05, 0x20,
		  0x20),
	REG_ENTRY(0x1014, 0x00),
	REG_ENTRY(0x1013, 0x00,
		  0x00),
	REG_ENTRY(0x1018, 0x00),
	REG_ENTRY(0x101D, 0xCF,
		  0x01, 0x00,
		  0x01, 0x5D),
	REG_ENTRY(0x102F, 0x08,
		  0x04, 0x08,
		  0x10, 0x18,
		  0x20, 0x28,
		  0x30, 0x38,
		  0x40, 0x50,
		  0x60, 0x70,
		  0x80, 0xA0,
		  0xC0, 0xE0),
	REG_ENTRY(0x1041, 0x00),
	REG_ENTRY(0x2000, 0x7F),
	REG_ENTRY(0x202B, 0x04,
		  0x03, 0x00),
	REG_ENTRY(0x2031, 0x60,
		  0x08),
	REG_ENTRY(0x2034, 0x70),
	REG_ENTRY(0x2036, 0x19,
		  0x08, 0x10),
	REG_ENTRY(0x203C, 0x01,
		  0x04, 0x01,
		  0x38),
	REG_ENTRY(0x2048, 0x00,
		  0x10, 0x40,
		  0x00, 0x08,
		  0x20, 0x00,
		  0x38, 0xE0,
		  0x00, 0x1C,
		  0x70, 0x00,
		  0x1A, 0xC0,
		  0x00, 0x06,
		  0xB0),
	REG_ENTRY(0x2061, 0x00,
		  0x00, 0xC8),
	REG_ENTRY(0x2080, 0x41,
		  0xE0, 0xF0,
		  0x01, 0x10,
		  0x10, 0x01,
		  0x06, 0x0C,
		  0x12, 0x1C,
		  0x30, 0x10,
		  0x02, 0x08,
		  0x0D, 0x14,
		  0x1D, 0x30,
		  0x08, 0x0A,
		  0x0F, 0x14,
		  0x18, 0x20,
		  0x10, 0x00,
		  0x01, 0x01,
		  0x11, 0x06,
		  0x20, 0x10),
	REG_ENTRY(0x2590, 0x01),
	REG_ENTRY(0x2800, 0x09),
	REG_ENTRY(0x2804, 0x02,
		  0x03, 0x03,
		  0x08, 0x04,
		  0x0C, 0x03),
	REG_ENTRY(0x280F, 0x03,
		  0x03, 0x00,
		  0x09),
	REG_ENTRY(0x2821, 0xEE),
	REG_ENTRY(0x282A, 0x0F,
		  0x08),
	REG_ENTRY(0x282E, 0x2F),
	REG_ENTRY(0x3010, 0x00),
	REG_ENTRY(0x3013, 0x01),
	REG_ENTRY(0x3019, 0x00,
		  0x00, 0x20,
		  0xFF),
	REG_ENTRY(0x3020, 0x00,
		  0x00),
	REG_ENTRY(0x3024, 0x00,
		  0x12, 0x03,
		  0x81, 0x01,
		  0x15, 0x60,
		  0x2A, 0x00,
		  0x03, 0x00,
		  0x00),
	REG_ENTRY(0x3031, 0x01),
	REG_ENTRY(0x3034, 0x00,
		  0x01),
	REG_ENTRY(0x3051, 0x00),
	REG_ENTRY(0x305C, 0x03),
	REG_ENTRY(0x3060, 0x00,
		  0xFA, 0xFF,
		  0xFF, 0xFF,
		  0xFF, 0xFF,
		  0xFF, 0xFF,
		  0xFF, 0xFF,
		  0xFF, 0xFF,
		  0xFF, 0xFF,
		  0xFF, 0xFF,
		  0xFF, 0xFF,
		  0xFF, 0xFF,
		  0xFF, 0xFF,
		  0xFF, 0xFF,
		  0xFF, 0xFF,
		  0xFF, 0xFF,
		  0xFF, 0xFF,
		  0xFF, 0x00,
		  0x00, 0x00,
		  0x20, 0x00,
		  0x20, 0x00,
		  0x20, 0x00,
		  0x04),
	REG_ENTRY(0x3094, 0x02,
		  0x02, 0x00,
		  0x02, 0x00,
		  0x02),
	REG_ENTRY(0x309E, 0x05,
		  0x02, 0x02,
		  0x00, 0x08,
		  0x00, 0x20,
		  0x04, 0x02,
		  0x02, 0x02,
		  0x00, 0x02,
		  0x34),
	REG_ENTRY(0x30B0, 0x03),
	REG_ENTRY(0x30C4, 0x10,
		  0x01, 0x2F,
		  0x00, 0x00),
	REG_ENTRY(0x30CB, 0xFF,
		  0xFF, 0x7F,
		  0x7F),
	REG_ENTRY(0x30D3, 0x01,
		  0xFF, 0x00,
		  0x40, 0x00,
		  0xA7, 0x00,
		  0x01, 0x40,
		  0x00, 0x27,
		  0x05, 0x07,
		  0x40, 0x00,
		  0x27, 0x05,
		  0x47, 0x30,
		  0x00, 0x27,
		  0x05, 0x87,
		  0x30, 0x00,
		  0x27, 0x05,
		  0x00, 0x40,
		  0x00, 0xA7,
		  0x00, 0x01,
		  0x40, 0x00,
		  0x27, 0x05,
		  0x07, 0x40,
		  0x00, 0x27,
		  0x05, 0x47,
		  0x30, 0x00,
		  0x27, 0x05,
		  0x87, 0x30,
		  0x00, 0x27,
		  0x05),
	REG_ENTRY(0x310B, 0x10),
	REG_ENTRY(0x3112, 0x04,
		  0xA0, 0x67,
		  0x42, 0x10,
		  0x0A, 0x3F),
	REG_ENTRY(0x311A, 0x30),
	REG_ENTRY(0x311C, 0x10,
		  0x06, 0x0F,
		  0x0E, 0x0D,
		  0x0F, 0x00,
		  0x1D),
	REG_ENTRY(0x3126, 0x03,
		  0xC4, 0x57),
	REG_ENTRY(0x312A, 0x11,
		  0x41),
	REG_ENTRY(0x312E, 0x00,
		  0x00, 0x0C),
	REG_ENTRY(0x3141, 0x2A,
		  0x9F),
	REG_ENTRY(0x3147, 0x18),
	REG_ENTRY(0x3149, 0x28),
	REG_ENTRY(0x314B, 0x01),
	REG_ENTRY(0x3150, 0x50),
	REG_ENTRY(0x3152, 0x00),
	REG_ENTRY(0x3156, 0x2C),
	REG_ENTRY(0x315A, 0x0A,
		  0x2F, 0xE0),
	REG_ENTRY(0x315F, 0x02,
		  0x1F),
	REG_ENTRY(0x3163, 0x1F,
		  0x7F, 0x7F),
	REG_ENTRY(0x317B, 0x94,
		  0x00, 0x02),
	REG_ENTRY(0x318C, 0x00),
	REG_ENTRY(0x3500, 0x78,
		  0x0A, 0x77,
		  0x02, 0x14,
		  0x03, 0x00,
		  0x00, 0x00,
		  0x00, 0xFF,
		  0x00, 0x00,
		  0x01),
	REG_ENTRY(0x350F, 0x00,
		  0x02, 0x00,
		  0x7F, 0x00,
		  0x00, 0x01,
		  0x00, 0x02,
		  0x00, 0x7F,
		  0x00, 0x5F,
		  0x00, 0x02,
		  0x10, 0x04,
		  0x03, 0x00),
	REG_ENTRY(0x3523, 0x60,
		  0x08, 0x19,
		  0x08, 0x10),
	REG_ENTRY(0x352A, 0x01,
		  0x04, 0x01,
		  0x39),
	REG_ENTRY(0x3535, 0x02,
		  0x03, 0x03,
		  0x08, 0x04,
		  0x0C, 0x03),
	REG_ENTRY(0x3540, 0x03,
		  0x03, 0x00,
		  0x09),
	REG_ENTRY(0x3549, 0x04,
		  0x35, 0x21,
		  0x01, 0xE0,
		  0xF0, 0x10,
		  0x10, 0x10,
		  0x20, 0x10,
		  0x01, 0x06,
		  0x0C, 0x12,
		  0x1C, 0x30,
		  0x78, 0x0A,
		  0x77, 0x01,
		  0x1C, 0x03,
		  0x00, 0x01,
		  0x01, 0x00,
		  0xFF, 0x00,
		  0x00, 0x01),
	REG_ENTRY(0x3569, 0x00,
		  0x02, 0x00,
		  0x7F, 0x00,
		  0x00, 0x01,
		  0x00, 0x02,
		  0x00, 0x3F,
		  0x00, 0x2F,
		  0x00, 0x02,
		  0x24, 0x04,
		  0x03, 0x00),
	REG_ENTRY(0x357D, 0x60,
		  0x08, 0x19,
		  0x08, 0x10),
	REG_ENTRY(0x3584, 0x01,
		  0x04, 0x01,
		  0x39),
	REG_ENTRY(0x358F, 0x02,
		  0x03, 0x03,
		  0x08, 0x04,
		  0x0C, 0x03),
	REG_ENTRY(0x359A, 0x03,
		  0x03, 0x00,
		  0x09),
	REG_ENTRY(0x35A3, 0x02,
		  0x03, 0x21,
		  0x01, 0xE0,
		  0xF0, 0x10,
		  0x10, 0x10,
		  0x20, 0x10,
		  0x01, 0x06,
		  0x0C, 0x12,
		  0x1C, 0x30,
		  0x78, 0x0A,
		  0x77, 0x00,
		  0x94, 0x03,
		  0x00, 0x03,
		  0x03, 0x00,
		  0xFF, 0x00,
		  0x01, 0x01),
	REG_ENTRY(0x35C3, 0x00,
		  0x00, 0x00,
		  0x7F, 0x00,
		  0x00, 0x01,
		  0x00, 0x02,
		  0x00, 0x0F,
		  0x00, 0x0B,
		  0x00),
	REG_ENTRY(0x35D3, 0x04),
	REG_ENTRY(0x35D7, 0x18,
		  0x01, 0x20,
		  0x08, 0x14,
		  0x70),
	REG_ENTRY(0x35DE, 0x00,
		  0x01),
	REG_ENTRY(0x35E9, 0x02,
		  0x03, 0x03,
		  0x08, 0x04,
		  0x0C, 0x03),
	REG_ENTRY(0x35F4, 0x03,
		  0x03, 0x00,
		  0x09),
	REG_ENTRY(0x35FD, 0x00,
		  0x5E),
	REG_ENTRY(0x0104, 0x01),
	REG_ENTRY(0x0100, 0x01),
};

/* I2C Helper Functions */
static int hm0360_burst_write(const struct device *dev, uint16_t start_addr,
		const uint8_t *buf, uint32_t num_bytes)
{
	const struct hm0360_config *cfg = dev->config;
	uint8_t addr_buffer[2];
	struct i2c_msg msg[2];

	addr_buffer[0] = start_addr >> 8;
	addr_buffer[1] = start_addr & 0xFF;

	msg[0].buf = addr_buffer;
	msg[0].len = 2;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)buf;
	msg[1].len = num_bytes;
	msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer_dt(&cfg->i2c, msg, 2);
}

static int hm0360_read_reg(const struct device *dev, uint16_t addr, uint8_t *val)
{
	const struct hm0360_config *cfg = dev->config;
	uint8_t addr_buffer[2];

	addr_buffer[0] = addr >> 8;
	addr_buffer[1] = addr & 0xFF;

	return i2c_write_read_dt(&cfg->i2c, addr_buffer, sizeof(addr_buffer), val, 1);
}

static int hm0360_write_reg(const struct device *dev, uint16_t addr, uint8_t val)
{
	return hm0360_burst_write(dev, addr, &val, 1);
}

static int hm0360_write_regs(const struct device *dev,
		const struct hm0360_burst_reg *regs, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		int ret = hm0360_burst_write(dev, regs[i].addr, regs[i].data, regs[i].count);

		if (ret) {
			LOG_ERR("Failed to write reg 0x%04x: %d", regs[i].addr, ret);
			return ret;
		}
	}
	return 0;
}

/* Hardware Control Functions */
static int hm0360_hw_reset(const struct device *dev)
{
	const struct hm0360_config *cfg = dev->config;
	int ret;

	/* Configure GPIOs */
	ret = gpio_pin_configure_dt(&cfg->resetn_gpio, GPIO_OUTPUT_ACTIVE);
	ret |= gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_INACTIVE);
	ret |= gpio_pin_configure_dt(&cfg->xsleepn_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		LOG_ERR("GPIO configuration failed");
		return ret;
	}

	/* Reset sequence */
	k_usleep(100);
	gpio_pin_set_dt(&cfg->power_gpio, 1);
	gpio_pin_set_dt(&cfg->resetn_gpio, 0);
	k_usleep(400);
	gpio_pin_set_dt(&cfg->xsleepn_gpio, 0);
	k_usleep(39);

	return 0;
}

static int hm0360_software_reset(const struct device *dev)
{
	int ret = hm0360_write_reg(dev, HM0360_SOFTWARE_RESET_REGISTER, 0);

	if (ret) {
		LOG_ERR("Soft-reset failed: %d", ret);
		return ret;
	}

	return 0;
}

static int hm0360_validate_chipid(const struct device *dev)
{
	uint16_t chipid = 0;
	uint8_t val;
	int ret;

	ret = hm0360_read_reg(dev, HM0360_CHIP_ID_L_REGISTER, &val);
	if (ret) {
		return ret;
	}
	chipid = val;

	ret = hm0360_read_reg(dev, HM0360_CHIP_ID_H_REGISTER, &val);
	if (ret) {
		return ret;
	}
	chipid |= val << 8;

	if (chipid != HM0360_CHIP_ID_VAL) {
		LOG_ERR("Invalid Chip-ID: 0x%x", chipid);
		return -ENOTSUP;
	}

	return 0;
}

static bool hm0360_validate_format(const struct video_format *fmt)
{
	/* Validate format */
	for (size_t i = 0; fmts[i].pixelformat; i++) {
		if (fmt->pixelformat == fmts[i].pixelformat &&
			fmt->width == fmts[i].width_min &&
			fmt->height == fmts[i].height_min) {
			return true;
		}
	}
	return false;
}

/* Video API Implementation */
static int hm0360_set_fmt(const struct device *dev, enum video_endpoint_id ep,
						 struct video_format *fmt)
{
	struct hm0360_data *drv_data = dev->data;
	const struct hm0360_config *cfg = dev->config;
	int ret;

	/* Skip if format hasn't changed */
	if (!memcmp(&drv_data->fmt, fmt, sizeof(*fmt))) {
		return 0;
	}

	if (!hm0360_validate_format(fmt)) {
		LOG_ERR("Unsupported format/resolution");
		return -ENOTSUP;
	}

	/* Find and apply resolution config */
	for (size_t i = 0; i < ARRAY_SIZE(resolution_configs); i++) {
		const struct hm0360_resolution_config *res = &resolution_configs[i];

		if (fmt->width == res->width &&
			fmt->height == res->height &&
			cfg->fps == res->fps) {
			ret = hm0360_write_regs(dev, res->params, res->size_params);
			if (ret) {
				return ret;
			}

			drv_data->fmt = *fmt;
			LOG_DBG("Format set: %dx%d, pixfmt: 0x%x",
					fmt->width, fmt->height, fmt->pixelformat);
			return 0;
		}
	}

	return -ENOTSUP;
}

static int hm0360_get_fmt(const struct device *dev, enum video_endpoint_id ep,
		struct video_format *fmt)
{
	struct hm0360_data *drv_data = dev->data;

	*fmt = drv_data->fmt;
	return 0;
}

static int hm0360_stream_start(const struct device *dev)
{
	struct hm0360_data *drv_data = dev->data;
	uint8_t mode;
	int ret;

	switch (drv_data->capture_mode) {
	case CONTINUOUS_CAPTURE:
		mode = HM0360_MODE_I2C_TRIGGER_CONT_STREAMING;
		break;
	case SNAPSHOT_CAPTURE:
		/* Set frame count first */
		ret = hm0360_write_reg(dev, HM0360_PMU_CFG_7, drv_data->numframes);
		if (ret) {
			return ret;
		}
		mode = HM0360_MODE_I2C_TRIGGER_SNAPSHOT_N_FRAMES;
		break;
	default:
		return -ENOTSUP;
	}

	return hm0360_write_reg(dev, HM0360_MODE_SELECT_REGISTER, mode);
}

static int hm0360_stream_stop(const struct device *dev)
{
	return hm0360_write_reg(dev, HM0360_MODE_SELECT_REGISTER, HM0360_MODE_I2C_TRIGGER_SLEEP);
}

static int hm0360_stream_set(const struct device *dev, bool enable)
{
	if (enable) {
		return hm0360_stream_start(dev);
	} else {
		return hm0360_stream_stop(dev);
	}
}

static int hm0360_get_caps(const struct device *dev, enum video_endpoint_id ep,
		struct video_caps *caps)
{
	caps->format_caps = fmts;
	return 0;
}

static int hm0360_set_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	struct hm0360_data *drv_data = dev->data;

	switch (cid) {
	case VIDEO_HM0360_CID_SNAPSHOT_CAPTURE:
		drv_data->capture_mode = SNAPSHOT_CAPTURE;
		drv_data->numframes = *(uint8_t *)value;
		LOG_DBG("Snapshot mode, frames: %d", drv_data->numframes);
		return 0;
	case VIDEO_HM0360_CID_CONTINUOUS_CAPTURE:
		drv_data->capture_mode = CONTINUOUS_CAPTURE;
		drv_data->numframes = 0;
		LOG_DBG("Continuous mode");
		return 0;
	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(video, hm0360_driver_api) = {
	.set_format = hm0360_set_fmt,
	.get_format = hm0360_get_fmt,
	.get_caps = hm0360_get_caps,
	.set_stream = hm0360_stream_set,
	.set_ctrl = hm0360_set_ctrl,
};

static int hm0360_init(const struct device *dev)
{
	const struct hm0360_config *cfg = dev->config;
	int ret;

	/* Hardware reset */
	ret = hm0360_hw_reset(dev);
	if (ret) {
		LOG_ERR("HW reset failed: %d", ret);
		return ret;
	}

	/* Validate chip ID */
	ret = hm0360_validate_chipid(dev);
	if (ret) {
		return ret;
	}

	/* Software reset */
	ret = hm0360_software_reset(dev);
	if (ret) {
		return ret;
	}

	/* Load initial configuration */
	ret = hm0360_write_regs(dev, hm0360_init_config,
			ARRAY_SIZE(hm0360_init_config));
	if (ret) {
		LOG_ERR("Init config failed: %d", ret);
		return ret;
	}

	ret = hm0360_stream_stop(dev);
	if (ret) {
		return ret;
	}

	/* Set default format */
	struct video_format fmt = {
		.pixelformat = VIDEO_PIX_FMT_BGGR8,
		.width = 640,
		.height = 480,
		.pitch = 640
	};

	ret = hm0360_set_fmt(dev, VIDEO_EP_OUT, &fmt);
	if (ret) {
		return ret;
	}

	/* HM0360 can be configured @ 1MHz, but Stream ON/OFF commands are not
	 * sent after that. So, configuring the sensor at 1MHz, but switching
	 * to lower speed (400 kHz) after configuration for normal operation.
	 */
	return i2c_configure(cfg->i2c.bus, I2C_SPEED_SET(I2C_SPEED_FAST));
}

/* Device configuration */
static const struct hm0360_config hm0360_config_0 = {
	.i2c = I2C_DT_SPEC_INST_GET(0),
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	.resetn_gpio = GPIO_DT_SPEC_INST_GET_OR(0, reset_gpios, {0}),
#endif
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
	.power_gpio = GPIO_DT_SPEC_INST_GET_OR(0, power_gpios, {0}),
#endif
#if DT_INST_NODE_HAS_PROP(0, xsleep_gpios)
	.xsleepn_gpio = GPIO_DT_SPEC_INST_GET_OR(0, xsleep_gpios, {0}),
#endif
	.fps = DT_INST_PROP_OR(0, fps, 60),
};

static struct hm0360_data hm0360_data_0 = {
	.capture_mode = CONTINUOUS_CAPTURE,
	.numframes = 1,
};

DEVICE_DT_INST_DEFINE(0, hm0360_init, NULL, &hm0360_data_0, &hm0360_config_0,
		POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, &hm0360_driver_api);
