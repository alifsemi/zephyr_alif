/*
 * SPDX-FileCopyrightText: Copyright Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ovti_ov5675

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ov5675);

#define OV5675_CHIP_ID_VAL       0x5675

#define OV5675_REG_CHIPID_H      0x300b
#define OV5675_REG_CHIPID_L      0x300c

#define OV5675_REG_MODE_SELECT   0x0100
#define OV5675_MODE_STANDBY      0x00
#define OV5675_MODE_STREAMING    0x01

struct ov5675_config {
	const struct gpio_dt_spec reset_gpio;
	const struct gpio_dt_spec power_gpio;
	struct i2c_dt_spec i2c;
};

struct ov5675_data {
	struct video_format fmt;
	bool is_streaming;
};

struct ov5675_reg {
	uint16_t addr;
	uint8_t value;
};

/* OV5675 1296x972 10-bit RAW register configuration */
static const struct ov5675_reg ov5675_1296x972_regs[] = {
	{0x0103, 0x01}, {0x0100, 0x00}, {0x0300, 0x04}, {0x0302, 0x8d},
	{0x0303, 0x00}, {0x030d, 0x26}, {0x3002, 0x21}, {0x3107, 0x23},
	{0x3501, 0x20}, {0x3503, 0x0c}, {0x3508, 0x03}, {0x3509, 0x00},
	{0x3600, 0x66}, {0x3602, 0x30}, {0x3610, 0xa5}, {0x3612, 0x93},
	{0x3620, 0x80}, {0x3642, 0x0e}, {0x3661, 0x00}, {0x3662, 0x08},
	{0x3664, 0xf3}, {0x3665, 0x9e}, {0x3667, 0xa5}, {0x366e, 0x55},
	{0x366f, 0x55}, {0x3670, 0x11}, {0x3671, 0x11}, {0x3672, 0x11},
	{0x3673, 0x11}, {0x3714, 0x28}, {0x371a, 0x3e}, {0x3733, 0x10},
	{0x3734, 0x00}, {0x373d, 0x24}, {0x3764, 0x20}, {0x3765, 0x20},
	{0x3766, 0x12}, {0x37a1, 0x14}, {0x37a8, 0x1c}, {0x37ab, 0x0f},
	{0x37c2, 0x14}, {0x37cb, 0x00}, {0x37cc, 0x00}, {0x37cd, 0x00},
	{0x37ce, 0x00}, {0x37d8, 0x02}, {0x37d9, 0x04}, {0x37dc, 0x04},
	{0x3800, 0x00}, {0x3801, 0x00}, {0x3802, 0x00}, {0x3803, 0x00},
	{0x3804, 0x0a}, {0x3805, 0x3f}, {0x3806, 0x07}, {0x3807, 0xb7},
	{0x3808, 0x05}, {0x3809, 0x10}, {0x380a, 0x03}, {0x380b, 0xcc},
	{0x380c, 0x02}, {0x380d, 0xee}, {0x380e, 0x07}, {0x380f, 0xd0},
	{0x3811, 0x08}, {0x3813, 0x0d}, {0x3814, 0x03}, {0x3815, 0x01},
	{0x3816, 0x03}, {0x3817, 0x01}, {0x381e, 0x02}, {0x3820, 0x8b},
	{0x3821, 0x01}, {0x3832, 0x04}, {0x3c80, 0x01}, {0x3c82, 0x00},
	{0x3c83, 0xc8}, {0x3c8c, 0x0f}, {0x3c8d, 0xa0}, {0x3c90, 0x07},
	{0x3c91, 0x00}, {0x3c92, 0x00}, {0x3c93, 0x00}, {0x3c94, 0xd0},
	{0x3c95, 0x50}, {0x3c96, 0x35}, {0x3c97, 0x00}, {0x4001, 0xe0},
	{0x4008, 0x00}, {0x4009, 0x07}, {0x400f, 0x80}, {0x4013, 0x02},
	{0x4040, 0x00}, {0x4041, 0x03}, {0x404c, 0x50}, {0x404e, 0x20},
	{0x4500, 0x06}, {0x4503, 0x00}, {0x450a, 0x04}, {0x4809, 0x04},
	{0x480c, 0x12}, {0x4819, 0x70}, {0x4825, 0x32}, {0x4826, 0x32},
	{0x482a, 0x06}, {0x4833, 0x08}, {0x4837, 0x0d}, {0x5000, 0x77},
	{0x5b00, 0x01}, {0x5b01, 0x10}, {0x5b02, 0x01}, {0x5b03, 0xdb},
	{0x5b05, 0x6c}, {0x5e10, 0xfc}, {0x3500, 0x00}, {0x3501, 0x1f},
	{0x3502, 0x20}, {0x3503, 0x08}, {0x3508, 0x04}, {0x3509, 0x00},
	{0x3832, 0x48}, {0x5780, 0x3e}, {0x5781, 0x0f}, {0x5782, 0x44},
	{0x5783, 0x02}, {0x5784, 0x01}, {0x5785, 0x01}, {0x5786, 0x00},
	{0x5787, 0x04}, {0x5788, 0x02}, {0x5789, 0x0f}, {0x578a, 0xfd},
	{0x578b, 0xf5}, {0x578c, 0xf5}, {0x578d, 0x03}, {0x578e, 0x08},
	{0x578f, 0x0c}, {0x5790, 0x08}, {0x5791, 0x06}, {0x5792, 0x00},
	{0x5793, 0x52}, {0x5794, 0xa3}, {0x4003, 0x40}, {0x3107, 0x01},
	{0x3c80, 0x08}, {0x3c83, 0xb1}, {0x3c8c, 0x10}, {0x3c8d, 0x00},
	{0x3c90, 0x00}, {0x3c94, 0x00}, {0x3c95, 0x00}, {0x3c96, 0x00},
	{0x37cb, 0x09}, {0x37cc, 0x15}, {0x37cd, 0x1f}, {0x37ce, 0x1f},
};

static int ov5675_write_reg(const struct i2c_dt_spec *spec, uint16_t addr, uint8_t value)
{
	int ret;
	struct i2c_msg msg;
	uint8_t buf[3];

	buf[0] = (addr >> 8) & 0xFF;
	buf[1] = addr & 0xFF;
	buf[2] = value;

	msg.len = 3U;
	msg.buf = buf;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	ret = i2c_transfer_dt(spec, &msg, 1);

	return ret;
}

static int ov5675_read_reg(const struct device *dev, uint16_t addr, uint8_t *value)
{
	const struct ov5675_config *cfg = dev->config;
	uint8_t addr_buf[2];

	addr_buf[0] = addr >> 8;
	addr_buf[1] = addr & 0xFF;

	return i2c_write_read_dt(&cfg->i2c, addr_buf, sizeof(addr_buf), value, 1);
}

static int ov5675_write_all(const struct device *dev, const struct ov5675_reg *regs, size_t num)
{
	const struct ov5675_config *cfg = dev->config;

	for (size_t i = 0; i < num; i++) {
		int err = ov5675_write_reg(&cfg->i2c, regs[i].addr, regs[i].value);

		if (err) {
			LOG_ERR("Failed to write reg 0x%04x: %d", regs[i].addr, err);
			return err;
		}
	}

	return 0;
}

static int ov5675_hard_reset(const struct device *dev)
{
	const struct ov5675_config *cfg = dev->config;
	int ret;

	if (cfg->reset_gpio.port) {
		ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure reset GPIO: %d", ret);
			return ret;
		}
		gpio_pin_set_dt(&cfg->reset_gpio, 0);
		k_msleep(2);
	}

	if (cfg->power_gpio.port) {
		ret = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure power GPIO: %d", ret);
			return ret;
		}
		gpio_pin_set_dt(&cfg->power_gpio, 1);
		k_msleep(1);
	}

	if (cfg->reset_gpio.port) {
		gpio_pin_set_dt(&cfg->reset_gpio, 1);
	}

	k_msleep(20);

	return 0;
}

static int ov5675_init(const struct device *dev)
{
	const struct ov5675_config *cfg = dev->config;
	struct ov5675_data *data = dev->data;
	uint8_t val_h, val_l;
	uint16_t chip_id;
	int ret;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	ret = ov5675_hard_reset(dev);
	if (ret) {
		LOG_ERR("Failed to hard-reset sensor: %d", ret);
		return ret;
	}

	ret = ov5675_read_reg(dev, OV5675_REG_CHIPID_H, &val_h);
	if (ret) {
		LOG_ERR("Failed to read chip ID high byte: %d", ret);
		return -ENODEV;
	}

	ret = ov5675_read_reg(dev, OV5675_REG_CHIPID_L, &val_l);
	if (ret) {
		LOG_ERR("Failed to read chip ID low byte: %d", ret);
		return -ENODEV;
	}

	chip_id = ((uint16_t)val_h << 8) | val_l;
	if (chip_id != OV5675_CHIP_ID_VAL) {
		LOG_ERR("Wrong chip ID: 0x%04x (expected 0x%04x)", chip_id, OV5675_CHIP_ID_VAL);
		return -ENODEV;
	}

	ret = ov5675_write_reg(&cfg->i2c, OV5675_REG_MODE_SELECT, OV5675_MODE_STANDBY);
	if (ret) {
		LOG_ERR("Failed to set standby mode: %d", ret);
		return ret;
	}

	data->is_streaming = false;

	return 0;
}

static int ov5675_set_fmt(const struct device *dev, enum video_endpoint_id ep,
			  struct video_format *fmt)
{
	ARG_UNUSED(ep);

	struct ov5675_data *data = dev->data;
	int ret;

	if (fmt->pixelformat != VIDEO_PIX_FMT_Y10P) {
		LOG_ERR("Unsupported pixel format");
		return -ENOTSUP;
	}

	if (fmt->width != 1296 || fmt->height != 972) {
		LOG_ERR("Unsupported resolution %ux%u", fmt->width, fmt->height);
		return -ENOTSUP;
	}

	if (!memcmp(&data->fmt, fmt, sizeof(data->fmt))) {
		return 0;
	}

	data->fmt = *fmt;

	ret = ov5675_write_all(dev, ov5675_1296x972_regs, ARRAY_SIZE(ov5675_1296x972_regs));
	if (ret) {
		LOG_ERR("Failed to write sensor config: %d", ret);
		return ret;
	}

	data->is_streaming = false;

	return 0;
}

static int ov5675_get_fmt(const struct device *dev, enum video_endpoint_id ep,
			  struct video_format *fmt)
{
	ARG_UNUSED(ep);

	struct ov5675_data *data = dev->data;

	*fmt = data->fmt;

	return 0;
}

static const struct video_format_cap ov5675_fmts[] = {
	{
		.pixelformat = VIDEO_PIX_FMT_Y10P,
		.width_min = 1296,
		.width_max = 1296,
		.height_min = 972,
		.height_max = 972,
		.width_step = 0,
		.height_step = 0,
	},
	{0},
};

static int ov5675_get_caps(const struct device *dev, enum video_endpoint_id ep,
			   struct video_caps *caps)
{
	caps->format_caps = ov5675_fmts;
	return 0;
}

static int ov5675_set_stream(const struct device *dev, bool enable)
{
	struct ov5675_data *data = dev->data;
	const struct ov5675_config *cfg = dev->config;

	uint8_t val;
	int ret;

	if (enable && data->is_streaming) {
		return 0;
	}

	if (!enable && !data->is_streaming) {
		return 0;
	}

	val = enable ? OV5675_MODE_STREAMING : OV5675_MODE_STANDBY;

	ret = ov5675_write_reg(&cfg->i2c, OV5675_REG_MODE_SELECT, val);
	if (ret) {
		LOG_ERR("Failed to %s streaming: %d", enable ? "start" : "stop", ret);
		return ret;
	}

	data->is_streaming = enable;

	return 0;
}

static DEVICE_API(video, ov5675_driver_api) = {
	.set_format = ov5675_set_fmt,
	.get_format = ov5675_get_fmt,
	.get_caps   = ov5675_get_caps,
	.set_stream = ov5675_set_stream,
};

#define OV5675_DEVICE_DEFINE(i)                                                    \
	static const struct ov5675_config ov5675_cfg_##i = {                       \
		.i2c        = I2C_DT_SPEC_INST_GET(i),                            \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(i, reset_gpios, {}),       \
		.power_gpio = GPIO_DT_SPEC_INST_GET_OR(i, power_gpios, {}),       \
	};                                                                         \
                                                                                   \
	static struct ov5675_data ov5675_data_##i;                                 \
                                                                                   \
	DEVICE_DT_INST_DEFINE(i, &ov5675_init, NULL, &ov5675_data_##i,            \
			      &ov5675_cfg_##i, POST_KERNEL,                        \
			      CONFIG_VIDEO_INIT_PRIORITY, &ov5675_driver_api);

DT_INST_FOREACH_STATUS_OKAY(OV5675_DEVICE_DEFINE)
