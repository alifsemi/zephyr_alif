/*
 * SPDX-FileCopyrightText: Copyright Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ovti_ov5647

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ov5647);

#define OV5647_CHIP_ID_VAL   0x5647

#define OV5647_SW_STANDBY               0x0100
#define OV5647_SW_RESET                 0x0103
#define OV5647_REG_CHIPID_H             0x300a
#define OV5647_REG_CHIPID_L             0x300b
#define OV5640_REG_PAD_OUT              0x300d
#define OV5647_REG_FRAME_OFF_NUMBER     0x4202
#define OV5647_REG_MIPI_CTRL00          0x4800
#define OV5647_REG_MIPI_CTRL14          0x4814

#define MIPI_CTRL00_CLOCK_LANE_GATE     BIT(5)
#define MIPI_CTRL00_LINE_SYNC_ENABLE    BIT(4)
#define MIPI_CTRL00_BUS_IDLE            BIT(2)
#define MIPI_CTRL00_CLOCK_LANE_DISABLE  BIT(0)

struct ov5647_config {
	const struct gpio_dt_spec reset_gpio;
	struct i2c_dt_spec i2c;
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pcfg;
#endif
};

struct ov5647_data {
	struct video_format fmt;
	bool is_streaming;
#ifdef CONFIG_PM_DEVICE
	bool needs_reinit;
#endif
};

struct ov5647_reg {
	uint16_t addr;
	uint8_t value;
};

static const struct ov5647_reg sensor_oe_enable_regs[] = {
    {0x3000, 0x0f},
    {0x3001, 0xff},
    {0x3002, 0xe4},
};

/**
\brief OV5647 Camera Sensor Resolution 640x480
*/
static const struct ov5647_reg ov5647_640x480_10bpp[] = {
    {0x0100, 0x00}, {0x0103, 0x01}, {0x3034, 0x08}, {0x3035, 0x11}, {0x3036, 0x46}, {0x303c, 0x11},
    {0x3821, 0x07}, {0x3820, 0x41}, {0x370c, 0x03}, {0x3612, 0x59}, {0x3618, 0x00}, {0x5000, 0x06},
    {0x5001, 0x01}, {0x5002, 0x41}, {0x5003, 0x08}, {0x5a00, 0x08}, {0x3000, 0xff}, {0x3001, 0xff},
    {0x3002, 0xff}, {0x301d, 0xf0}, {0x3a18, 0x00}, {0x3a19, 0xf8}, {0x3c01, 0x80}, {0x3b07, 0x0c},
    {0x380c, 0x07}, {0x380d, 0x3c}, {0x3814, 0x35}, {0x3815, 0x35}, {0x3708, 0x64}, {0x3709, 0x52},
    {0x3808, 0x02}, {0x3809, 0x80}, {0x380a, 0x01}, {0x380b, 0xe0}, {0x3800, 0x00}, {0x3801, 0x10},
    {0x3802, 0x00}, {0x3803, 0x00}, {0x3804, 0x0a}, {0x3805, 0x2f}, {0x3806, 0x07}, {0x3807, 0x9f},
    {0x3630, 0x2e}, {0x3632, 0xe2}, {0x3633, 0x23}, {0x3634, 0x44}, {0x3620, 0x64}, {0x3621, 0xe0},
    {0x3600, 0x37}, {0x3704, 0xa0}, {0x3703, 0x5a}, {0x3715, 0x78}, {0x3717, 0x01}, {0x3731, 0x02},
    {0x370b, 0x60}, {0x3705, 0x1a}, {0x3f05, 0x02}, {0x3f06, 0x10}, {0x3f01, 0x0a}, {0x3a08, 0x01},
    {0x3a09, 0x2e}, {0x3a0a, 0x00}, {0x3a0b, 0xfb}, {0x3a0d, 0x02}, {0x3a0e, 0x01}, {0x3a0f, 0x58},
    {0x3a10, 0x50}, {0x3a1b, 0x58}, {0x3a1e, 0x50}, {0x3a11, 0x60}, {0x3a1f, 0x28}, {0x4001, 0x02},
    {0x4004, 0x02}, {0x4000, 0x09}, {0x4050, 0x6e}, {0x4051, 0x8f}, {0x3000, 0x00}, {0x3001, 0x00},
    {0x3002, 0x00}, {0x3017, 0xe0}, {0x301c, 0xfc}, {0x3636, 0x06}, {0x3016, 0x08}, {0x3827, 0xec},
    {0x3018, 0x44}, {0x3035, 0x21}, {0x3106, 0xf5}, {0x3034, 0x1a}, {0x301c, 0xf8}, {0x350a, 0x01},
    {0x350b, 0x8f}, {0x3500, 0x00}, {0x3501, 0x0F}, {0x3502, 0xFF}, {0x0100, 0x01},
};

static int ov5647_write_reg(const struct i2c_dt_spec *spec, uint16_t addr, uint8_t value)
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

static int ov5647_read_reg(const struct device *dev, uint16_t addr, uint8_t *value)
{
	const struct ov5647_config *cfg = dev->config;
	uint8_t addr_buf[2];

	addr_buf[0] = addr >> 8;
	addr_buf[1] = addr & 0xFF;

	return i2c_write_read_dt(&cfg->i2c, addr_buf, sizeof(addr_buf), value, 1);
}

static int ov5647_write_all(const struct device *dev, const struct ov5647_reg *regs, size_t num)
{
	const struct ov5647_config *cfg = dev->config;

	for (size_t i = 0; i < num; i++) {
		int err = ov5647_write_reg(&cfg->i2c, regs[i].addr, regs[i].value);

		if (err) {
			LOG_ERR("Failed to write reg 0x%04x: %d", regs[i].addr, err);
			return err;
		}
	}

	return 0;
}

static int ov5647_hard_reset(const struct device *dev)
{
	const struct ov5647_config *cfg = dev->config;
	int ret;

	if (cfg->reset_gpio.port) {
		ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure reset GPIO: %d", ret);
			return ret;
		}
		gpio_pin_set_dt(&cfg->reset_gpio, 1);
		k_msleep(20);
	}

	return 0;
}

static int ov5647_program_fmt(const struct device *dev, const struct video_format *fmt)
{
	int ret;

	if (fmt->width != 640 || fmt->height != 480) {
		LOG_ERR("Unsupported resolution %ux%u", fmt->width, fmt->height);
		return -ENOTSUP;
	}
	
	ret = ov5647_write_all(dev, ov5647_640x480_10bpp, ARRAY_SIZE(ov5647_640x480_10bpp));
	if (ret) {
		LOG_ERR("Failed to write config: %d", ret);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int ov5647_reinit_after_pm(const struct device *dev);
#endif

/**
  \fn           int32_t OV5647_Stream_On(void)
  \brief        Start streaming OV5647 Camera Sensor
  \param[in]    none
  \return       \ref execution_status
  */
static int32_t ov5647_Stream_On(const struct device *dev)
{
    const struct ov5647_config *cfg = dev->config;
    int32_t ret = 0;
    
    ret = ov5647_write_reg(&cfg->i2c, OV5647_REG_MIPI_CTRL00,
                               MIPI_CTRL00_CLOCK_LANE_GATE|MIPI_CTRL00_BUS_IDLE |
                               MIPI_CTRL00_LINE_SYNC_ENABLE);

    if (ret) {
	return -ENODEV;
    }
    
    ret = ov5647_write_reg(&cfg->i2c, OV5647_REG_FRAME_OFF_NUMBER, 0x00);

    if (ret) {
	return -ENODEV;
     }
    
    return ov5647_write_reg(&cfg->i2c, OV5640_REG_PAD_OUT, 0x00);
}

/**
  \fn           int32_t OV5647_Stream_Off(void)
  \brief        Stop streaming OV5647 Camera Sensor
  \param[in]    none
  \return       \ref execution_status
  */
static int32_t ov5647_Stream_Off(const struct device *dev)
{
    const struct ov5647_config *cfg = dev->config;
    int32_t ret = 0;

    ret         = ov5647_write_reg(&cfg->i2c, OV5647_REG_MIPI_CTRL00,
                           MIPI_CTRL00_CLOCK_LANE_GATE | MIPI_CTRL00_BUS_IDLE |
                               MIPI_CTRL00_CLOCK_LANE_DISABLE);
    if (ret) {
	return -ENODEV;
     }

    ret = ov5647_write_reg(&cfg->i2c, OV5647_REG_FRAME_OFF_NUMBER, 0x0F);
    if (ret) {
	return -ENODEV;
     }

    return ov5647_write_reg(&cfg->i2c, OV5640_REG_PAD_OUT, 0x01);
}

static int ov5647_init(const struct device *dev)
{
	const struct ov5647_config *cfg = dev->config;
	struct ov5647_data *data = dev->data;
	uint8_t val_h, val_l;
	uint16_t chip_id;
	int ret;

#ifdef CONFIG_PINCTRL
	if (cfg->pcfg) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0 && ret != -ENOENT) {
			LOG_ERR("Failed to apply default pinctrl state: %d", ret);
			return ret;
		}
	}
#endif

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	ret = ov5647_hard_reset(dev);
	if (ret) {
		LOG_ERR("Failed to hard-reset sensor: %d", ret);
		return ret;
	}

	ret = ov5647_read_reg(dev, OV5647_REG_CHIPID_H, &val_h);
	if (ret) {
		LOG_ERR("Failed to read chip ID high byte: %d", ret);
		return -ENODEV;
	}

	ret = ov5647_read_reg(dev, OV5647_REG_CHIPID_L, &val_l);
	if (ret) {
		LOG_ERR("Failed to read chip ID low byte: %d", ret);
		return -ENODEV;
	}

	chip_id = ((uint16_t)val_h << 8) | val_l;
	if (chip_id != OV5647_CHIP_ID_VAL) {
		LOG_ERR("Wrong chip ID: 0x%04x (expected 0x%04x)", chip_id, OV5647_CHIP_ID_VAL);
		return -ENODEV;
	}

	ret = ov5647_write_reg(&cfg->i2c, OV5647_SW_RESET, 0x00);
	if (ret) {
		return -ENODEV;
        }
	
	ret = ov5647_write_all(dev, sensor_oe_enable_regs, ARRAY_SIZE(sensor_oe_enable_regs));
	if (ret) {
		return -ENODEV;
	}
	
	ret = ov5647_Stream_Off(dev);
	if (ret) {
		return -ENODEV;
	}
	
	data->is_streaming = false;

	return 0;
}

static int ov5647_set_fmt(const struct device *dev, enum video_endpoint_id ep,
			  struct video_format *fmt)
{
	ARG_UNUSED(ep);

	struct ov5647_data *data = dev->data;
	int ret;

#ifdef CONFIG_PM_DEVICE
	ret = ov5647_reinit_after_pm(dev);
	if (ret) {
		return ret;
	}
#endif

	if (fmt->pixelformat != VIDEO_PIX_FMT_Y10P) {
		LOG_ERR("Unsupported pixel format");
		return -ENOTSUP;
	}

	if (!memcmp(&data->fmt, fmt, sizeof(data->fmt))) {
		return 0;
	}

	ret = ov5647_program_fmt(dev, fmt);
	if (ret) {
		return ret;
	}

	data->fmt = *fmt;
	data->is_streaming = false;

	return 0;
}

static int ov5647_get_fmt(const struct device *dev, enum video_endpoint_id ep,
			  struct video_format *fmt)
{
	ARG_UNUSED(ep);

	struct ov5647_data *data = dev->data;

	*fmt = data->fmt;

	return 0;
}

static const struct video_format_cap ov5647_fmts[] = {
	{
		.pixelformat = VIDEO_PIX_FMT_Y10P,
		.width_min = 640, .width_max = 640,
		.height_min = 480, .height_max = 480,
		.width_step = 0,   .height_step = 0,
	},
	{0},
};

static int ov5647_get_caps(const struct device *dev, enum video_endpoint_id ep,
			   struct video_caps *caps)
{
	caps->format_caps = ov5647_fmts;
	return 0;
}

static int ov5647_set_stream(const struct device *dev, bool enable)
{
	struct ov5647_data *data = dev->data;

	int ret;

#ifdef CONFIG_PM_DEVICE
	if (enable) {
		ret = ov5647_reinit_after_pm(dev);
		if (ret) {
			return ret;
		}
	}
#endif

	if (enable && data->is_streaming) {
		return 0;
	}

	if (!enable && !data->is_streaming) {
		return 0;
	}

	if(enable){
		ret = ov5647_Stream_On(dev);
	}
	else{
		ret = ov5647_Stream_Off(dev);
	}
	if (ret) {
		LOG_ERR("Failed to %s streaming: %d", enable ? "start" : "stop", ret);
		return ret;
	}

	data->is_streaming = enable;

	return 0;
}


static DEVICE_API(video, ov5647_driver_api) = {
	.set_format = ov5647_set_fmt,
	.get_format = ov5647_get_fmt,
	.get_caps   = ov5647_get_caps,
	.set_stream = ov5647_set_stream,
};

#if defined(CONFIG_PM_DEVICE)

/** OV5647 Camera: Suspend */
static int ov5647_suspend(const struct device *dev)
{

	return 0;
}

/*
 * Sleep-free resume: restore pinmux only and mark the sensor for
 * re-init. Reset, I2C and stabilization delays run later on an
 * application thread via set_stream/set_fmt/set_ctrl.
 */
static int ov5647_resume(const struct device *dev)
{

	return 0;
}

static int ov5647_reinit_after_pm(const struct device *dev)
{

	return 0;
}

static int ov5647_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		return ov5647_resume(dev);

	case PM_DEVICE_ACTION_SUSPEND:
		return ov5647_suspend(dev);

	case PM_DEVICE_ACTION_TURN_OFF:
	case PM_DEVICE_ACTION_TURN_ON:
		return 0;

	default:
		return -ENOTSUP;
	}
}
#endif /* CONFIG_PM_DEVICE */

#define OV5647_PINCTRL_DEFINE(i) \
	IF_ENABLED(CONFIG_PINCTRL, \
		(COND_CODE_1(DT_INST_PINCTRL_HAS_IDX(i, 0), \
			     (PINCTRL_DT_INST_DEFINE(i);), ())))

#define OV5647_PINCTRL_CONFIG(i) \
	IF_ENABLED(CONFIG_PINCTRL, \
		(COND_CODE_1(DT_INST_PINCTRL_HAS_IDX(i, 0), \
			     (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),), ())))

#define OV5647_DEVICE_DEFINE(i)                                                    \
	OV5647_PINCTRL_DEFINE(i)                                                   \
	static const struct ov5647_config ov5647_cfg_##i = {                       \
		.i2c        = I2C_DT_SPEC_INST_GET(i),                            \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(i, reset_gpios, {}),       \
		OV5647_PINCTRL_CONFIG(i)                                          \
	};                                                                         \
	static struct ov5647_data ov5647_data_##i;                                 \
	DEVICE_DT_INST_DEFINE(i, &ov5647_init, NULL,                               \
			&ov5647_data_##i, &ov5647_cfg_##i, POST_KERNEL,            \
			CONFIG_VIDEO_INIT_PRIORITY, &ov5647_driver_api);

DT_INST_FOREACH_STATUS_OKAY(OV5647_DEVICE_DEFINE)
