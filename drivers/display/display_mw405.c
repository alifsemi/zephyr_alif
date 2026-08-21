/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT focuslcds_mw405

#include <zephyr/drivers/display.h>
#include <zephyr/drivers/mipi_dsi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(panel_mw405, CONFIG_DISPLAY_LOG_LEVEL);

#define DCS_CMD_PAGE			0xFF
#define DCS_CMD_PAGE_SET_PAGE_0		0x0
#define DCS_CMD_PAGE_SET_PAGE_1		0x1
#define DCS_CMD_PAGE_SET_PAGE_2		0x2
#define DCS_CMD_PAGE_SET_PAGE_3		0x3
#define DCS_CMD_PAGE_SET_PAGE_4		0x4
#define DCS_CMD_PAGE_SET_PAGE_5		0x5
#define DCS_CMD_PAGE_SET_PAGE_6		0x6
#define DCS_CMD_PAGE_SET_PAGE_7		0x7

#define TIMINGS_HSYNC_LEN		4
#define TIMINGS_HBP			5
#define TIMINGS_HFP			5
#define TIMINGS_HACT			480
#define TIMINGS_VSYNC_LEN		2
#define TIMINGS_VBP			10
#define TIMINGS_VFP			10
#define TIMINGS_VACT			800

/* DPI Interface settings. */
#define DPI_INTERFACE_16_BIT		5
#define DPI_INTERFACE_18_BIT		6
#define DPI_INTERFACE_24_BIT		7
#define DPI_INTERFACE_SHIFT		4

#define DISPLAY_ACCESS_CTL_GS		BIT(0)
#define DISPLAY_ACCESS_CTL_SS		BIT(1)
#define MIPI_DCS_DISPLAY_ACCESS_CMD	0x36

enum video_mode_type {
	BURST_MODE = 0,
	NON_BURST_MODE_SYNC_PULSE = 1,
	NON_BURST_MODE_SYNC_EVENTS = 2,
};

enum cmd_mode_type {
	CMD_LP = 0,
	CMD_HS = 1,
};

struct mw405_config {
	const struct device *mipi_dsi;
	const struct gpio_dt_spec reset_gpio;
	const struct gpio_dt_spec bl_gpio;
	uint8_t num_lanes;
	uint8_t pixel_format;
	uint16_t active_width;
	uint16_t active_height;
	uint8_t channel;
	enum video_mode_type vid_mode;
	enum cmd_mode_type cmd_type;
};

struct mw405_data {
	enum display_orientation orientation;
#ifdef CONFIG_PM_DEVICE
	const struct device *dev;
	struct k_work_delayable reset_work;
	struct k_work_sync reset_work_sync;
	struct k_sem resume_sem;
#endif /* CONFIG_PM_DEVICE */
};

#ifdef CONFIG_PM_DEVICE
static void mw405_reset_work_handler(struct k_work *work);
static inline void mw405_wait_ready(struct mw405_data *data);
#endif

enum mw405_op {
	MW405_SWITCH_PAGE,
	MW405_COMMAND,
};

struct mw405_instr {
	enum mw405_op op;
	union arg {
		struct cmd {
			uint8_t cmd;
			uint8_t data;
		} cmd;
		uint8_t page;
	} arg;
};

#define MW405_SWITCH_PAGE_INSTR(_page)		\
	{					\
		.op = MW405_SWITCH_PAGE,	\
		.arg = {			\
			.page = (_page),	\
		}				\
	}

#define MW405_COMMAND_INSTR(_cmd, _data)		\
	{						\
		.op = MW405_COMMAND,			\
		.arg = {				\
			.cmd = {			\
				.cmd = (_cmd),		\
				.data = (_data),	\
			},				\
		},					\
	}

static const struct mw405_instr init_cmds[] = {
	/* Change to Page 1 */
	MW405_SWITCH_PAGE_INSTR(DCS_CMD_PAGE_SET_PAGE_1),
	MW405_COMMAND_INSTR(0x08, 0x10), /* SDO always output without tri-state */
	MW405_COMMAND_INSTR(0x20, 0x00),
	MW405_COMMAND_INSTR(0x21, 0x01), /* DE Active High */
	MW405_COMMAND_INSTR(0x30, 0x02), /* Resolution 480 X 800 */
	MW405_COMMAND_INSTR(0x31, 0x00), /* Column Inversion */
	MW405_COMMAND_INSTR(0x40, 0x14), /* DDVDH/DDVDL voltage control */
	MW405_COMMAND_INSTR(0x41, 0x22), /* avdd +5.0V, avee -5.0V*/
	MW405_COMMAND_INSTR(0x42, 0x02), /*
					  * VGL=DDVDH+VCIP -DDVDL,
					  * VGH=2DDVDL-VCIP
					  */
	MW405_COMMAND_INSTR(0x43, 0x84), /* Set VGH Clamp level = 12.5 V */
	MW405_COMMAND_INSTR(0x44, 0x8A), /* Set VGL Clamp level = -12.5 V */
	MW405_COMMAND_INSTR(0x50, 0x78), /* Positive Gamma VREG1OUT = 4.5 V */
	MW405_COMMAND_INSTR(0x51, 0x78), /* Negative Gamma VREG2OUT = -4.5 V */
	MW405_COMMAND_INSTR(0x52, 0x00),
	MW405_COMMAND_INSTR(0x53, 0x2B), /* VCOM = -0.7250 V (regs 0x52~0x53)*/
	MW405_COMMAND_INSTR(0x54, 0x00),
	MW405_COMMAND_INSTR(0x55, 0x2B), /* VCOM voltage factor */
	MW405_COMMAND_INSTR(0x60, 0x07),
	MW405_COMMAND_INSTR(0x61, 0x06),
	MW405_COMMAND_INSTR(0x62, 0x06),
	MW405_COMMAND_INSTR(0x63, 0x04),
	/* Positive Gamma Control */
	MW405_COMMAND_INSTR(0xA0, 0x00), MW405_COMMAND_INSTR(0xA1, 0x0B),
	MW405_COMMAND_INSTR(0xA2, 0x19), MW405_COMMAND_INSTR(0xA3, 0x10),
	MW405_COMMAND_INSTR(0xA4, 0x06), MW405_COMMAND_INSTR(0xA5, 0x0F),
	MW405_COMMAND_INSTR(0xA6, 0x09), MW405_COMMAND_INSTR(0xA7, 0x06),
	MW405_COMMAND_INSTR(0xA8, 0x0C), MW405_COMMAND_INSTR(0xA9, 0x0E),
	MW405_COMMAND_INSTR(0xAA, 0x16), MW405_COMMAND_INSTR(0xAB, 0x0D),
	MW405_COMMAND_INSTR(0xAC, 0x15), MW405_COMMAND_INSTR(0xAD, 0x0F),
	MW405_COMMAND_INSTR(0xAE, 0x11), MW405_COMMAND_INSTR(0xAF, 0x00),
	/* Negative Gamma Control */
	MW405_COMMAND_INSTR(0xC0, 0x00), MW405_COMMAND_INSTR(0xC1, 0x24),
	MW405_COMMAND_INSTR(0xC2, 0x29), MW405_COMMAND_INSTR(0xC3, 0x0C),
	MW405_COMMAND_INSTR(0xC4, 0x07), MW405_COMMAND_INSTR(0xC5, 0x03),
	MW405_COMMAND_INSTR(0xC6, 0x03), MW405_COMMAND_INSTR(0xC7, 0x03),
	MW405_COMMAND_INSTR(0xC8, 0x03), MW405_COMMAND_INSTR(0xC9, 0x09),
	MW405_COMMAND_INSTR(0xCA, 0x0D), MW405_COMMAND_INSTR(0xCB, 0x01),
	MW405_COMMAND_INSTR(0xCC, 0x06), MW405_COMMAND_INSTR(0xCD, 0x1B),
	MW405_COMMAND_INSTR(0xCE, 0x08), MW405_COMMAND_INSTR(0xCF, 0x00),

	/* Change to Page 6 */
	MW405_SWITCH_PAGE_INSTR(DCS_CMD_PAGE_SET_PAGE_6),
	/* GIP (Gate driver in Panel) timing */
	MW405_COMMAND_INSTR(0x00, 0x20), MW405_COMMAND_INSTR(0x01, 0x04),
	MW405_COMMAND_INSTR(0x02, 0x00), MW405_COMMAND_INSTR(0x03, 0x00),
	MW405_COMMAND_INSTR(0x04, 0x01), MW405_COMMAND_INSTR(0x05, 0x01),
	MW405_COMMAND_INSTR(0x06, 0x88), MW405_COMMAND_INSTR(0x07, 0x04),
	MW405_COMMAND_INSTR(0x08, 0x01), MW405_COMMAND_INSTR(0x09, 0x90),
	MW405_COMMAND_INSTR(0x0A, 0x03), MW405_COMMAND_INSTR(0x0B, 0x01),
	MW405_COMMAND_INSTR(0x0C, 0x01), MW405_COMMAND_INSTR(0x0D, 0x01),
	MW405_COMMAND_INSTR(0x0E, 0x00), MW405_COMMAND_INSTR(0x0F, 0x00),
	MW405_COMMAND_INSTR(0x10, 0x55), MW405_COMMAND_INSTR(0x11, 0x53),
	MW405_COMMAND_INSTR(0x12, 0x01), MW405_COMMAND_INSTR(0x13, 0x0D),
	MW405_COMMAND_INSTR(0x14, 0x0D), MW405_COMMAND_INSTR(0x15, 0x43),
	MW405_COMMAND_INSTR(0x16, 0x0B), MW405_COMMAND_INSTR(0x17, 0x00),
	MW405_COMMAND_INSTR(0x18, 0x00), MW405_COMMAND_INSTR(0x19, 0x00),
	MW405_COMMAND_INSTR(0x1A, 0x00), MW405_COMMAND_INSTR(0x1B, 0x00),
	MW405_COMMAND_INSTR(0x1C, 0x00), MW405_COMMAND_INSTR(0x1D, 0x00),
	MW405_COMMAND_INSTR(0x20, 0x01), MW405_COMMAND_INSTR(0x21, 0x23),
	MW405_COMMAND_INSTR(0x22, 0x45), MW405_COMMAND_INSTR(0x23, 0x67),
	MW405_COMMAND_INSTR(0x24, 0x01), MW405_COMMAND_INSTR(0x25, 0x23),
	MW405_COMMAND_INSTR(0x26, 0x45), MW405_COMMAND_INSTR(0x27, 0x67),
	MW405_COMMAND_INSTR(0x30, 0x02), MW405_COMMAND_INSTR(0x31, 0x22),
	MW405_COMMAND_INSTR(0x32, 0x11), MW405_COMMAND_INSTR(0x33, 0xAA),
	MW405_COMMAND_INSTR(0x34, 0xBB), MW405_COMMAND_INSTR(0x35, 0x66),
	MW405_COMMAND_INSTR(0x36, 0x00), MW405_COMMAND_INSTR(0x37, 0x22),
	MW405_COMMAND_INSTR(0x38, 0x22), MW405_COMMAND_INSTR(0x39, 0x22),
	MW405_COMMAND_INSTR(0x3A, 0x22), MW405_COMMAND_INSTR(0x3B, 0x22),
	MW405_COMMAND_INSTR(0x3C, 0x22), MW405_COMMAND_INSTR(0x3D, 0x22),
	MW405_COMMAND_INSTR(0x3E, 0x22), MW405_COMMAND_INSTR(0x3F, 0x22),
	MW405_COMMAND_INSTR(0x40, 0x22),

	/* Change to Page 5 */
	MW405_SWITCH_PAGE_INSTR(DCS_CMD_PAGE_SET_PAGE_5),
	/* Backlight Control */
	MW405_COMMAND_INSTR(0x09, 0xFC), /* */
	MW405_COMMAND_INSTR(0x07, 0xBC), /*
					  * Light Adaptive backlight control ON
					  * Enable SRE function, Backlight LED ON
					  */

	/* Change to Page 0 */
	MW405_SWITCH_PAGE_INSTR(DCS_CMD_PAGE_SET_PAGE_0),
	MW405_COMMAND_INSTR(0x36, 0), /* Fix Panel RGB-BGR order to RGB
				       * and Horizontal/Vertical flip to zero.
				       */
};

static int mw405_switch_page(const struct device *dev, uint8_t page)
{
	uint8_t buf[] = { DCS_CMD_PAGE, 0x98, 0x06, 0x04, page};
	const struct mw405_config *config = dev->config;

	return mipi_dsi_dcs_write(config->mipi_dsi,
			config->channel,
			DCS_CMD_PAGE,
			buf,
			sizeof(buf));
}

static int mw405_configure(const struct device *dev)
{
	const struct mw405_config *config = dev->config;
	int ret;
	uint8_t tmp;

	LOG_INF("MW-405 Configuration.");
	for (int i = 0; i < ARRAY_SIZE(init_cmds); i++) {
		if (init_cmds[i].op == MW405_SWITCH_PAGE) {
			ret = mw405_switch_page(dev, init_cmds[i].arg.page);
		} else if (init_cmds[i].op == MW405_COMMAND) {
			ret = mipi_dsi_dcs_write(config->mipi_dsi,
					config->channel,
					init_cmds[i].arg.cmd.cmd,
					&init_cmds[i].arg.cmd.data,
					1);
		}
		if (ret)
			return ret;
	}

	/*
	 * Setup the Pixel Interface.
	 * Page 0 DCS command - MIPI_DCS_SET_PIXEL_FORMAT 1st parameter
	 *	bits [4:6] hold the RGB Interface format.
	 *		16-bit/pixel : 0x5 @ bit[4:6]
	 *		18-bit/pixel : 0x6 @ bit[4:6]
	 *		24-bit/pixel : 0x7 @ bit[4:6]
	 */
	switch (config->pixel_format) {
	case MIPI_DSI_PIXFMT_RGB888:
		tmp = DPI_INTERFACE_24_BIT << DPI_INTERFACE_SHIFT;
		break;
	case MIPI_DSI_PIXFMT_RGB565:
		tmp = DPI_INTERFACE_16_BIT << DPI_INTERFACE_SHIFT;
		break;
	case MIPI_DSI_PIXFMT_RGB666_PACKED:
	case MIPI_DSI_PIXFMT_RGB666:
		tmp = DPI_INTERFACE_18_BIT << DPI_INTERFACE_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	ret = mipi_dsi_dcs_write(config->mipi_dsi, config->channel,
			MIPI_DCS_SET_PIXEL_FORMAT, &tmp, 1);
	if (ret) {
		return ret;
	}

	/* Exit sleep Mode. */
	ret = mipi_dsi_dcs_write(config->mipi_dsi, config->channel,
			MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
	if (ret)
		return ret;

	/*
	 * Added sleep for 5ms to ensure that the Display comes out of Sleep.
	 * When no sleep added before sending the DISPLAY_ON command, the
	 * DSI host was failing with ACK error.
	 */
	k_msleep(5);

	/* Display On. */
	ret = mipi_dsi_dcs_write(config->mipi_dsi, config->channel,
			MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
	if (ret)
		return ret;

	return 0;
}

/* Generic APIs */

static int mw405_blanking_on(const struct device *dev)
{
	const struct mw405_config *config = dev->config;

#ifdef CONFIG_PM_DEVICE
	mw405_wait_ready((struct mw405_data *)dev->data);
#endif /* CONFIG_PM_DEVICE */

	if (config->bl_gpio.port != NULL)
		return gpio_pin_set_dt(&config->bl_gpio, 0);
	else
		return -ENOTSUP;
}

static int mw405_blanking_off(const struct device *dev)
{
	const struct mw405_config *config = dev->config;

#ifdef CONFIG_PM_DEVICE
	mw405_wait_ready((struct mw405_data *)dev->data);
#endif /* CONFIG_PM_DEVICE */

	if (config->bl_gpio.port != NULL)
		return gpio_pin_set_dt(&config->bl_gpio, 1);
	else
		return -ENOTSUP;
}

static int mw405_set_brightness(const struct device *dev,
		const uint8_t brightness)
{
	return -ENOTSUP;
}

static int mw405_set_contrast(const struct device *dev,
		const uint8_t contrast)
{
	return -ENOTSUP;
}

static int mw405_set_orientation(const struct device *dev,
		const enum display_orientation orientation)
{
	const struct mw405_config *config = dev->config;
	struct mw405_data *data = dev->data;
	uint8_t tmp;
	int ret;

#ifdef CONFIG_PM_DEVICE
	mw405_wait_ready((struct mw405_data *)dev->data);
#endif /* CONFIG_PM_DEVICE */

	if (data->orientation == orientation)
		return 0;

	/*
	 * Page 0 DCS command - 0x36 (Display Access Control) 1st parameter
	 *	bit[0:1] controls the panel orientation.
	 *		bit[0] - Panel Flip Horizontal
	 *		bit[1] - Panel Flip Vertical
	 *	bit[3] controls Panel RGB-BGR order
	 */
	switch (orientation) {
	case DISPLAY_ORIENTATION_ROTATED_180:
		tmp = DISPLAY_ACCESS_CTL_GS | DISPLAY_ACCESS_CTL_SS;
		break;
	case DISPLAY_ORIENTATION_NORMAL:
		tmp = 0;
		break;
	default:
		LOG_ERR("Unsupported Orientation.");
		return -ENOTSUP;
	}
	ret = mipi_dsi_dcs_write(config->mipi_dsi,
			config->channel, MIPI_DCS_DISPLAY_ACCESS_CMD, &tmp, 1);
	if (ret)
		LOG_ERR("Failed to set orientation of panel.");
	else
		data->orientation = orientation;
	return 0;
}

static void mw405_get_capabilities(const struct device *dev,
		struct display_capabilities *capabilities)
{
	const struct mw405_config *config = dev->config;
	struct mw405_data *data = dev->data;

	memset(capabilities, 0, sizeof(*capabilities));
	capabilities->x_resolution = config->active_width;
	capabilities->y_resolution = config->active_height;
	capabilities->supported_pixel_formats = config->pixel_format;
	capabilities->current_pixel_format = config->pixel_format;
	capabilities->current_orientation = data->orientation;
}

static int mw405_set_pixel_format(const struct device *dev,
		enum display_pixel_format pixel_format)
{
	return -ENOTSUP;
}

static int mw405_read(const struct device *dev,
		const uint16_t x,
		const uint16_t y,
		const struct display_buffer_descriptor *desc,
		void *buf)
{
	return -ENOTSUP;
}

static int mw405_write(const struct device *dev,
		const uint16_t x,
		const uint16_t y,
		const struct display_buffer_descriptor *desc,
		const void *buf)
{
	return -ENOTSUP;
}

static void *mw405_get_framebuffer(const struct device *dev)
{
	LOG_ERR("Frame-buffer not supported.");
	return NULL;
}

static const struct display_driver_api mw405_api = {
	.read = mw405_read,
	.write = mw405_write,
	.get_framebuffer = mw405_get_framebuffer,
	.blanking_on = mw405_blanking_on,
	.blanking_off = mw405_blanking_off,
	.set_brightness = mw405_set_brightness,
	.set_contrast = mw405_set_contrast,
	.get_capabilities = mw405_get_capabilities,
	.set_pixel_format = mw405_set_pixel_format,
	.set_orientation = mw405_set_orientation,
};

static int mw405_init(const struct device *dev)
{
	const struct mw405_config *config = dev->config;
	struct mw405_data *data = dev->data;
	struct mipi_dsi_device mdev;
	int ret;

#ifdef CONFIG_PM_DEVICE
	data->dev = dev;
	k_work_init_delayable(&data->reset_work, mw405_reset_work_handler);
	k_sem_init(&data->resume_sem, 1, 1);
#endif

	mdev.data_lanes = config->num_lanes;
	mdev.pixfmt = config->pixel_format;

	mdev.timings.hactive = TIMINGS_HACT;
	mdev.timings.hfp = TIMINGS_HFP;
	mdev.timings.hbp = TIMINGS_HBP;
	mdev.timings.hsync = TIMINGS_HSYNC_LEN;
	mdev.timings.vactive = TIMINGS_VACT;
	mdev.timings.vfp = TIMINGS_VFP;
	mdev.timings.vbp = TIMINGS_VBP;
	mdev.timings.vsync = TIMINGS_VSYNC_LEN;

	mdev.mode_flags = MIPI_DSI_MODE_VIDEO |
		MIPI_DSI_MODE_EOT_PACKET;

	if (config->cmd_type == CMD_LP)
		mdev.mode_flags |= MIPI_DSI_MODE_LPM;

	switch (config->vid_mode) {
	case BURST_MODE:
		mdev.mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
		break;
	case NON_BURST_MODE_SYNC_PULSE:
		mdev.mode_flags |=  MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
		break;
	case NON_BURST_MODE_SYNC_EVENTS:
	default:
		break;
	}

	ret = mipi_dsi_attach(config->mipi_dsi, config->channel, &mdev);
	if (ret < 0) {
		LOG_ERR("Could not attach to MIPI-DSI host");
		return ret;
	}

	if (config->reset_gpio.port != NULL) {
		ret = gpio_pin_configure_dt(&config->reset_gpio,
				GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure reset GPIO (%d)", ret);
			return ret;
		}
		/*
		 * Resetting the panel as per datasheet. Reset process:
		 *	1. Hold the Reset pin high.
		 *	2. Send a low pulse for >= 10 us
		 *	3. Wait for 5ms after releasing Reset pin.
		 *	4. Sleep-out command cannot be sent for 120ms
		 */
		k_msleep(1);
		gpio_pin_set_dt(&config->reset_gpio, 0);
		k_msleep(1);
		gpio_pin_set_dt(&config->reset_gpio, 1);
		k_msleep(120);
	}

	if (config->bl_gpio.port != NULL) {
		ret = gpio_pin_configure_dt(&config->bl_gpio,
				GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure Backlight GPIO (%d)", ret);
			return ret;
		}
	}

	ret = mw405_configure(dev);
	if (ret) {
		LOG_ERR("Failed to configure Panel. (%d)", ret);
		return ret;
	}

	data->orientation = DISPLAY_ORIENTATION_NORMAL;

	return 0;
}

#ifdef CONFIG_PM_DEVICE
/**
 * @brief Block the calling thread until any in-progress resume work
 * has completed.
 *
 * Internal only — called from within display API entry points below
 * so the application doesn't need to be aware of resume timing.
 * MUST only be called from a context that permits blocking
 * (application threads) — never from suspend()/resume() themselves.
 */
static inline void mw405_wait_ready(struct mw405_data *data)
{
	k_sem_take(&data->resume_sem, K_FOREVER);
	k_sem_give(&data->resume_sem);
}

/**
 * @brief MW405 suspend handler.
 *
 * Turns the display off, enters panel sleep mode, and disables the
 * backlight.
 *
 * @param dev MW405 device struct.
 * @return 0 on success, negative errno otherwise.
 */
static int mw405_suspend(const struct device *dev)
{
	const struct mw405_config *config = dev->config;
	struct mw405_data *data = dev->data;
	int ret;

	/*
	 * Cancel any pending resume work and wait for a running
	 * resume handler to complete.
	 */
	(void)k_work_cancel_delayable_sync(&data->reset_work,
					   &data->reset_work_sync);

	/*
	 * If resume work was cancelled before its handler ran,
	 * it never gets a chance to give resume_sem back.
	 *
	 * k_sem_give() is safe here because resume_sem has a
	 * limit of 1; if the worker already gave it, this is a no-op.
	 */
	k_sem_give(&data->resume_sem);

	/* Display OFF */
	ret = mipi_dsi_dcs_write(config->mipi_dsi,
				 config->channel,
				 MIPI_DCS_SET_DISPLAY_OFF,
				 NULL,
				 0);
	if (ret) {
		LOG_ERR("DISPLAY_OFF failed");
		return ret;
	}

	/* Enter sleep mode */
	ret = mipi_dsi_dcs_write(config->mipi_dsi,
				 config->channel,
				 MIPI_DCS_ENTER_SLEEP_MODE,
				 NULL,
				 0);
	if (ret) {
		LOG_ERR("ENTER_SLEEP failed");
		return ret;
	}

	/* Disable backlight */
	if (config->bl_gpio.port != NULL) {
		ret = gpio_pin_set_dt(&config->bl_gpio, 0);
		if (ret < 0) {
			LOG_ERR("Failed to turn Backlight GPIO OFF (%d)", ret);
			return ret;
		}
	}

	LOG_DBG("PM: Suspended %s", dev->name);

	return 0;
}

static void mw405_reset_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct mw405_data *data = CONTAINER_OF(dwork, struct mw405_data, reset_work);
	const struct device *dev = data->dev;
	const struct mw405_config *config = dev->config;
	struct mipi_dsi_device mdev;
	int ret;

	mdev.data_lanes = config->num_lanes;
	mdev.pixfmt = config->pixel_format;

	mdev.timings.hactive = TIMINGS_HACT;
	mdev.timings.hfp = TIMINGS_HFP;
	mdev.timings.hbp = TIMINGS_HBP;
	mdev.timings.hsync = TIMINGS_HSYNC_LEN;
	mdev.timings.vactive = TIMINGS_VACT;
	mdev.timings.vfp = TIMINGS_VFP;
	mdev.timings.vbp = TIMINGS_VBP;
	mdev.timings.vsync = TIMINGS_VSYNC_LEN;

	mdev.mode_flags = MIPI_DSI_MODE_VIDEO |
		MIPI_DSI_MODE_EOT_PACKET;

	if (config->cmd_type == CMD_LP) {
		mdev.mode_flags |= MIPI_DSI_MODE_LPM;
	}

	switch (config->vid_mode) {
	case BURST_MODE:
		mdev.mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
		break;
	case NON_BURST_MODE_SYNC_PULSE:
		mdev.mode_flags |=  MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
		break;
	case NON_BURST_MODE_SYNC_EVENTS:
	default:
		break;
	}

	/* Re-attach to DSI host */
	ret = mipi_dsi_attach(config->mipi_dsi, config->channel, &mdev);
	if (ret < 0) {
		LOG_ERR("Failed to re-attach to DSI host");
		goto done;
	}

	/* Hardware reset */
	if (config->reset_gpio.port) {
		gpio_pin_set_dt(&config->reset_gpio, 1);
		k_msleep(1);

		gpio_pin_set_dt(&config->reset_gpio, 0);
		k_msleep(1);

		gpio_pin_set_dt(&config->reset_gpio, 1);

		k_msleep(120);
	}

	/* Reprogram panel registers */
	ret = mw405_configure(dev);
	if (ret) {
		LOG_ERR("Failed to reconfigure panel");
		goto done;
	}

	data->orientation = DISPLAY_ORIENTATION_NORMAL;
	LOG_DBG("PM: Resumed %s", dev->name);

done:
	k_sem_give(&data->resume_sem);
}

/**
 * @brief MW405 resume handler.
 *
 * Re-attaches to the DSI host, performs a full hardware reset, reprograms
 * all panel registers, and re-enables the backlight.
 *
 * @param dev MW405 device struct.
 * @return 0 on success, negative errno otherwise.
 */
static int mw405_resume(const struct device *dev)
{
	const struct mw405_config *config = dev->config;
	struct mw405_data *data = dev->data;
	int ret;

	/* Hardware reset */
	if (config->reset_gpio.port) {
		ret = gpio_pin_configure_dt(&config->reset_gpio,
				GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure reset GPIO (%d)", ret);
			return ret;
		}
	}

	if (config->bl_gpio.port != NULL) {
		ret = gpio_pin_configure_dt(&config->bl_gpio,
				GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure Backlight GPIO (%d)", ret);
			return ret;
		}
	}

	/* DSI re-attach, hardware reset pulse, and panel reconfiguration all
	 * involve blocking delays; offload them to the system workqueue
	 * rather than blocking this calling context.
	 */
	(void)k_sem_take(&data->resume_sem, K_NO_WAIT);
	k_work_schedule(&data->reset_work, K_NO_WAIT);

	return 0;
}

/**
 * @brief MW405 PM device action handler.
 *
 * Handles power management state transitions for the MW405 panel.
 *
 * @param dev MW405 device struct.
 * @param action PM device action to perform.
 * @return 0 on success, negative errno otherwise.
 */
static int mw405_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {

	case PM_DEVICE_ACTION_SUSPEND:
		return mw405_suspend(dev);

	case PM_DEVICE_ACTION_RESUME:
		return mw405_resume(dev);

	case PM_DEVICE_ACTION_TURN_OFF:
	case PM_DEVICE_ACTION_TURN_ON:
	/* Power domain handling is automatic via the PM framework */
		return 0;

	default:
		return -ENOTSUP;
	}
}
#endif

#define MW405_PANEL(i)									\
	static const struct mw405_config config_##i = {					\
		.mipi_dsi = DEVICE_DT_GET(DT_INST_BUS(i)),				\
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(i, reset_gpios, {0}),		\
		.bl_gpio = GPIO_DT_SPEC_INST_GET_OR(i, bl_gpios, {0}),			\
		.num_lanes = DT_INST_PROP_BY_IDX(i, data_lanes, 0),			\
		.pixel_format = DT_INST_PROP(i, pixel_format),				\
		.active_width = DT_INST_PROP(i, width),					\
		.active_height = DT_INST_PROP(i, height),				\
		.channel = DT_INST_REG_ADDR(i),						\
		.vid_mode = DT_INST_ENUM_IDX_OR(i, video_mode, BURST_MODE),		\
		.cmd_type = DT_INST_ENUM_IDX_OR(i, command_tx_mode, CMD_LP),		\
	};										\
	static struct mw405_data data_##i;						\
	IF_ENABLED(CONFIG_PM_DEVICE, (PM_DEVICE_DT_INST_DEFINE(i, mw405_pm_action);))	\
	DEVICE_DT_INST_DEFINE(i,							\
			&mw405_init,							\
			PM_DEVICE_DT_INST_GET(i),					\
			&data_##i,							\
			&config_##i,							\
			POST_KERNEL,							\
			CONFIG_APPLICATION_INIT_PRIORITY,				\
			&mw405_api);

DT_INST_FOREACH_STATUS_OKAY(MW405_PANEL)
