/* GPIO I2C Multiplexer using TI TS3A24157RSER */

/*
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gpio_i2c_mux

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ALIF_I2C_MUX);

#define MAX_NUM_CHANNELS    2
#define I2C_TOGGLE_DELAY_US 10
#define I2C_LOCK_TIMEOUT_US (I2C_TOGGLE_DELAY_US * 100)

struct i2c_mux_config {
	const struct device *bus;
	const struct gpio_dt_spec ctrl_gpio;
};

struct i2c_mux_data {
	struct k_mutex lock;
	uint8_t current_channel;
};

struct i2c_mux_channel_config {
	const struct device *root;
	uint8_t chan_num;
};

static inline const struct i2c_mux_config *
get_mux_config_from_channel(const struct device *dev)
{
	const struct i2c_mux_channel_config *channel_config = dev->config;

	return channel_config->root->config;
}

static inline struct i2c_mux_data *
get_mux_data_from_channel(const struct device *dev)
{
	const struct i2c_mux_channel_config *channel_config = dev->config;

	return channel_config->root->data;
}

static int i2c_mux_set_channel(const struct device *dev, uint8_t channel_number)
{
	const struct i2c_mux_config *config = dev->config;
	struct i2c_mux_data *data = dev->data;

	/* Change channel if it is different from current channel */
	if (data->current_channel != channel_number) {
		switch (channel_number) {
		case 0:
			gpio_pin_set_dt(&config->ctrl_gpio, 0);
			break;
		case 1:
			gpio_pin_set_dt(&config->ctrl_gpio, 1);
			break;
		default:
			LOG_ERR("This is a GPIO controlled I2C mux! "
				 "Only 1/0 Allowed");
			return -ENODEV;
		}
		data->current_channel = channel_number;
	}
	return 0;
}

static int i2c_mux_channel_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_mux_config *cfg = get_mux_config_from_channel(dev);

	return i2c_configure(cfg->bus, dev_config);
}

static int i2c_mux_transfer(const struct device *dev,
		struct i2c_msg *msgs,
		uint8_t num_msgs,
		uint16_t addr)
{
	struct i2c_mux_data *data = get_mux_data_from_channel(dev);
	const struct i2c_mux_channel_config *channel_config =
		dev->config;
	const struct i2c_mux_config *config =
		get_mux_config_from_channel(dev);
	int ret;

	ret = k_mutex_lock(&data->lock, K_USEC(I2C_TOGGLE_DELAY_US));
	if (ret) {
		return ret;
	}

	ret = i2c_mux_set_channel(channel_config->root, channel_config->chan_num);
	if (ret) {
		goto i2c_mux_mutex_unlock;
	}

	k_busy_wait(I2C_TOGGLE_DELAY_US);
	ret = i2c_transfer(config->bus, msgs, num_msgs, addr);

i2c_mux_mutex_unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

static DEVICE_API(i2c, i2c_mux_api) = {
	.configure = i2c_mux_channel_configure,
	.transfer = i2c_mux_transfer,
};

static int i2c_mux_init(const struct device *dev)
{
	const struct i2c_mux_config *cfg = dev->config;
	struct i2c_mux_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->ctrl_gpio.port)) {
		LOG_ERR("I2C Multiplexer\'s Control GPIO is not ready!");
		return -ENODEV;
	}

	/* Control GPIO is configured and set to default to zero state. */
	ret = gpio_pin_configure_dt(&cfg->ctrl_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_ERR("Failed to configure control GPIO!");
		return ret;
	}

	data->current_channel = 0;

	return 0;
}

static int i2c_mux_channel_init(const struct device *dev)
{
	const struct i2c_mux_channel_config *channel_config = dev->config;

	if (!device_is_ready(channel_config->root)) {
		LOG_ERR("I2C mux root %s is not ready", channel_config->root->name);
		return -ENODEV;
	}

	if (channel_config->chan_num >= MAX_NUM_CHANNELS) {
		LOG_ERR("Incorrect node settings for %s node", dev->name);
		return -ENODEV;
	}

	return 0;
}

#define I2C_MUX_CHILD_DEFINE(node)                                          \
	static const struct i2c_mux_channel_config                          \
	UTIL_CAT(i2c_mux_channel_, DT_REG_ADDR(node)) = {                   \
		.chan_num = DT_REG_ADDR(node),                              \
		.root = DEVICE_DT_GET(DT_PARENT(node)),                     \
	};                                                                  \
	DEVICE_DT_DEFINE(node,                                              \
			i2c_mux_channel_init,                               \
			NULL,                                               \
			NULL,                                               \
			&UTIL_CAT(i2c_mux_channel_, DT_REG_ADDR(node)),     \
			POST_KERNEL,                                        \
			CONFIG_I2C_INIT_PRIORITY,                           \
			&i2c_mux_api);

#define I2C_MUX_CONTROLLER(i)                                                \
                                                                             \
	static struct i2c_mux_data i2c_mux_data_##i = {                      \
		.lock = Z_MUTEX_INITIALIZER(i2c_mux_data_##i.lock),          \
	};                                                                   \
                                                                             \
	static const struct i2c_mux_config i2c_mux_config_##i = {            \
		.bus = DEVICE_DT_GET(DT_INST_PHANDLE(i, i2c_inst)),          \
		.ctrl_gpio = GPIO_DT_SPEC_INST_GET(i, ctrl_gpios),           \
	};                                                                   \
                                                                             \
	I2C_DEVICE_DT_INST_DEFINE(i,                                         \
			&i2c_mux_init,                                       \
			NULL,                                                \
			&i2c_mux_data_##i,                                   \
			&i2c_mux_config_##i,                                 \
			POST_KERNEL,                                         \
			CONFIG_I2C_INIT_PRIORITY,                            \
			NULL);                                               \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(i, I2C_MUX_CHILD_DEFINE);

DT_INST_FOREACH_STATUS_OKAY(I2C_MUX_CONTROLLER)
