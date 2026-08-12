/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

#define DT_DRV_COMPAT alif_lpi2c

struct lpi2c_config {
	DEVICE_MMIO_ROM;
	void (*irq_config_func)(const struct device *dev);
	const struct pinctrl_dev_config *pcfg;
};

struct lpi2c_data {
	DEVICE_MMIO_RAM;
	uint8_t *xfr_buf;
	uint32_t xfr_len;
	uint32_t tx_curr_cnt;
	uint32_t rx_curr_cnt;
	struct i2c_target_config *slave_cfg;
};

/* lpi2c register macros */
#define LPI2C_DATA_REG (0x00)		/* data  register            */
#define LPI2C_INBOUND_FIFO_REG  (0x10)  /* inbound fifo     register */
#define LPI2C_OUTBOUND_FIFO_REG (0x20)  /* outbound fifo    register */

/* Macro */
#define LPI2C_FIFO_EMPTY (0x20) /* LPI2C Fifo empty     */
#define LPI2C_FIFO_FULL (0x10)  /* LPI2C Fifo full      */
#define LPI2C_AVL_DATA (0X0F)   /* LPI2C number of data */

#define LPI2C_MAX_FIFO_LEN			(8U)	/* Maximum data length */

/*
 *  bus speed = 1 / 100 * 10^3  = 10us
 *  1 bit period = 10us
 *
 *  1 byte = 10us * 8bits = 80us
 *
 */
#define LPI2C_1BYTE_XFER_TIME_US	(80)

static int lpi2c_initialize(const struct device *dev)
{
	int err;
	const struct lpi2c_config *config = dev->config;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}
	config->irq_config_func(dev);

	return 0;
}

static void lpi2c_isr(const struct device *dev)
{
	uint32_t regs = DEVICE_MMIO_GET(dev);
	uint8_t read_byte;
	struct lpi2c_data *data = dev->data;
	const struct i2c_target_callbacks *slave_cb = data->slave_cfg->callbacks;

	read_byte = sys_read8(regs + LPI2C_DATA_REG);

	if (slave_cb->write_received) {
		slave_cb->write_received(data->slave_cfg, read_byte);
	}
}

static int lpi2c_slave_register(const struct device *dev,
				struct i2c_target_config *cfg)
{
	int ret = 0;
	struct lpi2c_data *data = dev->data;

	data->slave_cfg = cfg;
	return ret;
}

static uint32_t lpi2c_fifo_rem_len(const struct device *dev)
{
	uint32_t regs = DEVICE_MMIO_GET(dev);
	uint8_t avl_data;

	avl_data = sys_read8(regs + LPI2C_OUTBOUND_FIFO_REG) & LPI2C_AVL_DATA;

	return (LPI2C_MAX_FIFO_LEN - avl_data);
}

static int lpi2c_transfer(const struct device *dev, struct i2c_msg *msgs,
			  uint8_t num_msgs, uint16_t slave_address)
{
	struct lpi2c_data *data = dev->data;
	uintptr_t regs = DEVICE_MMIO_GET(dev);
	struct i2c_msg *cur_msg = msgs;
	uint8_t msg_left = num_msgs;
	uint8_t xmit_data;

	ARG_UNUSED(slave_address);

	/* Process all the messages */
	while (msg_left > 0) {
		data->xfr_buf = cur_msg->buf;
		data->xfr_len = cur_msg->len;
		data->tx_curr_cnt = 0U;

		while (data->tx_curr_cnt < data->xfr_len) {

			if (lpi2c_fifo_rem_len(dev)) {

				/* load value to be transmit */
				xmit_data = data->xfr_buf[0];

				data->tx_curr_cnt++;
				data->xfr_buf++;

				sys_write8(xmit_data, (regs + LPI2C_DATA_REG));
			}
		}

		/* wait for fifo to be empty */
		while (!(sys_read8(regs + LPI2C_OUTBOUND_FIFO_REG) & LPI2C_FIFO_EMPTY)) {
		}

		/* 80us delay */
		k_usleep(LPI2C_1BYTE_XFER_TIME_US);

		cur_msg++;
		msg_left--;
	}
	return 0;
}

static const struct i2c_driver_api funcs = {
		.transfer = lpi2c_transfer,
		.target_register = lpi2c_slave_register,
};

#define LPI2C_DEVICE_INIT_DW(inst)                                                                 \
	static void lpi2c_config_func_##inst(const struct device *dev);                            \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static struct lpi2c_data data_##inst;                                                      \
	const struct lpi2c_config config_##inst = {                                                \
			DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                   \
			.irq_config_func = lpi2c_config_func_##inst,                               \
			.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                              \
	};                                                                                         \
	I2C_DEVICE_DT_INST_DEFINE(inst, lpi2c_initialize, NULL, &data_##inst,                      \
				&config_##inst, POST_KERNEL,                                       \
				CONFIG_I2C_INIT_PRIORITY, &funcs);                                 \
	static void lpi2c_config_func_##inst(const struct device *dev)                             \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), lpi2c_isr,            \
				DEVICE_DT_INST_GET(inst), 0);                                      \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}

DT_INST_FOREACH_STATUS_OKAY(LPI2C_DEVICE_INIT_DW)
