/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_apmemory_aps512xxn

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include "ospi_hal.h"
#include "ospi.h"

LOG_MODULE_REGISTER(memc_alif_aps512xxn, CONFIG_MEMC_LOG_LEVEL);

#define DEVICE_NODE DT_NODELABEL(aps512xxn)
#define CONTROLLER_NODE  DT_PARENT(DEVICE_NODE)

/* APS256XXN Device ID */
#define APS256XXN_ID                           0xDE

/* APS256XXN Commands */
#define APS256XXN_CMD_SYNC_READ                0x0000
#define APS256XXN_CMD_SYNC_WRITE               0x8080
#define APS256XXN_CMD_LINEAR_BURST_READ        0x2020
#define APS256XXN_CMD_LINEAR_BURST_WRITE       0xA0A0
#define APS256XXN_CMD_MODE_REGISTER_READ       0x4040
#define APS256XXN_CMD_MODE_REGISTER_WRITE      0xC0C0
#define APS256XXN_CMD_GLOBAL_RESET             0xFFFF

/* APS256XXN macros */
#define APS256XXN_INIT_REG_READ_WAIT_CYCLES    4
#define APS256XXN_REG_READ_WAIT_CYCLES         3
#define APS256XXN_REG_WRITE_WAIT_CYCLES        0
#define APS256XXN_RESET_WAIT_CYCLES            3
#define APS256XXN_FAST_READ_WRITE_WAIT_CYCLES  3

/* APS256XXN Register address */
#define APS256XXN_MODE_REG0_ADDR               0x0
#define APS256XXN_MODE_REG1_ADDR               0x1
#define APS256XXN_MODE_REG2_ADDR               0x2
#define APS256XXN_MODE_REG3_ADDR               0x3
#define APS256XXN_MODE_REG4_ADDR               0x4
#define APS256XXN_MODE_REG6_ADDR               0x6
#define APS256XXN_MODE_REG8_ADDR               0x8

/* APS256XXN OSPI macros */
#define APS256XXN_OSPI_DDR_DRIVE_EDGE          1
#define APS256XXN_OSPI_RX_SAMPLE_DELAY         0
#define APS256XXN_OSPI_RX_FIFO_THRESHOLD       0
#define APS256XXN_OSPI_DFS                     16

/**IRQ declaration */
typedef void (*irq_config_func_t)(const struct device *dev);

/* APS256XXN Device Configuration */
struct alif_ospi_aps512xxn_config {
	struct ospi_regs *regs;
	struct ospi_aes_regs *aes_regs;
	uint32_t input_clk;
	uint32_t bus_speed;
	uint32_t cs_pin;
	const struct pinctrl_dev_config *pcfg;
	irq_config_func_t irq_config;
	uint8_t  signal_delay;
	uint8_t  rxds_delay;
	uint8_t  xip_wait_cycles;
	bool dual_octal;
};

/* APS256XXN Device Data */
struct alif_ospi_aps512xxn_data {
	HAL_OSPI_Handle_T ospi_handle;
	struct ospi_trans_config trans_conf;
	struct k_event event;
};

static int32_t err_map_alif_hal_to_zephyr(int32_t err)
{
	int err_code;

	switch (err) {
	case OSPI_ERR_NONE:
		err_code = 0;
		break;
	case OSPI_ERR_INVALID_PARAM:
	case OSPI_ERR_INVALID_HANDLE:
		err_code = -EINVAL;
		break;
	case OSPI_ERR_INVALID_STATE:
		err_code = -EPERM;
		break;
	case OSPI_ERR_CTRL_BUSY:
		err_code = -EBUSY;
		break;
	default:
		err_code = -EIO;
	}

	return err_code;
}


static void ospi_hal_event_update(uint32_t event_status, void *user_data)
{
	struct alif_ospi_aps512xxn_data *dev_data = (struct alif_ospi_aps512xxn_data *)user_data;

	k_event_post(&dev_data->event, event_status);
}

static int aps512xxn_global_reset(const struct device *dev)
{
	const struct alif_ospi_aps512xxn_config *config = dev->config;
	struct alif_ospi_aps512xxn_data *data = dev->data;
	int32_t ret;
	uint32_t cmd_buff, event;

	data->trans_conf.addr_len = OSPI_ADDR_LENGTH_0_BITS;
	data->trans_conf.wait_cycles = APS256XXN_RESET_WAIT_CYCLES;

	ret = alif_hal_ospi_prepare_transfer(data->ospi_handle, &data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	k_event_clear(&data->event, OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST);

	ospi_control_ss(config->regs, config->cs_pin, SPI_SS_STATE_ENABLE);

	cmd_buff = APS256XXN_CMD_GLOBAL_RESET;
	ret = alif_hal_ospi_send(data->ospi_handle, &cmd_buff, 1U);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	event = k_event_wait(&data->event,
			OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST, false, K_FOREVER);
	/* Check the Event Status*/
	if (!(event & OSPI_EVENT_TRANSFER_COMPLETE)) {
		ret = -EIO;
	}

	ospi_control_ss(config->regs, config->cs_pin, SPI_SS_STATE_DISABLE);

	return ret;
}

static int aps512xxn_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t value)
{
	const struct alif_ospi_aps512xxn_config *config = dev->config;
	struct alif_ospi_aps512xxn_data *data = dev->data;
	int32_t ret;
	uint32_t cmd_buff[3], event;

	data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;
	data->trans_conf.wait_cycles = APS256XXN_REG_WRITE_WAIT_CYCLES;

	ret = alif_hal_ospi_prepare_transfer(data->ospi_handle, &data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	k_event_clear(&data->event, OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST);

	ospi_control_ss(config->regs, config->cs_pin, SPI_SS_STATE_ENABLE);

	cmd_buff[0] = APS256XXN_CMD_MODE_REGISTER_WRITE;
	cmd_buff[1] = reg_addr;
	cmd_buff[2] = (value << 8);

	ret = alif_hal_ospi_send(data->ospi_handle, cmd_buff, 3U);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	event = k_event_wait(&data->event,
			OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST, false, K_FOREVER);
	/* Check the Event Status*/
	if (!(event & OSPI_EVENT_TRANSFER_COMPLETE)) {
		ret = -EIO;
	}

	ospi_control_ss(config->regs, config->cs_pin, SPI_SS_STATE_DISABLE);

	return ret;
}

static int aps512xxn_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *value,
		uint8_t wait_cycles)
{
	const struct alif_ospi_aps512xxn_config *config = dev->config;
	struct alif_ospi_aps512xxn_data *data = dev->data;
	int32_t ret;
	uint32_t cmd_buff[2], event;
	uint16_t data_buff;

	data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;
	data->trans_conf.wait_cycles = wait_cycles;

	ret = alif_hal_ospi_prepare_transfer(data->ospi_handle, &data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	k_event_clear(&data->event, OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST);

	ospi_control_ss(config->regs, config->cs_pin, SPI_SS_STATE_ENABLE);

	cmd_buff[0] = APS256XXN_CMD_MODE_REGISTER_READ;
	cmd_buff[1] = reg_addr;
	ret = alif_hal_ospi_transfer(data->ospi_handle, cmd_buff, &data_buff, 1);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	event = k_event_wait(&data->event,
			OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST, false, K_FOREVER);
	/* Check the Event Status*/
	if (!(event & OSPI_EVENT_TRANSFER_COMPLETE)) {
		ret = -EIO;
		goto irq_failed;
	}

	*value = (data_buff >> 8);

irq_failed:
	ospi_control_ss(config->regs, config->cs_pin, SPI_SS_STATE_DISABLE);

	return ret;
}

static int memc_alif_ospi_aps512xxn_init(const struct device *dev)
{
	const struct alif_ospi_aps512xxn_config *config = dev->config;
	struct alif_ospi_aps512xxn_data *data = dev->data;
	struct ospi_xip_config aps512xxn_xip_cfg;
	uint8_t id_reg = 0;
	int32_t ret;
	struct ospi_init init_config;

	k_event_init(&(data->event));

	pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	/* IRQ Init */
	config->irq_config(dev);

	memset(&init_config, 0, sizeof(struct ospi_init));
	memset(&data->trans_conf, 0, sizeof(struct ospi_trans_config));

	/* OSPI Initialize Configuration */
	init_config.core_clk = config->input_clk;
	init_config.bus_speed = config->bus_speed;
	init_config.tx_fifo_threshold = DT_PROP(CONTROLLER_NODE, tx_fifo_threshold);
	init_config.rx_fifo_threshold = APS256XXN_OSPI_RX_FIFO_THRESHOLD;
	init_config.rx_sample_delay = APS256XXN_OSPI_RX_SAMPLE_DELAY;
	init_config.ddr_drive_edge = DT_PROP(CONTROLLER_NODE, ddr_drive_edge);
	init_config.cs_pin = config->cs_pin;
	init_config.rx_ds_delay = config->rxds_delay;
	init_config.baud2_delay = OSPI_BAUD2_DELAY_DISABLE;
	init_config.base_regs = (uint32_t *) config->regs;
	init_config.aes_regs = (uint32_t *) config->aes_regs;
	init_config.event_cb = ospi_hal_event_update;
	init_config.user_data = data;

	ret = alif_hal_ospi_initialize(&data->ospi_handle, &init_config);
	if (ret != 0) {
		LOG_ERR("Error in OSPI Initialize");
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	/* Common OSPI transfer settings for RAM */
	data->trans_conf.frame_size = APS256XXN_OSPI_DFS;
	data->trans_conf.frame_format = OSPI_FRF_OCTAL;
	data->trans_conf.ddr_enable = (OSPI_DDR_ENABLE | OSPI_INST_DDR_ENABLE);
	data->trans_conf.inst_len = OSPI_INST_LENGTH_16_BITS;

	ret = alif_hal_ospi_prepare_transfer(data->ospi_handle, &data->trans_conf);
	if (ret != 0) {
		LOG_ERR("Error in OSPI Initial Configuration");
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	if (config->bus_speed == 0) {
		LOG_ERR("OSPI Bus Speed can't be zero");
		return -EINVAL;
	}

	/* APS256XXN global reset */
	ret = aps512xxn_global_reset(dev);
	if (ret != 0) {
		LOG_ERR("APS512XXN Global Reset Failed");
		return ret;
	}

	/* Read APS256XXN ID */
	ret = aps512xxn_read_reg(dev, APS256XXN_MODE_REG2_ADDR, &id_reg,
				APS256XXN_INIT_REG_READ_WAIT_CYCLES);
	if ((ret != 0) || (id_reg != APS256XXN_ID)) {
		LOG_ERR("APS512XXN Read ID Reg Failed");
		return ret;
	}

	/* Set minimum read latency (4 cycles), variable latency */
	ret = aps512xxn_write_reg(dev, APS256XXN_MODE_REG0_ADDR, 0x04);
	if (ret != 0) {
		LOG_ERR("APS512XXN Mode Reg 0 Config Failed");
		return ret;
	}

	/* Set minimum write latency (4 cycles), auto slow refresh rate */
	ret = aps512xxn_write_reg(dev, APS256XXN_MODE_REG4_ADDR, 0x98);
	if (ret != 0) {
		LOG_ERR("APS512XXN Mode Reg 4 Config Failed");
		return ret;
	}

	/* Set transmission mode (x8/x16), 32-byte wrap, no row cross */
	ret = aps512xxn_write_reg(dev, APS256XXN_MODE_REG8_ADDR, config->dual_octal ? 0x41 : 0x01);
	if (ret != 0) {
		LOG_ERR("APS512XXN Mode Reg 8 Config Failed");
		return ret;
	}

	aps512xxn_xip_cfg.xip_mod_bits = 0;
	aps512xxn_xip_cfg.incr_cmd = APS256XXN_CMD_LINEAR_BURST_READ;
	aps512xxn_xip_cfg.wrap_cmd = APS256XXN_CMD_SYNC_READ;
	aps512xxn_xip_cfg.write_incr_cmd = APS256XXN_CMD_LINEAR_BURST_WRITE;
	aps512xxn_xip_cfg.write_wrap_cmd = APS256XXN_CMD_SYNC_WRITE;
	aps512xxn_xip_cfg.xip_cnt_time_out = config->xip_wait_cycles;
	aps512xxn_xip_cfg.xip_wait_cycles = APS256XXN_FAST_READ_WRITE_WAIT_CYCLES;

	/* OSPI xip configuration */
	ospi_psram_xip_init(config->regs, &aps512xxn_xip_cfg, config->dual_octal);

	aes_enable_xip(config->aes_regs);

	return 0;
}

static void OSPI_IRQHandler(const struct device *dev)
{
	struct alif_ospi_aps512xxn_data *data = (struct alif_ospi_aps512xxn_data *)dev->data;

	/* ospi-irq handler */
	alif_hal_ospi_irq_handler(data->ospi_handle);
}

/* PINCTRL Definition Macro for Node */
PINCTRL_DT_DEFINE(CONTROLLER_NODE);

static void ospi_irq_config_func(const struct device *dev)
{
	IRQ_CONNECT(DT_IRQN(CONTROLLER_NODE), DT_IRQ(CONTROLLER_NODE, priority), OSPI_IRQHandler,
	    DEVICE_DT_GET(DEVICE_NODE), 0);
	irq_enable(DT_IRQN(CONTROLLER_NODE));
}

static const struct alif_ospi_aps512xxn_config aps512xxn_config = {
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(CONTROLLER_NODE),
	.regs = (struct ospi_regs *) DT_REG_ADDR(CONTROLLER_NODE),
	.aes_regs = (struct ospi_aes_regs *) DT_PROP_BY_IDX(CONTROLLER_NODE, aes_reg, 0),
	.input_clk = DT_PROP(CONTROLLER_NODE, clock_frequency),
	.bus_speed = DT_PROP(CONTROLLER_NODE, bus_speed),
	.signal_delay = DT_PROP(CONTROLLER_NODE, baud2_delay),
	.rxds_delay = DT_PROP(CONTROLLER_NODE, rx_ds_delay),
	.xip_wait_cycles = DT_PROP(CONTROLLER_NODE, xip_wait_cycles),
	.dual_octal = DT_PROP(DEVICE_NODE, x16_data_transfer_mode),
	.irq_config = ospi_irq_config_func
};

static struct alif_ospi_aps512xxn_data aps512xxn_data;

DEVICE_DT_DEFINE(DEVICE_NODE,
		&memc_alif_ospi_aps512xxn_init,
		NULL,
		&aps512xxn_data,
		&aps512xxn_config,
		POST_KERNEL,
		CONFIG_MEMC_INIT_PRIORITY,
		NULL);
