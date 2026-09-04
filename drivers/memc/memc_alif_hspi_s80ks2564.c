/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_infineon_s80ks2564

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "ospi.h"

LOG_MODULE_REGISTER(memc_alif_s80ks2564, CONFIG_MEMC_LOG_LEVEL);

#define S80KS2564_DDR_DRIVE_EDGE        1
#define S80KS2564_RX_SAMPLE_DELAY       5
#define S80KS2564_WRITE_CONFIG_REG_DFS  16
#define S80KS2564_DFS                   32
#define S80KS2564_CMD_BUF_SIZE          3
#define SPI_MODE_DUAL_OCTAL             1

#define S80KS2564_BURST_LEN_POS         0
#define S80KS2564_WRAPPED_BURST_SEQ_POS 2
#define S80KS2564_FIXED_LATENCY_EN_POS  3
#define S80KS2564_INIT_LATENCY_POS      4
#define S80KS2564_DRIVE_STRENGTH_POS    12
#define S80KS2564_OPERATION_MODE_POS    15

/* Device Configuration */
struct alif_hspi_s80ks2564_config {
	struct ospi_regs *regs;
	struct ospi_aes_regs *aes_regs;
	uint32_t input_clk;
	uint32_t bus_speed;
	uint32_t cs_pin;
	const struct pinctrl_dev_config *pcfg;
	const struct gpio_dt_spec reset_gpio;
	uint8_t  latency_val;
	uint8_t  signal_delay;
	uint8_t  rxds_delay;
};

static int memc_alif_hspi_s80ks2564_init(const struct device *dev)
{
	const struct alif_hspi_s80ks2564_config *config = dev->config;
	struct ospi_transfer hspi_cfg;
	uint32_t cmd_buff[S80KS2564_CMD_BUF_SIZE], baud;
	int32_t ret;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Could not configure OSPI pins (%d)", ret);
		return ret;
	}

	if (config->reset_gpio.port != NULL) {
		ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure reset GPIO (%d)", ret);
			return ret;
		}
		ret = gpio_pin_set_dt(&config->reset_gpio, GPIO_ACTIVE_LOW);
		if (ret < 0) {
			LOG_ERR("Could not set reset GPIO (%d) to low", ret);
			return ret;
		}

		ret = gpio_pin_set_dt(&config->reset_gpio, GPIO_ACTIVE_HIGH);
		if (ret < 0) {
			LOG_ERR("Could not set reset GPIO (%d) to high", ret);
			return ret;
		}
	}

	if (config->bus_speed == 0) {
		return -EINVAL;
	}

	baud = (config->input_clk / config->bus_speed);

	if (baud == 0) {
		return -ENOTSUP;
	}

	if (baud < 4) {
		aes_set_signal_delay(config->aes_regs, config->signal_delay);
	}

	ospi_set_bus_speed(config->regs, config->bus_speed, config->input_clk);

	ospi_set_ddr_drive_edge(config->regs, S80KS2564_DDR_DRIVE_EDGE);
	ospi_set_rx_sample_delay(config->regs, S80KS2564_RX_SAMPLE_DELAY);
	aes_set_rxds_delay(config->aes_regs, config->rxds_delay);

	/*
	 * CA bit assignment for Configuration Register 0 write operation
	 * bit[47] - bit[40] -> 60h
	 * bit[39] - bit[32] -> 00h
	 * bit[31] - bit[24] -> 01h
	 * bit[23] - bit[16] -> 00h
	 * bit[15] - bit[8]  -> 00h
	 * bit[7]  - bit[0]  -> 00h
	 */
	cmd_buff[0] = 0x60000100; /* bit[8] - bit[47] */
	cmd_buff[1] = 0x0;        /* bit[0] - bit[7] */
	cmd_buff[2] = ((1 << S80KS2564_OPERATION_MODE_POS)
		| (0 << S80KS2564_DRIVE_STRENGTH_POS)    /* 34 ohm drive strength */
		| (((config->latency_val - 5) & 0xF) << S80KS2564_INIT_LATENCY_POS) /* latency */
		| (0 << S80KS2564_FIXED_LATENCY_EN_POS)    /* variable latency */
		| (1 << S80KS2564_WRAPPED_BURST_SEQ_POS)   /* standard wrapped burst sequence */
		| (2 << S80KS2564_BURST_LEN_POS));         /* 16-word (32-byte) wrap */

	hspi_cfg.spi_frf        = SPI_FRF_OCTAL;
	hspi_cfg.ddr            = 1;
	hspi_cfg.inst_len       = SPI_CTRLR0_INST_L_0bit;
	hspi_cfg.addr_len       = 12;
	hspi_cfg.dummy_cycle    = 0;
	hspi_cfg.tx_total_cnt   = 3;
	hspi_cfg.tx_current_cnt = 0;
	hspi_cfg.tx_buff        = cmd_buff;

	ospi_set_dfs(config->regs, S80KS2564_WRITE_CONFIG_REG_DFS);

	ospi_control_ss(config->regs, config->cs_pin, SPI_SS_STATE_ENABLE);
	ospi_hyperbus_send(config->regs, &hspi_cfg);
	ospi_control_ss(config->regs, config->cs_pin, SPI_SS_STATE_DISABLE);

	ospi_set_dfs(config->regs, S80KS2564_DFS);

	ospi_hyperbus_xip_init(config->regs, config->latency_val, SPI_MODE_DUAL_OCTAL);

	aes_enable_xip(config->aes_regs);

	return 0;
}

#define DEVICE_NODE DT_NODELABEL(s80ks2564)
#define CONTROL_NODE  DT_PARENT(DEVICE_NODE)

/* PINCTRL Definition Macro for Node */
PINCTRL_DT_DEFINE(CONTROL_NODE);

static const struct alif_hspi_s80ks2564_config s80ks2564_config = {
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(CONTROL_NODE),
	.regs = (struct ospi_regs *) DT_REG_ADDR(CONTROL_NODE),
	.aes_regs = (struct ospi_aes_regs *) DT_PROP_BY_IDX(CONTROL_NODE, aes_reg, 0),
	.input_clk = DT_PROP(CONTROL_NODE, clock_frequency),
	.bus_speed = DT_PROP(CONTROL_NODE, bus_speed),
	.signal_delay = DT_PROP(CONTROL_NODE, baud2_signal_delay),
	.rxds_delay = DT_PROP(CONTROL_NODE, rx_ds_delay),
	.latency_val = DT_PROP(DEVICE_NODE, latency),
	.reset_gpio = GPIO_DT_SPEC_GET_OR(DEVICE_NODE, reset_gpios, {0})
};

DEVICE_DT_DEFINE(DEVICE_NODE,
			&memc_alif_hspi_s80ks2564_init,
			NULL,
			NULL,
			&s80ks2564_config,
			POST_KERNEL,
			CONFIG_MEMC_INIT_PRIORITY,
			NULL);
