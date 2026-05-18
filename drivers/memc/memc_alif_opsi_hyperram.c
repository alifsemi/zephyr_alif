/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT issi_wvh64m8eall

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control.h>

#include "ospi_hal.h"
#include "ospi.h"

LOG_MODULE_REGISTER(memc_alif_hyperram, CONFIG_MEMC_LOG_LEVEL);

#define DEVICE_NODE DT_NODELABEL(ospi_ram)
#define CONTROLLER_NODE DT_PARENT(DEVICE_NODE)

/* HyperRAM OSPI macros */
#define HYPERRAM_OSPI_DFS		16U

typedef void (*irq_config_func_t)(const struct device *dev);

struct alif_ospi_hyperram_config {
	struct ospi_regs *regs;
	struct ospi_aes_regs *aes_regs;
	const struct device *clk_dev;
	clock_control_subsys_t clkid;
	uint32_t bus_speed;
	uint32_t cs_pin;
	uint32_t rx_fifo_threshold;
	uint32_t rx_sample_delay;
	const struct pinctrl_dev_config *pcfg;
	irq_config_func_t irq_config;
	uint8_t rxds_delay;
};

struct alif_ospi_hyperram_data {
	uint32_t freq;
	HAL_OSPI_Handle_T ospi_handle;
	struct ospi_trans_config trans_conf;
	struct k_event event;
};

static int32_t err_map_alif_hal_to_zephyr(int32_t err)
{
	switch (err) {
	case OSPI_ERR_NONE:
		return 0;
	case OSPI_ERR_INVALID_PARAM:
	case OSPI_ERR_INVALID_HANDLE:
		return -EINVAL;
	case OSPI_ERR_INVALID_STATE:
		return -EPERM;
	case OSPI_ERR_CTRL_BUSY:
		return -EBUSY;
	default:
		return -EIO;
	}
}

static void ospi_hal_event_update(uint32_t event_status, void *user_data)
{
	struct alif_ospi_hyperram_data *dev_data = (struct alif_ospi_hyperram_data *)user_data;

	k_event_post(&dev_data->event, event_status);
}

static int memc_alif_ospi_hyperram_init(const struct device *dev)
{
	const struct alif_ospi_hyperram_config *config = dev->config;
	struct alif_ospi_hyperram_data *data = dev->data;
	struct ospi_xip_config xip_cfg;
	struct ospi_init init_config;
	int32_t ret;

	if (config->bus_speed == 0U) {
		LOG_ERR("OSPI bus speed can't be zero");
		return -EINVAL;
	}

	k_event_init(&data->event);

	pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	/* IRQ init */
	config->irq_config(dev);

	memset(&init_config, 0, sizeof(init_config));
	memset(&data->trans_conf, 0, sizeof(data->trans_conf));

	if (!device_is_ready(config->clk_dev)) {
		LOG_ERR("clock controller device not ready");
		return -ENODEV;
	}

#if defined(CONFIG_ENSEMBLE_GEN2)
	ret = clock_control_configure(config->clk_dev, config->clkid, NULL);
	if (ret != 0) {
		LOG_ERR("Unable to configure clock: err:%d", ret);
		return ret;
	}

	ret = clock_control_on(config->clk_dev, config->clkid);
	if (ret != 0) {
		LOG_ERR("Unable to turn on clock: err:%d", ret);
		return ret;
	}
#endif
	ret = clock_control_get_rate(config->clk_dev, config->clkid, &data->freq);
	if (ret != 0) {
		LOG_ERR("Unable to get clock rate: err:%d", ret);
		return ret;
	}

	init_config.core_clk = data->freq;
	init_config.bus_speed = config->bus_speed;
	init_config.tx_fifo_threshold = DT_PROP(CONTROLLER_NODE, tx_fifo_threshold);
	init_config.rx_fifo_threshold = config->rx_fifo_threshold;
	init_config.rx_sample_delay = config->rx_sample_delay;
	init_config.ddr_drive_edge = DT_PROP(CONTROLLER_NODE, ddr_drive_edge);
	init_config.cs_pin = config->cs_pin;
	init_config.rx_ds_delay = config->rxds_delay;
	init_config.baud2_delay = DT_PROP(CONTROLLER_NODE, baud2_delay);
	init_config.base_regs = (uint32_t *)config->regs;
	init_config.aes_regs = (uint32_t *)config->aes_regs;
	init_config.event_cb = ospi_hal_event_update;
	init_config.user_data = data;

	ret = alif_hal_ospi_initialize(&data->ospi_handle, &init_config);
	if (ret != 0) {
		LOG_ERR("Error in OSPI initialize");
		return err_map_alif_hal_to_zephyr(ret);
	}

	/* HyperRAM RW transactions are octal DDR commands with 16-bit frames. */
	data->trans_conf.frame_size = HYPERRAM_OSPI_DFS;
	data->trans_conf.frame_format = OSPI_FRF_OCTAL;
	data->trans_conf.ddr_enable = OSPI_DDR_ENABLE;

	ret = alif_hal_ospi_prepare_transfer(data->ospi_handle, &data->trans_conf);
	if (ret != 0) {
		LOG_ERR("Error in OSPI initial configuration");
		return err_map_alif_hal_to_zephyr(ret);
	}

	xip_cfg.xip_wait_cycles = DT_PROP(CONTROLLER_NODE, xip_wait_cycles);
	xip_cfg.xip_cs_pin = config->cs_pin;

	/*
	 * HyperRAM parts in this family power up in a usable default state.
	 * The controller-side XiP timing is driven from the OSPI node's
	 * xip-wait-cycles property.
	 */
	ospi_hyperbus_xip_init(config->regs, &xip_cfg);

	aes_enable_xip(config->aes_regs);

	return 0;
}

static void OSPI_IRQHandler(const struct device *dev)
{
	struct alif_ospi_hyperram_data *data = (struct alif_ospi_hyperram_data *)dev->data;

	alif_hal_ospi_irq_handler(data->ospi_handle);
}

/* PINCTRL Definition Macro for Node */
PINCTRL_DT_DEFINE(CONTROLLER_NODE);

static void ospi_irq_config_func(const struct device *dev)
{
	IRQ_CONNECT(DT_IRQN(CONTROLLER_NODE), DT_IRQ(CONTROLLER_NODE, priority),
		    OSPI_IRQHandler, DEVICE_DT_GET(DEVICE_NODE), 0);
	irq_enable(DT_IRQN(CONTROLLER_NODE));
}

static const struct alif_ospi_hyperram_config hyperram_config = {
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(CONTROLLER_NODE),
	.regs = (struct ospi_regs *)DT_REG_ADDR(CONTROLLER_NODE),
	.aes_regs = (struct ospi_aes_regs *)DT_PROP_BY_IDX(CONTROLLER_NODE, aes_reg, 0),
	.bus_speed = DT_PROP(CONTROLLER_NODE, bus_speed),
	.rx_fifo_threshold = 0,
	.rx_sample_delay = 0,
	.rxds_delay = DT_PROP(CONTROLLER_NODE, rx_ds_delay),
	.irq_config = ospi_irq_config_func,
	.clk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(CONTROLLER_NODE)),
	.clkid = (clock_control_subsys_t)DT_CLOCKS_CELL(CONTROLLER_NODE, clkid)
};

static struct alif_ospi_hyperram_data hyperram_data;

DEVICE_DT_DEFINE(DEVICE_NODE,
		 &memc_alif_ospi_hyperram_init,
		 NULL,
		 &hyperram_data,
		 &hyperram_config,
		 POST_KERNEL,
		 CONFIG_MEMC_INIT_PRIORITY,
		 NULL);
